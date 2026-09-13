#include "cli/term.h"

#include <QList>

#include <cerrno>
#include <csignal>
#include <cstring>
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace term
{

bool colors = true;
bool unicode = true;

namespace
{

termios saved_termios;
bool raw_active = false;
bool alt_active = false;
bool mouse_active = false;

// Restoring the terminal has to work from a signal handler, and a frame that
// is half written is a corrupt frame, so output goes through write() in a loop
// rather than through stdio.
void writeAll(const char *data, size_t len)
{
    size_t sent = 0;
    while(sent < len)
    {
        const ssize_t written = ::write(STDOUT_FILENO, data + sent, len - sent);
        if(written <= 0)
        {
            if(errno == EINTR)
                continue;
            return;
        }
        sent += static_cast<size_t>(written);
    }
}

void writeRaw(const char *s)
{
    writeAll(s, strlen(s));
}

void restore()
{
    if(mouse_active)
    {
        writeRaw("\x1b[?1006l\x1b[?1002l\x1b[?1000l");
        mouse_active = false;
    }
    if(alt_active)
    {
        writeRaw("\x1b[?25h\x1b[?1049l");
        alt_active = false;
    }
    if(raw_active)
    {
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &saved_termios);
        raw_active = false;
    }
}

void onFatalSignal(int sig)
{
    restore();
    // Re-raise with the default handler, so the exit status still says what
    // actually happened rather than claiming a clean exit.
    signal(sig, SIG_DFL);
    raise(sig);
}

// "#RRGGBB" to its three components. QColor lives in QtGui, which this binary
// deliberately does not link, so the theme colours are parsed here.
bool parseHex(const QString &hex, int *r, int *g, int *b)
{
    QString s = hex;
    if(s.startsWith('#'))
        s.remove(0, 1);
    if(s.size() != 6)
        return false;

    bool ok = false;
    *r = s.mid(0, 2).toInt(&ok, 16);
    if(!ok) return false;
    *g = s.mid(2, 2).toInt(&ok, 16);
    if(!ok) return false;
    *b = s.mid(4, 2).toInt(&ok, 16);
    return ok;
}

}

bool isTty()
{
    return isatty(STDIN_FILENO) == 1 && isatty(STDOUT_FILENO) == 1;
}

Size size()
{
    winsize ws;
    if(ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && ws.ws_col > 0 && ws.ws_row > 0)
    {
        return Size{ws.ws_row, ws.ws_col};
    }
    return Size{};
}

void enter(bool with_mouse)
{
    if(raw_active)
        return;

    tcgetattr(STDIN_FILENO, &saved_termios);
    atexit(restore);
    for(int sig : {SIGINT, SIGTERM, SIGHUP, SIGQUIT, SIGSEGV, SIGABRT})
        signal(sig, onFatalSignal);

    termios raw = saved_termios;
    // Canonical mode, echo and signal generation all off: every keystroke has
    // to reach the tool as a key, including Ctrl-C, which it treats as quit.
    raw.c_lflag &= ~(ECHO | ICANON | ISIG | IEXTEN);
    raw.c_iflag &= ~(IXON | ICRNL | BRKINT | INPCK | ISTRIP);
    raw.c_oflag &= ~(OPOST);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
    raw_active = true;

    writeRaw("\x1b[?1049h\x1b[?25l\x1b[2J");
    alt_active = true;

    if(with_mouse)
        setMouse(true);
}

void leave()
{
    restore();
}

bool mouseOn()
{
    return mouse_active;
}

void setMouse(bool on)
{
    if(on == mouse_active)
        return;

    // 1000 is button reporting, 1002 adds motion while a button is held (the
    // drag-to-pan gesture), 1006 asks for the SGR encoding, which is the only
    // one that survives past column 223.
    writeRaw(on ? "\x1b[?1000h\x1b[?1002h\x1b[?1006h"
                : "\x1b[?1006l\x1b[?1002l\x1b[?1000l");
    mouse_active = on;
}

void KeyReader::feed(const QByteArray &bytes)
{
    buf_.append(bytes);
}

bool KeyReader::next(Event *out)
{
    if(buf_.isEmpty())
        return false;

    const unsigned char c = static_cast<unsigned char>(buf_.at(0));

    if(c == 0x1b && buf_.size() >= 2)
    {
        // CSI sequence: ESC [ ... final-byte
        if(buf_.at(1) == '[')
        {
            int end = -1;
            for(int i = 2; i < buf_.size(); i++)
            {
                const char ch = buf_.at(i);
                if((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') || ch == '~')
                {
                    end = i;
                    break;
                }
            }
            if(end < 0)
            {
                // Still arriving. Wait for the rest rather than reporting the
                // pieces as separate keys.
                return false;
            }

            const QByteArray seq = buf_.mid(2, end - 2 + 1);
            buf_.remove(0, end + 1);

            Event e;
            const char final_byte = seq.at(seq.size() - 1);
            const QByteArray body = seq.left(seq.size() - 1);

            if(body.startsWith('<') && (final_byte == 'M' || final_byte == 'm'))
            {
                const QList<QByteArray> parts = body.mid(1).split(';');
                if(parts.size() == 3)
                {
                    const int b = parts[0].toInt();
                    e.code = Code::Mouse;
                    e.x = parts[1].toInt() - 1;
                    e.y = parts[2].toInt() - 1;
                    e.shift = (b & 4) != 0;
                    if(b & 64)
                        e.mouse = (b & 1) ? MouseKind::WheelDown : MouseKind::WheelUp;
                    else if(final_byte == 'm')
                        e.mouse = MouseKind::Release;
                    else if(b & 32)
                        e.mouse = MouseKind::Drag;
                    else
                        e.mouse = MouseKind::Press;
                    *out = e;
                    return true;
                }
                return next(out);
            }

            // Modifiers arrive as a second parameter (ESC [ 1 ; 2 A); the only
            // one the tool acts on is shift.
            const QList<QByteArray> params = body.split(';');
            if(params.size() >= 2)
                e.shift = params[1].toInt() == 2 || params[1].toInt() == 6;

            switch(final_byte)
            {
                case 'A': e.code = Code::Up; break;
                case 'B': e.code = Code::Down; break;
                case 'C': e.code = Code::Right; break;
                case 'D': e.code = Code::Left; break;
                case 'H': e.code = Code::Home; break;
                case 'F': e.code = Code::End; break;
                case 'Z': e.code = Code::BackTab; break;
                case '~':
                {
                    const int n = params.isEmpty() ? 0 : params[0].toInt();
                    switch(n)
                    {
                        case 1: case 7: e.code = Code::Home; break;
                        case 2: e.code = Code::Insert; break;
                        case 3: e.code = Code::Delete; break;
                        case 4: case 8: e.code = Code::End; break;
                        case 5: e.code = Code::PageUp; break;
                        case 6: e.code = Code::PageDown; break;
                        case 11: case 12: case 13: case 14:
                            e.code = Code::Function; e.fn = n - 10; break;
                        case 15: e.code = Code::Function; e.fn = 5; break;
                        case 17: case 18: case 19: case 20: case 21:
                            e.code = Code::Function; e.fn = n - 11; break;
                        case 23: case 24:
                            e.code = Code::Function; e.fn = n - 12; break;
                        default: e.code = Code::None; break;
                    }
                    break;
                }
                default: e.code = Code::None; break;
            }

            if(e.code == Code::None)
                return next(out);
            *out = e;
            return true;
        }

        // SS3 sequence: ESC O P..S are F1..F4 on many terminals.
        if(buf_.at(1) == 'O' && buf_.size() >= 3)
        {
            const char k = buf_.at(2);
            buf_.remove(0, 3);
            Event e;
            switch(k)
            {
                case 'P': e.code = Code::Function; e.fn = 1; break;
                case 'Q': e.code = Code::Function; e.fn = 2; break;
                case 'R': e.code = Code::Function; e.fn = 3; break;
                case 'S': e.code = Code::Function; e.fn = 4; break;
                case 'H': e.code = Code::Home; break;
                case 'F': e.code = Code::End; break;
                default: return next(out);
            }
            *out = e;
            return true;
        }
    }

    buf_.remove(0, 1);

    Event e;
    if(c == 0x1b)
    {
        e.code = Code::Escape;
    }
    else if(c == '\r' || c == '\n')
    {
        e.code = Code::Enter;
    }
    else if(c == '\t')
    {
        e.code = Code::Tab;
    }
    else if(c == 127 || c == 8)
    {
        e.code = Code::Backspace;
    }
    else if(c < 32)
    {
        e.code = Code::Char;
        e.ctrl = true;
        e.ch = QChar('a' + c - 1);
    }
    else if(c < 128)
    {
        e.code = Code::Char;
        e.ch = QChar(c);
    }
    else
    {
        // A UTF-8 lead byte. Nothing the tool binds is non-ASCII, so the whole
        // sequence is dropped rather than half-decoded.
        while(!buf_.isEmpty() && (static_cast<unsigned char>(buf_.at(0)) & 0xC0) == 0x80)
            buf_.remove(0, 1);
        return next(out);
    }

    *out = e;
    return true;
}

QString fg(const QString &hex)
{
    int r = 0, g = 0, b = 0;
    if(!colors || !parseHex(hex, &r, &g, &b))
        return QString();
    return QString("\x1b[38;2;%1;%2;%3m").arg(r).arg(g).arg(b);
}

QString bg(const QString &hex)
{
    int r = 0, g = 0, b = 0;
    if(!colors || !parseHex(hex, &r, &g, &b))
        return QString();
    return QString("\x1b[48;2;%1;%2;%3m").arg(r).arg(g).arg(b);
}

Line::Line(int width)
    : width_(width < 0 ? 0 : width)
{
}

Line &Line::color(const QString &hex)
{
    out_ += fg(hex);
    return *this;
}

Line &Line::back(const QString &hex)
{
    out_ += bg(hex);
    return *this;
}

Line &Line::bold()
{
    if(colors)
        out_ += "\x1b[1m";
    return *this;
}

Line &Line::dim()
{
    if(colors)
        out_ += "\x1b[2m";
    return *this;
}

Line &Line::reverse()
{
    if(colors)
        out_ += "\x1b[7m";
    return *this;
}

Line &Line::off()
{
    if(colors)
        out_ += "\x1b[0m";
    return *this;
}

Line &Line::text(const QString &s)
{
    if(room() <= 0)
        return *this;

    QString piece = s;
    piece.remove(QChar('\n'));
    piece.remove(QChar('\r'));
    piece.remove(QChar('\t'));
    if(piece.size() > room())
        piece = piece.left(room());

    out_ += piece;
    used_ += piece.size();
    return *this;
}

Line &Line::fill(QChar c, int count)
{
    const int n = qMin(count, room());
    if(n > 0)
    {
        out_ += QString(n, c);
        used_ += n;
    }
    return *this;
}

Line &Line::pad(int column)
{
    if(column > used_)
        fill(QChar(' '), column - used_);
    return *this;
}

Line &Line::right(const QString &s)
{
    const int len = qMin(s.size(), room());
    pad(width_ - len);
    return text(s.right(len));
}

QString Line::str() const
{
    return colors ? out_ + "\x1b[0m" : out_;
}

void Screen::resize(int rows, int cols)
{
    if(rows == rows_ && cols == cols_)
        return;

    rows_ = rows;
    cols_ = cols;
    next_ = QVector<QString>(rows);
    shown_ = QVector<QString>(rows);
    full_ = true;
}

void Screen::set(int row, const QString &line)
{
    if(row >= 0 && row < next_.size())
        next_[row] = line;
}

void Screen::flush()
{
    QString out;
    if(full_)
        out += "\x1b[2J";

    for(int i = 0; i < next_.size(); i++)
    {
        if(!full_ && next_[i] == shown_[i])
            continue;
        out += QString("\x1b[%1;1H\x1b[K").arg(i + 1);
        out += next_[i];
        shown_[i] = next_[i];
    }
    full_ = false;

    if(out.isEmpty())
        return;

    const QByteArray bytes = out.toUtf8();
    writeAll(bytes.constData(), static_cast<size_t>(bytes.size()));
}

}
