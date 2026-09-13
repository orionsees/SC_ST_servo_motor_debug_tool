#ifndef CLI_TERM_H
#define CLI_TERM_H

#include <QByteArray>
#include <QChar>
#include <QString>
#include <QVector>

// Everything the full-screen tool needs from the terminal itself: raw mode,
// the alternate screen, key and mouse decoding, and a line builder that knows
// the difference between the characters it prints and the escape sequences it
// does not.
namespace term
{

struct Size
{
    int rows = 24;
    int cols = 80;
};

bool isTty();
Size size();

// Raw mode, alternate screen, hidden cursor, and optionally mouse reporting.
// leave() puts all of it back, and so do the atexit hook and the signal
// handlers installed by enter(), so the terminal is never left in raw mode
// however the program ends.
void enter(bool with_mouse);
void leave();
bool mouseOn();
void setMouse(bool on);

// Cleared by --plain and --ascii respectively.
extern bool colors;
extern bool unicode;

enum class Code
{
    None, Char, Enter, Escape, Backspace, Tab, BackTab,
    Up, Down, Left, Right, Home, End, PageUp, PageDown,
    Delete, Insert, Function, Mouse
};

enum class MouseKind
{
    None, WheelUp, WheelDown, Press, Drag, Release
};

struct Event
{
    Code code = Code::None;
    QChar ch;                       // Code::Char
    bool ctrl = false;              // a control-modified character
    int fn = 0;                     // Code::Function: 1..12
    MouseKind mouse = MouseKind::None;
    int x = 0;                      // Code::Mouse, 0-based
    int y = 0;
    bool shift = false;
};

// Turns the byte stream from the tty into events, keeping a half-arrived
// escape sequence between reads rather than mistaking its pieces for keys.
class KeyReader
{
public:
    void feed(const QByteArray &bytes);
    bool next(Event *out);

private:
    QByteArray buf_;
};

QString fg(const QString &hex);
QString bg(const QString &hex);

// One screen row under construction. Attributes are appended verbatim and
// never counted, so text() can truncate against the real visible width.
class Line
{
public:
    explicit Line(int width);

    Line &color(const QString &hex);
    Line &back(const QString &hex);
    Line &bold();
    Line &dim();
    Line &reverse();
    Line &off();

    Line &text(const QString &s);
    Line &fill(QChar c, int count);
    Line &pad(int column);
    Line &right(const QString &s);

    int used() const { return used_; }
    int room() const { return width_ - used_; }
    QString str() const;

private:
    int width_;
    int used_ = 0;
    QString out_;
};

// Double-buffered screen: only the rows that actually changed are written, so
// a 20 Hz refresh does not flicker and does not flood a slow terminal.
class Screen
{
public:
    void resize(int rows, int cols);
    void invalidate() { full_ = true; }
    void set(int row, const QString &line);
    void flush();

    int rows() const { return rows_; }
    int cols() const { return cols_; }

private:
    QVector<QString> next_;
    QVector<QString> shown_;
    int rows_ = 0;
    int cols_ = 0;
    bool full_ = true;
};

}

#endif
