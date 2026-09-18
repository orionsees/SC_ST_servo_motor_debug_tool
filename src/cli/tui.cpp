#include "cli/tui.h"

#include <QCoreApplication>
#include <QFile>
#include <QRegExp>
#include <QSocketNotifier>
#include <QTextStream>
#include <QTimer>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <unistd.h>

#include "confirm_steps.h"
#include "register_snapshot.h"

namespace cli
{

using feetech_servo::decodeServoStatus;
using feetech_servo::findMemConfig;
using feetech_servo::getMemConfig;
using feetech_servo::seriesName;
using feetech_servo::supportsMidpointCalibration;
using feetech_servo::TORQUE_ENABLE_ADDRESS;

namespace
{

// The window's palette, as far as a terminal can carry it.
const QString COL_TEXT = "#E8EDF2";
const QString COL_DIM = "#9AA4B0";
const QString COL_FAINT = "#5C6570";
const QString COL_ACCENT = "#4C9AFF";
const QString COL_BORDER = "#3A4550";
const QString COL_RELEASED = "#3FB950";
const QString COL_ENGAGED = "#E0A030";
const QString COL_ALARM = "#E05252";
const QString COL_UNKNOWN = "#6B7280";
const QString COL_PANEL = "#1B1F24";

const QString SERIES_COLORS[7] = {
    "#E8EDF2",      // position
    "#4C9AFF",      // goal
    "#FFA94D",      // torque
    "#51CF66",      // speed
    "#22D3EE",      // current
    "#FFD43B",      // temperature
    "#C77DFF",      // voltage
};

const char *SERIES_NAMES[7] = {
    "Position", "Goal", "Torque", "Speed", "Current", "Temp", "Voltage"
};

const int PANEL_WIDTH = 26;

// A port can be given as a full device path, which is mostly directory. The
// last component is the part that identifies it.
QString shortPort(const QString &port)
{
    const int slash = port.lastIndexOf('/');
    return slash >= 0 ? port.mid(slash + 1) : port;
}

QString dot()
{
    return term::unicode ? QString(QChar(0x25CF)) : QString("*");
}

QString degreeSign()
{
    return term::unicode ? QString(QChar(0x00B0)) : QString(" deg");
}

// A horizontal bar for the goal slider, filled to the fraction given.
QString bar(double fraction, int width)
{
    const int filled = qBound(0, static_cast<int>(std::lround(fraction * width)), width);
    if(term::unicode)
        return QString(filled, QChar(0x2501)) + QString(width - filled, QChar(0x2500));
    return QString(filled, '=') + QString(width - filled, '-');
}

}

Tui::Tui(const ConnectionOptions &options, QObject *parent)
    : QObject(parent)
    , session_(true, options.isRemote())
    , options_(options)
{
    clock_.start();
    control_touch_.start();
    busy_since_.start();
}

Tui::~Tui()
{
    shutdown();
}

int Tui::run()
{
    if(!term::isTty())
    {
        fprintf(stderr, "servobench: the full-screen tool needs a terminal. "
                        "Try a command instead -- servobench-cli --help.\n");
        return 2;
    }

    term::enter(true);

    // Keys arrive through the event loop, so a slow transaction never eats
    // them: it only delays them.
    stdin_notifier_ = new QSocketNotifier(STDIN_FILENO, QSocketNotifier::Read, this);
    connect(stdin_notifier_, &QSocketNotifier::activated, this, &Tui::onStdinReady);

    // While the bus is mid-transaction the tool stops reading keys, so a
    // multi-step EPROM write cannot be interrupted half way through, and keeps
    // repainting, so the screen does not look frozen.
    session_.setBusyHook([this](bool busy) {
        if(stdin_notifier_ != nullptr)
            stdin_notifier_->setEnabled(!busy);
        if(busy)
        {
            busy_since_.restart();
            render();
        }
    });

    setupTimers();

    QString error;
    if(!session_.open(options_, &error))
    {
        setMessage(error + "  Press c to change the port, o to try again.", true);
    }
    else
    {
        options_ = session_.options();
        setMessage(QString("Opened %1. Press s to search the bus.").arg(options_.port));
    }

    render();
    const int code = QCoreApplication::exec();
    shutdown();
    return code;
}

void Tui::shutdown()
{
    if(quitting_)
        return;
    quitting_ = true;

    if(recorder_.active())
        recorder_.stop();
    if(stdin_notifier_ != nullptr)
        stdin_notifier_->setEnabled(false);

    term::leave();
}

void Tui::setupTimers()
{
    render_timer_ = new QTimer(this);
    connect(render_timer_, &QTimer::timeout, this, &Tui::render);
    render_timer_->start(60);

    telemetry_timer_ = new QTimer(this);
    connect(telemetry_timer_, &QTimer::timeout, this, &Tui::onTelemetryTick);
    telemetry_timer_->start(40);

    search_timer_ = new QTimer(this);
    connect(search_timer_, &QTimer::timeout, this, &Tui::onSearchTick);

    torque_timer_ = new QTimer(this);
    connect(torque_timer_, &QTimer::timeout, this, &Tui::onTorqueTick);
    torque_timer_->start(1000);

    programming_timer_ = new QTimer(this);
    connect(programming_timer_, &QTimer::timeout, this, &Tui::onProgrammingTick);
    programming_timer_->start(200);

    calibration_timer_ = new QTimer(this);
    connect(calibration_timer_, &QTimer::timeout, this, &Tui::onCalibrationTick);
    calibration_timer_->start(200);

    auto_debug_timer_ = new QTimer(this);
    connect(auto_debug_timer_, &QTimer::timeout, this, &Tui::onAutoDebugTick);

    record_timer_ = new QTimer(this);
    connect(record_timer_, &QTimer::timeout, this, &Tui::onRecordTick);
    record_timer_->start(50);
}

// --- state helpers ---

bool Tui::servoSelected() const
{
    return selected_index_ >= 0 && selected_index_ < session_.servos().size();
}

int Tui::selectedId() const
{
    return servoSelected() ? session_.servos()[selected_index_].id : -1;
}

bool Tui::ready() const
{
    return session_.isOpen() && !searching_ && servoSelected();
}

bool Tui::pollsSuspended() const
{
    return session_.busy() || overlay_ == Overlay::Confirm;
}

ModelSeries Tui::selectedSeries() const
{
    return servoSelected() ? session_.servos()[selected_index_].series : ModelSeries::STS;
}

int Tui::countsPerRev() const
{
    return feetech_servo::countsPerRev(selectedSeries());
}

int Tui::maxCount() const
{
    return countsPerRev() - 1;
}

const std::vector<MemoryConfig> &Tui::registers() const
{
    return getMemConfig(selectedSeries());
}

QString Tui::servoColor(int id) const
{
    // A servo reporting a fault outranks whatever its torque state is: that is
    // the thing worth noticing about the row.
    const int fault = fault_state_.value(id, -1);
    if(fault > 0)
        return COL_ALARM;

    const int torque = torque_state_.value(id, -1);
    if(torque < 0)
        return COL_UNKNOWN;
    return torque != 0 ? COL_ENGAGED : COL_RELEASED;
}

QString Tui::torqueSummary() const
{
    int on = 0;
    int off = 0;
    int unknown = 0;
    for(const ServoEntry &entry : session_.servos())
    {
        const int torque = torque_state_.value(entry.id, -1);
        if(torque < 0)
            unknown++;
        else if(torque != 0)
            on++;
        else
            off++;
    }

    if(on + off == 0)
        return "unknown";
    if(unknown > 0 || (on > 0 && off > 0))
        return QString("mixed %1on/%2off").arg(on).arg(off);
    return on > 0 ? "all on" : "all off";
}

void Tui::setMessage(const QString &text, bool warning)
{
    message_ = text;
    message_warning_ = warning;
}

void Tui::appendSample(const ServoStatus &status)
{
    latest_ = status;
    pos_buf_.push(status.pos);
    goal_buf_.push(status.goal);
    torque_buf_.push(status.torque);
    speed_buf_.push(status.speed);
    current_buf_.push(status.current);
    temp_buf_.push(status.temp);
    voltage_buf_.push(status.voltage);
    time_buf_.push(clock_.elapsed());
}

// --- input ---

void Tui::onStdinReady()
{
    char buffer[512];
    const ssize_t count = ::read(STDIN_FILENO, buffer, sizeof(buffer));

    if(count == 0 || (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR))
    {
        // The terminal has gone -- the window was closed, or the session
        // ended. Without this the notifier would keep firing on a dead fd and
        // spin the tool at 100% CPU forever.
        shutdown();
        QCoreApplication::quit();
        return;
    }

    if(count < 0)
        return;

    keys_.feed(QByteArray(buffer, static_cast<int>(count)));

    term::Event event;
    while(keys_.next(&event))
        handleKey(event);

    render();
}

void Tui::handleKey(const term::Event &event)
{
    if(event.code == term::Code::Mouse)
    {
        handleMouse(event);
        return;
    }

    if(overlay_ != Overlay::None)
    {
        handleOverlayKey(event);
        return;
    }

    // Quit is deliberately reachable from anywhere, including mid-search.
    if((event.code == term::Code::Char && event.ctrl && event.ch == QChar('c'))
       || (event.code == term::Code::Char && !event.ctrl && event.ch == QChar('q')))
    {
        shutdown();
        QCoreApplication::quit();
        return;
    }

    handleGlobalKey(event);
}

void Tui::handleGlobalKey(const term::Event &event)
{
    const QChar ch = event.ch;

    if(event.code == term::Code::Tab)
    {
        tab_ = tab_ == Tab::Debug ? Tab::Programming
             : tab_ == Tab::Programming ? Tab::Calibration : Tab::Debug;
        if(tab_ == Tab::Calibration)
            updateCalibrationGate(true);
        return;
    }
    if(event.code == term::Code::BackTab)
    {
        tab_ = tab_ == Tab::Debug ? Tab::Calibration
             : tab_ == Tab::Calibration ? Tab::Programming : Tab::Debug;
        if(tab_ == Tab::Calibration)
            updateCalibrationGate(true);
        return;
    }
    if(event.code == term::Code::Function && event.fn >= 1 && event.fn <= 3)
    {
        tab_ = event.fn == 1 ? Tab::Debug : event.fn == 2 ? Tab::Programming : Tab::Calibration;
        if(tab_ == Tab::Calibration)
            updateCalibrationGate(true);
        return;
    }

    if(event.code == term::Code::Char && !event.ctrl)
    {
        if(ch == QChar('?'))
        {
            overlay_ = Overlay::Help;
            return;
        }
        if(ch == QChar('o'))
        {
            toggleConnection();
            return;
        }
        if(ch == QChar('c'))
        {
            menu("Connection", {
                {QChar('p'), "Port", [this] {
                    QStringList names;
                    for(const PortInfo &port : availablePorts())
                        names << port.name;
                    ask(QString("Port (available: %1)").arg(names.join(", ")), options_.port,
                        [this](const QString &text) {
                            if(text.trimmed().isEmpty())
                                return;
                            options_.port = text.trimmed();
                            setMessage(QString("Port set to %1. Press o to open it.").arg(options_.port));
                        });
                }},
                {QChar('b'), "Baud rate", [this] {
                    ask("Baud rate", QString::number(options_.baud), [this](const QString &text) {
                        bool ok = false;
                        const int baud = text.toInt(&ok);
                        if(!ok || baud <= 0)
                        {
                            setMessage("Baud rate must be a number.", true);
                            return;
                        }
                        options_.baud = baud;
                        setMessage("Baud rate set. Reopen the port for it to take effect.");
                    });
                }},
                {QChar('y'), "Parity", [this] {
                    options_.parity = options_.parity == QSerialPort::NoParity ? QSerialPort::OddParity
                                    : options_.parity == QSerialPort::OddParity ? QSerialPort::EvenParity
                                    : QSerialPort::NoParity;
                    setMessage("Parity changed. Reopen the port for it to take effect.");
                }},
                {QChar('t'), "Timeout", [this] {
                    ask("Timeout (ms)", QString::number(options_.timeout), [this](const QString &text) {
                        bool ok = false;
                        const int timeout = text.toInt(&ok);
                        if(!ok || timeout < 0)
                        {
                            setMessage("Timeout must be a number of milliseconds.", true);
                            return;
                        }
                        options_.timeout = timeout;
                        setMessage("Timeout set. Reopen the port for it to take effect.");
                    });
                }},
            });
            return;
        }
        if(ch == QChar('s'))
        {
            if(searching_)
                stopSearch();
            else
                startSearch();
            return;
        }
        if(ch == QChar('n'))
        {
            stepSelection(1);
            return;
        }
        if(ch == QChar('N'))
        {
            stepSelection(-1);
            return;
        }
        if(ch == QChar('t'))
        {
            toggleTorqueAll();
            return;
        }
        if(ch == QChar('z'))
        {
            term::setMouse(!term::mouseOn());
            setMessage(term::mouseOn()
                           ? "Mouse reporting on: wheel zooms, drag pans."
                           : "Mouse reporting off, so the terminal can select text again.");
            return;
        }
    }

    switch(tab_)
    {
        case Tab::Debug: handleDebugKey(event); break;
        case Tab::Programming: handleProgrammingKey(event); break;
        case Tab::Calibration: handleCalibrationKey(event); break;
    }
}

void Tui::handleDebugKey(const term::Event &event)
{
    const QChar ch = event.ch;

    if(event.code == term::Code::Left || event.code == term::Code::Right
       || event.code == term::Code::Up || event.code == term::Code::Down
       || event.code == term::Code::Home || event.code == term::Code::End)
    {
        int goal = goal_;
        if(event.code == term::Code::Left)
            goal -= 1;
        else if(event.code == term::Code::Right)
            goal += 1;
        else if(event.code == term::Code::Down)
            goal -= 50;
        else if(event.code == term::Code::Up)
            goal += 50;
        else if(event.code == term::Code::Home)
            goal = 0;
        else
            goal = maxCount();

        sendGoal(qBound(0, goal, maxCount()));
        return;
    }

    if(event.code != term::Code::Char || event.ctrl)
        return;

    if(ch.isDigit() && ch != QChar('0'))
    {
        const int index = ch.digitValue() - 1;
        if(index >= 0 && index < 7)
        {
            series_visible_[index] = !series_visible_[index];
            setMessage(QString("%1 %2").arg(SERIES_NAMES[index],
                                            series_visible_[index] ? "shown" : "hidden"));
        }
        return;
    }

    const double width = plot_cols_ * 2.0;
    const double height = plot_rows_ * 4.0;

    if(ch == QChar('+') || ch == QChar('='))
    {
        view_.zoomTime(width - 1, width, 1.15);
        return;
    }
    if(ch == QChar('-') || ch == QChar('_'))
    {
        view_.zoomTime(width - 1, width, 1.0 / 1.15);
        return;
    }
    if(ch == QChar('>'))
    {
        view_.zoomValues((height - 1) / 2.0, height, 1.15);
        return;
    }
    if(ch == QChar('<'))
    {
        view_.zoomValues((height - 1) / 2.0, height, 1.0 / 1.15);
        return;
    }
    if(ch == QChar('h'))
    {
        view_.panBy(width * 0.1, 0, width, height);
        return;
    }
    if(ch == QChar('l'))
    {
        view_.panBy(-width * 0.1, 0, width, height);
        return;
    }
    if(ch == QChar('0'))
    {
        view_.reset();
        setMessage("Plot view reset.");
        return;
    }
    if(ch == QChar('u'))
    {
        ask("Upper limit line (counts, 0 for none)", QString::number(up_limit_),
            [this](const QString &text) { up_limit_ = qBound(0, text.toInt(), 1200); });
        return;
    }
    if(ch == QChar('d'))
    {
        ask("Lower limit line (counts, 0 for none)", QString::number(down_limit_),
            [this](const QString &text) { down_limit_ = qBound(0, text.toInt(), 1200); });
        return;
    }
    if(ch == QChar('g'))
    {
        ask("Goal position (counts)", QString::number(goal_), [this](const QString &text) {
            bool ok = false;
            const int goal = text.toInt(&ok);
            if(!ok)
            {
                setMessage("Position must be a number of counts.", true);
                return;
            }
            if(goal < 0 || goal > maxCount())
            {
                setMessage(QString("%1 is outside the encoder range 0..%2.").arg(goal).arg(maxCount()), true);
                return;
            }
            sendGoal(goal);
        });
        return;
    }
    if(ch == QChar('a'))
    {
        const double current = angle_in_radians_ ? countsToRadians(selectedSeries(), goal_)
                                                 : countsToDegrees(selectedSeries(), goal_);
        ask(QString("Joint angle (%1 from the midpoint)").arg(angle_in_radians_ ? "rad" : "deg"),
            QString::number(current, 'f', angle_in_radians_ ? 3 : 1),
            [this](const QString &text) {
                bool ok = false;
                const double angle = text.toDouble(&ok);
                if(!ok)
                {
                    setMessage("Angle must be a number.", true);
                    return;
                }
                setGoalFromAngle(angle);
            });
        return;
    }
    if(ch == QChar('U'))
    {
        angle_in_radians_ = !angle_in_radians_;
        setMessage(QString("Angles in %1.").arg(angle_in_radians_ ? "radians" : "degrees"));
        return;
    }
    if(ch == QChar('m'))
    {
        menu("Write mode", {
            {QChar('w'), "Write - command the move now", [this] {
                mode_ = WriteMode::Write;
                setMessage("Write mode: each goal is commanded immediately.");
            }},
            {QChar('s'), "Sync Write - one packet to every servo", [this] {
                mode_ = WriteMode::SyncWrite;
                setMessage("Sync write: goals go to every detected servo at once, unacknowledged.");
            }},
            {QChar('r'), "Reg Write - queue, then Action", [this] {
                mode_ = WriteMode::RegWrite;
                setMessage("Reg write: goals are queued. Press x to send Action.");
            }},
        });
        return;
    }
    if(ch == QChar('T'))
    {
        toggleTorqueSelected();
        return;
    }
    if(ch == QChar('x'))
    {
        if(!ready())
            return;
        // Action runs what a reg write queued. In the other modes there is
        // nothing queued to run, and the window disables the button outright.
        if(mode_ != WriteMode::RegWrite)
        {
            setMessage("Action runs a queued reg write. Press m to switch to reg write first.",
                       true);
            return;
        }
        session_.regWriteAction(selectedId());
        setMessage(QString("Action sent to ID %1.").arg(selectedId()));
        return;
    }
    if(ch == QChar('v'))
    {
        ask("Speed, acceleration and goal time (space separated)",
            QString("%1 %2 %3").arg(speed_).arg(acc_).arg(time_),
            [this](const QString &text) {
                const QStringList parts = splitFields(text, QRegExp("\\s+"));
                if(parts.size() < 3)
                {
                    setMessage("Give three numbers: speed, acceleration, goal time.", true);
                    return;
                }
                speed_ = parts[0].toInt();
                acc_ = parts[1].toInt();
                time_ = parts[2].toInt();
                setMessage(QString("Speed %1, acceleration %2, goal time %3.")
                               .arg(speed_).arg(acc_).arg(time_));
            });
        return;
    }
    if(ch == QChar('w'))
    {
        toggleSweep();
        return;
    }
    if(ch == QChar('e'))
    {
        toggleStep();
        return;
    }
    if(ch == QChar('E'))
    {
        ask("Auto debug: start end hold(ms) step step-delay(ms)",
            QString("%1 %2 %3 %4 %5").arg(auto_start_).arg(auto_end_).arg(auto_hold_ms_)
                .arg(auto_step_).arg(auto_delay_ms_),
            [this](const QString &text) {
                const QStringList parts = splitFields(text, QRegExp("\\s+"));
                if(parts.size() < 5)
                {
                    setMessage("Give five numbers: start, end, hold, step size, step delay.", true);
                    return;
                }
                auto_start_ = qBound(0, parts[0].toInt(), maxCount());
                auto_end_ = qBound(0, parts[1].toInt(), maxCount());
                auto_hold_ms_ = qMax(1, parts[2].toInt());
                auto_step_ = qMax(1, parts[3].toInt());
                auto_delay_ms_ = qMax(1, parts[4].toInt());
                setMessage("Auto debug settings updated.");
            });
        return;
    }
    if(ch == QChar('r'))
    {
        toggleRecording();
        return;
    }
    if(ch == QChar('R'))
    {
        ask("Record file and flush interval (seconds)",
            QString("%1 %2").arg(record_path_).arg(record_interval_),
            [this](const QString &text) {
                const QStringList parts = splitFields(text, QRegExp("\\s+"));
                if(parts.isEmpty())
                    return;
                record_path_ = parts[0];
                if(parts.size() > 1)
                    record_interval_ = qMax(1, parts[1].toInt());
                setMessage(QString("Recording to %1 every %2 s.").arg(record_path_).arg(record_interval_));
            });
        return;
    }
    if(ch == QChar('X'))
    {
        if(recorder_.active())
        {
            setMessage("Stop recording first, with r.", true);
            return;
        }
        recorder_.reset();
        setMessage("Row counter cleared.");
        return;
    }
}

void Tui::handleProgrammingKey(const term::Event &event)
{
    const int count = static_cast<int>(registers().size());
    if(count == 0)
        return;

    if(event.code == term::Code::Up)
    {
        register_row_ = qMax(0, register_row_ - 1);
        return;
    }
    if(event.code == term::Code::Down)
    {
        register_row_ = qMin(count - 1, register_row_ + 1);
        return;
    }
    if(event.code == term::Code::PageUp)
    {
        register_row_ = qMax(0, register_row_ - 10);
        return;
    }
    if(event.code == term::Code::PageDown)
    {
        register_row_ = qMin(count - 1, register_row_ + 10);
        return;
    }
    if(event.code == term::Code::Home)
    {
        register_row_ = 0;
        return;
    }
    if(event.code == term::Code::End)
    {
        register_row_ = count - 1;
        return;
    }

    if(event.code != term::Code::Char && event.code != term::Code::Enter)
        return;
    if(event.code == term::Code::Char && event.ctrl)
        return;

    const QChar ch = event.ch;
    const MemoryConfig &config = registers()[register_row_];

    if(ch == QChar('w') || event.code == term::Code::Enter)
    {
        if(!ready())
        {
            setMessage("No servo selected.", true);
            return;
        }
        if(config.is_readonly)
        {
            setMessage(QString("[%1] %2 is read-only.").arg(config.address).arg(config.name), true);
            return;
        }

        std::optional<int> shown;
        if(register_row_ < register_values_.size())
            shown = register_values_[register_row_];
        const QString range = (config.min_val == -1 && config.max_val == -1)
                            ? QString("no documented range")
                            : QString("range %1..%2").arg(config.min_val).arg(config.max_val);
        ask(QString("[%1] %2  (%3, %4)")
                .arg(config.address).arg(config.name)
                .arg(config.is_eprom ? "EPROM" : "SRAM")
                .arg(range),
            QString::number(shown.value_or(0)),
            [this](const QString &text) { writeSelectedRegister(text); });
        return;
    }
    if(ch == QChar('S'))
    {
        if(!ready())
        {
            setMessage("No servo selected.", true);
            return;
        }
        ask("Save registers to file", QString("~/servo%1_registers.json").arg(selectedId()),
            [this](const QString &path) { saveRegisterSnapshot(path); });
        return;
    }
    if(ch == QChar('L'))
    {
        if(!ready())
        {
            setMessage("No servo selected.", true);
            return;
        }
        ask("Load registers from file", QString("~/servo%1_registers.json").arg(selectedId()),
            [this](const QString &path) { loadRegisterSnapshot(path); });
        return;
    }
    if(ch == QChar('M'))
    {
        setMidpoint();
        return;
    }
    if(ch == QChar('/'))
    {
        ask("Jump to register", QString(), [this](const QString &text) {
            QString error;
            const MemoryConfig *config = findRegister(selectedSeries(), text, &error);
            if(config == nullptr)
            {
                setMessage(error, true);
                return;
            }
            for(std::size_t i = 0; i < registers().size(); i++)
            {
                if(registers()[i].address == config->address)
                {
                    register_row_ = static_cast<int>(i);
                    break;
                }
            }
        });
        return;
    }
}

void Tui::handleCalibrationKey(const term::Event &event)
{
    if(event.code == term::Code::Up)
    {
        calib_row_ = qMax(0, calib_row_ - 1);
        return;
    }
    if(event.code == term::Code::Down)
    {
        calib_row_ = qMin(calib_rows_.size() - 1, calib_row_ + 1);
        return;
    }

    if(event.code != term::Code::Char || event.ctrl)
        return;

    const QChar ch = event.ch;

    if(ch == QChar('k'))
    {
        updateCalibrationGate(true);
        return;
    }
    if(!calib_gate_open_ && ch != QChar('k'))
    {
        setMessage("Calibration is gated on torque being released everywhere. Press t, then k.", true);
        return;
    }
    if(ch == QChar('j'))
    {
        QStringList current;
        for(const CalibRow &row : calib_rows_)
            current << row.name;
        ask("Joint names, comma separated, in ascending servo ID order", current.join(","),
            [this](const QString &text) { applyJointNames(text); });
        return;
    }
    if(ch == QChar('J'))
    {
        if(calib_row_ < 0 || calib_row_ >= calib_rows_.size())
            return;
        ask(QString("Joint name for ID %1").arg(calib_rows_[calib_row_].id),
            calib_rows_[calib_row_].name,
            [this](const QString &text) {
                if(calib_row_ >= 0 && calib_row_ < calib_rows_.size())
                    calib_rows_[calib_row_].name = text.trimmed();
            });
        return;
    }
    if(ch == QChar('D'))
    {
        if(calib_row_ >= 0 && calib_row_ < calib_rows_.size())
        {
            CalibRow &row = calib_rows_[calib_row_];
            row.drive_mode = row.drive_mode == 0 ? 1 : 0;
            setMessage(QString("ID %1 drive_mode = %2").arg(row.id).arg(row.drive_mode));
        }
        return;
    }
    if(ch == QChar('H'))
    {
        calibrationSetHome();
        return;
    }
    if(ch == QChar('R'))
    {
        toggleRangeRecording();
        return;
    }
    if(ch == QChar('X'))
    {
        ask("Export calibration to", "~/calibration.json",
            [this](const QString &path) { exportCalibration(path); });
        return;
    }
}

void Tui::handleMouse(const term::Event &event)
{
    if(tab_ != Tab::Debug || overlay_ != Overlay::None)
        return;

    const bool inside = event.x >= plot_left_ && event.x < plot_left_ + plot_cols_
                     && event.y >= plot_top_ && event.y < plot_top_ + plot_rows_;

    const double width = plot_cols_ * 2.0;
    const double height = plot_rows_ * 4.0;

    switch(event.mouse)
    {
        case term::MouseKind::WheelUp:
        case term::MouseKind::WheelDown:
        {
            if(!inside)
                return;
            const double factor = event.mouse == term::MouseKind::WheelUp ? 1.15 : 1.0 / 1.15;
            // Time by default, values with shift held: the two axes carry
            // different units, so zooming them together is rarely wanted.
            if(event.shift)
                view_.zoomValues((event.y - plot_top_) * 4.0, height, factor);
            else
                view_.zoomTime((event.x - plot_left_) * 2.0, width, factor);
            break;
        }
        case term::MouseKind::Press:
            if(inside)
            {
                dragging_ = true;
                drag_x_ = event.x;
                drag_y_ = event.y;
            }
            break;
        case term::MouseKind::Drag:
            if(dragging_)
            {
                view_.panBy((event.x - drag_x_) * 2.0, (event.y - drag_y_) * 4.0, width, height);
                drag_x_ = event.x;
                drag_y_ = event.y;
            }
            break;
        case term::MouseKind::Release:
            dragging_ = false;
            break;
        default:
            break;
    }

    cursor_cell_ = inside ? event.x - plot_left_ : -1;
}

void Tui::handleOverlayKey(const term::Event &event)
{
    if(event.code == term::Code::Escape
       || (event.code == term::Code::Char && event.ctrl && event.ch == QChar('c')))
    {
        closeOverlay();
        setMessage("Cancelled.");
        return;
    }

    if(overlay_ == Overlay::Help)
    {
        closeOverlay();
        return;
    }

    if(overlay_ == Overlay::Menu)
    {
        if(event.code != term::Code::Char)
            return;
        for(const MenuItem &item : menu_items_)
        {
            if(item.key == event.ch)
            {
                const std::function<void()> action = item.action;
                closeOverlay();
                if(action)
                    action();
                return;
            }
        }
        return;
    }

    if(overlay_ == Overlay::Confirm)
    {
        const bool accepted = event.code == term::Code::Enter
                           || (event.code == term::Code::Char && event.ch == QChar('y'));
        if(!accepted)
        {
            closeOverlay();
            setMessage("Cancelled.");
            return;
        }

        confirm_index_++;
        if(confirm_index_ < confirm_steps_.size())
            return;

        const std::function<void()> action = confirm_accepted_;
        closeOverlay();
        if(action)
            action();
        return;
    }

    // Prompt: a one-line editor, deliberately small -- arrows, backspace,
    // delete, and Ctrl-U to start again.
    if(event.code == term::Code::Enter)
    {
        const QString text = prompt_buffer_;
        const std::function<void(const QString &)> done = prompt_done_;
        closeOverlay();
        if(done)
            done(text);
        return;
    }
    // Any editing key keeps what is there; a printable one replaces it.
    if(event.code != term::Code::Char)
        prompt_fresh_ = false;

    if(event.code == term::Code::Backspace)
    {
        if(prompt_cursor_ > 0)
        {
            prompt_buffer_.remove(prompt_cursor_ - 1, 1);
            prompt_cursor_--;
        }
        return;
    }
    if(event.code == term::Code::Delete)
    {
        if(prompt_cursor_ < prompt_buffer_.size())
            prompt_buffer_.remove(prompt_cursor_, 1);
        return;
    }
    if(event.code == term::Code::Left)
    {
        prompt_cursor_ = qMax(0, prompt_cursor_ - 1);
        return;
    }
    if(event.code == term::Code::Right)
    {
        prompt_cursor_ = qMin(prompt_buffer_.size(), prompt_cursor_ + 1);
        return;
    }
    if(event.code == term::Code::Home)
    {
        prompt_cursor_ = 0;
        return;
    }
    if(event.code == term::Code::End)
    {
        prompt_cursor_ = prompt_buffer_.size();
        return;
    }
    if(event.code == term::Code::Char)
    {
        if(event.ctrl)
        {
            if(event.ch == QChar('u'))
            {
                prompt_buffer_.clear();
                prompt_cursor_ = 0;
            }
            else if(event.ch == QChar('w'))
            {
                while(prompt_cursor_ > 0 && prompt_buffer_.at(prompt_cursor_ - 1).isSpace())
                {
                    prompt_buffer_.remove(prompt_cursor_ - 1, 1);
                    prompt_cursor_--;
                }
                while(prompt_cursor_ > 0 && !prompt_buffer_.at(prompt_cursor_ - 1).isSpace())
                {
                    prompt_buffer_.remove(prompt_cursor_ - 1, 1);
                    prompt_cursor_--;
                }
            }
            return;
        }
        if(prompt_fresh_)
        {
            prompt_buffer_.clear();
            prompt_cursor_ = 0;
            prompt_fresh_ = false;
        }
        prompt_buffer_.insert(prompt_cursor_, event.ch);
        prompt_cursor_++;
    }
}

void Tui::ask(const QString &title, const QString &initial, std::function<void(const QString &)> done)
{
    overlay_ = Overlay::Prompt;
    overlay_title_ = title;
    prompt_buffer_ = initial;
    prompt_cursor_ = initial.size();
    prompt_fresh_ = !initial.isEmpty();
    prompt_done_ = std::move(done);
}

void Tui::confirm(const QString &title, const QStringList &steps, std::function<void()> accepted)
{
    overlay_ = Overlay::Confirm;
    overlay_title_ = title;
    confirm_steps_ = steps;
    confirm_index_ = 0;
    confirm_accepted_ = std::move(accepted);
}

void Tui::menu(const QString &title, const QVector<MenuItem> &items)
{
    overlay_ = Overlay::Menu;
    overlay_title_ = title;
    menu_items_ = items;
}

void Tui::closeOverlay()
{
    overlay_ = Overlay::None;
    overlay_title_.clear();
    prompt_buffer_.clear();
    prompt_cursor_ = 0;
    prompt_fresh_ = false;
    prompt_done_ = nullptr;
    confirm_steps_.clear();
    confirm_index_ = 0;
    confirm_accepted_ = nullptr;
    menu_items_.clear();
}

// --- polling ---

void Tui::onTelemetryTick()
{
    if(pollsSuspended() || !ready())
        return;

    // Telemetry is only worth the bus time when it is being looked at or
    // recorded; the other tabs have their own polls.
    if(tab_ != Tab::Debug && !recorder_.active())
        return;

    const ServoStatus status = session_.readStatus(selectedId());

    // The user may have picked a different servo while the sample was in
    // flight. Attributing one servo's position to another would drag the goal
    // to a position belonging to a different joint.
    if(status.id != selectedId())
        return;

    appendSample(status);

    // The goal control follows the servo while nobody is driving it, so a
    // joint moved by hand -- or by a sweep -- does not leave a stale number
    // sitting under the next keystroke.
    if(controlFollowEnabled() && status.pos >= 0 && status.pos <= maxCount())
        goal_ = status.pos;
}

void Tui::onSearchTick()
{
    if(!searching_)
        return;

    // A longer operation has the bus. Come back to the scan once it is done.
    if(pollsSuspended())
        return;

    if(search_id_ > 0xfd || !session_.isOpen())
    {
        stopSearch();
        return;
    }

    const ServoEntry entry = session_.probe(search_id_);
    if(entry.id >= 0)
    {
        session_.addServo(entry);
        if(selected_index_ < 0)
            selectServoAt(session_.servos().size() - 1);
        setMessage(QString("Found ID %1 (%2)").arg(entry.id).arg(entry.model));
    }
    search_id_++;
}

void Tui::onTorqueTick()
{
    if(!session_.isOpen() || searching_ || session_.servos().isEmpty() || pollsSuspended())
        return;
    refreshTorqueStates();
}

void Tui::onProgrammingTick()
{
    if(tab_ != Tab::Programming || !ready() || pollsSuspended())
        return;

    const int count = static_cast<int>(registers().size());
    if(register_values_.size() != count)
        register_values_ = QVector<std::optional<int>>(count);

    // Only the rows on screen, the same way the window only refreshes the
    // visible part of its table: the whole map is a round trip per register.
    const int rows = qMax(1, register_visible_);
    const int first = qBound(0, register_scroll_, qMax(0, count - 1));
    const int last = qMin(count - 1, first + rows - 1);

    const int id = selectedId();
    for(int i = first; i <= last; i++)
        register_values_[i] = session_.readRegister(id, registers()[i]);
}

void Tui::onCalibrationTick()
{
    if(tab_ != Tab::Calibration || pollsSuspended())
        return;

    if(calib_recording_)
    {
        for(CalibRow &row : calib_rows_)
        {
            const auto position = session_.readPosition(row.id);
            if(!position.has_value())
                continue;
            row.min_seen = qMin(row.min_seen, *position);
            row.max_seen = qMax(row.max_seen, *position);
            row.range_min = QString::number(row.min_seen);
            row.range_max = QString::number(row.max_seen);
        }
        return;
    }

    calib_gate_tick_++;
    if(calib_gate_tick_ >= 5)
    {
        calib_gate_tick_ = 0;
        updateCalibrationGate(false);
    }
}

void Tui::onAutoDebugTick()
{
    if(!ready())
    {
        auto_debug_timer_->stop();
        sweep_running_ = false;
        step_running_ = false;
        return;
    }

    if(sweep_running_)
    {
        auto_goal_ = (auto_goal_ == auto_start_) ? auto_end_ : auto_start_;
    }
    else if(step_running_)
    {
        auto_goal_ += step_increasing_ ? auto_step_ : -auto_step_;
        if(auto_goal_ > auto_end_)
        {
            auto_goal_ = auto_end_;
            step_increasing_ = false;
        }
        else if(auto_goal_ < auto_start_)
        {
            auto_goal_ = auto_start_;
            step_increasing_ = true;
        }
    }
    else
    {
        auto_debug_timer_->stop();
        return;
    }

    goal_ = auto_goal_;
    session_.commandPosition(selectedId(), WriteMode::Write, auto_goal_, 0, 0, 0);
}

void Tui::onRecordTick()
{
    if(!recorder_.active() || !ready())
        return;
    recorder_.append(latest_);
}

// --- actions ---

void Tui::toggleConnection()
{
    if(session_.isOpen())
    {
        session_.close();
        selected_index_ = -1;
        torque_state_.clear();
        fault_state_.clear();
        calib_rows_.clear();
        calib_gate_open_ = false;
        stopSearch();
        setMessage("Port closed.");
        return;
    }

    QString error;
    if(!session_.open(options_, &error))
    {
        setMessage(error, true);
        return;
    }
    options_ = session_.options();
    setMessage(QString("Opened %1 at %2 baud. Press s to search.")
                   .arg(options_.port).arg(options_.baud));
}

void Tui::startSearch()
{
    if(!session_.isOpen())
    {
        setMessage("Serial port is not open. Press o first.", true);
        return;
    }

    searching_ = true;
    search_id_ = 0;
    selected_index_ = -1;
    session_.clearServos();
    torque_state_.clear();
    fault_state_.clear();
    calib_rows_.clear();
    calib_gate_open_ = false;

    // The IDs on the bus may be different servos this time round.
    session_.post([this] { session_.bus()->invalidateModeCaches(); });

    search_timer_->start(1);
    setMessage("Searching the bus. Press s again to stop.");
}

void Tui::stopSearch()
{
    if(!searching_)
        return;

    searching_ = false;
    search_timer_->stop();

    if(session_.servos().isEmpty())
    {
        setMessage("No servos answered. Check the baud rate, the wiring and the power.", true);
    }
    else
    {
        if(selected_index_ < 0)
            selectServoAt(0);
        refreshTorqueStates();
        setMessage(QString("%1 servo(s) found.").arg(session_.servos().size()));
    }
}

void Tui::selectServoAt(int index)
{
    if(index < 0 || index >= session_.servos().size())
        return;

    selected_index_ = index;
    register_row_ = 0;
    register_scroll_ = 0;
    register_values_ = QVector<std::optional<int>>(static_cast<int>(registers().size()));
    pos_buf_.clear();
    goal_buf_.clear();
    torque_buf_.clear();
    speed_buf_.clear();
    current_buf_.clear();
    temp_buf_.clear();
    voltage_buf_.clear();
    time_buf_.clear();

    if(session_.isOpen() && !searching_)
    {
        const auto position = session_.readPosition(selectedId());
        if(position.has_value() && *position >= 0 && *position <= maxCount())
            goal_ = *position;
    }
}

void Tui::stepSelection(int delta)
{
    if(session_.servos().isEmpty())
        return;

    const int count = session_.servos().size();
    const int next = (selected_index_ < 0) ? 0 : (selected_index_ + delta + count) % count;
    selectServoAt(next);
}

void Tui::refreshTorqueStates()
{
    if(!session_.isOpen() || searching_)
    {
        torque_state_.clear();
        fault_state_.clear();
        return;
    }

    const QVector<int> ids = session_.ids();
    const QVector<Session::ServoFlags> flags = session_.readFlags(ids);
    if(flags.size() != ids.size())
        return;

    int on = 0;
    int off = 0;
    for(int i = 0; i < ids.size(); i++)
    {
        torque_state_[ids[i]] = flags[i].torque;
        fault_state_[ids[i]] = flags[i].fault;
        if(flags[i].torque > 0)
            on++;
        else if(flags[i].torque == 0)
            off++;
    }

    all_torque_on_ = on > 0 || off == 0;
    if(servoSelected())
        torque_selected_on_ = torque_state_.value(selectedId(), 0) != 0;
}

void Tui::toggleTorqueAll()
{
    if(!session_.isOpen())
    {
        setMessage("Serial port is not open.", true);
        return;
    }
    if(searching_)
    {
        setMessage("A servo search is still running.", true);
        return;
    }
    if(session_.servos().isEmpty())
    {
        setMessage("No servos detected. Press s to search first.", true);
        return;
    }

    const bool on = !all_torque_on_;
    const QVector<int> ids = session_.ids();
    const QStringList failed = session_.setTorque(ids, on);

    refreshTorqueStates();

    if(failed.isEmpty())
    {
        setMessage(QString("Torque %1 on %2 servo(s).").arg(on ? "on" : "off").arg(ids.size()));
        return;
    }
    setMessage(QString("ID %1 did not report Torque Enable = %2. Those servos may still be in "
                       "the previous state.").arg(failed.join(", ")).arg(on ? 1 : 0), true);
}

void Tui::toggleTorqueSelected()
{
    if(!ready())
        return;

    const bool on = !torque_selected_on_;
    const QStringList failed = session_.setTorque({selectedId()}, on);
    refreshTorqueStates();

    if(failed.isEmpty())
        setMessage(QString("ID %1 torque %2.").arg(selectedId()).arg(on ? "on" : "off"));
    else
        setMessage(QString("ID %1 did not accept the torque change.").arg(selectedId()), true);
}

bool Tui::controlFollowEnabled() const
{
    return ready()
        && !sweep_running_
        && !step_running_
        && overlay_ == Overlay::None
        && control_touch_.elapsed() > 1500;
}

void Tui::sendGoal(int goal)
{
    noteControlInteraction();

    goal_ = qBound(0, goal, maxCount());

    if(!ready())
        return;

    if(mode_ == WriteMode::SyncWrite)
        session_.syncCommandPosition(session_.ids(), goal_, time_, speed_, acc_);
    else
        session_.commandPosition(selectedId(), mode_, goal_, time_, speed_, acc_);
}

void Tui::setGoalFromAngle(double angle)
{
    const int goal = angleToCounts(selectedSeries(), angle, angle_in_radians_);
    if(goal < 0 || goal > maxCount())
    {
        setMessage(QString("%1 %2 maps to position %3, which is outside the encoder range 0..%4.")
                       .arg(angle).arg(angle_in_radians_ ? "rad" : "deg").arg(goal).arg(maxCount()),
                   true);
        return;
    }
    sendGoal(goal);
}

void Tui::toggleSweep()
{
    if(!ready())
        return;

    if(sweep_running_)
    {
        sweep_running_ = false;
        auto_debug_timer_->stop();
        setMessage("Sweep stopped.");
        return;
    }

    step_running_ = false;
    sweep_running_ = true;
    auto_goal_ = auto_start_;
    goal_ = auto_goal_;
    session_.commandPosition(selectedId(), WriteMode::Write, auto_goal_, 0, 0, 0);
    auto_debug_timer_->start(auto_hold_ms_);
    setMessage(QString("Sweeping %1 to %2 every %3 ms.")
                   .arg(auto_start_).arg(auto_end_).arg(auto_hold_ms_));
}

void Tui::toggleStep()
{
    if(!ready())
        return;

    if(step_running_)
    {
        step_running_ = false;
        auto_debug_timer_->stop();
        setMessage("Step stopped.");
        return;
    }

    sweep_running_ = false;
    step_running_ = true;
    step_increasing_ = true;
    auto_goal_ = auto_start_;
    goal_ = auto_goal_;
    session_.commandPosition(selectedId(), WriteMode::Write, auto_goal_, 0, 0, 0);
    auto_debug_timer_->start(auto_delay_ms_);
    setMessage(QString("Stepping %1 counts every %2 ms between %3 and %4.")
                   .arg(auto_step_).arg(auto_delay_ms_).arg(auto_start_).arg(auto_end_));
}

void Tui::toggleRecording()
{
    if(recorder_.active())
    {
        recorder_.stop();
        setMessage(QString("Recording stopped. %1 rows in %2.")
                       .arg(recorder_.rows()).arg(recorder_.path()));
        return;
    }

    if(!ready())
    {
        setMessage("No servo selected.", true);
        return;
    }

    QString error;
    if(!recorder_.start(record_path_, record_interval_, &error))
    {
        setMessage(error, true);
        return;
    }
    setMessage(QString("Recording ID %1 to %2.").arg(selectedId()).arg(recorder_.path()));
}

void Tui::writeSelectedRegister(const QString &text)
{
    if(!ready() || register_row_ < 0 || register_row_ >= static_cast<int>(registers().size()))
        return;

    const MemoryConfig config = registers()[register_row_];

    bool ok = false;
    const int value = text.toInt(&ok);
    if(!ok)
    {
        setMessage("That is not a number.", true);
        return;
    }

    const int before = selectedId();
    const RegisterWriteResult result = session_.writeRegister(before, config, value);

    // Writing the ID register moved the servo, so follow it to its new address.
    if(result.effective_id >= 0 && result.effective_id != before)
    {
        QVector<ServoEntry> servos = session_.servos();
        for(ServoEntry &entry : servos)
        {
            if(entry.id == before)
                entry.id = result.effective_id;
        }
        session_.setServos(servos);
        torque_state_.remove(before);
        fault_state_.remove(before);
    }

    const QString area = config.is_eprom ? "EPROM" : "SRAM";
    if(result.ok)
    {
        prog_status_ = QString("[%1] %2 = %3 saved to %4")
                           .arg(config.address).arg(config.name).arg(value).arg(area);
        setMessage(prog_status_);
    }
    else
    {
        prog_status_ = QString("[%1] %2: write failed").arg(config.address).arg(config.name);
        setMessage(QString("Address %1 (%2) still does not hold %3 after the write. The servo "
                           "either did not respond or rejected the value (out of range, or "
                           "outside the %4 area).")
                       .arg(config.address).arg(config.name).arg(value).arg(area),
                   true);
    }
}

void Tui::saveRegisterSnapshot(const QString &path)
{
    if(!ready())
        return;

    const int id = selectedId();
    RegisterSnapshot snapshot;
    snapshot.id = id;
    snapshot.series = seriesName(selectedSeries());
    snapshot.model = session_.modelFor(id);

    QStringList unread;
    for(const MemoryConfig &config : registers())
    {
        const std::optional<int> value = session_.readRegister(id, config);
        if(!value.has_value())
        {
            unread << QString::number(config.address);
            continue;
        }
        RegisterEntry entry;
        entry.address = config.address;
        entry.name = config.name;
        entry.writable = !config.is_readonly;
        entry.value = *value;
        snapshot.registers.append(entry);
    }

    if(snapshot.registers.isEmpty())
    {
        setMessage("No registers could be read from the servo.", true);
        return;
    }

    QFile file(expandHome(path));
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        setMessage(QString("Could not write %1.").arg(path), true);
        return;
    }
    file.write(buildRegisterSnapshotJson(snapshot));
    file.close();

    prog_status_ = QString("Saved %1 register(s) from ID %2 to %3")
                       .arg(snapshot.registers.size()).arg(id).arg(expandHome(path));
    if(!unread.isEmpty())
        prog_status_ += QString(" (no reply for address %1)").arg(unread.join(", "));
    setMessage(prog_status_);
}

void Tui::loadRegisterSnapshot(const QString &path)
{
    if(!ready())
        return;

    QFile file(expandHome(path));
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        setMessage(QString("Could not read %1.").arg(path), true);
        return;
    }
    const QByteArray data = file.readAll();
    file.close();

    RegisterSnapshot snapshot;
    QString error;
    if(!parseRegisterSnapshotJson(data, &snapshot, &error))
    {
        setMessage(QString("%1: %2").arg(path, error), true);
        return;
    }

    const QString target_series = seriesName(selectedSeries());
    if(snapshot.series != target_series)
    {
        setMessage(QString("This file was saved from a %1 servo but ID %2 is %3. Register "
                           "addresses differ between series, so loading it would write the "
                           "wrong registers. Refusing.")
                       .arg(snapshot.series).arg(selectedId()).arg(target_series),
                   true);
        return;
    }

    QVector<QPair<MemoryConfig, int>> planned;
    int skipped_readonly = 0;
    int skipped_unknown = 0;
    bool skipped_id = false;

    for(const RegisterEntry &entry : snapshot.registers)
    {
        const MemoryConfig *config = nullptr;
        for(const MemoryConfig &item : registers())
        {
            if(item.address == entry.address)
            {
                config = &item;
                break;
            }
        }
        if(config == nullptr)
        {
            skipped_unknown++;
            continue;
        }
        if(config->is_readonly)
        {
            skipped_readonly++;
            continue;
        }
        if(config->address == 5)
        {
            skipped_id = true;
            continue;
        }
        planned.append(qMakePair(*config, entry.value));
    }

    if(planned.isEmpty())
    {
        setMessage("Nothing in this file is writable to the servo.", true);
        return;
    }

    int eprom_count = 0;
    for(const auto &item : planned)
    {
        if(item.first.is_eprom)
            eprom_count++;
    }

    const int id = selectedId();
    QStringList steps;
    steps << QString("About to write %1 register(s) to servo ID %2 from:\n%3\n\n"
                     "%4 of them are EPROM registers, so those changes are permanent.")
                 .arg(planned.size()).arg(id).arg(expandHome(path)).arg(eprom_count);
    steps << QString("This replaces the servo's current configuration and cannot be undone.\n\n"
                     "Position Offset Value, Min/Max Position Limit, PID gains, torque limits and "
                     "protection settings will all be overwritten by the values in the file. "
                     "Save the current registers first if you might want them back.");
    steps << QString("Final confirmation.\n\n"
                     "Target: ID %1 (%2)\nSource file series: %3\nRegisters to write: %4\n\n"
                     "The servo ID register is never written by a load, so ID %1 stays as it is.")
                 .arg(id).arg(target_series).arg(snapshot.series).arg(planned.size());

    confirm("Load Registers", steps, [this, planned, skipped_readonly, skipped_unknown, skipped_id] {
        const int id = selectedId();
        QStringList failed;
        for(const auto &item : planned)
        {
            if(!session_.writeRegister(id, item.first, item.second).ok)
                failed << QString::number(item.first.address);
        }

        QStringList notes;
        notes << QString("wrote %1").arg(planned.size() - failed.size());
        if(!failed.isEmpty())
            notes << QString("failed %1").arg(failed.size());
        if(skipped_readonly > 0)
            notes << QString("skipped %1 read-only").arg(skipped_readonly);
        if(skipped_unknown > 0)
            notes << QString("skipped %1 unknown").arg(skipped_unknown);
        if(skipped_id)
            notes << "skipped ID";

        prog_status_ = "Load: " + notes.join(", ");
        setMessage(failed.isEmpty()
                       ? prog_status_
                       : prog_status_ + QString(". Did not verify: %1").arg(failed.join(", ")),
                   !failed.isEmpty());
        refreshTorqueStates();
    });
}

void Tui::setMidpoint()
{
    if(!ready())
    {
        setMessage("No servo selected.", true);
        return;
    }

    const ModelSeries series = selectedSeries();
    const MemoryConfig *offset = findMemConfig(series, "Position Offset Value");
    if(!supportsMidpointCalibration(series) || offset == nullptr)
    {
        setMessage("This servo series has no Position Offset Value register.", true);
        return;
    }

    const MemoryConfig config = *offset;
    const int id = selectedId();

    confirm("Set Midpoint",
            {QString("Store the current position of servo ID %1 as the midpoint (%2)?\n\n"
                     "This overwrites Position Offset Value (address %3) in EPROM, so it "
                     "survives a power cycle. The servo does not move.")
                 .arg(id).arg(countsPerRev() / 2).arg(config.address)},
            [this, id, config] {
                const MidpointResult result = session_.setMidpoint(id, config);
                if(!result.acked || result.after < 0)
                {
                    prog_status_ = "Set midpoint: no response";
                    setMessage("No response from the servo. The midpoint was not changed.", true);
                    return;
                }

                const int decoded = feetech_servo::decodeSignMagnitude(result.after, config.dir_bit);
                prog_status_ = QString("Midpoint set: offset = %1").arg(decoded);
                if(result.before >= 0 && result.after == result.before)
                {
                    setMessage(QString("Position Offset Value is unchanged (%1). Either the servo "
                                       "was already centred here, or the firmware rejected the "
                                       "calibration command.").arg(decoded), true);
                }
                else
                {
                    setMessage(prog_status_);
                }
            });
}

// --- calibration ---

void Tui::updateCalibrationGate(bool force)
{
    if(calib_recording_)
        return;

    QString message;
    bool open = false;

    if(!session_.isOpen())
    {
        message = "Serial port is not open.";
    }
    else if(searching_)
    {
        message = "A servo search is still running.";
    }
    else if(session_.servos().isEmpty())
    {
        message = "No servos detected. Press s to search first.";
    }
    else
    {
        const QVector<int> ids = session_.ids();
        const QVector<Session::ServoFlags> flags = session_.readFlags(ids);
        if(flags.size() != ids.size())
            return;

        QStringList torque_on;
        QStringList silent;
        for(int i = 0; i < ids.size(); i++)
        {
            if(flags[i].torque < 0)
                silent << QString::number(ids[i]);
            else if(flags[i].torque != 0)
                torque_on << QString::number(ids[i]);
        }

        if(!silent.isEmpty())
        {
            message = QString("No response from ID %1.").arg(silent.join(", "));
        }
        else if(!torque_on.isEmpty())
        {
            message = QString("Torque is still enabled on ID %1. Press t to release it.")
                          .arg(torque_on.join(", "));
        }
        else
        {
            message = QString("Torque disabled on %1 servo(s). Calibration is available.")
                          .arg(ids.size());
            open = true;
        }
    }

    calib_status_ = message;

    if(open != calib_gate_open_)
    {
        calib_gate_open_ = open;
        if(open)
            populateCalibrationTable();
    }
    else if(open && (force || calib_rows_.size() != session_.servos().size()))
    {
        populateCalibrationTable();
    }
}

void Tui::populateCalibrationTable()
{
    QMap<int, QString> previous_names;
    QMap<int, int> previous_drive;
    for(const CalibRow &row : calib_rows_)
    {
        previous_names[row.id] = row.name;
        previous_drive[row.id] = row.drive_mode;
    }

    QVector<ServoEntry> servos = session_.servos();
    std::sort(servos.begin(), servos.end(),
              [](const ServoEntry &a, const ServoEntry &b) { return a.id < b.id; });

    calib_rows_.clear();
    for(const ServoEntry &entry : servos)
    {
        CalibRow row;
        row.id = entry.id;
        row.series = entry.series;
        row.name = previous_names.value(entry.id);
        row.drive_mode = previous_drive.value(entry.id, 0);
        calib_rows_.append(row);
    }
    calib_row_ = qBound(0, calib_row_, qMax(0, calib_rows_.size() - 1));
}

void Tui::applyJointNames(const QString &text)
{
    const QStringList names = splitFields(text, QRegExp("[,;\n\t]"));
    if(names.isEmpty())
    {
        setMessage("No joint names found. Separate them with commas, in ascending servo ID order.",
                   true);
        return;
    }

    const int applied = qMin(calib_rows_.size(), names.size());
    for(int i = 0; i < applied; i++)
        calib_rows_[i].name = names[i];

    if(names.size() == calib_rows_.size())
    {
        setMessage(QString("Applied %1 joint name(s) in servo ID order.").arg(applied));
    }
    else
    {
        setMessage(QString("%1 name(s) given for %2 detected servo(s); the first %3 row(s) were "
                           "filled in ID order.")
                       .arg(names.size()).arg(calib_rows_.size()).arg(applied),
                   true);
    }
}

void Tui::calibrationSetHome()
{
    if(calib_rows_.isEmpty())
        return;

    QStringList affected;
    QStringList positions;
    for(const CalibRow &row : calib_rows_)
    {
        affected << QString::number(row.id);
        const auto position = session_.readPosition(row.id);
        positions << QString("    ID %1: %2")
                         .arg(row.id)
                         .arg(position.has_value() ? QString::number(*position) : QString("no reply"));
    }

    const QStringList steps = buildSetHomeConfirmations(calib_rows_.size(), affected.join(", "),
                                                        positions.join("\n"));

    confirm("Set Home", steps, [this] {
        QVector<int> ids;
        for(const CalibRow &row : calib_rows_)
            ids.append(row.id);

        const QVector<Session::HomeResult> results = session_.setHome(ids);
        if(results.size() != calib_rows_.size())
        {
            setMessage("The calibration write did not complete.", true);
            return;
        }

        QStringList failed;
        QStringList mismatched;
        for(int i = 0; i < results.size(); i++)
        {
            const Session::HomeResult &result = results[i];
            CalibRow &row = calib_rows_[i];

            if(!result.written)
            {
                failed << QString::number(result.id);
                continue;
            }
            if(result.corrected < 0 || std::abs(result.corrected - result.half_turn) > 2)
            {
                mismatched << QString("ID %1 reads %2, expected %3")
                                  .arg(result.id)
                                  .arg(result.corrected >= 0 ? QString::number(result.corrected)
                                                             : QString("no reply"))
                                  .arg(result.half_turn);
            }

            row.homing_offset = QString::number(result.homing_offset);
            row.min_seen = result.half_turn;
            row.max_seen = result.half_turn;
            row.range_min = QString::number(result.half_turn);
            row.range_max = QString::number(result.half_turn);
        }

        if(!failed.isEmpty())
        {
            setMessage(QString("Could not write calibration to ID %1.").arg(failed.join(", ")), true);
        }
        else if(!mismatched.isEmpty())
        {
            setMessage(QString("Offset written, but the servo does not report the midpoint: %1. "
                               "The firmware may apply the offset with the opposite sign.")
                           .arg(mismatched.join("; ")),
                       true);
        }
        else
        {
            calib_status_ = QString("Home set on %1 servo(s). Now record the range of motion.")
                                .arg(calib_rows_.size());
            setMessage(calib_status_);
        }
    });
}

void Tui::toggleRangeRecording()
{
    if(calib_recording_)
    {
        calib_recording_ = false;
        calib_status_ = "Recording stopped. Check the ranges, then export.";
        setMessage(calib_status_);
        return;
    }

    if(calib_rows_.isEmpty())
        return;

    for(CalibRow &row : calib_rows_)
    {
        const auto position = session_.readPosition(row.id);
        if(!position.has_value())
        {
            setMessage(QString("No response from ID %1.").arg(row.id), true);
            return;
        }
        row.min_seen = *position;
        row.max_seen = *position;
        row.range_min = QString::number(*position);
        row.range_max = QString::number(*position);
    }

    calib_recording_ = true;
    calib_status_ = "Recording. Move every joint slowly through its full range.";
    setMessage(calib_status_);
}

void Tui::exportCalibration(const QString &path)
{
    QVector<JointCalibration> joints;
    for(const CalibRow &row : calib_rows_)
    {
        JointCalibration joint;
        joint.name = row.name.trimmed();
        joint.id = row.id;
        joint.drive_mode = row.drive_mode;
        joint.homing_offset = row.homing_offset.toInt();
        joint.range_min = row.range_min.toInt();
        joint.range_max = row.range_max.toInt();
        joints.append(joint);
    }

    const QStringList problems = validateCalibrationJoints(joints);
    if(!problems.isEmpty())
    {
        setMessage(QString("Fix these before exporting: %1").arg(problems.join(" ")), true);
        return;
    }

    QFile file(expandHome(path));
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        setMessage(QString("Could not write %1.").arg(path), true);
        return;
    }

    QTextStream stream(&file);
    stream << buildCalibrationJson(joints);
    file.close();

    calib_status_ = QString("Exported %1 joint(s) to %2").arg(joints.size()).arg(expandHome(path));
    setMessage(calib_status_);
}

// --- drawing ---

QString Tui::renderTitle(int width) const
{
    term::Line line(width);
    line.back(COL_PANEL).color(COL_ACCENT).bold().text(" ServoBench ").off().back(COL_PANEL);

    line.color(COL_DIM).text(" ");
    if(session_.isOpen())
    {
        line.color(COL_RELEASED).text(dot() + " ");
        const QString parity = options_.parity == QSerialPort::NoParity ? "N"
                             : options_.parity == QSerialPort::OddParity ? "O" : "E";
        line.color(COL_TEXT).text(QString("%1 %2 ").arg(shortPort(options_.port)).arg(options_.baud)
                                  + "8" + parity + "1");
    }
    else
    {
        line.color(COL_UNKNOWN).text(dot() + " ");
        line.color(COL_DIM).text(QString("%1 closed").arg(options_.port.isEmpty() ? QString("no port")
                                                                                 : shortPort(options_.port)));
    }

    line.color(COL_BORDER).text(term::unicode ? QString("  %1  ").arg(QChar(0x2502)) : QString("  | "));

    if(servoSelected())
    {
        const ServoEntry &entry = session_.servos()[selected_index_];
        line.color(servoColor(entry.id)).text(dot() + " ");
        line.color(COL_TEXT).text(QString("ID %1  %2  %3")
                                      .arg(entry.id).arg(entry.model).arg(seriesName(entry.series)));
    }
    else
    {
        line.color(COL_DIM).text("no servo selected");
    }

    // Telemetry keeps the bus busy almost continuously, so only a transaction
    // slow enough to be worth waiting for says so.
    if(session_.busy() && busy_since_.isValid() && busy_since_.elapsed() > 150)
        line.color(COL_ENGAGED).text("   working...");

    line.color(COL_FAINT).right("?  keys    q  quit ");
    return line.str();
}

QString Tui::renderTabs(int width) const
{
    term::Line line(width);
    const QString names[3] = {"Debug", "Programming", "Calibration"};
    const Tab tabs[3] = {Tab::Debug, Tab::Programming, Tab::Calibration};

    line.text(" ");
    for(int i = 0; i < 3; i++)
    {
        if(tabs[i] == tab_)
            line.back(COL_ACCENT).color("#0B0E11").bold().text(QString(" %1 ").arg(names[i])).off();
        else
            line.color(COL_DIM).text(QString(" %1 ").arg(names[i]));
        line.color(COL_BORDER).text(i < 2 ? (term::unicode ? QString(QChar(0x2502)) : QString("|"))
                                          : QString(" "));
    }

    QString note;
    if(searching_)
        note = QString("searching: ping ID %1").arg(search_id_);
    else if(recorder_.active())
        note = QString("recording: %1 rows").arg(recorder_.rows());
    else if(sweep_running_)
        note = "sweep running";
    else if(step_running_)
        note = "step running";
    else if(calib_recording_)
        note = "recording range";

    if(!note.isEmpty())
        line.color(COL_ENGAGED).right(note + " ");
    return line.str();
}

QStringList Tui::renderLeftPanel(int height) const
{
    QStringList lines;
    const int w = PANEL_WIDTH;

    auto heading = [&](const QString &text) {
        lines << term::Line(w).color(COL_FAINT).text(" " + text.toUpper()).pad(w).str();
    };
    auto field = [&](const QString &name, const QString &value, const QString &color) {
        lines << term::Line(w).color(COL_DIM).text(QString("  %1 ").arg(name, -8))
                     .color(color).text(value).pad(w).str();
    };

    heading("connection");
    field("port", options_.port.isEmpty() ? QString("-") : shortPort(options_.port), COL_TEXT);
    field("baud", QString::number(options_.baud), COL_TEXT);
    field("parity", options_.parity == QSerialPort::NoParity ? "none"
                  : options_.parity == QSerialPort::OddParity ? "odd" : "even", COL_TEXT);
    field("timeout", QString("%1 ms").arg(options_.timeout), COL_TEXT);
    lines << term::Line(w).color(COL_DIM).text("  o  ")
                 .color(session_.isOpen() ? COL_RELEASED : COL_UNKNOWN)
                 .text(session_.isOpen() ? "close port" : "open port").pad(w).str();

    lines << term::Line(w).pad(w).str();
    heading(QString("servos (%1)").arg(session_.servos().size()));

    if(session_.servos().isEmpty())
    {
        lines << term::Line(w).color(COL_FAINT)
                     .text(searching_ ? "  scanning the bus..." : "  s  search the bus").pad(w).str();
    }

    // The list scrolls around the selection, so a 20-servo arm stays usable in
    // a short terminal.
    const int reserved = static_cast<int>(lines.size()) + 3;
    const int room = qMax(3, height - reserved);
    int first = 0;
    if(session_.servos().size() > room && selected_index_ >= 0)
        first = qBound(0, selected_index_ - room / 2, session_.servos().size() - room);

    for(int i = first; i < session_.servos().size() && i < first + room; i++)
    {
        const ServoEntry &entry = session_.servos()[i];
        const QString color = servoColor(entry.id);
        term::Line line(w);

        if(i == selected_index_)
            line.back("#22303F");

        line.color(color).text(" " + dot() + " ");
        line.color(i == selected_index_ ? COL_TEXT : COL_DIM)
            .text(QString("%1  %2").arg(entry.id, -4).arg(entry.model));

        const int fault = fault_state_.value(entry.id, -1);
        if(fault > 0)
        {
            const QStringList faults = decodeServoStatus(entry.series, fault);
            line.color(COL_ALARM).text("  " + faults.value(0));
        }
        lines << line.pad(w).str();
    }

    if(session_.servos().size() > first + room)
    {
        lines << term::Line(w).color(COL_FAINT)
                     .text(QString("  ... %1 more").arg(session_.servos().size() - first - room))
                     .pad(w).str();
    }

    while(lines.size() < height - 2)
        lines << term::Line(w).pad(w).str();

    const QString summary = torqueSummary();
    const QString color = summary == "all off" ? COL_RELEASED
                        : summary == "all on" ? COL_ENGAGED
                        : summary == "unknown" ? COL_UNKNOWN : COL_ALARM;
    lines << term::Line(w).color(COL_DIM).text("  torque  ").color(color).text(summary).pad(w).str();
    lines << term::Line(w).color(COL_DIM).text("  t  ")
                 .color(COL_TEXT).text(all_torque_on_ ? "release all" : "engage all").pad(w).str();

    while(lines.size() > height)
        lines.removeLast();
    while(lines.size() < height)
        lines << term::Line(w).pad(w).str();
    return lines;
}

QStringList Tui::renderDebug(int height, int width)
{
    QStringList lines;

    const int plot_height = qMax(6, height - 8);
    const PlotGeometry geometry = plotGeometry(plot_height, width);
    plot_left_ = PANEL_WIDTH + 1 + geometry.left_cell;
    plot_top_ = 2;
    plot_cols_ = geometry.area_cols;
    plot_rows_ = geometry.area_rows;

    PlotInput input;
    const Ring<int> *buffers[7] = {&pos_buf_, &goal_buf_, &torque_buf_, &speed_buf_,
                                   &current_buf_, &temp_buf_, &voltage_buf_};
    const bool left_axis[7] = {true, true, false, false, false, false, false};
    const double gain[7] = {1.0, 1.0, 1.0, 0.2, 1.0, 1.0, 1.0};
    const char marks[7] = {'o', '+', '*', 'x', '.', '#', '~'};

    for(int i = 0; i < 7; i++)
    {
        PlotSeries series;
        series.name = SERIES_NAMES[i];
        series.color = SERIES_COLORS[i];
        series.visible = series_visible_[i];
        series.data = buffers[i];
        series.left_axis = left_axis[i];
        series.gain = gain[i];
        series.mark = marks[i];
        input.series.append(series);
    }
    input.times = &time_buf_;
    input.now_ms = clock_.elapsed();
    input.up_limit = up_limit_;
    input.down_limit = down_limit_;
    input.cursor_cell = cursor_cell_;

    const PlotOutput plot = renderPlot(plot_height, width, input, view_);
    lines << plot.lines;

    // Legend, doubling as the key map for the series toggles.
    term::Line legend(width);
    legend.text(" ");
    for(int i = 0; i < 7; i++)
    {
        legend.color(COL_FAINT).text(QString::number(i + 1));
        legend.color(SERIES_COLORS[i]).text(series_visible_[i] ? dot() : QString("-"));
        legend.color(series_visible_[i] ? COL_TEXT : COL_FAINT).text(SERIES_NAMES[i]);
        legend.text(" ");
    }
    if(up_limit_ != 0 || down_limit_ != 0)
        legend.color("#FF6B9D").text(QString(" limits %1/%2").arg(up_limit_).arg(down_limit_));
    lines << legend.pad(width).str();

    const ModelSeries series = selectedSeries();
    term::Line readout(width);
    readout.color(COL_DIM).text(" pos ").color(COL_TEXT).text(QString::number(latest_.pos));
    readout.color(COL_DIM).text("  angle ")
           .color(COL_TEXT).text(QString("%1%2 / %3 rad")
                                     .arg(countsToDegrees(series, latest_.pos), 0, 'f', 1)
                                     .arg(degreeSign())
                                     .arg(countsToRadians(series, latest_.pos), 0, 'f', 3));
    readout.color(COL_DIM).text("  goal ").color(COL_TEXT).text(QString::number(latest_.goal));
    readout.color(COL_DIM).text("  moving ").color(COL_TEXT).text(QString::number(latest_.move));
    lines << readout.pad(width).str();

    term::Line readout2(width);
    readout2.color(COL_DIM).text(" load ").color(COL_TEXT).text(QString::number(latest_.torque));
    readout2.color(COL_DIM).text("  speed ").color(COL_TEXT).text(QString::number(latest_.speed));
    readout2.color(COL_DIM).text("  current ").color(COL_TEXT).text(QString::number(latest_.current));
    readout2.color(COL_DIM).text("  temp ").color(COL_TEXT).text(QString("%1 C").arg(latest_.temp));
    readout2.color(COL_DIM).text("  volt ")
            .color(COL_TEXT).text(QString("%1 V").arg(latest_.voltage / 10.0, 0, 'f', 1));
    lines << readout2.pad(width).str();

    // Goal slider, the one control that is worth drawing rather than printing.
    const int bar_width = qMax(10, width - 34);
    term::Line slider(width);
    slider.color(COL_DIM).text(" goal ");
    slider.color(COL_ACCENT).text(bar(maxCount() > 0 ? double(goal_) / maxCount() : 0.0, bar_width));
    slider.color(COL_TEXT).text(QString(" %1 / %2").arg(goal_).arg(maxCount()));
    lines << slider.pad(width).str();

    term::Line control(width);
    control.color(COL_DIM).text(" mode ")
           .color(COL_TEXT).text(mode_ == WriteMode::Write ? "write"
                                : mode_ == WriteMode::SyncWrite ? "sync write" : "reg write");
    control.color(COL_DIM).text("   torque ")
           .color(torque_selected_on_ ? COL_ENGAGED : COL_RELEASED)
           .text(torque_selected_on_ ? "on" : "off");
    control.color(COL_DIM).text("   speed ").color(COL_TEXT).text(QString::number(speed_));
    control.color(COL_DIM).text("  acc ").color(COL_TEXT).text(QString::number(acc_));
    control.color(COL_DIM).text("  time ").color(COL_TEXT).text(QString::number(time_));
    control.color(COL_DIM).text("   angle unit ").color(COL_TEXT)
           .text(angle_in_radians_ ? "rad" : "deg");
    lines << control.pad(width).str();

    term::Line automation(width);
    automation.color(COL_DIM).text(" sweep ")
              .color(sweep_running_ ? COL_ENGAGED : COL_TEXT)
              .text(QString("%1 to %2 hold %3ms").arg(auto_start_).arg(auto_end_).arg(auto_hold_ms_));
    automation.color(COL_DIM).text("   step ")
              .color(step_running_ ? COL_ENGAGED : COL_TEXT)
              .text(QString("%1 every %2ms").arg(auto_step_).arg(auto_delay_ms_));
    if(sweep_running_ || step_running_)
        automation.color(COL_ENGAGED).text(QString("   -> %1").arg(auto_goal_));
    lines << automation.pad(width).str();

    term::Line record(width);
    record.color(COL_DIM).text(" record ")
          .color(recorder_.active() ? COL_ENGAGED : COL_TEXT)
          .text(recorder_.active() ? QString("%1  %2 rows").arg(recorder_.path()).arg(recorder_.rows())
                                   : QString("%1  idle").arg(record_path_));

    if(!plot.readout.isEmpty())
    {
        record.color(COL_DIM).text("    ");
        for(int i = 0; i < plot.readout.size(); i++)
            record.color(plot.readout_colors.value(i, COL_TEXT)).text(plot.readout[i] + "  ");
    }
    lines << record.pad(width).str();

    while(lines.size() < height)
        lines << term::Line(width).pad(width).str();
    while(lines.size() > height)
        lines.removeLast();
    return lines;
}

QStringList Tui::renderProgramming(int height, int width)
{
    QStringList lines;
    const auto &configs = registers();
    const int count = static_cast<int>(configs.size());

    lines << term::Line(width).color(COL_FAINT)
                 .text(QString(" %1  %2  %3  %4  %5")
                           .arg("ADDR", -5).arg("REGISTER", -30).arg("VALUE", 8)
                           .arg("AREA", -6).arg("R/W", -4))
                 .pad(width).str();

    const int table_rows = qMax(3, height - 3);
    register_visible_ = table_rows;
    register_scroll_ = qBound(qMax(0, register_row_ - table_rows + 1),
                              register_scroll_,
                              qMax(0, qMin(register_row_, count - table_rows)));

    for(int i = register_scroll_; i < count && i < register_scroll_ + table_rows; i++)
    {
        const MemoryConfig &config = configs[i];
        std::optional<int> value;
        if(i < register_values_.size())
            value = register_values_[i];
        const bool selected = i == register_row_;

        term::Line line(width);
        if(selected)
            line.back("#22303F");

        line.color(selected ? COL_TEXT : (config.is_readonly ? COL_FAINT : COL_DIM));
        line.text(QString(" %1  %2  %3  %4  %5")
                      .arg(config.address, -5)
                      .arg(config.name, -30)
                      .arg(value.has_value() ? QString::number(*value) : QString("-"), 8)
                      .arg(config.is_eprom ? "EPROM" : "SRAM", -6)
                      .arg(config.is_readonly ? "R" : "R/W", -4));
        lines << line.pad(width).str();
    }

    while(lines.size() < height - 2)
        lines << term::Line(width).pad(width).str();

    lines << term::Line(width).color(COL_DIM).text(" " + prog_status_).pad(width).str();

    const MemoryConfig &current = configs[qBound(0, register_row_, count - 1)];
    QString detail = QString(" [%1] %2 - %3, %4")
                         .arg(current.address).arg(current.name)
                         .arg(current.is_eprom ? "EPROM (permanent)" : "SRAM (until power off)")
                         .arg(current.is_readonly ? "read-only" : "writable");
    // A -1/-1 range in the table means "not documented", not an actual range.
    if(!current.is_readonly && !(current.min_val == -1 && current.max_val == -1))
        detail += QString(", documented range %1..%2").arg(current.min_val).arg(current.max_val);
    lines << term::Line(width).color(COL_FAINT).text(detail).pad(width).str();

    while(lines.size() > height)
        lines.removeLast();
    return lines;
}

QStringList Tui::renderCalibration(int height, int width) const
{
    QStringList lines;

    lines << term::Line(width).color(calib_gate_open_ ? COL_RELEASED : COL_ENGAGED)
                 .text(" " + calib_status_).pad(width).str();
    lines << term::Line(width).pad(width).str();

    lines << term::Line(width).color(COL_FAINT)
                 .text(QString(" %1  %2  %3  %4  %5  %6")
                           .arg("ID", -4).arg("JOINT NAME", -22).arg("DRIVE", -6)
                           .arg("HOMING", 8).arg("RANGE MIN", 10).arg("RANGE MAX", 10))
                 .pad(width).str();

    for(int i = 0; i < calib_rows_.size(); i++)
    {
        const CalibRow &row = calib_rows_[i];
        term::Line line(width);
        if(i == calib_row_)
            line.back("#22303F");

        line.color(i == calib_row_ ? COL_TEXT : COL_DIM);
        line.text(QString(" %1  %2  %3  %4  %5  %6")
                      .arg(row.id, -4)
                      .arg(row.name.isEmpty() ? QString("(unnamed)") : row.name, -22)
                      .arg(row.drive_mode, -6)
                      .arg(row.homing_offset, 8)
                      .arg(row.range_min, 10)
                      .arg(row.range_max, 10));
        lines << line.pad(width).str();
    }

    if(calib_rows_.isEmpty())
    {
        lines << term::Line(width).color(COL_FAINT)
                     .text("  No servos yet. Search the bus, release torque, then press k.")
                     .pad(width).str();
    }

    while(lines.size() < height - 3)
        lines << term::Line(width).pad(width).str();

    lines << term::Line(width).color(COL_FAINT)
                 .text(" One row per detected servo, in ascending servo ID order. Paste the joint "
                       "names in that same order with j.")
                 .pad(width).str();
    lines << term::Line(width).color(COL_FAINT)
                 .text(" 1. H set home (middle pose)   2. R record range   3. X export JSON")
                 .pad(width).str();

    while(lines.size() > height)
        lines.removeLast();
    while(lines.size() < height)
        lines << term::Line(width).pad(width).str();
    return lines;
}

QStringList Tui::renderOverlay(int height, int width) const
{
    QStringList lines;
    for(int i = 0; i < height; i++)
        lines << QString();

    if(overlay_ == Overlay::None)
        return lines;

    // The box is sized first, because the warnings are wrapped to fit it.
    const int box_width = qBound(46, width - 20, width - 4);
    const int wrap_width = box_width - 6;

    QStringList body;
    QString title = overlay_title_;

    if(overlay_ == Overlay::Help)
    {
        title = "Keys";
        body << "GLOBAL"
             << "  Tab / Shift-Tab    next / previous tab (also F1 F2 F3)"
             << "  o                  open or close the serial port"
             << "  c                  connection settings: port, baud, parity, timeout"
             << "  s                  search the bus for servos (press again to stop)"
             << "  n / N              select the next / previous servo"
             << "  t                  torque on or off, every detected servo"
             << "  z                  mouse reporting on/off (off lets you select text)"
             << "  ? / q              this help / quit"
             << ""
             << "DEBUG"
             << "  1..7               show or hide a plot series"
             << "  + -                zoom the time axis      < >   zoom the value axes"
             << "  h l                pan                     0     reset the view"
             << "  wheel / drag       same, with the mouse; shift+wheel zooms values"
             << "  arrows             goal -1/+1, -50/+50     Home/End  goal 0 / max"
             << "  g / a / U          goal in counts / angle / switch deg and rad"
             << "  m                  write mode: write, sync write, reg write"
             << "  T / x              torque this servo / send Action for a reg write"
             << "  v                  speed, acceleration and goal time"
             << "  w / e / E          sweep / step / auto debug settings"
             << "  r / R / X          record to file / settings / clear the counter"
             << "  u / d              upper and lower limit lines on the plot"
             << ""
             << "PROGRAMMING"
             << "  arrows PgUp PgDn   move through the register map"
             << "  Enter or w         write the selected register"
             << "  / S L M            find a register / save / load / set midpoint"
             << ""
             << "CALIBRATION"
             << "  k                  recheck the torque gate"
             << "  j / J / D          joint names / rename this row / toggle drive_mode"
             << "  H / R / X          set home / record the range / export JSON";
    }
    else if(overlay_ == Overlay::Prompt)
    {
        body << "";
        body << (prompt_fresh_ ? "  [" + prompt_buffer_ + "]" : "  " + prompt_buffer_ + "_");
        body << "";
        body << (prompt_fresh_
                     ? "  Type to replace, or use the arrows to edit. Enter accepts, Esc cancels."
                     : "  Enter to accept, Esc to cancel, Ctrl-U to clear.");
    }
    else if(overlay_ == Overlay::Confirm)
    {
        title = QString("%1 - confirmation %2 of %3")
                    .arg(overlay_title_).arg(confirm_index_ + 1).arg(confirm_steps_.size());
        for(const QString &paragraph : confirm_steps_.value(confirm_index_).split('\n'))
        {
            // Wrapped by hand: these are long, deliberate warnings, and they
            // have to stay readable rather than be cut off at the box edge.
            QString current;
            for(const QString &word : paragraph.split(' '))
            {
                if(!current.isEmpty() && current.size() + word.size() + 1 > wrap_width)
                {
                    body << "  " + current;
                    current.clear();
                }
                current += (current.isEmpty() ? QString() : QString(" ")) + word;
            }
            body << "  " + current;
        }
        body << "";
        body << "  y or Enter to continue, Esc to cancel.";
    }
    else if(overlay_ == Overlay::Menu)
    {
        body << "";
        for(const MenuItem &item : menu_items_)
            body << QString("  %1   %2").arg(item.key).arg(item.label);
        body << "";
        body << "  Esc to cancel.";
    }

    const int box_height = qMin(height, body.size() + 4);
    const int top = qMax(0, (height - box_height) / 2);
    const int left = qMax(0, (width - box_width) / 2);
    const int right_edge = left + box_width - 1;

    // Every row of the box ends on the same column, so the fills are measured
    // against what has actually been emitted rather than assumed.
    auto edge = [&](bool top_edge) {
        const QChar corner_l = term::unicode ? QChar(top_edge ? 0x256D : 0x2570) : QChar('+');
        const QChar corner_r = term::unicode ? QChar(top_edge ? 0x256E : 0x256F) : QChar('+');
        const QChar dash = term::unicode ? QChar(0x2500) : QChar('-');

        term::Line line(width);
        line.pad(left).back(COL_PANEL).color(COL_BORDER).text(QString(corner_l));
        if(top_edge && !title.isEmpty())
        {
            line.color(COL_ACCENT).bold().text(" " + title.left(box_width - 6) + " ").off();
            line.back(COL_PANEL).color(COL_BORDER);
        }
        line.fill(dash, qMax(0, right_edge - line.used()));
        line.pad(right_edge).text(QString(corner_r));
        return line.str();
    };

    auto content_row = [&](const QString &content) {
        const QChar side = term::unicode ? QChar(0x2502) : QChar('|');
        term::Line line(width);
        line.pad(left).back(COL_PANEL).color(COL_BORDER).text(QString(side));
        line.color(COL_TEXT).text(content.left(box_width - 2));
        line.pad(right_edge).color(COL_BORDER).text(QString(side));
        return line.str();
    };

    int row = top;
    lines[row++] = edge(true);
    for(int i = 0; i < body.size() && row < top + box_height - 1 && row < height; i++)
        lines[row++] = content_row(body[i]);
    while(row < top + box_height - 1 && row < height)
        lines[row++] = content_row(QString());
    if(row < height)
        lines[row] = edge(false);

    return lines;
}

QString Tui::renderHints(int width) const
{
    term::Line line(width);
    line.color(COL_FAINT);

    switch(tab_)
    {
        case Tab::Debug:
            line.text(" arrows goal  g goal  a angle  m mode  T torque  w sweep  e step  "
                      "r record  1-7 series  +/- zoom");
            break;
        case Tab::Programming:
            line.text(" arrows select  Enter write  / find  S save  L load  M set midpoint");
            break;
        case Tab::Calibration:
            line.text(" k recheck  j names  J rename  D drive_mode  H set home  R record range  "
                      "X export");
            break;
    }
    return line.pad(width).str();
}

void Tui::render()
{
    if(quitting_)
        return;

    const term::Size size = term::size();
    screen_.resize(size.rows, size.cols);

    const int width = size.cols;
    const int height = size.rows;

    if(height < 12 || width < 60)
    {
        screen_.set(0, term::Line(width).color(COL_ENGAGED)
                          .text("ServoBench needs at least 60x12. Resize the terminal.").str());
        for(int i = 1; i < height; i++)
            screen_.set(i, QString());
        screen_.flush();
        return;
    }

    // Two rows for the message: several of them are whole sentences, and a
    // truncated warning is worse than a tall footer.
    const int MESSAGE_ROWS = 2;
    const int body_height = height - 3 - MESSAGE_ROWS;
    const int body_width = width - PANEL_WIDTH - 1;

    const QStringList panel = renderLeftPanel(body_height);
    QStringList body;
    switch(tab_)
    {
        case Tab::Debug: body = renderDebug(body_height, body_width); break;
        case Tab::Programming: body = renderProgramming(body_height, body_width); break;
        case Tab::Calibration: body = renderCalibration(body_height, body_width); break;
    }

    QStringList rows;
    const QString divider = term::fg(COL_BORDER) + (term::unicode ? QString(QChar(0x2502))
                                                                  : QString("|"));
    for(int i = 0; i < body_height; i++)
        rows << panel.value(i) + divider + body.value(i);

    const QStringList overlay = renderOverlay(body_height, width);
    for(int i = 0; i < body_height && i < overlay.size(); i++)
    {
        if(!overlay[i].isNull())
            rows[i] = overlay[i];
    }

    screen_.set(0, renderTitle(width));
    screen_.set(1, renderTabs(width));
    for(int i = 0; i < body_height; i++)
        screen_.set(2 + i, rows[i]);

    QStringList message_lines;
    QString rest = message_;
    while(!rest.isEmpty() && message_lines.size() < MESSAGE_ROWS)
    {
        if(rest.size() <= width - 2)
        {
            message_lines << rest;
            break;
        }
        // Break on a space where there is one, so a sentence does not split
        // mid-word.
        int cut = rest.lastIndexOf(' ', width - 2);
        if(cut <= 0)
            cut = width - 2;
        message_lines << rest.left(cut);
        rest = rest.mid(cut).trimmed();
    }

    for(int i = 0; i < MESSAGE_ROWS; i++)
    {
        term::Line message(width);
        message.color(message_warning_ ? COL_ALARM : COL_DIM)
               .text(" " + message_lines.value(i));
        screen_.set(height - 1 - MESSAGE_ROWS + i, message.pad(width).str());
    }
    screen_.set(height - 1, renderHints(width));

    screen_.flush();
}

}
