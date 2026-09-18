#ifndef CLI_SESSION_H
#define CLI_SESSION_H

#include <QCoreApplication>
#include <QEventLoop>
#include <QFile>
#include <QPointer>
#include <QRegExp>
#include <QSerialPort>
#include <QString>
#include <QStringList>
#include <QThread>
#include <QVector>

#include <functional>
#include <memory>
#include <optional>

#include "servo/servo_bus.h"

namespace cli
{

using feetech_servo::MemoryConfig;
using feetech_servo::ModelSeries;
using feetech_servo::MidpointResult;
using feetech_servo::RegisterWriteResult;
using feetech_servo::ServoStatus;

struct ConnectionOptions
{
    QString port;                                           // empty: pick one
    int baud = 1000000;
    QSerialPort::Parity parity = QSerialPort::NoParity;
    int timeout = 50;

    // Set to drive a bus on another machine instead of a local serial port.
    // The port, baud, parity and timeout above then describe the serial port
    // on that machine, which is chosen and opened exactly as a local one is.
    QString host;
    quint16 net_port = 0;
    QString token;

    bool isRemote() const { return !host.isEmpty(); }
};

using feetech_servo::PortInfo;

// Every serial port on this machine worth offering. For the ports on the far
// end of a network link, ask the bus instead: IServoBus::listPorts.
QVector<PortInfo> availablePorts();

struct ServoEntry
{
    int id = -1;
    QString model;
    ModelSeries series = ModelSeries::STS;
};

// Encoder counts and joint angles, measured from the calibrated midpoint.
double countsToRadians(ModelSeries series, int count);
double countsToDegrees(ModelSeries series, int count);
int angleToCounts(ModelSeries series, double angle, bool is_radians);

// Splits text on a separator and hands back the trimmed, non-empty pieces.
//
// QString::SkipEmptyParts became Qt::SkipEmptyParts in Qt 5.14, and neither
// spelling compiles against every Qt 5 this has to build on -- the Jetson's
// JetPack 5 image is still on 5.12 -- so the filtering happens here instead of
// in a split flag.
QStringList splitFields(const QString &text, const QRegExp &separator);

// A register by name or by address. Names match case-insensitively and ignore
// runs of whitespace and underscores, so "goal_position" and "Goal Position"
// both find address 42. An unambiguous prefix is accepted too.
const MemoryConfig *findRegister(ModelSeries series, const QString &token, QString *error);

enum class WriteMode
{
    Write,
    SyncWrite,
    RegWrite
};

// Owns the bus and the servo table, and is the only thing in the terminal
// tool that talks to a servo.
//
// In threaded mode the bus runs on its own thread exactly as it does in the
// GUI, and call() waits for a result while the caller's event loop keeps
// running -- so a full-screen refresh and a key press are still serviced
// during a slow transaction. The one-shot commands have nothing to keep alive,
// so they run the bus on the calling thread and call() is a direct call.
class Session : public QObject
{
    Q_OBJECT

public:
    // remote picks which bus is built: one on this machine's serial port, or
    // one reached over TCP. It cannot be changed afterwards, so it is settled
    // here rather than at open() -- by then the bus is already on its thread.
    explicit Session(bool threaded, bool remote = false, QObject *parent = nullptr);
    ~Session() override;

    // Reaches the machine holding the bus, without opening a serial port on it
    // yet. A no-op on a local session, where the bus is already here. Needed on
    // its own only by callers that want the port list before choosing one.
    bool reach(const ConnectionOptions &options, QString *error);

    // reach(), then open the serial port. Fills error and returns false at
    // whichever step failed.
    bool open(const ConnectionOptions &options, QString *error);

    // Serial ports on whichever machine holds the bus. Needs the transport to
    // be up for a remote session, so it is only meaningful after open().
    QVector<PortInfo> ports();

    bool isRemote() const { return remote_; }
    int latencyMs() const;
    void close();
    bool isOpen() const { return open_; }
    const ConnectionOptions &options() const { return options_; }

    // Called with true when a transaction starts and false when it ends, so
    // the full-screen tool can stop reading keys for the duration -- a
    // multi-step EPROM write must not be interrupted half way through.
    void setBusyHook(std::function<void(bool)> hook) { busy_hook_ = std::move(hook); }
    bool busy() const { return busy_; }

    // The bus itself, for the few callers that want to post something it has
    // no wrapper for. Anything it is handed still has to run on the bus, so it
    // goes through call() or post() like everything else.
    feetech_servo::IServoBus *bus() { return bus_; }

    // --- servo table ---

    const QVector<ServoEntry> &servos() const { return servos_; }
    void setServos(const QVector<ServoEntry> &servos);
    void addServo(const ServoEntry &entry);
    void clearServos();
    QVector<int> ids() const;
    bool knows(int id) const;
    ModelSeries seriesFor(int id) const;
    QString modelFor(int id) const;
    int countsPerRev(int id) const;

    // --- transactions ---

    // Ping, and on an answer read the model number. A servo that does not
    // answer comes back with id == -1.
    ServoEntry probe(int id);

    // Pings every ID in the range. progress(id) runs before each ping and
    // aborts the scan by returning false.
    QVector<ServoEntry> scan(int from, int to, const std::function<bool(int)> &progress);

    // Reads a register, and says whether the servo answered at all.
    //
    // ServoBus::readRegister cannot: it reports a silent servo and a value of
    // -1 the same way, and a sign-magnitude register such as Position Offset
    // Value legitimately holds negative values -- a calibrated arm usually
    // does. So the raw read is decoded here, where a failed read is still
    // distinguishable from a real answer.
    std::optional<int> readRegister(int id, const MemoryConfig &config);
    RegisterWriteResult writeRegister(int id, const MemoryConfig &config, int value);
    std::optional<int> readPosition(int id);
    ServoStatus readStatus(int id);
    int readByte(int id, uint8_t address);

    // Torque state and fault byte for a set of servos, in one visit to the bus.
    struct ServoFlags
    {
        int torque = -1;
        int fault = -1;
    };
    QVector<ServoFlags> readFlags(const QVector<int> &ids);

    // Writes Torque Enable to every servo first and only then reads them all
    // back, so an arm releases together rather than one joint at a time.
    // Returns the IDs that did not end up in the requested state.
    QStringList setTorque(const QVector<int> &ids, bool on);

    MidpointResult setMidpoint(int id, const MemoryConfig &offset);

    void commandPosition(int id, WriteMode mode, int pos, int time, int speed, int acc);
    void syncCommandPosition(const QVector<int> &ids, int pos, int time, int speed, int acc);
    void regWriteAction(int id);

    // One row of a Set Home sweep, as the GUI reports it.
    struct HomeResult
    {
        int id = -1;
        bool written = false;
        int homing_offset = 0;
        int half_turn = 0;
        int corrected = -1;
    };
    QVector<HomeResult> setHome(const QVector<int> &ids);

    // Runs fn on the bus and waits for its result.
    template<typename F>
    auto call(F &&fn) -> decltype(fn())
    {
        using Result = decltype(fn());

        if(bus_thread_ == nullptr)
        {
            BusyMark mark(this);
            return fn();
        }

        // Heap-allocated so the result outlives fn no matter which thread
        // finishes with it last.
        struct Shared { Result value{}; bool done = false; };
        auto shared = std::make_shared<Shared>();

        QEventLoop loop;
        QPointer<QEventLoop> loop_ptr(&loop);

        QMetaObject::invokeMethod(bus_, [shared, loop_ptr, f = std::forward<F>(fn)]() mutable {
            Result produced = f();
            QMetaObject::invokeMethod(qApp, [shared, loop_ptr, produced]() {
                shared->value = produced;
                shared->done = true;
                if(loop_ptr)
                    loop_ptr->quit();
            }, Qt::QueuedConnection);
        }, Qt::QueuedConnection);

        BusyMark mark(this);
        loop.exec(QEventLoop::ExcludeUserInputEvents);

        // exec() returns immediately once the application is quitting, so the
        // reply may still be in flight. The bus thread is alive until the
        // destructor stops it, so keep pumping rather than handing back a
        // default-built result that the caller would read as real data.
        while(!shared->done)
        {
            QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents, 5);
            if(!shared->done)
                QThread::msleep(1);
        }

        return shared->value;
    }

    // Queues fn on the bus and returns straight away, for commands whose
    // result nothing waits on.
    template<typename F>
    void post(F &&fn)
    {
        if(bus_thread_ == nullptr)
        {
            BusyMark mark(this);
            fn();
            // Nothing is waiting for a reply here, and a one-shot command is
            // about to exit without ever running an event loop, so the port
            // has to be drained by hand or the packet is simply lost.
            bus_->flushWrites();
            return;
        }
        QMetaObject::invokeMethod(bus_, std::forward<F>(fn), Qt::QueuedConnection);
    }

private:
    // Holds the busy flag for one transaction, and tells the hook about it.
    // Nested calls only report the outermost one.
    struct BusyMark
    {
        explicit BusyMark(Session *s)
            : session(s)
            , was_busy(s->busy_)
        {
            session->busy_ = true;
            if(!was_busy && session->busy_hook_)
                session->busy_hook_(true);
        }

        ~BusyMark()
        {
            session->busy_ = was_busy;
            if(!was_busy && session->busy_hook_)
                session->busy_hook_(false);
        }

        Session *session;
        bool was_busy;
    };

    feetech_servo::IServoBus *bus_;
    QThread *bus_thread_ = nullptr;
    ConnectionOptions options_;
    QVector<ServoEntry> servos_;
    bool open_ = false;
    bool remote_ = false;
    bool busy_ = false;
    std::function<void(bool)> busy_hook_;
};

// The GUI's Data analysis recorder: same header, same row format, same
// batching, so a log from either tool loads the same way.
class Recorder
{
public:
    bool start(const QString &path, int interval_s, QString *error);
    void append(const ServoStatus &status);
    void stop();
    // Clears the row counter between runs, as the window's Empty button does.
    void reset();

    bool active() const { return active_; }
    quint64 rows() const { return rows_; }
    QString path() const { return path_; }

private:
    bool flushSection();

    bool active_ = false;
    QString path_;
    QString pending_;
    quint64 rows_ = 0;
    int interval_s_ = 1;
};

// Expands a leading ~ the way the GUI's record file field does.
QString expandHome(const QString &path);

}

#endif
