#ifndef SERVO_BUS_H
#define SERVO_BUS_H

#include <QAtomicInt>
#include <QElapsedTimer>
#include <QObject>
#include <QSerialPort>
#include <QString>
#include <QThread>
#include <QVector>
#include <optional>
#include <vector>

#include "servo_driver.h"
#include "port_list.h"

namespace feetech_servo
{

// Monotonic clock every telemetry sample is stamped from. Process-wide, so
// two samples are comparable no matter which bus produced them -- and, once a
// sample has crossed a network, still comparable to each other on the machine
// that reads them.
QElapsedTimer& busClock();

// One telemetry sample. ok is false when the servo did not answer at all.
struct ServoStatus
{
    int id = -1;
    int pos = 0;
    int torque = 0;
    int speed = 0;
    int current = 0;
    int temp = 0;
    int voltage = 0;
    int move = 0;
    int goal = 0;
    bool ok = false;
    // When the sample was taken, on the clock of whichever machine owns the
    // serial port. Over a network link the reader's own clock would measure
    // the link's jitter rather than the servo's motion, so the plot uses this
    // instead. -1 means nothing stamped it.
    qint64 t_ms = -1;
};

struct RegisterWriteResult
{
    bool ok = false;
    // Writing the ID register moves the servo to a new address part way
    // through the write, so the caller has to be told where it ended up.
    int effective_id = -1;
};

struct MidpointResult
{
    bool acked = false;
    int before = -1;
    int after = -1;
};

// Every servo transaction, behind one interface, so the window and the
// terminal tool can drive a bus that is either on this machine's serial port
// (ServoBus) or on a robot at the other end of a socket (RemoteServoBus)
// without knowing which.
//
// Implementations live on their own thread. The UI never calls these
// directly: it either posts a command or asks for a result through
// MainWindow::runOnBus, which keeps the window repainting while it waits.
//
// Because there is exactly one bus thread, its event queue is also the bus
// lock -- two transactions can never interleave on the wire.
class IServoBus : public QObject
{
    Q_OBJECT

public:
    explicit IServoBus(QObject *parent = nullptr);
    ~IServoBus() override;

    // --- transport ---
    //
    // A serial bus is reachable the moment the process starts, so the local
    // implementation treats all of this as already done. Only the networked
    // one has a connection to make, lose, or measure.

    // True when this bus reaches its servos over a network rather than a
    // serial port on this machine. The window uses it to decide whether the
    // link fields are worth showing, and nothing else.
    virtual bool isRemote() const { return false; }

    // Reaches the machine that owns the serial port. Must be called, and must
    // succeed, before open(). Returns false and fills error on failure.
    virtual bool connectTransport(const QString &host, quint16 port,
                                  const QString &token, QString *error);
    virtual void disconnectTransport() {}

    // False once the link has dropped. A local bus is always reachable, so a
    // servo that stops answering there is a servo fault; on a remote bus the
    // two have to stay distinguishable, or a dropped connection reads as every
    // servo failing at once.
    virtual bool transportAlive() const { return true; }

    // Round trip to the machine owning the port, in milliseconds, as last
    // measured. 0 on a local bus.
    virtual int latencyMs() const { return 0; }

    // --- the bus itself ---

    // Serial ports on whichever machine owns the bus.
    virtual QVector<PortInfo> listPorts() = 0;

    virtual bool open(const QString &port_name, int baud, QSerialPort::Parity parity, int timeout) = 0;
    virtual void close() = 0;

    virtual int readByte(uint8_t id, ModelSeries series, uint8_t address) = 0;
    virtual int readWord(uint8_t id, ModelSeries series, uint8_t address) = 0;
    virtual void writeByte(uint8_t id, ModelSeries series, uint8_t address, uint8_t value) = 0;

    // Reads a register through its MemoryConfig, decoding sign-magnitude where
    // the register's min/max range says it uses it. Returns -1 if the servo did
    // not answer.
    virtual int readRegister(uint8_t id, ModelSeries series, const MemoryConfig &config) = 0;

    // Present Position, always decoded as sign-magnitude.
    //
    // It cannot go through readRegister: Present Position is read-only, so the
    // register table carries -1/-1 for its min and max, and isSignMagnitude
    // keys off a range that spans zero. A servo sitting just below its zero
    // reports 0x8003, which has to read as -3 and not as 32771 -- calibration
    // derives the homing offset from this and writes it to EPROM.
    //
    // Returns nullopt if the servo did not answer.
    virtual std::optional<int> readPosition(uint8_t id, ModelSeries series) = 0;

    // Unlock EPROM if needed, write, relock, then verify by reading back.
    virtual RegisterWriteResult writeRegister(uint8_t id, ModelSeries series,
                                              const MemoryConfig &config, int value) = 0;

    // Store the servo's current position as its midpoint.
    virtual MidpointResult setMidpoint(uint8_t id, ModelSeries series, const MemoryConfig &offset) = 0;

    virtual int readModelNumber(uint8_t id) = 0;
    virtual int ping(uint8_t id) = 0;

    virtual void writePos(uint8_t id, ModelSeries series, int pos, int time, int speed, int acc) = 0;
    virtual void regWritePos(uint8_t id, ModelSeries series, int pos, int time, int speed, int acc) = 0;
    virtual void syncWritePos(const std::vector<uint8_t> &ids, ModelSeries series,
                              int pos, int time, int speed, int acc) = 0;
    virtual void regWriteAction(uint8_t id) = 0;
    virtual void enableTorque(uint8_t id, ModelSeries series, bool on) = 0;

    // Pushes anything still sitting in the port's write buffer onto the wire.
    // A command that expects a reply flushes on its own by waiting for one;
    // the unacknowledged ones -- sync write, and Action addressed to the
    // broadcast ID -- do not. Under an event loop that does not matter, since
    // the loop drains the port, so this is only needed by a caller that is
    // about to stop running one.
    virtual void flushWrites() = 0;

    virtual ServoStatus readStatus(uint8_t id, ModelSeries series) = 0;

    // Reads a full telemetry sample and answers with statusReady. This is the
    // one operation the UI does not wait for: it asks for the next sample only
    // after the previous one lands, so a slow bus paces itself instead of
    // piling up requests.
    virtual void pollStatus(int id, int series) = 0;

    virtual void invalidateModeCaches() = 0;

    // --- ID scan ---

    // Pings every ID from..to, answering with scanProgress before each and
    // scanFound for each servo that replies, then scanFinished. It runs as one
    // operation rather than as a ping per call because a networked bus would
    // otherwise pay a round trip for each of the 254 addresses.
    //
    // Like every other transaction this runs on the bus thread and holds it for
    // the duration, so callers must post it rather than wait on it.
    virtual void startScan(int from, int to) = 0;

    // Stops a scan early. Thread-safe and non-blocking by design -- a local
    // bus thread is inside the scan loop at this point and could not service a
    // queued call, so this is the one bus method that may be called directly
    // from the UI thread.
    virtual void abortScan() { scan_abort_.storeRelaxed(1); }

signals:
    void openedChanged(bool is_open);
    void statusReady(const feetech_servo::ServoStatus &status);

    // The link to a remote bus dropped. Never emitted by a local bus.
    void transportLost(const QString &reason);

    void scanProgress(int id);
    void scanFound(int id, int model_number);
    // completed is false when abortScan cut the sweep short.
    void scanFinished(bool completed);

protected:
    // Catches a caller that reached a transaction directly from the UI thread
    // instead of going through runOnBus or postToBus. Compiled out of release
    // builds; in a debug build it turns a silent data race into an assertion
    // at the exact call that broke the rule.
    void assertOnBusThread() const
    {
        Q_ASSERT_X(QThread::currentThread() == thread(), "IServoBus",
                   "servo transaction called from outside the bus thread");
    }

    // Cleared at the top of a scan, set by abortScan from any thread.
    QAtomicInt scan_abort_{0};
};

// The bus on this machine's own serial port.
class ServoBus : public IServoBus
{
    Q_OBJECT

public:
    explicit ServoBus(QObject *parent = nullptr);
    ~ServoBus() override;

    QVector<PortInfo> listPorts() override { return localPorts(); }

    bool open(const QString &port_name, int baud, QSerialPort::Parity parity, int timeout) override;
    void close() override;

    int readByte(uint8_t id, ModelSeries series, uint8_t address) override;
    int readWord(uint8_t id, ModelSeries series, uint8_t address) override;
    void writeByte(uint8_t id, ModelSeries series, uint8_t address, uint8_t value) override;

    int readRegister(uint8_t id, ModelSeries series, const MemoryConfig &config) override;
    std::optional<int> readPosition(uint8_t id, ModelSeries series) override;
    RegisterWriteResult writeRegister(uint8_t id, ModelSeries series,
                                      const MemoryConfig &config, int value) override;
    MidpointResult setMidpoint(uint8_t id, ModelSeries series, const MemoryConfig &offset) override;

    int readModelNumber(uint8_t id) override;
    int ping(uint8_t id) override;

    void writePos(uint8_t id, ModelSeries series, int pos, int time, int speed, int acc) override;
    void regWritePos(uint8_t id, ModelSeries series, int pos, int time, int speed, int acc) override;
    void syncWritePos(const std::vector<uint8_t> &ids, ModelSeries series,
                      int pos, int time, int speed, int acc) override;
    void regWriteAction(uint8_t id) override;
    void enableTorque(uint8_t id, ModelSeries series, bool on) override;
    void flushWrites() override;

    ServoStatus readStatus(uint8_t id, ModelSeries series) override;
    void pollStatus(int id, int series) override;
    void invalidateModeCaches() override;

    void startScan(int from, int to) override;

    bool isOpen() const { return serial_->isOpen(); }

private:
    // Selects the driver for a series and puts the wire endianness that series
    // needs on the port. Every entry point goes through this, so a bus with
    // more than one series on it stays correct.
    ServoDriver* use(ModelSeries series);

    QSerialPort *serial_;
    SCSerial *scserial_;
    StsDriver *sts_driver_;
    ScsDriver *scs_driver_;
};

}

Q_DECLARE_METATYPE(feetech_servo::ServoStatus)

#endif
