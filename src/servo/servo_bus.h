#ifndef SERVO_BUS_H
#define SERVO_BUS_H

#include <QObject>
#include <QSerialPort>
#include <QString>
#include <QThread>
#include <optional>
#include <vector>

#include "servo_driver.h"

namespace feetech_servo
{

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

// Owns the serial port and every servo transaction, and lives on its own
// thread. The UI never touches the port: it either posts a command or asks for
// a result through MainWindow::runOnBus, which keeps the window repainting
// while it waits.
//
// Because there is exactly one bus thread, its event queue is also the bus
// lock -- two transactions can never interleave on the wire.
class ServoBus : public QObject
{
    Q_OBJECT

public:
    explicit ServoBus(QObject *parent = nullptr);
    ~ServoBus() override;

    // Everything below runs a servo transaction, so it must be called on the
    // bus thread -- from a functor handed to runOnBus, never called directly
    // from the UI thread.

    bool open(const QString &port_name, int baud, QSerialPort::Parity parity, int timeout);
    void close();

    int readByte(uint8_t id, ModelSeries series, uint8_t address);
    int readWord(uint8_t id, ModelSeries series, uint8_t address);
    void writeByte(uint8_t id, ModelSeries series, uint8_t address, uint8_t value);

    // Reads a register through its MemoryConfig, decoding sign-magnitude where
    // the register's min/max range says it uses it. Returns -1 if the servo did
    // not answer.
    int readRegister(uint8_t id, ModelSeries series, const MemoryConfig &config);

    // Present Position, always decoded as sign-magnitude.
    //
    // It cannot go through readRegister: Present Position is read-only, so the
    // register table carries -1/-1 for its min and max, and isSignMagnitude
    // keys off a range that spans zero. A servo sitting just below its zero
    // reports 0x8003, which has to read as -3 and not as 32771 -- calibration
    // derives the homing offset from this and writes it to EPROM.
    //
    // Returns nullopt if the servo did not answer.
    std::optional<int> readPosition(uint8_t id, ModelSeries series);

    // Unlock EPROM if needed, write, relock, then verify by reading back.
    RegisterWriteResult writeRegister(uint8_t id, ModelSeries series,
                                      const MemoryConfig &config, int value);

    // Store the servo's current position as its midpoint.
    MidpointResult setMidpoint(uint8_t id, ModelSeries series, const MemoryConfig &offset);

    int readModelNumber(uint8_t id);
    int ping(uint8_t id);

    void writePos(uint8_t id, ModelSeries series, int pos, int time, int speed, int acc);
    void regWritePos(uint8_t id, ModelSeries series, int pos, int time, int speed, int acc);
    void syncWritePos(const std::vector<uint8_t> &ids, ModelSeries series,
                      int pos, int time, int speed, int acc);
    void regWriteAction(uint8_t id);
    void enableTorque(uint8_t id, ModelSeries series, bool on);

    ServoStatus readStatus(uint8_t id, ModelSeries series);

    // Reads a full telemetry sample and answers with statusReady. This is the
    // one operation the UI does not wait for: it asks for the next sample only
    // after the previous one lands, so a slow bus paces itself instead of
    // piling up requests.
    void pollStatus(int id, int series);

    void invalidateModeCaches();

signals:
    void openedChanged(bool is_open);
    void statusReady(const feetech_servo::ServoStatus &status);

private:
    // Selects the driver for a series and puts the wire endianness that series
    // needs on the port. Every entry point goes through this, so a bus with
    // more than one series on it stays correct.
    ServoDriver* use(ModelSeries series);

    // Catches a caller that reached a transaction directly from the UI thread
    // instead of going through runOnBus or postToBus. Compiled out of release
    // builds; in a debug build it turns a silent data race into an assertion
    // at the exact call that broke the rule.
    void assertOnBusThread() const
    {
        Q_ASSERT_X(QThread::currentThread() == thread(), "ServoBus",
                   "servo transaction called from outside the bus thread");
    }

    QSerialPort *serial_;
    SCSerial *scserial_;
    StsDriver *sts_driver_;
    ScsDriver *scs_driver_;
};

}

Q_DECLARE_METATYPE(feetech_servo::ServoStatus)

#endif
