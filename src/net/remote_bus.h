#ifndef NET_REMOTE_BUS_H
#define NET_REMOTE_BUS_H

#include <QAtomicInt>
#include <QElapsedTimer>
#include <QJsonObject>
#include <QString>
#include <QTcpSocket>
#include <QTimer>

#include "net/protocol.h"
#include "servo/servo_bus.h"

namespace servobench_net
{

// A bus whose servos are on another machine's serial port, reached over TCP.
//
// It is a drop-in for ServoBus: same interface, same thread rule -- it lives on
// the bus thread and every transaction is called from there -- so the window
// and the terminal tool drive it without knowing the servos are not local.
//
// Each transaction is one request and one reply, so the caller blocks for a
// round trip rather than for a serial exchange. Events pushed by the server
// (telemetry, scan results, the port opening or closing) arrive whenever the
// bus thread is back in its event loop, and are also picked up mid-request,
// since a reply may have events queued in front of it.
class RemoteServoBus : public feetech_servo::IServoBus
{
    Q_OBJECT

public:
    explicit RemoteServoBus(QObject *parent = nullptr);
    ~RemoteServoBus() override;

    bool isRemote() const override { return true; }

    bool connectTransport(const QString &host, quint16 port,
                          const QString &token, QString *error) override;
    void disconnectTransport() override;
    // Both are read from the UI thread while the bus thread is writing them,
    // so both are atomic. Neither is worth a lock: one is a light that says
    // whether the link is up, the other a number next to it.
    bool transportAlive() const override { return alive_.loadRelaxed() != 0; }
    int latencyMs() const override { return latency_ms_.loadRelaxed(); }

    // What the server said about itself during the handshake. Empty, or false,
    // until a handshake has succeeded.
    QString serverDescription() const { return server_description_; }

    // The serial port the server is configured to hold, and whether it is open
    // right now. A client that was not told which port to use takes this one:
    // on a robot the device is settled when the server is started, and asking
    // the user to name it again on every connection would only be a chance to
    // name the wrong one.
    QString serverDevice() const { return server_device_; }
    bool serverPortOpen() const { return server_port_open_; }

    QVector<feetech_servo::PortInfo> listPorts() override;

    bool open(const QString &port_name, int baud, QSerialPort::Parity parity, int timeout) override;
    void close() override;

    int readByte(uint8_t id, feetech_servo::ModelSeries series, uint8_t address) override;
    int readWord(uint8_t id, feetech_servo::ModelSeries series, uint8_t address) override;
    void writeByte(uint8_t id, feetech_servo::ModelSeries series, uint8_t address, uint8_t value) override;

    int readRegister(uint8_t id, feetech_servo::ModelSeries series,
                     const feetech_servo::MemoryConfig &config) override;
    std::optional<int> readPosition(uint8_t id, feetech_servo::ModelSeries series) override;
    feetech_servo::RegisterWriteResult writeRegister(uint8_t id, feetech_servo::ModelSeries series,
                                                     const feetech_servo::MemoryConfig &config,
                                                     int value) override;
    feetech_servo::MidpointResult setMidpoint(uint8_t id, feetech_servo::ModelSeries series,
                                              const feetech_servo::MemoryConfig &offset) override;

    int readModelNumber(uint8_t id) override;
    int ping(uint8_t id) override;

    void writePos(uint8_t id, feetech_servo::ModelSeries series, int pos, int time, int speed, int acc) override;
    void regWritePos(uint8_t id, feetech_servo::ModelSeries series, int pos, int time, int speed, int acc) override;
    void syncWritePos(const std::vector<uint8_t> &ids, feetech_servo::ModelSeries series,
                      int pos, int time, int speed, int acc) override;
    void regWriteAction(uint8_t id) override;
    void enableTorque(uint8_t id, feetech_servo::ModelSeries series, bool on) override;
    void flushWrites() override;

    feetech_servo::ServoStatus readStatus(uint8_t id, feetech_servo::ModelSeries series) override;
    void pollStatus(int id, int series) override;
    void invalidateModeCaches() override;

    // Asks the server to sweep, and returns without waiting. The sweep's
    // progress arrives as events, which keeps the bus thread in its event loop
    // for the duration -- so the heartbeat keeps going out and abortScan can
    // still be delivered.
    void startScan(int from, int to) override;
    void abortScan() override;

private slots:
    void onReadyRead();
    void onSocketClosed();
    void onHeartbeat();

private:
    // Sends a request and waits for its reply. Returns false and leaves the
    // link dead on timeout or disconnection -- which is not the same thing as
    // a servo failing to answer, and must never be reported as one.
    bool request(const QString &op, const QJsonObject &args, QJsonObject *result);

    // Sends a request whose reply nothing needs. Still waits for it: the
    // server answers every request, and leaving replies unread would put them
    // in front of the next one.
    void command(const QString &op, const QJsonObject &args);

    // Reads frames until one carries seq, handing every event seen on the way
    // to dispatchEvent.
    bool awaitReply(int seq, QJsonObject *result, QString *error);

    void dispatchEvent(const QJsonObject &message);
    void fail(const QString &reason);

    // How long to wait for a reply. Scaled off the serial timeout, because a
    // telemetry sample is eight serial transactions and each of them may wait
    // that long for a servo that is not there.
    int replyTimeoutMs() const;

    QTcpSocket *socket_;
    QTimer *heartbeat_;
    servobench_net::FrameReader reader_;

    QAtomicInt alive_{0};
    // Set while a request owns the socket, so the readyRead slot stands down
    // rather than swallowing the reply that request is waiting for.
    bool in_request_ = false;
    // Set while connectTransport is still handshaking. A failure there is
    // reported by connectTransport itself, so it must not also surface as a
    // link that dropped.
    bool in_handshake_ = false;
    // Why the link last failed, for connectTransport to report.
    QString last_error_;
    int seq_ = 0;
    QAtomicInt latency_ms_{0};
    int serial_timeout_ms_ = 50;
    QString server_description_;
    QString server_device_;
    bool server_port_open_ = false;
};

}

#endif
