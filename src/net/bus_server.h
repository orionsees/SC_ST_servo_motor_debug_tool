#ifndef NET_BUS_SERVER_H
#define NET_BUS_SERVER_H

#include <QCoreApplication>
#include <QJsonObject>
#include <QPointer>
#include <QSerialPort>
#include <QSet>
#include <QString>
#include <QTcpServer>
#include <QTcpSocket>
#include <QThread>
#include <QTimer>

#include "net/protocol.h"
#include "servo/servo_bus.h"

namespace servobench_net
{

// What to do with the servos when the client goes away.
enum class DisconnectPolicy
{
    // Change nothing. The last command stands and torque stays as it was, so a
    // joint holding a load keeps holding it -- and a wheel-mode servo keeps
    // turning.
    Hold,
    // Command every servo this session touched to the position it is in.
    // Torque stays on, so an arm still holds itself up, and a servo that was
    // moving stops where it is. This puts a wheel-mode servo into position
    // mode, which is exactly what stopping one requires.
    Stop,
    // Torque off. An arm under gravity will fall, so this is only right for a
    // machine where being limp is the safe state.
    Release,
};

DisconnectPolicy disconnectPolicyFromName(const QString &name, bool *ok);
QString disconnectPolicyName(DisconnectPolicy policy);

struct ServerConfig
{
    // Loopback unless asked otherwise: a robot's motors should not become
    // reachable from the whole network because someone left a default alone.
    QString bind = QStringLiteral("127.0.0.1");
    quint16 port = DEFAULT_PORT;
    // Empty means no token is required, which is only reasonable on loopback.
    QString token;
    DisconnectPolicy on_disconnect = DisconnectPolicy::Stop;

    // Opened at startup when a device is given, so a client can connect and
    // start work without choosing a port first. Otherwise the client opens one
    // from the list the server offers.
    QString device;
    int baud = 1000000;
    QSerialPort::Parity parity = QSerialPort::NoParity;
    int timeout = 50;

    bool verbose = false;
};

// Serves one ServoBench client, and holds the serial port it drives.
//
// Exactly one client at a time: two would interleave packets on a bus whose
// protocol has no way to tell whose reply is whose, so a second connection is
// turned away with a reason rather than quietly corrupting the first.
//
// Socket work happens on the thread that owns this object and servo
// transactions on the bus thread, the same split the window uses. Nothing here
// blocks on the bus, so a scan that takes seconds still leaves the server able
// to hear the client asking it to stop.
class BusServer : public QObject
{
    Q_OBJECT

public:
    explicit BusServer(const ServerConfig &config, QObject *parent = nullptr);
    ~BusServer() override;

    bool start(QString *error);

private slots:
    void onNewConnection();
    void onClientReadyRead();
    void onClientDisconnected();
    void onDeadman();

    void onBusOpenedChanged(bool is_open);
    void onScanProgress(int id);
    void onScanFound(int id, int model_number);
    void onScanFinished(bool completed);

private:
    void handleMessage(const QJsonObject &message);
    void handleRequest(int seq, const QString &op, const QJsonObject &args);
    bool handleHello(int seq, const QJsonObject &args);

    void sendReply(int seq, const QJsonObject &result);
    void sendError(int seq, const QString &message);
    void sendEvent(const QString &name, const QJsonObject &data = {});
    void sendFrame(const QJsonObject &message);
    void sendTo(QTcpSocket *socket, const QJsonObject &message);

    // Closes a socket without cutting off whatever was just written to it.
    void closeGently(QTcpSocket *socket);

    void adoptClient(QTcpSocket *socket);
    void dropClient(const QString &reason);
    void applyDisconnectPolicy();

    // Remembers a servo this client has addressed, so the disconnect policy
    // knows what it is responsible for. A policy that swept every possible ID
    // would spend 254 timeouts doing it, at the exact moment the link is
    // already in trouble.
    void noteTouched(const QJsonObject &args);

    void log(const QString &line) const;

    // Runs fn on the bus thread and answers seq with whatever it returns. The
    // reply is dropped if the client it was computed for has since gone, so a
    // late answer never reaches whoever connected next.
    template<typename F>
    void onBus(int seq, F &&fn)
    {
        QPointer<BusServer> self(this);
        const quint64 session = session_;

        QMetaObject::invokeMethod(bus_, [self, seq, session, f = std::forward<F>(fn)]() mutable {
            const QJsonObject result = f();
            QMetaObject::invokeMethod(qApp, [self, seq, session, result] {
                if(!self.isNull() && self->session_ == session && self->client_ != nullptr)
                    self->sendReply(seq, result);
            }, Qt::QueuedConnection);
        }, Qt::QueuedConnection);
    }

    ServerConfig config_;
    QTcpServer *server_;
    QTcpSocket *client_ = nullptr;
    feetech_servo::ServoBus *bus_;
    QThread *bus_thread_;
    QTimer *deadman_;
    FrameReader reader_;

    // Bumped on every client change, so work still in flight for the previous
    // one can be recognised and discarded.
    quint64 session_ = 0;
    // The port the bus last tried to open, and whether it is open now. Tracked
    // from the bus's own signal rather than from what a client asked for, so
    // what the log and the handshake report is what is actually true.
    QString device_;
    bool port_open_ = false;
    bool greeted_ = false;
    bool scanning_ = false;
    // (id, series) pairs this client has addressed.
    QSet<QPair<int, int>> touched_;
};

}

#endif
