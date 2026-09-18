#include "net/remote_bus.h"

#include <QCoreApplication>
#include <QJsonArray>

namespace servobench_net
{

using feetech_servo::MemoryConfig;
using feetech_servo::MidpointResult;
using feetech_servo::ModelSeries;
using feetech_servo::PortInfo;
using feetech_servo::RegisterWriteResult;
using feetech_servo::ServoStatus;

namespace
{
// How long to wait for the TCP connection itself. Short, because this runs
// while the user is looking at the Open button.
constexpr int CONNECT_TIMEOUT_MS = 4000;
// Idle keepalive. The server drops a client it has not heard from in three of
// these and applies its disconnect policy to the servos.
constexpr int HEARTBEAT_MS = 2000;
// How long each blocking read waits before looking at the clock again.
constexpr int READ_SLICE_MS = 50;
}

RemoteServoBus::RemoteServoBus(QObject *parent)
    : IServoBus(parent)
{
    qRegisterMetaType<feetech_servo::ServoStatus>();

    // Both are children, so moving this object to the bus thread takes them
    // with it and every socket call still happens on that one thread.
    socket_ = new QTcpSocket(this);
    heartbeat_ = new QTimer(this);
    heartbeat_->setInterval(HEARTBEAT_MS);

    connect(socket_, &QTcpSocket::readyRead, this, &RemoteServoBus::onReadyRead);
    connect(socket_, &QTcpSocket::stateChanged, this, &RemoteServoBus::onSocketClosed);
    connect(heartbeat_, &QTimer::timeout, this, &RemoteServoBus::onHeartbeat);
}

RemoteServoBus::~RemoteServoBus() = default;

int RemoteServoBus::replyTimeoutMs() const
{
    // A telemetry sample is eight serial transactions, and each one waits the
    // serial timeout for a servo that is not answering. Anything shorter would
    // report a slow bus as a dead link.
    return qMax(5000, serial_timeout_ms_ * 12);
}

bool RemoteServoBus::connectTransport(const QString &host, quint16 port,
                                      const QString &token, QString *error)
{
    assertOnBusThread();

    const auto fail_with = [error](const QString &message) {
        if(error != nullptr)
            *error = message;
        return false;
    };

    disconnectTransport();
    reader_ = FrameReader();

    socket_->connectToHost(host, port);
    if(!socket_->waitForConnected(CONNECT_TIMEOUT_MS))
    {
        return fail_with(QString("Cannot reach %1:%2 -- %3")
                             .arg(host).arg(port).arg(socket_->errorString()));
    }

    // Nagle would hold a small request back waiting for company, which on a
    // protocol that is nothing but small request/reply pairs is pure latency.
    socket_->setSocketOption(QAbstractSocket::LowDelayOption, 1);

    alive_.storeRelaxed(1);
    in_handshake_ = true;
    last_error_.clear();

    QJsonObject result;
    const bool greeted = request(op::HELLO, QJsonObject{{"proto", PROTOCOL_VERSION},
                                                        {"client", QCoreApplication::applicationName()},
                                                        {"token", token}},
                                 &result);
    in_handshake_ = false;

    if(!greeted)
    {
        const QString reason = last_error_.isEmpty()
                             ? QString("The server did not answer the handshake.")
                             : last_error_;
        disconnectTransport();
        return fail_with(reason);
    }

    const int server_proto = result.value("proto").toInt(-1);
    if(server_proto != PROTOCOL_VERSION)
    {
        disconnectTransport();
        return fail_with(QString("Protocol mismatch: this client speaks version %1, "
                                 "the server speaks %2. Update whichever is older.")
                             .arg(PROTOCOL_VERSION).arg(server_proto));
    }

    server_description_ = QString("%1 %2")
                              .arg(result.value("server").toString("servobench"),
                                   result.value("version").toString());
    server_device_ = result.value("device").toString();
    server_port_open_ = result.value("open").toBool();
    heartbeat_->start();
    return true;
}

void RemoteServoBus::disconnectTransport()
{
    assertOnBusThread();
    heartbeat_->stop();
    alive_.storeRelaxed(0);
    if(socket_->state() != QAbstractSocket::UnconnectedState)
    {
        socket_->disconnectFromHost();
        if(socket_->state() != QAbstractSocket::UnconnectedState)
            socket_->waitForDisconnected(1000);
    }
    socket_->abort();
}

void RemoteServoBus::fail(const QString &reason)
{
    if(alive_.loadRelaxed() == 0)
        return;

    alive_.storeRelaxed(0);
    heartbeat_->stop();
    last_error_ = reason;

    // A handshake that fails is reported by connectTransport, which has the
    // user's attention already. Announcing it a second time as a link that
    // dropped would be one message too many for a connection that never was.
    if(in_handshake_)
        return;

    emit transportLost(reason);
    // The port on the far side may well still be open, but nothing here can
    // reach it any more, so the window has to stop presenting it as usable.
    emit openedChanged(false);
}

void RemoteServoBus::onSocketClosed()
{
    if(socket_->state() == QAbstractSocket::UnconnectedState)
        fail(QString("Connection to the servo server was lost: %1").arg(socket_->errorString()));
}

void RemoteServoBus::onHeartbeat()
{
    if(alive_.loadRelaxed() == 0)
        return;
    QJsonObject result;
    request(op::HEARTBEAT, {}, &result);
}

void RemoteServoBus::onReadyRead()
{
    // While a request is in flight its own read loop owns the socket. Letting
    // this run too would swallow the very reply that loop is waiting for.
    if(in_request_)
        return;

    reader_.feed(socket_->readAll());
    for(;;)
    {
        const std::optional<QJsonObject> message = reader_.next();
        if(!message.has_value())
            break;
        if(message->contains("event"))
            dispatchEvent(*message);
    }

    if(reader_.failed())
        fail(reader_.error());
}

void RemoteServoBus::dispatchEvent(const QJsonObject &message)
{
    const QString name = message.value("event").toString();
    const QJsonObject data = message.value("data").toObject();

    if(name == QLatin1String(ev::OPENED_CHANGED))
    {
        emit openedChanged(data.value("open").toBool());
    }
    else if(name == QLatin1String(ev::SCAN_PROGRESS))
    {
        emit scanProgress(data.value("id").toInt());
    }
    else if(name == QLatin1String(ev::SCAN_FOUND))
    {
        emit scanFound(data.value("id").toInt(), data.value("model").toInt());
    }
    else if(name == QLatin1String(ev::SCAN_FINISHED))
    {
        emit scanFinished(data.value("completed").toBool());
    }
}

bool RemoteServoBus::awaitReply(int seq, QJsonObject *result, QString *error)
{
    QElapsedTimer clock;
    clock.start();
    const int budget = replyTimeoutMs();

    for(;;)
    {
        for(;;)
        {
            const std::optional<QJsonObject> message = reader_.next();
            if(!message.has_value())
                break;

            if(message->contains("event"))
            {
                dispatchEvent(*message);
                continue;
            }

            const int answered = message->value("seq").toInt(-1);

            if(answered != seq && answered != 0)
            {
                // A reply to a request nobody is waiting for any more. There
                // is no correct way to act on it, but the stream is still in
                // step, so it is dropped rather than treated as a fault.
                continue;
            }

            // Sequence zero is the server objecting to the connection itself,
            // which it may do before we have asked for anything. It answers
            // whatever we are waiting for, and everything after it.

            if(!message->value("ok").toBool())
            {
                *error = message->value("error").toString("the server refused the request");
                return false;
            }

            *result = message->value("result").toObject();
            return true;
        }

        if(reader_.failed())
        {
            *error = reader_.error();
            return false;
        }

        if(socket_->state() != QAbstractSocket::ConnectedState)
        {
            *error = QString("the connection dropped while waiting for a reply");
            return false;
        }

        if(clock.elapsed() > budget)
        {
            *error = QString("the server did not answer within %1 ms").arg(budget);
            return false;
        }

        if(socket_->waitForReadyRead(READ_SLICE_MS))
            reader_.feed(socket_->readAll());
    }
}

bool RemoteServoBus::request(const QString &op, const QJsonObject &args, QJsonObject *result)
{
    assertOnBusThread();
    if(alive_.loadRelaxed() == 0)
        return false;

    const int seq = ++seq_;

    QElapsedTimer clock;
    clock.start();

    in_request_ = true;
    socket_->write(encodeFrame(makeRequest(seq, op, args)));

    QString error;
    const bool ok = socket_->flush() && awaitReply(seq, result, &error);
    in_request_ = false;

    if(!ok)
    {
        const QString detail = error.isEmpty() ? socket_->errorString() : error;
        fail(in_handshake_ ? detail : QString("%1 failed: %2").arg(op, detail));
        return false;
    }

    // A single sample would swing with every hiccup, so the reading the window
    // shows is smoothed. Heavily weighted to history, since what matters is
    // whether the link is healthy, not what one packet did.
    const int sample = static_cast<int>(clock.elapsed());
    const int previous = latency_ms_.loadRelaxed();
    latency_ms_.storeRelaxed((previous == 0) ? sample : (previous * 3 + sample) / 4);
    return true;
}

void RemoteServoBus::command(const QString &op, const QJsonObject &args)
{
    QJsonObject ignored;
    request(op, args, &ignored);
}

QVector<PortInfo> RemoteServoBus::listPorts()
{
    QVector<PortInfo> ports;
    QJsonObject result;
    if(!request(op::LIST_PORTS, {}, &result))
        return ports;

    for(const QJsonValue &entry : result.value("ports").toArray())
        ports.append(decodePort(entry.toObject()));
    return ports;
}

bool RemoteServoBus::open(const QString &port_name, int baud, QSerialPort::Parity parity, int timeout)
{
    serial_timeout_ms_ = timeout;

    QJsonObject result;
    const bool ok = request(op::OPEN, QJsonObject{{"port", port_name},
                                                  {"baud", baud},
                                                  {"parity", static_cast<int>(parity)},
                                                  {"timeout", timeout}},
                            &result)
                 && result.value("open").toBool();

    // The server announces the change as an event too, but a failed open
    // produces no event and the window still has to be told.
    if(!ok)
        emit openedChanged(false);
    return ok;
}

void RemoteServoBus::close()
{
    command(op::CLOSE, {});
}

int RemoteServoBus::readByte(uint8_t id, ModelSeries series, uint8_t address)
{
    QJsonObject result;
    if(!request(op::READ_BYTE, QJsonObject{{"id", id}, {"series", series}, {"addr", address}}, &result))
        return -1;
    return result.value("value").toInt(-1);
}

int RemoteServoBus::readWord(uint8_t id, ModelSeries series, uint8_t address)
{
    QJsonObject result;
    if(!request(op::READ_WORD, QJsonObject{{"id", id}, {"series", series}, {"addr", address}}, &result))
        return -1;
    return result.value("value").toInt(-1);
}

void RemoteServoBus::writeByte(uint8_t id, ModelSeries series, uint8_t address, uint8_t value)
{
    command(op::WRITE_BYTE, QJsonObject{{"id", id}, {"series", series},
                                        {"addr", address}, {"value", value}});
}

int RemoteServoBus::readRegister(uint8_t id, ModelSeries series, const MemoryConfig &config)
{
    QJsonObject result;
    if(!request(op::READ_REGISTER,
                QJsonObject{{"id", id}, {"series", series}, {"addr", config.address}}, &result))
        return -1;
    return result.value("value").toInt(-1);
}

std::optional<int> RemoteServoBus::readPosition(uint8_t id, ModelSeries series)
{
    QJsonObject result;
    if(!request(op::READ_POSITION, QJsonObject{{"id", id}, {"series", series}}, &result))
        return std::nullopt;
    const QJsonValue value = result.value("value");
    if(value.isNull() || value.isUndefined())
        return std::nullopt;
    return value.toInt();
}

RegisterWriteResult RemoteServoBus::writeRegister(uint8_t id, ModelSeries series,
                                                  const MemoryConfig &config, int value)
{
    QJsonObject result;
    if(!request(op::WRITE_REGISTER, QJsonObject{{"id", id}, {"series", series},
                                                {"addr", config.address}, {"value", value}},
                &result))
        return RegisterWriteResult();
    return decodeWriteResult(result);
}

MidpointResult RemoteServoBus::setMidpoint(uint8_t id, ModelSeries series, const MemoryConfig &offset)
{
    QJsonObject result;
    if(!request(op::SET_MIDPOINT,
                QJsonObject{{"id", id}, {"series", series}, {"addr", offset.address}}, &result))
        return MidpointResult();
    return decodeMidpoint(result);
}

int RemoteServoBus::readModelNumber(uint8_t id)
{
    QJsonObject result;
    if(!request(op::READ_MODEL_NUMBER, QJsonObject{{"id", id}}, &result))
        return -1;
    return result.value("value").toInt(-1);
}

int RemoteServoBus::ping(uint8_t id)
{
    QJsonObject result;
    if(!request(op::PING, QJsonObject{{"id", id}}, &result))
        return -1;
    return result.value("value").toInt(-1);
}

void RemoteServoBus::writePos(uint8_t id, ModelSeries series, int pos, int time, int speed, int acc)
{
    command(op::WRITE_POS, QJsonObject{{"id", id}, {"series", series}, {"pos", pos},
                                       {"time", time}, {"speed", speed}, {"acc", acc}});
}

void RemoteServoBus::regWritePos(uint8_t id, ModelSeries series, int pos, int time, int speed, int acc)
{
    command(op::REG_WRITE_POS, QJsonObject{{"id", id}, {"series", series}, {"pos", pos},
                                           {"time", time}, {"speed", speed}, {"acc", acc}});
}

void RemoteServoBus::syncWritePos(const std::vector<uint8_t> &ids, ModelSeries series,
                                  int pos, int time, int speed, int acc)
{
    QJsonArray id_array;
    for(uint8_t id : ids)
        id_array.append(id);

    command(op::SYNC_WRITE_POS, QJsonObject{{"ids", id_array}, {"series", series}, {"pos", pos},
                                            {"time", time}, {"speed", speed}, {"acc", acc}});
}

void RemoteServoBus::regWriteAction(uint8_t id)
{
    command(op::REG_WRITE_ACTION, QJsonObject{{"id", id}});
}

void RemoteServoBus::enableTorque(uint8_t id, ModelSeries series, bool on)
{
    command(op::ENABLE_TORQUE, QJsonObject{{"id", id}, {"series", series}, {"on", on}});
}

void RemoteServoBus::flushWrites()
{
    command(op::FLUSH_WRITES, {});
}

ServoStatus RemoteServoBus::readStatus(uint8_t id, ModelSeries series)
{
    QJsonObject result;
    if(!request(op::READ_STATUS, QJsonObject{{"id", id}, {"series", series}}, &result))
    {
        ServoStatus status;
        status.id = id;
        return status;
    }
    return decodeStatus(result.value("status").toObject());
}

void RemoteServoBus::pollStatus(int id, int series)
{
    if(id < 0 || alive_.loadRelaxed() == 0)
    {
        emit statusReady(ServoStatus());
        return;
    }
    emit statusReady(readStatus(static_cast<uint8_t>(id), static_cast<ModelSeries>(series)));
}

void RemoteServoBus::invalidateModeCaches()
{
    command(op::INVALIDATE_MODE_CACHES, {});
}

void RemoteServoBus::startScan(int from, int to)
{
    scan_abort_.storeRelaxed(0);

    QJsonObject result;
    if(!request(op::START_SCAN, QJsonObject{{"from", from}, {"to", to}}, &result))
    {
        // Nothing on the far side is going to answer with scanFinished, so the
        // window would sit waiting for a sweep that never started.
        emit scanFinished(false);
    }
}

void RemoteServoBus::abortScan()
{
    scan_abort_.storeRelaxed(1);

    // Unlike a local bus, the thread that runs the sweep is on the far side of
    // the socket, so stopping it means sending a message -- and the socket may
    // only be touched from the bus thread.
    QMetaObject::invokeMethod(this, [this]{ command(op::ABORT_SCAN, {}); }, Qt::QueuedConnection);
}

}
