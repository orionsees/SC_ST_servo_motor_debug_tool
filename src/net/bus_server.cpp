#include "net/bus_server.h"

#include <QHostAddress>
#include <QJsonArray>
#include <QTextStream>

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
// The client sends a heartbeat every two seconds. Three missed in a row is
// taken as gone -- long enough not to trip over a stalled Wi-Fi moment, short
// enough that a robot does not keep running a command from a laptop that
// closed its lid.
constexpr int DEADMAN_MS = 6000;

ModelSeries seriesOf(const QJsonObject &args)
{
    return static_cast<ModelSeries>(args.value("series").toInt(feetech_servo::ModelSeries::STS));
}

uint8_t idOf(const QJsonObject &args)
{
    return static_cast<uint8_t>(args.value("id").toInt());
}

// "/dev/ttyUSB0" and "ttyUSB0" are the same port. A shell user reaches for the
// full path and the port list carries the bare name, so both spellings turn up
// and have to compare equal.
QString bareName(const QString &device)
{
    return device.startsWith(QLatin1String("/dev/")) ? device.mid(5) : device;
}
}

DisconnectPolicy disconnectPolicyFromName(const QString &name, bool *ok)
{
    if(ok != nullptr)
        *ok = true;

    const QString lowered = name.trimmed().toLower();
    if(lowered == QLatin1String("hold"))
        return DisconnectPolicy::Hold;
    if(lowered == QLatin1String("stop"))
        return DisconnectPolicy::Stop;
    if(lowered == QLatin1String("release"))
        return DisconnectPolicy::Release;

    if(ok != nullptr)
        *ok = false;
    return DisconnectPolicy::Stop;
}

QString disconnectPolicyName(DisconnectPolicy policy)
{
    switch(policy)
    {
        case DisconnectPolicy::Hold:    return "hold";
        case DisconnectPolicy::Stop:    return "stop";
        case DisconnectPolicy::Release: return "release";
    }
    return "stop";
}

BusServer::BusServer(const ServerConfig &config, QObject *parent)
    : QObject(parent)
    , config_(config)
{
    server_ = new QTcpServer(this);
    connect(server_, &QTcpServer::newConnection, this, &BusServer::onNewConnection);

    // The same split the window uses: servo transactions on their own thread,
    // so a blocking serial read never stops this from hearing the client.
    bus_ = new feetech_servo::ServoBus();
    bus_thread_ = new QThread(this);
    bus_->moveToThread(bus_thread_);

    connect(bus_, &feetech_servo::IServoBus::openedChanged, this, &BusServer::onBusOpenedChanged);
    connect(bus_, &feetech_servo::IServoBus::scanProgress, this, &BusServer::onScanProgress);
    connect(bus_, &feetech_servo::IServoBus::scanFound, this, &BusServer::onScanFound);
    connect(bus_, &feetech_servo::IServoBus::scanFinished, this, &BusServer::onScanFinished);

    deadman_ = new QTimer(this);
    deadman_->setInterval(DEADMAN_MS);
    deadman_->setSingleShot(true);
    connect(deadman_, &QTimer::timeout, this, &BusServer::onDeadman);
}

BusServer::~BusServer()
{
    // Close the port on the bus thread, and wait for it: QSerialPort::close()
    // unregisters notifiers belonging to that thread, so it cannot be left to
    // ~QSerialPort running on this one.
    if(bus_thread_->isRunning())
    {
        QMetaObject::invokeMethod(bus_, [this]{ bus_->close(); }, Qt::BlockingQueuedConnection);
        bus_thread_->quit();
        bus_thread_->wait();
    }
    delete bus_;
}

void BusServer::log(const QString &line) const
{
    QTextStream(stdout) << line << "\n";
    QTextStream(stdout).flush();
}

bool BusServer::start(QString *error)
{
    bus_thread_->start();

    QHostAddress address;
    if(!address.setAddress(config_.bind))
    {
        if(config_.bind == QLatin1String("any") || config_.bind == QLatin1String("*"))
        {
            address = QHostAddress::Any;
        }
        else
        {
            *error = QString("'%1' is not an address to bind to").arg(config_.bind);
            return false;
        }
    }

    if(!server_->listen(address, config_.port))
    {
        *error = QString("Cannot listen on %1:%2 -- %3")
                     .arg(config_.bind).arg(config_.port).arg(server_->errorString());
        return false;
    }

    if(!config_.device.isEmpty())
    {
        device_ = config_.device;
        const ServerConfig c = config_;
        bool opened = false;
        QMetaObject::invokeMethod(bus_, [this, c, &opened] {
            opened = bus_->open(c.device, c.baud, c.parity, c.timeout);
        }, Qt::BlockingQueuedConnection);

        if(!opened)
        {
            *error = QString("Cannot open %1. Check the device name, and that this user is "
                             "in the dialout group.").arg(config_.device);
            return false;
        }
    }

    log(QString("servobench serving on %1:%2 (%3, on-disconnect: %4)")
            .arg(config_.bind)
            .arg(config_.port)
            .arg(config_.token.isEmpty() ? "no token" : "token required")
            .arg(disconnectPolicyName(config_.on_disconnect)));

    if(address == QHostAddress::Any && config_.token.isEmpty())
    {
        log("Warning: listening on every interface with no token. Anyone who can reach "
            "this port can move the servos. Pass --token, or bind to 127.0.0.1 and "
            "reach it through an SSH tunnel.");
    }

    return true;
}

void BusServer::onNewConnection()
{
    while(QTcpSocket *incoming = server_->nextPendingConnection())
    {
        if(client_ != nullptr)
        {
            // Two clients on one bus would interleave packets that the servo
            // protocol gives no way to tell apart, so the second is told why
            // rather than left to corrupt the first one's session.
            log(QString("Refused %1: a client is already connected.")
                    .arg(incoming->peerAddress().toString()));
            // Sequence zero: this is about the connection, not about a
            // request -- the client has not made one yet.
            sendTo(incoming, makeError(
                0, "This server already has a client. Only one at a time can drive the bus."));
            closeGently(incoming);
            continue;
        }

        adoptClient(incoming);
    }
}

void BusServer::adoptClient(QTcpSocket *socket)
{
    client_ = socket;
    session_++;
    greeted_ = false;
    scanning_ = false;
    touched_.clear();
    reader_ = FrameReader();

    client_->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    connect(client_, &QTcpSocket::readyRead, this, &BusServer::onClientReadyRead);
    connect(client_, &QTcpSocket::disconnected, this, &BusServer::onClientDisconnected);

    deadman_->start();
    log(QString("Client connected from %1.").arg(client_->peerAddress().toString()));
}

void BusServer::onClientDisconnected()
{
    if(client_ == nullptr)
        return;

    log("Client disconnected.");
    dropClient(QString());
}

void BusServer::onDeadman()
{
    if(client_ == nullptr)
        return;

    log(QString("No heartbeat for %1 ms -- assuming the client is gone.").arg(DEADMAN_MS));
    dropClient("heartbeat timeout");
}

void BusServer::dropClient(const QString &reason)
{
    if(client_ == nullptr)
        return;

    deadman_->stop();

    // Whatever the bus is doing for this client, it is doing it for nobody.
    if(scanning_)
    {
        bus_->abortScan();
        scanning_ = false;
    }

    QTcpSocket *going = client_;
    client_ = nullptr;
    session_++;

    going->disconnect(this);
    closeGently(going);

    if(!reason.isEmpty())
        log(QString("Dropped the client: %1").arg(reason));

    applyDisconnectPolicy();
    touched_.clear();
}

void BusServer::applyDisconnectPolicy()
{
    if(config_.on_disconnect == DisconnectPolicy::Hold || touched_.isEmpty())
    {
        if(config_.on_disconnect == DisconnectPolicy::Hold)
            log("on-disconnect is hold: leaving the servos as they are.");
        return;
    }

    const QSet<QPair<int, int>> servos = touched_;
    const DisconnectPolicy policy = config_.on_disconnect;

    log(QString("Applying on-disconnect=%1 to %2 servo(s).")
            .arg(disconnectPolicyName(policy)).arg(servos.size()));

    QMetaObject::invokeMethod(bus_, [this, servos, policy] {
        for(const QPair<int, int> &servo : servos)
        {
            const auto id = static_cast<uint8_t>(servo.first);
            const auto series = static_cast<ModelSeries>(servo.second);

            if(policy == DisconnectPolicy::Release)
            {
                bus_->enableTorque(id, series, false);
                continue;
            }

            // Stop: command the position it is already in. Torque stays on, so
            // a loaded joint keeps holding, and a servo that was travelling --
            // or turning continuously in wheel mode -- comes to rest here.
            const std::optional<int> pos = bus_->readPosition(id, series);
            if(pos.has_value())
                bus_->writePos(id, series, *pos, 0, 0, 0);
        }
        bus_->flushWrites();
    }, Qt::QueuedConnection);
}

void BusServer::onClientReadyRead()
{
    if(client_ == nullptr)
        return;

    // Any traffic at all means the client is still there.
    deadman_->start();

    reader_.feed(client_->readAll());
    for(;;)
    {
        const std::optional<QJsonObject> message = reader_.next();
        if(!message.has_value())
            break;
        handleMessage(*message);
        if(client_ == nullptr)
            return;
    }

    if(reader_.failed())
    {
        log(QString("Dropping the client: %1").arg(reader_.error()));
        dropClient(reader_.error());
    }
}

void BusServer::handleMessage(const QJsonObject &message)
{
    const int seq = message.value("seq").toInt(-1);
    const QString op = message.value("op").toString();

    if(seq < 0 || op.isEmpty())
    {
        sendError(seq, "not a request");
        return;
    }

    if(op == QLatin1String(::servobench_net::op::HELLO))
    {
        handleHello(seq, message.value("args").toObject());
        return;
    }

    if(!greeted_)
    {
        sendError(seq, "say hello first");
        return;
    }

    handleRequest(seq, op, message.value("args").toObject());
}

bool BusServer::handleHello(int seq, const QJsonObject &args)
{
    const int proto = args.value("proto").toInt(-1);
    if(proto != PROTOCOL_VERSION)
    {
        sendError(seq, QString("Protocol mismatch: this server speaks version %1, the client "
                               "speaks %2. Update whichever is older.")
                           .arg(PROTOCOL_VERSION).arg(proto));
        dropClient("protocol mismatch");
        return false;
    }

    if(!config_.token.isEmpty() && args.value("token").toString() != config_.token)
    {
        log("Refused a client: wrong token.");
        sendError(seq, "Wrong token.");
        dropClient("wrong token");
        return false;
    }

    greeted_ = true;
    sendReply(seq, QJsonObject{
        {"proto", PROTOCOL_VERSION},
        {"server", QCoreApplication::applicationName()},
        {"version", QCoreApplication::applicationVersion()},
        {"device", device_},
        {"open", port_open_},
        {"on_disconnect", disconnectPolicyName(config_.on_disconnect)},
    });
    log(QString("Handshake accepted from %1.").arg(args.value("client").toString("a client")));
    return true;
}

void BusServer::noteTouched(const QJsonObject &args)
{
    const QJsonValue id = args.value("id");
    if(id.isDouble())
    {
        touched_.insert({id.toInt(), args.value("series").toInt(ModelSeries::STS)});
        return;
    }

    for(const QJsonValue &entry : args.value("ids").toArray())
        touched_.insert({entry.toInt(), args.value("series").toInt(ModelSeries::STS)});
}

void BusServer::handleRequest(int seq, const QString &op, const QJsonObject &args)
{
    namespace o = ::servobench_net::op;

    // Answered here rather than on the bus: the point of a heartbeat is to
    // prove the link is alive, which it does whether or not a servo is.
    if(op == QLatin1String(o::HEARTBEAT))
    {
        sendReply(seq, {});
        return;
    }

    if(op == QLatin1String(o::ABORT_SCAN))
    {
        // Thread-safe by design, and it has to be: the bus thread is inside
        // the sweep and could not service a queued call until it ended.
        bus_->abortScan();
        sendReply(seq, {});
        return;
    }

    if(op == QLatin1String(o::START_SCAN))
    {
        const int from = args.value("from").toInt(0);
        const int to = args.value("to").toInt(0xfd);
        scanning_ = true;
        // Answered before the sweep starts, because the client is not waiting
        // on the result -- it is waiting on the events the sweep produces.
        sendReply(seq, QJsonObject{{"started", true}});
        QMetaObject::invokeMethod(bus_, [this, from, to] { bus_->startScan(from, to); },
                                  Qt::QueuedConnection);
        return;
    }

    if(op == QLatin1String(o::LIST_PORTS))
    {
        const QString configured = device_;
        onBus(seq, [this, configured] {
            QVector<PortInfo> found = bus_->listPorts();

            // A device this server was pointed at deliberately belongs in the
            // list even when the OS does not enumerate it -- a udev symlink
            // such as /dev/servo-bus, or a pty. Leaving it out would make the
            // one port the server was set up to drive the one port a client
            // could not choose.
            if(!configured.isEmpty())
            {
                bool listed = false;
                for(const PortInfo &port : found)
                    listed = listed || bareName(port.name) == bareName(configured);

                if(!listed)
                {
                    PortInfo entry;
                    entry.name = configured;
                    entry.description = "configured on the server";
                    found.prepend(entry);
                }
            }

            QJsonArray ports;
            for(const PortInfo &port : found)
                ports.append(encodePort(port));
            return QJsonObject{{"ports", ports}};
        });
        return;
    }

    if(op == QLatin1String(o::OPEN))
    {
        const QString port = args.value("port").toString();
        const int baud = args.value("baud").toInt(1000000);
        const auto parity = static_cast<QSerialPort::Parity>(args.value("parity").toInt());
        const int timeout = args.value("timeout").toInt(50);
        log(QString("Client asked for %1 at %2 baud.").arg(port).arg(baud));
        device_ = port;
        onBus(seq, [this, port, baud, parity, timeout] {
            return QJsonObject{{"open", bus_->open(port, baud, parity, timeout)}};
        });
        return;
    }

    if(op == QLatin1String(o::CLOSE))
    {
        onBus(seq, [this] { bus_->close(); return QJsonObject(); });
        return;
    }

    if(op == QLatin1String(o::INVALIDATE_MODE_CACHES))
    {
        onBus(seq, [this] { bus_->invalidateModeCaches(); return QJsonObject(); });
        return;
    }

    if(op == QLatin1String(o::FLUSH_WRITES))
    {
        onBus(seq, [this] { bus_->flushWrites(); return QJsonObject(); });
        return;
    }

    // The disconnect policy only answers for servos this client actually
    // commanded. A servo that was merely read has not been disturbed, and
    // "stop" would otherwise write a goal -- and, on one in wheel mode, a work
    // mode -- to a servo the client only ever looked at.
    static const QSet<QString> commands = {
        QLatin1String(o::WRITE_BYTE),   QLatin1String(o::WRITE_REGISTER),
        QLatin1String(o::SET_MIDPOINT), QLatin1String(o::WRITE_POS),
        QLatin1String(o::REG_WRITE_POS), QLatin1String(o::SYNC_WRITE_POS),
        QLatin1String(o::REG_WRITE_ACTION), QLatin1String(o::ENABLE_TORQUE),
    };
    if(commands.contains(op))
        noteTouched(args);

    const uint8_t id = idOf(args);
    const ModelSeries series = seriesOf(args);

    if(op == QLatin1String(o::PING))
    {
        onBus(seq, [this, id] { return QJsonObject{{"value", bus_->ping(id)}}; });
        return;
    }

    if(op == QLatin1String(o::READ_MODEL_NUMBER))
    {
        onBus(seq, [this, id] { return QJsonObject{{"value", bus_->readModelNumber(id)}}; });
        return;
    }

    if(op == QLatin1String(o::READ_STATUS))
    {
        onBus(seq, [this, id, series] {
            return QJsonObject{{"status", encodeStatus(bus_->readStatus(id, series))}};
        });
        return;
    }

    if(op == QLatin1String(o::READ_POSITION))
    {
        onBus(seq, [this, id, series] {
            const std::optional<int> pos = bus_->readPosition(id, series);
            return QJsonObject{{"value", pos.has_value() ? QJsonValue(*pos) : QJsonValue()}};
        });
        return;
    }

    if(op == QLatin1String(o::READ_BYTE))
    {
        const auto address = static_cast<uint8_t>(args.value("addr").toInt());
        onBus(seq, [this, id, series, address] {
            return QJsonObject{{"value", bus_->readByte(id, series, address)}};
        });
        return;
    }

    if(op == QLatin1String(o::READ_WORD))
    {
        const auto address = static_cast<uint8_t>(args.value("addr").toInt());
        onBus(seq, [this, id, series, address] {
            return QJsonObject{{"value", bus_->readWord(id, series, address)}};
        });
        return;
    }

    if(op == QLatin1String(o::WRITE_BYTE))
    {
        const auto address = static_cast<uint8_t>(args.value("addr").toInt());
        const auto value = static_cast<uint8_t>(args.value("value").toInt());
        onBus(seq, [this, id, series, address, value] {
            bus_->writeByte(id, series, address, value);
            return QJsonObject();
        });
        return;
    }

    if(op == QLatin1String(o::ENABLE_TORQUE))
    {
        const bool on = args.value("on").toBool();
        onBus(seq, [this, id, series, on] {
            bus_->enableTorque(id, series, on);
            return QJsonObject();
        });
        return;
    }

    if(op == QLatin1String(o::REG_WRITE_ACTION))
    {
        onBus(seq, [this, id] { bus_->regWriteAction(id); return QJsonObject(); });
        return;
    }

    if(op == QLatin1String(o::WRITE_POS) || op == QLatin1String(o::REG_WRITE_POS))
    {
        const int pos = args.value("pos").toInt();
        const int time = args.value("time").toInt();
        const int speed = args.value("speed").toInt();
        const int acc = args.value("acc").toInt();
        const bool reg = (op == QLatin1String(o::REG_WRITE_POS));
        onBus(seq, [this, id, series, pos, time, speed, acc, reg] {
            if(reg)
                bus_->regWritePos(id, series, pos, time, speed, acc);
            else
                bus_->writePos(id, series, pos, time, speed, acc);
            return QJsonObject();
        });
        return;
    }

    if(op == QLatin1String(o::SYNC_WRITE_POS))
    {
        std::vector<uint8_t> ids;
        for(const QJsonValue &entry : args.value("ids").toArray())
            ids.push_back(static_cast<uint8_t>(entry.toInt()));

        const int pos = args.value("pos").toInt();
        const int time = args.value("time").toInt();
        const int speed = args.value("speed").toInt();
        const int acc = args.value("acc").toInt();
        onBus(seq, [this, ids, series, pos, time, speed, acc] {
            bus_->syncWritePos(ids, series, pos, time, speed, acc);
            return QJsonObject();
        });
        return;
    }

    // The register operations take an address and look the register up here,
    // against this machine's tables. A client built from another revision
    // therefore cannot talk us into writing the wrong width, or into skipping
    // the EPROM unlock a register needs.
    if(op == QLatin1String(o::READ_REGISTER) || op == QLatin1String(o::WRITE_REGISTER)
       || op == QLatin1String(o::SET_MIDPOINT))
    {
        const auto address = static_cast<uint8_t>(args.value("addr").toInt());
        const MemoryConfig *config = feetech_servo::findMemConfigByAddress(series, address);
        if(config == nullptr)
        {
            sendError(seq, QString("no register at address %1 for series %2")
                               .arg(address).arg(feetech_servo::seriesName(series)));
            return;
        }

        if(op == QLatin1String(o::READ_REGISTER))
        {
            onBus(seq, [this, id, series, config] {
                return QJsonObject{{"value", bus_->readRegister(id, series, *config)}};
            });
            return;
        }

        if(op == QLatin1String(o::SET_MIDPOINT))
        {
            onBus(seq, [this, id, series, config] {
                return encodeMidpoint(bus_->setMidpoint(id, series, *config));
            });
            return;
        }

        const int value = args.value("value").toInt();
        onBus(seq, [this, id, series, config, value] {
            return encodeWriteResult(bus_->writeRegister(id, series, *config, value));
        });
        return;
    }

    sendError(seq, QString("unknown operation '%1'").arg(op));
}

void BusServer::onBusOpenedChanged(bool is_open)
{
    port_open_ = is_open;
    log(is_open ? QString("Port %1 is open.").arg(device_)
                : QString("Port %1 is closed.").arg(device_));
    sendEvent(ev::OPENED_CHANGED, QJsonObject{{"open", is_open}});
}

void BusServer::onScanProgress(int id)
{
    sendEvent(ev::SCAN_PROGRESS, QJsonObject{{"id", id}});
}

void BusServer::onScanFound(int id, int model_number)
{
    sendEvent(ev::SCAN_FOUND, QJsonObject{{"id", id}, {"model", model_number}});
}

void BusServer::onScanFinished(bool completed)
{
    scanning_ = false;
    sendEvent(ev::SCAN_FINISHED, QJsonObject{{"completed", completed}});
}

void BusServer::sendReply(int seq, const QJsonObject &result)
{
    sendFrame(makeReply(seq, result));
}

void BusServer::sendError(int seq, const QString &message)
{
    sendFrame(makeError(seq, message));
}

void BusServer::sendEvent(const QString &name, const QJsonObject &data)
{
    sendFrame(makeEvent(name, data));
}

void BusServer::sendFrame(const QJsonObject &message)
{
    if(client_ == nullptr)
        return;
    sendTo(client_, message);
}

void BusServer::sendTo(QTcpSocket *socket, const QJsonObject &message)
{
    if(socket == nullptr || socket->state() != QAbstractSocket::ConnectedState)
        return;

    socket->write(encodeFrame(message));
    socket->flush();
}

void BusServer::closeGently(QTcpSocket *socket)
{
    // A client being turned away has just been told why, and abort() would
    // reset the connection out from under that message -- it would read as a
    // dropped connection rather than as the reason it was refused. Let the
    // bytes go out, then close, and only then let go of the socket.
    //
    // A client that hung up first is already gone, so there is nothing to
    // flush and nothing to wait for.
    if(socket->state() == QAbstractSocket::ConnectedState)
    {
        socket->waitForBytesWritten(1000);
        socket->disconnectFromHost();
    }

    if(socket->state() == QAbstractSocket::UnconnectedState)
    {
        socket->deleteLater();
        return;
    }

    connect(socket, &QTcpSocket::disconnected, socket, &QObject::deleteLater);
}

}
