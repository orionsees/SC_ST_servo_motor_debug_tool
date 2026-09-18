#include "net/protocol.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QtEndian>

namespace servobench_net
{

using feetech_servo::MidpointResult;
using feetech_servo::PortInfo;
using feetech_servo::RegisterWriteResult;
using feetech_servo::ServoStatus;

namespace
{
constexpr int HEADER_BYTES = 4;
}

QByteArray encodeFrame(const QJsonObject &message)
{
    const QByteArray payload = QJsonDocument(message).toJson(QJsonDocument::Compact);

    QByteArray frame;
    frame.resize(HEADER_BYTES);
    qToBigEndian<quint32>(static_cast<quint32>(payload.size()),
                          reinterpret_cast<uchar *>(frame.data()));
    frame.append(payload);
    return frame;
}

std::optional<QJsonObject> FrameReader::next()
{
    if(failed())
        return std::nullopt;

    if(buffer_.size() < HEADER_BYTES)
        return std::nullopt;

    const quint32 length =
        qFromBigEndian<quint32>(reinterpret_cast<const uchar *>(buffer_.constData()));

    if(length > static_cast<quint32>(MAX_FRAME_BYTES))
    {
        // Either the peer is not speaking this protocol at all or something
        // has desynchronised the stream. Neither is recoverable by reading
        // further, because we no longer know where the next frame starts.
        error_ = QString("frame of %1 bytes exceeds the %2 byte limit")
                     .arg(length)
                     .arg(MAX_FRAME_BYTES);
        return std::nullopt;
    }

    if(static_cast<quint32>(buffer_.size()) < HEADER_BYTES + length)
        return std::nullopt;

    const QByteArray payload = buffer_.mid(HEADER_BYTES, static_cast<int>(length));
    buffer_.remove(0, HEADER_BYTES + static_cast<int>(length));

    QJsonParseError parse{};
    const QJsonDocument document = QJsonDocument::fromJson(payload, &parse);
    if(parse.error != QJsonParseError::NoError || !document.isObject())
    {
        error_ = QString("malformed frame: %1").arg(parse.errorString());
        return std::nullopt;
    }

    return document.object();
}

QJsonObject makeRequest(int seq, const QString &op, const QJsonObject &args)
{
    return QJsonObject{{"seq", seq}, {"op", op}, {"args", args}};
}

QJsonObject makeReply(int seq, const QJsonObject &result)
{
    return QJsonObject{{"seq", seq}, {"ok", true}, {"result", result}};
}

QJsonObject makeError(int seq, const QString &message)
{
    return QJsonObject{{"seq", seq}, {"ok", false}, {"error", message}};
}

QJsonObject makeEvent(const QString &name, const QJsonObject &data)
{
    return QJsonObject{{"event", name}, {"data", data}};
}

QJsonObject encodeStatus(const ServoStatus &status)
{
    return QJsonObject{
        {"id", status.id},
        {"pos", status.pos},
        {"torque", status.torque},
        {"speed", status.speed},
        {"current", status.current},
        {"temp", status.temp},
        {"voltage", status.voltage},
        {"move", status.move},
        {"goal", status.goal},
        {"ok", status.ok},
        {"t", static_cast<double>(status.t_ms)},
    };
}

ServoStatus decodeStatus(const QJsonObject &object)
{
    ServoStatus status;
    status.id = object.value("id").toInt(-1);
    status.pos = object.value("pos").toInt();
    status.torque = object.value("torque").toInt();
    status.speed = object.value("speed").toInt();
    status.current = object.value("current").toInt();
    status.temp = object.value("temp").toInt();
    status.voltage = object.value("voltage").toInt();
    status.move = object.value("move").toInt();
    status.goal = object.value("goal").toInt();
    status.ok = object.value("ok").toBool();
    status.t_ms = static_cast<qint64>(object.value("t").toDouble(-1));
    return status;
}

QJsonObject encodePort(const PortInfo &port)
{
    return QJsonObject{
        {"name", port.name},
        {"description", port.description},
        {"manufacturer", port.manufacturer},
        {"vid", port.has_vendor_id},
    };
}

PortInfo decodePort(const QJsonObject &object)
{
    PortInfo port;
    port.name = object.value("name").toString();
    port.description = object.value("description").toString();
    port.manufacturer = object.value("manufacturer").toString();
    port.has_vendor_id = object.value("vid").toBool();
    return port;
}

QJsonObject encodeWriteResult(const RegisterWriteResult &result)
{
    return QJsonObject{{"ok", result.ok}, {"effective_id", result.effective_id}};
}

RegisterWriteResult decodeWriteResult(const QJsonObject &object)
{
    RegisterWriteResult result;
    result.ok = object.value("ok").toBool();
    result.effective_id = object.value("effective_id").toInt(-1);
    return result;
}

QJsonObject encodeMidpoint(const MidpointResult &result)
{
    return QJsonObject{
        {"acked", result.acked},
        {"before", result.before},
        {"after", result.after},
    };
}

MidpointResult decodeMidpoint(const QJsonObject &object)
{
    MidpointResult result;
    result.acked = object.value("acked").toBool();
    result.before = object.value("before").toInt(-1);
    result.after = object.value("after").toInt(-1);
    return result;
}

}
