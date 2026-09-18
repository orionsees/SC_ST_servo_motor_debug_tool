#ifndef NET_PROTOCOL_H
#define NET_PROTOCOL_H

#include <QByteArray>
#include <QJsonObject>
#include <QString>
#include <QVector>
#include <optional>

#include "servo/servo_bus.h"

// The wire between a ServoBench window and the machine that actually holds the
// serial port.
//
// One TCP connection carries length-prefixed JSON: a four-byte big-endian byte
// count, then that many bytes of UTF-8. JSON rather than a packed struct
// because the whole exchange is a few hundred bytes per telemetry sample --
// about 3 kB/s at the rate the window polls -- so nothing is bought by making
// it unreadable, and a great deal is lost: as it stands the protocol can be
// driven from a shell with netcat, or from a Python script that never links
// against any of this.
//
// Three shapes travel over it:
//
//   request  {"seq": 7, "op": "readStatus", "args": {...}}
//   reply    {"seq": 7, "ok": true, "result": {...}}
//            {"seq": 7, "ok": false, "error": "..."}
//   event    {"event": "status", "data": {...}}
//
// Every request draws exactly one reply carrying its seq. Events are
// unsolicited and carry none, so a client waiting on a reply can tell the two
// apart without having to track what it asked for.
//
// Sequence numbers start at 1. Zero is reserved for an error the server raises
// against the connection rather than against a request -- being turned away
// because another client already holds the bus, say, which has to be sayable
// before the client has asked for anything. A client treats a zero-seq error
// as the answer to whatever it is waiting for, and to everything after it.
//
// The granularity is deliberate. Feetech's protocol is strict
// request/response, so tunnelling raw serial bytes would spend a network round
// trip on every transaction -- eight of them for a single telemetry sample,
// 254 for an ID scan. Cutting here instead means one round trip per operation,
// and it keeps the multi-step sequences (unlock EPROM, write, relock, verify)
// on the robot, where a dropped link cannot strand a servo with its EPROM
// open.

namespace servobench_net
{

// Bumped when a change would make an old client and a new server disagree
// about what a message means. The handshake refuses a mismatch outright rather
// than letting it show up later as a servo behaving strangely.
constexpr int PROTOCOL_VERSION = 1;

constexpr quint16 DEFAULT_PORT = 5555;

// Frames are small by construction; this only exists so a confused or hostile
// peer cannot make us allocate on its say-so.
constexpr int MAX_FRAME_BYTES = 1 << 20;

namespace op
{
constexpr const char *HELLO = "hello";
constexpr const char *LIST_PORTS = "listPorts";
constexpr const char *OPEN = "open";
constexpr const char *CLOSE = "close";
constexpr const char *READ_BYTE = "readByte";
constexpr const char *READ_WORD = "readWord";
constexpr const char *WRITE_BYTE = "writeByte";
constexpr const char *READ_REGISTER = "readRegister";
constexpr const char *READ_POSITION = "readPosition";
constexpr const char *WRITE_REGISTER = "writeRegister";
constexpr const char *SET_MIDPOINT = "setMidpoint";
constexpr const char *READ_MODEL_NUMBER = "readModelNumber";
constexpr const char *PING = "ping";
constexpr const char *WRITE_POS = "writePos";
constexpr const char *REG_WRITE_POS = "regWritePos";
constexpr const char *SYNC_WRITE_POS = "syncWritePos";
constexpr const char *REG_WRITE_ACTION = "regWriteAction";
constexpr const char *ENABLE_TORQUE = "enableTorque";
constexpr const char *FLUSH_WRITES = "flushWrites";
constexpr const char *READ_STATUS = "readStatus";
constexpr const char *INVALIDATE_MODE_CACHES = "invalidateModeCaches";
constexpr const char *START_SCAN = "startScan";
constexpr const char *ABORT_SCAN = "abortScan";
constexpr const char *HEARTBEAT = "heartbeat";
}

// Unsolicited, server to client. Telemetry is deliberately not among them:
// the window asks for each sample and waits for the reply, exactly as it does
// on a local bus, so one code path serves both.
namespace ev
{
constexpr const char *OPENED_CHANGED = "openedChanged";
constexpr const char *SCAN_PROGRESS = "scanProgress";
constexpr const char *SCAN_FOUND = "scanFound";
constexpr const char *SCAN_FINISHED = "scanFinished";
}

// --- framing ---

// One frame, ready for the socket.
QByteArray encodeFrame(const QJsonObject &message);

// Accumulates whatever the socket hands over and yields complete messages.
// A frame can arrive split across reads, or several can arrive in one, so no
// caller may assume a read maps to a message.
class FrameReader
{
public:
    void feed(const QByteArray &bytes) { buffer_.append(bytes); }

    // The next complete message, or nullopt when more bytes are needed.
    // Sets failed() if the peer sent something that cannot be a frame, after
    // which the connection is no longer trustworthy and must be dropped.
    std::optional<QJsonObject> next();

    bool failed() const { return !error_.isEmpty(); }
    QString error() const { return error_; }

private:
    QByteArray buffer_;
    QString error_;
};

// --- message builders ---

QJsonObject makeRequest(int seq, const QString &op, const QJsonObject &args = {});
QJsonObject makeReply(int seq, const QJsonObject &result);
QJsonObject makeError(int seq, const QString &message);
QJsonObject makeEvent(const QString &name, const QJsonObject &data = {});

// --- value codecs ---
//
// Registers cross the wire as a series and an address, never as a copy of a
// MemoryConfig: the table that decides a register's size, its sign-magnitude
// encoding and whether it needs the EPROM unlocked is then always the one on
// the machine holding the port, so a client built from another revision cannot
// corrupt a write with a stale copy of it.

QJsonObject encodeStatus(const feetech_servo::ServoStatus &status);
feetech_servo::ServoStatus decodeStatus(const QJsonObject &object);

QJsonObject encodePort(const feetech_servo::PortInfo &port);
feetech_servo::PortInfo decodePort(const QJsonObject &object);

QJsonObject encodeWriteResult(const feetech_servo::RegisterWriteResult &result);
feetech_servo::RegisterWriteResult decodeWriteResult(const QJsonObject &object);

QJsonObject encodeMidpoint(const feetech_servo::MidpointResult &result);
feetech_servo::MidpointResult decodeMidpoint(const QJsonObject &object);

}

#endif
