#include "cli/session.h"

#include <QDir>
#include <QSerialPortInfo>
#include <QTextStream>
#include <cmath>

namespace cli
{

using feetech_servo::countsPerRev;
using feetech_servo::findMemConfig;
using feetech_servo::getMemConfig;
using feetech_servo::getModelSeries;
using feetech_servo::getModelType;
using feetech_servo::TORQUE_ENABLE_ADDRESS;

QVector<PortInfo> availablePorts()
{
    QVector<PortInfo> with_vid;
    QVector<PortInfo> all;

    for(const QSerialPortInfo &info : QSerialPortInfo::availablePorts())
    {
        PortInfo entry;
        entry.name = info.portName();
        entry.description = info.description();
        entry.manufacturer = info.manufacturer();
        entry.has_vendor_id = info.hasVendorIdentifier();

        all.append(entry);
        if(entry.has_vendor_id)
            with_vid.append(entry);
    }

    return with_vid.isEmpty() ? all : with_vid;
}

QStringList splitFields(const QString &text, const QRegExp &separator)
{
    QStringList out;
    for(const QString &piece : text.split(separator))
    {
        const QString trimmed = piece.trimmed();
        if(!trimmed.isEmpty())
            out << trimmed;
    }
    return out;
}

double countsToRadians(ModelSeries series, int count)
{
    const int per_rev = countsPerRev(series);
    return (count - per_rev / 2) * (2.0 * M_PI / per_rev);
}

double countsToDegrees(ModelSeries series, int count)
{
    const int per_rev = countsPerRev(series);
    return (count - per_rev / 2) * (360.0 / per_rev);
}

int angleToCounts(ModelSeries series, double angle, bool is_radians)
{
    const int per_rev = countsPerRev(series);
    const double per_count = is_radians ? (2.0 * M_PI / per_rev) : (360.0 / per_rev);
    return per_rev / 2 + static_cast<int>(std::lround(angle / per_count));
}

namespace
{

// Register names carry double spaces and one long-standing typo, and nobody
// wants to type either. Comparison happens on a squashed, lowercased form.
QString normalise(const QString &name)
{
    QString out;
    for(const QChar &c : name)
    {
        if(c.isLetterOrNumber())
            out += c.toLower();
    }
    return out;
}

}

const MemoryConfig *findRegister(ModelSeries series, const QString &token, QString *error)
{
    const auto &configs = getMemConfig(series);
    const QString trimmed = token.trimmed();

    bool numeric = false;
    const int address = trimmed.toInt(&numeric);
    if(numeric)
    {
        for(const auto &config : configs)
        {
            if(config.address == address)
                return &config;
        }
        if(error)
            *error = QString("No register at address %1 on a %2 servo.")
                         .arg(address).arg(feetech_servo::seriesName(series));
        return nullptr;
    }

    const QString wanted = normalise(trimmed);
    const MemoryConfig *prefix_hit = nullptr;
    int prefix_count = 0;

    for(const auto &config : configs)
    {
        const QString name = normalise(config.name);
        if(name == wanted)
            return &config;
        if(name.startsWith(wanted) && !wanted.isEmpty())
        {
            prefix_hit = &config;
            prefix_count++;
        }
    }

    if(prefix_count == 1)
        return prefix_hit;

    if(error)
    {
        *error = prefix_count > 1
               ? QString("\"%1\" matches %2 registers. Use the full name or the address.")
                     .arg(trimmed).arg(prefix_count)
               : QString("No register called \"%1\" on a %2 servo.")
                     .arg(trimmed).arg(feetech_servo::seriesName(series));
    }
    return nullptr;
}

Session::Session(bool threaded, QObject *parent)
    : QObject(parent)
{
    bus_ = new feetech_servo::ServoBus();

    connect(bus_, &feetech_servo::ServoBus::openedChanged, this,
            [this](bool is_open) { open_ = is_open; });

    if(threaded)
    {
        bus_thread_ = new QThread(this);
        bus_->moveToThread(bus_thread_);
        bus_thread_->start();
    }
}

Session::~Session()
{
    if(bus_thread_ != nullptr)
    {
        // Close on the bus thread and wait for it: QSerialPort::close()
        // unregisters notifiers that belong to that thread, so it must not be
        // left to ~QSerialPort on this one.
        QMetaObject::invokeMethod(bus_, [this] { bus_->close(); }, Qt::BlockingQueuedConnection);
        bus_thread_->quit();
        bus_thread_->wait();
    }
    else if(open_)
    {
        bus_->close();
    }

    delete bus_;
}

bool Session::open(const ConnectionOptions &options, QString *error)
{
    options_ = options;

    if(options_.port.isEmpty())
    {
        const QVector<PortInfo> ports = availablePorts();
        if(ports.isEmpty())
        {
            if(error)
                *error = "No serial ports found.";
            return false;
        }
        options_.port = ports.first().name;
    }

    // A bare port name is what the GUI's dropdown carries; a full device path
    // is what a shell user reaches for. Both work.
    QString device = options_.port;
    if(device.startsWith("/dev/"))
        device = device.mid(5);

    const ConnectionOptions o = options_;
    const bool ok = call([this, device, o] {
        return bus_->open(device, o.baud, o.parity, o.timeout);
    });

    if(!ok && error)
    {
        *error = QString("Could not open %1. Check the device exists and that you are in "
                         "the dialout group.").arg(options_.port);
    }
    return ok;
}

void Session::close()
{
    call([this] { bus_->close(); return true; });
    clearServos();
}

void Session::setServos(const QVector<ServoEntry> &servos)
{
    servos_ = servos;
}

void Session::addServo(const ServoEntry &entry)
{
    for(ServoEntry &existing : servos_)
    {
        if(existing.id == entry.id)
        {
            existing = entry;
            return;
        }
    }
    servos_.append(entry);
}

void Session::clearServos()
{
    servos_.clear();
}

QVector<int> Session::ids() const
{
    QVector<int> out;
    out.reserve(servos_.size());
    for(const ServoEntry &entry : servos_)
        out.append(entry.id);
    return out;
}

bool Session::knows(int id) const
{
    for(const ServoEntry &entry : servos_)
    {
        if(entry.id == id)
            return true;
    }
    return false;
}

ModelSeries Session::seriesFor(int id) const
{
    for(const ServoEntry &entry : servos_)
    {
        if(entry.id == id)
            return entry.series;
    }
    // An ID that was never scanned is addressed as an STS, which is what the
    // GUI falls back to as well.
    return ModelSeries::STS;
}

QString Session::modelFor(int id) const
{
    for(const ServoEntry &entry : servos_)
    {
        if(entry.id == id)
            return entry.model;
    }
    return QString("Unknown");
}

int Session::countsPerRev(int id) const
{
    return feetech_servo::countsPerRev(seriesFor(id));
}

ServoEntry Session::probe(int id)
{
    const int answered = call([this, id] { return bus_->ping(static_cast<uint8_t>(id)); });

    ServoEntry entry;
    if(answered <= 0)
        return entry;

    const int model_number = call([this, answered] {
        return bus_->readModelNumber(static_cast<uint8_t>(answered));
    });

    entry.id = answered;
    entry.model = getModelType(model_number);
    entry.series = getModelSeries(entry.model);
    return entry;
}

QVector<ServoEntry> Session::scan(int from, int to, const std::function<bool(int)> &progress)
{
    QVector<ServoEntry> found;
    for(int id = from; id <= to; id++)
    {
        if(progress && !progress(id))
            break;

        const ServoEntry entry = probe(id);
        if(entry.id >= 0)
            found.append(entry);
    }
    return found;
}

std::optional<int> Session::readRegister(int id, const MemoryConfig &config)
{
    const ModelSeries series = seriesFor(id);
    const int raw = call([this, id, series, config] {
        const uint8_t target = static_cast<uint8_t>(id);
        return (config.size == 2) ? bus_->readWord(target, series, config.address)
                                  : bus_->readByte(target, series, config.address);
    });

    if(raw < 0)
        return std::nullopt;
    if(feetech_servo::isSignMagnitude(config))
        return feetech_servo::decodeSignMagnitude(raw, config.dir_bit);
    return raw;
}

RegisterWriteResult Session::writeRegister(int id, const MemoryConfig &config, int value)
{
    const ModelSeries series = seriesFor(id);
    return call([this, id, series, config, value] {
        return bus_->writeRegister(static_cast<uint8_t>(id), series, config, value);
    });
}

std::optional<int> Session::readPosition(int id)
{
    const ModelSeries series = seriesFor(id);
    return call([this, id, series] {
        return bus_->readPosition(static_cast<uint8_t>(id), series);
    });
}

ServoStatus Session::readStatus(int id)
{
    const ModelSeries series = seriesFor(id);
    return call([this, id, series] {
        return bus_->readStatus(static_cast<uint8_t>(id), series);
    });
}

int Session::readByte(int id, uint8_t address)
{
    const ModelSeries series = seriesFor(id);
    return call([this, id, series, address] {
        return bus_->readByte(static_cast<uint8_t>(id), series, address);
    });
}

QVector<Session::ServoFlags> Session::readFlags(const QVector<int> &ids)
{
    QVector<QPair<int, ModelSeries>> targets;
    targets.reserve(ids.size());
    for(int id : ids)
        targets.append(qMakePair(id, seriesFor(id)));

    // Torque state and fault state for every servo in one visit. Both are one
    // byte, so folding the status read in costs a round trip per servo and
    // means a faulted joint shows up without being selected.
    return call([this, targets] {
        QVector<ServoFlags> out;
        out.reserve(targets.size());
        for(const auto &target : targets)
        {
            const uint8_t id = static_cast<uint8_t>(target.first);
            ServoFlags flags;
            flags.torque = bus_->readByte(id, target.second, TORQUE_ENABLE_ADDRESS);
            flags.fault = bus_->readByte(id, target.second, feetech_servo::SERVO_STATUS_ADDRESS);
            out.append(flags);
        }
        return out;
    });
}

QStringList Session::setTorque(const QVector<int> &ids, bool on)
{
    QVector<QPair<int, ModelSeries>> targets;
    targets.reserve(ids.size());
    for(int id : ids)
        targets.append(qMakePair(id, seriesFor(id)));

    const uint8_t value = on ? 1 : 0;

    const QVector<int> read_back = call([this, targets, value] {
        for(const auto &target : targets)
            bus_->writeByte(static_cast<uint8_t>(target.first), target.second,
                            TORQUE_ENABLE_ADDRESS, value);

        QVector<int> out;
        out.reserve(targets.size());
        for(const auto &target : targets)
        {
            out.append(bus_->readByte(static_cast<uint8_t>(target.first), target.second,
                                      TORQUE_ENABLE_ADDRESS));
        }
        return out;
    });

    QStringList failed;
    for(int i = 0; i < ids.size() && i < read_back.size(); i++)
    {
        if(read_back[i] < 0 || (read_back[i] != 0) != on)
            failed << QString::number(ids[i]);
    }
    return failed;
}

MidpointResult Session::setMidpoint(int id, const MemoryConfig &offset)
{
    const ModelSeries series = seriesFor(id);
    return call([this, id, series, offset] {
        return bus_->setMidpoint(static_cast<uint8_t>(id), series, offset);
    });
}

void Session::commandPosition(int id, WriteMode mode, int pos, int time, int speed, int acc)
{
    const ModelSeries series = seriesFor(id);
    const uint8_t target = static_cast<uint8_t>(id);

    if(mode == WriteMode::RegWrite)
    {
        post([this, target, series, pos, time, speed, acc] {
            bus_->regWritePos(target, series, pos, time, speed, acc);
        });
        return;
    }

    if(mode == WriteMode::SyncWrite)
    {
        // A sync write addressed at one servo is still a sync write: the
        // packet goes out unacknowledged, which is the point of the mode.
        syncCommandPosition({id}, pos, time, speed, acc);
        return;
    }

    post([this, target, series, pos, time, speed, acc] {
        bus_->writePos(target, series, pos, time, speed, acc);
    });
}

void Session::syncCommandPosition(const QVector<int> &ids, int pos, int time, int speed, int acc)
{
    if(ids.isEmpty())
        return;

    // One sync write reaches every servo in a single packet, so the series --
    // and with it the wire endianness -- has to be the same for all of them.
    const ModelSeries series = seriesFor(ids.first());
    std::vector<uint8_t> targets;
    targets.reserve(ids.size());
    for(int id : ids)
        targets.push_back(static_cast<uint8_t>(id));

    post([this, targets, series, pos, time, speed, acc] {
        bus_->syncWritePos(targets, series, pos, time, speed, acc);
    });
}

void Session::regWriteAction(int id)
{
    const uint8_t target = static_cast<uint8_t>(id);
    post([this, target] { bus_->regWriteAction(target); });
}

QVector<Session::HomeResult> Session::setHome(const QVector<int> &ids)
{
    QVector<QPair<int, ModelSeries>> targets;
    targets.reserve(ids.size());
    for(int id : ids)
        targets.append(qMakePair(id, seriesFor(id)));

    // The whole sweep is one visit to the bus. Nothing else can slip a
    // transaction in between a servo's offset write and its verification.
    return call([this, targets] {
        QVector<HomeResult> out;
        out.reserve(targets.size());

        for(const auto &target : targets)
        {
            const uint8_t id = static_cast<uint8_t>(target.first);
            const ModelSeries series = target.second;
            const int max_res = feetech_servo::countsPerRev(series) - 1;
            const int half_turn = max_res / 2;

            HomeResult row;
            row.id = target.first;
            row.half_turn = half_turn;

            auto write = [&](const char *name, int value) {
                const MemoryConfig *config = findMemConfig(series, name);
                return config && bus_->writeRegister(id, series, *config, value).ok;
            };
            auto position = [&]() { return bus_->readPosition(id, series).value_or(-1); };

            bool ok = write("Position Offset Value", 0);
            ok = write("Min Position Limit", 0) && ok;
            ok = write("Max Position Limit", max_res) && ok;

            const int raw = position();
            if(!ok || raw < 0)
            {
                out.append(row);
                continue;
            }

            row.homing_offset = half_turn - raw;
            if(!write("Position Offset Value", row.homing_offset))
            {
                out.append(row);
                continue;
            }

            row.written = true;
            row.corrected = position();
            out.append(row);
        }
        return out;
    });
}

QString expandHome(const QString &path)
{
    if(path == "~")
        return QDir::homePath();
    if(path.startsWith("~/"))
        return QDir::homePath() + path.mid(1);
    return path;
}

bool Recorder::start(const QString &path, int interval_s, QString *error)
{
    path_ = expandHome(path);
    interval_s_ = interval_s > 0 ? interval_s : 1;
    rows_ = 0;
    pending_.clear();

    QFile file(path_);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        if(error)
            *error = QString("Could not write %1.").arg(path_);
        return false;
    }

    QTextStream stream(&file);
    stream << "No,Pos,Gol,Ft,V,C,T,Vol\n";
    file.close();

    active_ = true;
    return true;
}

void Recorder::append(const ServoStatus &status)
{
    if(!active_)
        return;

    rows_++;
    pending_ += QString("%1,%2,%3,%4,%5,%6,%7,%8,END\n")
                    .arg(rows_)
                    .arg(status.pos)
                    .arg(status.goal)
                    .arg(status.torque)
                    .arg(status.speed)
                    .arg(status.current)
                    .arg(status.temp)
                    .arg(status.voltage);

    // Batched the way the GUI batches: a section is 20 samples per second of
    // the configured interval, so the file is not reopened for every sample.
    const quint64 section = 20ull * static_cast<quint64>(interval_s_);
    if(section > 0 && rows_ % section == 0)
        flushSection();
}

void Recorder::stop()
{
    if(!active_)
        return;

    flushSection();
    active_ = false;
}

void Recorder::reset()
{
    if(active_)
        return;
    rows_ = 0;
    pending_.clear();
}

bool Recorder::flushSection()
{
    if(pending_.isEmpty())
        return true;

    QFile file(path_);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append))
        return false;

    QTextStream stream(&file);
    stream << pending_;
    file.close();
    pending_.clear();
    return true;
}

}
