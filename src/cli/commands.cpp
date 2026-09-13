#include "cli/commands.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegExp>
#include <QSerialPortInfo>
#include <QTextStream>
#include <QThread>

#include <algorithm>
#include <cmath>
#include <csignal>
#include <memory>
#include <unistd.h>

#include "cli/calib_state.h"
#include "confirm_steps.h"
#include "register_snapshot.h"

namespace cli
{

using feetech_servo::decodeServoStatus;
using feetech_servo::getMemConfig;
using feetech_servo::findMemConfig;
using feetech_servo::seriesName;
using feetech_servo::supportsMidpointCalibration;

namespace
{

volatile sig_atomic_t g_interrupted = 0;

void onInterrupt(int)
{
    g_interrupted = 1;
}

QTextStream &out()
{
    static QTextStream stream(stdout);
    return stream;
}

QTextStream &err()
{
    static QTextStream stream(stderr);
    return stream;
}

void fail(const QString &message)
{
    err() << "servobench: " << message << "\n";
    err().flush();
}

// Options that take no value. Everything else consumes the next argument.
const QSet<QString> &booleanFlags()
{
    static const QSet<QString> flags = {
        "help", "version", "yes", "json", "csv", "all", "wait", "quiet",
        "plain", "ascii", "no-mouse", "verbose",
    };
    return flags;
}

QString expandShort(const QString &name)
{
    static const QMap<QString, QString> shorts = {
        {"p", "port"}, {"b", "baud"}, {"t", "timeout"},
        {"h", "help"}, {"y", "yes"}, {"j", "json"}, {"v", "verbose"},
    };
    return shorts.value(name, name);
}

bool stdinIsTty()
{
    return isatty(STDIN_FILENO) == 1;
}

QString registerArea(const MemoryConfig &config)
{
    return config.is_eprom ? "EPROM" : "SRAM";
}

// "1,3,5" and "1-6" and any mix of the two.
QVector<int> parseIdList(const QString &text, bool *ok)
{
    QVector<int> ids;
    *ok = true;

    for(const QString &piece : splitFields(text, QRegExp("[,;\\s]+")))
    {
        if(piece.contains('-'))
        {
            const QStringList bounds = piece.split('-');
            bool from_ok = false;
            bool to_ok = false;
            const int from = bounds.value(0).toInt(&from_ok);
            const int to = bounds.value(1).toInt(&to_ok);
            if(bounds.size() != 2 || !from_ok || !to_ok || from > to)
            {
                *ok = false;
                return {};
            }
            for(int id = from; id <= to; id++)
                ids.append(id);
            continue;
        }

        bool piece_ok = false;
        const int id = piece.toInt(&piece_ok);
        if(!piece_ok)
        {
            *ok = false;
            return {};
        }
        ids.append(id);
    }
    return ids;
}

// A fixed-width table, so a terminal dump lines up and `awk` still works on it.
void printTable(const QStringList &headers, const QVector<QStringList> &rows)
{
    QVector<int> widths;
    for(const QString &header : headers)
        widths.append(header.size());

    for(const QStringList &row : rows)
    {
        for(int i = 0; i < row.size() && i < widths.size(); i++)
            widths[i] = qMax(widths[i], row[i].size());
    }

    QString header_line;
    for(int i = 0; i < headers.size(); i++)
        header_line += headers[i].leftJustified(widths[i] + 2, ' ');
    out() << header_line.trimmed() << "\n";

    for(const QStringList &row : rows)
    {
        QString line;
        for(int i = 0; i < row.size(); i++)
            line += row[i].leftJustified(widths[i] + 2, ' ');
        out() << line.trimmed() << "\n";
    }
    out().flush();
}

// Each step of a dangerous action, one prompt at a time, exactly as the GUI
// puts up one message box per step.
bool confirmSteps(const QString &title, const QStringList &steps, bool assume_yes)
{
    if(assume_yes)
        return true;

    if(!stdinIsTty())
    {
        fail(QString("%1 needs %2 confirmations and stdin is not a terminal. "
                     "Pass --yes if you really mean it.")
                 .arg(title).arg(steps.size()));
        return false;
    }

    QTextStream in(stdin);
    for(int i = 0; i < steps.size(); i++)
    {
        out() << "\n--- " << title << ": confirmation " << (i + 1) << " of "
              << steps.size() << " ---\n" << steps[i] << "\n\nContinue? [y/N] ";
        out().flush();

        const QString answer = in.readLine().trimmed().toLower();
        if(answer != "y" && answer != "yes")
        {
            out() << "Cancelled.\n";
            out().flush();
            return false;
        }
    }
    return true;
}

// Opens the port, or explains why it could not.
std::unique_ptr<Session> openSession(const CommandLine &line)
{
    ConnectionOptions options;
    QString error;
    if(!connectionFrom(line, &options, &error))
    {
        fail(error);
        return nullptr;
    }

    auto session = std::make_unique<Session>(false);
    if(!session->open(options, &error))
    {
        fail(error);
        return nullptr;
    }

    if(line.flags.contains("verbose"))
    {
        err() << "Opened " << session->options().port << " at "
              << session->options().baud << " baud.\n";
        err().flush();
    }
    return session;
}

// Pings one servo so its series is known before anything else is addressed to
// it. Register addresses and wire endianness both depend on getting this right.
bool requireServo(Session &session, int id)
{
    if(id < 0 || id > 253)
    {
        fail(QString("Servo ID %1 is outside the range 0-253.").arg(id));
        return false;
    }

    const ServoEntry entry = session.probe(id);
    if(entry.id < 0)
    {
        fail(QString("No response from ID %1 on %2. Check the ID, the baud rate and the wiring.")
                 .arg(id).arg(session.options().port));
        return false;
    }

    session.addServo(entry);
    return true;
}

// The servos a whole-bus command should act on: the ones named by --ids, or
// everything a scan can find.
QVector<ServoEntry> gatherServos(Session &session, const CommandLine &line, bool *ok)
{
    *ok = true;

    if(line.options.contains("ids"))
    {
        bool parsed = false;
        const QVector<int> ids = parseIdList(line.value("ids"), &parsed);
        if(!parsed || ids.isEmpty())
        {
            fail("--ids takes a list like 1,2,3 or 1-6.");
            *ok = false;
            return {};
        }

        QVector<ServoEntry> found;
        for(int id : ids)
        {
            const ServoEntry entry = session.probe(id);
            if(entry.id < 0)
            {
                fail(QString("No response from ID %1.").arg(id));
                *ok = false;
                return {};
            }
            session.addServo(entry);
            found.append(entry);
        }
        return found;
    }

    const int from = line.intValue("from", 0);
    const int to = line.intValue("to", 253);
    const bool show_progress = isatty(STDERR_FILENO) == 1;

    const QVector<ServoEntry> found = session.scan(from, to, [&](int id) {
        if(show_progress)
        {
            err() << QString("\rPinging ID %1 ...   ").arg(id, 3);
            err().flush();
        }
        return g_interrupted == 0;
    });

    if(show_progress)
    {
        err() << "\r                        \r";
        err().flush();
    }

    session.setServos(found);
    return found;
}

QJsonObject statusToJson(const ServoStatus &status, ModelSeries series)
{
    QJsonObject object;
    object["id"] = status.id;
    object["ok"] = status.ok;
    object["position"] = status.pos;
    object["goal"] = status.goal;
    object["torque"] = status.torque;
    object["speed"] = status.speed;
    object["current"] = status.current;
    object["temperature"] = status.temp;
    object["voltage"] = status.voltage;
    object["volts"] = status.voltage / 10.0;
    object["moving"] = status.move;
    object["radians"] = countsToRadians(series, status.pos);
    object["degrees"] = countsToDegrees(series, status.pos);
    return object;
}

QString faultText(ModelSeries series, int fault)
{
    if(fault < 0)
        return "no reply";
    if(fault == 0)
        return "-";
    return decodeServoStatus(series, fault).join(", ");
}

}

bool interrupted()
{
    return g_interrupted != 0;
}

void installInterruptHandler()
{
    signal(SIGINT, onInterrupt);
    signal(SIGTERM, onInterrupt);
}

QString CommandLine::value(const QString &name, const QString &fallback) const
{
    return options.value(name, fallback);
}

int CommandLine::intValue(const QString &name, int fallback, bool *ok) const
{
    if(ok != nullptr)
        *ok = true;
    if(!options.contains(name))
        return fallback;

    bool parsed = false;
    const int value = options.value(name).toInt(&parsed);
    if(!parsed)
    {
        if(ok != nullptr)
            *ok = false;
        return fallback;
    }
    return value;
}

double CommandLine::doubleValue(const QString &name, double fallback, bool *ok) const
{
    if(ok != nullptr)
        *ok = true;
    if(!options.contains(name))
        return fallback;

    bool parsed = false;
    const double value = options.value(name).toDouble(&parsed);
    if(!parsed)
    {
        if(ok != nullptr)
            *ok = false;
        return fallback;
    }
    return value;
}

CommandLine parseCommandLine(const QStringList &args)
{
    CommandLine line;

    for(int i = 0; i < args.size(); i++)
    {
        QString token = args[i];

        if(token.startsWith("--") && token.size() > 2)
        {
            token = token.mid(2);
            QString name = token;
            QString value;
            bool has_inline = false;

            const int equals = token.indexOf('=');
            if(equals >= 0)
            {
                name = token.left(equals);
                value = token.mid(equals + 1);
                has_inline = true;
            }

            name = expandShort(name);

            if(booleanFlags().contains(name) && !has_inline)
            {
                line.flags.insert(name);
                continue;
            }
            if(has_inline)
            {
                line.options.insert(name, value);
                continue;
            }
            if(i + 1 >= args.size())
            {
                line.error = QString("--%1 needs a value.").arg(name);
                return line;
            }
            line.options.insert(name, args[++i]);
            continue;
        }

        if(token.startsWith('-') && token.size() > 1 && !token.at(1).isDigit())
        {
            const QString name = expandShort(token.mid(1));
            if(booleanFlags().contains(name))
            {
                line.flags.insert(name);
                continue;
            }
            if(i + 1 >= args.size())
            {
                line.error = QString("-%1 needs a value.").arg(token.mid(1));
                return line;
            }
            line.options.insert(name, args[++i]);
            continue;
        }

        if(line.command.isEmpty())
            line.command = token;
        else
            line.positional.append(token);
    }

    return line;
}

bool connectionFrom(const CommandLine &line, ConnectionOptions *out, QString *error)
{
    ConnectionOptions options;
    options.port = line.value("port");

    bool ok = false;
    options.baud = line.intValue("baud", 1000000, &ok);
    if(!ok)
    {
        *error = "--baud takes a number, for example 1000000.";
        return false;
    }

    options.timeout = line.intValue("timeout", 50, &ok);
    if(!ok)
    {
        *error = "--timeout takes a number of milliseconds.";
        return false;
    }

    const QString parity = line.value("parity", "none").toLower();
    if(parity == "none")
        options.parity = QSerialPort::NoParity;
    else if(parity == "odd")
        options.parity = QSerialPort::OddParity;
    else if(parity == "even")
        options.parity = QSerialPort::EvenParity;
    else
    {
        *error = "--parity takes none, odd or even.";
        return false;
    }

    *out = options;
    return true;
}

namespace
{

int runPorts(const CommandLine &line)
{
    const bool show_all = line.flags.contains("all");

    QVector<PortInfo> ports = availablePorts();
    if(show_all)
    {
        ports.clear();
        for(const QSerialPortInfo &info : QSerialPortInfo::availablePorts())
        {
            PortInfo entry;
            entry.name = info.portName();
            entry.description = info.description();
            entry.manufacturer = info.manufacturer();
            entry.has_vendor_id = info.hasVendorIdentifier();
            ports.append(entry);
        }
    }

    if(ports.isEmpty())
    {
        out() << "No serial ports found.\n";
        out().flush();
        return 1;
    }

    if(line.flags.contains("json"))
    {
        QJsonArray array;
        for(const PortInfo &port : ports)
        {
            QJsonObject object;
            object["name"] = port.name;
            object["device"] = "/dev/" + port.name;
            object["description"] = port.description;
            object["manufacturer"] = port.manufacturer;
            object["usb"] = port.has_vendor_id;
            array.append(object);
        }
        out() << QJsonDocument(array).toJson(QJsonDocument::Indented);
        out().flush();
        return 0;
    }

    QVector<QStringList> rows;
    for(const PortInfo &port : ports)
    {
        rows.append(QStringList()
                    << port.name
                    << (port.description.isEmpty() ? QString("-") : port.description)
                    << (port.manufacturer.isEmpty() ? QString("-") : port.manufacturer)
                    << (port.has_vendor_id ? "usb" : "-"));
    }
    printTable(QStringList() << "PORT" << "DESCRIPTION" << "MANUFACTURER" << "BUS", rows);

    if(!show_all)
        out() << "\nPorts without a USB vendor id are hidden. Pass --all to see them.\n";
    out().flush();
    return 0;
}

int runScan(const CommandLine &line)
{
    auto session = openSession(line);
    if(!session)
        return 1;

    bool ok = false;
    const QVector<ServoEntry> found = gatherServos(*session, line, &ok);
    if(!ok)
        return 1;

    if(found.isEmpty())
    {
        out() << "No servos answered on " << session->options().port << ".\n";
        out().flush();
        return 1;
    }

    const QVector<Session::ServoFlags> flags = session->readFlags(session->ids());

    if(line.flags.contains("json"))
    {
        QJsonArray array;
        for(int i = 0; i < found.size(); i++)
        {
            QJsonObject object;
            object["id"] = found[i].id;
            object["model"] = found[i].model;
            object["series"] = seriesName(found[i].series);
            object["counts_per_rev"] = feetech_servo::countsPerRev(found[i].series);
            if(i < flags.size())
            {
                object["torque"] = flags[i].torque;
                object["fault"] = flags[i].fault;
                QJsonArray faults;
                for(const QString &fault : decodeServoStatus(found[i].series, flags[i].fault))
                    faults.append(fault);
                object["faults"] = faults;
            }
            array.append(object);
        }
        out() << QJsonDocument(array).toJson(QJsonDocument::Indented);
        out().flush();
        return 0;
    }

    QVector<QStringList> rows;
    for(int i = 0; i < found.size(); i++)
    {
        const Session::ServoFlags servo_flags = i < flags.size() ? flags[i] : Session::ServoFlags();
        QString torque = "unknown";
        if(servo_flags.torque == 0)
            torque = "off";
        else if(servo_flags.torque > 0)
            torque = "on";

        rows.append(QStringList()
                    << QString::number(found[i].id)
                    << found[i].model
                    << seriesName(found[i].series)
                    << torque
                    << faultText(found[i].series, servo_flags.fault));
    }
    printTable(QStringList() << "ID" << "MODEL" << "SERIES" << "TORQUE" << "FAULTS", rows);
    out() << "\n" << found.size() << " servo(s) on " << session->options().port << ".\n";
    out().flush();
    return 0;
}

int runInfo(const CommandLine &line)
{
    if(line.positional.isEmpty())
    {
        fail("info needs a servo ID.");
        return 2;
    }

    auto session = openSession(line);
    if(!session)
        return 1;

    const int id = line.positional.first().toInt();
    if(!requireServo(*session, id))
        return 1;

    const ModelSeries series = session->seriesFor(id);
    const ServoStatus status = session->readStatus(id);
    const QVector<Session::ServoFlags> flags = session->readFlags({id});
    const Session::ServoFlags servo_flags = flags.isEmpty() ? Session::ServoFlags() : flags.first();

    // A register that the series does not have reads as absent rather than as
    // -1, so a potentiometer servo does not appear to have a broken offset.
    auto reg = [&](const char *name) -> std::optional<int> {
        const MemoryConfig *config = findMemConfig(series, name);
        if(config == nullptr)
            return std::nullopt;
        return session->readRegister(id, *config);
    };

    const std::optional<int> min_limit = reg("Min Position Limit");
    const std::optional<int> max_limit = reg("Max Position Limit");
    const std::optional<int> offset = reg("Position Offset Value");
    const std::optional<int> mode = reg("Work Mode");
    const std::optional<int> firmware_major = reg("Firmare Main Version NO.");
    const std::optional<int> firmware_minor = reg("Firmware Secondary Version NO.");

    if(line.flags.contains("json"))
    {
        QJsonObject object = statusToJson(status, series);
        object["model"] = session->modelFor(id);
        object["series"] = seriesName(series);
        object["counts_per_rev"] = feetech_servo::countsPerRev(series);
        object["torque_enable"] = servo_flags.torque;
        object["fault"] = servo_flags.fault;
        QJsonArray faults;
        for(const QString &fault : decodeServoStatus(series, servo_flags.fault))
            faults.append(fault);
        object["faults"] = faults;
        if(min_limit)
            object["min_position_limit"] = *min_limit;
        if(max_limit)
            object["max_position_limit"] = *max_limit;
        if(offset)
            object["position_offset"] = *offset;
        if(mode)
            object["work_mode"] = *mode;
        object["firmware"] = QString("%1.%2")
                                 .arg(firmware_major.value_or(-1))
                                 .arg(firmware_minor.value_or(-1));
        out() << QJsonDocument(object).toJson(QJsonDocument::Indented);
        out().flush();
        return 0;
    }

    out() << "ID              " << id << "\n"
          << "Model           " << session->modelFor(id) << " (" << seriesName(series) << ", "
          << feetech_servo::countsPerRev(series) << " counts/rev)\n"
          << "Firmware        " << firmware_major.value_or(-1) << "."
          << firmware_minor.value_or(-1) << "\n"
          << "Torque Enable   " << (servo_flags.torque < 0 ? QString("no reply")
                                                           : QString::number(servo_flags.torque))
          << "\n"
          << "Servo Status    " << faultText(series, servo_flags.fault) << "\n"
          << "Position        " << status.pos << "  ("
          << QString::number(countsToDegrees(series, status.pos), 'f', 1) << " deg, "
          << QString::number(countsToRadians(series, status.pos), 'f', 3) << " rad)\n"
          << "Goal            " << status.goal << "\n"
          << "Speed           " << status.speed << "\n"
          << "Load            " << status.torque << "\n"
          << "Current         " << status.current << "\n"
          << "Temperature     " << status.temp << " C\n"
          << "Voltage         " << QString::number(status.voltage / 10.0, 'f', 1) << " V\n"
          << "Moving          " << status.move << "\n";

    if(min_limit && max_limit)
        out() << "Position limits " << *min_limit << " .. " << *max_limit << "\n";
    if(offset)
        out() << "Position offset " << *offset << "\n";
    if(mode)
        out() << "Work Mode       " << *mode << "\n";
    out().flush();
    return 0;
}

int runStatus(const CommandLine &line)
{
    if(line.positional.isEmpty())
    {
        fail("status needs a servo ID.");
        return 2;
    }

    auto session = openSession(line);
    if(!session)
        return 1;

    const int id = line.positional.first().toInt();
    if(!requireServo(*session, id))
        return 1;

    const ModelSeries series = session->seriesFor(id);
    const ServoStatus status = session->readStatus(id);

    if(line.flags.contains("json"))
    {
        out() << QJsonDocument(statusToJson(status, series)).toJson(QJsonDocument::Indented);
        out().flush();
        return status.ok ? 0 : 1;
    }

    out() << "pos=" << status.pos
          << " goal=" << status.goal
          << " load=" << status.torque
          << " speed=" << status.speed
          << " current=" << status.current
          << " temp=" << status.temp
          << " volt=" << QString::number(status.voltage / 10.0, 'f', 1)
          << " moving=" << status.move
          << " deg=" << QString::number(countsToDegrees(series, status.pos), 'f', 1)
          << "\n";
    out().flush();
    return status.ok ? 0 : 1;
}

// Samples until the count, the duration or Ctrl-C says to stop, pacing itself
// so a slow bus stretches the interval instead of piling up requests.
int sampleLoop(Session &session, int id, double hz, int count, double duration_s,
               const std::function<void(const ServoStatus &, double)> &sink)
{
    const double interval_ms = hz > 0 ? 1000.0 / hz : 0.0;
    QElapsedTimer clock;
    clock.start();

    int taken = 0;
    while(!interrupted())
    {
        const qint64 started = clock.elapsed();
        const ServoStatus status = session.readStatus(id);
        sink(status, clock.elapsed() / 1000.0);
        taken++;

        if(count > 0 && taken >= count)
            break;
        if(duration_s > 0 && clock.elapsed() / 1000.0 >= duration_s)
            break;

        const qint64 spent = clock.elapsed() - started;
        const qint64 wait = static_cast<qint64>(interval_ms) - spent;
        if(wait > 0)
            QThread::msleep(static_cast<unsigned long>(wait));
    }
    return taken;
}

int runMonitor(const CommandLine &line)
{
    if(line.positional.isEmpty())
    {
        fail("monitor needs a servo ID.");
        return 2;
    }

    auto session = openSession(line);
    if(!session)
        return 1;

    const int id = line.positional.first().toInt();
    if(!requireServo(*session, id))
        return 1;

    const ModelSeries series = session->seriesFor(id);
    const double hz = line.doubleValue("hz", 10.0);
    const int count = line.intValue("count", 0);
    const double duration = line.doubleValue("duration", 0.0);
    const bool as_json = line.flags.contains("json");
    const bool as_csv = line.flags.contains("csv");

    if(as_csv)
        out() << "t,pos,goal,load,speed,current,temp,volt,moving\n";

    sampleLoop(*session, id, hz, count, duration, [&](const ServoStatus &status, double t) {
        if(as_json)
        {
            QJsonObject object = statusToJson(status, series);
            object["t"] = t;
            out() << QJsonDocument(object).toJson(QJsonDocument::Compact) << "\n";
        }
        else if(as_csv)
        {
            out() << QString::number(t, 'f', 3) << "," << status.pos << "," << status.goal << ","
                  << status.torque << "," << status.speed << "," << status.current << ","
                  << status.temp << "," << status.voltage << "," << status.move << "\n";
        }
        else
        {
            out() << QString("%1s  pos %2  goal %3  load %4  speed %5  cur %6  %7C  %8V  mov %9\n")
                         .arg(t, 7, 'f', 2)
                         .arg(status.pos, 5)
                         .arg(status.goal, 5)
                         .arg(status.torque, 5)
                         .arg(status.speed, 5)
                         .arg(status.current, 5)
                         .arg(status.temp, 3)
                         .arg(status.voltage / 10.0, 5, 'f', 1)
                         .arg(status.move);
        }
        out().flush();
    });

    return 0;
}

int runRecord(const CommandLine &line)
{
    if(line.positional.isEmpty())
    {
        fail("record needs a servo ID.");
        return 2;
    }

    const QString path = line.value("file", "~/record.txt");

    auto session = openSession(line);
    if(!session)
        return 1;

    const int id = line.positional.first().toInt();
    if(!requireServo(*session, id))
        return 1;

    Recorder recorder;
    QString error;
    if(!recorder.start(path, line.intValue("interval", 30), &error))
    {
        fail(error);
        return 1;
    }

    const double hz = line.doubleValue("hz", 20.0);
    const int count = line.intValue("count", 0);
    const double duration = line.doubleValue("duration", 0.0);
    const bool quiet = line.flags.contains("quiet");

    if(!quiet)
    {
        err() << "Recording ID " << id << " to " << recorder.path()
              << " at " << hz << " Hz. Ctrl-C to stop.\n";
        err().flush();
    }

    sampleLoop(*session, id, hz, count, duration, [&](const ServoStatus &status, double) {
        recorder.append(status);
        if(!quiet && isatty(STDERR_FILENO) == 1 && recorder.rows() % 10 == 0)
        {
            err() << QString("\r%1 rows").arg(recorder.rows());
            err().flush();
        }
    });

    recorder.stop();
    if(!quiet)
    {
        err() << "\r";
        err().flush();
    }
    out() << recorder.rows() << " rows written to " << recorder.path() << "\n";
    out().flush();
    return 0;
}

int runRead(const CommandLine &line)
{
    if(line.positional.size() < 2)
    {
        fail("read needs a servo ID and a register.");
        return 2;
    }

    auto session = openSession(line);
    if(!session)
        return 1;

    const int id = line.positional[0].toInt();
    if(!requireServo(*session, id))
        return 1;

    const ModelSeries series = session->seriesFor(id);
    QString error;
    const MemoryConfig *config = findRegister(series, line.positional[1], &error);
    if(config == nullptr)
    {
        fail(error);
        return 2;
    }

    const std::optional<int> value = session->readRegister(id, *config);
    if(!value.has_value())
    {
        fail(QString("No reply for address %1 (%2).").arg(config->address).arg(config->name));
        return 1;
    }

    if(line.flags.contains("json"))
    {
        QJsonObject object;
        object["id"] = id;
        object["address"] = config->address;
        object["name"] = config->name;
        object["value"] = *value;
        object["area"] = registerArea(*config);
        object["writable"] = !config->is_readonly;
        out() << QJsonDocument(object).toJson(QJsonDocument::Indented);
    }
    else if(line.flags.contains("quiet"))
    {
        out() << *value << "\n";
    }
    else
    {
        out() << QString("[%1] %2 = %3  (%4, %5)\n")
                     .arg(config->address)
                     .arg(config->name)
                     .arg(*value)
                     .arg(registerArea(*config))
                     .arg(config->is_readonly ? "read-only" : "read/write");
    }
    out().flush();
    return 0;
}

int runWrite(const CommandLine &line)
{
    if(line.positional.size() < 3)
    {
        fail("write needs a servo ID, a register and a value.");
        return 2;
    }

    auto session = openSession(line);
    if(!session)
        return 1;

    const int id = line.positional[0].toInt();
    if(!requireServo(*session, id))
        return 1;

    const ModelSeries series = session->seriesFor(id);
    QString error;
    const MemoryConfig *config = findRegister(series, line.positional[1], &error);
    if(config == nullptr)
    {
        fail(error);
        return 2;
    }

    if(config->is_readonly)
    {
        fail(QString("[%1] %2 is read-only.").arg(config->address).arg(config->name));
        return 2;
    }

    bool parsed = false;
    const int value = line.positional[2].toInt(&parsed);
    if(!parsed)
    {
        fail(QString("\"%1\" is not a number.").arg(line.positional[2]));
        return 2;
    }

    if(config->min_val != -1 || config->max_val != -1)
    {
        if(value < config->min_val || value > config->max_val)
        {
            fail(QString("%1 is outside the documented range %2..%3 for [%4] %5.")
                     .arg(value).arg(config->min_val).arg(config->max_val)
                     .arg(config->address).arg(config->name));
            return 2;
        }
    }

    // Writing the ID register is the one write that moves the servo to a new
    // address part way through, so it is worth a confirmation of its own.
    if(config->address == 5 && !line.flags.contains("yes"))
    {
        const QStringList steps = {
            QString("About to change the ID of servo %1 to %2.\n\n"
                    "This is an EPROM write, so it is permanent, and every later command "
                    "has to address the servo as ID %2.")
                .arg(id).arg(value)
        };
        if(!confirmSteps("Change ID", steps, false))
            return 1;
    }

    const RegisterWriteResult result = session->writeRegister(id, *config, value);

    if(!result.ok)
    {
        fail(QString("[%1] %2 still does not hold %3 after the write. The servo either did not "
                     "respond or rejected the value (out of range, or outside the %4 area).")
                 .arg(config->address).arg(config->name).arg(value).arg(registerArea(*config)));
        return 1;
    }

    out() << QString("[%1] %2 = %3 saved to %4")
                 .arg(config->address).arg(config->name).arg(value).arg(registerArea(*config));
    if(result.effective_id != id)
        out() << QString(" (servo is now ID %1)").arg(result.effective_id);
    out() << "\n";
    out().flush();
    return 0;
}

int runDump(const CommandLine &line)
{
    if(line.positional.isEmpty())
    {
        fail("dump needs a servo ID.");
        return 2;
    }

    auto session = openSession(line);
    if(!session)
        return 1;

    const int id = line.positional.first().toInt();
    if(!requireServo(*session, id))
        return 1;

    const ModelSeries series = session->seriesFor(id);
    const auto &configs = getMemConfig(series);

    QVector<std::optional<int>> values;
    values.reserve(static_cast<int>(configs.size()));
    for(const MemoryConfig &config : configs)
        values.append(session->readRegister(id, config));

    if(line.flags.contains("json"))
    {
        QJsonArray array;
        for(std::size_t i = 0; i < configs.size(); i++)
        {
            QJsonObject object;
            object["address"] = configs[i].address;
            object["name"] = configs[i].name;
            const std::optional<int> value = values[static_cast<int>(i)];
            if(value.has_value())
                object["value"] = *value;
            object["area"] = registerArea(configs[i]);
            object["writable"] = !configs[i].is_readonly;
            array.append(object);
        }
        out() << QJsonDocument(array).toJson(QJsonDocument::Indented);
        out().flush();
        return 0;
    }

    QVector<QStringList> rows;
    for(std::size_t i = 0; i < configs.size(); i++)
    {
        const std::optional<int> value = values[static_cast<int>(i)];
        rows.append(QStringList()
                    << QString::number(configs[i].address)
                    << configs[i].name
                    << (value.has_value() ? QString::number(*value) : QString("-"))
                    << registerArea(configs[i])
                    << (configs[i].is_readonly ? "R" : "R/W"));
    }
    printTable(QStringList() << "ADDR" << "REGISTER" << "VALUE" << "AREA" << "ACCESS", rows);
    out().flush();
    return 0;
}

int runRegsSave(Session &session, int id, const QString &path)
{
    const ModelSeries series = session.seriesFor(id);
    const auto &configs = getMemConfig(series);

    RegisterSnapshot snapshot;
    snapshot.id = id;
    snapshot.series = seriesName(series);
    snapshot.model = session.modelFor(id);

    QStringList unread;
    for(const MemoryConfig &config : configs)
    {
        const std::optional<int> value = session.readRegister(id, config);
        if(!value.has_value())
        {
            unread << QString::number(config.address);
            continue;
        }

        RegisterEntry entry;
        entry.address = config.address;
        entry.name = config.name;
        entry.writable = !config.is_readonly;
        entry.value = *value;
        snapshot.registers.append(entry);
    }

    if(snapshot.registers.isEmpty())
    {
        fail("No registers could be read from the servo.");
        return 1;
    }

    QFile file(expandHome(path));
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        fail(QString("Could not write %1.").arg(path));
        return 1;
    }
    file.write(buildRegisterSnapshotJson(snapshot));
    file.close();

    out() << QString("Saved %1 register(s) from ID %2 to %3")
                 .arg(snapshot.registers.size()).arg(id).arg(expandHome(path));
    if(!unread.isEmpty())
        out() << QString(" (no reply for address %1)").arg(unread.join(", "));
    out() << "\n";
    out().flush();
    return 0;
}

int runRegsLoad(Session &session, const CommandLine &line, int id, const QString &path)
{
    QFile file(expandHome(path));
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        fail(QString("Could not read %1.").arg(path));
        return 1;
    }
    const QByteArray data = file.readAll();
    file.close();

    RegisterSnapshot snapshot;
    QString error;
    if(!parseRegisterSnapshotJson(data, &snapshot, &error))
    {
        fail(QString("%1: %2").arg(path, error));
        return 1;
    }

    const ModelSeries series = session.seriesFor(id);
    const QString target_series = seriesName(series);
    if(snapshot.series != target_series)
    {
        fail(QString("This file was saved from a %1 servo but ID %2 is %3. Register addresses "
                     "differ between series, so loading it would write the wrong registers.")
                 .arg(snapshot.series).arg(id).arg(target_series));
        return 1;
    }

    const auto &configs = getMemConfig(series);
    QVector<QPair<const MemoryConfig *, int>> planned;
    QStringList skipped_readonly;
    QStringList skipped_unknown;
    bool skipped_id = false;

    for(const RegisterEntry &entry : snapshot.registers)
    {
        const MemoryConfig *config = nullptr;
        for(const MemoryConfig &item : configs)
        {
            if(item.address == entry.address)
            {
                config = &item;
                break;
            }
        }

        if(config == nullptr)
        {
            skipped_unknown << QString::number(entry.address);
            continue;
        }
        if(config->is_readonly)
        {
            skipped_readonly << QString::number(entry.address);
            continue;
        }
        if(config->address == 5)
        {
            skipped_id = true;
            continue;
        }
        planned.append(qMakePair(config, entry.value));
    }

    if(planned.isEmpty())
    {
        fail("Nothing in this file is writable to the servo.");
        return 1;
    }

    int eprom_count = 0;
    for(const auto &item : planned)
    {
        if(item.first->is_eprom)
            eprom_count++;
    }

    QStringList steps;
    steps << QString("About to write %1 register(s) to servo ID %2 from:\n%3\n\n"
                     "%4 of them are EPROM registers, so those changes are permanent.")
                 .arg(planned.size()).arg(id).arg(expandHome(path)).arg(eprom_count);
    steps << QString("This replaces the servo's current configuration and cannot be undone.\n\n"
                     "Position Offset Value, Min/Max Position Limit, PID gains, torque limits and "
                     "protection settings will all be overwritten by the values in the file. "
                     "Save the current registers first if you might want them back.");
    steps << QString("Final confirmation.\n\n"
                     "Target: ID %1 (%2)\nSource file series: %3\nRegisters to write: %4\n\n"
                     "The servo ID register is never written by a load, so ID %1 stays as it is.")
                 .arg(id).arg(target_series).arg(snapshot.series).arg(planned.size());

    if(!confirmSteps("Load Registers", steps, line.flags.contains("yes")))
        return 1;

    QStringList failed;
    for(const auto &item : planned)
    {
        if(!session.writeRegister(id, *item.first, item.second).ok)
            failed << QString::number(item.first->address);
    }

    QStringList notes;
    notes << QString("wrote %1").arg(planned.size() - failed.size());
    if(!failed.isEmpty())
        notes << QString("failed %1").arg(failed.size());
    if(!skipped_readonly.isEmpty())
        notes << QString("skipped %1 read-only").arg(skipped_readonly.size());
    if(!skipped_unknown.isEmpty())
        notes << QString("skipped %1 unknown").arg(skipped_unknown.size());
    if(skipped_id)
        notes << "skipped ID";

    out() << "Load: " << notes.join(", ") << "\n";
    if(!failed.isEmpty())
        out() << "These addresses did not verify after writing: " << failed.join(", ") << "\n";
    out().flush();
    return failed.isEmpty() ? 0 : 1;
}

int runRegs(const CommandLine &line)
{
    if(line.positional.size() < 3)
    {
        fail("regs takes save or load, a servo ID and a file.");
        return 2;
    }

    const QString action = line.positional[0].toLower();
    if(action != "save" && action != "load")
    {
        fail("regs takes save or load.");
        return 2;
    }

    auto session = openSession(line);
    if(!session)
        return 1;

    const int id = line.positional[1].toInt();
    if(!requireServo(*session, id))
        return 1;

    return action == "save" ? runRegsSave(*session, id, line.positional[2])
                            : runRegsLoad(*session, line, id, line.positional[2]);
}

int runTorque(const CommandLine &line)
{
    if(line.positional.size() < 2)
    {
        fail("torque needs a servo ID (or \"all\") and on/off.");
        return 2;
    }

    const QString state = line.positional[1].toLower();
    if(state != "on" && state != "off")
    {
        fail("torque takes on or off.");
        return 2;
    }
    const bool on = state == "on";

    auto session = openSession(line);
    if(!session)
        return 1;

    QVector<int> targets;
    if(line.positional[0].toLower() == "all")
    {
        bool ok = false;
        const QVector<ServoEntry> found = gatherServos(*session, line, &ok);
        if(!ok)
            return 1;
        if(found.isEmpty())
        {
            fail("No servos detected.");
            return 1;
        }
        targets = session->ids();
    }
    else
    {
        const int id = line.positional[0].toInt();
        if(!requireServo(*session, id))
            return 1;
        targets.append(id);
    }

    const QStringList failed = session->setTorque(targets, on);

    if(failed.isEmpty())
    {
        out() << QString("Torque %1 on %2 servo(s).\n").arg(state).arg(targets.size());
        out().flush();
        return 0;
    }

    out() << QString("Torque %1: %2 of %3 failed.\n").arg(state).arg(failed.size()).arg(targets.size());
    fail(QString("ID %1 did not report Torque Enable = %2. Those servos may still be in the "
                 "previous state.").arg(failed.join(", ")).arg(on ? 1 : 0));
    return 1;
}

WriteMode modeFrom(const CommandLine &line, bool *ok)
{
    *ok = true;
    const QString mode = line.value("mode", "write").toLower();
    if(mode == "write")
        return WriteMode::Write;
    if(mode == "reg" || mode == "regwrite" || mode == "reg-write")
        return WriteMode::RegWrite;
    if(mode == "sync" || mode == "syncwrite" || mode == "sync-write")
        return WriteMode::SyncWrite;

    *ok = false;
    return WriteMode::Write;
}

// Waits for Moving to clear, which is how the servo says it has arrived.
void waitForArrival(Session &session, int id, int timeout_ms)
{
    QElapsedTimer clock;
    clock.start();
    while(clock.elapsed() < timeout_ms && !interrupted())
    {
        const MemoryConfig *config = findMemConfig(session.seriesFor(id), "Moving Status");
        if(config == nullptr)
            return;
        if(session.readRegister(id, *config).value_or(-1) == 0)
            return;
        QThread::msleep(20);
    }
}

int commandPositionCommon(Session &session, const CommandLine &line, int id, int goal)
{
    const int max_count = session.countsPerRev(id) - 1;
    if(goal < 0 || goal > max_count)
    {
        fail(QString("Position %1 is outside the encoder range 0..%2.").arg(goal).arg(max_count));
        return 2;
    }

    bool mode_ok = false;
    const WriteMode mode = modeFrom(line, &mode_ok);
    if(!mode_ok)
    {
        fail("--mode takes write, reg or sync.");
        return 2;
    }

    const int speed = line.intValue("speed", 0);
    const int acc = line.intValue("acc", 0);
    const int time = line.intValue("time", 0);

    session.commandPosition(id, mode, goal, time, speed, acc);

    if(mode == WriteMode::RegWrite && !line.flags.contains("quiet"))
    {
        out() << QString("Queued %1 on ID %2. Send \"action\" to run it.\n").arg(goal).arg(id);
        out().flush();
        return 0;
    }

    if(line.flags.contains("wait"))
        waitForArrival(session, id, line.intValue("wait-timeout", 5000));

    if(!line.flags.contains("quiet"))
    {
        const ServoStatus status = session.readStatus(id);
        out() << QString("ID %1 goal %2, now at %3 (%4 deg)\n")
                     .arg(id).arg(goal).arg(status.pos)
                     .arg(countsToDegrees(session.seriesFor(id), status.pos), 0, 'f', 1);
        out().flush();
    }
    return 0;
}

int runPos(const CommandLine &line)
{
    if(line.positional.size() < 2)
    {
        fail("pos needs a servo ID and a position in counts.");
        return 2;
    }

    auto session = openSession(line);
    if(!session)
        return 1;

    const int id = line.positional[0].toInt();
    if(!requireServo(*session, id))
        return 1;

    bool parsed = false;
    const int goal = line.positional[1].toInt(&parsed);
    if(!parsed)
    {
        fail(QString("\"%1\" is not a position.").arg(line.positional[1]));
        return 2;
    }

    return commandPositionCommon(*session, line, id, goal);
}

int runAngle(const CommandLine &line)
{
    if(line.positional.size() < 2)
    {
        fail("angle needs a servo ID and an angle.");
        return 2;
    }

    auto session = openSession(line);
    if(!session)
        return 1;

    const int id = line.positional[0].toInt();
    if(!requireServo(*session, id))
        return 1;

    bool parsed = false;
    const double angle = line.positional[1].toDouble(&parsed);
    if(!parsed)
    {
        fail(QString("\"%1\" is not an angle.").arg(line.positional[1]));
        return 2;
    }

    const QString unit = line.value("unit", "deg").toLower();
    if(unit != "deg" && unit != "rad")
    {
        fail("--unit takes deg or rad.");
        return 2;
    }

    const ModelSeries series = session->seriesFor(id);
    const int goal = angleToCounts(series, angle, unit == "rad");
    const int max_count = feetech_servo::countsPerRev(series) - 1;
    if(goal < 0 || goal > max_count)
    {
        fail(QString("%1 %2 maps to position %3, which is outside the encoder range 0..%4.")
                 .arg(angle).arg(unit).arg(goal).arg(max_count));
        return 2;
    }

    return commandPositionCommon(*session, line, id, goal);
}

int runAction(const CommandLine &line)
{
    auto session = openSession(line);
    if(!session)
        return 1;

    // No ID means the broadcast address, which is how one Action starts every
    // servo that has a queued REG WRITE.
    int id = 0xfe;
    if(!line.positional.isEmpty())
    {
        id = line.positional.first().toInt();
        if(!requireServo(*session, id))
            return 1;
    }

    session->regWriteAction(id);
    if(!line.flags.contains("quiet"))
    {
        out() << (id == 0xfe ? QString("Action broadcast.\n")
                             : QString("Action sent to ID %1.\n").arg(id));
        out().flush();
    }
    return 0;
}

int runMidpoint(const CommandLine &line)
{
    if(line.positional.isEmpty())
    {
        fail("midpoint needs a servo ID.");
        return 2;
    }

    auto session = openSession(line);
    if(!session)
        return 1;

    const int id = line.positional.first().toInt();
    if(!requireServo(*session, id))
        return 1;

    const ModelSeries series = session->seriesFor(id);
    const MemoryConfig *offset = findMemConfig(series, "Position Offset Value");
    if(!supportsMidpointCalibration(series) || offset == nullptr)
    {
        fail("This servo series has no Position Offset Value register.");
        return 1;
    }

    const QStringList steps = {
        QString("Store the current position of servo ID %1 as the midpoint (%2)?\n\n"
                "This overwrites Position Offset Value (address %3) in EPROM, so it survives a "
                "power cycle. The servo does not move.")
            .arg(id).arg(feetech_servo::countsPerRev(series) / 2).arg(offset->address)
    };
    if(!confirmSteps("Set Midpoint", steps, line.flags.contains("yes")))
        return 1;

    const MidpointResult result = session->setMidpoint(id, *offset);
    if(!result.acked || result.after < 0)
    {
        fail("No response from the servo. The midpoint was not changed.");
        return 1;
    }

    const int decoded = feetech_servo::decodeSignMagnitude(result.after, offset->dir_bit);
    out() << QString("Midpoint set: offset = %1\n").arg(decoded);
    if(result.before >= 0 && result.after == result.before)
    {
        out() << "Position Offset Value is unchanged. Either the servo was already centred "
                 "here, or the firmware rejected the calibration command.\n";
    }
    out().flush();
    return 0;
}

int runSweepOrStep(const CommandLine &line, bool stepping)
{
    const QString name = stepping ? "step" : "sweep";
    if(line.positional.isEmpty())
    {
        fail(QString("%1 needs a servo ID.").arg(name));
        return 2;
    }

    auto session = openSession(line);
    if(!session)
        return 1;

    const int id = line.positional.first().toInt();
    if(!requireServo(*session, id))
        return 1;

    const int max_count = session->countsPerRev(id) - 1;
    const int start = qBound(0, line.intValue("start", 0), max_count);
    const int end = qBound(0, line.intValue("end", max_count), max_count);
    const int hold = qMax(1, line.intValue("hold", 2500));
    const int size = qMax(1, line.intValue("size", 10));
    const int delay = qMax(1, line.intValue("delay", 10));
    const int cycles = line.intValue("cycles", 0);
    const int speed = line.intValue("speed", 0);
    const int acc = line.intValue("acc", 0);

    if(start >= end)
    {
        fail("--start must be below --end.");
        return 2;
    }

    if(!line.flags.contains("quiet"))
    {
        err() << QString("%1 ID %2 between %3 and %4. Ctrl-C to stop.\n")
                     .arg(name).arg(id).arg(start).arg(end);
        err().flush();
    }

    int goal = start;
    bool increasing = true;
    int completed = 0;
    session->commandPosition(id, WriteMode::Write, goal, 0, speed, acc);

    while(!interrupted())
    {
        QThread::msleep(static_cast<unsigned long>(stepping ? delay : hold));
        if(interrupted())
            break;

        if(!stepping)
        {
            goal = (goal == start) ? end : start;
            if(goal == start)
                completed++;
        }
        else
        {
            goal += increasing ? size : -size;
            if(goal > end)
            {
                goal = end;
                increasing = false;
            }
            else if(goal < start)
            {
                goal = start;
                increasing = true;
                completed++;
            }
        }

        session->commandPosition(id, WriteMode::Write, goal, 0, speed, acc);

        if(cycles > 0 && completed >= cycles)
            break;
    }

    if(!line.flags.contains("quiet"))
    {
        out() << QString("Stopped after %1 cycle(s) at %2.\n").arg(completed).arg(goal);
        out().flush();
    }
    return 0;
}

QString calibStatePath(const CommandLine &line)
{
    return expandHome(line.value("state", defaultCalibStatePath()));
}

int runCalibHome(const CommandLine &line)
{
    auto session = openSession(line);
    if(!session)
        return 1;

    bool ok = false;
    const QVector<ServoEntry> found = gatherServos(*session, line, &ok);
    if(!ok)
        return 1;
    if(found.isEmpty())
    {
        fail("No servos detected.");
        return 1;
    }

    QVector<int> ids = session->ids();
    std::sort(ids.begin(), ids.end());

    // Calibration is gated on torque being released, exactly as the GUI gates
    // it: a joint that is holding position cannot be posed by hand.
    const QVector<Session::ServoFlags> flags = session->readFlags(ids);
    QStringList torque_on;
    QStringList silent;
    for(int i = 0; i < ids.size() && i < flags.size(); i++)
    {
        if(flags[i].torque < 0)
            silent << QString::number(ids[i]);
        else if(flags[i].torque != 0)
            torque_on << QString::number(ids[i]);
    }

    if(!silent.isEmpty())
    {
        fail(QString("No response from ID %1.").arg(silent.join(", ")));
        return 1;
    }
    if(!torque_on.isEmpty())
    {
        fail(QString("Torque is still enabled on ID %1. Release it first: "
                     "servobench-cli torque all off").arg(torque_on.join(", ")));
        return 1;
    }

    QStringList affected;
    QStringList positions;
    for(int id : ids)
    {
        affected << QString::number(id);
        const auto position = session->readPosition(id);
        positions << QString("    ID %1: %2")
                         .arg(id)
                         .arg(position.has_value() ? QString::number(*position) : QString("no reply"));
    }

    const QStringList steps = buildSetHomeConfirmations(ids.size(), affected.join(", "),
                                                        positions.join("\n"));
    if(!confirmSteps("Set Home", steps, line.flags.contains("yes")))
        return 1;

    const QVector<Session::HomeResult> results = session->setHome(ids);

    CalibState state;
    QString error;
    loadCalibState(calibStatePath(line), &state, &error);
    state.port = session->options().port;

    QStringList failed;
    QStringList mismatched;
    QVector<QStringList> rows;

    for(const Session::HomeResult &result : results)
    {
        if(!result.written)
        {
            failed << QString::number(result.id);
            continue;
        }

        if(result.corrected < 0 || std::abs(result.corrected - result.half_turn) > 2)
        {
            mismatched << QString("ID %1 reads %2, expected %3")
                              .arg(result.id)
                              .arg(result.corrected >= 0 ? QString::number(result.corrected)
                                                         : QString("no reply"))
                              .arg(result.half_turn);
        }

        JointCalibration *joint = state.find(result.id);
        if(joint == nullptr)
        {
            JointCalibration fresh;
            fresh.id = result.id;
            state.joints.append(fresh);
            joint = state.find(result.id);
        }
        joint->homing_offset = result.homing_offset;
        joint->range_min = result.half_turn;
        joint->range_max = result.half_turn;

        rows.append(QStringList()
                    << QString::number(result.id)
                    << QString::number(result.homing_offset)
                    << QString::number(result.corrected));
    }

    std::sort(state.joints.begin(), state.joints.end(),
              [](const JointCalibration &a, const JointCalibration &b) { return a.id < b.id; });

    if(!saveCalibState(calibStatePath(line), state, &error))
        fail(error);

    if(!rows.isEmpty())
        printTable(QStringList() << "ID" << "HOMING_OFFSET" << "READS", rows);

    if(!failed.isEmpty())
    {
        fail(QString("Could not write calibration to ID %1.").arg(failed.join(", ")));
        return 1;
    }
    if(!mismatched.isEmpty())
    {
        fail(QString("Offset written, but the servo does not report the midpoint:\n%1\n"
                     "The firmware may apply the offset with the opposite sign.")
                 .arg(mismatched.join("\n")));
        return 1;
    }

    out() << QString("Home set on %1 servo(s). State in %2.\n"
                     "Next: servobench-cli calib record\n")
                 .arg(results.size()).arg(calibStatePath(line));
    out().flush();
    return 0;
}

int runCalibRecord(const CommandLine &line)
{
    const QString path = calibStatePath(line);

    CalibState state;
    QString error;
    if(!loadCalibState(path, &state, &error))
    {
        fail(error);
        return 1;
    }
    if(state.joints.isEmpty())
    {
        fail("No joints in the calibration state. Run \"calib home\" first.");
        return 1;
    }

    auto session = openSession(line);
    if(!session)
        return 1;

    for(JointCalibration &joint : state.joints)
    {
        const ServoEntry entry = session->probe(joint.id);
        if(entry.id < 0)
        {
            fail(QString("No response from ID %1.").arg(joint.id));
            return 1;
        }
        session->addServo(entry);

        const auto position = session->readPosition(joint.id);
        if(!position.has_value())
        {
            fail(QString("No response from ID %1.").arg(joint.id));
            return 1;
        }
        joint.range_min = *position;
        joint.range_max = *position;
    }

    const double duration = line.doubleValue("duration", 0.0);
    err() << "Recording range. Move every joint slowly through its full travel.\n"
          << (duration > 0 ? QString("Stopping after %1 s.\n").arg(duration)
                           : QString("Press Ctrl-C when done.\n"));
    err().flush();

    QElapsedTimer clock;
    clock.start();
    const bool show_progress = isatty(STDERR_FILENO) == 1;

    while(!interrupted())
    {
        for(JointCalibration &joint : state.joints)
        {
            const auto position = session->readPosition(joint.id);
            if(!position.has_value())
                continue;
            joint.range_min = qMin(joint.range_min, *position);
            joint.range_max = qMax(joint.range_max, *position);
        }

        if(show_progress)
        {
            QStringList parts;
            for(const JointCalibration &joint : state.joints)
                parts << QString("%1:%2..%3").arg(joint.id).arg(joint.range_min).arg(joint.range_max);
            err() << "\r" << parts.join("  ") << "   ";
            err().flush();
        }

        if(duration > 0 && clock.elapsed() / 1000.0 >= duration)
            break;

        QThread::msleep(50);
    }

    if(show_progress)
    {
        err() << "\n";
        err().flush();
    }

    if(!saveCalibState(path, state, &error))
    {
        fail(error);
        return 1;
    }

    QVector<QStringList> rows;
    for(const JointCalibration &joint : state.joints)
    {
        rows.append(QStringList()
                    << QString::number(joint.id)
                    << (joint.name.isEmpty() ? QString("-") : joint.name)
                    << QString::number(joint.homing_offset)
                    << QString::number(joint.range_min)
                    << QString::number(joint.range_max));
    }
    printTable(QStringList() << "ID" << "NAME" << "HOMING_OFFSET" << "RANGE_MIN" << "RANGE_MAX", rows);
    out() << "\nNext: servobench-cli calib export calibration.json --names a,b,c\n";
    out().flush();
    return 0;
}

int applyNames(CalibState *state, const QString &text)
{
    const QStringList names = splitFields(text, QRegExp("[,;\n\t]"));
    const int applied = qMin(state->joints.size(), names.size());
    for(int i = 0; i < applied; i++)
        state->joints[i].name = names[i];
    return applied;
}

int runCalibNames(const CommandLine &line)
{
    const QString path = calibStatePath(line);

    CalibState state;
    QString error;
    if(!loadCalibState(path, &state, &error))
    {
        fail(error);
        return 1;
    }

    const QString text = line.positional.isEmpty() ? line.value("names") : line.positional.join(",");
    if(text.isEmpty())
    {
        fail("calib names takes a comma separated list, in ascending servo ID order.");
        return 2;
    }

    const int applied = applyNames(&state, text);
    if(!saveCalibState(path, state, &error))
    {
        fail(error);
        return 1;
    }

    out() << QString("Applied %1 of %2 joint name(s), in servo ID order.\n")
                 .arg(applied).arg(state.joints.size());
    out().flush();
    return applied == state.joints.size() ? 0 : 1;
}

int runCalibShow(const CommandLine &line)
{
    const QString path = calibStatePath(line);

    CalibState state;
    QString error;
    if(!loadCalibState(path, &state, &error))
    {
        fail(error);
        return 1;
    }

    out() << "State file: " << path << "\n";
    if(!state.port.isEmpty())
        out() << "Recorded on: " << state.port << " at " << state.updated << "\n\n";

    QVector<QStringList> rows;
    for(const JointCalibration &joint : state.joints)
    {
        rows.append(QStringList()
                    << QString::number(joint.id)
                    << (joint.name.isEmpty() ? QString("-") : joint.name)
                    << QString::number(joint.drive_mode)
                    << QString::number(joint.homing_offset)
                    << QString::number(joint.range_min)
                    << QString::number(joint.range_max));
    }
    printTable(QStringList() << "ID" << "NAME" << "DRIVE_MODE" << "HOMING_OFFSET"
                             << "RANGE_MIN" << "RANGE_MAX", rows);
    out().flush();
    return 0;
}

int runCalibExport(const CommandLine &line)
{
    if(line.positional.isEmpty())
    {
        fail("calib export needs an output file.");
        return 2;
    }

    const QString path = calibStatePath(line);
    CalibState state;
    QString error;
    if(!loadCalibState(path, &state, &error))
    {
        fail(error);
        return 1;
    }

    if(line.options.contains("names"))
        applyNames(&state, line.value("names"));

    QVector<JointCalibration> joints = state.joints;
    const QStringList problems = validateCalibrationJoints(joints);
    if(!problems.isEmpty())
    {
        fail(QString("Fix these before exporting:\n%1").arg(problems.join("\n")));
        return 1;
    }

    const QString target = expandHome(line.positional.first());
    QFile file(target);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        fail(QString("Could not write %1.").arg(target));
        return 1;
    }

    QTextStream stream(&file);
    stream << buildCalibrationJson(joints);
    file.close();

    if(line.options.contains("names") && !saveCalibState(path, state, &error))
        fail(error);

    out() << QString("Exported %1 joint(s) to %2\n").arg(joints.size()).arg(target);
    out().flush();
    return 0;
}

int runCalib(const CommandLine &line)
{
    if(line.positional.isEmpty())
    {
        fail("calib takes home, record, names, show or export.");
        return 2;
    }

    CommandLine inner = line;
    const QString action = inner.positional.takeFirst().toLower();

    if(action == "home")
        return runCalibHome(inner);
    if(action == "record")
        return runCalibRecord(inner);
    if(action == "names")
        return runCalibNames(inner);
    if(action == "show")
        return runCalibShow(inner);
    if(action == "export")
        return runCalibExport(inner);

    fail(QString("Unknown calib step \"%1\".").arg(action));
    return 2;
}

}

void printUsage()
{
    out() <<
"ServoBench - Feetech SCS/STS servo bench, terminal edition\n"
"\n"
"Usage:\n"
"  servobench-cli [connection options] <command> [arguments]\n"
"  servobench-cli                       start the full-screen tool\n"
"\n"
"Connection options (every command that talks to a servo):\n"
"  -p, --port NAME      serial port, /dev/ttyUSB0 or ttyUSB0 (default: first USB port)\n"
"  -b, --baud RATE      default 1000000\n"
"      --parity MODE    none, odd or even (default none)\n"
"  -t, --timeout MS     read timeout, default 50\n"
"\n"
"Looking around:\n"
"  ports [--all] [--json]              list serial ports\n"
"  scan [--ids L] [--from N] [--to N] [--json]\n"
"                                      ping the bus and list the servos found\n"
"  info <id> [--json]                  model, firmware, limits, faults, telemetry\n"
"  status <id> [--json]                one telemetry sample\n"
"  monitor <id> [--hz N] [--count N] [--duration S] [--csv|--json]\n"
"                                      stream telemetry until Ctrl-C\n"
"  record <id> --file F [--hz N] [--duration S] [--interval S]\n"
"                                      log telemetry in the GUI's CSV format\n"
"\n"
"Registers:\n"
"  read <id> <reg> [--json] [--quiet]  read one register by name or address\n"
"  write <id> <reg> <value> [--yes]    EPROM-aware write, verified by read-back\n"
"  dump <id> [--json]                  the whole register map\n"
"  regs save <id> <file>               register snapshot as JSON\n"
"  regs load <id> <file> [--yes]       restore writable registers from a snapshot\n"
"\n"
"Moving:\n"
"  torque <id|all> on|off [--ids L]    enable or release torque, verified\n"
"  pos <id> <counts> [--speed N] [--acc N] [--time N] [--mode M] [--wait]\n"
"  angle <id> <value> [--unit deg|rad] [same options as pos]\n"
"  action [<id>]                       run a queued REG WRITE (default: broadcast)\n"
"  sweep <id> --start N --end N [--hold MS] [--cycles N]\n"
"  step <id> --start N --end N [--size N] [--delay MS] [--cycles N]\n"
"    --mode is write (default), reg or sync.\n"
"\n"
"Calibration (writes EPROM; each step confirms three times unless --yes):\n"
"  midpoint <id> [--yes]               store the current position as the midpoint\n"
"  calib home [--ids L] [--yes]        zero the offset, reset limits, capture the pose\n"
"  calib record [--duration S]         track min and max while you move each joint\n"
"  calib names a,b,c                   name the joints, in ascending servo ID order\n"
"  calib show                          print the recorded calibration state\n"
"  calib export <file> [--names a,b,c] write LeRobot calibration.json\n"
"    The three steps share a state file: --state PATH, default\n"
"    " << defaultCalibStatePath() << "\n"
"\n"
"Full-screen tool:\n"
"  tui [--plain] [--ascii] [--no-mouse]\n"
"    --plain drops colour, --ascii drops the braille plot and box drawing.\n"
"\n"
"Other:\n"
"  --help [command]     this text, or help for one command\n"
"  --version            version and Qt build\n"
"\n"
"Examples:\n"
"  servobench-cli scan\n"
"  servobench-cli -p ttyUSB0 info 1\n"
"  servobench-cli monitor 1 --hz 50 --csv > run.csv\n"
"  servobench-cli write 1 \"Position P Gain\" 24\n"
"  servobench-cli torque all off && servobench-cli calib home --yes\n";
    out().flush();
}

void printCommandHelp(const QString &command)
{
    static const QMap<QString, QString> help = {
        {"scan",
         "scan [--ids 1,2,3 | --from N --to N] [--json]\n\n"
         "Pings every ID in turn and reads the model number of everything that answers, "
         "then reports torque state and any fault bits. This is what fills the servo list "
         "in the window, and most other commands need a servo's series, which comes from "
         "its model number."},
        {"read",
         "read <id> <register> [--json] [--quiet]\n\n"
         "The register is an address (42) or a name (\"Goal Position\", goal_position, "
         "goalposition). Names match case-insensitively and ignore spaces and underscores, "
         "and an unambiguous prefix is enough. --quiet prints the bare value, for scripts."},
        {"write",
         "write <id> <register> <value> [--yes]\n\n"
         "Unlocks EPROM if the register lives there, writes, relocks, then verifies by "
         "reading back. Torque state is held across the write and put back afterwards, "
         "the way the window does it. Writing address 5 changes the servo's ID and asks "
         "first unless --yes is given."},
        {"pos",
         "pos <id> <counts> [--speed N] [--acc N] [--time N] [--mode write|reg|sync] [--wait]\n\n"
         "--mode reg queues the move without running it; \"action\" then starts every "
         "queued servo at once. --mode sync sends an unacknowledged sync write. --wait "
         "polls Moving Status until the servo says it has arrived."},
        {"calib",
         "calib home | record | names | show | export\n\n"
         "The same three steps as the Calibration tab, and gated the same way: torque has "
         "to be released on every servo first.\n\n"
         "  1. calib home    zeroes Position Offset Value, resets Min/Max Position Limit, "
         "then stores the current pose as the midpoint of every joint.\n"
         "  2. calib record  follows the joints while you move them and keeps the extremes.\n"
         "  3. calib export  validates and writes LeRobot's calibration.json.\n\n"
         "What the window keeps in its table between button presses lives in a state file "
         "here instead, so the steps can be separate commands."},
        {"tui",
         "tui [--plain] [--ascii] [--no-mouse]\n\n"
         "The full-screen tool: live plot, servo control, register map and calibration, "
         "with the same gating as the window. Press ? inside it for the key map."},
    };

    const QString text = help.value(command.toLower());
    if(text.isEmpty())
    {
        printUsage();
        return;
    }
    out() << text << "\n";
    out().flush();
}

int runCommand(const CommandLine &line)
{
    installInterruptHandler();

    const QString command = line.command.toLower();

    if(command == "ports")
        return runPorts(line);
    if(command == "scan")
        return runScan(line);
    if(command == "info")
        return runInfo(line);
    if(command == "status")
        return runStatus(line);
    if(command == "monitor")
        return runMonitor(line);
    if(command == "record")
        return runRecord(line);
    if(command == "read")
        return runRead(line);
    if(command == "write")
        return runWrite(line);
    if(command == "dump")
        return runDump(line);
    if(command == "regs")
        return runRegs(line);
    if(command == "torque")
        return runTorque(line);
    if(command == "pos")
        return runPos(line);
    if(command == "angle")
        return runAngle(line);
    if(command == "action")
        return runAction(line);
    if(command == "midpoint")
        return runMidpoint(line);
    if(command == "sweep")
        return runSweepOrStep(line, false);
    if(command == "step")
        return runSweepOrStep(line, true);
    if(command == "calib")
        return runCalib(line);

    fail(QString("Unknown command \"%1\". Try servobench-cli --help.").arg(line.command));
    return 2;
}

}
