#ifndef CLI_CALIB_STATE_H
#define CLI_CALIB_STATE_H

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStandardPaths>
#include <QString>
#include <QVector>

#include "calibration_json.h"

// Calibration is three steps -- set home, record the range, export -- and in a
// shell each of those is its own process. What the GUI keeps in its table
// between button presses is kept here in a small state file instead, so the
// steps compose the same way.
namespace cli
{

struct CalibState
{
    QString port;
    QString updated;
    QVector<JointCalibration> joints;

    JointCalibration *find(int id)
    {
        for(JointCalibration &joint : joints)
        {
            if(joint.id == id)
                return &joint;
        }
        return nullptr;
    }
};

inline QString defaultCalibStatePath()
{
    QString base = QStandardPaths::writableLocation(QStandardPaths::CacheLocation);
    if(base.isEmpty())
        base = QDir::homePath() + "/.cache/servobench";
    return base + "/calibration-state.json";
}

inline bool loadCalibState(const QString &path, CalibState *out, QString *error)
{
    QFile file(path);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if(error)
            *error = QString("No calibration state at %1. Run \"calib home\" first.").arg(path);
        return false;
    }

    const QByteArray data = file.readAll();
    file.close();

    QJsonParseError parse_error;
    const QJsonDocument doc = QJsonDocument::fromJson(data, &parse_error);
    if(!doc.isObject())
    {
        if(error)
            *error = QString("%1 is not valid JSON: %2").arg(path, parse_error.errorString());
        return false;
    }

    const QJsonObject root = doc.object();
    out->port = root.value("port").toString();
    out->updated = root.value("updated").toString();
    out->joints.clear();

    for(const QJsonValue &value : root.value("joints").toArray())
    {
        const QJsonObject entry = value.toObject();
        JointCalibration joint;
        joint.id = entry.value("id").toInt(-1);
        joint.name = entry.value("name").toString();
        joint.drive_mode = entry.value("drive_mode").toInt(0);
        joint.homing_offset = entry.value("homing_offset").toInt(0);
        joint.range_min = entry.value("range_min").toInt(0);
        joint.range_max = entry.value("range_max").toInt(0);
        if(joint.id >= 0)
            out->joints.append(joint);
    }
    return true;
}

inline bool saveCalibState(const QString &path, const CalibState &state, QString *error)
{
    QDir().mkpath(QFileInfo(path).absolutePath());

    QJsonArray joints;
    for(const JointCalibration &joint : state.joints)
    {
        QJsonObject entry;
        entry["id"] = joint.id;
        entry["name"] = joint.name;
        entry["drive_mode"] = joint.drive_mode;
        entry["homing_offset"] = joint.homing_offset;
        entry["range_min"] = joint.range_min;
        entry["range_max"] = joint.range_max;
        joints.append(entry);
    }

    QJsonObject root;
    root["port"] = state.port;
    root["updated"] = QDateTime::currentDateTime().toString(Qt::ISODate);
    root["joints"] = joints;

    QFile file(path);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        if(error)
            *error = QString("Could not write %1.").arg(path);
        return false;
    }
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    file.close();
    return true;
}

}

#endif
