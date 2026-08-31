#ifndef REGISTER_SNAPSHOT_H
#define REGISTER_SNAPSHOT_H

#include <QByteArray>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QString>
#include <QVector>

struct RegisterEntry
{
    int address = 0;
    QString name;
    int value = 0;
    bool writable = false;
};

struct RegisterSnapshot
{
    int id = 0;
    QString model;
    QString series;
    QVector<RegisterEntry> registers;
};

inline QByteArray buildRegisterSnapshotJson(const RegisterSnapshot &snapshot)
{
    QJsonObject servo;
    servo["id"] = snapshot.id;
    servo["model"] = snapshot.model;
    servo["series"] = snapshot.series;

    QJsonArray regs;
    for (const RegisterEntry &e : snapshot.registers)
    {
        QJsonObject o;
        o["address"] = e.address;
        o["name"] = e.name;
        o["value"] = e.value;
        o["writable"] = e.writable;
        regs.append(o);
    }

    QJsonObject root;
    root["servo"] = servo;
    root["registers"] = regs;
    return QJsonDocument(root).toJson(QJsonDocument::Indented);
}

inline bool parseRegisterSnapshotJson(const QByteArray &data, RegisterSnapshot *out, QString *error)
{
    QJsonParseError parse_error;
    const QJsonDocument doc = QJsonDocument::fromJson(data, &parse_error);
    if (doc.isNull())
    {
        *error = QString("Not valid JSON: %1").arg(parse_error.errorString());
        return false;
    }
    if (!doc.isObject())
    {
        *error = "Top level of the file is not a JSON object.";
        return false;
    }

    const QJsonObject root = doc.object();
    if (!root.contains("servo") || !root.value("servo").isObject())
    {
        *error = "Missing \"servo\" section.";
        return false;
    }
    if (!root.contains("registers") || !root.value("registers").isArray())
    {
        *error = "Missing \"registers\" array.";
        return false;
    }

    const QJsonObject servo = root.value("servo").toObject();
    out->id = servo.value("id").toInt(-1);
    out->model = servo.value("model").toString();
    out->series = servo.value("series").toString();
    if (out->series.isEmpty())
    {
        *error = "Missing \"series\" in the servo section.";
        return false;
    }

    out->registers.clear();
    const QJsonArray regs = root.value("registers").toArray();
    for (const QJsonValue &v : regs)
    {
        if (!v.isObject())
        {
            *error = "A register entry is not an object.";
            return false;
        }
        const QJsonObject o = v.toObject();
        if (!o.contains("address") || !o.contains("value"))
        {
            *error = "A register entry is missing \"address\" or \"value\".";
            return false;
        }
        RegisterEntry e;
        e.address = o.value("address").toInt(-1);
        e.name = o.value("name").toString();
        e.value = o.value("value").toInt();
        e.writable = o.value("writable").toBool(false);
        if (e.address < 0 || e.address > 255)
        {
            *error = QString("Register address %1 is out of range.").arg(e.address);
            return false;
        }
        out->registers.append(e);
    }

    if (out->registers.isEmpty())
    {
        *error = "The file contains no registers.";
        return false;
    }
    return true;
}

#endif
