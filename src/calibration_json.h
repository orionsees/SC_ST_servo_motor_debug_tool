#ifndef CALIBRATION_JSON_H
#define CALIBRATION_JSON_H

#include <QString>
#include <QStringList>
#include <QVector>

struct JointCalibration
{
    QString name;
    int id = 0;
    int drive_mode = 0;
    int homing_offset = 0;
    int range_min = 0;
    int range_max = 0;
};

inline QString buildCalibrationJson(const QVector<JointCalibration> &joints)
{
    QString out = "{\n";
    for(int i = 0; i < joints.size(); i++)
    {
        const JointCalibration &j = joints[i];
        out += QString("    \"%1\": {\n").arg(j.name);
        out += QString("        \"id\": %1,\n").arg(j.id);
        out += QString("        \"drive_mode\": %1,\n").arg(j.drive_mode);
        out += QString("        \"homing_offset\": %1,\n").arg(j.homing_offset);
        out += QString("        \"range_min\": %1,\n").arg(j.range_min);
        out += QString("        \"range_max\": %1\n").arg(j.range_max);
        out += (i + 1 < joints.size()) ? "    },\n" : "    }\n";
    }
    out += "}\n";
    return out;
}

inline QStringList validateCalibrationJoints(const QVector<JointCalibration> &joints)
{
    QStringList problems;
    if(joints.isEmpty())
    {
        problems << "No servos detected.";
        return problems;
    }

    QStringList seen;
    for(const JointCalibration &j : joints)
    {
        if(j.name.trimmed().isEmpty())
        {
            problems << QString("ID %1 has no joint name.").arg(j.id);
        }
        else if(seen.contains(j.name))
        {
            problems << QString("Joint name \"%1\" is used more than once.").arg(j.name);
        }
        else
        {
            seen << j.name;
        }

        if(j.range_min >= j.range_max)
        {
            problems << QString("ID %1 has range_min (%2) >= range_max (%3).")
                            .arg(j.id).arg(j.range_min).arg(j.range_max);
        }
    }
    return problems;
}

#endif
