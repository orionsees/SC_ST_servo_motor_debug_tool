#ifndef CONFIRM_STEPS_H
#define CONFIRM_STEPS_H

#include <QString>
#include <QStringList>

const int DANGEROUS_ACTION_CONFIRMATIONS = 3;

inline QStringList buildSetHomeConfirmations(int servo_count, const QString &ids, const QString &positions)
{
    QStringList steps;

    steps << QString("About to write calibration to %1 servo(s): ID %2.\n\n"
                     "For each servo this writes Position Offset Value and resets "
                     "Min Position Limit and Max Position Limit to full range.\n\n"
                     "All three are EPROM registers, so the change is permanent.\n\n"
                     "Continue?")
                 .arg(servo_count).arg(ids);

    steps << QString("This overwrites the existing calibration and cannot be undone.\n\n"
                     "Every Position Offset Value and every Min/Max Position Limit currently "
                     "stored on these %1 servo(s) will be lost. If you depend on the current "
                     "calibration, cancel now and back up its JSON first.\n\n"
                     "EPROM has a finite write budget, so do not repeat this casually.\n\n"
                     "Continue?")
                 .arg(servo_count);

    steps << QString("Final confirmation.\n\n"
                     "The arm must ALREADY be held at the pose you want as the midpoint. "
                     "Whatever each servo reads right now becomes its zero.\n\n"
                     "Positions being captured:\n%1\n\n"
                     "Commit calibration to %2 servo(s)?")
                 .arg(positions).arg(servo_count);

    return steps;
}

#endif
