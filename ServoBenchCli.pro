QT += core serialport
QT -= gui

CONFIG += c++17 console
CONFIG -= app_bundle

TARGET = servobench-cli

INCLUDEPATH += $$PWD/src

SOURCES += \
    src/cli/main.cpp \
    src/cli/commands.cpp \
    src/cli/plot.cpp \
    src/cli/session.cpp \
    src/cli/term.cpp \
    src/cli/tui.cpp \
    src/servo/scserial.cpp \
    src/servo/servo_bus.cpp

HEADERS += \
    src/cli/commands.h \
    src/cli/plot.h \
    src/cli/session.h \
    src/cli/term.h \
    src/cli/tui.h \
    src/calibration_json.h \
    src/confirm_steps.h \
    src/register_snapshot.h \
    src/servo/scserial.h \
    src/servo/scscl.h \
    src/servo/servo_bus.h \
    src/servo/servo_driver.h \
    src/servo/servo_types.h \
    src/servo/sms_sts.h

qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
