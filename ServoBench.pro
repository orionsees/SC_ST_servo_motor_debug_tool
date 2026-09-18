QT += core gui network serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

TARGET = servobench

INCLUDEPATH += $$PWD/src

SOURCES += \
    src/main.cpp \
    src/mainwindow.cpp \
    src/graphwidget.cpp \
    src/net/protocol.cpp \
    src/net/remote_bus.cpp \
    src/servo/scserial.cpp \
    src/servo/servo_bus.cpp

HEADERS += \
    src/mainwindow.h \
    src/net/protocol.h \
    src/net/remote_bus.h \
    src/servo/servo_bus.h \
    src/calibration_json.h \
    src/register_snapshot.h \
    src/theme.h \
    src/confirm_steps.h \
    src/graphwidget.h \
    src/servo/port_list.h \
    src/servo/scserial.h \
    src/servo/servo_driver.h \
    src/servo/servo_types.h \
    src/servo/sms_sts.h \
    src/servo/scscl.h

FORMS += \
    src/mainwindow.ui

qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
