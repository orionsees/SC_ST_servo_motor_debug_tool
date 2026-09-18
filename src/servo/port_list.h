#ifndef SERVO_PORT_LIST_H
#define SERVO_PORT_LIST_H

#include <QSerialPortInfo>
#include <QString>
#include <QVector>

namespace feetech_servo
{

// One serial port a bus could be opened on. Over a network link these describe
// the ports on the robot, not on the machine running the window, so the field
// names deliberately say nothing about which side they came from.
struct PortInfo
{
    QString name;
    QString description;
    QString manufacturer;
    bool has_vendor_id = false;
};

// Every serial port worth offering. A USB adapter always carries a vendor
// identifier, which is what separates the one real adapter from the kernel's
// 30-odd ttyS stubs. If nothing has one -- a genuine motherboard COM port
// would not -- everything is listed instead of an empty list.
inline QVector<PortInfo> localPorts()
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

}

#endif
