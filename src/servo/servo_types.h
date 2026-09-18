#ifndef SERVO_MEM_CONFIG_H
#define SERVO_MEM_CONFIG_H

#include <map>
#include <vector>
#include <QString>
#include <QStringList>

namespace feetech_servo
{

enum ModelSeries
{
	SMCL,
	SMBL,
	STS,
	SCS
};

struct MemoryConfig
{
    uint8_t address;
    QString name;
    uint8_t size;
    uint16_t default_value;
    int8_t dir_bit;
    bool is_eprom;
    bool is_readonly;
    int16_t min_val;
    int16_t max_val;
};

inline const std::vector<MemoryConfig> SMCLMemConfig =
{
    {0, "Firmare Main Version NO.", 1, 0, -1, true, true, -1, -1},
    {1, "Firmware Secondary Version NO.", 1, 0, -1, true, true, -1, -1},
    {3, "Servo Main Version", 1, 0, -1, true, true, -1, -1},
    {4, "Servo Sub Version", 1, 0, -1, true, true, -1, -1},
    {5, "ID", 1, 0, -1, true, false, 0, 253},
    {6, "Baud Rate", 1, 4, -1, true, false, 0, 254},
    {7, "Return Delay Time", 1, 250, -1, true, false, 0, 254},
    {8, "Status Return Level", 1, 1, -1, true, false, 0, 1},
    {9, "Min Position Limit", 2, 0, 15, true, false, -1, -1},
    {11, "Max Position Limit", 2, 0, 15, true, false, -1, -1},
    {13, "Max Temperature limit", 1, 80, -1, true, false, 0, 100},
    {14, "Max Input Voltage", 1, 140, -1, true, false, 0, 254},
    {15, "Min Input Voltage", 1, 80, -1, true, false, 0, 254},
    {16, "Max Torque Limit", 2, 1000, -1, true, false, 0, 1000},
    {18, "Setting Byte", 1, 0, -1, true, false, 0, 254},
    {19, "Protection Switch", 1, 44, -1, true, false, 0, 254},
    {20, "LED Alarm Condition", 1, 47, -1, true, false, 0, 254},
    {21, "Position P Gain", 1, 32, -1, true, false, 0, 254},
    {22, "Position D Gain", 1, 0, -1, true, false, 0, 254},
    {23, "Position I Gain", 1, 0, -1, true, false, 0, 254},
    {24, "Punch", 2, 0, -1, true, false, 0, 1000},
    {26, "CW Dead Band", 1, 0, -1, true, false, 0, 32},
    {27, "CCW Dead Band", 1, 0, -1, true, false, 0, 32},
    {28, "Overload Current", 2, 0, -1, true, false, 0, 1023},
    {33, "Position Offset Value", 2, 0, 15, true, false, -2047, 2047},
    {35, "Work Mode", 1, 0, -1, true, false, 0, 2},
    {36, "Overcurrent Protection Time", 1, 100, -1, true, false, 0, 254},
    {37, "Protect Torque", 1, 40, -1, true, false, 0, 254},
    {38, "Overload Protection Time", 1, 80, -1, true, false, 0, 254},
    {39, "Overload Torque", 1, 80, -1, true, false, 0, 254},
    {40, "Torque Enable", 1, 0, -1, false, false, 0, 254},
    {41, "Goal  Acceleration", 1, 0, -1, false, false, 0, 254},
    {42, "Goal Position", 2, 0, -1, false, false, -32766, 32766},
    {44, "Running Time", 2, 0, 15, false, false, -32766, 32766},
    {46, "Goal  Velocity", 2, 0, 15, false, false, 0, 32766},
    {48, "Lock", 1, 1, -1, false, false, 0, 1},
    {56, "Present Position", 2, 0, 15, false, true, -1, -1},
    {58, "Present Velocity", 2, 0, 15, false, true, -1, -1},
    {60, "Present PWM", 2, 0, 10, false, true, -1, -1},
    {62, "Present Input Voltage", 1, 0, -1, false, true, -1, -1},
    {63, "Present Temperature", 1, 0, -1, false, true, -1, -1},
    {64, "Sync Write Flag", 1, 0, -1, false, true, -1, -1},
    {65, "Hardware Error Status", 1, 0, -1, false, true, -1, -1},
    {66, "Moving Status", 1, 0, -1, false, true, -1, -1},
    {69, "Present Current", 2, 0, 15, false, true, -1, -1},
};

inline const std::vector<MemoryConfig> SMBLMemConfig =
{
    {0, "Firmare Main Version NO.", 1, 0, -1, true, true, -1, -1},
    {1, "Firmware Secondary Version NO.", 1, 0, -1, true, true, -1, -1},
    {3, "Servo Main Version", 1, 0, -1, true, true, -1, -1},
    {4, "Servo Sub Version", 1, 0, -1, true, true, -1, -1},
    {5, "ID", 1, 0, -1, true, false, 0, 253},
    {6, "Baud Rate", 1, 4, -1, true, false, 0, 11},
    {7, "Return Delay Time", 1, 250, -1, true, false, 0, 254},
    {8, "Status Return Level", 1, 1, -1, true, false, 0, 1},
    {9, "Min Position Limit", 2, 0, 15, true, false, -1, -1},
    {11, "Max Position Limit", 2, 0, 15, true, false, -1, -1},
    {13, "Max Temperature limit", 1, 80, -1, true, false, 0, 100},
    {14, "Max Input Voltage", 1, 140, -1, true, false, 0, 254},
    {15, "Min Input Voltage", 1, 80, -1, true, false, 0, 254},
    {16, "Max Torque Limit", 2, 1000, -1, true, false, 0, 1000},
    {18, "Setting Byte", 1, 0, -1, true, false, 0, 254},
    {19, "Protection Switch", 1, 44, -1, true, false, 0, 254},
    {20, "LED Alarm Condition", 1, 47, -1, true, false, 0, 254},
    {21, "Position P Gain", 1, 32, -1, true, false, 0, 254},
    {22, "Position D Gain", 1, 0, -1, true, false, 0, 254},
    {23, "Position I Gain", 1, 0, -1, true, false, 0, 254},
    {24, "Punch", 2, 0, -1, true, false, 0, 1000},
    {26, "CW Dead Band", 1, 0, -1, true, false, 0, 32},
    {27, "CCW Dead Band", 1, 0, -1, true, false, 0, 32},
    {28, "Overload Current", 2, 0, -1, true, false, 0, 511},
    {30, "Angular Resolution", 1, 1, -1, true, false, 1, 100},
    {31, "Position Offset Value", 2, 0, 15, true, false, -2047, 2047},
    {33, "Work Mode", 1, 0, -1, true, false, 0, 2},
    {34, "Protect Torque", 1, 40, -1, true, false, 0, 254},
    {35, "Overload Protection Time", 1, 80, -1, true, false, 0, 254},
    {36, "Overload Torque", 1, 80, -1, true, false, 0, 254},
    {37, "Velocity P Gain", 1, 32, -1, true, false, 0, 254},
    {38, "Overcurrent Protection Time", 1, 100, -1, true, false, 0, 254},
    {39, "Velocity I Gain", 1, 0, -1, true, false, 0, 254},
    {40, "Torque Enable", 1, 0, -1, false, false, 0, 254},
    {41, "Goal  Acceleration", 1, 0, -1, false, false, 0, 254},
    {42, "Goal Position", 2, 0, 15, false, false, -32766, 32766},
    {44, "Running Time", 2, 0, 10, false, false, -32766, 32766},
    {46, "Goal  Velocity", 2, 0, 15, false, false, -32766, 32766},
    {48, "Torque Limit", 2, 1000, -1, false, false, 0, 1000},
    {55, "Lock", 1, 1, -1, false, false, 0, 1},
    {56, "Present Position", 2, 0, 15, false, true, -1, -1},
    {58, "Present Velocity", 2, 0, 15, false, true, -1, -1},
    {60, "Present PWM", 2, 0, 10, false, true, -1, -1},
    {62, "Present Input Voltage", 1, 0, -1, false, true, -1, -1},
    {63, "Present Temperature", 1, 0, -1, false, true, -1, -1},
    {64, "Sync Write Flag", 1, 0, -1, false, true, -1, -1},
    {65, "Hardware Error Status", 1, 0, -1, false, true, -1, -1},
    {66, "Moving Status", 1, 0, -1, false, true, -1, -1},
    {69, "Present Current", 2, 0, 15, false, true, -1, -1},
};

inline const std::vector<MemoryConfig> STSMemConfig =
{
    {0, "Firmare Main Version NO.", 1, 0, -1, true, true, -1, -1},
    {1, "Firmware Secondary Version NO.", 1, 0, -1, true, true, -1, -1},
    {3, "Servo Main Version", 1, 0, -1, true, true, -1, -1},
    {4, "Servo Sub Version", 1, 0, -1, true, true, -1, -1},
    {5, "ID", 1, 0, -1, true, false, 0, 253},
    {6, "Baud Rate", 1, 4, -1, true, false, 0, 7},
    {7, "Return Delay Time", 1, 250, -1, true, false, 0, 254},
    {8, "Status Return Level", 1, 1, -1, true, false, 0, 1},
    {9, "Min Position Limit", 2, 0, 15, true, false, -1, -1},
    {11, "Max Position Limit", 2, 0, 15, true, false, -1, -1},
    {13, "Max Temperature limit", 1, 80, -1, true, false, 0, 100},
    {14, "Max Input Voltage", 1, 140, -1, true, false, 0, 254},
    {15, "Min Input Voltage", 1, 80, -1, true, false, 0, 254},
    {16, "Max Torque Limit", 2, 1000, -1, true, false, 0, 1000},
    {18, "Setting Byte", 1, 0, -1, true, false, 0, 254},
    {19, "Protection Switch", 1, 44, -1, true, false, 0, 254},
    {20, "LED Alarm Condition", 1, 47, -1, true, false, 0, 254},
    {21, "Position P Gain", 1, 32, -1, true, false, 0, 254},
    {22, "Position D Gain", 1, 0, -1, true, false, 0, 254},
    {23, "Position I Gain", 1, 0, -1, true, false, 0, 254},
    {24, "Punch", 2, 0, -1, true, false, 0, 1000},
    {26, "CW Dead Band", 1, 0, -1, true, false, 0, 32},
    {27, "CCW Dead Band", 1, 0, -1, true, false, 0, 32},
    {28, "Overload Current", 2, 0, -1, true, false, 0, 511},
    {30, "Angular Resolution", 1, 1, -1, true, false, 1, 100},
    {31, "Position Offset Value", 2, 0, 15, true, false, -2047, 2047},
    {33, "Work Mode", 1, 0, -1, true, false, 0, 3},
    {34, "Protect Torque", 1, 40, -1, true, false, 0, 254},
    {35, "Overload Protection Time", 1, 80, -1, true, false, 0, 254},
    {36, "Overload Torque", 1, 80, -1, true, false, 0, 254},
    {37, "Velocity P Gain", 1, 32, -1, true, false, 0, 254},
    {38, "Overcurrent Protection Time", 1, 100, -1, true, false, 0, 254},
    {39, "Velocity I Gain", 1, 0, -1, true, false, 0, 254},
    {40, "Torque Enable", 1, 0, -1, false, false, 0, 254},
    {41, "Goal  Acceleration", 1, 0, -1, false, false, 0, 254},
    {42, "Goal Position", 2, 0, 15, false, false, -32766, 32766},
    {46, "Goal  Velocity", 2, 0, 15, false, false, -1000, 1000},
    {48, "Torque Limit", 2, 1000, -1, false, false, 0, 1000},
    {55, "Lock", 1, 1, -1, false, false, 0, 1},
    {56, "Present Position", 2, 0, 15, false, true, -1, -1},
    {58, "Present Velocity", 2, 0, 15, false, true, -1, -1},
    {60, "Present PWM", 2, 0, 10, false, true, -1, -1},
    {62, "Present Input Voltage", 1, 0, -1, false, true, -1, -1},
    {63, "Present Temperature", 1, 0, -1, false, true, -1, -1},
    {64, "Sync Write Flag", 1, 0, -1, false, true, -1, -1},
    {65, "Hardware Error Status", 1, 0, -1, false, true, -1, -1},
    {66, "Moving Status", 1, 0, -1, false, true, -1, -1},
    {69, "Present Current", 2, 0, 15, false, true, -1, -1},
};

inline const std::vector<MemoryConfig> SCSMemConfig =
{
    {0, "Firmare Main Version NO.", 1, 0, -1, true, true, -1, -1},
    {1, "Firmware Secondary Version NO.", 1, 0, -1, true, true, -1, -1},
    {3, "Servo Main Version", 1, 0, -1, true, true, -1, -1},
    {4, "Servo Sub Version", 1, 0, -1, true, true, -1, -1},
    {5, "ID", 1, 0, -1, true, false, 0, 253},
    {6, "Baud Rate", 1, 4, -1, true, false, 0, 10},
    {7, "Return Delay Time", 1, 250, -1, true, false, 0, 254},
    {8, "Status Return Level", 1, 1, -1, true, false, 0, 1},
    {9, "Min Position Limit", 2, 0, 15, true, false, 0, 1023},
    {11, "Max Position Limit", 2, 0, 15, true, false, 0, 1023},
    {13, "Max Temperature limit", 1, 80, -1, true, false, 0, 100},
    {14, "Max Input Voltage", 1, 140, -1, true, false, 0, 254},
    {15, "Min Input Voltage", 1, 80, -1, true, false, 0, 254},
    {16, "Max Torque Limit", 2, 1000, -1, true, false, 0, 1000},
    {19, "Protection Switch", 1, 32, -1, true, false, 0, 254},
    {20, "LED Alarm Condition", 1, 37, -1, true, false, 0, 254},
    {21, "Position P Gain", 1, 32, -1, true, false, 0, 254},
    {22, "Position D Gain", 1, 0, -1, true, false, 0, 254},
    {23, "Position I Gain", 1, 0, -1, true, false, 0, 254},
    {24, "Punch", 2, 0, -1, true, false, 0, 1000},
    {26, "CW Dead Band", 1, 2, -1, true, false, 0, 32},
    {27, "CCW Dead Band", 1, 2, -1, true, false, 0, 32},
    {37, "Protect Torque", 1, 40, -1, true, false, 0, 254},
    {38, "Overload Protection Time", 1, 80, -1, true, false, 0, 254},
    {39, "Overload Torque", 1, 80, -1, true, false, 0, 254},
    {40, "Torque Enable", 1, 0, -1, false, false, 0, 2},
    {42, "Goal Position", 2, 0, -1, false, false, 0, 1023},
    {44, "Running Time", 2, 0, 15, false, false, -32766, 32766},
    {46, "Goal  Velocity", 2, 0, -1, false, false, 0, 32766},
    {48, "Lock", 1, 1, -1, false, false, 0, 1},
    {56, "Present Position", 2, 0, 15, false, true, -1, -1},
    {58, "Present Velocity", 2, 0, 15, false, true, -1, -1},
    {60, "Present PWM", 2, 0, 10, false, true, -1, -1},
    {62, "Present Input Voltage", 1, 0, -1, false, true, -1, -1},
    {63, "Present Temperature", 1, 0, -1, false, true, -1, -1},
    {64, "Sync Write Flag", 1, 0, -1, false, true, -1, -1},
    {65, "Hardware Error Status", 1, 0, -1, false, true, -1, -1},
    {66, "Moving Status", 1, 0, -1, false, true, -1, -1},
};

inline const std::vector<MemoryConfig>& getMemConfig(ModelSeries series)
{
    switch(series)
    {
        case ModelSeries::SCS:
            return SCSMemConfig;
        case ModelSeries::STS:
            return STSMemConfig;
        case ModelSeries::SMBL:
            return SMBLMemConfig;
        case ModelSeries::SMCL:
            return SMCLMemConfig;
        default:
            return STSMemConfig;
    }
}

inline const MemoryConfig* findMemConfig(ModelSeries series, const QString &name)
{
    for(const auto &item : getMemConfig(series))
    {
        if(item.name == name)
        {
            return &item;
        }
    }
    return nullptr;
}

// The same lookup by register address. A networked bus sends a register as its
// series and address rather than as a copy of its MemoryConfig, so the table
// that decides sizes, sign-magnitude and EPROM locking is the one on the
// machine holding the serial port -- a client built from another revision
// cannot corrupt a write with a stale copy.
inline const MemoryConfig* findMemConfigByAddress(ModelSeries series, uint8_t address)
{
    for(const auto &item : getMemConfig(series))
    {
        if(item.address == address)
        {
            return &item;
        }
    }
    return nullptr;
}

// Servo Status (address 65) reports which protections have tripped. Unloading
// Condition (19) and LED Alarm Condition (20) select which are enabled and
// which flash the LED, and all three share this bit layout.
constexpr uint8_t SERVO_STATUS_ADDRESS = 65;

// Bit meanings, from Feetech's current memory tables:
//   磁编码SMS&STS-内存表解析_220328  (magnetic encoder: STS / SMS)
//   电位器SCS-内存表解析_220402      (potentiometer:    SCS)
// and corroborated by Feetech's own SDK (FTServo_Python,
// scservo_sdk/protocol_packet_handler.py), which decodes the identical byte:
//   ERRBIT_VOLTAGE 1  ERRBIT_ANGLE 2  ERRBIT_OVERHEAT 4
//   ERRBIT_OVERELE 8  ERRBIT_OVERLOAD 32      -- and nothing for 16.
//
// Bit 4 is deliberately left unnamed. Feetech's older STS table (V3.6) called
// it "Angle", but both current official tables mark it "--", their own tutorial
// calls its weight an "empty address", and the SDK defines no constant for it.
// Newer vendor sources win, and an unlabelled bit is better than a wrong label.
//
// Note the trap if you compare against the SDK: its ERRBIT_ANGLE is bit 1 (the
// position sensor), not the old tables' "Angle" at bit 4.
inline QString servoStatusBitName(ModelSeries series, int bit)
{
    // The potentiometer servos have no encoder or current sensing, so those
    // bits are documented as 无 (none) rather than as faults.
    const bool potentiometer = (series == ModelSeries::SCS);

    switch(bit)
    {
        case 0: return "voltage out of range";
        case 1: return potentiometer ? QString() : QString("position sensor fault");
        case 2: return "overheated";
        case 3: return potentiometer ? QString() : QString("overcurrent");
        case 5: return "overloaded";
        default: return QString();
    }
}

// Every fault currently set in a Servo Status byte, in plain words. Empty when
// the servo reports no fault. Bits with no documented meaning are reported as
// "unknown bit N" rather than silently dropped -- a servo asserting one is
// telling us something, even if the datasheet does not say what.
inline QStringList decodeServoStatus(ModelSeries series, int status)
{
    QStringList faults;
    if(status <= 0)
        return faults;

    for(int bit = 0; bit < 8; bit++)
    {
        if((status & (1 << bit)) == 0)
            continue;

        const QString name = servoStatusBitName(series, bit);
        faults << (name.isEmpty() ? QString("unknown bit %1").arg(bit) : name);
    }
    return faults;
}

// Encoder counts in a full turn. The UI needs this to convert between counts
// and joint angles, which it does without touching the bus.
inline int countsPerRev(ModelSeries series)
{
    return (series == ModelSeries::SCS) ? 1024 : 4096;
}

// Only the series with a Position Offset Value register can store a midpoint.
inline bool supportsMidpointCalibration(ModelSeries series)
{
    return series != ModelSeries::SCS;
}

inline QString seriesName(ModelSeries series)
{
    switch(series)
    {
        case ModelSeries::SCS:  return "SCS";
        case ModelSeries::STS:  return "STS";
        case ModelSeries::SMBL: return "SMBL";
        case ModelSeries::SMCL: return "SMCL";
    }
    return "STS";
}

// A register that stores its sign in a dedicated bit rather than as two's
// complement. dir_bit names that bit, and the register only counts as
// sign-magnitude if its range actually spans zero.
inline bool isSignMagnitude(const MemoryConfig &config)
{
    return config.dir_bit >= 0
        && config.min_val < 0 && config.max_val > 0
        && config.max_val <= ((1 << config.dir_bit) - 1);
}

inline int decodeSignMagnitude(int raw, int8_t dir_bit)
{
    if(raw & (1 << dir_bit))
    {
        return -(raw & ~(1 << dir_bit));
    }
    return raw;
}

inline uint16_t encodeSignMagnitude(int value, int8_t dir_bit)
{
    if(value < 0)
    {
        return static_cast<uint16_t>((-value) | (1 << dir_bit));
    }
    return static_cast<uint16_t>(value);
}

}

#endif
