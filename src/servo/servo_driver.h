#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <set>
#include <vector>

#include "scserial.h"

namespace feetech_servo
{

// Torque Enable sits at the same address on every supported series, so holding
// and restoring it does not need to know which one is selected.
constexpr uint8_t TORQUE_ENABLE_ADDRESS = 40;

// One servo family behind a common interface, so callers do not have to branch
// on ModelSeries for every position command and every telemetry read. The
// series-specific classes (SMS_STS, SCSCL) stay as they are; the drivers below
// adapt them to a single shape.
class ServoDriver
{
public:
    explicit ServoDriver(SCSerial *serial)
        : serial_(serial)
    {
    }

    virtual ~ServoDriver() = default;

    // --- series traits ---

    // Value for SCSerial::set_end(): the SCS series puts words on the wire the
    // other way round from the rest.
    virtual uint8_t endianness() const = 0;
    virtual int counts_per_rev() const = 0;
    virtual bool supports_midpoint_calibration() const { return false; }

    // --- position commands ---
    // Arguments are the ints the UI works in; each driver narrows them to the
    // wire types. Each series ignores the ones it has no register for: SCS has
    // no acceleration, STS/SM has no goal time.
    virtual int write_pos(uint8_t id, int pos, int time, int speed, int acc) = 0;
    virtual int reg_write_pos(uint8_t id, int pos, int time, int speed, int acc) = 0;
    virtual void sync_write_pos(const std::vector<uint8_t> &ids, int pos, int time, int speed, int acc) = 0;

    // --- torque ---

    int enable_torque(uint8_t id, uint8_t enable) const
    {
        return serial_->write_byte(id, TORQUE_ENABLE_ADDRESS, enable);
    }

    // Reads the torque state so a caller can put it back after an operation
    // that disturbs it.
    //
    // 0..2 are the torque states the servo can be left in. Anything else is a
    // failed read, or a command value such as 128 (calibrate midpoint), and is
    // not something we can put back.
    int hold_torque_state(uint8_t id) const
    {
        const int state = serial_->read_byte(id, TORQUE_ENABLE_ADDRESS);
        return (state >= 0 && state <= 2) ? state : -1;
    }

    void restore_torque_state(uint8_t id, int state) const
    {
        if(state < 0)
            return;

        if(serial_->read_byte(id, TORQUE_ENABLE_ADDRESS) == state)
            return;

        serial_->write_byte(id, TORQUE_ENABLE_ADDRESS, static_cast<uint8_t>(state));
    }

    // Stores the current position as the midpoint. Only the series that have a
    // Position Offset Value register support this.
    virtual int calibration_offset(uint8_t id) { (void)id; return 0; }

    // --- work mode ---

    // Put the servo into position mode if it is not there already. A no-op on
    // series with no Work Mode register.
    virtual void ensure_position_mode(uint8_t id) { (void)id; }

    // Call after anything that may have changed Work Mode, or a servo's ID,
    // behind the driver's back -- an EPROM write, or a fresh bus search.
    virtual void invalidate_mode_cache() {}

    // --- telemetry ---

    virtual int read_position(int id) = 0;
    virtual int read_speed(int id) = 0;
    virtual int read_load(int id) = 0;
    virtual int read_current(int id) = 0;
    virtual int read_temperature(int id) = 0;
    virtual int read_voltage(int id) = 0;
    virtual int read_move(int id) = 0;
    virtual int read_goal(int id) = 0;

protected:
    SCSerial *serial_;
};

// STS and SM servos. Position is sign-magnitude, goal time is not used, and
// Work Mode has to be checked before a position command.
class StsDriver : public ServoDriver
{
public:
    explicit StsDriver(SCSerial *serial)
        : ServoDriver(serial)
        , sts_(serial)
    {
    }

    uint8_t endianness() const override { return 0; }
    int counts_per_rev() const override { return countsPerRev(ModelSeries::STS); }
    bool supports_midpoint_calibration() const override { return true; }

    int write_pos(uint8_t id, int pos, int, int speed, int acc) override
    {
        ensure_position_mode(id);
        return sts_.write_pos_ex(id, static_cast<int16_t>(pos),
                                 static_cast<uint16_t>(speed), static_cast<uint8_t>(acc));
    }

    int reg_write_pos(uint8_t id, int pos, int, int speed, int acc) override
    {
        ensure_position_mode(id);
        return sts_.reg_write_pos_ex(id, static_cast<int16_t>(pos),
                                     static_cast<uint16_t>(speed), static_cast<uint8_t>(acc));
    }

    void sync_write_pos(const std::vector<uint8_t> &ids, int pos, int, int speed, int acc) override
    {
        // sync_write_pos_ex takes non-const arrays and sets the sign bit on the
        // positions in place, so give it buffers of our own.
        std::vector<uint8_t> id_buf(ids);
        std::vector<int16_t> goals(ids.size(), static_cast<int16_t>(pos));
        std::vector<uint16_t> speeds(ids.size(), static_cast<uint16_t>(speed));
        std::vector<uint8_t> accs(ids.size(), static_cast<uint8_t>(acc));
        sts_.sync_write_pos_ex(id_buf.data(), static_cast<uint8_t>(ids.size()),
                               goals.data(), speeds.data(), accs.data());
    }

    int calibration_offset(uint8_t id) override { return sts_.calibration_offset(id); }

    void ensure_position_mode(uint8_t id) override
    {
        if(position_mode_ids_.count(id) != 0)
            return;

        // Work Mode is an EPROM register, and writing it makes the servo
        // re-initialise, which clears Torque Enable. This used to be sent ahead
        // of every position command, so only touch it when the servo is not
        // already in position mode, and put the torque state back when it is.
        const int mode = serial_->read_byte(id, SMS_STS_MODE);
        if(mode < 0)
            return;

        if(mode != 0)
        {
            const int torque = hold_torque_state(id);
            sts_.rotation_mode(id);
            restore_torque_state(id, torque);
        }

        position_mode_ids_.insert(id);
    }

    void invalidate_mode_cache() override { position_mode_ids_.clear(); }

    int read_position(int id) override { return sts_.read_position(id); }
    int read_speed(int id) override { return sts_.read_speed(id); }
    int read_load(int id) override { return sts_.read_load(id); }
    int read_current(int id) override { return sts_.read_current(id); }
    int read_temperature(int id) override { return sts_.read_temperature(id); }
    int read_voltage(int id) override { return sts_.read_voltage(id); }
    int read_move(int id) override { return sts_.read_move(id); }
    int read_goal(int id) override { return sts_.read_goal(id); }

private:
    SMS_STS sts_;
    std::set<int> position_mode_ids_;
};

// SCS servos. Half the resolution, a goal time instead of an acceleration, and
// no Work Mode or Position Offset Value register.
class ScsDriver : public ServoDriver
{
public:
    explicit ScsDriver(SCSerial *serial)
        : ServoDriver(serial)
        , scs_(serial)
    {
    }

    uint8_t endianness() const override { return 1; }
    int counts_per_rev() const override { return countsPerRev(ModelSeries::SCS); }

    int write_pos(uint8_t id, int pos, int time, int speed, int) override
    {
        return scs_.write_pos(id, static_cast<uint16_t>(pos),
                              static_cast<uint16_t>(time), static_cast<uint16_t>(speed));
    }

    int reg_write_pos(uint8_t id, int pos, int time, int speed, int) override
    {
        return scs_.reg_write_pos(id, static_cast<uint16_t>(pos),
                                  static_cast<uint16_t>(time), static_cast<uint16_t>(speed));
    }

    void sync_write_pos(const std::vector<uint8_t> &ids, int pos, int time, int speed, int) override
    {
        std::vector<uint8_t> id_buf(ids);
        std::vector<uint16_t> goals(ids.size(), static_cast<uint16_t>(pos));
        std::vector<uint16_t> times(ids.size(), static_cast<uint16_t>(time));
        std::vector<uint16_t> speeds(ids.size(), static_cast<uint16_t>(speed));
        scs_.sync_write_pos(id_buf.data(), static_cast<uint8_t>(ids.size()),
                            goals.data(), times.data(), speeds.data());
    }

    int read_position(int id) override { return scs_.read_position(id); }
    int read_speed(int id) override { return scs_.read_speed(id); }
    int read_load(int id) override { return scs_.read_load(id); }
    int read_current(int id) override { return scs_.read_current(id); }
    int read_temperature(int id) override { return scs_.read_temperature(id); }
    int read_voltage(int id) override { return scs_.read_voltage(id); }
    int read_move(int id) override { return scs_.read_move(id); }
    int read_goal(int id) override { return scs_.read_goal(id); }

private:
    SCSCL scs_;
};

}

#endif
