#include "servo/servo_bus.h"

#include <QThread>

namespace feetech_servo
{

QElapsedTimer& busClock()
{
    static QElapsedTimer clock = []{
        QElapsedTimer t;
        t.start();
        return t;
    }();
    return clock;
}

IServoBus::IServoBus(QObject *parent)
    : QObject(parent)
{
    // Start it here rather than at the first sample, so a timestamp always
    // says how far into the run the sample was taken.
    busClock();
}

IServoBus::~IServoBus() = default;

bool IServoBus::connectTransport(const QString &, quint16, const QString &, QString *)
{
    // A serial bus is already where its servos are.
    return true;
}

ServoBus::ServoBus(QObject *parent)
    : IServoBus(parent)
{
    qRegisterMetaType<feetech_servo::ServoStatus>();

    // The port is a child, so moving this object to the bus thread takes the
    // port with it and every QSerialPort call still happens on its own thread.
    serial_ = new QSerialPort(this);
    scserial_ = new SCSerial(serial_);
    sts_driver_ = new StsDriver(scserial_);
    scs_driver_ = new ScsDriver(scserial_);
}

ServoBus::~ServoBus()
{
    // The port is deliberately not closed here: by this point the bus thread
    // has stopped, and QSerialPort::close() unregisters notifiers that belong
    // to that thread. Owners close the port on the bus thread before stopping
    // it; whatever is left is closed by ~QSerialPort as a child of this object.
    delete scs_driver_;
    delete sts_driver_;
    delete scserial_;
}

ServoDriver* ServoBus::use(ModelSeries series)
{
    ServoDriver *servo = (series == ModelSeries::SCS)
                       ? static_cast<ServoDriver *>(scs_driver_)
                       : static_cast<ServoDriver *>(sts_driver_);
    scserial_->set_end(servo->endianness());
    return servo;
}

bool ServoBus::open(const QString &port_name, int baud, QSerialPort::Parity parity, int timeout)
{
    assertOnBusThread();
    if(serial_->isOpen())
    {
        serial_->close();
    }

    serial_->setPortName(port_name);
    serial_->setBaudRate(baud);
    serial_->setParity(parity);
    serial_->setDataBits(QSerialPort::DataBits::Data8);
    serial_->setStopBits(QSerialPort::StopBits::OneStop);
    serial_->setFlowControl(QSerialPort::FlowControl::NoFlowControl);

    const bool ok = serial_->open(QIODevice::ReadWrite);
    if(ok)
    {
        scserial_->set_timeout(timeout);
        invalidateModeCaches();
    }
    emit openedChanged(ok);
    return ok;
}

void ServoBus::close()
{
    assertOnBusThread();
    if(serial_->isOpen())
    {
        serial_->close();
    }
    invalidateModeCaches();
    emit openedChanged(false);
}

void ServoBus::invalidateModeCaches()
{
    assertOnBusThread();
    scs_driver_->invalidate_mode_cache();
    sts_driver_->invalidate_mode_cache();
}

int ServoBus::readByte(uint8_t id, ModelSeries series, uint8_t address)
{
    assertOnBusThread();
    use(series);
    return scserial_->read_byte(id, address);
}

int ServoBus::readWord(uint8_t id, ModelSeries series, uint8_t address)
{
    assertOnBusThread();
    use(series);
    return scserial_->read_word(id, address);
}

void ServoBus::writeByte(uint8_t id, ModelSeries series, uint8_t address, uint8_t value)
{
    assertOnBusThread();
    use(series);
    scserial_->write_byte(id, address, value);
}

int ServoBus::readRegister(uint8_t id, ModelSeries series, const MemoryConfig &config)
{
    assertOnBusThread();
    use(series);
    const int raw = (config.size == 2) ? scserial_->read_word(id, config.address)
                                       : scserial_->read_byte(id, config.address);
    if(raw < 0)
        return -1;
    if(isSignMagnitude(config))
        return decodeSignMagnitude(raw, config.dir_bit);
    return raw;
}

std::optional<int> ServoBus::readPosition(uint8_t id, ModelSeries series)
{
    assertOnBusThread();
    const MemoryConfig *config = findMemConfig(series, "Present Position");
    if(config == nullptr)
        return std::nullopt;

    use(series);
    const int raw = scserial_->read_word(id, config->address);
    if(raw < 0)
        return std::nullopt;

    return decodeSignMagnitude(raw, config->dir_bit);
}

RegisterWriteResult ServoBus::writeRegister(uint8_t id, ModelSeries series,
                                            const MemoryConfig &config, int value)
{
    assertOnBusThread();
    ServoDriver *servo = use(series);
    const MemoryConfig *lock = findMemConfig(series, "Lock");
    const bool need_unlock = config.is_eprom && lock != nullptr;

    const uint16_t raw = isSignMagnitude(config) ? encodeSignMagnitude(value, config.dir_bit)
                                                 : static_cast<uint16_t>(value);
    const uint16_t expect = (config.size == 2) ? raw : static_cast<uint8_t>(raw);

    // Writing Torque Enable itself is the one case where the servo is meant to
    // change torque state, so leave that one alone.
    const int torque = (config.address == TORQUE_ENABLE_ADDRESS)
                     ? -1
                     : servo->hold_torque_state(id);

    if(need_unlock)
    {
        scserial_->write_byte(id, lock->address, 0);
    }

    if(config.size == 2)
    {
        scserial_->write_word(id, config.address, raw);
    }
    else
    {
        scserial_->write_byte(id, config.address, static_cast<uint8_t>(raw));
    }

    // Writing the ID register moves the servo to a new address, so everything
    // after this point has to be addressed there.
    const uint8_t effective = (config.address == 5) ? static_cast<uint8_t>(raw) : id;

    if(need_unlock)
    {
        QThread::msleep(20);
        scserial_->write_byte(effective, lock->address, 1);
    }

    const int read_back = (config.size == 2) ? scserial_->read_word(effective, config.address)
                                             : scserial_->read_byte(effective, config.address);

    servo->restore_torque_state(effective, torque);
    servo->invalidate_mode_cache();

    RegisterWriteResult result;
    result.effective_id = effective;
    result.ok = read_back >= 0 && static_cast<uint16_t>(read_back) == expect;
    return result;
}

MidpointResult ServoBus::setMidpoint(uint8_t id, ModelSeries series, const MemoryConfig &offset)
{
    assertOnBusThread();
    ServoDriver *servo = use(series);
    const MemoryConfig *lock = findMemConfig(series, "Lock");

    MidpointResult result;
    result.before = scserial_->read_word(id, offset.address);

    // calibration_offset() works by writing 128 into Torque Enable, so the
    // servo comes out of it with torque off unless the state is put back.
    const int torque = servo->hold_torque_state(id);

    if(lock != nullptr)
    {
        scserial_->write_byte(id, lock->address, 0);
    }

    result.acked = servo->calibration_offset(id) != 0;
    QThread::msleep(20);

    if(lock != nullptr)
    {
        scserial_->write_byte(id, lock->address, 1);
    }

    result.after = scserial_->read_word(id, offset.address);

    servo->restore_torque_state(id, torque);
    servo->invalidate_mode_cache();
    return result;
}

int ServoBus::readModelNumber(uint8_t id)
{
    assertOnBusThread();
    return scserial_->read_model_number(id);
}

int ServoBus::ping(uint8_t id)
{
    assertOnBusThread();
    return scserial_->ping(id);
}

void ServoBus::writePos(uint8_t id, ModelSeries series, int pos, int time, int speed, int acc)
{
    assertOnBusThread();
    use(series)->write_pos(id, pos, time, speed, acc);
}

void ServoBus::regWritePos(uint8_t id, ModelSeries series, int pos, int time, int speed, int acc)
{
    assertOnBusThread();
    use(series)->reg_write_pos(id, pos, time, speed, acc);
}

void ServoBus::syncWritePos(const std::vector<uint8_t> &ids, ModelSeries series,
                            int pos, int time, int speed, int acc)
{
    assertOnBusThread();
    use(series)->sync_write_pos(ids, pos, time, speed, acc);
}

void ServoBus::regWriteAction(uint8_t id)
{
    assertOnBusThread();
    scserial_->reg_write_action(id);
}

void ServoBus::flushWrites()
{
    assertOnBusThread();
    if(serial_->isOpen())
    {
        serial_->waitForBytesWritten(200);
    }
}

void ServoBus::enableTorque(uint8_t id, ModelSeries series, bool on)
{
    assertOnBusThread();
    use(series)->enable_torque(id, on ? 1 : 0);
}

ServoStatus ServoBus::readStatus(uint8_t id, ModelSeries series)
{
    assertOnBusThread();
    ServoDriver *servo = use(series);

    ServoStatus status;
    status.id = id;
    status.pos = servo->read_position(id);
    status.torque = servo->read_load(id);
    status.speed = servo->read_speed(id);
    status.current = servo->read_current(id);
    status.temp = servo->read_temperature(id);
    status.voltage = servo->read_voltage(id);
    status.move = servo->read_move(id);
    status.goal = servo->read_goal(id);

    // A servo that is not answering reads -1 everywhere; position is the one
    // the whole panel keys off, so treat it as the liveness check.
    status.ok = status.pos >= 0;
    status.t_ms = busClock().elapsed();
    return status;
}

void ServoBus::pollStatus(int id, int series)
{
    assertOnBusThread();
    if(!serial_->isOpen() || id < 0)
    {
        emit statusReady(ServoStatus());
        return;
    }
    emit statusReady(readStatus(static_cast<uint8_t>(id), static_cast<ModelSeries>(series)));
}

void ServoBus::startScan(int from, int to)
{
    assertOnBusThread();
    scan_abort_.storeRelaxed(0);

    bool completed = true;
    for(int id = from; id <= to; id++)
    {
        if(scan_abort_.loadRelaxed() != 0)
        {
            completed = false;
            break;
        }

        emit scanProgress(id);

        const int found = ping(static_cast<uint8_t>(id));
        if(found > 0)
        {
            emit scanFound(found, readModelNumber(static_cast<uint8_t>(found)));
        }
    }

    emit scanFinished(completed);
}

}
