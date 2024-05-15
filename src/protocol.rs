use embassy_stm32::can;

#[repr(u8)]
#[derive(num_derive::FromPrimitive, defmt::Format)]
pub enum CanRequestId {
    // Skip 0 to avoid issues with sender mistakenly sending a zeroed buffer
    /// Read all currently configured sensors and reply with messages containing sensor readings
    ReadSensors = 1,
    /// Change the CAN id of this node.  This setting is persisted in flash.
    SetCanId = 2,
    /// Configure the sensor in sensor slot 1.
    ///
    /// This sensor is assumed to be connected to I2C bus 1.
    ConfigSensor1 = 3,
    /// Configure the sensor in sensor slot 2.
    ///
    /// This sensor is assumed to be connected to I2C bus 2.
    ConfigSensor2 = 4,
}

#[repr(u8)]
#[derive(num_derive::FromPrimitive, defmt::Format)]
pub enum CanResponseId {
    // Skip 0 to avoid issues with sender mistakenly sending a zeroed buffer
    Ok = 1,
    Error = 2,
    SensorReading = 3,
}

#[repr(u8)]
#[derive(num_derive::FromPrimitive, defmt::Format)]
pub enum ErrorCode {
    // Skip 0 to avoid issues with sender mistakenly sending a zeroed buffer
    MissingCommand = 1,
    UnknownCommand = 2,
    MessageTooShort = 3,
    BadCanIdParam = 4,
    BadSensorKindParam = 5,
    BadSensorConfigParam = 6,
}

#[repr(u8)]
#[derive(num_derive::FromPrimitive, defmt::Format)]
pub enum SensorKind {
    /// Unassigned sensor kind. Only allowed if the sensor is also disabled.
    Unassigned = 0,
    AS5600 = 1,
}

#[repr(u8)]
#[derive(num_derive::FromPrimitive, defmt::Format)]
pub enum SensorId {
    // Skip 0 to avoid issues with sender mistakenly sending a zeroed buffer
    Sensor1 = 1,
    Sensor2 = 2,
}

#[derive(defmt::Format)]
pub struct SensorConfig(u8);

bitflags::bitflags! {
    impl SensorConfig: u8 {
        /// Enable the sensor; ensure it is reachable and not in an error state.
        const Enable = 0b00000001;
        /// Subscribe to the sensor; have this device send out update events when the sensor
        /// value changes.
        const Subscribe = 0b00000010;
    }
}

pub fn ok_frame(id: can::StandardId) -> can::Frame {
    defmt::unwrap!(can::frame::Frame::new_data(id, &[CanResponseId::Ok as u8]))
}

pub fn error_frame(id: can::StandardId, error_code: ErrorCode) -> can::Frame {
    defmt::unwrap!(can::frame::Frame::new_data(
        id,
        &[CanResponseId::Error as u8, error_code as u8,]
    ))
}

pub fn readings_frame(id: can::StandardId, sensor: SensorId, value: u16) -> can::Frame {
    let [v0, v1] = value.to_be_bytes();
    defmt::unwrap!(can::frame::Frame::new_data(
        id,
        &[CanResponseId::SensorReading as u8, sensor as u8, v0, v1]
    ))
}
