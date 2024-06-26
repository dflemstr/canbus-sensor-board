//! CAN-bus protocol for the device.
//!
//! # Overview
//!
//! A CAN-bus message is up to 8 bytes long and will always use the ID of this device for all
//! messages.  Changing the CAN ID applies immediately and all following messages will use the
//! new ID, including the resulting OK/ACK message.
//!
//! Here's the allowed combinations of 8 byte message payloads.  Empty cells are reserved and
//! should be omitted by CAN senders, by reducing the length of the CAN frame.  However, CAN
//! receivers need to assume that received messages might be longer with trailing bytes for
//! forwards compatibility.
//!
//! | B0     | B1 | B2 | B3 | B4 | B5 | B6 | B7 | Description |
//! |--------|----------------------------------|-------------|
//! | `0x01` |    |    |    |    |    |    |    | A message indicating that the previous message was handled successfully. |
//! | `0x02` |  E |    |    |    |    |    |    | A message indicating that the previous message generated an error; look at the following `ErrorCode` for details |
//! | `0x03` |  S | v0 | v1 |    |    |    |    | A message with sensor readings from the specified sensor. |
//! | `0x04` |  S |    |    |    |    |    |    | Read the value off of the specified sensor. This will generate a `SensorReading` response. |
//! | `0x05` |id0 |id1 |  W |    |    |    |    | Update the CAN ID of this node; optionally persisting it. The following `Ok` response will use the new ID. Wait for the response before sending new messages, as messages will otherwise be ignored. |
//! | `0x06` |  S | SK | SC |  W |    |    |    | Configure the specified sensor. |
//!
//! B0 = Byte of type [`CanMessageKind`][], the discriminator values are used literally in the above
//!   table. <br>
//! Empty cell = Reserved byte, must be omitted by the writer by limiting CAN message length,
//!   however old readers must tolerate new writers possibly sending longer messages with these
//!   bytes populated.  It is safe to assume that an old message can always be handled correctly
//!   even if ignoring the newer bytes; breaking changes to a message should use a new
//!   [`CanMessageKind`][] instead of extending an old one.<br>
//! S = Byte of type [`SensorId`][]<br>
//! SK = Byte of type [`SensorKind`][]<br>
//! SC = Byte of type [`SensorConfig`][]<br>
//! E = Byte of type [`ErrorCode`][]<br>
//! W = Set to 1 to write the configuration persistently to flash.<br>
//! `[id0, id1]` = 12-bit CAN-bus ID padded with `0`s in big endian byte order.<br>
//! `[v0, v1]` = 16-bit sensor reading in big endian byte order. Angles are represented from
//!   `0x0000` to `0xffff` for a full rotation.<br>
//!

use embassy_stm32::can;

#[repr(u8)]
#[derive(num_derive::FromPrimitive, defmt::Format)]
pub enum CanMessageKind {
    // Skip 0 to avoid issues with sender mistakenly sending a zeroed buffer
    /// The previous message was handled successfully.
    Ok = 1,
    /// The previous message generated an error; an `ErrorCode` follows.
    Error = 2,
    /// The message contains a sensor reading for a previously configured sensor.  Sent by the
    /// device as a response to a previous `ReadSensor` command, or from a sensor that is
    /// enabled and configured with the `SensorConfig::Subscribe` bit set.
    SensorReading = 3,
    /// Read the value for the specified sensor, or use sensor id 0 to read all sensors.
    ReadSensor = 4,
    /// Change the CAN id of this node.
    SetCanId = 5,
    /// Configure a sensor.
    ConfigSensor = 6,
}

#[repr(u8)]
#[derive(num_derive::FromPrimitive, defmt::Format)]
pub enum ErrorCode {
    // Skip 0 to avoid issues with sender mistakenly sending a zeroed buffer
    MissingCommand = 1,
    UnknownCommand = 2,
    MessageTooShort = 3,
    BadCanIdParam = 4,
    BadSensorIdParam = 5,
    BadSensorKindParam = 6,
    BadSensorConfigParam = 7,
}

#[repr(u8)]
#[derive(Clone, Copy, num_derive::FromPrimitive, defmt::Format)]
pub enum SensorKind {
    /// Unassigned sensor kind. Only allowed if the sensor is also disabled.
    Unassigned = 0,
    AS5600 = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, num_derive::FromPrimitive, defmt::Format)]
pub enum SensorId {
    // Skip 0 to avoid issues with sender mistakenly sending a zeroed buffer
    Sensor1 = 1,
    Sensor2 = 2,
}

#[derive(Clone, Copy, defmt::Format)]
#[repr(transparent)]
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
    defmt::unwrap!(can::frame::Frame::new_data(id, &[CanMessageKind::Ok as u8]))
}

pub fn error_frame(id: can::StandardId, error_code: ErrorCode) -> can::Frame {
    defmt::unwrap!(can::frame::Frame::new_data(
        id,
        &[CanMessageKind::Error as u8, error_code as u8]
    ))
}

pub fn readings_frame(id: can::StandardId, sensor: SensorId, value: u16) -> can::Frame {
    let [v0, v1] = value.to_be_bytes();
    defmt::unwrap!(can::frame::Frame::new_data(
        id,
        &[CanMessageKind::SensorReading as u8, sensor as u8, v0, v1]
    ))
}
