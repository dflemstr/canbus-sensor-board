#![no_std]
#![no_main]

use embassy_stm32::can;
use embassy_stm32::flash;
use embassy_stm32::pac;
use embassy_stm32::peripherals;
// For link-time dependencies:
use {defmt_rtt as _, panic_probe as _};

const CAN_ID_FLASH_OFFSET: u32 = 0;
const DEFAULT_CAN_ID: u16 = 1;

#[repr(u8)]
#[derive(num_derive::FromPrimitive, defmt::Format)]
enum CanRequestId {
    // Skip 0 to avoid issues with sender mistakenly sending a zeroed buffer
    ReadSensors = 1,
    SetCanId = 2,
    ConfigSensor1 = 3,
    ConfigSensor2 = 4,
}

#[repr(u8)]
#[derive(num_derive::FromPrimitive, defmt::Format)]
enum CanResponseId {
    // Skip 0 to avoid issues with sender mistakenly sending a zeroed buffer
    Ok = 1,
    Error = 2,
    SensorReadings = 3,
}

#[repr(u8)]
#[derive(num_derive::FromPrimitive, defmt::Format)]
enum ErrorCode {
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
enum SensorKind {
    /// Undefined sensor kind. Only allowed if the sensor is also disabled.
    Undefined = 0,
    AS5600 = 1,
}

#[derive(defmt::Format)]
struct SensorConfig(u8);

bitflags::bitflags! {
    impl SensorConfig: u8 {
        /// Enable the sensor; ensure it is reachable and not in an error state.
        const Enable = 0b00000001;
        /// Subscribe to the sensor; have this device send out update events when the sensor
        /// value changes.
        const Subscribe = 0b00000010;
    }
}

embassy_stm32::bind_interrupts!(struct Irqs {
    USB_LP_CAN1_RX0 => can::Rx0InterruptHandler<peripherals::CAN>;
    CAN1_RX1 => can::Rx1InterruptHandler<peripherals::CAN>;
    CAN1_SCE => can::SceInterruptHandler<peripherals::CAN>;
    USB_HP_CAN1_TX => can::TxInterruptHandler<peripherals::CAN>;
});

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    // No special requirements, e.g. no external crystal
    let p = embassy_stm32::init(embassy_stm32::Config::default());

    // Set alternate pin mapping to B8/B9
    pac::AFIO.mapr().modify(|w| w.set_can1_remap(2));

    let mut flash = flash::Flash::new_blocking(p.FLASH);

    let mut current_id = read_can_id(&mut flash);

    let mut can = can::Can::new(p.CAN, p.PB8, p.PB9, Irqs);

    can.modify_config()
        .set_loopback(false)
        .set_silent(false)
        .set_bitrate(1_000_000);

    can.enable().await;

    configure_recv_filters(&mut can, current_id);

    loop {
        match can.read().await {
            Ok(can::frame::Envelope { ts, frame }) => {
                // Happens if id recently changed, before we were able to update the hardware level
                // recv filters.
                if *frame.id() != can::Id::Standard(current_id) {
                    continue;
                }

                defmt::debug!("processing frame with id={} ts={}", current_id.as_raw(), ts);
                let data = frame.data();
                match handle_frame(&mut current_id, data, &mut can, &mut flash).await {
                    Ok(()) => {
                        can.write(&ok_frame(current_id)).await;
                    }
                    Err(code) => {
                        can.write(&error_frame(current_id, code)).await;
                    }
                }
            }
            Err(err) => {
                defmt::error!("canbus error: {}", err);
            }
        }
    }
}

async fn handle_frame(
    current_id: &mut can::StandardId,
    data: &[u8],
    can: &mut can::Can<'_, peripherals::CAN>,
    flash: &mut flash::Flash<'_, flash::Blocking>,
) -> Result<(), ErrorCode> {
    use num_traits::FromPrimitive as _;

    let (&cmd_byte, data) = data.split_first().ok_or(ErrorCode::MissingCommand)?;

    match CanRequestId::from_u8(cmd_byte).ok_or(ErrorCode::UnknownCommand)? {
        CanRequestId::SetCanId => {
            if let &[id0, id1, ..] = data {
                let id = u16::from_be_bytes([id0, id1]);
                let id = can::StandardId::new(id).ok_or(ErrorCode::BadCanIdParam)?;

                // Cheap check to save some flash write cycles
                if *current_id != id {
                    // First ensure id gets persisted
                    write_can_id(flash, id);
                    // ...before we convince ourselves that this is the new id.
                    *current_id = id;
                    // ...and finally we ensure that any new messages received must have the new id.
                    configure_recv_filters(can, *current_id);

                    // At this point there might still be messages in the input registers that use
                    // the old id, so we must manually filter by the current id when handling
                    // subsequent messages.
                }
                Ok(())
            } else {
                Err(ErrorCode::MessageTooShort)
            }
        }
        CanRequestId::ReadSensors => {
            // TODO
            Ok(())
        }
        CanRequestId::ConfigSensor1 => {
            if let &[sensor_kind, sensor_config, ..] = data {
                let sensor_kind =
                    SensorKind::from_u8(sensor_kind).ok_or(ErrorCode::BadSensorKindParam)?;
                let sensor_config = SensorConfig::from_bits(sensor_config)
                    .ok_or(ErrorCode::BadSensorConfigParam)?;
                // TODO actually configure stuff
                defmt::info!(
                    "configuring sensor 1 as kind={}, enable={}, subscribe={}",
                    sensor_kind,
                    sensor_config.contains(SensorConfig::Enable),
                    sensor_config.contains(SensorConfig::Subscribe),
                );
                Ok(())
            } else {
                Err(ErrorCode::MessageTooShort)
            }
        }
        CanRequestId::ConfigSensor2 => {
            if let &[sensor_kind, sensor_config, ..] = data {
                let sensor_kind =
                    SensorKind::from_u8(sensor_kind).ok_or(ErrorCode::BadSensorKindParam)?;
                let sensor_config = SensorConfig::from_bits(sensor_config)
                    .ok_or(ErrorCode::BadSensorConfigParam)?;
                // TODO actually configure stuff
                defmt::info!(
                    "configuring sensor 2 as kind={}, enable={}, subscribe={}",
                    sensor_kind,
                    sensor_config.contains(SensorConfig::Enable),
                    sensor_config.contains(SensorConfig::Subscribe),
                );
                Ok(())
            } else {
                Err(ErrorCode::MessageTooShort)
            }
        }
    }
}

fn ok_frame(id: can::StandardId) -> can::Frame {
    defmt::unwrap!(can::frame::Frame::new_data(id, &[CanResponseId::Ok as u8]))
}

fn error_frame(id: can::StandardId, error_code: ErrorCode) -> can::Frame {
    defmt::unwrap!(can::frame::Frame::new_data(
        id,
        &[CanResponseId::Error as u8, error_code as u8,]
    ))
}

fn read_can_id(flash: &mut flash::Flash<flash::Blocking>) -> can::StandardId {
    // Store as 32 bit because flash write size is 4
    let mut bytes = [0; 4];
    defmt::unwrap!(flash.blocking_read(CAN_ID_FLASH_OFFSET, &mut bytes));
    let raw_id = u32::from_ne_bytes(bytes);
    let mut raw_id = defmt::unwrap!(u16::try_from(raw_id));

    if raw_id == 0 {
        raw_id = DEFAULT_CAN_ID;
    }

    defmt::unwrap!(can::StandardId::new(raw_id))
}

fn write_can_id(flash: &mut flash::Flash<flash::Blocking>, id: can::StandardId) {
    // Store as 32 bit because flash write size is 4
    let bytes = u32::from(id.as_raw()).to_ne_bytes();
    defmt::unwrap!(flash.blocking_write(CAN_ID_FLASH_OFFSET, &bytes));
}

fn configure_recv_filters(can: &mut can::Can<peripherals::CAN>, id: can::StandardId) {
    can.modify_filters().enable_bank(
        0,
        can::Fifo::Fifo0,
        can::filter::Mask32::frames_with_std_id(id, can::StandardId::MAX),
    );
}
