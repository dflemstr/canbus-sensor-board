use crate::{config, protocol, sensors, status};
use embassy_futures::select;
use embassy_stm32::{can, flash, peripherals};

pub struct Core<'a, S> {
    can: can::Can<'a, peripherals::CAN>,
    flash: flash::Flash<'a, flash::Blocking>,
    can_id: can::StandardId,
    sensor1: &'a sensors::Sensor1<'a>,
    sensor1_readings: sensors::ReadingsReceiver,
    sensor2: &'a sensors::Sensor2<'a>,
    sensor2_readings: sensors::ReadingsReceiver,
    signaller: S,
}

impl<'a, S> Core<'a, S> {
    pub async fn initialize(
        mut can: can::Can<'a, peripherals::CAN>,
        mut flash: flash::Flash<'a, flash::Blocking>,
        sensor1: &'a sensors::Sensor1<'a>,
        sensor1_readings: sensors::ReadingsReceiver,
        sensor2: &'a sensors::Sensor2<'a>,
        sensor2_readings: sensors::ReadingsReceiver,
        signaller: S,
    ) -> Self {
        can.modify_config()
            .set_loopback(false)
            .set_silent(false)
            .set_bitrate(config::CAN_BITRATE);

        can.enable().await;

        let can_id = read_can_id(&mut flash);
        configure_recv_filters(&mut can, can_id);

        Self {
            can,
            flash,
            can_id,
            sensor1,
            sensor1_readings,
            sensor2,
            sensor2_readings,
            signaller,
        }
    }
}

impl<'a, S> Core<'a, S>
where
    S: status::Signaller,
{
    pub async fn run_forever(mut self) {
        loop {
            match select::select3(
                self.can.read(),
                self.sensor1_readings.receive(),
                self.sensor2_readings.receive(),
            )
            .await
            {
                select::Either3::First(Ok(can::frame::Envelope { ts, frame })) => {
                    // Happens if id recently changed, before we were able to update the hardware level
                    // recv filters.
                    if *frame.id() != can::Id::Standard(self.can_id) {
                        continue;
                    }

                    defmt::debug!(
                        "processing frame with id={} ts={}",
                        self.can_id.as_raw(),
                        ts
                    );
                    let data = frame.data();
                    match self.handle_frame(data).await {
                        Ok(()) => {
                            self.can.write(&protocol::ok_frame(self.can_id)).await;
                        }
                        Err(code) => {
                            self.can
                                .write(&protocol::error_frame(self.can_id, code))
                                .await;
                        }
                    }
                }
                select::Either3::First(Err(err)) => {
                    defmt::error!("canbus error: {}", err);
                    self.signaller.signal_error(status::ErrorKind::Can).await;
                }
                select::Either3::Second(val) => {
                    self.can
                        .write(&protocol::readings_frame(
                            self.can_id,
                            protocol::SensorId::Sensor1,
                            val,
                        ))
                        .await;
                }
                select::Either3::Third(val) => {
                    self.can
                        .write(&protocol::readings_frame(
                            self.can_id,
                            protocol::SensorId::Sensor2,
                            val,
                        ))
                        .await;
                }
            }
        }
    }

    async fn handle_frame(&mut self, data: &[u8]) -> Result<(), protocol::ErrorCode> {
        use num_traits::FromPrimitive as _;

        let (&cmd_byte, data) = data
            .split_first()
            .ok_or(protocol::ErrorCode::MissingCommand)?;

        match protocol::CanRequestId::from_u8(cmd_byte)
            .ok_or(protocol::ErrorCode::UnknownCommand)?
        {
            protocol::CanRequestId::SetCanId => {
                if let &[id0, id1, ..] = data {
                    let id = u16::from_be_bytes([id0, id1]);
                    let id = can::StandardId::new(id).ok_or(protocol::ErrorCode::BadCanIdParam)?;

                    // Cheap check to save some flash write cycles
                    if self.can_id != id {
                        // First ensure id gets persisted
                        write_can_id(&mut self.flash, id);
                        // ...before we convince ourselves that this is the new id.
                        self.can_id = id;
                        // ...and finally we ensure that any new messages received must have the new id.
                        configure_recv_filters(&mut self.can, self.can_id);

                        // At this point there might still be messages in the input registers that use
                        // the old id, so we must manually filter by the current id when handling
                        // subsequent messages.
                    }
                    Ok(())
                } else {
                    Err(protocol::ErrorCode::MessageTooShort)
                }
            }
            protocol::CanRequestId::ReadSensors => {
                if let Ok(value) = self.sensor1.read_angle().await {
                    self.can
                        .write(&protocol::readings_frame(
                            self.can_id,
                            protocol::SensorId::Sensor1,
                            value,
                        ))
                        .await;
                }
                if let Ok(value) = self.sensor2.read_angle().await {
                    self.can
                        .write(&protocol::readings_frame(
                            self.can_id,
                            protocol::SensorId::Sensor2,
                            value,
                        ))
                        .await;
                }
                Ok(())
            }
            protocol::CanRequestId::ConfigSensor1 => self.sensor1.handle_config_msg(data).await,
            protocol::CanRequestId::ConfigSensor2 => self.sensor2.handle_config_msg(data).await,
        }
    }
}

fn read_can_id(flash: &mut flash::Flash<flash::Blocking>) -> can::StandardId {
    let mut bytes = [0; flash::WRITE_SIZE];
    defmt::unwrap!(flash.blocking_read(config::CAN_ID_FLASH_OFFSET, &mut bytes));
    let raw_id = u32::from_ne_bytes(bytes);
    let mut raw_id = defmt::unwrap!(u16::try_from(raw_id));

    if raw_id == 0 {
        raw_id = config::DEFAULT_CAN_ID;
    }

    defmt::unwrap!(can::StandardId::new(raw_id))
}

fn write_can_id(flash: &mut flash::Flash<flash::Blocking>, id: can::StandardId) {
    // Store as 32 bit because flash write size is 4
    let bytes = u32::from(id.as_raw()).to_ne_bytes();
    defmt::unwrap!(flash.blocking_write(config::CAN_ID_FLASH_OFFSET, &bytes));
}

fn configure_recv_filters(can: &mut can::Can<peripherals::CAN>, id: can::StandardId) {
    can.modify_filters().enable_bank(
        0,
        can::Fifo::Fifo0,
        can::filter::Mask32::frames_with_std_id(id, can::StandardId::MAX),
    );
}
