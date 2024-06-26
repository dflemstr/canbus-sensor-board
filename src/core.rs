use crate::{config, protocol, sensors, status};
use embassy_futures::select;
use embassy_stm32::{can, flash, peripherals};

const CAN_ID_CRC: crc::Crc<u16> = crc::Crc::<u16>::new(&crc::CRC_15_CAN);
const SENSOR_CONFIG_CRC: crc::Crc<u16> = crc::Crc::<u16>::new(&crc::CRC_15_CAN);

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

        let can_id = read_can_id(&mut flash, config::flash_offset_can_id());
        configure_recv_filters(&mut can, can_id);

        defmt::debug!("check if we have persistent config for sensor 1...");
        if let Some((kind, config)) =
            read_sensor_config(&mut flash, config::flash_offset_sensor1())
        {
            sensor1.configure(kind, config).await;
        }

        defmt::debug!("check if we have persistent config for sensor 2...");
        if let Some((kind, config)) =
            read_sensor_config(&mut flash, config::flash_offset_sensor2())
        {
            sensor2.configure(kind, config).await;
        }

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
                        "processing frame with id={=u16} ts={=u64:us}",
                        self.can_id.as_raw(),
                        ts.as_micros()
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

        match protocol::CanMessageKind::from_u8(cmd_byte)
            .ok_or(protocol::ErrorCode::UnknownCommand)?
        {
            protocol::CanMessageKind::Ok => Ok(()),
            protocol::CanMessageKind::Error => Ok(()),
            protocol::CanMessageKind::SensorReading => Ok(()),
            protocol::CanMessageKind::SetCanId => {
                if let &[id0, id1, persist, ..] = data {
                    let id = u16::from_be_bytes([id0, id1]);
                    let id = can::StandardId::new(id).ok_or(protocol::ErrorCode::BadCanIdParam)?;
                    self.set_can_id(id, persist == 1);
                    Ok(())
                } else {
                    Err(protocol::ErrorCode::MessageTooShort)
                }
            }
            protocol::CanMessageKind::ReadSensor => {
                if let &[id, ..] = data {
                    let (s1, s2) = if id == 0 {
                        (true, true) // Read both sensors
                    } else {
                        match protocol::SensorId::from_u8(id)
                            .ok_or(protocol::ErrorCode::BadSensorIdParam)?
                        {
                            protocol::SensorId::Sensor1 => (true, false),
                            protocol::SensorId::Sensor2 => (false, true),
                        }
                    };
                    if s1 {
                        if let Ok(value) = self.sensor1.read_angle().await {
                            self.can
                                .write(&protocol::readings_frame(
                                    self.can_id,
                                    protocol::SensorId::Sensor1,
                                    value,
                                ))
                                .await;
                        }
                    }
                    if s2 {
                        if let Ok(value) = self.sensor2.read_angle().await {
                            self.can
                                .write(&protocol::readings_frame(
                                    self.can_id,
                                    protocol::SensorId::Sensor2,
                                    value,
                                ))
                                .await;
                        }
                    }
                }
                Ok(())
            }
            protocol::CanMessageKind::ConfigSensor => {
                if let &[sensor, ref rest @ ..] = data {
                    if sensor == 0 {
                        // Configure all sensors the same
                        self.sensor1
                            .handle_config_msg(
                                rest,
                                &mut self.flash,
                                config::flash_offset_sensor1(),
                            )
                            .await?;
                        self.sensor2
                            .handle_config_msg(
                                rest,
                                &mut self.flash,
                                config::flash_offset_sensor2(),
                            )
                            .await?;
                        Ok(())
                    } else {
                        match protocol::SensorId::from_u8(sensor)
                            .ok_or(protocol::ErrorCode::BadSensorIdParam)?
                        {
                            protocol::SensorId::Sensor1 => {
                                self.sensor1
                                    .handle_config_msg(
                                        rest,
                                        &mut self.flash,
                                        config::flash_offset_sensor1(),
                                    )
                                    .await
                            }
                            protocol::SensorId::Sensor2 => {
                                self.sensor2
                                    .handle_config_msg(
                                        rest,
                                        &mut self.flash,
                                        config::flash_offset_sensor2(),
                                    )
                                    .await
                            }
                        }
                    }
                } else {
                    Err(protocol::ErrorCode::MessageTooShort)
                }
            }
        }
    }

    fn set_can_id(&mut self, id: can::StandardId, persist: bool) {

        // Cheap check to avoid work
        if self.can_id != id {
            if persist {
                // First ensure id gets persisted.
                write_can_id(&mut self.flash, config::flash_offset_can_id(), id);
            }
            // ...before we convince ourselves that this is the new id.
            self.can_id = id;
            // ...and finally we ensure that any new messages received must have the new id.
            configure_recv_filters(&mut self.can, self.can_id);

            // At this point there might still be messages in the input registers that use
            // the old id, so we must manually filter by the current id when handling
            // subsequent messages.
        }
    }
}

fn read_can_id(flash: &mut flash::Flash<flash::Blocking>, offset: u32) -> can::StandardId {
    defmt::trace!("reading can id from offset {=u32:x}", offset);
    let mut bytes = [0; flash::WRITE_SIZE];
    defmt::unwrap!(flash.blocking_read(offset, &mut bytes));
    let [id0, id1, crc0, crc1] = bytes;
    let crc = u16::from_ne_bytes([crc0, crc1]);
    let id_bytes = [id0, id1];
    let raw_id = if CAN_ID_CRC.checksum(&id_bytes) == crc {
        let id = u16::from_ne_bytes(id_bytes);
        defmt::debug!(
            "stored CAN ID passed CRC validity check; using flash value {=u16}",
            id
        );
        id
    } else {
        defmt::debug!(
            "stored CAN ID did not pass CRC validity check; using default value {=u16}",
            config::DEFAULT_CAN_ID
        );
        config::DEFAULT_CAN_ID
    };

    defmt::unwrap!(can::StandardId::new(raw_id))
}

fn write_can_id(flash: &mut flash::Flash<flash::Blocking>, offset: u32, id: can::StandardId) {
    defmt::trace!("writing can id {=u16} to offset {=u32:x}", id.as_raw(), offset);
    let id_bytes @ [id0, id1] = u16::from(id.as_raw()).to_ne_bytes();
    let [crc0, crc1] = CAN_ID_CRC.checksum(&id_bytes).to_ne_bytes();
    write_flash(flash, offset, [id0, id1, crc0, crc1])
}

fn configure_recv_filters(can: &mut can::Can<peripherals::CAN>, id: can::StandardId) {
    can.modify_filters().enable_bank(
        0,
        can::Fifo::Fifo0,
        can::filter::Mask32::frames_with_std_id(id, can::StandardId::MAX),
    );
}

fn read_sensor_config(
    flash: &mut flash::Flash<flash::Blocking>,
    offset: u32,
) -> Option<(protocol::SensorKind, protocol::SensorConfig)> {
    use num_traits::FromPrimitive as _;

    defmt::trace!("reading sensor config from offset {=u32:x}", offset);

    let mut bytes = [0; flash::WRITE_SIZE];
    defmt::unwrap!(flash.blocking_read(offset, &mut bytes));
    let [kind_byte, config_byte, crc0, crc1] = bytes;
    let crc = u16::from_ne_bytes([crc0, crc1]);
    if SENSOR_CONFIG_CRC.checksum(&[kind_byte, config_byte]) == crc {
        let kind = protocol::SensorKind::from_u8(kind_byte)?;
        let config = protocol::SensorConfig::from_bits(config_byte)?;
        defmt::debug!(
            "stored sensor config passed CRC validity check; using kind={}, enabled={=bool}, subscribed={=bool}",
            kind,
            config.contains(protocol::SensorConfig::Enable),
            config.contains(protocol::SensorConfig::Subscribe),
        );
        Some((kind, config))
    } else {
        defmt::debug!("stored sensor config did not pass CRC check; ignoring.");
        None
    }
}

pub fn write_sensor_config(
    flash: &mut flash::Flash<flash::Blocking>,
    offset: u32,
    kind: protocol::SensorKind,
    config: protocol::SensorConfig,
) {
    defmt::trace!("writing sensor config to offset {=u32:x}", offset);
    let bytes @ [kind_byte, config_byte] = [kind as u8, config.bits()];
    let [crc0, crc1] = SENSOR_CONFIG_CRC.checksum(&bytes).to_ne_bytes();
    write_flash(flash, offset, [kind_byte, config_byte, crc0, crc1])
}

// Erases the full sector starting at `offset` and writes `data` to the beginning of it.
fn write_flash(flash: &mut flash::Flash<flash::Blocking>, offset: u32, data: [u8; 4]) {
    defmt::unwrap!(flash.blocking_erase(offset, offset + flash::MAX_ERASE_SIZE as u32));
    defmt::unwrap!(flash.blocking_write(offset, &data))
}
