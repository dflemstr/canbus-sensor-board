#![no_std]
#![no_main]

use embassy_futures::select;
use embassy_stm32::{
    peripherals,
    pac,
    mode,
    i2c,
    flash,
    time,
    can,
    gpio
};
use embassy_sync::{
    blocking_mutex::raw,
    channel,
    mutex
};
// For link-time dependencies:
use {defmt_rtt as _, panic_probe as _};

// Store the value at the end of flash; very likely won't over-write application code.  But we could
// tweak the linker script to not place code here if we want to be fool-proof.
const CAN_ID_FLASH_OFFSET: u32 = (flash::FLASH_SIZE - flash::WRITE_SIZE) as u32;

/// The default CAN ID to use if no override is stored in flash.
const DEFAULT_CAN_ID: u16 = 1;

/// Time between sensor polls if the value recently changed
const POLL_DELAY_CHANGED: embassy_time::Duration = embassy_time::Duration::from_millis(10);

/// Time between sensor polls if the value has mostly remained the same
const POLL_DELAY_UNCHANGED: embassy_time::Duration = embassy_time::Duration::from_millis(100);

/// Time between sensor polls if the sensor previously reported an error
const POLL_DELAY_ERROR: embassy_time::Duration = embassy_time::Duration::from_millis(1000);

/// Max number of samples to buffer in queue to be sent over the CAN bus.
const SAMPLE_BUFFER_SIZE: usize = 64;

/// Delay between error blinks during error conditions
const ERROR_DELAY_BETWEEN_BLINKS: embassy_time::Duration = embassy_time::Duration::from_millis(300);
/// Delay between each group of blinks during error conditions
const ERROR_DELAY_BETWEEN_BLINK_GROUPS: embassy_time::Duration =
    embassy_time::Duration::from_millis(700);
/// Number of times we will blink out the same error code before moving on.
const ERROR_NUM_BLINK_GROUPS: usize = 3;

/// Number of blinks for canbus error
const ERROR_NUM_BLINKS_CANBUS: u32 = 2;
/// Number of blinks for sensor error
const ERROR_NUM_BLINKS_SENSOR: u32 = 3;

/// Delay between blinkenlights changing during startup
const STARTUP_BLINKENLIGHTS_DELAY: embassy_time::Duration = embassy_time::Duration::from_millis(10);

// For now let's only support one sensor per I2C bus. One could probably do something smart here
// to support an API for sharing buses.
type Sensor<'a, I2C> = mutex::Mutex<raw::ThreadModeRawMutex, SensorState<'a, I2C>>;
type Sensor1 = Sensor<'static, peripherals::I2C1>;
type Sensor2 = Sensor<'static, peripherals::I2C2>;

type ReadingsChannel = channel::Channel<raw::ThreadModeRawMutex, u16, SAMPLE_BUFFER_SIZE>;
type ReadingsSender = channel::Sender<'static, raw::ThreadModeRawMutex, u16, SAMPLE_BUFFER_SIZE>;
type ReadingsReceiver =
    channel::Receiver<'static, raw::ThreadModeRawMutex, u16, SAMPLE_BUFFER_SIZE>;
type Led = mutex::Mutex<raw::ThreadModeRawMutex, gpio::Output<'static>>;

embassy_stm32::bind_interrupts!(struct Irqs {
    USB_LP_CAN1_RX0 => can::Rx0InterruptHandler<peripherals::CAN>;
    CAN1_RX1 => can::Rx1InterruptHandler<peripherals::CAN>;
    CAN1_SCE => can::SceInterruptHandler<peripherals::CAN>;
    USB_HP_CAN1_TX => can::TxInterruptHandler<peripherals::CAN>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>;
});

#[repr(u8)]
#[derive(num_derive::FromPrimitive, defmt::Format)]
enum CanRequestId {
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
enum CanResponseId {
    // Skip 0 to avoid issues with sender mistakenly sending a zeroed buffer
    Ok = 1,
    Error = 2,
    SensorReading = 3,
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
    /// Unassigned sensor kind. Only allowed if the sensor is also disabled.
    Unassigned = 0,
    AS5600 = 1,
}

#[repr(u8)]
#[derive(num_derive::FromPrimitive, defmt::Format)]
enum SensorId {
    // Skip 0 to avoid issues with sender mistakenly sending a zeroed buffer
    Sensor1 = 1,
    Sensor2 = 2,
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

struct SensorState<'a, I2C>
where
    I2C: i2c::Instance,
{
    enabled: bool,
    subscribed: bool,
    kind: Option<SensorKindState<'a, I2C>>,
}

enum SensorKindState<'a, I2C>
where
    I2C: i2c::Instance,
{
    Unassigned(i2c::I2c<'a, I2C, mode::Async>),
    AS5600(as5600::asynch::As5600<i2c::I2c<'a, I2C, mode::Async>>),
}

#[derive(Clone, Copy)]
struct Leds<'a> {
    r_led: &'a Led,
    y_led: &'a Led,
    g_led: &'a Led,
}

struct AS5600Error<E>(as5600::error::Error<E>);
struct AS5600StatusError(as5600::status::Error);
struct AS5600ConfigurationError(as5600::configuration::error::Error);

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    // No special requirements, e.g. no external crystal
    let p = embassy_stm32::init(embassy_stm32::Config::default());

    // Set alternate pin mapping to B8/B9
    pac::AFIO.mapr().modify(|w| w.set_can1_remap(2));

    let mut flash = flash::Flash::new_blocking(p.FLASH);

    let initial_can_id = read_can_id(&mut flash);

    static G_LED: static_cell::StaticCell<Led> = static_cell::StaticCell::new();
    static Y_LED: static_cell::StaticCell<Led> = static_cell::StaticCell::new();
    static R_LED: static_cell::StaticCell<Led> = static_cell::StaticCell::new();

    let g_led = G_LED.init(mutex::Mutex::new(gpio::Output::new(
        p.PB12,
        gpio::Level::Low,
        gpio::Speed::Low,
    )));
    let y_led = Y_LED.init(mutex::Mutex::new(gpio::Output::new(
        p.PA12,
        gpio::Level::Low,
        gpio::Speed::Low,
    )));
    let r_led = R_LED.init(mutex::Mutex::new(gpio::Output::new(
        p.PA15,
        gpio::Level::Low,
        gpio::Speed::Low,
    )));

    let leds = Leds {
        g_led,
        y_led,
        r_led,
    };

    let mut can = can::Can::new(p.CAN, p.PB8, p.PB9, Irqs);

    can.modify_config()
        .set_loopback(false)
        .set_silent(false)
        .set_bitrate(1_000_000);

    can.enable().await;

    configure_recv_filters(&mut can, initial_can_id);

    let i2c1 = i2c::I2c::new(
        p.I2C1,
        p.PB6,
        p.PB7,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH7,
        time::Hertz(500_000),
        Default::default(),
    );
    let i2c2 = i2c::I2c::new(
        p.I2C2,
        p.PB10,
        p.PB11,
        Irqs,
        p.DMA1_CH4,
        p.DMA1_CH5,
        time::Hertz(500_000),
        Default::default(),
    );

    // Store these as static so that they get `'static` lifetime and survive when `main()` exits
    static SENSOR1: static_cell::StaticCell<Sensor1> = static_cell::StaticCell::new();
    static SENSOR1_CHANNEL: ReadingsChannel = channel::Channel::new();
    static SENSOR2: static_cell::StaticCell<Sensor2> = static_cell::StaticCell::new();
    static SENSOR2_CHANNEL: ReadingsChannel = channel::Channel::new();

    let sensor1 = SENSOR1.init(mutex::Mutex::new(SensorState::new(i2c1)));
    let sensor2 = SENSOR2.init(mutex::Mutex::new(SensorState::new(i2c2)));

    defmt::unwrap!(spawner.spawn(show_startup_blinkenlights(leds)));
    defmt::unwrap!(spawner.spawn(poll_sensor1(
        spawner.clone(),
        sensor1,
        SENSOR1_CHANNEL.sender(),
        leds
    )));
    defmt::unwrap!(spawner.spawn(poll_sensor2(
        spawner.clone(),
        sensor2,
        SENSOR2_CHANNEL.sender(),
        leds
    )));
    defmt::unwrap!(spawner.spawn(handle_can_requests(
        spawner.clone(),
        can,
        flash,
        initial_can_id,
        sensor1,
        SENSOR1_CHANNEL.receiver(),
        sensor2,
        SENSOR2_CHANNEL.receiver(),
        leds,
    )));

    // OK, all tasks started, now we go to sleep. Purely interrupt driven from here.
}

#[embassy_executor::task]
async fn show_startup_blinkenlights(leds: Leds<'static>) {
    leds.show_startup_blinkenlights().await;
}

#[embassy_executor::task(pool_size = 4)]
async fn show_error_code(leds: Leds<'static>, blinks: u32) {
    leds.show_error_code(blinks).await
}

#[embassy_executor::task]
async fn handle_can_requests(
    spawner: embassy_executor::Spawner,
    mut can: can::Can<'static, peripherals::CAN>,
    mut flash: flash::Flash<'static, flash::Blocking>,
    initial_can_id: can::StandardId,
    sensor1: &'static Sensor1,
    sensor1_readings: ReadingsReceiver,
    sensor2: &'static Sensor2,
    sensor2_readings: ReadingsReceiver,
    leds: Leds<'static>,
) {
    let mut current_id = initial_can_id;
    loop {
        match select::select3(
            can.read(),
            sensor1_readings.receive(),
            sensor2_readings.receive(),
        )
        .await
        {
            select::Either3::First(Ok(can::frame::Envelope { ts, frame })) => {
                // Happens if id recently changed, before we were able to update the hardware level
                // recv filters.
                if *frame.id() != can::Id::Standard(current_id) {
                    continue;
                }

                defmt::debug!("processing frame with id={} ts={}", current_id.as_raw(), ts);
                let data = frame.data();
                match handle_frame(
                    &mut current_id,
                    data,
                    sensor1,
                    sensor2,
                    &mut can,
                    &mut flash,
                )
                .await
                {
                    Ok(()) => {
                        can.write(&ok_frame(current_id)).await;
                    }
                    Err(code) => {
                        can.write(&error_frame(current_id, code)).await;
                    }
                }
            }
            select::Either3::First(Err(err)) => {
                defmt::error!("canbus error: {}", err);
                defmt::unwrap!(spawner.spawn(show_error_code(leds, ERROR_NUM_BLINKS_CANBUS)));
            }
            select::Either3::Second(val) => {
                can.write(&readings_frame(current_id, SensorId::Sensor1, val))
                    .await;
            }
            select::Either3::Third(val) => {
                can.write(&readings_frame(current_id, SensorId::Sensor2, val))
                    .await;
            }
        }
    }
}

// Need two copies of this function because it can't be generic. But they can share a generic impl
#[embassy_executor::task]
async fn poll_sensor1(
    spawner: embassy_executor::Spawner,
    sensor1: &'static Sensor1,
    readings: ReadingsSender,
    leds: Leds<'static>,
) {
    poll_sensor(spawner, sensor1, readings, leds).await;
}

#[embassy_executor::task]
async fn poll_sensor2(
    spawner: embassy_executor::Spawner,
    sensor2: &'static Sensor2,
    readings: ReadingsSender,
    leds: Leds<'static>,
) {
    poll_sensor(spawner, sensor2, readings, leds).await;
}

async fn poll_sensor<'a, I2C>(
    spawner: embassy_executor::Spawner,
    sensor: &Sensor<'a, I2C>,
    readings: ReadingsSender,
    leds: Leds<'static>,
) where
    I2C: i2c::Instance,
{
    let mut prev_angle = 0;
    let mut changed = false;
    let mut error = false;
    loop {
        if changed {
            embassy_time::Timer::after(POLL_DELAY_CHANGED).await;
            changed = false;
        } else if error {
            embassy_time::Timer::after(POLL_DELAY_ERROR).await;
            error = false;
        } else {
            embassy_time::Timer::after(POLL_DELAY_UNCHANGED).await;
        }

        let mut state = sensor.lock().await;

        if state.enabled && state.subscribed {
            if let Ok(angle) = state.read_angle().await {
                if prev_angle != angle {
                    defmt::info!("new angle value: {}", angle);
                    readings.send(angle).await;
                    prev_angle = angle;
                }
            } else {
                defmt::unwrap!(spawner.spawn(show_error_code(leds, ERROR_NUM_BLINKS_SENSOR)));
                error = true;
            }
        }

        // Happens automatically anyway, but keeping it here as a reminder for future refactors
        drop(state);
    }
}

async fn handle_frame(
    current_id: &mut can::StandardId,
    data: &[u8],
    sensor1: &Sensor1,
    sensor2: &Sensor2,
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
        CanRequestId::ConfigSensor1 => config_sensor(data, sensor1).await,
        CanRequestId::ConfigSensor2 => config_sensor(data, sensor2).await,
    }
}

async fn config_sensor<'a, I2C>(data: &[u8], state: &Sensor<'a, I2C>) -> Result<(), ErrorCode>
where
    I2C: i2c::Instance,
{
    use num_traits::FromPrimitive as _;

    if let &[sensor_kind, sensor_config, ..] = data {
        let sensor_kind = SensorKind::from_u8(sensor_kind).ok_or(ErrorCode::BadSensorKindParam)?;
        let sensor_config =
            SensorConfig::from_bits(sensor_config).ok_or(ErrorCode::BadSensorConfigParam)?;
        let enabled = sensor_config.contains(SensorConfig::Enable);
        let subscribed = sensor_config.contains(SensorConfig::Subscribe);
        defmt::info!(
            "configuring sensor 2 as kind={}, enabled={}, subscribed={}",
            sensor_kind,
            enabled,
            subscribed,
        );

        let mut state = state.lock().await;
        state.enabled = enabled;
        state.subscribed = subscribed;
        match sensor_kind {
            SensorKind::Unassigned => {
                state.make_unassigned();
            }
            SensorKind::AS5600 => {
                state.make_as5600();
            }
        }
        Ok(())
    } else {
        Err(ErrorCode::MessageTooShort)
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

fn readings_frame(id: can::StandardId, sensor: SensorId, value: u16) -> can::Frame {
    let [v0, v1] = value.to_be_bytes();
    defmt::unwrap!(can::frame::Frame::new_data(
        id,
        &[CanResponseId::SensorReading as u8, sensor as u8, v0, v1]
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

impl<'a, I2C> SensorState<'a, I2C>
where
    I2C: i2c::Instance,
{
    fn new(i2c: i2c::I2c<'a, I2C, mode::Async>) -> Self {
        Self {
            enabled: false,
            subscribed: false,
            kind: Some(SensorKindState::Unassigned(i2c)),
        }
    }

    /// Read the current sensor angle, with a range from 0x0000 to 0xffff.
    // TODO: maybe we should return some more detailed error value here
    async fn read_angle(&mut self) -> Result<u16, ()> {
        if let Some(kind) = &mut self.kind {
            match kind {
                SensorKindState::Unassigned(_) => Err(()),
                SensorKindState::AS5600(dev) => {
                    match dev.angle().await {
                        Ok(v) => {
                            // Convert 12 bit value to 16 bit value by padding with 4 bits
                            Ok(v << 4)
                        }
                        Err(e) => {
                            defmt::error!("could not read AS5600 angle: {}", AS5600Error(e));
                            Err(())
                        }
                    }
                }
            }
        } else {
            Err(())
        }
    }

    fn make_unassigned(&mut self) {
        self.kind = self.kind.take().map(|k| k.into_unassigned());
    }

    fn make_as5600(&mut self) {
        self.kind = self.kind.take().map(|k| k.into_as5600());
    }
}

impl<'a, I2C> SensorKindState<'a, I2C>
where
    I2C: i2c::Instance,
{
    fn into_unassigned(self) -> Self {
        match self {
            SensorKindState::Unassigned(i2c) => SensorKindState::Unassigned(i2c),
            SensorKindState::AS5600(dev) => SensorKindState::Unassigned(dev.release()),
        }
    }

    fn into_as5600(self) -> Self {
        match self {
            SensorKindState::Unassigned(i2c) => {
                SensorKindState::AS5600(as5600::asynch::As5600::new(i2c))
            }
            SensorKindState::AS5600(dev) => SensorKindState::AS5600(dev),
        }
    }
}

impl<'a> Leds<'a> {
    async fn show_startup_blinkenlights(&self) {
        let d = STARTUP_BLINKENLIGHTS_DELAY;

        for _ in 0..3 {
            self.g_led.lock().await.set_level(gpio::Level::High);
            embassy_time::Timer::after(d).await;
            self.y_led.lock().await.set_level(gpio::Level::High);
            embassy_time::Timer::after(d).await;
            self.r_led.lock().await.set_level(gpio::Level::High);
            embassy_time::Timer::after(d).await;
            self.g_led.lock().await.set_level(gpio::Level::Low);
            embassy_time::Timer::after(d).await;
            self.y_led.lock().await.set_level(gpio::Level::Low);
            embassy_time::Timer::after(d).await;
            self.r_led.lock().await.set_level(gpio::Level::Low);
            embassy_time::Timer::after(d).await;
        }
    }
    async fn show_error_code(&self, blinks: u32) {
        for _ in 0..ERROR_NUM_BLINK_GROUPS {
            for _ in 0..blinks {
                self.r_led.lock().await.set_level(gpio::Level::High);
                embassy_time::Timer::after(ERROR_DELAY_BETWEEN_BLINKS).await;
                self.r_led.lock().await.set_level(gpio::Level::Low);
                embassy_time::Timer::after(ERROR_DELAY_BETWEEN_BLINKS).await;
            }
            embassy_time::Timer::after(ERROR_DELAY_BETWEEN_BLINK_GROUPS).await;
        }
    }
}

impl<E> defmt::Format for AS5600Error<E>
where
    E: defmt::Format,
{
    fn format(&self, fmt: defmt::Formatter) {
        match self.0 {
            as5600::error::Error::Communication(ref e) => {
                defmt::write!(fmt, "Communication({})", e);
            }
            as5600::error::Error::Status(ref e) => {
                defmt::write!(fmt, "Status({})", AS5600StatusError(*e));
            }
            as5600::error::Error::Configuration(ref e) => {
                defmt::write!(fmt, "Configuration({})", AS5600ConfigurationError(*e));
            }
            as5600::error::Error::MaximumPositionPersistsReached => {
                defmt::write!(fmt, "MaximumPositionPersistsReached");
            }
            as5600::error::Error::MagnetRequired => {
                defmt::write!(fmt, "MagnetRequired");
            }
            as5600::error::Error::MangConfigPersistenceExhausted => {
                defmt::write!(fmt, "MangConfigPersistenceExhausted");
            }
        }
    }
}

impl defmt::Format for AS5600StatusError {
    fn format(&self, fmt: defmt::Formatter) {
        match self.0 {
            as5600::status::Error::InvalidBitPattern(b) => {
                defmt::write!(fmt, "InvalidBitPattern({})", b);
            }
        }
    }
}

impl defmt::Format for AS5600ConfigurationError {
    fn format(&self, fmt: defmt::Formatter) {
        match self.0 {
            as5600::configuration::error::Error::PowerModeBitPattern(b) => {
                defmt::write!(fmt, "PowerModeBitPattern({})", b);
            }
            as5600::configuration::error::Error::HysteresisBitPattern(b) => {
                defmt::write!(fmt, "HysteresisBitPattern({})", b);
            }
            as5600::configuration::error::Error::PwmFreqBitPattern(b) => {
                defmt::write!(fmt, "PwmFreqBitPattern({})", b);
            }
            as5600::configuration::error::Error::SlowFilterModeBitPattern(b) => {
                defmt::write!(fmt, "SlowFilterModeBitPattern({})", b);
            }
            as5600::configuration::error::Error::FastFilterThresholdBitPattern(b) => {
                defmt::write!(fmt, "FastFilterThresholdBitPattern({})", b);
            }
            as5600::configuration::error::Error::WatchdogState(b) => {
                defmt::write!(fmt, "WatchdogState({})", b);
            }
            as5600::configuration::error::Error::OutputStageBitPattern(b) => {
                defmt::write!(fmt, "OutputStageBitPattern({})", b);
            }
        }
    }
}
