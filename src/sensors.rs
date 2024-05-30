use crate::{config, protocol, status};
use embassy_stm32::{flash, i2c, mode, peripherals};
use embassy_sync::blocking_mutex::raw;
use embassy_sync::{channel, mutex};

pub type Sensor1<'a> = Sensor<'a, peripherals::I2C1>;
pub type Sensor2<'a> = Sensor<'a, peripherals::I2C2>;

pub type ReadingsChannel =
    channel::Channel<raw::ThreadModeRawMutex, u16, { config::SAMPLE_BUFFER_SIZE }>;
pub type ReadingsSender =
    channel::Sender<'static, raw::ThreadModeRawMutex, u16, { config::SAMPLE_BUFFER_SIZE }>;
pub type ReadingsReceiver =
    channel::Receiver<'static, raw::ThreadModeRawMutex, u16, { config::SAMPLE_BUFFER_SIZE }>;

pub struct Sensor<'a, I2C>
where
    I2C: i2c::Instance,
{
    state: mutex::Mutex<raw::ThreadModeRawMutex, State<'a, I2C>>,
}

struct State<'a, I2C>
where
    I2C: i2c::Instance,
{
    enabled: bool,
    subscribed: bool,
    kind: Option<Driver<'a, I2C>>,
}

enum Driver<'a, I2C>
where
    I2C: i2c::Instance,
{
    Unassigned(i2c::I2c<'a, I2C, mode::Async>),
    AS5600(as5600::asynch::As5600<i2c::I2c<'a, I2C, mode::Async>>),
}

// Newtypes to enable impl of defmt::Format
#[repr(transparent)]
struct AS5600Error<E>(as5600::error::Error<E>);
#[repr(transparent)]
struct AS5600StatusError(as5600::status::Error);
#[repr(transparent)]
struct AS5600ConfigurationError(as5600::configuration::error::Error);

impl<'a, I2C> Sensor<'a, I2C>
where
    I2C: i2c::Instance,
{
    pub fn new(i2c: i2c::I2c<'a, I2C, mode::Async>) -> Self {
        let state = mutex::Mutex::new(State::new(i2c));
        Self { state }
    }

    pub async fn handle_config_msg(
        &self,
        can_message: &[u8],
        flash: &mut flash::Flash<'_, flash::Blocking>,
        flash_offset: u32,
    ) -> Result<(), protocol::ErrorCode> {
        use num_traits::FromPrimitive as _;

        if let &[sensor_kind, sensor_config, persist, ..] = can_message {
            let sensor_kind = protocol::SensorKind::from_u8(sensor_kind)
                .ok_or(protocol::ErrorCode::BadSensorKindParam)?;
            let sensor_config = protocol::SensorConfig::from_bits(sensor_config)
                .ok_or(protocol::ErrorCode::BadSensorConfigParam)?;

            if persist == 1 {
                crate::core::write_sensor_config(flash, flash_offset, sensor_kind, sensor_config)
                    .map_err(|_| protocol::ErrorCode::SensorConfigAlreadyWritten)?;
            }

            self.configure(sensor_kind, sensor_config).await;

            Ok(())
        } else {
            Err(protocol::ErrorCode::MessageTooShort)
        }
    }

    pub async fn configure(&self, kind: protocol::SensorKind, config: protocol::SensorConfig) {
        let enabled = config.contains(protocol::SensorConfig::Enable);
        let subscribed = config.contains(protocol::SensorConfig::Subscribe);
        defmt::info!(
            "configuring sensor as kind={}, enabled={=bool}, subscribed={=bool}",
            kind,
            enabled,
            subscribed,
        );
        let mut state = self.state.lock().await;
        state.enabled = enabled;
        state.subscribed = subscribed;
        match kind {
            protocol::SensorKind::Unassigned => {
                state.make_unassigned();
            }
            protocol::SensorKind::AS5600 => {
                state.make_as5600();
            }
        }
    }

    pub async fn read_angle(&self) -> Result<u16, ()> {
        self.state.lock().await.read_angle().await
    }

    pub async fn poll_forever<S>(&self, readings: ReadingsSender, signaller: S)
    where
        S: status::Signaller,
    {
        let mut prev_angle = 0;
        let mut changed = false;
        let mut error = false;
        loop {
            let delay = if changed {
                changed = false;
                config::POLL_DELAY_CHANGED
            } else if error {
                error = false;
                config::POLL_DELAY_ERROR
            } else {
                config::POLL_DELAY_UNCHANGED
            };
            embassy_time::Timer::after(delay).await;

            let mut state = self.state.lock().await;

            if state.enabled() && state.subscribed() {
                if let Ok(angle) = state.read_angle().await {
                    if (prev_angle as i32 - angle as i32).abs() as u16
                        > config::SUBSCRIPTION_MIN_ANGLE_DELTA
                    {
                        defmt::info!("new angle value: {=u16:#x}", angle);
                        readings.send(angle).await;
                        prev_angle = angle;
                    }
                } else {
                    signaller.signal_error(status::ErrorKind::Sensors).await;
                    error = true;
                }
            }

            // Happens automatically anyway, but keeping it here as a reminder for future refactors
            // so we make sure that the lock is not kept across loop iterations.
            drop(state);
        }
    }
}

impl<'a, I2C> State<'a, I2C>
where
    I2C: i2c::Instance,
{
    fn new(i2c: i2c::I2c<'a, I2C, mode::Async>) -> Self {
        Self {
            enabled: false,
            subscribed: false,
            kind: Some(Driver::Unassigned(i2c)),
        }
    }

    fn enabled(&self) -> bool {
        self.enabled
    }

    fn subscribed(&self) -> bool {
        self.subscribed
    }

    /// Read the current sensor angle, with a range from 0x0000 to 0xffff.
    // TODO: maybe we should return some more detailed error value here
    async fn read_angle(&mut self) -> Result<u16, ()> {
        if let Some(kind) = &mut self.kind {
            match kind {
                Driver::Unassigned(_) => Err(()),
                Driver::AS5600(dev) => {
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

impl<'a, I2C> Driver<'a, I2C>
where
    I2C: i2c::Instance,
{
    fn into_unassigned(self) -> Self {
        match self {
            Driver::Unassigned(i2c) => Driver::Unassigned(i2c),
            Driver::AS5600(dev) => Driver::Unassigned(dev.release()),
        }
    }

    fn into_as5600(self) -> Self {
        match self {
            Driver::Unassigned(i2c) => Driver::AS5600(as5600::asynch::As5600::new(i2c)),
            Driver::AS5600(dev) => Driver::AS5600(dev),
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
