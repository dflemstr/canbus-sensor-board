use embassy_stm32::{flash, time};

// Store the values at the end of flash; very likely won't over-write application code.  But we
// could tweak the linker script to not place code here if we want to be fool-proof.
pub const SENSOR1_CONFIG_FLASH_OFFSET: u32 = (flash::FLASH_SIZE - 3 * flash::WRITE_SIZE) as u32;
pub const SENSOR2_CONFIG_FLASH_OFFSET: u32 = (flash::FLASH_SIZE - 2 * flash::WRITE_SIZE) as u32;
pub const CAN_ID_FLASH_OFFSET: u32 = (flash::FLASH_SIZE - 1 * flash::WRITE_SIZE) as u32;

pub const CAN_BITRATE: u32 = 1_000_000;

/// The default CAN ID to use if no override is stored in flash.
pub const DEFAULT_CAN_ID: u16 = 1;

pub const I2C1_BITRATE: time::Hertz = time::hz(500_000);
pub const I2C1_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_millis(100);
pub const I2C2_BITRATE: time::Hertz = time::hz(500_000);
pub const I2C2_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_millis(100);
pub const WATCHDOG_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_micros(1000);
pub const WATCHDOG_LED_BLINK_INTERVAL: embassy_time::Duration =
    embassy_time::Duration::from_secs(5);
pub const WATCHDOG_LED_ON_DURATION: embassy_time::Duration =
    embassy_time::Duration::from_millis(100);

/// Time between sensor polls if the value recently changed
pub const POLL_DELAY_CHANGED: embassy_time::Duration = embassy_time::Duration::from_millis(10);

/// Time between sensor polls if the value has mostly remained the same
pub const POLL_DELAY_UNCHANGED: embassy_time::Duration = embassy_time::Duration::from_millis(100);

/// Time between sensor polls if the sensor previously reported an error
pub const POLL_DELAY_ERROR: embassy_time::Duration = embassy_time::Duration::from_millis(1000);

/// The delta in angle (between 0x0000-0xffff) that needs to happen before we send out a new
/// "subscription" message.
pub const SUBSCRIPTION_MIN_ANGLE_DELTA: u16 = 0x0080;

/// Max number of samples to buffer in queue to be sent over the CAN bus.
pub const SAMPLE_BUFFER_SIZE: usize = 64;

/// Delay between error blinks during error conditions
pub const ERROR_DELAY_BETWEEN_BLINKS: embassy_time::Duration =
    embassy_time::Duration::from_millis(300);
/// Delay between each group of blinks during error conditions
pub const ERROR_DELAY_BETWEEN_BLINK_GROUPS: embassy_time::Duration =
    embassy_time::Duration::from_millis(700);
/// Number of times we will blink out the same error code before moving on.
pub const ERROR_NUM_BLINK_GROUPS: u32 = 3;
