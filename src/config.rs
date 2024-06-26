use core::ptr;
use embassy_stm32::{flash, time};

// Linker tricks to get unique addresses for these vars
#[link_section = ".appconfig.sensor1"]
static APPCONFIG_SENSOR1: u32 = 0xffff;
#[link_section = ".appconfig.sensor2"]
static APPCONFIG_SENSOR2: u32 = 0xffff;
#[link_section = ".appconfig.can_id"]
static APPCONFIG_CAN_ID: u32 = 0xffff;

// These need to be functions because they need to use the linker to compute some values
pub fn flash_offset_sensor1() -> u32 {
    (ptr::addr_of!(APPCONFIG_SENSOR1) as usize - flash::FLASH_BASE) as u32
}

pub fn flash_offset_sensor2() -> u32 {
    (ptr::addr_of!(APPCONFIG_SENSOR2) as usize - flash::FLASH_BASE) as u32
}

pub fn flash_offset_can_id() -> u32 {
    (ptr::addr_of!(APPCONFIG_CAN_ID) as usize - flash::FLASH_BASE) as u32
}

pub const CAN_BITRATE: u32 = 1_000_000;

/// The default CAN ID to use if no override is stored in flash.
pub const DEFAULT_CAN_ID: u16 = 1;

pub const I2C1_BITRATE: time::Hertz = time::hz(500_000);
pub const I2C1_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_millis(100);
pub const I2C2_BITRATE: time::Hertz = time::hz(500_000);
pub const I2C2_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_millis(100);
pub const WATCHDOG_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_millis(100);
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
