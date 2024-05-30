#![no_std]
#![no_main]

use embassy_stm32::{can, flash, gpio, i2c, pac, peripherals, wdg};
use embassy_sync::mutex;
// For link-time dependencies:
use {defmt_rtt as _, panic_probe as _};

mod config;
mod core;
mod leds;
mod protocol;
mod sensors;
mod status;

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

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    // Store these as static so that they get `'static` lifetime and survive when `main()` exits
    static LED: static_cell::StaticCell<leds::Led> = static_cell::StaticCell::new();
    static SENSOR1: static_cell::StaticCell<sensors::Sensor1> = static_cell::StaticCell::new();
    static SENSOR1_CHANNEL: sensors::ReadingsChannel = sensors::ReadingsChannel::new();
    static SENSOR2: static_cell::StaticCell<sensors::Sensor2> = static_cell::StaticCell::new();
    static SENSOR2_CHANNEL: sensors::ReadingsChannel = sensors::ReadingsChannel::new();
    static WDG: static_cell::StaticCell<wdg::IndependentWatchdog<peripherals::IWDG>> =
        static_cell::StaticCell::new();

    // No special requirements, e.g. no external crystal
    let p = embassy_stm32::init(embassy_stm32::Config::default());

    // Set canbus alternate pin mapping to B8/B9
    pac::AFIO.mapr().modify(|w| w.set_can1_remap(2));

    let flash = flash::Flash::new_blocking(p.FLASH);

    let led = LED.init(mutex::Mutex::new(gpio::Output::new(
        p.PA2,
        gpio::Level::Low,
        gpio::Speed::Low,
    )));

    let leds = leds::Leds::new(led);

    let can = can::Can::new(p.CAN, p.PB8, p.PB9, Irqs);

    let mut i2c_config = i2c::Config::default();
    i2c_config.timeout = config::I2C1_TIMEOUT;
    let i2c1 = i2c::I2c::new(
        p.I2C1,
        p.PB6,
        p.PB7,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH7,
        config::I2C1_BITRATE,
        i2c_config,
    );
    i2c_config.timeout = config::I2C2_TIMEOUT;
    let i2c2 = i2c::I2c::new(
        p.I2C2,
        p.PB10,
        p.PB11,
        Irqs,
        p.DMA1_CH4,
        p.DMA1_CH5,
        config::I2C2_BITRATE,
        i2c_config,
    );

    let sensor1 = SENSOR1.init(sensors::Sensor::new(i2c1));
    let sensor2 = SENSOR2.init(sensors::Sensor::new(i2c2));

    let wdg_timeout_us = (config::WATCHDOG_TIMEOUT.as_micros() * 12 / 10) as u32;
    let wdg = WDG.init(wdg::IndependentWatchdog::new(p.IWDG, wdg_timeout_us));

    let signaller = status::LedsSignaller::new(leds, spawner);

    let core = core::Core::initialize(
        can,
        flash,
        sensor1,
        SENSOR1_CHANNEL.receiver(),
        sensor2,
        SENSOR2_CHANNEL.receiver(),
        signaller,
    )
    .await;

    wdg.unleash();

    defmt::unwrap!(spawner.spawn(poll_sensor1(sensor1, SENSOR1_CHANNEL.sender(), signaller)));
    defmt::unwrap!(spawner.spawn(poll_sensor2(sensor2, SENSOR2_CHANNEL.sender(), signaller)));
    defmt::unwrap!(spawner.spawn(run_core(core)));
    defmt::unwrap!(spawner.spawn(pet_watchdog(wdg, leds)));

    // OK, all tasks started, now we go to sleep. Purely interrupt driven from here.
}

#[embassy_executor::task]
async fn run_core(core: core::Core<'static, status::LedsSignaller>) {
    core.run_forever().await;
}

// Need two copies of this function because it can't be generic. But they can share a generic impl
#[embassy_executor::task]
async fn poll_sensor1(
    sensor1: &'static sensors::Sensor1<'static>,
    readings: sensors::ReadingsSender,
    signaller: status::LedsSignaller,
) {
    sensor1.poll_forever(readings, signaller).await;
}

#[embassy_executor::task]
async fn poll_sensor2(
    sensor2: &'static sensors::Sensor2<'static>,
    readings: sensors::ReadingsSender,
    signaller: status::LedsSignaller,
) {
    sensor2.poll_forever(readings, signaller).await;
}

#[embassy_executor::task]
async fn pet_watchdog(
    wdg: &'static mut wdg::IndependentWatchdog<'static, peripherals::IWDG>,
    leds: leds::Leds<'static>,
) {
    let sleep_delay = config::WATCHDOG_TIMEOUT;
    let pets_per_blink_period =
        config::WATCHDOG_LED_BLINK_INTERVAL.as_micros() / sleep_delay.as_micros();
    let pets_per_led_on = config::WATCHDOG_LED_ON_DURATION.as_micros() / sleep_delay.as_micros();
    defmt::info!(
        "starting to pet watchdog every {=u64:us}s (blink every {=u64:us}s for {=u64:us}s)",
        config::WATCHDOG_TIMEOUT.as_micros(),
        config::WATCHDOG_LED_BLINK_INTERVAL.as_micros(),
        config::WATCHDOG_LED_ON_DURATION.as_micros()
    );
    defmt::debug!("pets_per_blink_period={=u64}", pets_per_blink_period);
    defmt::debug!("pets_per_led_on={=u64}", pets_per_led_on);
    loop {
        for _ in 0..(pets_per_blink_period - pets_per_led_on) {
            wdg.pet();
            embassy_time::Timer::after(sleep_delay).await;
        }
        leds.blink_watchdog(true).await;
        for _ in 0..pets_per_led_on {
            wdg.pet();
            embassy_time::Timer::after(sleep_delay).await;
        }
        leds.blink_watchdog(false).await;
    }
}
