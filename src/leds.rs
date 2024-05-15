use crate::config;
use embassy_stm32::gpio;
use embassy_sync::blocking_mutex::raw;
use embassy_sync::mutex;

pub type Led = mutex::Mutex<raw::ThreadModeRawMutex, gpio::Output<'static>>;

/// Convenience wrapper around the available LEDs.
#[derive(Clone, Copy)]
pub struct Leds<'a> {
    r_led: &'a Led,
    y_led: &'a Led,
    g_led: &'a Led,
}

impl<'a> Leds<'a> {
    pub fn new(r_led: &'a Led, y_led: &'a Led, g_led: &'a Led) -> Self {
        Self {
            r_led,
            y_led,
            g_led,
        }
    }
}

impl Leds<'static> {
    pub fn show_startup_blinkenlights(&self, spawner: embassy_executor::Spawner) {
        defmt::unwrap!(spawner.spawn(show_startup_blinkenlights(*self)));
    }
    pub fn show_error_code(&self, spawner: embassy_executor::Spawner, blinks: u32) {
        defmt::unwrap!(spawner.spawn(show_error_code(*self, blinks)))
    }
}

#[embassy_executor::task(pool_size = 4)]
async fn show_error_code(leds: Leds<'static>, blinks: u32) {
    // Lock LED so that other concurrent LED patterns don't mess up our error code.
    let mut r_led = leds.r_led.lock().await;
    for _ in 0..config::ERROR_NUM_BLINK_GROUPS {
        for _ in 0..blinks {
            r_led.set_level(gpio::Level::High);
            embassy_time::Timer::after(config::ERROR_DELAY_BETWEEN_BLINKS).await;
            r_led.set_level(gpio::Level::Low);
            embassy_time::Timer::after(config::ERROR_DELAY_BETWEEN_BLINKS).await;
        }
        embassy_time::Timer::after(config::ERROR_DELAY_BETWEEN_BLINK_GROUPS).await;
    }
}

#[embassy_executor::task]
async fn show_startup_blinkenlights(leds: Leds<'static>) {
    let d = config::STARTUP_BLINKENLIGHTS_DELAY;

    for _ in 0..3 {
        // Re-lock each led because it's not crucial to get through these, and we rather want
        // to prioritize an error code or similar getting through if there is one.
        leds.g_led.lock().await.set_level(gpio::Level::High);
        embassy_time::Timer::after(d).await;
        leds.y_led.lock().await.set_level(gpio::Level::High);
        embassy_time::Timer::after(d).await;
        leds.r_led.lock().await.set_level(gpio::Level::High);
        embassy_time::Timer::after(d).await;
        leds.g_led.lock().await.set_level(gpio::Level::Low);
        embassy_time::Timer::after(d).await;
        leds.y_led.lock().await.set_level(gpio::Level::Low);
        embassy_time::Timer::after(d).await;
        leds.r_led.lock().await.set_level(gpio::Level::Low);
        embassy_time::Timer::after(d).await;
    }
}
