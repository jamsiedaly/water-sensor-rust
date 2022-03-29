//! # GPIO 'Blinky' Example
//!
//! This application demonstrates how to control a GPIO pin on the RP2040.
//!
//! It may need to be adapted to your particular board layout and/or pin assignment.
//!
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

// Some traits we need
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use rp2040_hal::clocks::Clock;
use embedded_hal::adc::OneShot;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Time between reading the sensor and updating the LEDs
const read_delay_ms: u32 = 1000;

/// Custom values used to interpret the signal value returned by the sensor]
/// The lower the value the wetter the soil being measured
const very_dry: u16 = 3750;
const dry: u16 = 3250;
const moist: u16 = 3000;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then toggles a GPIO pin in
/// an infinite loop. If there is an LED connected to that pin, it will blink.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
        .ok()
        .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);

    // Configure GPIO12 to GPIO15 as an output
    let mut led_pin_green = pins.gpio15.into_push_pull_output();
    let mut led_pin_yellow = pins.gpio14.into_push_pull_output();
    let mut led_pin_red_1 = pins.gpio13.into_push_pull_output();
    let mut led_pin_red_2 = pins.gpio12.into_push_pull_output();

    adc.enable_temp_sensor();
    let mut water_sensor = pins.gpio28.into_floating_input();

    let mut previous_reading = 0;
    loop {
        let moisture_level: u16 = adc.read(&mut water_sensor).unwrap();
        if previous_reading != moisture_level {
            previous_reading = moisture_level;
        }
        led_pin_red_2.set_low().unwrap();
        led_pin_red_1.set_low().unwrap();
        led_pin_yellow.set_low().unwrap();
        led_pin_green.set_low().unwrap();
        if moisture_level > very_dry {
            led_pin_red_2.set_high().unwrap();
        } else if moisture_level > dry {
            led_pin_red_1.set_high().unwrap();
        } else if moisture_level > moist {
            led_pin_yellow.set_high().unwrap();
        } else {
            led_pin_green.set_high().unwrap();
        }
        delay.delay_ms(read_delay_ms);
    }
}