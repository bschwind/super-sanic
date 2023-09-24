#![no_main]
#![no_std]

use panic_reset as _;
use rp2040_hal::{
    gpio::{FunctionPio0, Pin},
    pio::PIOExt,
    Watchdog,
};

use rp2040_hal::pac;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

const EXTERNAL_CRYSTAL_FREQUENCY_HZ: u32 = 12_000_000;

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let _clocks = rp2040_hal::clocks::init_clocks_and_plls(
        EXTERNAL_CRYSTAL_FREQUENCY_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = rp2040_hal::Sio::new(pac.SIO);

    let pins =
        rp2040_hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

    // let mut led_pin = pins.gpio25.into_push_pull_output();
    let led_pin: Pin<_, FunctionPio0, _> = pins.gpio25.into_function();
    let led_pin_id = led_pin.id().num;

    const MAX_DELAY: u8 = 31;
    let mut a = pio::Assembler::<32>::new();
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();
    // Set pin as Out
    a.set(pio::SetDestination::PINDIRS, 1);
    // Define begin of program loop
    a.bind(&mut wrap_target);
    // Set pin low
    a.set_with_delay(pio::SetDestination::PINS, 0, MAX_DELAY);
    // Set pin high
    a.set_with_delay(pio::SetDestination::PINS, 1, MAX_DELAY);
    // Define end of program loop
    a.bind(&mut wrap_source);

    let program = a.assemble_with_wrap(wrap_source, wrap_target);
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program).unwrap();
    let (int, frac) = (0, 0); // as slow as possible (0 is interpreted as 65536)

    let (sm, _, _) = rp2040_hal::pio::PIOBuilder::from_program(installed)
        .set_pins(led_pin_id, 1)
        .clock_divisor_fixed_point(int, frac)
        .build(sm0);

    sm.start();

    loop {
        cortex_m::asm::wfi();
    }
}
