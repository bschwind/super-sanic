#![no_main]
#![no_std]

use crate::clocks::set_system_clock_exact;
use fugit::RateExtU32;
use panic_reset as _;
use rp2040_hal::{
    gpio::{FunctionPio0, Pin},
    pac,
    pio::PIOExt,
    Watchdog,
};

mod clocks;

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

    // Start Clocks
    let desired_system_clock = 132.MHz();
    set_system_clock_exact(
        desired_system_clock,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let sio = rp2040_hal::Sio::new(pac.SIO);

    let pins =
        rp2040_hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

    let led_pin: Pin<_, FunctionPio0, _> = pins.gpio25.into_function();
    let led_pin_id = led_pin.id().num;

    #[rustfmt::skip]
    let pio_program = pio_proc::pio_asm!(
        ".wrap_target",
        "set pins, 0 [31]",
        "set pins, 0 [31]",
        "set pins, 0 [31]",
        "set pins, 0 [31]",
        "set pins, 0 [31]",
        "set pins, 0 [31]",
        "set pins, 0 [31]",
        "set pins, 0 [31]",
        "set pins, 0 [31]",
        "set pins, 1 [31]",
        ".wrap",
    );

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&pio_program.program).unwrap();
    let (int, frac) = (0, 0); // as slow as possible (0 is interpreted as 65536)

    let (mut sm, _, _) = rp2040_hal::pio::PIOBuilder::from_program(installed)
        .set_pins(led_pin_id, 1)
        .clock_divisor_fixed_point(int, frac)
        .build(sm0);

    sm.set_pindirs([(led_pin_id, rp2040_hal::pio::PinDir::Output)]);

    sm.start();

    loop {
        cortex_m::asm::wfi();
    }
}
