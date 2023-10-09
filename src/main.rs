#![no_main]
#![no_std]

use crate::clocks::set_system_clock_exact;
use fugit::RateExtU32;
use panic_reset as _;
use rp2040_hal::{
    dma::DMAExt,
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

static mut AUDIO_BUF: &mut [u32; 960] = &mut [0; 960];

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // Start Clocks
    // Found with: https://github.com/bschwind/rp2040-clock-calculator
    let desired_system_clock = 61440000.Hz();
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
    let dout_pin: Pin<_, FunctionPio0, _> = pins.gpio6.into_function();
    let bit_clock_pin: Pin<_, FunctionPio0, _> = pins.gpio7.into_function();
    let left_right_clock_pin: Pin<_, FunctionPio0, _> = pins.gpio8.into_function();

    let led_pin_id = led_pin.id().num;
    let dout_pin_id = dout_pin.id().num;
    let left_right_clock_pin_id = left_right_clock_pin.id().num;
    let bit_clock_pin_id = bit_clock_pin.id().num;

    #[rustfmt::skip]
    let pio_program = pio_proc::pio_asm!(
        ".side_set 2",
        "    set x, 30          side 0b01", // side 0bWB - W = Word Clock, B = Bit Clock
        "left_data:",
        "    out pins, 1        side 0b00",
        "    jmp x-- left_data  side 0b01",
        "    out pins 1         side 0b10",
        "    set x, 30          side 0b11",
        "right_data:",
        "    out pins 1         side 0b10",
        "    jmp x-- right_data side 0b11",
        "    out pins 1         side 0b00",
    );

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&pio_program.program).unwrap();

    let (mut sm, _fifo_rx, mut fifo_tx) = rp2040_hal::pio::PIOBuilder::from_program(installed)
        .out_pins(dout_pin_id, 1)
        .side_set_pin_base(bit_clock_pin_id)
        .out_shift_direction(rp2040_hal::pio::ShiftDirection::Left)
        .clock_divisor_fixed_point(10, 0)
        .buffers(rp2040_hal::pio::Buffers::OnlyTx)
        .autopull(true)
        .pull_threshold(32)
        .build(sm0);

    sm.set_pindirs([
        (led_pin_id, rp2040_hal::pio::PinDir::Output),
        (dout_pin_id, rp2040_hal::pio::PinDir::Output),
        (left_right_clock_pin_id, rp2040_hal::pio::PinDir::Output),
        (bit_clock_pin_id, rp2040_hal::pio::PinDir::Output),
    ]);

    let mut tone_generator = ToneGenerator::new(300, 48_000);

    // Fill the buffer with data once
    unsafe {
        for frame in AUDIO_BUF.chunks_mut(2) {
            let val = tone_generator.next() as u32;
            frame[0] = val;
            frame[1] = val;
        }
    }

    let mut dma = pac.DMA.split(&mut pac.RESETS);

    sm.start();

    loop {
        let mut transfer_config =
            rp2040_hal::dma::single_buffer::Config::new(dma.ch0, unsafe { &*AUDIO_BUF }, fifo_tx);
        transfer_config.pace(rp2040_hal::dma::Pace::PreferSink);

        let transfer = transfer_config.start();

        // Here is where we should fill the buffer with more data while the PIO is outputting audio data.
        // TODO - Use double-buffered DMA

        let (ch0, _sent_buf, old_fifo_tx) = transfer.wait();
        dma.ch0 = ch0;
        fifo_tx = old_fifo_tx;
    }
}

struct ToneGenerator {
    delta: f64,
    current: f64,
}

impl ToneGenerator {
    pub fn new(frequency_hz: u32, sample_rate: u32) -> Self {
        let delta = (frequency_hz as f64 * core::f64::consts::TAU) / sample_rate as f64;

        Self { delta, current: 0.0 }
    }

    pub fn next(&mut self) -> i32 {
        let val = libm::sin(self.current);
        self.current += self.delta;

        if self.current > core::f64::consts::TAU {
            self.current -= core::f64::consts::TAU;
        }

        libm::floor(val * i32::MAX as f64) as i32
    }
}
