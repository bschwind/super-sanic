#![no_main]
#![no_std]

use crate::clocks::set_system_clock_exact;
use arrayvec::ArrayVec;
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

// For single-buffered DMA transfers, this only seems to work if it holds
// enough data for 4 sine-wave periods.
static mut AUDIO_BUF: &mut [u32; 9600] = &mut [0; 9600];

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

    // let led_pin: Pin<_, FunctionPio0, _> = pins.gpio25.into_function();

    // PIO Globals
    let (mut pio, sm0, sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // I2S output
    let dac_dout_pin: Pin<_, FunctionPio0, _> = pins.gpio6.into_function();
    let dac_bit_clock_pin: Pin<_, FunctionPio0, _> = pins.gpio7.into_function();
    let dac_left_right_clock_pin: Pin<_, FunctionPio0, _> = pins.gpio8.into_function();

    let dac_dout_pin_id = dac_dout_pin.id().num;
    let dac_bit_clock_pin_id = dac_bit_clock_pin.id().num;
    let dac_left_right_clock_pin_id = dac_left_right_clock_pin.id().num;

    #[rustfmt::skip]
    let dac_pio_program = pio_proc::pio_asm!(
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

    let installed = pio.install(&dac_pio_program.program).unwrap();

    let (mut dac_sm, _fifo_rx, mut fifo_tx) = rp2040_hal::pio::PIOBuilder::from_program(installed)
        .out_pins(dac_dout_pin_id, 1)
        .side_set_pin_base(dac_bit_clock_pin_id)
        .out_shift_direction(rp2040_hal::pio::ShiftDirection::Left)
        .clock_divisor_fixed_point(10, 0)
        .buffers(rp2040_hal::pio::Buffers::OnlyTx)
        .autopull(true)
        .pull_threshold(32)
        .build(sm0);

    dac_sm.set_pindirs([
        (dac_dout_pin_id, rp2040_hal::pio::PinDir::Output),
        (dac_left_right_clock_pin_id, rp2040_hal::pio::PinDir::Output),
        (dac_bit_clock_pin_id, rp2040_hal::pio::PinDir::Output),
    ]);
    // End I2S output

    // I2S Input
    let mic_data_pin: Pin<_, FunctionPio0, _> = pins.gpio10.into_function();
    let mic_bit_clock_pin: Pin<_, FunctionPio0, _> = pins.gpio11.into_function();
    let mic_left_right_clock_pin: Pin<_, FunctionPio0, _> = pins.gpio12.into_function();

    let mic_data_pin_id = mic_data_pin.id().num;
    let mic_bit_clock_pin_id = mic_bit_clock_pin.id().num;
    let mic_left_right_clock_pin_id = mic_left_right_clock_pin.id().num;

    #[rustfmt::skip]
    let mic_pio_program = pio_proc::pio_asm!(
        ".side_set 2",
        "    set x, 30          side 0b00", // side 0bWB - W = Word Clock, B = Bit Clock
        "left_data:",
        "    in pins, 1         side 0b01",
        "    jmp x-- left_data  side 0b00",
        "    in pins 1          side 0b11",
        "    set x, 30          side 0b10",
        "right_data:",
        "    in pins 1          side 0b11",
        "    jmp x-- right_data side 0b10",
        "    in pins 1          side 0b01",
    );

    // let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&mic_pio_program.program).unwrap();

    let (mut mic_sm, mut fifo_rx, _fifo_tx) = rp2040_hal::pio::PIOBuilder::from_program(installed)
        .in_pin_base(mic_data_pin_id)
        .side_set_pin_base(mic_bit_clock_pin_id)
        .in_shift_direction(rp2040_hal::pio::ShiftDirection::Left)
        .clock_divisor_fixed_point(10, 0)
        .buffers(rp2040_hal::pio::Buffers::OnlyRx)
        .autopush(true)
        .push_threshold(32)
        .build(sm1);

    mic_sm.set_pindirs([
        (mic_data_pin_id, rp2040_hal::pio::PinDir::Input),
        (mic_left_right_clock_pin_id, rp2040_hal::pio::PinDir::Output),
        (mic_bit_clock_pin_id, rp2040_hal::pio::PinDir::Output),
    ]);
    // End I2S input

    // let mut tone_generator = ToneGenerator::new(300, 48_000);

    // // Fill the buffer with data once
    // unsafe {
    //     for frame in AUDIO_BUF.chunks_mut(2) {
    //         let val = tone_generator.next() as u32;
    //         frame[0] = val;
    //         frame[1] = val;
    //     }
    // }

    // let mut dma = pac.DMA.split(&mut pac.RESETS);

    // let sm_group = dac_sm.with(mic_sm);
    // sm_group.start();

    let mut buffer: ArrayVec<u32, 2> = ArrayVec::new();

    loop {
        if let Some(val) = fifo_rx.read() {
            buffer.push(val);
        }

        if buffer.len() == 2 {
            let val = buffer.pop() + buffer.pop();
            fifo_tx.write(val);
            fifo_tx.write(val);
        }

        // let mut transfer_config =
        //     rp2040_hal::dma::single_buffer::Config::new(dma.ch0, fifo_rx, fifo_tx);
        // transfer_config.pace(rp2040_hal::dma::Pace::PreferSink);

        // let transfer = transfer_config.start();

        // // Here is where we should fill the buffer with more data while the PIO is outputting audio data.
        // // TODO - Use double-buffered DMA

        // let (ch0, old_fifo_rx, old_fifo_tx) = transfer.wait();
        // dma.ch0 = ch0;
        // fifo_tx = old_fifo_tx;
        // fifo_rx = old_fifo_rx;
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

        libm::floor(val * 0.5 * i32::MAX as f64) as i32
    }
}
