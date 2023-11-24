use arrayvec::ArrayVec;
use rp2040_hal::{
    pio::{Rx, Tx, ValidStateMachine},
    usb::UsbBus,
};
use usb_device::prelude::UsbDevice;
use usbd_audio::AudioClass;

/// 2 microphones maximum for now.
pub struct MicrophoneArray<'a, RxSm: ValidStateMachine, TxSm: ValidStateMachine> {
    // TODO clean this up, we're assuming 48khz and 2 channels here
    usb_device: UsbDevice<'a, UsbBus>,
    usb_audio: AudioClass<'a, UsbBus>,
    fifo_rx: Rx<RxSm>,
    fifo_tx: Tx<TxSm>,
    usb_audio_buffers: [ArrayVec<u8, 288>; 2],
    /// 4 samples = 28mm separation at 48khz
    sample_delay: SampleDelay<u32, 4>,
}

impl<'a, RxSm: ValidStateMachine, TxSm: ValidStateMachine> MicrophoneArray<'a, RxSm, TxSm> {
    pub fn new(
        usb_device: UsbDevice<'a, UsbBus>,
        usb_audio: AudioClass<'a, UsbBus>,
        fifo_rx: Rx<RxSm>,
        fifo_tx: Tx<TxSm>,
    ) -> Self {
        Self {
            usb_device,
            usb_audio,
            fifo_rx,
            fifo_tx,
            usb_audio_buffers: [ArrayVec::from([0; 288]), ArrayVec::from([0; 288])],
            sample_delay: SampleDelay::new(),
        }
    }

    pub fn run(&mut self) -> ! {
        let mut front_sample: Option<u32> = None;
        loop {
            if let Some(val) = self.fifo_rx.read() {
                if let Some(front) = front_sample {
                    let back = val;
                    let back_delayed = self.sample_delay.process(val);

                    let additive_channel = front + back;
                    let differential_channel = front.wrapping_sub(back_delayed);

                    self.fifo_tx.write(additive_channel);
                    self.fifo_tx.write(differential_channel);

                    (1..4).for_each(|i| {
                        self.usb_audio_buffers[0].try_push(additive_channel.to_be_bytes()[i]).ok();
                    });
                    (1..4).for_each(|i| {
                        self.usb_audio_buffers[0]
                            .try_push(differential_channel.to_be_bytes()[i])
                            .ok();
                    });

                    front_sample = None;
                } else {
                    front_sample = Some(val);
                };
            }

            if self.usb_audio_buffers[0].is_full() {
                self.usb_audio_buffers[1] = self.usb_audio_buffers[0].clone();
                self.usb_audio_buffers[0].clear();
            }

            if self.usb_device.poll(&mut [&mut self.usb_audio])
                && self.usb_audio_buffers[1].is_full()
            {
                self.usb_audio.write(&self.usb_audio_buffers[1]).ok();
                self.usb_audio_buffers[1].clear();
            };
        }
    }
}

struct SampleDelay<T, const N: usize> {
    buffer: ArrayVec<T, N>,
    position: usize,
}

impl<T: Copy + Default, const N: usize> SampleDelay<T, N> {
    pub fn new() -> Self {
        Self { buffer: ArrayVec::from([T::default(); N]), position: 0 }
    }

    pub fn process(&mut self, input_sample: T) -> T {
        let output_sample = self.buffer[self.position];
        self.buffer[self.position] = input_sample;
        self.position = (self.position + 1) % self.buffer.len();

        output_sample
    }
}
