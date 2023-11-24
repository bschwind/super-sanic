use arrayvec::ArrayVec;
use rp2040_hal::{
    pio::{Rx, Tx, ValidStateMachine},
    usb::UsbBus,
};
use usb_device::prelude::UsbDevice;
use usbd_audio::AudioClass;

/// 2 microphones maximum for now.
pub struct MicrophoneArray<'a, RxSm: ValidStateMachine, TxSm: ValidStateMachine> {
    usb_device: UsbDevice<'a, UsbBus>,
    usb_audio: AudioClass<'a, UsbBus>,
    buffer: ArrayVec<u32, 2>,
    fifo_rx: Rx<RxSm>,
    fifo_tx: Tx<TxSm>,
    usb_audio_buffers: [ArrayVec<u8, 144>; 2],
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
            buffer: ArrayVec::new(),
            fifo_rx,
            fifo_tx,
            usb_audio_buffers: [ArrayVec::from([0; 144]), ArrayVec::from([0; 144])],
        }
    }

    pub fn run(&mut self) -> ! {
        loop {
            if let Some(val) = self.fifo_rx.read() {
                match self.buffer.len() {
                    0 => self.buffer.push(val),
                    // 1 => buffer.push(back_mic_buffer.process(val)),
                    1 => self.buffer.push(val),
                    _ => (),
                }
            }

            if self.buffer.len() == 2 {
                let front = self.buffer.pop().unwrap();
                let back = self.buffer.pop().unwrap();
                let val = front + back;
                // let val = front.wrapping_sub(back);
                self.fifo_tx.write(val);
                self.fifo_tx.write(val);

                let sample = val.to_le_bytes();
                self.usb_audio_buffers[0].try_push(sample[1]).ok();
                self.usb_audio_buffers[0].try_push(sample[2]).ok();
                self.usb_audio_buffers[0].try_push(sample[3]).ok();
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
