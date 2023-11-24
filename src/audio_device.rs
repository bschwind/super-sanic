use rp2040_hal::usb::UsbBus;
use usb_device::{
    class_prelude::UsbBusAllocator,
    prelude::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_audio::{AudioClass, AudioClassBuilder, Format, StreamConfig, TerminalType};

pub fn init(usb_bus: &UsbBusAllocator<UsbBus>) -> (AudioClass<UsbBus>, UsbDevice<UsbBus>) {
    let usb_audio = AudioClassBuilder::new()
        .input(
            StreamConfig::new_discrete(Format::S24le, 2, &[48000], TerminalType::InMicrophone)
                .unwrap(),
        )
        .build(usb_bus)
        .unwrap();

    let usb_device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .max_packet_size_0(64)
        .manufacturer("tonari")
        .product("Audio port")
        .serial_number("42")
        .build();

    (usb_audio, usb_device)
}
