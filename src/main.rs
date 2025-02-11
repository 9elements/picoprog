#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]
#![allow(incomplete_features)]
#![feature(impl_trait_in_assoc_type)]
#![feature(type_alias_impl_trait)]

use assign_resources::assign_resources;
use core::panic::PanicInfo;
use cortex_m::peripheral::SCB;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::flash::{Async, Flash};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{self, PIO0, SPI0, USB};
use embassy_rp::pio::InterruptHandler as PIOInterruptHandler;
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_rp::usb::{Driver, InterruptHandler as USBInterruptHandler};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Config as UsbConfig, UsbDevice};
use embassy_usb_logger::with_class;
use heapless::String;
use static_cell::StaticCell;
use ufmt::uwrite;

mod serprog;
mod uart;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => USBInterruptHandler<USB>;
    PIO0_IRQ_0 => PIOInterruptHandler<PIO0>;
});

assign_resources! {
    uart: UartResources{
        peripheral: PIO0,
        tx: PIN_0,
        rx: PIN_1,
    }
    spi: SpiResources{
        peripheral: SPI0,
        clk: PIN_2,
        mosi: PIN_3,
        mosi_dma: DMA_CH2,
        miso: PIN_4,
        miso_dma: DMA_CH3,
        cs: PIN_5,
        led: PIN_25,
    }
}

const FLASH_SIZE: usize = 2 * 1024 * 1024;

// According to Serial Flasher Protocol Specification - version 1

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let r = split_resources!(p);
    let driver = Driver::new(p.USB, Irqs);

    let mut flash = Flash::<_, Async, FLASH_SIZE>::new(p.FLASH, p.DMA_CH4);
    let mut uid: [u8; 8] = [0; 8];
    flash.blocking_unique_id(&mut uid).unwrap_or_default();

    static UID_STR: StaticCell<String<16>> = StaticCell::new();
    let uid_str = UID_STR.init(String::<16>::new());
    for byte in uid.iter() {
        uwrite!(uid_str, "{:02X}", *byte).unwrap_or_default();
    }

    let config = {
        let mut config = UsbConfig::new(0x1ced, 0xc0fe);
        config.manufacturer = Some("9elements");
        config.product = Some("Picoprog");
        config.serial_number = Some(uid_str.as_str());
        config.max_power = 100;
        config.max_packet_size_0 = 64;

        // Required for windows compatibility.
        // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
        config.device_class = 0xEF;
        config.device_sub_class = 0x02;
        config.device_protocol = 0x01;
        config.composite_with_iads = true;
        config
    };

    let mut builder = {
        static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

        let builder = embassy_usb::Builder::new(
            driver,
            config,
            CONFIG_DESCRIPTOR.init([0; 256]),
            BOS_DESCRIPTOR.init([0; 256]),
            &mut [], // no msos descriptors
            CONTROL_BUF.init([0; 64]),
        );
        builder
    };

    let logger_class = {
        static STATE: StaticCell<State> = StaticCell::new();
        let state = STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, 64)
    };

    let uart_class = {
        static STATE: StaticCell<State> = StaticCell::new();
        let state = STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, 64)
    };

    let serprog_class = {
        static STATE: StaticCell<State> = StaticCell::new();
        let state = STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, 64)
    };

    let usb = builder.build();
    // We can't really recover here so just unwrap
    spawner.spawn(usb_task(usb)).unwrap();
    spawner.spawn(logger_task(logger_class)).unwrap();
    spawner.spawn(uart::uart_task(uart_class, r.uart)).unwrap();
    spawner.spawn(serprog_task(serprog_class, r.spi)).unwrap();

    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await;
    }
}

type CustomUsbDriver = Driver<'static, USB>;
type CustomUsbDevice = UsbDevice<'static, CustomUsbDriver>;

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("USB buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

#[embassy_executor::task]
async fn usb_task(mut usb: CustomUsbDevice) -> ! {
    usb.run().await
}

#[embassy_executor::task]
async fn logger_task(class: CdcAcmClass<'static, CustomUsbDriver>) {
    with_class!(1024, log::LevelFilter::Info, class).await
}

#[embassy_executor::task]
async fn serprog_task(mut class: CdcAcmClass<'static, CustomUsbDriver>, r: SpiResources) -> ! {
    let mut config = SpiConfig::default();
    config.frequency = 12_000_000; // 12 MHz

    let spi = Spi::new(
        r.peripheral,
        r.clk,
        r.mosi,
        r.miso,
        r.mosi_dma,
        r.miso_dma,
        config,
    );
    let cs = Output::new(r.cs, Level::High);
    let led = Output::new(r.led, Level::Low);

    let set_freq_cb = move |spi: &mut Spi<'_, SPI0, embassy_rp::spi::Async>, freq| {
        spi.set_frequency(freq);
    };

    let mut serprog = serprog::Serprog::new(spi, cs, led, Some(set_freq_cb));
    let mut buf = [0; 64];

    loop {
        class.wait_connection().await;
        if let Err(e) = class.read_packet(&mut buf).await {
            log::error!("Error reading packet: {:?}", e);
            continue;
        }
        let cmd = serprog::SerprogCommand::try_from(buf[0]).unwrap_or(serprog::SerprogCommand::Nop);
        serprog.handle_command(cmd, &mut class, &mut buf).await;
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // Print out the panic info
    log::error!("Panic occurred: {:?}", info);

    // Reboot the system
    SCB::sys_reset();
}
