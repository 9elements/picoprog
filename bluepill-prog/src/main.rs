#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]
#![allow(incomplete_features)]
#![feature(impl_trait_in_assoc_type)]
#![feature(type_alias_impl_trait)]

use defmt_rtt as _; // global logger

use assign_resources::assign_resources;
use core::panic::PanicInfo;
use cortex_m::peripheral::SCB;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::{self, USB};
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::usb::{Driver, InterruptHandler as USBInterruptHandler};
use embassy_stm32::Peri;
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Config as UsbConfig, UsbDevice};
use heapless::String;
use static_cell::StaticCell;
use ufmt::uwrite;

use defmt::{error, info};

bind_interrupts!(struct Irqs {
    USB_LP_CAN1_RX0 => USBInterruptHandler<USB>;
});

assign_resources! {
    usb: UsbResources {
        peripheral: USB,
        dm: PA11,
        dp: PA12,
    }
    spi: SpiResources{
        peripheral: SPI1,
        clk: PA5,
        mosi: PA7,
        mosi_dma: DMA1_CH3,
        miso: PA6,
        miso_dma: DMA1_CH2,
        cs: PA4,
        led: PC13,
    }
}

// According to Serial Flasher Protocol Specification - version 1
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            // Oscillator for bluepill, Bypass for nucleos.
            mode: HseMode::Oscillator,
        });
        config.rcc.pll = Some(Pll {
            src: PllSource::HSE,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL9,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
    }
    let p = embassy_stm32::init(config);
    let mut r = split_resources!(p);

    {
        // BluePill board has a pull-up resistor on the D+ line.
        // Pull the D+ pin down to send a RESET condition to the USB bus.
        // This forced reset is needed only for development, without it host
        // will not reset your device when you upload new firmware.
        let _dp = Output::new(r.usb.dp.reborrow(), Level::Low, Speed::Low);
        Timer::after_millis(10).await;
    }

    info!("hello");

    let driver = Driver::new(r.usb.peripheral, Irqs, r.usb.dp, r.usb.dm);

    let uid: [u8; 8] = embassy_stm32::uid::uid()[..8].try_into().unwrap();

    static UID_STR: StaticCell<String<16>> = StaticCell::new();
    let uid_str = UID_STR.init(String::<16>::new());
    for byte in uid.iter() {
        uwrite!(uid_str, "{:02X}", *byte).unwrap_or_default();
    }

    let config = {
        let mut config = UsbConfig::new(0x1ced, 0xc0fe);
        config.manufacturer = Some("9elements");
        config.product = Some("bluepill-prog");
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

    let serprog_class = {
        static STATE: StaticCell<State> = StaticCell::new();
        let state = STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, 64)
    };

    let usb = builder.build();

    // We can't really recover here so just unwrap
    spawner.spawn(usb_task(usb).unwrap());
    spawner.spawn(serprog_task(serprog_class, r.spi).unwrap());

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
            EndpointError::BufferOverflow => defmt::panic!("USB buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

#[embassy_executor::task]
async fn usb_task(mut usb: CustomUsbDevice) -> ! {
    usb.run().await
}

#[embassy_executor::task]
async fn serprog_task(mut class: CdcAcmClass<'static, CustomUsbDriver>, r: SpiResources) -> ! {
    let mut config = SpiConfig::default();
    config.frequency = Hertz(12_000_000); // 12 MHz

    let spi = Spi::new(
        r.peripheral,
        r.clk,
        r.mosi,
        r.miso,
        r.mosi_dma,
        r.miso_dma,
        config,
    );
    let cs = Output::new(r.cs, Level::High, Speed::Low);
    let led = Output::new(r.led, Level::Low, Speed::Low);

    // Define a callback function to set the SPI frequency
    let set_freq_cb = move |spi: &mut Spi<'_, embassy_stm32::mode::Async>, freq| {
        let mut config = SpiConfig::default();
        config.frequency = Hertz(freq);
        let _ = spi.set_config(&config);
    };

    loop {
        class.wait_connection().await;
        let serprog = serprog::Serprog::new(spi, cs, led, class, Some(set_freq_cb));
        serprog.run_loop().await
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // Print out the panic info
    error!("Panic occurred: {:?}", info);

    // Reboot the system
    SCB::sys_reset();
}
