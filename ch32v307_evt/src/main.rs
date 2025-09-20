#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]
#![allow(incomplete_features)]
#![feature(impl_trait_in_assoc_type)]
#![feature(type_alias_impl_trait)]

use defmt_rtt as _; // global logger

use ch32_hal::gpio::{Level, Output};
use ch32_hal::spi::{Config as SpiConfig, Spi};
use ch32_hal::time::Hertz;
use ch32_hal::usb::EndpointDataBuffer64;
use ch32_hal::usbhs::{self, Driver};
use ch32_hal::{self as hal, bind_interrupts, peripherals, Config};
use core::panic::PanicInfo;
use embassy_executor::Spawner;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Config as UsbConfig, UsbDevice};
use heapless::String;
use qingke::riscv;
use static_cell::StaticCell;
use ufmt::uwrite;

use defmt::{error, info};
use embedded_hal_async::spi::SpiBus;

// Simple error type for SPI wrapper
#[derive(Debug)]
struct SpiError;

impl embedded_hal::spi::Error for SpiError {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        embedded_hal::spi::ErrorKind::Other
    }
}

// Wrapper to make blocking SPI work with async SpiBus trait
struct SpiWrapper<T: ch32_hal::spi::Instance, M: ch32_hal::mode::Mode> {
    spi: Spi<'static, T, M>,
}

impl<T: ch32_hal::spi::Instance, M: ch32_hal::mode::Mode> SpiWrapper<T, M> {
    fn new(spi: Spi<'static, T, M>) -> Self {
        Self { spi }
    }
}

impl<T: ch32_hal::spi::Instance, M: ch32_hal::mode::Mode> embedded_hal_async::spi::ErrorType
    for SpiWrapper<T, M>
{
    type Error = SpiError;
}

impl<T: ch32_hal::spi::Instance, M: ch32_hal::mode::Mode> SpiBus<u8> for SpiWrapper<T, M> {
    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        use embedded_hal::spi::SpiBus;
        <Spi<'static, T, M> as embedded_hal::spi::SpiBus<u8>>::read(&mut self.spi, words).map_err(|_| SpiError)
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        use embedded_hal::spi::SpiBus;
        <Spi<'static, T, M> as embedded_hal::spi::SpiBus<u8>>::write(&mut self.spi, words).map_err(|_| SpiError)
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        use embedded_hal::spi::SpiBus;
        <Spi<'static, T, M> as embedded_hal::spi::SpiBus<u8>>::transfer(&mut self.spi, read, write).map_err(|_| SpiError)
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        use embedded_hal::spi::SpiBus;
        <Spi<'static, T, M> as embedded_hal::spi::SpiBus<u8>>::transfer_in_place(&mut self.spi, words).map_err(|_| SpiError)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        use embedded_hal::spi::SpiBus;
        <Spi<'static, T, M> as embedded_hal::spi::SpiBus<u8>>::flush(&mut self.spi).map_err(|_| SpiError)
    }
}

bind_interrupts!(struct Irqs {
    USBHS => usbhs::InterruptHandler<peripherals::USBHS>;
    USBHS_WKUP => usbhs::WakeupInterruptHandler<peripherals::USBHS>;
});

struct UsbResources {
    peripheral: peripherals::USBHS,
    dm: peripherals::PB6,
    dp: peripherals::PB7,
}

struct SpiResources {
    peripheral: peripherals::SPI1,
    clk: peripherals::PA5,
    mosi: peripherals::PA7,
    miso: peripherals::PA6,
    cs: peripherals::PA4,
    led: peripherals::PA0,
}

// According to Serial Flasher Protocol Specification - version 1
#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    // setup clocks
    let cfg = Config {
        rcc: ch32_hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSI,
        ..Default::default()
    };
    let p = hal::init(cfg);

    // Split the peripherals manually
    let usb_resources = UsbResources {
        peripheral: p.USBHS,
        dm: p.PB6,
        dp: p.PB7,
    };

    let spi_resources = SpiResources {
        peripheral: p.SPI1,
        clk: p.PA5,
        mosi: p.PA7,
        miso: p.PA6,
        cs: p.PA4,
        led: p.PA0,
    };

    info!("hello");

    static BUFFER: StaticCell<[EndpointDataBuffer64; 4]> = StaticCell::new();
    let buffer = BUFFER.init(core::array::from_fn(|_| EndpointDataBuffer64::default()));

    let driver = Driver::new(
        usb_resources.peripheral,
        Irqs,
        usb_resources.dp,
        usb_resources.dm,
        buffer,
    );

    // Use a simple UID since CH32 doesn't have the same UID function
    let uid: [u8; 8] = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0];

    static UID_STR: StaticCell<String<16>> = StaticCell::new();
    let uid_str = UID_STR.init(String::<16>::new());
    for byte in uid.iter() {
        uwrite!(uid_str, "{:02X}", *byte).unwrap_or_default();
    }

    let config = {
        let mut config = UsbConfig::new(0x1ced, 0xc0fe);
        config.manufacturer = Some("9elements");
        config.product = Some("ch32v307-prog");
        config.serial_number = Some(uid_str.as_str());
        config
    };

    let mut builder = {
        static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static MSOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

        let builder = embassy_usb::Builder::new(
            driver,
            config,
            CONFIG_DESCRIPTOR.init([0; 256]),
            BOS_DESCRIPTOR.init([0; 256]),
            MSOS_DESCRIPTOR.init([0; 256]),
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
    spawner.spawn(usb_task(usb).expect("Failed to create USB task"));
    spawner
        .spawn(serprog_task(serprog_class, spi_resources).expect("Failed to create serprog task"));

    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await;
    }
}

type CustomUsbDriver = Driver<'static, peripherals::USBHS, 4, 64>;
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
    config.frequency = Hertz::mhz(20); // 12 MHz

    let spi = Spi::new_blocking(r.peripheral, r.clk, r.mosi, r.miso, config);
    let spi_wrapper = SpiWrapper::new(spi);
    let cs = Output::new(r.cs, Level::High, Default::default());
    let led = Output::new(r.led, Level::Low, Default::default());

    // Define a callback function to set the SPI frequency
    let set_freq_cb = move |_spi: &mut SpiWrapper<peripherals::SPI1, ch32_hal::mode::Blocking>,
                            _freq| {
        // Note: CH32 SPI may not support runtime frequency changes like STM32
        // This is a placeholder for the interface
    };

    loop {
        class.wait_connection().await;
        let serprog = serprog::Serprog::new(spi_wrapper, cs, led, class, Some(set_freq_cb));
        serprog.run_loop().await
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // Print out the panic info
    error!("Panic occurred: {:?}", info);

    // Reboot the system - CH32 doesn't have SCB, use different reset method
    unsafe {
        riscv::asm::ebreak();
    }
    loop {}
}
