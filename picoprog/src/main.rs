#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]
#![allow(incomplete_features)]
#![feature(impl_trait_in_assoc_type)]
#![feature(type_alias_impl_trait)]

use assign_resources::assign_resources;
use core::panic::PanicInfo;
use cortex_m::peripheral::SCB;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::flash::{Async, Flash};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{self, PIO0, SPI0, USB};
use embassy_rp::pio::InterruptHandler as PIOInterruptHandler;
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_rp::usb::{Driver, InterruptHandler as USBInterruptHandler};
use embassy_rp::Peri;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Config as UsbConfig, UsbDevice};
use heapless::String;
use static_cell::StaticCell;
use ufmt::uwrite;

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
const USB_BUFFER_SIZE: usize = 64;

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

    let uart_class = {
        static STATE: StaticCell<State> = StaticCell::new();
        let state = STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, USB_BUFFER_SIZE.try_into().unwrap())
    };

    let serprog_class = {
        static STATE: StaticCell<State> = StaticCell::new();
        let state = STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, USB_BUFFER_SIZE.try_into().unwrap())
    };

    let usb = builder.build();
    // We can't really recover here so just unwrap
    spawner.spawn(usb_task(usb).unwrap());
    spawner.spawn(uart::uart_task(uart_class, r.uart).unwrap());
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
            EndpointError::BufferOverflow => panic!("USB buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

#[embassy_executor::task]
async fn usb_task(mut usb: CustomUsbDevice) -> ! {
    usb.run().await
}

/// Provider for SPI device that creates SPI on demand
struct SpiProvider {
    peripheral: Peri<'static, SPI0>,
    clk: Peri<'static, peripherals::PIN_2>,
    mosi: Peri<'static, peripherals::PIN_3>,
    miso: Peri<'static, peripherals::PIN_4>,
    mosi_dma: Peri<'static, peripherals::DMA_CH2>,
    miso_dma: Peri<'static, peripherals::DMA_CH3>,
    cs: Peri<'static, peripherals::PIN_5>,
}

/// Reset a GPIO pin to disconnected/high-impedance state on RP2040.
/// This mimics what Flex::drop() does.
fn reset_pin_rp2040(pin: u8) {
    use embassy_rp::pac;
    let io = pac::IO_BANK0;
    let pads = pac::PADS_BANK0;

    // Reset pad control to defaults
    pads.gpio(pin as usize).write(|_| {});

    // Set function select to NULL (disconnected) and reset overrides
    io.gpio(pin as usize).ctrl().write(|w| {
        w.set_funcsel(pac::io::vals::Gpio0ctrlFuncsel::NULL as _);
        w.set_inover(pac::io::vals::Inover::NORMAL);
        w.set_outover(pac::io::vals::Outover::NORMAL);
    });
}

impl serprog::SpiDeviceProvider for SpiProvider {
    type Guard<'a>
        = SpiGuard<'a>
    where
        Self: 'a;

    fn acquire(&mut self, freq_hz: u32) -> Self::Guard<'_> {
        let mut config = SpiConfig::default();
        config.frequency = freq_hz;

        // Reborrow the peripherals - pins are now configured as SPI outputs
        let spi = Spi::new(
            self.peripheral.reborrow(),
            self.clk.reborrow(),
            self.mosi.reborrow(),
            self.miso.reborrow(),
            self.mosi_dma.reborrow(),
            self.miso_dma.reborrow(),
            config,
        );
        let cs = Output::new(self.cs.reborrow(), Level::High);

        SpiGuard {
            spi,
            cs,
            // Store pin numbers for reset on drop
            pin_nums: [2, 3, 4, 5], // CLK=2, MOSI=3, MISO=4, CS=5
        }
    }
}

/// Guard that holds SPI and CS, resetting pins to high-impedance (disconnected) on drop.
struct SpiGuard<'a> {
    spi: Spi<'a, SPI0, embassy_rp::spi::Async>,
    cs: Output<'a>,
    /// Pin numbers to reset on drop: [CLK, MOSI, MISO, CS]
    pin_nums: [u8; 4],
}

impl<'a> serprog::SpiCsGuard for SpiGuard<'a> {
    type Spi = Spi<'a, SPI0, embassy_rp::spi::Async>;
    type Cs = Output<'a>;

    fn spi_cs(&mut self) -> (&mut Self::Spi, &mut Self::Cs) {
        (&mut self.spi, &mut self.cs)
    }
}

impl<'a> Drop for SpiGuard<'a> {
    fn drop(&mut self) {
        // Note: spi and cs are dropped automatically after this function returns.
        // We reset pins here, but the actual SPI peripheral drop happens after.
        // This is fine because we're just resetting GPIO config, and the SPI
        // peripheral doesn't hold any GPIO state that would conflict.

        // Reset all SPI pins to disconnected/high-impedance state
        for &pin in &self.pin_nums {
            reset_pin_rp2040(pin);
        }
    }
}

#[embassy_executor::task]
async fn serprog_task(class: CdcAcmClass<'static, CustomUsbDriver>, r: SpiResources) -> ! {
    let spi_provider = SpiProvider {
        peripheral: r.peripheral,
        clk: r.clk,
        mosi: r.mosi,
        miso: r.miso,
        mosi_dma: r.mosi_dma,
        miso_dma: r.miso_dma,
        cs: r.cs,
    };

    let led = Output::new(r.led, Level::Low);

    serprog::run_loop::<_, _, _, USB_BUFFER_SIZE>(spi_provider, led, class).await
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // Print out the panic info
    log::error!("Panic occurred: {:?}", info);

    // Reboot the system
    SCB::sys_reset();
}
