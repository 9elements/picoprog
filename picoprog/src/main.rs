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
use embassy_rp::gpio::{Flex, Level, Output, Pull};
use embassy_rp::peripherals::{self, PIO0, SPI0, USB};
use embassy_rp::pio::InterruptHandler as PIOInterruptHandler;
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_rp::usb::{Driver, InterruptHandler as USBInterruptHandler};
use embassy_rp::Peri;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Config as UsbConfig, UsbDevice};
use embedded_hal_async::spi::SpiBus;
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
        CdcAcmClass::new(&mut builder, state, 64)
    };

    let serprog_class = {
        static STATE: StaticCell<State> = StaticCell::new();
        let state = STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, 64)
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

#[embassy_executor::task]
async fn serprog_task(class: CdcAcmClass<'static, CustomUsbDriver>, r: SpiResources) -> ! {
    let mut config = SpiConfig::default();
    config.frequency = 12_000_000; // 12 MHz

    // Capture pin numbers for idle/active reconfiguration
    use embassy_rp::gpio::Pin as _;
    let clk_pin_num = r.clk.pin();
    let mosi_pin_num = r.mosi.pin();
    let miso_pin_num = r.miso.pin();
    let cs_pin_num = r.cs.pin();

    let spi = Spi::new(
        r.peripheral,
        r.clk,
        r.mosi,
        r.miso,
        r.mosi_dma,
        r.miso_dma,
        config,
    );
    // Local wrapper types to satisfy orphan rules and implement SerprogSpi here.
    struct SpiWrapper<'d> {
        spi: Spi<'d, SPI0, embassy_rp::spi::Async>,
        cs: Output<'static>,
        clk_pin: u8,
        mosi_pin: u8,
        miso_pin: u8,
        cs_pin: u8,
    }

    struct SpiWrapperIdle<'d> {
        spi: Spi<'d, SPI0, embassy_rp::spi::Async>,
        clk_pin: u8,
        mosi_pin: u8,
        miso_pin: u8,
        cs_pin: u8,
    }

    impl<'d> serprog::SerprogSpiIdle<u8> for SpiWrapperIdle<'d> {
        type Active = SpiWrapper<'d>;
        fn active(self) -> Self::Active {
            // Reconfigure SPI pins back to peripheral function.
            fn set_spi_funcsel(pin: u8) {
                use embassy_rp::pac;
                let gpio = pac::IO_BANK0.gpio(pin as _);
                gpio.ctrl().write(|w| w.set_funcsel(1));
                let pads = pac::PADS_BANK0.gpio(pin as _);
                pads.write(|w| {
                    #[cfg(feature = "_rp235x")]
                    w.set_iso(false);
                    w.set_schmitt(true);
                    w.set_slewfast(false);
                    w.set_ie(true);
                    w.set_od(false);
                    w.set_pue(false);
                    w.set_pde(false);
                });
            }

            set_spi_funcsel(self.clk_pin);
            set_spi_funcsel(self.mosi_pin);
            set_spi_funcsel(self.miso_pin);

            // Recreate CS output as High (inactive).
            let cs =
                unsafe { Output::new(embassy_rp::gpio::AnyPin::steal(self.cs_pin), Level::High) };

            SpiWrapper {
                spi: self.spi,
                cs,
                clk_pin: self.clk_pin,
                mosi_pin: self.mosi_pin,
                miso_pin: self.miso_pin,
                cs_pin: self.cs_pin,
            }
        }
    }

    impl<'d> serprog::SerprogSpi<u8> for SpiWrapper<'d> {
        type Error = <Spi<'d, SPI0, embassy_rp::spi::Async> as embedded_hal::spi::ErrorType>::Error;
        type Idle = SpiWrapperIdle<'d>;

        fn change_freq(&mut self, freq: u32) {
            self.spi.set_frequency(freq);
        }

        fn get_freq(&self) -> u32 {
            0
        }

        fn cs_set_low(&mut self) -> Result<(), ()> {
            self.cs.set_low();
            Ok(())
        }

        fn cs_set_high(&mut self) -> Result<(), ()> {
            self.cs.set_high();
            Ok(())
        }

        async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
            <Spi<'d, SPI0, embassy_rp::spi::Async> as SpiBus<u8>>::read(&mut self.spi, words).await
        }

        async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
            <Spi<'d, SPI0, embassy_rp::spi::Async> as SpiBus<u8>>::write(&mut self.spi, words).await
        }

        async fn flush(&mut self) -> Result<(), Self::Error> {
            <Spi<'d, SPI0, embassy_rp::spi::Async> as SpiBus<u8>>::flush(&mut self.spi).await
        }

        fn idle(self) -> Self::Idle {
            // Set all used GPIOs to input (SIO function, no pulls)
            fn set_gpio_input(pin: u8) {
                unsafe {
                    let mut flex = Flex::new(embassy_rp::gpio::AnyPin::steal(pin));
                    flex.set_as_input();
                    flex.set_pull(Pull::None);
                }
            }

            // Drop CS output to free the pin, then set as input
            let cs_pin = self.cs_pin;
            core::mem::drop(self.cs);
            set_gpio_input(cs_pin);

            set_gpio_input(self.clk_pin);
            set_gpio_input(self.mosi_pin);
            set_gpio_input(self.miso_pin);

            SpiWrapperIdle {
                spi: self.spi,
                clk_pin: self.clk_pin,
                mosi_pin: self.mosi_pin,
                miso_pin: self.miso_pin,
                cs_pin,
            }
        }
    }
    // Create CS and LED outputs
    let cs = unsafe { Output::new(embassy_rp::gpio::AnyPin::steal(cs_pin_num), Level::High) };
    let led = Output::new(r.led, Level::Low);

    let serprog = serprog::Serprog::new(
        SpiWrapper {
            spi,
            cs,
            clk_pin: clk_pin_num,
            mosi_pin: mosi_pin_num,
            miso_pin: miso_pin_num,
            cs_pin: cs_pin_num,
        },
        led,
        class,
    );
    serprog.run_loop().await
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // Print out the panic info
    log::error!("Panic occurred: {:?}", info);

    // Reboot the system
    SCB::sys_reset();
}
