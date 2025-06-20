#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]
#![allow(incomplete_features)]
#![feature(impl_trait_in_assoc_type)]
#![feature(type_alias_impl_trait)]

use {
    assign_resources::assign_resources,
    defmt::*,
    defmt_rtt as _,
    embassy_executor::Spawner,
    embassy_stm32::{
        bind_interrupts,
        gpio::{Level, Output, Speed},
        mode::Async,
        ospi::{
            AddressSize, Config as OspiConfig, DummyCycles, MemorySize, MemoryType, Ospi,
            OspiError, OspiWidth, TransferConfig,
        },
        peripherals::{self, USB_OTG_HS},
        time::Hertz,
        uid::uid_hex,
        usb::{Config as UsbDrvConfig, Driver, InterruptHandler as USBInterruptHandler},
        Config as BoardConfig,
    },
    embassy_time::Timer,
    embassy_usb::{
        class::cdc_acm::{CdcAcmClass, State},
        Config as UsbConfig, UsbDevice,
    },
    panic_probe as _,
    serprog::{Address, MultiIOMode, MultiIOSpi, MultiIOTransaction, NoCs, NoSpi},
    static_cell::StaticCell,
};

bind_interrupts!(struct UsbIrqs {
    OTG_HS => USBInterruptHandler<USB_OTG_HS>;
});

assign_resources! {
    ospi: OspiResources{
        peripheral: OCTOSPI1,
        ncs1: PA2,
        ncs2: PA0,
        clk: PA3,
        nclk: PB12,
        dqs: PA1,
        io0: PB1,
        io1: PB0,
        io2: PA7,
        io3: PA6,
        io4: PC1,
        io5: PC2,
        io6: PC3,
        io7: PC0,
        dma: GPDMA1_CH7,
    }
    usb: UsbResources{
        peripheral: USB_OTG_HS,
        dm: PA11,
        dp: PA12,
    }
}

// Wrapper struct to work around orphan rules
pub struct OspiWrapper<'d, T: embassy_stm32::ospi::Instance>(Ospi<'d, T, Async>);

impl<'d, T: embassy_stm32::ospi::Instance> OspiWrapper<'d, T> {
    pub fn new(ospi: Ospi<'d, T, Async>) -> Self {
        Self(ospi)
    }

    fn build_transfer_config(
        &self,
        transaction: MultiIOTransaction,
    ) -> Result<TransferConfig, OspiError> {
        let (iwidth, adwidth, dwidth) = match transaction.mode {
            MultiIOMode::SingleIO111 => (OspiWidth::SING, OspiWidth::SING, OspiWidth::SING),
            MultiIOMode::DualOut112 => (OspiWidth::SING, OspiWidth::SING, OspiWidth::DUAL),
            MultiIOMode::DualIO122 => (OspiWidth::SING, OspiWidth::DUAL, OspiWidth::DUAL),
            MultiIOMode::QuadOut114 => (OspiWidth::SING, OspiWidth::SING, OspiWidth::QUAD),
            MultiIOMode::QuadIO144 => (OspiWidth::SING, OspiWidth::QUAD, OspiWidth::QUAD),
            MultiIOMode::QPI444 => (OspiWidth::QUAD, OspiWidth::QUAD, OspiWidth::QUAD),
        };

        let (address, adsize) = match transaction.address {
            Some(Address::Addr24(addr)) => (Some(addr), AddressSize::_24bit),
            Some(Address::Addr32(addr)) => (Some(addr), AddressSize::_32bit),
            None => (None, AddressSize::_24bit), // We don't care about address size
        };

        let dummy = match transaction.dummy_cycles {
            0 => DummyCycles::_0,
            1 => DummyCycles::_1,
            2 => DummyCycles::_2,
            3 => DummyCycles::_3,
            4 => DummyCycles::_4,
            5 => DummyCycles::_5,
            6 => DummyCycles::_6,
            7 => DummyCycles::_7,
            8 => DummyCycles::_8,
            9 => DummyCycles::_9,
            10 => DummyCycles::_10,
            11 => DummyCycles::_11,
            12 => DummyCycles::_12,
            13 => DummyCycles::_13,
            14 => DummyCycles::_14,
            15 => DummyCycles::_15,
            16 => DummyCycles::_16,
            17 => DummyCycles::_17,
            18 => DummyCycles::_18,
            19 => DummyCycles::_19,
            20 => DummyCycles::_20,
            21 => DummyCycles::_21,
            22 => DummyCycles::_22,
            23 => DummyCycles::_23,
            24 => DummyCycles::_24,
            25 => DummyCycles::_25,
            26 => DummyCycles::_26,
            27 => DummyCycles::_27,
            28 => DummyCycles::_28,
            29 => DummyCycles::_29,
            30 => DummyCycles::_30,
            31 => DummyCycles::_31,
            _ => {
                // Return an error for invalid dummy cycles
                return Err(OspiError::InvalidCommand);
            }
        };

        let mut config = TransferConfig {
            iwidth,
            instruction: Some(transaction.opcode as u32),
            adwidth,
            address,
            adsize,
            dwidth,
            dummy,
            ..Default::default()
        };

        // Handle mode byte if requested
        if transaction.mode_byte {
            config.abwidth = dwidth; // Use same width as data for alternate bytes
            config.alternate_bytes = Some(0xff); // Mode byte value, could be parameterized
        }

        Ok(config)
    }
}

impl<'d, T: embassy_stm32::ospi::Instance> MultiIOSpi for OspiWrapper<'d, T> {
    type Error = OspiError;

    const MAX_TRANSACTION_SIZE: usize = 4096;

    async fn reset(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn read(&mut self, transaction: MultiIOTransaction, buf: &mut [u8]) -> Result<(), Self::Error> {
        let config = self.build_transfer_config(transaction)?;
        self.0.read(buf, config).await
    }

    async fn write(&mut self, transaction: MultiIOTransaction, buf: &[u8]) -> Result<(), Self::Error> {
        let config = self.build_transfer_config(transaction)?;
        self.0.write(buf, config).await
    }

    fn supported_modes(&self) -> u8 {
        // Support Single I/O, Dual Output, Dual I/O, Quad Output, Quad I/O, QPI
        // Bit positions correspond to MultiIOMode enum values
        (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5)
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let board_cfg = {
        use embassy_stm32::rcc::*;
        let mut config = BoardConfig::default();
        config.rcc.hse = Some(Hse {
            freq: Hertz(16_000_000), // 16 MHz
            mode: HseMode::Oscillator,
        });
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSE,    // 16 MHz
            prediv: PllPreDiv::DIV2,   // 8 MHz
            mul: PllMul::MUL60,        // 480 MHz
            divp: Some(PllDiv::DIV15), // 32 MHz (for USBOTG)
            divq: Some(PllDiv::DIV10), // 48 MHz (for USB)
            divr: Some(PllDiv::DIV3),  // 160 MHz (for SYS, OctoSPI and ADC)
        });
        config.rcc.pll2 = Some(Pll {
            source: PllSource::HSE,  // 16 MHz
            prediv: PllPreDiv::DIV2, // 8 MHz
            mul: PllMul::MUL25,      // 200 MHz intermediate frequency
            divp: None,
            divq: Some(PllDiv::DIV2), // 200/2 = 100 MHz for pll2q
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.voltage_range = VoltageScale::RANGE1;
        config.rcc.mux.otghssel = mux::Otghssel::PLL1_P;
        config.rcc.mux.octospisel = mux::Octospisel::PLL2_Q;
        config
    };
    let p = embassy_stm32::init(board_cfg);
    let r = split_resources!(p);
    let uid = uid_hex();

    info!("Started Flashcrab {}", uid);

    let mut usb_drv_cfg = UsbDrvConfig::default();
    // Do not enable vbus_detection. This is a safe default that works in all boards.
    // However, if your USB device is self-powered (can stay powered on if USB is unplugged), you need
    // to enable vbus_detection to comply with the USB spec. If you enable it, the board
    // has to support it or USB won't work at all. See docs on `vbus_detection` for details.
    usb_drv_cfg.vbus_detection = false;
    static EP_OUT_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();
    let ep_out_buffer = EP_OUT_BUFFER.init([0; 1024]);
    let driver = Driver::new_hs(
        r.usb.peripheral,
        UsbIrqs,
        r.usb.dp,
        r.usb.dm,
        ep_out_buffer,
        usb_drv_cfg,
    );

    let usb_cfg = {
        let mut config = UsbConfig::new(0x1ced, 0xc0fe);
        config.manufacturer = Some("9elements");
        config.product = Some("Flashcrab");
        config.serial_number = Some(uid_hex());

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
            usb_cfg,
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

    unwrap!(spawner.spawn(usb_task(usb)));
    unwrap!(spawner.spawn(serprog_task(serprog_class, r.ospi)));

    loop {
        Timer::after_millis(1000).await;
    }
}

type CustomUsbDriver = Driver<'static, USB_OTG_HS>;
type CustomUsbDevice = UsbDevice<'static, CustomUsbDriver>;

#[embassy_executor::task]
async fn usb_task(mut usb: CustomUsbDevice) -> ! {
    usb.run().await
}

#[embassy_executor::task]
async fn serprog_task(mut class: CdcAcmClass<'static, CustomUsbDriver>, r: OspiResources) -> ! {
    loop {
        class.wait_connection().await;

        let ospi_cfg = {
            let mut config = OspiConfig::default();
            config.memory_type = MemoryType::Standard;
            config.device_size = MemorySize::_32MiB;
            config.clock_prescaler = 4;
            config.sample_shifting = true;
            config
        };

        let ospi = Ospi::new_octospi(
            r.peripheral,
            r.clk,
            r.io0,
            r.io1,
            r.io2,
            r.io3,
            r.io4,
            r.io5,
            r.io6,
            r.io7,
            r.ncs1,
            r.dma,
            ospi_cfg,
        );

        // Dummy CS and LED pins for the serprog interface
        //let cs = Output::new(r.ncs2, Level::High, Speed::VeryHigh);
        let led = Output::new(r.dqs, Level::Low, Speed::Medium); // Use DQS pin as LED

        let ospi_wrapper = OspiWrapper::new(ospi);
        let serprog = serprog::Serprog::new(None::<NoSpi>, Some(ospi_wrapper), NoCs, led, class);

        serprog.run_loop().await
    }
}
