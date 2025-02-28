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
    embassy_futures::join::join,
    embassy_stm32::{
        bind_interrupts,
        dac::{DacCh1, Mode, Value},
        dma::NoDma,
        gpio::{Level, Output, Speed},
        hash::{Algorithm, DataType, Hash, InterruptHandler as HashInterruptHandler},
        mode::Blocking,
        ospi::{
            AddressSize, Config as OspiConfig, DummyCycles,
            Instance, MemorySize, MemoryType, Ospi, OspiWidth, TransferConfig,
        },
        peripherals::{self, HASH, UART4, USB_OTG_HS},
        time::Hertz,
        uid::uid_hex,
        usart::{Config as UartConfig, InterruptHandler as UartInterruptHandler, Uart},
        usb::{Config as UsbDrvConfig, Driver, InterruptHandler as USBInterruptHandler},
        Config as BoardConfig,
    },
    embassy_time::Timer,
    embassy_usb::{
        class::cdc_acm::{CdcAcmClass, State},
        Config as UsbConfig, UsbDevice,
    },
    panic_probe as _,
    static_cell::StaticCell,
};

bind_interrupts!(struct UsbIrqs {
    OTG_HS => USBInterruptHandler<USB_OTG_HS>;
});

bind_interrupts!(struct HashIrqs {
    HASH => HashInterruptHandler<HASH>;
});

bind_interrupts!(struct UartIrqs {
    UART4 => UartInterruptHandler<UART4>;
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
        dma: GPDMA1_CH0,
    }
    usb: UsbResources{
        peripheral: USB_OTG_HS,
        dm: PA11,
        dp: PA12,
    }
    i2c: I2cResources{
        peripheral: I2C1,
        sda: PB9,
        scl: PB8,
    }
    uart: UartResources{
        peripheral: UART4,
        tx: PC10,
        rx: PC11,
        tx_dma: GPDMA1_CH4,
        rx_dma: GPDMA1_CH5,
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let board_cfg = {
        use embassy_stm32::rcc::*;
        let mut config = BoardConfig::default();
        config.rcc.ls = LsConfig::default_lsi(); // 32 kHz (for DAC)
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
            source: PllSource::HSE,   // 16 MHz
            prediv: PllPreDiv::DIV2,  // 8 MHz
            mul: PllMul::MUL25,       // 200 MHz intermediate frequency
            divp: None,
            divq: Some(PllDiv::DIV2), // 200/2 = 100 MHz for pll2q
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.voltage_range = VoltageScale::RANGE1;
        config.rcc.mux.otghssel = mux::Otghssel::PLL1_P;
        config.rcc.mux.octospisel = mux::Octospisel::PLL2_Q;
        config.rcc.mux.dac1sel = mux::Dacsel::LSI;
        config.rcc.mux.adcdacsel = mux::Adcdacsel::SYS;
        config
    };
    let p = embassy_stm32::init(board_cfg);
    let r = split_resources!(p);
    let uid = uid_hex();

    info!("Started Flashcrab {}", uid);

    let mut usb_drv_cfg = UsbDrvConfig::default();
    usb_drv_cfg.vbus_detection = true;
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

    let flasher_class = {
        static STATE: StaticCell<State> = StaticCell::new();
        let state = STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, 512)
    };

    let mut dac = DacCh1::new(p.DAC1, NoDma, p.PA4);

    dac.set_mode(Mode::NormalExternalBuffered);
    dac.set(Value::Bit8(255)); // 140 = 1.8V, 193 = 2.5V, 255 = 3.3V
    dac.set_enable(true);

    let mut oe = Output::new(p.PC8, Level::Low, Speed::Medium);
    oe.set_high();

    let mut hw_hasher = Hash::new(p.HASH, p.GPDMA1_CH1, HashIrqs);
    let mut context = hw_hasher.start(Algorithm::SHA1, DataType::Width32, None);
    hw_hasher.update_blocking(&mut context, b"Hello, World!");
    let mut hw_digest: [u8; 32] = [0; 32];
    hw_hasher.finish_blocking(context, &mut hw_digest);

    info!("HW Hash: {}", hw_digest);

    unwrap!(spawner.spawn(usb_task(builder.build())));
    unwrap!(spawner.spawn(flasher_task(flasher_class, r.ospi)));

    let mut led_1 = Output::new(p.PC7, Level::Low, Speed::Medium);
    let mut led_2 = Output::new(p.PB7, Level::Low, Speed::Medium);
    let mut led_3 = Output::new(p.PG2, Level::Low, Speed::Medium);

    //let mut sclk = Output::new(p.PA3, Level::Low, Speed::VeryHigh);

    // loop {
    //     sclk.set_high();
    //     Timer::after_nanos(100).await;
    //     sclk.set_low();
    //     Timer::after_nanos(100).await;
    // }
    loop {
        led_1.set_high();
        Timer::after_millis(200).await;
        led_1.set_low();

        led_2.set_high();
        Timer::after_millis(200).await;
        led_2.set_low();

        led_3.set_high();
        Timer::after_millis(200).await;
        led_3.set_low();
    }
}

type CustomUsbDriver = Driver<'static, USB_OTG_HS>;
type CustomUsbDevice = UsbDevice<'static, CustomUsbDriver>;

#[embassy_executor::task]
async fn usb_task(mut usb: CustomUsbDevice) -> ! {
    usb.run().await
}

#[embassy_executor::task]
async fn flasher_task(mut _class: CdcAcmClass<'static, CustomUsbDriver>, r: OspiResources) -> ! {
    let ospi_cfg = {
        let mut config = OspiConfig::default();
        config.memory_type = MemoryType::Standard;
        config.device_size = MemorySize::_32MiB;
        //config.chip_select_high_time = ChipSelectHighTime::_1Cycle;
        config.clock_prescaler = 4;
        config.sample_shifting = true;
        config
    };

    let ospi = Ospi::new_blocking_octospi(
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
        ospi_cfg,
    );

    let mut flash = FlashMemory::new(ospi).await;

    let flash_id = flash.read_id();
    info!("FLASH ID: {=[u8]:x}", flash_id);

    let mut wr_buf = [0u8; 8];
    for i in 0..8 {
        wr_buf[i] = i as u8;
    }
    let mut rd_buf = [0u8; 8];
    flash.erase_sector(0).await;
    flash.write_memory(0, &wr_buf, true).await;
    flash.read_memory(0, &mut rd_buf, true);
    info!("WRITE BUF: {=[u8]:#X}", wr_buf);
    info!("READ BUF: {=[u8]:#X}", rd_buf);

    loop {
        let _flash_id = flash.read_id();
        info!("FLASH ID: {=[u8]:x}", flash_id);
        Timer::after_millis(500).await;
    }
}

#[embassy_executor::task]
async fn uart_task(class: CdcAcmClass<'static, CustomUsbDriver>, r: UartResources) {
    let config = UartConfig::default(); // TODO: make this configurable by reading line coding

    // TODO: Buffers are weird...
    let mut usb_tx_buf = [0; 64];
    let mut usb_rx_buf = [0; 64];
    let mut uart_tx_buf = [0; 1];
    let mut uart_rx_buf = [0; 1];

    let uart = Uart::new(
        r.peripheral,
        r.rx,
        r.tx,
        UartIrqs,
        r.tx_dma,
        r.rx_dma,
        config,
    )
    .unwrap();

    let (mut tx, mut rx) = uart.split();
    let (mut sender, mut receiver) = class.split();

    let tx_future = async {
        loop {
            receiver.wait_connection().await;
            if let Err(_e) = receiver.read_packet(&mut usb_tx_buf).await {
                continue;
            }
            let len = usb_tx_buf.iter().position(|&x| x == 0).unwrap_or(64);
            let mut offset = 0;
            while offset < len {
                let chunk_len = (len - offset).min(1);
                if let Some(slice) = usb_tx_buf.get(offset..offset + chunk_len) {
                    uart_tx_buf[..chunk_len].copy_from_slice(slice);
                    if let Err(_e) = tx.write(&uart_tx_buf[..chunk_len]).await {
                        break;
                    }
                }
                offset += chunk_len;
            }
        }
    };

    let rx_future = async {
        loop {
            sender.wait_connection().await;
            if let Err(_e) = rx.read(&mut uart_rx_buf).await {
                continue;
            }
            let len = uart_rx_buf.iter().position(|&x| x == 0).unwrap_or(1);
            if let Some(slice) = uart_rx_buf.get(..len) {
                usb_rx_buf[..len].copy_from_slice(slice);
                if let Err(_e) = sender.write_packet(&usb_rx_buf[..len]).await {}
            }
        }
    };

    join(tx_future, rx_future).await;
}

const MEMORY_PAGE_SIZE: usize = 8;

const CMD_QUAD_READ: u8 = 0x6B;

const CMD_QUAD_WRITE_PG: u8 = 0x32;

const CMD_READ_ID: u8 = 0x9F;

const CMD_ENABLE_RESET: u8 = 0x66;
const CMD_RESET: u8 = 0x99;

const CMD_WRITE_ENABLE: u8 = 0x06;

const CMD_CHIP_ERASE: u8 = 0xC7;
const CMD_SECTOR_ERASE: u8 = 0x20;
const CMD_BLOCK_ERASE_32K: u8 = 0x52;
const CMD_BLOCK_ERASE_64K: u8 = 0xD8;

const CMD_READ_SR: u8 = 0x05;
const CMD_READ_CR: u8 = 0x35;

const CMD_WRITE_SR: u8 = 0x01;
const CMD_WRITE_CR: u8 = 0x31;

/// Implementation of access to flash chip.
/// Chip commands are hardcoded as it depends on used chip.
/// This implementation is using chip GD25Q64C from Giga Device
pub struct FlashMemory<I: Instance> {
    ospi: Ospi<'static, I, Blocking>,
}

impl<I: Instance> FlashMemory<I> {
    pub async fn new(ospi: Ospi<'static, I, Blocking>) -> Self {
        let mut memory = Self { ospi };

        memory.reset_memory().await;
        memory.enable_quad();
        memory
    }

    async fn qpi_mode(&mut self) {
        // Enter qpi mode
        self.exec_command(0x38).await;

        // Set read param
        let transaction = TransferConfig {
            iwidth: OspiWidth::QUAD,
            dwidth: OspiWidth::QUAD,
            instruction: Some(0xC0),
            ..Default::default()
        };
        self.enable_write().await;
        self.ospi.blocking_write(&[0x30_u8], transaction).unwrap();
        self.wait_write_finish();
    }

    pub async fn disable_mm(&mut self) {
        self.ospi.disable_memory_mapped_mode();
    }

    pub async fn enable_mm(&mut self) {
        self.qpi_mode().await;

        let read_config = TransferConfig {
            iwidth: OspiWidth::QUAD,
            isize: AddressSize::_8Bit,
            adwidth: OspiWidth::QUAD,
            adsize: AddressSize::_24bit,
            dwidth: OspiWidth::QUAD,
            instruction: Some(0x0B), // Fast read in QPI mode
            dummy: DummyCycles::_8,
            ..Default::default()
        };

        let write_config = TransferConfig {
            iwidth: OspiWidth::SING,
            isize: AddressSize::_8Bit,
            adwidth: OspiWidth::SING,
            adsize: AddressSize::_24bit,
            dwidth: OspiWidth::QUAD,
            instruction: Some(0x32), // Write config
            dummy: DummyCycles::_0,
            ..Default::default()
        };
        self.ospi
            .enable_memory_mapped_mode(read_config, write_config)
            .unwrap();
    }

    fn enable_quad(&mut self) {
        let cr = self.read_cr();
        // info!("Read cr: {:x}", cr);
        self.write_cr(cr | 0x02);
        // info!("Read cr after writing: {:x}", cr);
    }

    pub fn disable_quad(&mut self) {
        let cr = self.read_cr();
        self.write_cr(cr & (!(0x02)));
    }

    async fn exec_command_4(&mut self, cmd: u8) {
        let transaction = TransferConfig {
            iwidth: OspiWidth::QUAD,
            adwidth: OspiWidth::NONE,
            // adsize: AddressSize::_24bit,
            dwidth: OspiWidth::NONE,
            instruction: Some(cmd as u32),
            address: None,
            dummy: DummyCycles::_0,
            ..Default::default()
        };
        self.ospi.blocking_command(&transaction).unwrap();
    }

    async fn exec_command(&mut self, cmd: u8) {
        let transaction = TransferConfig {
            iwidth: OspiWidth::SING,
            adwidth: OspiWidth::NONE,
            // adsize: AddressSize::_24bit,
            dwidth: OspiWidth::NONE,
            instruction: Some(cmd as u32),
            address: None,
            dummy: DummyCycles::_0,
            ..Default::default()
        };
        // info!("Excuting command: {:x}", transaction.instruction);
        self.ospi.blocking_command(&transaction).unwrap();
    }

    pub async fn reset_memory(&mut self) {
        self.exec_command_4(CMD_ENABLE_RESET).await;
        self.exec_command_4(CMD_RESET).await;
        self.exec_command(CMD_ENABLE_RESET).await;
        self.exec_command(CMD_RESET).await;
        self.wait_write_finish();
    }

    pub async fn enable_write(&mut self) {
        self.exec_command(CMD_WRITE_ENABLE).await;
    }

    pub fn read_id(&mut self) -> [u8; 3] {
        let mut buffer = [0; 3];
        let transaction: TransferConfig = TransferConfig {
            iwidth: OspiWidth::SING,
            isize: AddressSize::_8Bit,
            adwidth: OspiWidth::NONE,
            // adsize: AddressSize::_24bit,
            dwidth: OspiWidth::SING,
            instruction: Some(CMD_READ_ID as u32),
            ..Default::default()
        };
        // info!("Reading id: 0x{:X}", transaction.instruction);
        self.ospi.blocking_read(&mut buffer, transaction).unwrap();
        buffer
    }

    pub fn read_id_4(&mut self) -> [u8; 3] {
        let mut buffer = [0; 3];
        let transaction: TransferConfig = TransferConfig {
            iwidth: OspiWidth::SING,
            isize: AddressSize::_8Bit,
            adwidth: OspiWidth::NONE,
            dwidth: OspiWidth::QUAD,
            instruction: Some(CMD_READ_ID as u32),
            ..Default::default()
        };
        info!("Reading id: 0x{:X}", transaction.instruction);
        self.ospi.blocking_read(&mut buffer, transaction).unwrap();
        buffer
    }

    pub fn read_memory(&mut self, addr: u32, buffer: &mut [u8], use_dma: bool) {
        let transaction = TransferConfig {
            iwidth: OspiWidth::SING,
            adwidth: OspiWidth::SING,
            adsize: AddressSize::_24bit,
            dwidth: OspiWidth::QUAD,
            instruction: Some(CMD_QUAD_READ as u32),
            address: Some(addr),
            dummy: DummyCycles::_8,
            ..Default::default()
        };
        if use_dma {
            self.ospi.blocking_read(buffer, transaction).unwrap();
        } else {
            self.ospi.blocking_read(buffer, transaction).unwrap();
        }
    }

    fn wait_write_finish(&mut self) {
        while (self.read_sr() & 0x01) != 0 {}
    }

    async fn perform_erase(&mut self, addr: u32, cmd: u8) {
        let transaction = TransferConfig {
            iwidth: OspiWidth::SING,
            adwidth: OspiWidth::SING,
            adsize: AddressSize::_24bit,
            dwidth: OspiWidth::NONE,
            instruction: Some(cmd as u32),
            address: Some(addr),
            dummy: DummyCycles::_0,
            ..Default::default()
        };
        self.enable_write().await;
        self.ospi.blocking_command(&transaction).unwrap();
        self.wait_write_finish();
    }

    pub async fn erase_sector(&mut self, addr: u32) {
        self.perform_erase(addr, CMD_SECTOR_ERASE).await;
    }

    pub async fn erase_block_32k(&mut self, addr: u32) {
        self.perform_erase(addr, CMD_BLOCK_ERASE_32K).await;
    }

    pub async fn erase_block_64k(&mut self, addr: u32) {
        self.perform_erase(addr, CMD_BLOCK_ERASE_64K).await;
    }

    pub async fn erase_chip(&mut self) {
        self.exec_command(CMD_CHIP_ERASE).await;
    }

    async fn write_page(&mut self, addr: u32, buffer: &[u8], len: usize, use_dma: bool) {
        defmt::assert!(
            (len as u32 + (addr & 0x000000ff)) <= MEMORY_PAGE_SIZE as u32,
            "write_page(): page write length exceeds page boundary (len = {}, addr = {:X}",
            len,
            addr
        );

        let transaction = TransferConfig {
            iwidth: OspiWidth::SING,
            adsize: AddressSize::_24bit,
            adwidth: OspiWidth::SING,
            dwidth: OspiWidth::QUAD,
            instruction: Some(CMD_QUAD_WRITE_PG as u32),
            address: Some(addr),
            dummy: DummyCycles::_0,
            ..Default::default()
        };
        self.enable_write().await;
        if use_dma {
            self.ospi.blocking_write(buffer, transaction).unwrap();
        } else {
            self.ospi.blocking_write(buffer, transaction).unwrap();
        }
        self.wait_write_finish();
    }

    pub async fn write_memory(&mut self, addr: u32, buffer: &[u8], use_dma: bool) {
        let mut left = buffer.len();
        let mut place = addr;
        let mut chunk_start = 0;

        while left > 0 {
            let max_chunk_size = MEMORY_PAGE_SIZE - (place & 0x000000ff) as usize;
            let chunk_size = if left >= max_chunk_size {
                max_chunk_size
            } else {
                left
            };
            let chunk = &buffer[chunk_start..(chunk_start + chunk_size)];
            self.write_page(place, chunk, chunk_size, use_dma).await;
            place += chunk_size as u32;
            left -= chunk_size;
            chunk_start += chunk_size;
        }
    }

    fn read_register(&mut self, cmd: u8) -> u8 {
        let mut buffer = [0; 1];
        let transaction: TransferConfig = TransferConfig {
            iwidth: OspiWidth::SING,
            isize: AddressSize::_8Bit,
            adwidth: OspiWidth::NONE,
            adsize: AddressSize::_24bit,
            dwidth: OspiWidth::SING,
            instruction: Some(cmd as u32),
            address: None,
            dummy: DummyCycles::_0,
            ..Default::default()
        };
        self.ospi.blocking_read(&mut buffer, transaction).unwrap();
        // info!("Read w25q64 register: 0x{:x}", buffer[0]);
        buffer[0]
    }

    fn write_register(&mut self, cmd: u8, value: u8) {
        let buffer = [value; 1];
        let transaction: TransferConfig = TransferConfig {
            iwidth: OspiWidth::SING,
            isize: AddressSize::_8Bit,
            instruction: Some(cmd as u32),
            adsize: AddressSize::_24bit,
            adwidth: OspiWidth::NONE,
            dwidth: OspiWidth::SING,
            address: None,
            dummy: DummyCycles::_0,
            ..Default::default()
        };
        self.ospi.blocking_write(&buffer, transaction).unwrap();
    }

    pub fn read_sr(&mut self) -> u8 {
        self.read_register(CMD_READ_SR)
    }

    pub fn read_cr(&mut self) -> u8 {
        self.read_register(CMD_READ_CR)
    }

    pub fn write_sr(&mut self, value: u8) {
        self.write_register(CMD_WRITE_SR, value);
    }

    pub fn write_cr(&mut self, value: u8) {
        self.write_register(CMD_WRITE_CR, value);
    }
}
