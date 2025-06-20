#![no_std]
// disable warning for already-stabilized features.
// Needed to pass CI, because we deny warnings.
// We don't immediately remove them to not immediately break older nightlies.
// When all features are stable, we'll remove them.
#![cfg_attr(feature = "nightly", allow(stable_features, unknown_lints))]
#![cfg_attr(feature = "nightly", feature(async_fn_in_trait, impl_trait_projections))]
#![allow(async_fn_in_trait)]


use core::convert::From;
use core::result::Result::{Err, Ok};
use embassy_futures::{block_on, join::join};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::zerocopy_channel::{Channel, Receiver, Sender};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;
use num_enum::{IntoPrimitive, TryFromPrimitive};
use tock_registers::register_bitfields;
use tock_registers::LocalRegisterCopy;
use transport::Transport;
use zerocopy::byteorder::little_endian::{U16, U32};
use zerocopy::{FromBytes, FromZeros, Immutable, IntoBytes, Unaligned};

use defmt::{debug, error, Format};

pub mod cs_control;
pub mod multi_io_spi;
pub mod no_spi;
pub mod transport;

pub use cs_control::{CsControl, NoCs};
pub use multi_io_spi::{Address, MultiIOMode, MultiIOSpi, MultiIOTransaction, NoMultiIO};
pub use no_spi::NoSpi;

pub trait ChangeSpiFreq {
    const SUPPORTED: bool = false;

    fn change_frequency(&mut self, _freq: u32) {}
    fn get_frequency(&self) -> u32 {
        0
    }
}

#[derive(Format)]
pub enum SerprogError {
    TransportRead(&'static str),
    TransportWrite(&'static str),
    SpiTransfer(&'static str),
    SpiFlush(&'static str),
    CsSetLow(&'static str),
    CsSetHigh(&'static str),
    LedSetLow(&'static str),
    LedSetHigh(&'static str),
}

// Convert 3 bytes in little-endian format to u32
fn le_u24_to_u32(bytes: &[u8]) -> u32 {
    u32::from_le_bytes([bytes[0], bytes[1], bytes[2], 0])
}

const S_ACK: u8 = 0x06;
const S_NAK: u8 = 0x15;
const MAX_BUFFER_SIZE: usize = 16 << 20;
const MULTIIO_SPI_WRITE_SIZE: usize = 4096; // Max size for opcode + address + mode bytes + data
const MULTIIO_SPI_READ_DATA_SIZE: usize = 4096; // Max size for read/write data

#[derive(FromBytes, IntoBytes, Unaligned, Immutable)]
#[repr(C, packed)]
struct MultiIOSpiHeader {
    io_mode_and_direction: u8, // IO mode (bits 0-6) + read/write flag (bit 7)
    opcode_len: u8,
    addr_len: u8,
    mode_bytes_len: u8,
    dummy_cycles: u8,
    data_size: U32, // LE size of data
}

#[derive(FromBytes, IntoBytes, Unaligned, Immutable)]
#[repr(C, packed)]
struct QMultiIOSpiModesResponse {
    ack: u8,
    supported_modes: u8, // Bitmask of supported MultiIO modes
}

impl QMultiIOSpiModesResponse {
    fn new(supported_modes: u8) -> Self {
        Self {
            ack: S_ACK,
            supported_modes,
        }
    }
}

#[derive(FromBytes, IntoBytes, Unaligned, Immutable)]
#[repr(C, packed)]
struct SSpiFreqRequest {
    freq: U32,
}

#[derive(FromBytes, IntoBytes, Unaligned, Immutable)]
#[repr(C, packed)]
struct SSpiFreqResponse {
    ack: u8,
    freq: U32,
}

#[derive(FromBytes, IntoBytes, Unaligned, Immutable)]
#[repr(C, packed)]
struct QMaxLenResponse {
    ack: u8,
    size: [u8; 3], // 24-bit value
}

impl QMaxLenResponse {
    fn new(size: usize) -> Self {
        let bytes = size.to_le_bytes();
        Self {
            ack: S_ACK,
            size: [bytes[0], bytes[1], bytes[2]],
        }
    }
}

#[derive(FromBytes, IntoBytes, Unaligned, Immutable)]
#[repr(C, packed)]
struct QPgmNameResponse {
    ack: u8,
    name: [u8; 16], // Fixed size array for programmer name
}

impl QPgmNameResponse {
    fn new(name: &str) -> Self {
        let mut response = Self {
            ack: S_ACK,
            name: [0; 16],
        };
        // Copy up to 15 bytes of the name, leaving room for null terminator
        let bytes = name.as_bytes();
        let len = core::cmp::min(bytes.len(), 15);
        response.name[..len].copy_from_slice(&bytes[..len]);
        response
    }
}

#[derive(IntoBytes, Unaligned, Immutable)]
#[repr(C, packed)]
struct QIfaceResponse {
    ack: u8,
    version: U16,
}

#[derive(Debug, Eq, PartialEq, TryFromPrimitive, IntoPrimitive)]
#[repr(u8)]
pub enum SerprogCommand {
    Nop = 0x00,              // No operation
    QIface = 0x01,           // Query interface version
    QCmdMap = 0x02,          // Query supported commands bitmap
    QPgmName = 0x03,         // Query programmer name
    QSerBuf = 0x04,          // Query Serial Buffer Size
    QBustype = 0x05,         // Query supported bustypes
    QChipSize = 0x06,        // Query supported chipsize (2^n format)
    QOpBuf = 0x07,           // Query operation buffer size
    QWrNMaxLen = 0x08,       // Query Write to opbuf: Write-N maximum length
    RByte = 0x09,            // Read a single byte
    RNBytes = 0x0A,          // Read n bytes
    OInit = 0x0B,            // Initialize operation buffer
    OWriteB = 0x0C,          // Write opbuf: Write byte with address
    OWriteN = 0x0D,          // Write to opbuf: Write-N
    ODelay = 0x0E,           // Write opbuf: udelay
    OExec = 0x0F,            // Execute operation buffer
    SyncNop = 0x10,          // Special no-operation that returns NAK+ACK
    QRdNMaxLen = 0x11,       // Query read-n maximum length
    SBustype = 0x12,         // Set used bustype(s)
    OSpiOp = 0x13,           // Perform SPI operation
    SSpiFreq = 0x14,         // Set SPI clock frequency
    SPinState = 0x15,        // Enable/disable output drivers
    SSpiCs = 0x16,           // Select Chip Select to use
    SSpiMode = 0x17,         // Set SPI Mode
    SCSMode = 0x18,          // Set CS Mode
    QMultiIOSpiModes = 0x19, // Query available Multi-IO SPI modes
    MultiIOSpiOp = 0x1a,     // Perform Multi-IO SPI operation
}

#[derive(FromBytes, IntoBytes, Unaligned, Immutable)]
#[repr(C, packed)]
struct QCmdMapResponse {
    ack: u8,
    map: [u8; 4],    // First 32 bits for command flags
    zeros: [u8; 28], // Remaining bits as zeros, for future use
}

register_bitfields! [u32,
    Commands [
        Nop OFFSET(0) NUMBITS(1) [],
        QIface OFFSET(1) NUMBITS(1) [],
        QCmdMap OFFSET(2) NUMBITS(1) [],
        QPgmName OFFSET(3) NUMBITS(1) [],
        QSerBuf OFFSET(4) NUMBITS(1) [],
        QBustype OFFSET(5) NUMBITS(1) [],
        QChipSize OFFSET(6) NUMBITS(1) [],
        QOpBuf OFFSET(7) NUMBITS(1) [],
        QWrNMaxLen OFFSET(8) NUMBITS(1) [],
        RByte OFFSET(9) NUMBITS(1) [],
        RNBytes OFFSET(10) NUMBITS(1) [],
        OInit OFFSET(11) NUMBITS(1) [],
        OWriteB OFFSET(12) NUMBITS(1) [],
        OWriteN OFFSET(13) NUMBITS(1) [],
        ODelay OFFSET(14) NUMBITS(1) [],
        OExec OFFSET(15) NUMBITS(1) [],
        SyncNop OFFSET(16) NUMBITS(1) [],
        QRdNMaxLen OFFSET(17) NUMBITS(1) [],
        SBustype OFFSET(18) NUMBITS(1) [],
        OSpiOp OFFSET(19) NUMBITS(1) [],
        SSpiFreq OFFSET(20) NUMBITS(1) [],
        SPinState OFFSET(21) NUMBITS(1) [],
        SSpiCs OFFSET(22) NUMBITS(1) [],
        SSpiMode OFFSET(23) NUMBITS(1) [],
        SCSMode OFFSET(24) NUMBITS(1) [],
        QMultiIOSpiModes OFFSET(25) NUMBITS(1) [],
        MultiIOSpiOp OFFSET(26) NUMBITS(1) []
    ]
];

register_bitfields! [u8,
    IOModeAndDirection [
        IOMode OFFSET(0) NUMBITS(7) [
            SingleIO111 = 0,
            DualOut112 = 1,
            DualIO122 = 2,
            QuadOut114 = 3,
            QuadIO144 = 4,
            QPI444 = 5
        ],
        ReadWrite OFFSET(7) NUMBITS(1) [
            Write = 0,
            Read = 1
        ]
    ]
];

impl QCmdMapResponse {
    fn new(has_freq_callback: bool) -> Self {
        let mut response = Self {
            ack: S_ACK,
            map: [0; 4],
            zeros: [0; 28],
        };

        // Set supported commands using tock-registers
        let mut cmdmap = LocalRegisterCopy::<u32, Commands::Register>::new(0);
        let mut cmd_flags = Commands::Nop::SET
            + Commands::QIface::SET
            + Commands::QCmdMap::SET
            + Commands::QPgmName::SET
            + Commands::QSerBuf::SET
            + Commands::QWrNMaxLen::SET
            + Commands::QBustype::SET
            + Commands::SyncNop::SET
            + Commands::QRdNMaxLen::SET
            + Commands::OSpiOp::SET
            + Commands::SBustype::SET
            + Commands::SPinState::SET
            + Commands::QMultiIOSpiModes::SET
            + Commands::MultiIOSpiOp::SET;

        if has_freq_callback {
            cmd_flags += Commands::SSpiFreq::SET;
        }

        cmdmap.modify(cmd_flags);

        // Get the bits and store in first 4 bytes, rest stays as zeros
        let bits = cmdmap.get();
        response.map[0..4].copy_from_slice(&bits.to_le_bytes());

        response
    }
}

pub struct Serprog<SPI, MSPI, CS, LED, T: Transport> {
    spi: Option<SPI>,
    multi_spi: Option<MSPI>,
    cs: CS,
    led: LED,
    transport: T,
}

impl<SPI, MSPI, CS, LED, T> Serprog<SPI, MSPI, CS, LED, T>
where
    SPI: SpiBus<u8> + ChangeSpiFreq,
    MSPI: MultiIOSpi,
    CS: CsControl,
    LED: OutputPin,
    T: Transport,
{
    pub fn new(spi: Option<SPI>, multi_spi: Option<MSPI>, cs: CS, led: LED, transport: T) -> Self {
        Self {
            spi,
            multi_spi,
            cs,
            led,
            transport,
        }
    }

    pub async fn run_loop(mut self) -> ! {
        let mut buf = [0; 1];

        loop {
            if self.transport.read(&mut buf).await.is_err() {
                error!("Read error in main loop");
                continue;
            }

            let cmd = SerprogCommand::try_from(buf[0]).unwrap_or(SerprogCommand::Nop);
            if let Err(e) = self.handle_command(cmd).await {
                error!("Command error: {:?}", e);
            }
        }
    }

    async fn handle_command(&mut self, cmd: SerprogCommand) -> Result<(), SerprogError>
    where
        LED::Error: core::fmt::Debug,
    {
        match cmd {
            SerprogCommand::Nop => {
                debug!("Received Nop CMD");
                self.transport
                    .write(&[S_ACK])
                    .await
                    .map_err(|_| SerprogError::TransportWrite("Error writing ACK"))?;
                Ok(())
            }
            SerprogCommand::QIface => {
                debug!("Received QIface CMD");
                let response = QIfaceResponse {
                    ack: S_ACK,
                    version: U16::new(1),
                };
                self.transport
                    .write(response.as_bytes())
                    .await
                    .map_err(|_| SerprogError::TransportWrite("Error writing QIface response"))?;
                Ok(())
            }
            SerprogCommand::QCmdMap => {
                debug!("Received QCmdMap CMD");
                let response = QCmdMapResponse::new(SPI::SUPPORTED);
                self.transport
                    .write(response.as_bytes())
                    .await
                    .map_err(|_| SerprogError::TransportWrite("Error writing QCmdMap response"))?;
                Ok(())
            }
            SerprogCommand::QPgmName => {
                debug!("Received QPgmName CMD");
                let response = QPgmNameResponse::new("Picoprog");
                self.transport
                    .write(response.as_bytes())
                    .await
                    .map_err(|_| SerprogError::TransportWrite("Error writing QPgmName response"))?;
                Ok(())
            }
            SerprogCommand::QSerBuf => {
                debug!("Received QSerBuf CMD");
                self.transport
                    .write(&[S_ACK, 0xFF, 0xFF])
                    .await
                    .map_err(|_| SerprogError::TransportWrite("Error writing QSerBuf response"))?;
                Ok(())
            }
            SerprogCommand::QWrNMaxLen | SerprogCommand::QRdNMaxLen => {
                debug!("Received QWrNMaxLen/QRdNMaxLen CMD");

                let mut max_size = 0;

                // Check SPI availability and use MAX_BUFFER_SIZE
                if self.spi.is_some() {
                    max_size = max_size.max(MAX_BUFFER_SIZE);
                }

                // Check Multi-IO SPI availability and use its max transaction size
                if self.multi_spi.is_some() {
                    max_size = max_size.max(MSPI::MAX_TRANSACTION_SIZE);
                }

                let response = QMaxLenResponse::new(max_size);
                self.transport
                    .write(response.as_bytes())
                    .await
                    .map_err(|_| SerprogError::TransportWrite("Error writing QMaxLen response"))?;
                Ok(())
            }
            SerprogCommand::QBustype => {
                debug!("Received QBustype CMD");
                self.transport
                    .write(&[S_ACK, 0x08])
                    .await
                    .map_err(|_| SerprogError::TransportWrite("Error writing QBustype response"))?;
                Ok(())
            }
            SerprogCommand::SyncNop => {
                debug!("Received SyncNop CMD");
                self.transport
                    .write(&[S_NAK, S_ACK])
                    .await
                    .map_err(|_| SerprogError::TransportWrite("Error writing SyncNop response"))?;
                Ok(())
            }
            SerprogCommand::SBustype => {
                debug!("Received SBustype CMD");
                let mut buf = [0u8; 1];
                self.transport
                    .read(&mut buf)
                    .await
                    .map_err(|_| SerprogError::TransportRead("Error reading SBustype data"))?;
                if buf[0] == 0x08 {
                    debug!("Received SBustype 'SPI'");
                    self.transport
                        .write(&[S_ACK])
                        .await
                        .map_err(|_| SerprogError::TransportWrite("Error writing SBustype ACK"))?;
                } else {
                    debug!("Received unknown SBustype");
                    self.transport
                        .write(&[S_NAK])
                        .await
                        .map_err(|_| SerprogError::TransportWrite("Error writing SBustype NAK"))?;
                }
                Ok(())
            }
            SerprogCommand::OSpiOp => {
                debug!("Received OSpiOp CMD");

                // Check if SPI is available
                if self.spi.is_none() {
                    debug!("OSpiOp requested but SPI not available");
                    self.transport
                        .write(&[S_NAK])
                        .await
                        .map_err(|_| SerprogError::TransportWrite("Error writing OSpiOp NAK"))?;
                    return Ok(());
                }

                let mut sdata = [0_u8; 64];
                self.transport
                    .read(sdata.as_mut_slice())
                    .await
                    .map_err(|_| SerprogError::TransportRead("Error reading OSpiOp data"))?;

                let op_slen = le_u24_to_u32(&sdata[0..3]) as usize;
                let op_rlen = le_u24_to_u32(&sdata[3..6]) as usize;

                let mut usb_rx_spi_tx_buf = [([0u8; 64], 0); 4];
                let mut usb_rx_spi_tx_channel: Channel<'_, NoopRawMutex, ([u8; 64], usize)> =
                    Channel::new(&mut usb_rx_spi_tx_buf);
                let (usb_rx, spi_tx) = usb_rx_spi_tx_channel.split();

                let mut usb_tx_spi_rx_buf = [([0u8; 64], 0); 8];
                let mut usb_tx_spi_rx_channel: Channel<'_, NoopRawMutex, ([u8; 64], usize)> =
                    Channel::new(&mut usb_tx_spi_rx_buf);
                let (spi_rx, usb_tx) = usb_tx_spi_rx_channel.split();

                let usb_task = async |transport: &mut T,
                                      mut sender: Sender<NoopRawMutex, ([u8; 64], usize)>,
                                      sdata_size: usize,
                                      sdata_0: [u8; 64],
                                      mut receiver: Receiver<NoopRawMutex, ([u8; 64], usize)>,
                                      rdata_size: usize|
                       -> Result<(), SerprogError> {
                    // First block
                    let mut data_to_read = sdata_size;
                    {
                        let (buf, size) = sender.send().await;
                        let block_size = data_to_read.min(64 - 6);
                        buf[..block_size].copy_from_slice(&sdata_0[6..6 + block_size]);
                        *size = block_size;
                        sender.send_done();
                        data_to_read -= block_size;
                    }

                    while data_to_read > 0 {
                        let read_size = data_to_read.min(64);
                        let (buf, size) = sender.send().await;
                        *size = read_size;
                        transport.read(&mut buf[..read_size]).await.map_err(|_| {
                            SerprogError::TransportRead("Error reading OSpiOp data")
                        })?;
                        sender.send_done();
                        data_to_read -= read_size;
                    }
                    transport
                        .write(&[S_ACK])
                        .await
                        .map_err(|_| SerprogError::TransportWrite("Error writing SBustype ACK"))?;

                    let mut data_to_send = rdata_size;
                    while data_to_send > 0 {
                        let (buf, size) = receiver.receive().await;
                        let size = *size;
                        transport.write(&buf[..size]).await.map_err(|_| {
                            SerprogError::TransportWrite("Error writing SPI read data")
                        })?;
                        receiver.receive_done();
                        data_to_send -= size;
                    }
                    Ok(())
                };

                let spi_task = async |spi: &mut SPI,
                                      mut receiver: Receiver<NoopRawMutex, ([u8; 64], usize)>,
                                      sdata_size: usize,
                                      mut sender: Sender<NoopRawMutex, ([u8; 64], usize)>,
                                      rdata_size: usize,
                                      cs: &mut CS|
                       -> Result<(), SerprogError> {
                    spi.flush().await.map_err(|_| {
                        SerprogError::SpiFlush("Error flushing SPI before transfer")
                    })?;

                    cs.set_low()
                        .map_err(|_| SerprogError::CsSetLow("Error setting CS low"))?;
                    let mut data_to_write = sdata_size;
                    while data_to_write > 0 {
                        let (buf, size) = receiver.receive().await;
                        data_to_write -= *size;
                        spi.write(&buf[..*size])
                            .await
                            .map_err(|_| SerprogError::SpiTransfer("Error writing OSpiOp data"))?;
                        receiver.receive_done();
                    }
                    let mut data_to_read = rdata_size;
                    while data_to_read > 0 {
                        let (buf, size) = sender.send().await;
                        let read_size = data_to_read.min(buf.len());
                        spi.read(&mut buf[..read_size])
                            .await
                            .map_err(|_| SerprogError::SpiTransfer("Error reading OSpiOp data"))?;
                        *size = read_size;
                        sender.send_done();
                        data_to_read -= read_size;
                    }
                    cs.set_high()
                        .map_err(|_| SerprogError::CsSetHigh("Error setting CS high"))?;
                    debug!("OSpiOp CMD done");
                    Ok(())
                };

                let (spi_res, usb_res) = block_on(join(
                    spi_task(
                        self.spi.as_mut().unwrap(), // Safe because we checked above
                        spi_tx,
                        op_slen,
                        spi_rx,
                        op_rlen,
                        &mut self.cs,
                    ),
                    usb_task(&mut self.transport, usb_rx, op_slen, sdata, usb_tx, op_rlen),
                ));
                if let Err(spi_err) = spi_res {
                    self.transport
                        .write(&[S_NAK])
                        .await
                        .map_err(|_| SerprogError::TransportWrite("Failed to report SPI failed"))?;
                    return Err(spi_err);
                }
                usb_res?;

                Ok(())
            }
            SerprogCommand::SSpiFreq => {
                debug!("Received SSpiFreq CMD");

                // Check if SPI is available
                if let Some(ref mut spi) = self.spi {
                    let mut request = SSpiFreqRequest::new_zeroed();
                    self.transport
                        .read(request.as_mut_bytes())
                        .await
                        .map_err(|_| SerprogError::TransportRead("Error reading SSpiFreq data"))?;

                    // Parse the request using zerocopy
                    let try_freq = request.freq.get();

                    debug!("Setting SPI frequency: {:?}", try_freq);

                    // Change frequency using the trait method
                    spi.change_frequency(try_freq);

                    // Create and send response with actual frequency
                    let actual_freq = spi.get_frequency();
                    let response = SSpiFreqResponse {
                        ack: S_ACK,
                        freq: U32::new(actual_freq),
                    };

                    self.transport
                        .write(response.as_bytes())
                        .await
                        .map_err(|_| {
                            SerprogError::TransportWrite("Error writing SSpiFreq response")
                        })?;
                } else {
                    debug!("SSpiFreq requested but SPI not available");
                    // Still need to read the request data to keep protocol in sync
                    let mut request = SSpiFreqRequest::new_zeroed();
                    self.transport
                        .read(request.as_mut_bytes())
                        .await
                        .map_err(|_| SerprogError::TransportRead("Error reading SSpiFreq data"))?;

                    self.transport
                        .write(&[S_NAK])
                        .await
                        .map_err(|_| SerprogError::TransportWrite("Error writing SSpiFreq NAK"))?;
                }

                Ok(())
            }
            SerprogCommand::SPinState => {
                debug!("Received SPinState CMD");
                let mut buf = [0u8; 1];
                self.transport
                    .read(&mut buf)
                    .await
                    .map_err(|_| SerprogError::TransportRead("Error reading SPinState data"))?;
                if buf[0] == 0 {
                    self.led
                        .set_low()
                        .map_err(|_| SerprogError::LedSetLow("Error setting LED low"))?;
                } else {
                    self.led
                        .set_high()
                        .map_err(|_| SerprogError::LedSetHigh("Error setting LED high"))?;
                }
                self.transport
                    .write(&[S_ACK])
                    .await
                    .map_err(|_| SerprogError::TransportWrite("Error writing SPinState ACK"))?;

                Ok(())
            }
            SerprogCommand::SSpiCs => {
                debug!("Received SSpiCs CMD");
                let mut buf = [0u8; 1];
                self.transport
                    .read(&mut buf)
                    .await
                    .map_err(|_| SerprogError::TransportRead("Error reading SSpiCs data"))?;

                let cs_index = buf[0];
                match self.cs.select_cs(cs_index) {
                    Ok(()) => {
                        debug!("Selected CS {}", cs_index);
                        self.transport.write(&[S_ACK]).await.map_err(|_| {
                            SerprogError::TransportWrite("Error writing SSpiCs ACK")
                        })?;
                    }
                    Err(_) => {
                        debug!("Failed to select CS {}", cs_index);
                        self.transport.write(&[S_NAK]).await.map_err(|_| {
                            SerprogError::TransportWrite("Error writing SSpiCs NAK")
                        })?;
                    }
                }
                Ok(())
            }
            SerprogCommand::QMultiIOSpiModes => {
                debug!("Received QMultiIOSpiModes CMD");

                if let Some(ref multi_spi) = self.multi_spi {
                    let supported_modes = multi_spi.supported_modes();
                    let response = QMultiIOSpiModesResponse::new(supported_modes);
                    self.transport
                        .write(response.as_bytes())
                        .await
                        .map_err(|_| {
                            SerprogError::TransportWrite("Error writing QMultiIOSpiModes response")
                        })?;
                } else {
                    debug!("QMultiIOSpiModes requested but MultiIO SPI not available");
                    self.transport.write(&[S_NAK]).await.map_err(|_| {
                        SerprogError::TransportWrite("Error writing QMultiIOSpiModes NAK")
                    })?;
                }
                Ok(())
            }
            SerprogCommand::MultiIOSpiOp => {
                debug!("Received MultiIOSpiOp CMD");

                if let Some(ref mut multi_spi) = self.multi_spi {
                    let mut spi_write_buffer = [0u8; MULTIIO_SPI_WRITE_SIZE];

                    // Read initial 512-byte chunk (USB transfer size)
                    let initial_read_size = 512.min(MULTIIO_SPI_WRITE_SIZE);
                    self.transport
                        .read(&mut spi_write_buffer[..initial_read_size])
                        .await
                        .map_err(|_| {
                            SerprogError::TransportRead("Error reading MultiIOSpiOp initial data")
                        })?;

                    // Parse header using zerocopy from the first 9 bytes
                    let header = MultiIOSpiHeader::read_from_bytes(&spi_write_buffer[..9]).unwrap();

                    // Parse header fields
                    let io_mode_reg = LocalRegisterCopy::<u8, IOModeAndDirection::Register>::new(
                        header.io_mode_and_direction,
                    );
                    let io_mode = match MultiIOMode::try_from(
                        io_mode_reg.read(IOModeAndDirection::IOMode) as u8,
                    ) {
                        Ok(mode) => mode,
                        Err(_) => {
                            self.transport.write(&[S_NAK]).await.map_err(|_| {
                                SerprogError::TransportWrite("Error writing MultiIOSpiOp NAK")
                            })?;
                            return Ok(());
                        }
                    };
                    let is_read = io_mode_reg.is_set(IOModeAndDirection::ReadWrite);

                    let opcode_len = header.opcode_len as usize;
                    let addr_len = header.addr_len as usize;
                    let mode_bytes_len = header.mode_bytes_len as usize;
                    let dummy_cycles = header.dummy_cycles;
                    let data_len = header.data_size.get() as usize;

                    // Calculate total data size needed (header + opcode + address + mode bytes + write data)
                    let total_data_needed = 9
                        + opcode_len
                        + addr_len
                        + mode_bytes_len
                        + if is_read { 0 } else { data_len };

                    // Assert buffer size limits
                    assert!(
                        data_len <= MULTIIO_SPI_READ_DATA_SIZE,
                        "MultiIO data size too large"
                    );
                    assert!(
                        total_data_needed <= MULTIIO_SPI_WRITE_SIZE,
                        "MultiIO total data too large"
                    );

                    // If we need more data than what we read initially, read the rest
                    if total_data_needed > initial_read_size {
                        self.transport
                            .read(&mut spi_write_buffer[initial_read_size..total_data_needed])
                            .await
                            .map_err(|_| {
                                SerprogError::TransportRead(
                                    "Error reading MultiIOSpiOp remaining data",
                                )
                            })?;
                    }

                    // Use subslice for parsing the additional data (after the 9-byte header)
                    let additional_data_size =
                        opcode_len + addr_len + mode_bytes_len + if is_read { 0 } else { data_len };
                    let header_data = &spi_write_buffer[9..9 + additional_data_size];

                    // Parse address if present
                    let address = if addr_len > 0 {
                        let addr_start = opcode_len;
                        let addr_bytes = &header_data[addr_start..addr_start + addr_len];
                        match addr_len {
                            3 => Some(Address::Addr24(le_u24_to_u32(addr_bytes))),
                            4 => Some(Address::Addr32(u32::from_le_bytes([
                                addr_bytes[0],
                                addr_bytes[1],
                                addr_bytes[2],
                                addr_bytes[3],
                            ]))),
                            _ => {
                                self.transport.write(&[S_NAK]).await.map_err(|_| {
                                    SerprogError::TransportWrite("Error writing MultiIOSpiOp NAK")
                                })?;
                                return Ok(());
                            }
                        }
                    } else {
                        None
                    };

                    // Create transaction
                    let transaction = MultiIOTransaction {
                        mode: io_mode,
                        opcode: if opcode_len > 0 { header_data[0] } else { 0 },
                        address,
                        mode_byte: mode_bytes_len > 0,
                        dummy_cycles,
                    };

                    if is_read {
                        // Read operation
                        let mut read_data = [0u8; MULTIIO_SPI_READ_DATA_SIZE];
                        match multi_spi.read(transaction, &mut read_data[..data_len]).await {
                            Ok(()) => {
                                // Send ACK + read data
                                self.transport.write(&[S_ACK]).await.map_err(|_| {
                                    SerprogError::TransportWrite("Error writing MultiIOSpiOp ACK")
                                })?;
                                self.transport.write(&read_data[..data_len]).await.map_err(
                                    |_| {
                                        SerprogError::TransportWrite(
                                            "Error writing MultiIOSpiOp read data",
                                        )
                                    },
                                )?;
                            }
                            Err(_) => {
                                self.transport.write(&[S_NAK]).await.map_err(|_| {
                                    SerprogError::TransportWrite("Error writing MultiIOSpiOp NAK")
                                })?;
                            }
                        }
                    } else {
                        // Write operation - data is already in header_data after opcode/address/mode bytes
                        let write_data_start = opcode_len + addr_len + mode_bytes_len;
                        let write_data =
                            &header_data[write_data_start..write_data_start + data_len];

                        match multi_spi.write(transaction, write_data).await {
                            Ok(()) => {
                                self.transport.write(&[S_ACK]).await.map_err(|_| {
                                    SerprogError::TransportWrite("Error writing MultiIOSpiOp ACK")
                                })?;
                            }
                            Err(_) => {
                                self.transport.write(&[S_NAK]).await.map_err(|_| {
                                    SerprogError::TransportWrite("Error writing MultiIOSpiOp NAK")
                                })?;
                            }
                        }
                    }
                } else {
                    debug!("MultiIOSpiOp requested but MultiIO SPI not available");
                    self.transport.write(&[S_NAK]).await.map_err(|_| {
                        SerprogError::TransportWrite("Error writing MultiIOSpiOp NAK")
                    })?;
                }
                Ok(())
            }
            _ => {
                debug!("Received unknown CMD");
                self.transport.write(&[S_NAK]).await.map_err(|_| {
                    SerprogError::TransportWrite("Error writing unknown command NAK")
                })?;

                Ok(())
            }
        }
    }
}
