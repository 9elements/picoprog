#![no_std]

use core::convert::From;
use core::result::Result::{Err, Ok};
use embedded_hal::digital::OutputPin;
use num_enum::{IntoPrimitive, TryFromPrimitive};
use tock_registers::register_bitfields;
use tock_registers::LocalRegisterCopy;
use transport::{OSpiOpCallback, Transport};
use zerocopy::byteorder::little_endian::{U16, U32};
use zerocopy::{FromBytes, FromZeros, Immutable, IntoBytes, Unaligned};

use defmt::{debug, error, Format};

pub mod transport;

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
pub fn le_u24_to_u32(bytes: &[u8]) -> u32 {
    u32::from_le_bytes([bytes[0], bytes[1], bytes[2], 0])
}

const S_ACK: u8 = 0x06;
const S_NAK: u8 = 0x15;
const MAX_BUFFER_SIZE: usize = 16 << 20;

#[derive(Debug, Eq, PartialEq, TryFromPrimitive, IntoPrimitive)]
#[repr(u8)]
pub enum MultiIOMode {
    SingleIO111 = 0,
    DualOut112 = 1,
    DualIO122 = 2,
    QuadOut114 = 3,
    QuadIO144 = 4,
    QPI444 = 5,
}

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
    QMultiIOSpiModes = 0x17, // Query available Multi-IO SPI modes
    MultiIOSpiOp = 0x18,     // Perform Multi-IO SPI operation
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
        QMultiIOSpiModes OFFSET(23) NUMBITS(1) [],
        MultiIOSpiOp OFFSET(24) NUMBITS(1) []
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
    fn new(has_freq_callback: bool, has_ospi_op_callback: bool) -> Self {
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
            + Commands::SBustype::SET
            + Commands::SPinState::SET;

        if has_ospi_op_callback {
            cmd_flags += Commands::OSpiOp::SET;
            // Also enable MultiIO SPI commands if ospi callback is available
            cmd_flags += Commands::QMultiIOSpiModes::SET;
            cmd_flags += Commands::MultiIOSpiOp::SET;
        }

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

pub struct Serprog<SPI, CS, LED, T: Transport, F, O> {
    spi: SPI,
    cs: CS,
    led: LED,
    transport: T,
    freq_callback: Option<F>,
    ospi_op_callback: Option<O>,
}

impl<SPI, CS, LED, T, F, O> Serprog<SPI, CS, LED, T, F, O>
where
    CS: OutputPin,
    LED: OutputPin,
    T: Transport,
    F: FnMut(&mut SPI, u32) + Send + Sync,
    O: OSpiOpCallback<SPI, CS, T>,
{
    pub fn new(
        spi: SPI,
        cs: CS,
        led: LED,
        transport: T,
        freq_callback: Option<F>,
        ospi_op_callback: Option<O>,
    ) -> Self {
        Self {
            spi,
            cs,
            led,
            transport,
            freq_callback,
            ospi_op_callback,
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
        CS::Error: core::fmt::Debug,
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
                let response = QCmdMapResponse::new(
                    self.freq_callback.is_some(),
                    self.ospi_op_callback.is_some(),
                );
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
                let response = QMaxLenResponse::new(MAX_BUFFER_SIZE);
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
                if let Some(callback) = &mut self.ospi_op_callback {
                    callback
                        .handle_ospi_op(&mut self.spi, &mut self.cs, &mut self.transport)
                        .await
                } else {
                    debug!("OSpiOp not supported - no callback provided");
                    self.transport
                        .write(&[S_NAK])
                        .await
                        .map_err(|_| SerprogError::TransportWrite("Error writing OSpiOp NAK"))?;
                    Ok(())
                }
            }
            SerprogCommand::SSpiFreq => {
                debug!("Received SSpiFreq CMD");
                let mut request = SSpiFreqRequest::new_zeroed();
                self.transport
                    .read(request.as_mut_bytes())
                    .await
                    .map_err(|_| SerprogError::TransportRead("Error reading SSpiFreq data"))?;

                // Parse the request using zerocopy
                let try_freq = request.freq.get();

                debug!("Setting SPI frequency: {:?}", try_freq);

                // Call the frequency callback if set
                if let Some(callback) = &mut self.freq_callback {
                    (callback)(&mut self.spi, try_freq);
                }

                // Create and send response
                let response = SSpiFreqResponse {
                    ack: S_ACK,
                    freq: U32::new(try_freq), // TODO can we report what the hardware has set up?
                };

                self.transport
                    .write(response.as_bytes())
                    .await
                    .map_err(|_| SerprogError::TransportWrite("Error writing SSpiFreq response"))?;

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
            SerprogCommand::QMultiIOSpiModes => {
                debug!("Received QMultiIOSpiModes CMD");
                if let Some(callback) = &self.ospi_op_callback {
                    let supported_modes = callback.get_supported_multi_io_modes();
                    let response = QMultiIOSpiModesResponse::new(supported_modes);
                    self.transport
                        .write(response.as_bytes())
                        .await
                        .map_err(|_| {
                            SerprogError::TransportWrite("Error writing QMultiIOSpiModes response")
                        })?;
                } else {
                    debug!("QMultiIOSpiModes not supported - no callback provided");
                    self.transport.write(&[S_NAK]).await.map_err(|_| {
                        SerprogError::TransportWrite("Error writing QMultiIOSpiModes NAK")
                    })?;
                }
                Ok(())
            }
            SerprogCommand::MultiIOSpiOp => {
                debug!("Received MultiIOSpiOp CMD");
                if let Some(callback) = &mut self.ospi_op_callback {
                    callback
                        .handle_multi_io_spi_op(&mut self.spi, &mut self.cs, &mut self.transport)
                        .await
                } else {
                    debug!("MultiIOSpiOp not supported - no callback provided");
                    self.transport.write(&[S_NAK]).await.map_err(|_| {
                        SerprogError::TransportWrite("Error writing MultiIOSpiOp NAK")
                    })?;
                    Ok(())
                }
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
