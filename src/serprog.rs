use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_usb::driver::EndpointError;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;
use num_enum::{IntoPrimitive, TryFromPrimitive};
use tock_registers::register_bitfields;
use tock_registers::LocalRegisterCopy;
use zerocopy::byteorder::little_endian::{U16, U32};
use zerocopy::{FromBytes, Immutable, IntoBytes, Unaligned};

// Convert 3 bytes in little-endian format to u32
fn le_u24_to_u32(bytes: &[u8]) -> u32 {
    u32::from_le_bytes([bytes[0], bytes[1], bytes[2], 0])
}

const S_ACK: u8 = 0x06;
const S_NAK: u8 = 0x15;
const MAX_BUFFER_SIZE: usize = 2048;

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
    Nop = 0x00,        // No operation
    QIface = 0x01,     // Query interface version
    QCmdMap = 0x02,    // Query supported commands bitmap
    QPgmName = 0x03,   // Query programmer name
    QSerBuf = 0x04,    // Query Serial Buffer Size
    QBustype = 0x05,   // Query supported bustypes
    QChipSize = 0x06,  // Query supported chipsize (2^n format)
    QOpBuf = 0x07,     // Query operation buffer size
    QWrNMaxLen = 0x08, // Query Write to opbuf: Write-N maximum length
    RByte = 0x09,      // Read a single byte
    RNBytes = 0x0A,    // Read n bytes
    OInit = 0x0B,      // Initialize operation buffer
    OWriteB = 0x0C,    // Write opbuf: Write byte with address
    OWriteN = 0x0D,    // Write to opbuf: Write-N
    ODelay = 0x0E,     // Write opbuf: udelay
    OExec = 0x0F,      // Execute operation buffer
    SyncNop = 0x10,    // Special no-operation that returns NAK+ACK
    QRdNMaxLen = 0x11, // Query read-n maximum length
    SBustype = 0x12,   // Set used bustype(s)
    OSpiOp = 0x13,     // Perform SPI operation
    SSpiFreq = 0x14,   // Set SPI clock frequency
    SPinState = 0x15,  // Enable/disable output drivers
    SSpiCs = 0x16,     // Select Chip Select to use
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
        SSpiCs OFFSET(22) NUMBITS(1) []
    ]
];

impl QCmdMapResponse {
    fn new() -> Self {
        let mut response = Self {
            ack: S_ACK,
            map: [0; 4],
            zeros: [0; 28],
        };

        // Set supported commands using tock-registers
        let mut cmdmap = LocalRegisterCopy::<u32, Commands::Register>::new(0);
        cmdmap.modify(
            Commands::Nop::SET
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
                + Commands::SSpiFreq::SET
                + Commands::SPinState::SET,
        );

        // Get the bits and store in first 4 bytes, rest stays as zeros
        let bits = cmdmap.get();
        response.map[0..4].copy_from_slice(&bits.to_le_bytes());

        response
    }
}

pub struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("USB buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

pub struct Serprog<SPI, CS, LED> {
    spi: SPI,
    cs: CS,
    led: LED,
}

impl<SPI, CS, LED> Serprog<SPI, CS, LED>
where
    SPI: SpiBus<u8>,
    CS: OutputPin,
    LED: OutputPin,
{
    pub fn new(spi: SPI, cs: CS, led: LED) -> Self {
        Self { spi, cs, led }
    }

    pub async fn handle_command<'a, D: embassy_usb::driver::Driver<'static>>(
        &'a mut self,
        cmd: SerprogCommand,
        class: &'a mut CdcAcmClass<'static, D>,
        buf: &'a mut [u8],
    ) where
        SPI::Error: core::fmt::Debug,
        CS::Error: core::fmt::Debug,
        LED::Error: core::fmt::Debug,
    {
        match cmd {
            SerprogCommand::Nop => {
                log::debug!("Received Nop CMD");
                if let Err(e) = class.write_packet(&[S_ACK]).await {
                    log::error!("Error writing packet: {:?}", e);
                }
            }
            SerprogCommand::QIface => {
                log::debug!("Received QIface CMD");
                let response = QIfaceResponse {
                    ack: S_ACK,
                    version: U16::new(1),
                };
                if let Err(e) = class.write_packet(response.as_bytes()).await {
                    log::error!("Error writing packet: {:?}", e);
                }
            }
            SerprogCommand::QCmdMap => {
                log::debug!("Received QCmdMap CMD");
                let response = QCmdMapResponse::new();
                if let Err(e) = class.write_packet(response.as_bytes()).await {
                    log::error!("Error writing packet: {:?}", e);
                }
            }
            SerprogCommand::QPgmName => {
                log::debug!("Received QPgmName CMD");
                let response = QPgmNameResponse::new("Picoprog");
                if let Err(e) = class.write_packet(response.as_bytes()).await {
                    log::error!("Error writing packet: {:?}", e);
                }
            }
            SerprogCommand::QSerBuf => {
                log::debug!("Received QSerBuf CMD");
                if let Err(e) = class.write_packet(&[S_ACK, 0xFF, 0xFF]).await {
                    log::error!("Error writing packet: {:?}", e);
                }
            }
            SerprogCommand::QWrNMaxLen | SerprogCommand::QRdNMaxLen => {
                log::debug!("Received QWrNMaxLen/QRdNMaxLen CMD");
                let response = QMaxLenResponse::new(MAX_BUFFER_SIZE);
                if let Err(e) = class.write_packet(response.as_bytes()).await {
                    log::error!("Error writing packet: {:?}", e);
                }
            }
            SerprogCommand::QBustype => {
                log::debug!("Received QBustype CMD");
                if let Err(e) = class.write_packet(&[S_ACK, 0x08]).await {
                    log::error!("Error writing packet: {:?}", e);
                }
            }
            SerprogCommand::SyncNop => {
                log::debug!("Received SyncNop CMD");
                if let Err(e) = class.write_packet(&[S_NAK, S_ACK]).await {
                    log::error!("Error writing packet: {:?}", e);
                }
            }
            SerprogCommand::SBustype => {
                log::debug!("Received SBustype CMD");
                if let Err(e) = class.read_packet(buf).await {
                    log::error!("Error reading packet: {:?}", e);
                    return;
                }
                if buf[0] == 0x08 {
                    log::debug!("Received SBustype 'SPI'");
                    if let Err(e) = class.write_packet(&[S_ACK]).await {
                        log::error!("Error writing packet: {:?}", e);
                    }
                } else {
                    log::debug!("Received unknown SBustype");
                    if let Err(e) = class.write_packet(&[S_NAK]).await {
                        log::error!("Error writing packet: {:?}", e);
                    }
                }
            }
            SerprogCommand::OSpiOp => {
                log::debug!("Received OSpiOp CMD");
                if let Err(e) = class.read_packet(buf).await {
                    log::error!("Error reading packet: {:?}", e);
                    return;
                }
                let op_slen = le_u24_to_u32(&buf[0..3]);
                let op_rlen = le_u24_to_u32(&buf[3..6]);

                let mut sdata = [0_u8; MAX_BUFFER_SIZE];
                let mut rdata = [0_u8; MAX_BUFFER_SIZE];

                // Copy initial chunk from buf starting at byte position 6 (sdata)
                let initial_chunk_size = core::cmp::min(64 - 6, op_slen as usize);
                sdata[..initial_chunk_size].copy_from_slice(&buf[6..6 + initial_chunk_size]);
                let mut bytes_read = initial_chunk_size;

                // Read the remaining sdata in chunks
                while bytes_read < op_slen as usize {
                    let chunk_size = core::cmp::min(64, op_slen as usize - bytes_read);
                    if let Err(e) = class.read_packet(&mut buf[..chunk_size]).await {
                        log::error!("Error reading packet: {:?}", e);
                        return;
                    }
                    sdata[bytes_read..bytes_read + chunk_size].copy_from_slice(&buf[..chunk_size]);
                    bytes_read += chunk_size;
                }

                log::debug!(
                    "Starting SPI transfer, sdata: {:?}, rdata: {:?}",
                    &sdata[..op_slen as usize],
                    &rdata[..op_rlen as usize]
                );

                // This call is blocking according to the SPI HAL
                if (self.spi.flush().await).is_err() {
                    log::error!("Error flushing SPI");
                }

                if self.cs.set_low().is_err() {
                    log::error!("Error setting CS low");
                }

                match self.spi.write(&sdata[..op_slen as usize]).await {
                    Ok(_) => {
                        log::debug!("SPI transfer successful");
                        log::debug!("Received data (rdata): {:?}", &rdata[..op_rlen as usize]);
                        match self.spi.read(&mut rdata[..op_rlen as usize]).await {
                            Ok(_) => {
                                log::debug!("SPI read successful");
                                log::debug!(
                                    "Received data (rdata): {:?}",
                                    &rdata[..op_rlen as usize]
                                );
                                if let Err(e) = class.write_packet(&[S_ACK]).await {
                                    log::error!("Error writing packet: {:?}", e);
                                }

                                // Send the full rdata in chunks
                                let mut bytes_written = 0;
                                while bytes_written < op_rlen as usize {
                                    let chunk_size =
                                        core::cmp::min(64, op_rlen as usize - bytes_written);
                                    if let Err(e) = class
                                        .write_packet(
                                            &rdata[bytes_written..bytes_written + chunk_size],
                                        )
                                        .await
                                    {
                                        log::error!("Error writing rdata: {:?}", e);
                                    }
                                    bytes_written += chunk_size;
                                }
                            }
                            Err(e) => {
                                log::error!("SPI read error: {:?}", e);
                                if let Err(e) = class.write_packet(&[S_NAK]).await {
                                    log::error!("Error writing NAK: {:?}", e);
                                }
                            }
                        }
                    }
                    Err(_) => {
                        log::error!("SPI transfer error");
                        if let Err(e) = class.write_packet(&[S_NAK]).await {
                            log::error!("Error writing NAK: {:?}", e);
                        }
                    }
                }

                if self.cs.set_high().is_err() {
                    log::error!("Error setting CS high");
                }
            }
            SerprogCommand::SSpiFreq => {
                log::debug!("Received SSpiFreq CMD");
                if let Err(e) = class.read_packet(buf).await {
                    log::error!("Error reading packet: {:?}", e);
                    return;
                }

                // Parse the request using zerocopy
                let request = SSpiFreqRequest::read_from_bytes(&buf[0..4]).unwrap();
                let try_freq = request.freq.get();

                log::debug!("Setting SPI frequency: {:?}", try_freq);
                // SPI frequency setting not supported in generic implementation

                // Create and send response
                let response = SSpiFreqResponse {
                    ack: S_ACK,
                    freq: U32::new(try_freq), // TODO can we report what the hardware has set up?
                };

                if let Err(e) = class.write_packet(response.as_bytes()).await {
                    log::error!("Error writing packet: {:?}", e);
                }
            }
            SerprogCommand::SPinState => {
                log::debug!("Received SPinState CMD");
                if let Err(e) = class.read_packet(buf).await {
                    log::error!("Error reading packet: {:?}", e);
                    return;
                }
                if buf[0] == 0 {
                    if self.led.set_low().is_err() {
                        log::error!("Error setting LED low");
                    }
                } else if self.led.set_high().is_err() {
                    log::error!("Error setting LED high");
                }
                if let Err(e) = class.write_packet(&[S_ACK]).await {
                    log::error!("Error writing packet: {:?}", e);
                }
            }
            _ => {
                log::debug!("Received unknown CMD");
                if let Err(e) = class.write_packet(&[S_NAK]).await {
                    log::error!("Error writing packet: {:?}", e);
                }
            }
        }
    }
}
