#![no_std]

use core::convert::From;
use core::result::Result::{Err, Ok};
use embassy_futures::{block_on, join::join};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::zerocopy_channel::{Channel, Receiver, Sender};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;
use heapless::Vec;
use num_enum::{IntoPrimitive, TryFromPrimitive};
use tock_registers::register_bitfields;
use tock_registers::LocalRegisterCopy;
use transport::Transport;
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
fn le_u24_to_u32(bytes: &[u8]) -> u32 {
    u32::from_le_bytes([bytes[0], bytes[1], bytes[2], 0])
}

const S_ACK: u8 = 0x06;
const S_NAK: u8 = 0x15;
const MAX_BUFFER_SIZE: usize = 16 << 20;

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
            + Commands::SPinState::SET;

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

pub struct Serprog<SPI, CS, LED, T: Transport, F> {
    spi: SPI,
    cs: CS,
    led: LED,
    transport: T,
    freq_callback: Option<F>,
}

async fn ospiop_usb_task<T: Transport>(
    transport: &mut T,
    mut sender: Sender<'_, NoopRawMutex, Vec<u8, 64>>,
    sdata_size: usize,
    mut receiver: Receiver<'_, NoopRawMutex, Vec<u8, 64>>,
    rdata_size: usize,
) -> Result<(), SerprogError> {
    // First block - already contains header + initial data, send as-is
    let mut data_to_read = sdata_size;
    let first_block_data_size = data_to_read.min(64 - 6);
    sender.send_done();
    data_to_read -= first_block_data_size;

    while data_to_read > 0 {
        let read_size = data_to_read.min(64);
        let buf = sender.send().await;
        buf.clear();
        buf.resize(read_size, 0).ok();
        transport
            .read(buf.as_mut_slice())
            .await
            .map_err(|_| SerprogError::TransportRead("Error reading OSpiOp data"))?;
        sender.send_done();
        data_to_read -= read_size;
    }
    transport
        .write(&[S_ACK])
        .await
        .map_err(|_| SerprogError::TransportWrite("Error writing SBustype ACK"))?;

    let mut data_to_send = rdata_size;
    while data_to_send > 0 {
        let buf = receiver.receive().await;
        transport
            .write(buf.as_slice())
            .await
            .map_err(|_| SerprogError::TransportWrite("Error writing SPI read data"))?;
        data_to_send -= buf.len();
        receiver.receive_done();
    }
    Ok(())
}

async fn ospiop_spi_task<SPI: SpiBus<u8>, CS: OutputPin>(
    spi: &mut SPI,
    mut receiver: Receiver<'_, NoopRawMutex, Vec<u8, 64>>,
    sdata_size: usize,
    mut sender: Sender<'_, NoopRawMutex, Vec<u8, 64>>,
    rdata_size: usize,
    cs: &mut CS,
) -> Result<(), SerprogError> {
    spi.flush()
        .await
        .map_err(|_| SerprogError::SpiFlush("Error flushing SPI before transfer"))?;

    cs.set_low()
        .map_err(|_| SerprogError::CsSetLow("Error setting CS low"))?;
    let mut data_to_write = sdata_size;
    let mut is_first = true;
    while data_to_write > 0 {
        let buf = receiver.receive().await;
        let write_slice = if is_first {
            // First buffer: skip the 6-byte header
            is_first = false;
            let data_len = data_to_write.min(64 - 6);
            data_to_write -= data_len;
            &buf[6..6 + data_len]
        } else {
            // Subsequent buffers: use entire buffer
            data_to_write -= buf.len();
            buf.as_slice()
        };
        spi.write(write_slice)
            .await
            .map_err(|_| SerprogError::SpiTransfer("Error writing OSpiOp data"))?;
        receiver.receive_done();
    }
    let mut data_to_read = rdata_size;
    while data_to_read > 0 {
        let buf = sender.send().await;
        buf.clear();
        let read_size = data_to_read.min(64);
        buf.resize(read_size, 0).ok();
        spi.read(buf.as_mut_slice())
            .await
            .map_err(|_| SerprogError::SpiTransfer("Error reading OSpiOp data"))?;
        sender.send_done();
        data_to_read -= read_size;
    }
    cs.set_high()
        .map_err(|_| SerprogError::CsSetHigh("Error setting CS high"))?;
    debug!("OSpiOp CMD done");
    Ok(())
}

impl<SPI, CS, LED, T, F> Serprog<SPI, CS, LED, T, F>
where
    SPI: SpiBus<u8>,
    CS: OutputPin,
    LED: OutputPin,
    T: Transport,
    F: FnMut(&mut SPI, u32) + Send + Sync,
{
    pub fn new(spi: SPI, cs: CS, led: LED, transport: T, freq_callback: Option<F>) -> Self {
        Self {
            spi,
            cs,
            led,
            transport,
            freq_callback,
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
                let response = QCmdMapResponse::new(self.freq_callback.is_some());
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

                // Read directly into the first channel buffer
                let mut usb_rx_spi_tx_buf = [const { Vec::<u8, 64>::new() }; 4];
                usb_rx_spi_tx_buf[0].resize(64, 0).ok();
                self.transport
                    .read(usb_rx_spi_tx_buf[0].as_mut_slice())
                    .await
                    .map_err(|_| SerprogError::TransportRead("Error reading OSpiOp data"))?;

                // Parse header from the first buffer
                let op_slen = le_u24_to_u32(&usb_rx_spi_tx_buf[0][0..3]) as usize;
                let op_rlen = le_u24_to_u32(&usb_rx_spi_tx_buf[0][3..6]) as usize;

                let mut usb_rx_spi_tx_channel: Channel<'_, NoopRawMutex, Vec<u8, 64>> =
                    Channel::new(&mut usb_rx_spi_tx_buf);
                let (usb_rx, spi_tx) = usb_rx_spi_tx_channel.split();

                let mut usb_tx_spi_rx_buf = [const { Vec::<u8, 64>::new() }; 8];
                let mut usb_tx_spi_rx_channel: Channel<'_, NoopRawMutex, Vec<u8, 64>> =
                    Channel::new(&mut usb_tx_spi_rx_buf);
                let (spi_rx, usb_tx) = usb_tx_spi_rx_channel.split();

                let (spi_res, usb_res) = block_on(join(
                    ospiop_spi_task(
                        &mut self.spi,
                        spi_tx,
                        op_slen,
                        spi_rx,
                        op_rlen,
                        &mut self.cs,
                    ),
                    ospiop_usb_task(&mut self.transport, usb_rx, op_slen, usb_tx, op_rlen),
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
