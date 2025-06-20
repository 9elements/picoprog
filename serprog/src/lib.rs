#![no_std]

use core::convert::From;
use core::result::Result::{Err, Ok};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::zerocopy_channel::{Receiver, Sender};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;
use num_enum::{IntoPrimitive, TryFromPrimitive};
use tock_registers::register_bitfields;
use tock_registers::LocalRegisterCopy;
use zerocopy::byteorder::little_endian::{U16, U32};
use zerocopy::{FromBytes, FromZeros, Immutable, IntoBytes, Unaligned};

use defmt::{debug, error, Format};

pub mod usb_task;
use usb_task::UsbCommand;

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

pub struct Serprog<SPI, CS, LED, F> {
    spi: SPI,
    cs: CS,
    led: LED,
    usb_cmd_sender: Sender<'static, NoopRawMutex, UsbCommand>,
    usb_data_to_receiver: Receiver<'static, NoopRawMutex, Result<heapless::Vec<u8, 64>, ()>>,
    freq_callback: Option<F>,
}

impl<SPI, CS, LED, F> Serprog<SPI, CS, LED, F>
where
    SPI: SpiBus<u8>,
    CS: OutputPin,
    LED: OutputPin,
    F: FnMut(&mut SPI, u32) + Send + Sync,
{
    pub fn new(
        spi: SPI,
        cs: CS,
        led: LED,
        usb_cmd_sender: Sender<'static, NoopRawMutex, UsbCommand>,
        usb_data_to_receiver: Receiver<'static, NoopRawMutex, Result<heapless::Vec<u8, 64>, ()>>,
        freq_callback: Option<F>,
    ) -> Self {
        Self {
            spi,
            cs,
            led,
            usb_cmd_sender,
            usb_data_to_receiver,
            freq_callback,
        }
    }

    pub async fn run_loop(mut self) -> ! {
        loop {
            // Clear both sender and receiver to start each command from a clean sheet
            self.usb_cmd_sender.clear();
            self.usb_data_to_receiver.clear();

            // Request 1 byte read for command
            let mut cmd_buf = [0u8; 1];
            let cmd_byte = match self.usb_read(&mut cmd_buf).await {
                Ok(()) => cmd_buf[0],
                Err(_) => {
                    error!("Read error in main loop");
                    continue;
                }
            };

            let cmd = SerprogCommand::try_from(cmd_byte).unwrap_or(SerprogCommand::Nop);
            if let Err(e) = self.handle_command(cmd).await {
                error!("Command error: {:?}", e);
            }
        }
    }

    async fn usb_write(&mut self, data: &[u8]) -> Result<(), SerprogError> {
        let mut usb_buf = heapless::Vec::new();
        usb_buf
            .resize(data.len(), 0)
            .map_err(|_| SerprogError::TransportWrite("Buffer too small"))?;
        usb_buf[..data.len()].copy_from_slice(data);

        let cmd = self.usb_cmd_sender.send().await;
        *cmd = UsbCommand::Write { data: usb_buf };
        self.usb_cmd_sender.send_done();

        let result = self.usb_data_to_receiver.receive().await;
        match result {
            Ok(_) => {
                self.usb_data_to_receiver.receive_done();
                Ok(())
            }
            Err(_) => {
                self.usb_data_to_receiver.receive_done();
                Err(SerprogError::TransportWrite("USB write failed"))
            }
        }
    }

    async fn usb_read(&mut self, buf: &mut [u8]) -> Result<(), SerprogError> {
        let cmd = self.usb_cmd_sender.send().await;
        *cmd = UsbCommand::Read { size: buf.len() };
        self.usb_cmd_sender.send_done();

        let result = self.usb_data_to_receiver.receive().await;
        let data = match result {
            Ok(data) => data,
            Err(_) => {
                self.usb_data_to_receiver.receive_done();
                return Err(SerprogError::TransportRead("USB read failed"));
            }
        };
        let read_size = data.len().min(buf.len());
        buf[..read_size].copy_from_slice(&data[..read_size]);
        self.usb_data_to_receiver.receive_done();
        Ok(())
    }

    async fn handle_command(&mut self, cmd: SerprogCommand) -> Result<(), SerprogError>
    where
        CS::Error: core::fmt::Debug,
        LED::Error: core::fmt::Debug,
    {
        match cmd {
            SerprogCommand::Nop => {
                debug!("Received Nop CMD");
                self.usb_write(&[S_ACK]).await?;
                Ok(())
            }
            SerprogCommand::QIface => {
                debug!("Received QIface CMD");
                let response = QIfaceResponse {
                    ack: S_ACK,
                    version: U16::new(1),
                };
                self.usb_write(response.as_bytes()).await?;
                Ok(())
            }
            SerprogCommand::QCmdMap => {
                debug!("Received QCmdMap CMD");
                let response = QCmdMapResponse::new(self.freq_callback.is_some());
                self.usb_write(response.as_bytes()).await?;
                Ok(())
            }
            SerprogCommand::QPgmName => {
                debug!("Received QPgmName CMD");
                let response = QPgmNameResponse::new("Picoprog");
                self.usb_write(response.as_bytes()).await?;
                Ok(())
            }
            SerprogCommand::QSerBuf => {
                debug!("Received QSerBuf CMD");
                self.usb_write(&[S_ACK, 0xFF, 0xFF]).await?;
                Ok(())
            }
            SerprogCommand::QWrNMaxLen | SerprogCommand::QRdNMaxLen => {
                debug!("Received QWrNMaxLen/QRdNMaxLen CMD");
                let response = QMaxLenResponse::new(MAX_BUFFER_SIZE);
                self.usb_write(response.as_bytes()).await?;
                Ok(())
            }
            SerprogCommand::QBustype => {
                debug!("Received QBustype CMD");
                self.usb_write(&[S_ACK, 0x08]).await?;
                Ok(())
            }
            SerprogCommand::SyncNop => {
                debug!("Received SyncNop CMD");
                self.usb_write(&[S_NAK, S_ACK]).await?;
                Ok(())
            }
            SerprogCommand::SBustype => {
                debug!("Received SBustype CMD");
                let mut buf = [0u8; 1];
                self.usb_read(&mut buf).await?;
                if buf[0] == 0x08 {
                    debug!("Received SBustype 'SPI'");
                    self.usb_write(&[S_ACK]).await?;
                } else {
                    debug!("Received unknown SBustype");
                    self.usb_write(&[S_NAK]).await?;
                }
                Ok(())
            }
            SerprogCommand::OSpiOp => {
                debug!("Received OSpiOp CMD");
                let cmd = self.usb_cmd_sender.send().await;
                *cmd = UsbCommand::Read { size: 64 };
                self.usb_cmd_sender.send_done();

                let result = self.usb_data_to_receiver.receive().await;
                let sdata = match result {
                    Ok(data) => data,
                    Err(_) => {
                        self.usb_data_to_receiver.receive_done();
                        return Err(SerprogError::TransportRead("USB read failed"));
                    }
                };

                let op_slen = le_u24_to_u32(&sdata[0..3]) as usize;
                let op_rlen = le_u24_to_u32(&sdata[3..6]) as usize;

                // Handle SPI operation directly without complex async patterns
                self.spi
                    .flush()
                    .await
                    .map_err(|_| SerprogError::SpiFlush("Error flushing SPI before transfer"))?;

                self.cs
                    .set_low()
                    .map_err(|_| SerprogError::CsSetLow("Error setting CS low"))?;

                // Write phase: read data from USB and write to SPI
                let mut data_to_write = op_slen;

                // Handle first block from sdata
                if data_to_write > 0 {
                    assert!(sdata.len() - 6 <= data_to_write);
                    self.spi
                        .write(&sdata[6..])
                        .await
                        .map_err(|_| SerprogError::SpiTransfer("Error writing OSpiOp data"))?;
                    data_to_write -= sdata.len() - 6;
                }
                self.usb_data_to_receiver.receive_done();

                let usb_reader = async {
                    let mut remaining = data_to_write;
                    while remaining > 0 {
                        let chunk_size = remaining.min(64);
                        let cmd = self.usb_cmd_sender.send().await;
                        *cmd = UsbCommand::Read { size: chunk_size };
                        self.usb_cmd_sender.send_done();
                        remaining -= chunk_size;
                    }
                    Ok::<(), SerprogError>(())
                };

                let spi_writer = async {
                    let mut remaining = data_to_write;
                    while remaining > 0 {
                        let result = self.usb_data_to_receiver.receive().await;
                        let data = match result {
                            Ok(data) => data,
                            Err(_) => {
                                self.usb_data_to_receiver.receive_done();
                                return Err(SerprogError::TransportRead("USB read failed"));
                            }
                        };

                        self.spi
                            .write(data)
                            .await
                            .map_err(|_| SerprogError::SpiTransfer("Error writing OSpiOp data"))?;

                        remaining -= data.len();
                        self.usb_data_to_receiver.receive_done();
                    }
                    Ok::<(), SerprogError>(())
                };

                let (usb_result, spi_result) =
                    embassy_futures::join::join(usb_reader, spi_writer).await;
                usb_result?;
                spi_result?;

                // Send ACK after write phase
                self.usb_write(&[S_ACK]).await?;

                // Read phase: read from SPI and send to USB
                let mut data_to_read = op_rlen;
                while data_to_read > 0 {
                    let read_size = data_to_read.min(64);

                    let cmd = self.usb_cmd_sender.send().await;
                    *cmd = UsbCommand::Write {
                        data: heapless::Vec::new(),
                    };

                    if let UsbCommand::Write { data } = cmd {
                        // Use unsafe set_len to set the correct size without initialization
                        unsafe {
                            data.set_len(read_size);
                        }

                        // Read directly from SPI into the buffer
                        self.spi
                            .read(data)
                            .await
                            .map_err(|_| SerprogError::SpiTransfer("Error reading OSpiOp data"))?;
                    }

                    self.usb_cmd_sender.send_done();

                    // Wait for USB write to complete and check result
                    let result = self.usb_data_to_receiver.receive().await;
                    match result {
                        Ok(_) => {
                            self.usb_data_to_receiver.receive_done();
                        }
                        Err(_) => {
                            self.usb_data_to_receiver.receive_done();
                            return Err(SerprogError::TransportWrite("USB write failed during SPI read"));
                        }
                    }

                    data_to_read -= read_size;
                }

                self.cs
                    .set_high()
                    .map_err(|_| SerprogError::CsSetHigh("Error setting CS high"))?;
                debug!("OSpiOp CMD done");
                Ok(())
            }
            SerprogCommand::SSpiFreq => {
                debug!("Received SSpiFreq CMD");
                let mut request = SSpiFreqRequest::new_zeroed();
                self.usb_read(request.as_mut_bytes()).await?;

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

                self.usb_write(response.as_bytes()).await?;
                Ok(())
            }
            SerprogCommand::SPinState => {
                debug!("Received SPinState CMD");
                let mut buf = [0u8; 1];
                self.usb_read(&mut buf).await?;
                if buf[0] == 0 {
                    self.led
                        .set_low()
                        .map_err(|_| SerprogError::LedSetLow("Error setting LED low"))?;
                } else {
                    self.led
                        .set_high()
                        .map_err(|_| SerprogError::LedSetHigh("Error setting LED high"))?;
                }
                self.usb_write(&[S_ACK]).await?;
                Ok(())
            }
            _ => {
                debug!("Received unknown CMD");
                self.usb_write(&[S_NAK]).await?;
                Ok(())
            }
        }
    }
}
