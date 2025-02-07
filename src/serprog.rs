use embassy_rp::gpio::Output;
use embassy_rp::spi::{Async, Instance as SpiInstance, Spi};
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_usb::driver::EndpointError;

const S_ACK: u8 = 0x06;
const S_NAK: u8 = 0x15;
const MAX_BUFFER_SIZE: usize = 2048;

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

impl From<u8> for SerprogCommand {
    fn from(value: u8) -> Self {
        match value {
            0x00 => SerprogCommand::Nop,
            0x01 => SerprogCommand::QIface,
            0x02 => SerprogCommand::QCmdMap,
            0x03 => SerprogCommand::QPgmName,
            0x04 => SerprogCommand::QSerBuf,
            0x05 => SerprogCommand::QBustype,
            0x06 => SerprogCommand::QChipSize,
            0x07 => SerprogCommand::QOpBuf,
            0x08 => SerprogCommand::QWrNMaxLen,
            0x09 => SerprogCommand::RByte,
            0x0A => SerprogCommand::RNBytes,
            0x0B => SerprogCommand::OInit,
            0x0C => SerprogCommand::OWriteB,
            0x0D => SerprogCommand::OWriteN,
            0x0E => SerprogCommand::ODelay,
            0x0F => SerprogCommand::OExec,
            0x10 => SerprogCommand::SyncNop,
            0x11 => SerprogCommand::QRdNMaxLen,
            0x12 => SerprogCommand::SBustype,
            0x13 => SerprogCommand::OSpiOp,
            0x14 => SerprogCommand::SSpiFreq,
            0x15 => SerprogCommand::SPinState,
            0x16 => SerprogCommand::SSpiCs,
            _ => SerprogCommand::Nop,
        }
    }
}

pub const CMDMAP: u32 = (1 << SerprogCommand::Nop as u32)
    | (1 << SerprogCommand::QIface as u32)
    | (1 << SerprogCommand::QCmdMap as u32)
    | (1 << SerprogCommand::QPgmName as u32)
    | (1 << SerprogCommand::QSerBuf as u32)
    | (1 << SerprogCommand::QWrNMaxLen as u32)
    | (1 << SerprogCommand::QBustype as u32)
    | (1 << SerprogCommand::SyncNop as u32)
    | (1 << SerprogCommand::QRdNMaxLen as u32)
    | (1 << SerprogCommand::OSpiOp as u32)
    | (1 << SerprogCommand::SBustype as u32)
    | (1 << SerprogCommand::SSpiFreq as u32)
    | (1 << SerprogCommand::SPinState as u32);

pub struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("USB buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

pub struct Serprog<SPI: SpiInstance + 'static> {
    spi: Spi<'static, SPI, Async>,
    cs: Output<'static>,
    led: Output<'static>,
}

impl<SPI: SpiInstance + 'static> Serprog<SPI> {
    pub fn new(spi: Spi<'static, SPI, Async>, cs: Output<'static>, led: Output<'static>) -> Self {
        Self { spi, cs, led }
    }

    pub async fn handle_command<'a, D: embassy_usb::driver::Driver<'static>>(
        &'a mut self,
        cmd: SerprogCommand,
        class: &'a mut CdcAcmClass<'static, D>,
        buf: &'a mut [u8],
    ) {
        match cmd {
            SerprogCommand::Nop => {
                log::debug!("Received Nop CMD");
                if let Err(e) = class.write_packet(&[S_ACK]).await {
                    log::error!("Error writing packet: {:?}", e);
                }
            }
            SerprogCommand::QIface => {
                log::debug!("Received QIface CMD");
                if let Err(e) = class.write_packet(&[S_ACK, 0x01, 0x00]).await {
                    log::error!("Error writing packet: {:?}", e);
                }
            }
            SerprogCommand::QCmdMap => {
                log::debug!("Received QCmdMap CMD");
                let cmdmap_bytes = CMDMAP.to_le_bytes();
                let mut packet = [0; 33];
                packet[0] = S_ACK;
                packet[1] = cmdmap_bytes[0];
                packet[2] = cmdmap_bytes[1];
                packet[3] = cmdmap_bytes[2];
                packet[4] = cmdmap_bytes[3];
                if let Err(e) = class.write_packet(&packet).await {
                    log::error!("Error writing packet: {:?}", e);
                }
            }
            SerprogCommand::QPgmName => {
                log::debug!("Received QPgmName CMD");
                if let Err(e) = class
                    .write_packet(&[
                        S_ACK, b'P', b'i', b'c', b'o', b'p', b'r', b'o', b'g', 0, 0, 0, 0, 0, 0, 0,
                        0,
                    ])
                    .await
                {
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
                let size_24 = MAX_BUFFER_SIZE.to_le_bytes();
                if let Err(e) = class
                    .write_packet(&[S_ACK, size_24[0], size_24[1], size_24[2]])
                    .await
                {
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
                let op_slen = u32::from_le_bytes([buf[0], buf[1], buf[2], 0]);
                let op_rlen = u32::from_le_bytes([buf[3], buf[4], buf[5], 0]);

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
                if let Err(e) = self.spi.flush() {
                    log::error!("Error flushing SPI: {:?}", e);
                }

                self.cs.set_low();
                match self.spi.write(&sdata[..op_slen as usize]).await {
                    Ok(_) => {
                        log::debug!("SPI write successful");
                        log::debug!("Sent data (sdata): {:?}", &sdata[..op_slen as usize]);
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
                    Err(e) => {
                        log::error!("SPI write error: {:?}", e);
                        if let Err(e) = class.write_packet(&[S_NAK]).await {
                            log::error!("Error writing NAK: {:?}", e);
                        }
                    }
                }
                self.cs.set_high();
            }
            SerprogCommand::SSpiFreq => {
                log::debug!("Received SSpiFreq CMD");
                if let Err(e) = class.read_packet(buf).await {
                    log::error!("Error reading packet: {:?}", e);
                    return;
                }
                log::debug!("Received SSpiFreq packet: {:?}", buf);
                let try_freq = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
                log::debug!("Setting SPI frequency: {:?}", try_freq);
                self.spi.set_frequency(try_freq);
                let actual_freq = try_freq.to_le_bytes(); // TODO can we report what the hardware has set up?
                if let Err(e) = class
                    .write_packet(&[
                        S_ACK,
                        actual_freq[0],
                        actual_freq[1],
                        actual_freq[2],
                        actual_freq[3],
                    ])
                    .await
                {
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
                    self.led.set_low();
                } else {
                    self.led.set_high();
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
