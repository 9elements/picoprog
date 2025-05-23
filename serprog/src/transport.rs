use crate::{SerprogError, S_ACK, S_NAK};
use core::future;
use defmt::debug;
use embassy_futures::{block_on, join::join};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::zerocopy_channel::{Channel, Receiver, Sender};
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;

pub trait Transport {
    fn read(&mut self, buf: &mut [u8]) -> impl future::Future<Output = Result<(), ()>>;
    fn write(&mut self, data: &[u8]) -> impl future::Future<Output = Result<(), ()>>;
}

pub trait OSpiOpCallback<SPI, CS, T> {
    #[allow(async_fn_in_trait)]
    async fn handle_ospi_op(
        &mut self,
        spi: &mut SPI,
        cs: &mut CS,
        transport: &mut T,
    ) -> Result<(), SerprogError>
    where
        CS: OutputPin,
        T: Transport,
        CS::Error: core::fmt::Debug;
}

pub struct DefaultOSpiOpCallback;

impl<SPI: SpiBus<u8>, CS, T> OSpiOpCallback<SPI, CS, T> for DefaultOSpiOpCallback {
    async fn handle_ospi_op(
        &mut self,
        spi: &mut SPI,
        cs: &mut CS,
        transport: &mut T,
    ) -> Result<(), SerprogError>
    where
        SPI: SpiBus<u8>,
        CS: OutputPin,
        T: Transport,
        CS::Error: core::fmt::Debug,
    {
        let mut sdata = [0_u8; 64];
        transport
            .read(sdata.as_mut_slice())
            .await
            .map_err(|_| SerprogError::TransportRead("Error reading OSpiOp data"))?;

        let op_slen = crate::le_u24_to_u32(&sdata[0..3]) as usize;
        let op_rlen = crate::le_u24_to_u32(&sdata[3..6]) as usize;
        let mut usb_rx_spi_tx_buf = [([0u8; 64], 0); 4];
        let mut usb_rx_spi_tx_channel: Channel<'_, NoopRawMutex, ([u8; 64], usize)> =
            Channel::new(&mut usb_rx_spi_tx_buf);
        let (usb_rx, spi_tx) = usb_rx_spi_tx_channel.split();

        let mut usb_tx_spi_rx_buf = [([0u8; 64], 0); 8];
        let mut usb_tx_spi_rx_channel: Channel<'_, NoopRawMutex, ([u8; 64], usize)> =
            Channel::new(&mut usb_tx_spi_rx_buf);
        let (spi_rx, usb_tx) = usb_tx_spi_rx_channel.split();

        async fn usb_task<T: Transport>(
            transport: &mut T,
            mut sender: Sender<'_, NoopRawMutex, ([u8; 64], usize)>,
            sdata_size: usize,
            sdata_0: [u8; 64],
            mut receiver: Receiver<'_, NoopRawMutex, ([u8; 64], usize)>,
            rdata_size: usize,
        ) -> Result<(), SerprogError> {
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
                transport
                    .read(&mut buf[..read_size])
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
                let (buf, size) = receiver.receive().await;
                let size = *size;
                transport
                    .write(&buf[..size])
                    .await
                    .map_err(|_| SerprogError::TransportWrite("Error writing SPI read data"))?;
                receiver.receive_done();
                data_to_send -= size;
            }
            Ok(())
        }

        async fn spi_task<SPI: SpiBus<u8>, CS: OutputPin>(
            spi: &mut SPI,
            mut receiver: Receiver<'_, NoopRawMutex, ([u8; 64], usize)>,
            sdata_size: usize,
            mut sender: Sender<'_, NoopRawMutex, ([u8; 64], usize)>,
            rdata_size: usize,
            cs: &mut CS,
        ) -> Result<(), SerprogError>
        where
            CS::Error: core::fmt::Debug,
        {
            spi.flush()
                .await
                .map_err(|_| SerprogError::SpiFlush("Error flushing SPI before transfer"))?;

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
        }

        let (spi_res, usb_res) = block_on(join(
            spi_task(spi, spi_tx, op_slen, spi_rx, op_rlen, cs),
            usb_task(transport, usb_rx, op_slen, sdata, usb_tx, op_rlen),
        ));
        if let Err(spi_err) = spi_res {
            transport
                .write(&[S_NAK])
                .await
                .map_err(|_| SerprogError::TransportWrite("Failed to report SPI failed"))?;
            return Err(spi_err);
        }
        usb_res?;

        Ok(())
    }
}

impl<'d, D: embassy_usb::driver::Driver<'d>> Transport for CdcAcmClass<'d, D> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<(), ()> {
        let packet_size = self.max_packet_size() as usize;
        let buf_len = buf.len();

        // Use a buffer large enough for full speed and high speed
        let mut buffer = [0; 512];
        let mut size = 0;
        if buf_len < packet_size {
            let bytes_read = self
                .read_packet(&mut buffer[..packet_size])
                .await
                .map_err(|_| ())?;
            size = bytes_read;
            buf.copy_from_slice(&buffer[..buf_len]);
        } else {
            for chunk in buf.chunks_mut(packet_size) {
                let bytes_read = self.read_packet(chunk).await.map_err(|_| ())?;
                size += bytes_read;
                if bytes_read < chunk.len() {
                    break;
                }
            }
        }

        if size > buf_len {
            return Err(());
        }
        Ok(())
    }

    async fn write(&mut self, data: &[u8]) -> Result<(), ()> {
        for chunk in data.chunks(self.max_packet_size().into()) {
            self.write_packet(chunk).await.map_err(|_| ())?
        }
        Ok(())
    }
}
