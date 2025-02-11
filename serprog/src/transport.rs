use core::future;
use embassy_usb::class::cdc_acm::CdcAcmClass;

pub trait Transport {
    fn read(&mut self, buf: &mut [u8]) -> impl future::Future<Output = Result<(), ()>>;
    fn write(&mut self, data: &[u8]) -> impl future::Future<Output = Result<(), ()>>;
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
