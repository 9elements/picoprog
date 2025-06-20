use crate::ChangeSpiFreq;
use embedded_hal_async::spi::SpiBus;

/// A no-op SPI implementation for when SPI is not available
/// Use None::<NoSpi> as function argument in this case
pub struct NoSpi;

#[derive(Debug)]
pub struct NoSpiError;

impl embedded_hal::spi::Error for NoSpiError {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        embedded_hal::spi::ErrorKind::Other
    }
}

impl ChangeSpiFreq for NoSpi {
    const SUPPORTED: bool = false;
}

impl embedded_hal_async::spi::ErrorType for NoSpi {
    type Error = NoSpiError;
}

impl SpiBus<u8> for NoSpi {
    async fn read(&mut self, _words: &mut [u8]) -> Result<(), Self::Error> {
        Err(NoSpiError)
    }

    async fn write(&mut self, _words: &[u8]) -> Result<(), Self::Error> {
        Err(NoSpiError)
    }

    async fn transfer(&mut self, _read: &mut [u8], _write: &[u8]) -> Result<(), Self::Error> {
        Err(NoSpiError)
    }

    async fn transfer_in_place(&mut self, _words: &mut [u8]) -> Result<(), Self::Error> {
        Err(NoSpiError)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Err(NoSpiError)
    }
}
