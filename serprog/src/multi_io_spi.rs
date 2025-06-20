use num_enum::{IntoPrimitive, TryFromPrimitive};

#[derive(Debug, Clone, Copy, Eq, PartialEq, TryFromPrimitive, IntoPrimitive)]
#[repr(u8)]
pub enum MultiIOMode {
    SingleIO111 = 0,
    DualOut112 = 1,
    DualIO122 = 2,
    QuadOut114 = 3,
    QuadIO144 = 4,
    QPI444 = 5,
}

/// Address width and value for Multi-IO SPI operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Address {
    Addr24(u32),
    Addr32(u32),
}

/// Configuration for a Multi-IO SPI transaction
#[derive(Debug, Clone, Copy)]
pub struct MultiIOTransaction {
    pub mode: MultiIOMode,
    pub opcode: u8,
    pub address: Option<Address>,
    pub mode_byte: bool,
    pub dummy_cycles: u8,
}

/// Trait for Multi-IO SPI operations supporting different I/O modes
pub trait MultiIOSpi<Word: Copy + 'static = u8> {
    type Error: core::fmt::Debug;

    /// Maximum transaction buffer size supported by the implementation
    const MAX_TRANSACTION_SIZE: usize;

    /// Reset the Multi-IO SPI interface to a known state
    async fn reset(&mut self) -> Result<(), Self::Error>;

    /// Read data using Multi-IO SPI transaction
    async fn read(&mut self, transaction: MultiIOTransaction, buf: &mut [Word]) -> Result<(), Self::Error>;

    /// Write data using Multi-IO SPI transaction
    async fn write(&mut self, transaction: MultiIOTransaction, buf: &[Word]) -> Result<(), Self::Error>;

    /// Returns a bitmask of supported MultiIO modes
    /// Bit positions correspond to MultiIOMode enum values:
    /// bit 0: Single I/O (1-1-1), bit 1: Dual Output (1-1-2), etc.
    fn supported_modes(&self) -> Word;
}

/// No-operation MultiIO SPI that does nothing
pub struct NoMultiIO;

/// Error type for NoMultiIO operations
pub struct NoMultiIOError;

impl core::fmt::Debug for NoMultiIOError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "NoMultiIOError")
    }
}

impl MultiIOSpi for NoMultiIO {
    type Error = NoMultiIOError;

    const MAX_TRANSACTION_SIZE: usize = 0;

    async fn reset(&mut self) -> Result<(), Self::Error> {
        Err(NoMultiIOError)
    }

    async fn read(
        &mut self,
        _transaction: MultiIOTransaction,
        _buf: &mut [u8],
    ) -> Result<(), Self::Error> {
        Err(NoMultiIOError)
    }

    async fn write(&mut self, _transaction: MultiIOTransaction, _buf: &[u8]) -> Result<(), Self::Error> {
        Err(NoMultiIOError)
    }

    fn supported_modes(&self) -> u8 {
        0 // No modes supported
    }
}
