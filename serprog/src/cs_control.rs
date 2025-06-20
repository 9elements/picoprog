use embedded_hal::digital::OutputPin;

pub trait CsControl {
    type Error: core::fmt::Debug;

    fn set_low(&mut self) -> Result<(), Self::Error>;
    fn set_high(&mut self) -> Result<(), Self::Error>;
    fn select_cs(&mut self, cs_index: u8) -> Result<(), Self::Error>;
}

/// No-operation CS control that does nothing
pub struct NoCs;

/// Error type for NoCs operations
pub struct NoCsError;

impl core::fmt::Debug for NoCsError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "NoCsError")
    }
}

impl CsControl for NoCs {
    type Error = NoCsError;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Err(NoCsError)
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Err(NoCsError)
    }

    fn select_cs(&mut self, _cs_index: u8) -> Result<(), Self::Error> {
        Err(NoCsError)
    }
}

impl<T: OutputPin> CsControl for T
where
    T::Error: core::fmt::Debug,
{
    type Error = T::Error;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        OutputPin::set_low(self)
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        OutputPin::set_high(self)
    }

    fn select_cs(&mut self, cs_index: u8) -> Result<(), Self::Error> {
        // Default implementation for single CS pin
        // Only CS 0 is supported, return error for others
        if cs_index == 0 {
            Ok(())
        } else {
            // We can't return a proper error here since we don't know the specific error type
            // This is a limitation of the default implementation
            panic!("Multiple CS selection not supported for basic OutputPin")
        }
    }
}
