//! Interfaces to communicate with the LIS3DSH

/// Interface needed to read and write registers 
pub trait Interface {
    type Error;

    /// Should read from multiple registers, starting at address `addr`
    fn read_multiple_regs(&mut self, addr: u8, data: &mut [u8]) -> Result<(), Self::Error>;

    /// Should read the constents of a single register at address `addr`
    fn read_reg(&mut self, addr: u8) -> Result<u8, Self::Error>;

    /// Should write the `value` into the register at address `addr`
    fn write_reg(&mut self, addr: u8, value: u8) -> Result<(), Self::Error>;
}

pub use spi::SpiInterface as Spi;

mod spi {
    use embedded_hal as hal;
    use hal::{blocking::spi::Transfer, digital::v2::OutputPin};

    const READ_FLAG: u8 = 0x80;
    const ADDR_MASK: u8 = 0x7F;

    /// RAII wrapper for enabling and disabling the chip select pin
    ///
    /// Upon creating the CS pin is set to active, upon being dropped the
    /// CS pin is set to inactive
    struct ChipSelectGuard<'a, CSPIN>
    where
        CSPIN: OutputPin,
    {
        cs: &'a mut CSPIN,
    }

    impl<'a, CSPIN> ChipSelectGuard<'a, CSPIN>
    where
        CSPIN: OutputPin,
    {
        fn new(cs: &'a mut CSPIN) -> Self {
            let _ = cs.set_low();
            Self { cs }
        }
    }

    impl<'a, CSPIN> Drop for ChipSelectGuard<'a, CSPIN>
    where
        CSPIN: OutputPin,
    {
        fn drop(&mut self) {
            let _ = self.cs.set_high();
        }
    }

    /// SPI interface for interacting  with a LIS3DSH
    pub struct SpiInterface<SPI, CSPIN>
    where
        SPI: Transfer<u8>,
        CSPIN: OutputPin,
    {
        spi: SPI,
        cs: CSPIN,
    }

    impl<SPI, CSPIN> SpiInterface<SPI, CSPIN>
    where
        SPI: Transfer<u8>,
        CSPIN: OutputPin,
    {
        pub fn new(spi: SPI, cs: CSPIN) -> Self {
            Self { spi, cs }
        }

        pub fn into_inner(self) -> (SPI, CSPIN) {
            (self.spi, self.cs)
        }
    }

    impl<SPI, CSPIN> super::Interface for SpiInterface<SPI, CSPIN>
    where
        SPI: Transfer<u8>,
        CSPIN: OutputPin,
    {
        type Error = SPI::Error;

        fn read_multiple_regs(&mut self, addr: u8, data: &mut [u8]) -> Result<(), Self::Error> {
            let _cs = ChipSelectGuard::new(&mut self.cs);
            let cmd = (addr & ADDR_MASK) | READ_FLAG;
            let _ = self.spi.transfer(&mut [cmd])?;
            let _ = self.spi.transfer(data)?;
            Ok(())
        }

        fn read_reg(&mut self, addr: u8) -> Result<u8, Self::Error> {
            let _cs = ChipSelectGuard::new(&mut self.cs);
            let mut words = [(addr & ADDR_MASK) | READ_FLAG, 0];
            let reads = self.spi.transfer(&mut words)?;
            Ok(reads[1])
        }

        fn write_reg(&mut self, addr: u8, value: u8) -> Result<(), Self::Error> {
            let _cs = ChipSelectGuard::new(&mut self.cs);
            let mut words = [addr & ADDR_MASK, value];
            let _ = self.spi.transfer(&mut words)?;
            Ok(())
        }
    }
}

pub mod i2c {}
