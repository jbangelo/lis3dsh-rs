pub trait Interface {
    type Error;

    fn read_multiple_regs(&mut self, addr: u8, data: &mut [u8]) -> Result<(), Self::Error>;

    fn read_reg(&mut self, addr: u8) -> Result<u8, Self::Error>;

    fn write_reg(&mut self, addr: u8, value: u8) -> Result<(), Self::Error>;
}

pub use spi::SpiInterface as Spi;

mod spi {
    use embedded_hal as hal;
    use hal::{blocking::spi::Transfer, digital::v2::OutputPin};

    const READ_FLAG: u8 = 0x80;
    const ADDR_MASK: u8 = 0x7F;

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
