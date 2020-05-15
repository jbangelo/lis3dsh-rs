//! A driver for the LIS3DSH accelerometer
//!
//! This crate provides implementations using either the SPI and I2C interface,
//! with a simplified interface abstrcting away the serial port being used.
//! The `embedded_hal` crate is used to define the interface to both serial
//! buses.

#![no_std]

pub mod interface;

extern crate embedded_hal;

use embedded_hal as hal;

/// Representation of the `STATUS` register on the LIS3DSH
pub struct Status(u8);

impl Status {
    const ZYXOR_MASK: u8 = 0x80;
    const ZOR_MASK: u8 = 0x40;
    const YOR_MASK: u8 = 0x20;
    const XOR_MASK: u8 = 0x10;
    const ZYXDA_MASK: u8 = 0x08;
    const ZDA_MASK: u8 = 0x04;
    const YDA_MASK: u8 = 0x02;
    const XDA_MASK: u8 = 0x01;

    /// Checks to see if the `ZYXOR` bit is set in the `STATUS` register
    ///
    /// When this bit is set it indicates that some of the data has been
    /// overwritten before it was able to be read
    pub fn zyxor(&self) -> bool {
        (self.0 & Self::ZYXOR_MASK) != 0
    }

    /// Checks to see if the `ZOR` bit is set in the `STATUS` register
    ///
    /// When this bit is set it indicates that the Z axis data has been
    /// overwritten before it was able to be read
    pub fn zor(&self) -> bool {
        (self.0 & Self::ZOR_MASK) != 0
    }

    /// Checks to see if the `YOR` bit is set in the `STATUS` register
    ///
    /// When this bit is set it indicates that the Y axis data has been
    /// overwritten before it was able to be read
    pub fn yor(&self) -> bool {
        (self.0 & Self::YOR_MASK) != 0
    }

    /// Checks to see if the `XOR` bit is set in the `STATUS` register
    ///
    /// When this bit is set it indicates that the X axis data has been
    /// overwritten before it was able to be read
    pub fn xor(&self) -> bool {
        (self.0 & Self::XOR_MASK) != 0
    }

    /// Checks to see if the `ZYXDA` bit is set in the `STATUS` register
    ///
    /// When this bit is set it indicates that new data is available
    pub fn zyxda(&self) -> bool {
        (self.0 & Self::ZYXDA_MASK) != 0
    }

    /// Checks to see if the `ZDA` bit is set in the `STATUS` register
    ///
    /// When this bit is set it indicates that new Z axis data is available
    pub fn zda(&self) -> bool {
        (self.0 & Self::ZDA_MASK) != 0
    }

    /// Checks to see if the `YDA` bit is set in the `STATUS` register
    ///
    /// When this bit is set it indicates that new Y axis data is available
    pub fn yda(&self) -> bool {
        (self.0 & Self::YDA_MASK) != 0
    }

    /// Checks to see if the `XDA` bit is set in the `STATUS` register
    ///
    /// When this bit is set it indicates that new X axis data is available
    pub fn xda(&self) -> bool {
        (self.0 & Self::XDA_MASK) != 0
    }
}

const OUT_T_ADDR: u8 = 0x0c;
const WHO_AM_I_ADDR: u8 = 0x0f;
const CTRL_REG4_ADDR: u8 = 0x20;
const CTRL_REG3_ADDR: u8 = 0x23;
const STATUS_ADDR: u8 = 0x27;
const OUT_X_LOW_ADDR: u8 = 0x28;
const OUT_X_ADDR: u8 = OUT_X_LOW_ADDR;

/// The value present in the `WHO_AM_I` register on the LIS3DSH.
///
/// If another value is read, then a different IC is present
pub const EXPECTED_WHO_AM_I: u8 = 0x3f;

/// Driver for the LIS3DSH accelerometer.
pub struct Lis3dsh<IFACE>
where
    IFACE: interface::Interface,
{
    iface: IFACE,
}

impl<IFACE> Lis3dsh<IFACE>
where
    IFACE: interface::Interface,
{
    /// Initializes the accelerometer with these steps:
    ///     1. Soft reset
    ///     2. Block for 5ms
    ///     3. Enable X, Y, & Z channels at 100Hz
    ///     4. Enable the DRDY signal
    pub fn init<DELAY>(&mut self, delay: &mut DELAY) -> Result<(), IFACE::Error>
    where
        DELAY: hal::blocking::delay::DelayMs<u8>,
    {
        // Reset it
        self.iface.write_reg(CTRL_REG3_ADDR, 0x01)?;
        delay.delay_ms(5);
        // Enable the XYZ channels
        self.iface.write_reg(CTRL_REG4_ADDR, 0x67)?;
        self.iface.write_reg(CTRL_REG3_ADDR, 0xE8)
    }

    /// Reads the accelerometer data
    ///
    /// The returns array contains `[x, y, z]` in that order
    pub fn read_data(&mut self) -> Result<[i16; 3], IFACE::Error> {
        let mut data = [0; 6];
        self.iface.read_multiple_regs(OUT_X_ADDR, &mut data)?;
        let x_value = (data[0] as i16) | ((data[1] as i16) << 8);
        let y_value = (data[2] as i16) | ((data[3] as i16) << 8);
        let z_value = (data[4] as i16) | ((data[5] as i16) << 8);
        Ok([x_value, y_value, z_value])
    }

    /// Reads the tempurature data
    pub fn read_temp_data(&mut self) -> Result<i8, IFACE::Error> {
        let temp_data = self.iface.read_reg(OUT_T_ADDR)?;
        Ok(temp_data as i8)
    }

    /// Checks the `STATUS` register to see if data is ready
    pub fn is_data_ready(&mut self) -> Result<bool, IFACE::Error> {
        Ok(self.status()?.zyxda())
    }

    /// Gets the value in the `WHO_AM_I` register. Should match
    /// [EXPECTED_WHO_AM_I](constant.EXPECTED_WHO_AM_I.html)
    pub fn who_am_i(&mut self) -> Result<u8, IFACE::Error> {
        self.iface.read_reg(WHO_AM_I_ADDR)
    }

    /// Gets the value in  the `STATUS` register
    pub fn status(&mut self) -> Result<Status, IFACE::Error> {
        let status = self.iface.read_reg(STATUS_ADDR)?;
        Ok(Status(status))
    }
}

impl<SPI, CSPIN> Lis3dsh<interface::Spi<SPI, CSPIN>>
where
    SPI: hal::blocking::spi::Transfer<u8>,
    CSPIN: hal::digital::v2::OutputPin,
{
    /// Creates a `Lis3dsh` using the SPI interface
    ///
    /// The 4-wire variant is used, and a digital I/O pin
    /// is used as the chip select
    pub fn new_spi(spi: SPI, mut cs: CSPIN) -> Self {
        // Make sure the chip select is in the inactive state
        let _ = cs.set_high();
        let iface = interface::Spi::new(spi, cs);
        Self { iface }
    }

    /// Converts the `Lis3dsh` into the constituant SPI interface and
    /// chip select pin
    pub fn into_inner(self) -> (SPI, CSPIN) {
        self.iface.into_inner()
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
