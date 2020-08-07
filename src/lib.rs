//! A driver for the LIS3DSH accelerometer
//!
//! This crate provides implementations using either the SPI and I2C interface,
//! with a simplified interface abstrcting away the serial port being used.
//! The `embedded_hal` crate is used to define the interface to both serial
//! buses.

#![no_std]

pub mod interface;

use embedded_hal as hal;

pub use accelerometer;
use accelerometer::error::Error as AccelerometerError;
use accelerometer::vector::{F32x3, I16x3};
use accelerometer::{Accelerometer, RawAccelerometer};
use core::convert::{TryFrom, TryInto};
use core::fmt::Debug;
use num_enum::TryFromPrimitive;

/// Accelerometer errors, generic around another error type `E` representing
/// an (optional) cause of this error.
#[derive(Debug)]
pub enum Error<E> {
    /// bus error
    Bus(E),

    /// Invalid data rate selection
    InvalidDataRate,

    /// Invalid full-scale selection
    InvalidRange,

    /// Invalid address provided
    WrongAddress,
}

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

const ODR_MASK: u8 = 0xF0;

const CTRL_REG3_ADDR: u8 = 0x23;

const CTRL_REG5_ADDR: u8 = 0x24;

const FS_MASK: u8 = 0x38;

const STATUS_ADDR: u8 = 0x27;
const OUT_X_LOW_ADDR: u8 = 0x28;
const OUT_X_ADDR: u8 = OUT_X_LOW_ADDR;

/// The value present in the `WHO_AM_I` register on the LIS3DSH.
///
/// If another value is read, then a different IC is present
pub const EXPECTED_WHO_AM_I: u8 = 0x3f;

/// Output data rate.
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum DataRate {
    /// 1600Hz
    Hz_1600 = 0b1001,

    /// 800Hz
    Hz_800 = 0b1000,

    /// 400Hz
    Hz_400 = 0b0111,

    /// 100Hz
    Hz_100 = 0b0110,

    /// 50Hz
    Hz_50 = 0b0101,

    /// 25Hz
    Hz_25 = 0b0100,

    /// 12.5Hz
    Hz_12P5 = 0b0011,

    /// 6.25Hz
    Hz_6P25 = 0b0010,

    /// 3.125Hz
    Hz_3P125 = 0b0001,

    /// Power down
    PowerDown = 0b0000,
}

impl DataRate {
    pub fn bits(self) -> u8 {
        self as u8
    }

    pub fn sample_rate(self) -> f32 {
        match self {
            DataRate::Hz_1600 => 1600.0,
            DataRate::Hz_800 => 800.0,
            DataRate::Hz_400 => 400.0,
            DataRate::Hz_100 => 100.0,
            DataRate::Hz_50 => 50.0,
            DataRate::Hz_25 => 25.0,
            DataRate::Hz_12P5 => 12.5,
            DataRate::Hz_6P25 => 6.25,
            DataRate::Hz_3P125 => 3.125,
            DataRate::PowerDown => 0.0,
        }
    }
}

/// Full-scale selection.
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum Range {
    /// ±16g
    G16 = 0b100,

    /// ±8g
    G8 = 0b011,

    /// ±6g
    G6 = 0b010,

    /// ±4g
    G4 = 0b001,

    /// ±2g
    G2 = 0b000,
}

impl Range {
    pub fn bits(self) -> u8 {
        self as u8
    }
}

/// Driver for the LIS3DSH accelerometer.
pub struct Lis3dsh<IFACE>
where
    IFACE: interface::Interface,
{
    pub iface: IFACE,
}

impl<IFACE, E> Lis3dsh<IFACE>
where
    E: Debug,
    IFACE: interface::Interface<Error = E>,
{
    /// Initializes the accelerometer with these steps:
    ///     1. Read Device ID
    ///     2. Soft reset
    ///     3. Block for 5ms
    ///     4. Enable X, Y, & Z channels at 100Hz, 2G, BDU
    ///     5. Enable the DRDY signal on int1, enable int1, interrupt signal pulsed, interrupt active high
    pub fn init<DELAY>(&mut self, delay: &mut DELAY) -> Result<(), Error<E>>
    where
        DELAY: hal::blocking::delay::DelayMs<u8>,
    {
        if self.who_am_i()? != EXPECTED_WHO_AM_I {
            return Err(Error::WrongAddress);
        }

        // Reset it
        self.iface
            .write_reg(CTRL_REG3_ADDR, 0x01)
            .map_err(Error::Bus)?;
        delay.delay_ms(5);

        // Enable the XYZ channels and BDU
        self.iface
            .write_reg(CTRL_REG4_ADDR, 0x6F)
            .map_err(Error::Bus)?;

        self.iface
            .write_reg(CTRL_REG3_ADDR, 0xE8)
            .map_err(Error::Bus)
    }

    /// Reads the accelerometer data
    ///
    /// The returns array contains `[x, y, z]` in that order
    pub fn read_data(&mut self) -> Result<[i16; 3], Error<E>> {
        let mut data = [0; 6];
        self.iface
            .read_multiple_regs(OUT_X_ADDR, &mut data)
            .map_err(Error::Bus)?;

        let x_value = i16::from_le_bytes(data[0..2].try_into().unwrap());
        let y_value = i16::from_le_bytes(data[2..4].try_into().unwrap());
        let z_value = i16::from_le_bytes(data[4..6].try_into().unwrap());
        Ok([x_value, y_value, z_value])
    }

    /// Reads the tempurature data
    pub fn read_temp_data(&mut self) -> Result<i8, Error<E>> {
        let temp_data = self.iface.read_reg(OUT_T_ADDR).map_err(Error::Bus)?;

        Ok(temp_data as i8)
    }

    /// Checks the `STATUS` register to see if data is ready
    pub fn is_data_ready(&mut self) -> Result<bool, Error<E>> {
        Ok(self.status()?.zyxda())
    }

    /// Gets the value in the `WHO_AM_I` register. Should match
    /// [EXPECTED_WHO_AM_I](constant.EXPECTED_WHO_AM_I.html)
    pub fn who_am_i(&mut self) -> Result<u8, Error<E>> {
        self.iface.read_reg(WHO_AM_I_ADDR).map_err(Error::Bus)
    }

    /// Gets the value in  the `STATUS` register
    pub fn status(&mut self) -> Result<Status, Error<E>> {
        let status = self.iface.read_reg(STATUS_ADDR).map_err(Error::Bus)?;

        Ok(Status(status))
    }

    /// Full-scale selection.
    pub fn set_range(&mut self, range: Range) -> Result<(), Error<E>> {
        let mut ctrl5 = self.iface.read_reg(CTRL_REG5_ADDR).map_err(Error::Bus)?;

        ctrl5 &= !FS_MASK;

        ctrl5 |= range.bits() << 3;

        self.iface
            .write_reg(CTRL_REG5_ADDR, ctrl5)
            .map_err(Error::Bus)
    }

    /// Read Full-scale.
    pub fn get_range(&mut self) -> Result<Range, Error<E>> {
        let ctrl5 = self.iface.read_reg(CTRL_REG5_ADDR).map_err(Error::Bus)?;

        let fs = (ctrl5 >> 3) & 0x07;

        Range::try_from(fs).map_err(|_| Error::InvalidRange)
    }

    /// Data rate selection
    pub fn set_datarate(&mut self, datarate: DataRate) -> Result<(), Error<E>> {
        let mut ctrl4 = self.iface.read_reg(CTRL_REG4_ADDR).map_err(Error::Bus)?;

        ctrl4 &= !ODR_MASK;

        ctrl4 |= datarate.bits() << 4;

        self.iface
            .write_reg(CTRL_REG4_ADDR, ctrl4)
            .map_err(Error::Bus)
    }

    /// Read the current data selection rate.
    pub fn get_datarate(&mut self) -> Result<DataRate, Error<E>> {
        let ctrl4 = self.iface.read_reg(CTRL_REG4_ADDR).map_err(Error::Bus)?;
        let odr = (ctrl4 >> 4) & 0x0F;

        DataRate::try_from(odr).map_err(|_| Error::InvalidDataRate)
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

impl<IFACE, E> Accelerometer for Lis3dsh<IFACE>
where
    IFACE: interface::Interface<Error = E>,
    E: Debug,
{
    type Error = Error<E>;

    /// Get normalized ±g reading from the accelerometer. You should be reading
    /// based on data ready interrupt or if reading in a tight loop you should
    /// waiting for `is_data_ready`.
    fn accel_norm(&mut self) -> Result<F32x3, AccelerometerError<Self::Error>> {
        let range = self.get_range()?;

        let scale = match range {
            Range::G2 => 0.0006,
            Range::G4 => 0.0012,
            Range::G6 => 0.0018,
            Range::G8 => 0.0024,
            Range::G16 => 0.0073,
        };

        let acc_raw = self.accel_raw()?;
        let x = acc_raw.x as f32 * scale;
        let y = acc_raw.y as f32 * scale;
        let z = acc_raw.z as f32 * scale;

        Ok(F32x3::new(x, y, z))
    }

    /// Get the sample rate of the accelerometer data.
    fn sample_rate(&mut self) -> Result<f32, AccelerometerError<Self::Error>> {
        Ok(self.get_datarate()?.sample_rate())
    }
}

impl<IFACE, E> RawAccelerometer<I16x3> for Lis3dsh<IFACE>
where
    IFACE: interface::Interface<Error = E>,
    E: Debug,
{
    type Error = Error<E>;

    /// Get raw acceleration data from the accelerometer. You should be reading
    /// based on data ready interrupt or if reading in a tight loop you should
    /// waiting for `is_data_ready`.
    fn accel_raw(&mut self) -> Result<I16x3, AccelerometerError<Self::Error>> {
        let accel_bytes = self.read_data()?;

        let x = accel_bytes[0];
        let y = accel_bytes[1];
        let z = accel_bytes[2];

        Ok(I16x3::new(x, y, z))
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
