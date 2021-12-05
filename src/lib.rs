//
// lib.rs
//
// @author Natesh Narain <nnaraindev@gmail.com>
// @date Nov 12 2021
//
#![no_std]
#![allow(unused)]
mod regs;

use core::convert::TryFrom;

use regs::*;

pub use regs::{
    AccelerometerBandwidth, AccelerometerOutput, AccelerometerScale,
    GyroscopeFullScale, GyroscopeOutput,
};

use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

/// Enum containing all possible types of errors when interacting with the IMU
#[derive(Debug)]
pub enum Error<E> {
    CommunicationError(E),
    ChipDetectFailed,
    RegisterReadFailed,
}

// Allow converting the Error type from the I2C device to the error enum
impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::CommunicationError(e)
    }
}

// Value of the WHO_AM_I register
const CHIP_ID: u8 = 0x69;

// Earth gravity constant for acceleration conversion
const EARTH_GRAVITY: f32 = 9.80665;

/// 6-DoF IMU accelerometer + gyro
pub struct Lsm6ds33<I2C> {
    i2c: I2C,
    addr: u8,
}

impl<I2C, E> Lsm6ds33<I2C>
    where I2C: Write<Error = E> + Read<Error = E> + WriteRead<Error = E> {

    /// Create an instance of the Lsm6ds33 driver
    /// If the device cannot be detected on the bus, an error will be returned
    pub fn new(i2c: I2C, addr: u8) -> Result<Self, Error<E>> {
        let mut lsm = Lsm6ds33 {i2c, addr};

        let chip_detected = lsm.check()?;
        if chip_detected {
            lsm.set_auto_increment(true)?;
            Ok(lsm)
        }
        else {
            Err(Error::ChipDetectFailed)
        }
    }

    /// Set the accelerometer output rate
    pub fn set_accelerometer_output(&mut self, output: AccelerometerOutput) -> Result<(), Error<E>> {
        self.write_register_option(Register::Ctrl1XL, output)
    }

    /// Set the accelerometer operating range
    pub fn set_accelerometer_scale(&mut self, scale: AccelerometerScale) -> Result<(), Error<E>> {
        self.write_register_option(Register::Ctrl1XL, scale)
    }

    /// Set the accelerometer bandwidth filter
    pub fn set_accelerometer_bandwidth(&mut self, bandwidth: AccelerometerBandwidth) -> Result<(), Error<E>> {
        self.write_register_option(Register::Ctrl1XL, bandwidth)
    }

    /// Set the gyroscope output rate
    pub fn set_gyroscope_output(&mut self, output: GyroscopeOutput) -> Result<(), Error<E>> {
        self.write_register_option(Register::Ctrl2G, output)
    }

    /// Set the gyroscope operating range
    pub fn set_gyroscope_scale(&mut self, scale: GyroscopeFullScale) -> Result<(), Error<E>> {
        self.write_register_option(Register::Ctrl2G, scale)
    }

    /// Read the gyroscope data for each axis (RAD/s)
    pub fn read_gyro(&mut self) -> Result<(f32, f32, f32), Error<E>> {
        // Read the raw gyro data from the IMU
        let (x, y, z) = self.read_gyro_raw()?;
        // Get the set gyro full scale parameter
        let scale = self.read_gyroscope_scale()?.scale();
        // Convert raw data to float
        let (x, y, z) = (x as f32, y as f32, z as f32);
        // Convert to RAD/s (Raw gyro data is in milli-degrees per second per bit)
        Ok(
            (
                (x * scale / 1000.0).to_radians(),
                (y * scale / 1000.0).to_radians(),
                (z * scale / 1000.0).to_radians(),
            )
        )
    }

    /// Read the accelerometer data for each axis (m/s^2)
    pub fn read_accelerometer(&mut self) -> Result<(f32, f32, f32), Error<E>> {
        let (x, y, z) = self.read_accelerometer_raw()?;
        let scale = self.read_accelerometer_scale()?.scale();

        // Convert raw values to float
        let (x, y, z) = (x as f32, y as f32, z as f32);

        // Convert to m/s^2 (Raw value is in mg/bit)
        Ok(
            (
                (x * scale / 1000.0) * EARTH_GRAVITY,
                (y * scale / 1000.0) * EARTH_GRAVITY,
                (z * scale / 1000.0) * EARTH_GRAVITY,
            )
        )
    }

    /// Read the temperature
    pub fn read_temperature(&mut self) -> Result<f32, Error<E>> {
        let lo = self.read_register(Register::OutTempL)?;
        let hi = self.read_register(Register::OutTempH)?;

        // Raw temperature as signal 16-bit number
        let temperature = ((hi as i16) << 8) | (lo as i16);
        // As float
        let temperature = temperature as f32;
        // Converted given the temperature sensitively value 16 bits per C
        let temperature = (temperature / 16.0) + 25.0;

        Ok(temperature)
    }

    /// Check if there is new accelerometer data
    pub fn accel_data_available(&mut self) -> Result<bool, Error<E>> {
        self.read_status().map(|status| status & 0b1 != 0)
    }

    /// Check if there is new gyro scope data
    pub fn gyro_data_available(&mut self) -> Result<bool, Error<E>> {
        self.read_status().map(|status| status & 0b10 != 0)
    }

    /// Read the accelerometer scale value from the configuration register
    pub fn read_accelerometer_scale(&mut self) -> Result<AccelerometerScale, Error<E>> {
        self.read_register_option(Register::Ctrl1XL)
    }

    /// Read the gyroscope scale value from the configuration register
    pub fn read_gyroscope_scale(&mut self) -> Result<GyroscopeFullScale, Error<E>> {
        self.read_register_option(Register::Ctrl2G)
    }

    fn read_gyro_raw(&mut self) -> Result<(i16, i16, i16), Error<E>> {
        self.read_sensor(Register::OutXLG)
    }

    fn read_accelerometer_raw(&mut self) -> Result<(i16, i16, i16), Error<E>> {
        self.read_sensor(Register::OutXLXL)
    }

    fn check(&mut self) -> Result<bool, Error<E>> {
        self.read_register(Register::WhoAmI).map(|chip_id| chip_id == CHIP_ID)
    }

    fn set_auto_increment(&mut self, enabled: bool) -> Result<(), Error<E>> {
        self.write_bit(Register::Ctrl3C, enabled as u8, Ctrl3C::AutoIncrement as u8)
    }

    fn read_status(&mut self) -> Result<u8, Error<E>> {
        self.read_register(Register::StatusReg)
    }

    fn write_register_option<RO: RegisterOption>(&mut self, register: Register, ro: RO) -> Result<(), Error<E>> {
        self.write_bits(register, ro.value(), RO::mask(), RO::bit_offset())
    }

    fn read_register_option<RO: RegisterOption + TryFrom<u8>>(&mut self, register: Register) -> Result<RO, Error<E>> {
        let value = self.read_register(register)?;
        RO::try_from(value).map_err(|_| Error::RegisterReadFailed)
    }

    fn write_bit(&mut self, register: Register, value: u8, shift: u8) -> Result<(), Error<E>> {
        self.write_bits(register, value, 0x01, shift)
    }

    fn write_bits(&mut self, register: Register, new_value: u8, mask: u8, shift: u8) -> Result<(), Error<E>> {
        let current_value = self.read_register(register)?;
        let modified_value = (current_value & !(mask << shift)) | ((new_value & mask) << shift);
        self.write_register(register, modified_value)
    }

    fn read_sensor(&mut self, start_reg: Register) -> Result<(i16, i16, i16), Error<E>> {
        let mut res = [0u8; 6];
        self.i2c.write_read(self.addr, &[start_reg.into()], &mut res).map(|_| {
            (
                (res[0] as i16) | ((res[1] as i16) << 8),
                (res[2] as i16) | ((res[3] as i16) << 8),
                (res[4] as i16) | ((res[5] as i16) << 8)
            )
        })
        .map_err(Error::from)
    }

    // Read a byte from the given register
    fn read_register(&mut self, register: Register) -> Result<u8, Error<E>> {
        let mut res = [0u8];
        self.i2c.write_read(self.addr, &[register.into()], &mut res)?;
        Ok(res[0])
    }

    // Write the specified value to the given register
    fn write_register(&mut self, register: Register, value: u8) -> Result<(), Error<E>> {
        self.i2c.write(self.addr, &[register.into(), value]).map_err(Error::from)
    }

    /// Return the underlying I2C device
    pub fn release(self) -> I2C {
        self.i2c
    }
}
