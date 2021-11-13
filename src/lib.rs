//
// lib.rs
//
// @author Natesh Narain <nnaraindev@gmail.com>
// @date Nov 12 2021
//
#![no_std]
#![allow(unused)]
mod regs;

use regs::*;

pub use regs::{
    AccelerometerOutput, AccelerometerScale, AccelerometerBandwidth,
    GyroscopeOutput, GyroscopeFullScale, GyroscopeFullScale125Dps,
};

use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

const CHIP_ID: u8 = 0x69;

#[derive(Debug)]
pub enum Error<E> {
    CommunicationError(E),
    ChipDetectFailed,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::CommunicationError(e)
    }
}

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

    pub fn set_accelerometer_output(&mut self, output: AccelerometerOutput) -> Result<(), Error<E>> {
        self.write_register_option(Register::Ctrl1XL, output)
    }

    pub fn set_accelerometer_scale(&mut self, scale: AccelerometerScale) -> Result<(), Error<E>> {
        self.write_register_option(Register::Ctrl1XL, scale)
    }

    pub fn set_accelerometer_bandwidth(&mut self, bandwidth: AccelerometerBandwidth) -> Result<(), Error<E>> {
        self.write_register_option(Register::Ctrl1XL, bandwidth)
    }

    pub fn set_gyroscope_output(&mut self, output: GyroscopeOutput) -> Result<(), Error<E>> {
        self.write_register_option(Register::Ctrl2G, output)
    }

    pub fn read_gyro(&mut self) -> Result<(i16, i16, i16), Error<E>> {
        self.read_sensor(Register::OutXLG)
    }

    pub fn read_accel(&mut self) -> Result<(i16, i16, i16), Error<E>> {
        self.read_sensor(Register::OutXLXL)
    }

    pub fn accel_data_available(&mut self) -> Result<bool, Error<E>> {
        self.read_status().map(|status| status & 0b1 != 0)
    }

    pub fn gyro_data_available(&mut self) -> Result<bool, Error<E>> {
        self.read_status().map(|status| status & 0b10 != 0)
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
        self.write_bits(register, ro.value(), ro.mask(), ro.bit_offset())
    }

    fn write_bit(&mut self, register: Register, value: u8, shift: u8) -> Result<(), Error<E>> {
        self.write_bits(register, value, 0x01, shift)
    }

    fn write_bits(&mut self, register: Register, new_value: u8, mask: u8, shift: u8) -> Result<(), Error<E>> {
        let current_value = self.read_register(register)?;
        let modified_value = (current_value & !(mask << shift)) | ((new_value & mask) << shift);
        self.write_register(register, modified_value)
    }

    pub fn read_sensor(&mut self, start_reg: Register) -> Result<(i16, i16, i16), Error<E>> {
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
