//
// lib.rs
//
// @author Natesh Narain <nnaraindev@gmail.com>
// @date Nov 12 2021
//
#![no_std]
#![allow(unused)]
mod regs;

use arrayref::array_refs;
use core::convert::TryFrom;

use regs::*;

pub use regs::{
    AccelerometerBandwidth, AccelerometerOutput, AccelerometerScale, GyroscopeFullScale,
    GyroscopeOutput,
};

use maybe_async_cfg;

#[cfg(feature = "blocking")]
use embedded_hal::blocking::i2c::{Write, WriteRead};
#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c;

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
#[maybe_async_cfg::maybe(sync(feature = "blocking", keep_self), async(feature = "async"))]
pub struct Lsm6ds33<I2C> {
    i2c: I2C,
    addr: u8,
    accelerometer_scale: Option<AccelerometerScale>,
    gyroscope_scale: Option<GyroscopeFullScale>,
}

#[maybe_async_cfg::maybe(
    sync(feature = "blocking", keep_self),
    async(
        feature = "async",
        idents(Write(async = "I2c"), WriteRead(async = "I2c"))
    )
)]
impl<I2C, E> Lsm6ds33<I2C>
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
{
    /// Create an instance of the Lsm6ds33 driver
    /// If the device cannot be detected on the bus, an error will be returned
    pub async fn new(i2c: I2C, addr: u8) -> Result<Self, (I2C, Error<E>)> {
        let mut lsm = Lsm6ds33 {
            i2c,
            addr,
            accelerometer_scale: None,
            gyroscope_scale: None,
        };

        match lsm.check().await {
            Ok(true) => match lsm.set_auto_increment(true).await {
                Ok(()) => Ok(lsm),
                Err(e) => Err((lsm.release(), e)),
            },
            Ok(false) => Err((lsm.release(), Error::ChipDetectFailed)),
            Err(e) => Err((lsm.release(), e)),
        }
    }

    /// Set the accelerometer output rate
    pub async fn set_accelerometer_output(
        &mut self,
        output: AccelerometerOutput,
    ) -> Result<(), Error<E>> {
        self.write_register_option(Register::Ctrl1XL, output).await
    }

    /// Set the accelerometer operating range
    pub async fn set_accelerometer_scale(
        &mut self,
        scale: AccelerometerScale,
    ) -> Result<(), Error<E>> {
        match self.write_register_option(Register::Ctrl1XL, scale).await {
            Ok(()) => {
                self.accelerometer_scale = Some(scale);
                Ok(())
            }
            Err(e) => {
                self.accelerometer_scale = None;
                Err(e)
            }
        }
    }

    /// Set the accelerometer bandwidth filter
    pub async fn set_accelerometer_bandwidth(
        &mut self,
        bandwidth: AccelerometerBandwidth,
    ) -> Result<(), Error<E>> {
        self.write_register_option(Register::Ctrl1XL, bandwidth)
            .await
    }

    /// Set the gyroscope output rate
    pub async fn set_gyroscope_output(&mut self, output: GyroscopeOutput) -> Result<(), Error<E>> {
        self.write_register_option(Register::Ctrl2G, output).await
    }

    /// Set the gyroscope operating range
    pub async fn set_gyroscope_scale(&mut self, scale: GyroscopeFullScale) -> Result<(), Error<E>> {
        match self.write_register_option(Register::Ctrl2G, scale).await {
            Ok(()) => {
                self.gyroscope_scale = Some(scale);
                Ok(())
            }
            Err(e) => {
                self.accelerometer_scale = None;
                Err(e)
            }
        }
    }

    /// Set the low power mode
    pub async fn set_low_power_mode(&mut self, low_power: bool) -> Result<(), Error<E>> {
        // N.B. "1" means low-power, "0" means high-performance.
        self.write_bit(
            Register::Ctrl6C,
            low_power as u8,
            Ctrl6C::AccelHighPerformanceMode as u8,
        )
        .await?;
        self.write_bit(
            Register::Ctrl7G,
            low_power as u8,
            Ctrl7G::HighPerformanceMode as u8,
        )
        .await
    }

    /// Read all three sensors in one transaction. Returns temperature, gyro, accelerometer.
    pub async fn read_all(&mut self) -> Result<(f32, (f32, f32, f32), (f32, f32, f32)), Error<E>> {
        let gyro_scale = self.read_gyroscope_scale().await?;
        let accel_scale = self.read_accelerometer_scale().await?;
        let data = self.read_registers::<14>(Register::OutTempL).await?;
        let (temp, gyro, accel) = array_refs!(&data, 2, 6, 6);
        Ok((
            Self::convert_temp_data(temp),
            Self::convert_gyro_data(gyro, gyro_scale),
            Self::convert_accel_data(accel, accel_scale),
        ))
    }

    /// Read the gyroscope data for each axis (RAD/s)
    pub async fn read_gyro(&mut self) -> Result<(f32, f32, f32), Error<E>> {
        let scale = self.read_gyroscope_scale().await?;
        self.read_registers(Register::OutXLG)
            .await
            .map(|res| Self::convert_gyro_data(&res, scale))
    }

    fn convert_gyro_data(data: &[u8; 6], scale: GyroscopeFullScale) -> (f32, f32, f32) {
        // Convert raw data to float
        let (x, y, z) = Self::u8_to_f32(data);
        let scale = scale.scale();
        // Convert to RAD/s (Raw gyro data is in milli-degrees per second per bit)
        (
            (x * scale / 1000.0).to_radians(),
            (y * scale / 1000.0).to_radians(),
            (z * scale / 1000.0).to_radians(),
        )
    }

    /// Read the accelerometer data for each axis (m/s^2)
    pub async fn read_accelerometer(&mut self) -> Result<(f32, f32, f32), Error<E>> {
        let scale = self.read_accelerometer_scale().await?;
        self.read_registers(Register::OutXLXL)
            .await
            .map(|res| Self::convert_accel_data(&res, scale))
    }

    fn convert_accel_data(data: &[u8; 6], scale: AccelerometerScale) -> (f32, f32, f32) {
        // Convert raw values to float
        let (x, y, z) = Self::u8_to_f32(data);
        let scale = scale.scale();

        // Convert to m/s^2 (Raw value is in mg/bit)
        (
            (x * scale / 1000.0) * EARTH_GRAVITY,
            (y * scale / 1000.0) * EARTH_GRAVITY,
            (z * scale / 1000.0) * EARTH_GRAVITY,
        )
    }

    /// Read the temperature (degC)
    pub async fn read_temperature(&mut self) -> Result<f32, Error<E>> {
        let data = self.read_registers::<2>(Register::OutTempL).await?;
        Ok(Self::convert_temp_data(&data))
    }

    fn convert_temp_data(data: &[u8; 2]) -> f32 {
        let (lo, hi) = (data[0], data[1]);

        // Raw temperature as signal 16-bit number
        let temperature = ((hi as i16) << 8) | (lo as i16);
        // As float
        let temperature = temperature as f32;
        // Converted given the temperature sensitively value 16 bits per C
        let temperature = (temperature / 16.0) + 25.0;

        temperature
    }

    /// Check if there is new accelerometer data
    pub async fn accel_data_available(&mut self) -> Result<bool, Error<E>> {
        self.read_status().await.map(|status| status & 0b1 != 0)
    }

    /// Check if there is new gyro scope data
    pub async fn gyro_data_available(&mut self) -> Result<bool, Error<E>> {
        self.read_status().await.map(|status| status & 0b10 != 0)
    }

    /// Read the accelerometer scale value from the configuration register
    pub async fn read_accelerometer_scale(&mut self) -> Result<AccelerometerScale, Error<E>> {
        match self.accelerometer_scale {
            Some(v) => Ok(v),
            None => {
                let scale = self.read_register_option(Register::Ctrl1XL).await?;
                self.accelerometer_scale = Some(scale);
                Ok(scale)
            }
        }
    }

    /// Read the gyroscope scale value from the configuration register
    pub async fn read_gyroscope_scale(&mut self) -> Result<GyroscopeFullScale, Error<E>> {
        match self.gyroscope_scale {
            Some(v) => Ok(v),
            None => {
                let scale = self.read_register_option(Register::Ctrl2G).await?;
                self.gyroscope_scale = Some(scale);
                Ok(scale)
            }
        }
    }

    async fn check(&mut self) -> Result<bool, Error<E>> {
        self.read_register(Register::WhoAmI)
            .await
            .map(|chip_id| chip_id == CHIP_ID)
    }

    async fn set_auto_increment(&mut self, enabled: bool) -> Result<(), Error<E>> {
        self.write_bit(Register::Ctrl3C, enabled as u8, Ctrl3C::AutoIncrement as u8)
            .await
    }

    async fn read_status(&mut self) -> Result<u8, Error<E>> {
        self.read_register(Register::StatusReg).await
    }

    async fn write_register_option<RO: RegisterOption>(
        &mut self,
        register: Register,
        ro: RO,
    ) -> Result<(), Error<E>> {
        self.write_bits(register, ro.value(), RO::mask(), RO::bit_offset())
            .await
    }

    async fn read_register_option<RO: RegisterOption + TryFrom<u8>>(
        &mut self,
        register: Register,
    ) -> Result<RO, Error<E>> {
        let value = self.read_register(register).await?;
        RO::try_from(value).map_err(|_| Error::RegisterReadFailed)
    }

    async fn write_bit(
        &mut self,
        register: Register,
        value: u8,
        shift: u8,
    ) -> Result<(), Error<E>> {
        self.write_bits(register, value, 0x01, shift).await
    }

    async fn write_bits(
        &mut self,
        register: Register,
        new_value: u8,
        mask: u8,
        shift: u8,
    ) -> Result<(), Error<E>> {
        let current_value = self.read_register(register).await?;
        let modified_value = (current_value & !(mask << shift)) | ((new_value & mask) << shift);
        self.write_register(register, modified_value).await
    }

    fn u8_to_f32(res: &[u8; 6]) -> (f32, f32, f32) {
        let (x, y, z) = (
            (res[0] as i16) | ((res[1] as i16) << 8),
            (res[2] as i16) | ((res[3] as i16) << 8),
            (res[4] as i16) | ((res[5] as i16) << 8),
        );
        (x as f32, y as f32, z as f32)
    }

    // Read a byte from the given register
    async fn read_register(&mut self, register: Register) -> Result<u8, Error<E>> {
        let mut res = [0u8];
        self.i2c
            .write_read(self.addr, &[register.into()], &mut res)
            .await?;
        Ok(res[0])
    }

    async fn read_registers<const N: usize>(
        &mut self,
        start_reg: Register,
    ) -> Result<[u8; N], Error<E>> {
        let mut res = [0u8; N];
        self.i2c
            .write_read(self.addr, &[start_reg.into()], &mut res)
            .await?;
        Ok(res)
    }

    // Write the specified value to the given register
    async fn write_register(&mut self, register: Register, value: u8) -> Result<(), Error<E>> {
        self.i2c
            .write(self.addr, &[register.into(), value])
            .await
            .map_err(Error::from)
    }

    /// Return the underlying I2C device
    pub fn release(self) -> I2C {
        self.i2c
    }
}
