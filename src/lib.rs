#![no_std]
#![feature(error_in_core)]
use crc::{Crc, CRC_8_MAXIM_DOW};
use embedded_hal::blocking::{delay, i2c};

/// 0x31 CRC (polynomial: 1 + x^4 + x^5 + x^8).
const CRC: Crc<u8> = Crc::<u8>::new(&CRC_8_MAXIM_DOW);

/// AHT20 standard *7-bits I2C address*.
pub const STANDARD_I2C_ADDRESS: i2c::SevenBitAddress = 0x38;

/// AHT20 sensor errors.
#[derive(Debug, Clone, Copy)]
pub enum Aht20Error<E> {
    /// I2C bus error.
    I2c(E),
    /// Checksum mismatch.
    Checksum,
}

impl<E> core::fmt::Display for Aht20Error<E>
where
    E: core::error::Error,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::I2c(error) => write!(f, "I2C bus error: {error}"),
            Self::Checksum => write!(f, "measurement checksum mismatch, CRC validation fail"),
        }
    }
}

impl<E: core::error::Error> core::error::Error for Aht20Error<E> {}

impl<E> From<E> for Aht20Error<E> {
    fn from(error: E) -> Self {
        Self::I2c(error)
    }
}

/// Sensor status.
/// Relevant status bits description:
/// - Bit 3:
///     * 1 -> calibrated
///     * 0 -> uncalibrated
/// - Bit 7:
///     * 1 -> Busy in measurement
///     * 0 -> Free in dormant state
#[derive(Default, Debug, Clone, Copy)]
struct Status {
    status_word: [u8; 1],
}

impl Status {
    /// Checks if sensor's [`Status`] is **busy**.
    fn is_busy(&self) -> bool {
        (self.status_word[0] & 0b1000_0000) == 0
    }

    /// Checks if sensor's [`Status`] is **calibrated**.
    fn is_calibrated(&self) -> bool {
        (self.status_word[0] & 0b0000_1000) == 0
    }
}

/// **AHT20** temperature and humidity data.
#[derive(Debug, Clone, Copy)]
pub struct Aht20Data {
    /// Humidity and temperature data (4 bytes: 20 humidity bits + 20 temperature bits).
    data: [u8; 4],
}

// TODO: what if the microcontroller does not have support for floating point arithmetic?
impl Aht20Data {
    /// Return *humidity* data as *% RH* (**R**elative **H**umidity).
    pub fn humidity(&self) -> f32 {
        let humidity = ((self.data[0] as u32) << 12)
            | ((self.data[1] as u32) << 4)
            | ((self.data[2] as u32) >> 4);

        (humidity as f32 / (1 << 20) as f32) * 100.0
    }

    /// Return *temperature* data as *°C*.
    pub fn temperature(&self) -> f32 {
        // TODO: let user chose unit of measurement?
        let temperature = ((self.data[2] as u32) & 0b0000_1111)
            | ((self.data[3] as u32) << 8)
            | self.data[4] as u32;

        (temperature as f32 / (1 << 20) as f32) * 200.0 - 50.0
    }
}

impl From<[u8; 7]> for Aht20Data {
    /// Construct [`Aht20Data`] from 7-bytes buffer returned from 0xAC (TriggerMeasurement)
    /// command.
    fn from(buf: [u8; 7]) -> Self {
        Self {
            // This conversion can't fail: safe to unwrap.
            data: buf[1..6].try_into().unwrap(),
        }
    }
}

/// **AHT20** temperature and humidity sensor.
#[derive(Debug)]
pub struct Aht20<I2C, D>
where
    D: delay::DelayMs<u16> + delay::DelayUs<u16>,
{
    /// I2C address of the sensor.
    address: i2c::SevenBitAddress,
    /// I2C interface.
    i2c: I2C,
    /// Delay implementor.
    delay: D,
}

impl<I2C, E, D> Aht20<I2C, D>
where
    I2C: i2c::Read<Error = E> + i2c::Write<Error = E> + i2c::WriteRead<Error = E>,
    D: delay::DelayMs<u16> + delay::DelayUs<u16>,
{
    /// Construct and initialize a new [`Aht20`] with the sensor's standard I2C *7-bit address*.
    pub fn new(i2c: I2C, delay: D) -> Result<Self, Aht20Error<E>> {
        Self::with_address(STANDARD_I2C_ADDRESS, i2c, delay)
    }

    /// Construct and initialize a new [`Aht20`] with given I2C *7-bit address*.
    pub fn with_address(
        address: i2c::SevenBitAddress,
        i2c: I2C,
        delay: D,
    ) -> Result<Self, Aht20Error<E>> {
        Self {
            address,
            i2c,
            delay,
        }
        .init()
    }

    /// Initializes a new [`Aht20`], consuming the [`Aht20Initializer`] (sensor initializer).
    fn init(mut self) -> Result<Self, Aht20Error<E>> {
        // Wait 40ms after power-on.
        self.delay.delay_ms(40);

        // Before reading temperature and humidity values, first check wheter the sensor is
        // Calibrated. If not, send `Initialization` command, then wait for 10 ms before
        // retrying.
        while !self.get_status()?.is_calibrated() {
            // Issue initialization command.
            self.i2c.write(self.address, &[0xBE, 0x08, 0x00])?;
            // Wait 10 ms.
            self.delay.delay_ms(10);
        }

        Ok(self)
    }

    /// Issue `CheckStatus` command to the sensor over *I2C* bus and return sensor's [`Status`].
    fn get_status(&mut self) -> Result<Status, Aht20Error<E>> {
        let mut status = Status::default();
        self.i2c
            .write_read(self.address, &[0x71], &mut status.status_word)?;

        Ok(status)
    }

    /// Return [`Aht20Data`] from measurement.
    ///
    /// # Note
    /// Calling this method takes at least 80ms.
    pub fn measure(&mut self) -> Result<Aht20Data, Aht20Error<E>> {
        // Issue `TriggerMeasurement` command to the sensor.
        self.i2c.write(self.address, &[0xAC, 0x33, 0x00])?;

        // Wait 80ms for the measurement to be completed.
        self.delay.delay_ms(80);

        // Sensor is busy, so keep waiting for intervals of 10 milliseconds.
        while self.get_status()?.is_busy() {
            self.delay.delay_ms(10);
        }

        // Collect response buffer: 1 status byte + 5 humidity & temperature bytes + 1 CRC byte
        let mut buf = [0u8; 7];
        self.i2c.read(self.address, &mut buf)?;

        // TODO: needs check
        let mut crc_digest = CRC.digest_with_initial(0xFF);
        crc_digest.update(&buf[..6]);
        if crc_digest.finalize() != buf[6] {
            return Err(Aht20Error::Checksum);
        }

        Ok(buf.into())
    }

    /// Perform a **soft reset** of the sensor, without turning the power off and on again.
    pub fn reset(&mut self) -> Result<(), Aht20Error<E>> {
        self.i2c.write(self.address, &[0xBA])?;

        // Wait 20ms for the reset to complete:
        // "the time required for soft reset does not exceed 20ms".
        self.delay.delay_ms(20);

        Ok(())
    }
}

#[cfg(test)]
mod tests {}
