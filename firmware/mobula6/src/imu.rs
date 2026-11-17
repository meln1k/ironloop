//! IMU on SPI1: the Mobula6 ECO ships with either a Bosch BMI270 or a TDK
//! ICM-42688-P depending on the batch, so the driver probes for both.
//!
//! Samples come out in the sensor's own axes converted to the firmware's
//! FRD convention for a chip mounted with x forward and z up (x, -y, -z);
//! any further mounting rotation is [`Imu::alignment`], which the board
//! puts in the firmware's `Config` along with the ranges below. Both the axis
//! convention and the alignment need a bench check on the real quad, on a
//! BMI270 batch and on an ICM-42688 batch (tilt it and watch the attitude).

use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi::{self, Spi, mode::Master};
use embassy_stm32::time::Hertz;
use embassy_time::Timer;
use ironloop_fc::ImuAlignment;

use crate::bmi270_config::BMI270_CONFIG;

const GRAVITY: f32 = 9.80665;

/// Full-scale ranges. These are written into both chips' range registers
/// (below) and given to the firmware's `Config`, so its saturation and
/// spike checks always use the range the sensor really has.
const GYRO_RANGE_DPS: u16 = 2000;
const ACCEL_RANGE_G: u8 = 16;
/// Gyro range in rad/s.
pub const GYRO_RANGE: f32 = GYRO_RANGE_DPS as f32 * core::f32::consts::PI / 180.0;
/// Accelerometer range in m/s².
pub const ACCEL_RANGE: f32 = ACCEL_RANGE_G as f32 * GRAVITY;
/// Both chips give 16-bit samples spanning ±range: 16.384 LSB per deg/s
/// at ±2000 deg/s, which the datasheets round to 16.4 (Betaflight also uses
/// range / 2^15).
const GYRO_SCALE: f32 = GYRO_RANGE / 32768.0;
const ACCEL_SCALE: f32 = ACCEL_RANGE / 32768.0;

/// Range register fields for `GYRO_RANGE_DPS` and `ACCEL_RANGE_G`; another
/// range is a compile error rather than a silent mismatch.
const BMI_GYR_RANGE: u8 = match GYRO_RANGE_DPS {
    2000 => 0,
    1000 => 1,
    500 => 2,
    250 => 3,
    _ => panic!("unsupported gyro range"),
};
const BMI_ACC_RANGE: u8 = match ACCEL_RANGE_G {
    2 => 0,
    4 => 1,
    8 => 2,
    16 => 3,
    _ => panic!("unsupported accel range"),
};
/// ICM-42688 GYRO_FS_SEL / ACCEL_FS_SEL (bits 7:5), with ODR 2 kHz (0x5).
const ICM_GYRO_CONFIG0: u8 = match GYRO_RANGE_DPS {
    2000 => 0,
    1000 => 1,
    500 => 2,
    250 => 3,
    _ => panic!("unsupported gyro range"),
} << 5 | 0x05;
const ICM_ACCEL_CONFIG0: u8 = match ACCEL_RANGE_G {
    16 => 0,
    8 => 1,
    4 => 2,
    2 => 3,
    _ => panic!("unsupported accel range"),
} << 5 | 0x05;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Chip {
    Bmi270,
    Icm42688,
}

/// No usable IMU: an SPI error, an unknown chip ID, or a BMI270 that
/// rejected its configuration. The firmware reports it as the IMU arming
/// blocker, since no samples arrive.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Error;

impl From<spi::Error> for Error {
    fn from(_: spi::Error) -> Self {
        Error
    }
}

pub struct Imu {
    spi: Spi<'static, Async, Master>,
    cs: Output<'static>,
    pub chip: Chip,
}

impl Imu {
    pub async fn init(spi: Spi<'static, Async, Master>, cs: Output<'static>) -> Result<Self, Error> {
        let mut imu = Imu { spi, cs, chip: Chip::Bmi270 };
        Timer::after_millis(50).await; // power-up time for either chip

        // The BMI270 starts in I2C mode; any SPI access (a rising CS edge)
        // switches it to SPI, so the first read is a throwaway.
        let mut id = [0u8; 1];
        let _ = imu.read(0x00, &mut id).await;
        imu.read(0x00, &mut id).await?;
        if id[0] == 0x24 {
            imu.init_bmi270().await?;
            return Ok(imu);
        }
        imu.chip = Chip::Icm42688;
        imu.read(0x75, &mut id).await?;
        if id[0] == 0x47 {
            imu.init_icm42688().await?;
            return Ok(imu);
        }
        Err(Error)
    }

    /// How this chip sits on the board, as the firmware's `imu_alignment`.
    /// Happymodel's factory Betaflight dumps for this quad set
    /// `gyro_1_sensor_align = CW0` for both chips (the CrazyBee F4DX target
    /// default is CW90). Unverified on hardware per chip; split this arm
    /// if the bench shows the two footprints differ.
    pub fn alignment(&self) -> ImuAlignment {
        match self.chip {
            Chip::Bmi270 | Chip::Icm42688 => ImuAlignment::Cw0,
        }
    }

    fn set_speed(&mut self, hz: u32) {
        let mut cfg = spi::Config::default();
        cfg.mode = spi::MODE_3;
        cfg.frequency = Hertz(hz);
        let _ = self.spi.set_config(&cfg);
    }

    async fn read(&mut self, reg: u8, out: &mut [u8]) -> Result<(), Error> {
        // BMI270 SPI reads return one dummy byte before the data.
        let skip = 1 + (self.chip == Chip::Bmi270) as usize;
        let mut buf = [0u8; 16];
        let n = skip + out.len();
        buf[0] = reg | 0x80;
        self.cs.set_low();
        let r = self.spi.transfer_in_place(&mut buf[..n]).await;
        self.cs.set_high();
        r?;
        out.copy_from_slice(&buf[skip..n]);
        Ok(())
    }

    async fn write(&mut self, reg: u8, data: &[u8]) -> Result<(), Error> {
        self.cs.set_low();
        let mut r = self.spi.write(&[reg & 0x7F]).await;
        if r.is_ok() {
            r = self.spi.write(data).await;
        }
        self.cs.set_high();
        Timer::after_micros(2).await; // BMI270 needs ≥2 µs between writes in low power
        Ok(r?)
    }

    async fn init_bmi270(&mut self) -> Result<(), Error> {
        self.write(0x7E, &[0xB6]).await?; // CMD: soft reset
        Timer::after_millis(2).await;
        let mut id = [0u8; 1];
        let _ = self.read(0x00, &mut id).await; // back to SPI mode after reset
        self.write(0x7C, &[0x00]).await?; // PWR_CONF: advanced power save off
        Timer::after_micros(450).await;
        self.write(0x59, &[0x00]).await?; // INIT_CTRL: start config load
        self.write(0x5E, &BMI270_CONFIG).await?; // INIT_DATA: burst upload
        self.write(0x59, &[0x01]).await?; // INIT_CTRL: done
        Timer::after_millis(20).await;
        let mut status = [0u8; 1];
        self.read(0x21, &mut status).await?; // INTERNAL_STATUS
        if status[0] & 0x0F != 0x01 {
            return Err(Error);
        }
        self.write(0x7D, &[0x0E]).await?; // PWR_CTRL: accel, gyro, temp on
        self.write(0x40, &[0xAC]).await?; // ACC_CONF: 1600 Hz, normal filter, perf mode
        self.write(0x41, &[BMI_ACC_RANGE]).await?; // ACC_RANGE
        self.write(0x42, &[0xED]).await?; // GYR_CONF: 3200 Hz, normal filter, perf mode
        self.write(0x43, &[BMI_GYR_RANGE]).await?; // GYR_RANGE
        self.write(0x53, &[0x0A]).await?; // INT1_IO_CTRL: output, push-pull, active high
        self.write(0x58, &[0x04]).await?; // INT_MAP_DATA: data ready on INT1
        self.write(0x7C, &[0x02]).await?; // PWR_CONF: no power save
        Timer::after_millis(50).await;
        self.set_speed(8_000_000); // BMI270 allows up to 10 MHz
        Ok(())
    }

    async fn init_icm42688(&mut self) -> Result<(), Error> {
        self.write(0x11, &[0x01]).await?; // DEVICE_CONFIG: soft reset
        Timer::after_millis(2).await;
        self.write(0x4E, &[0x0F]).await?; // PWR_MGMT0: gyro + accel low-noise
        Timer::after_millis(1).await;
        self.write(0x4F, &[ICM_GYRO_CONFIG0]).await?; // GYRO_CONFIG0: range, 2 kHz
        self.write(0x50, &[ICM_ACCEL_CONFIG0]).await?; // ACCEL_CONFIG0: range, 2 kHz
        // INT_CONFIG: INT1 push-pull, active high, pulsed (bit 2 clear).
        // Pulsed is what the edge-triggered data-ready interrupt needs.
        self.write(0x14, &[0x03]).await?;
        self.write(0x64, &[0x00]).await?; // INT_CONFIG1: clear INT_ASYNC_RESET
        self.write(0x65, &[0x08]).await?; // INT_SOURCE0: UI data ready on INT1
        Timer::after_millis(50).await; // gyro start-up
        self.set_speed(24_000_000); // ICM-42688 allows up to 24 MHz
        Ok(())
    }

    /// One sample: gyro (rad/s) and accel (m/s², specific force) in FRD axes.
    pub async fn sample(&mut self) -> Result<([f32; 3], [f32; 3]), Error> {
        let mut b = [0u8; 12];
        let (accel, gyro) = match self.chip {
            Chip::Bmi270 => {
                self.read(0x0C, &mut b).await?; // ACC_X..GYR_Z, little-endian
                let v = |i: usize| i16::from_le_bytes([b[i], b[i + 1]]) as f32;
                ([v(0), v(2), v(4)], [v(6), v(8), v(10)])
            }
            Chip::Icm42688 => {
                self.read(0x1F, &mut b).await?; // ACCEL_DATA_X1..GYRO_DATA_Z0, big-endian
                let v = |i: usize| i16::from_be_bytes([b[i], b[i + 1]]) as f32;
                ([v(0), v(2), v(4)], [v(6), v(8), v(10)])
            }
        };
        let frd = |v: [f32; 3], k: f32| [v[0] * k, -v[1] * k, -v[2] * k];
        Ok((frd(gyro, GYRO_SCALE), frd(accel, ACCEL_SCALE)))
    }
}
