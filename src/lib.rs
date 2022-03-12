#![no_std]
mod constants;

use constants::{
    ErrorStatusMask::{self, *},
    Registers::{self, *},
};
use defmt::Format;

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
use embedded_hal::digital::v2::{InputPin, OutputPin};

static DEFAULT_I2C_ADRESS: u8 = 0x68;

#[derive(Clone, Copy, PartialEq, Debug, Format)]
pub enum SunriseError {
    // #[error("Error while reading from slave '{0}' on i2c bus")]
    // #[display(fmt = "Error while reading from slave '{:#x}' on i2c bus", _0)]
    I2CReadError(u8),
    // #[error("Error while writing to slave '{0:x}' on i2c bus")]
    I2CWriteError(u8),
    IllegalRegister,
    ReadyReadFail,
}

pub type SrRes<T> = Result<T, SunriseError>;

#[derive(Clone, Copy, Debug)]
pub struct ErrorStatus(pub u16);

impl defmt::Format for ErrorStatus {
    fn format(&self, fmt: defmt::Formatter) {
        let masks = [
            AlgorithmError,
            CalibrationError,
            Fatal,
            I2cError,
            LowInternalVoltage,
            MeasurementNotCompleted,
            MeasurementTimeout,
            MemoryError,
            OutOfRange,
            SelfDiagnosticsError,
        ];

        defmt::write!(fmt, "ErrorStatus:[");
        for mask in masks {
            if self.has_error(mask) {
                defmt::write!(fmt, "{},", mask);
            }
        }
        defmt::write!(fmt, "]");
    }
}

impl ErrorStatus {
    pub fn has_error(&self, msk: ErrorStatusMask) -> bool {
        (self.0 & (msk as u16)) > 0
    }
}

pub struct SunriseI2C<EN, RDY, TWI, D> {
    en_pin: Option<EN>,
    rdy_pin: RDY,
    i2c: TWI,
    dev_addr: u8,
    delay: D,
}

impl<EN, RDY, I2C, D, E> SunriseI2C<EN, RDY, I2C, D>
where
    EN: OutputPin,
    RDY: InputPin,
    I2C: WriteRead<Error = E> + Write<Error = E> + Read<Error = E>,
    // E: core::fmt::Debug,
    D: DelayMs<u32>,
{
    pub fn new(i2c: I2C, en_pin: EN, rdy_pin: RDY, delay: D) -> Self {
        Self {
            en_pin: Some(en_pin),
            rdy_pin,
            i2c,
            delay,
            dev_addr: DEFAULT_I2C_ADRESS,
        }
    }

    pub fn get_error_status(&mut self) -> SrRes<ErrorStatus> {
        let raw = self.read_reg16(ErrorstatusMsb)?;
        Ok(ErrorStatus(raw))
    }

    pub fn do_single_measurement(&mut self) -> SrRes<()> {
        self.write_reg8(StartSingleMeasurement, 1)?;
        // self.delay.delay_ms(2400);
        self.wait_rdy()
    }

    pub fn get_co2_filt_pcmp(&mut self) -> SrRes<u16> {
        self.read_reg16(Co2FpMsb)
    }

    pub fn set_abc_target(&mut self, target: u16) -> SrRes<()> {
        self.write_reg16(AbcTargetMsb, target)
    }

    pub fn get_abc_target(&mut self) -> SrRes<u16> {
        self.read_reg16(AbcTargetMsb)
    }

    pub fn get_temp(&mut self) -> SrRes<f32> {
        let temp_raw = self.read_reg16(TemperatureMsb)?;
        Ok(temp_raw as f32 / 100.0)
    }

    pub fn fw_rev(&mut self) -> SrRes<u16> {
        self.read_reg16(FirmwareRevMsb)
    }

    pub fn device_id(&mut self) -> SrRes<u32> {
        self.read_reg32(SensorIdMmsb)
    }

    pub fn disable(&mut self) {
        if let Some(en) = self.en_pin.as_mut() {
            defmt::trace!("Disable sensor");
            let _ = en.set_low();
        }
    }

    pub fn enable(&mut self) {
        if let Some(en) = self.en_pin.as_mut() {
            defmt::trace!("Enable sensor");
            let _ = en.set_high();
            self.delay.delay_ms(35);
        }
    }

    fn read_reg8(&mut self, reg: Registers) -> SrRes<u8> {
        let mut buf = [0u8];
        self.read_reg(reg, &mut buf)?;

        Ok(buf[0] as u8)
    }

    fn read_reg16(&mut self, reg: Registers) -> SrRes<u16> {
        let mut buf = [0u8; 2];
        self.read_reg(reg, &mut buf)?;

        Ok((buf[0] as u16) << 8 | (buf[1] as u16))
    }

    fn read_reg32(&mut self, reg: Registers) -> SrRes<u32> {
        let mut buf = [0u8; 4];
        self.read_reg(reg, &mut buf)?;

        Ok((buf[0] as u32) << 24 | (buf[1] as u32) << 16 | (buf[2] as u32) << 8 | (buf[3] as u32))
    }

    fn read_reg(&mut self, reg: Registers, buffer: &mut [u8]) -> SrRes<()> {
        self.wake();
        self.i2c
            .write_read(self.dev_addr, &[reg as u8], buffer)
            .or(Err(SunriseError::I2CReadError(self.dev_addr)))
    }

    fn write_reg16(&mut self, reg: Registers, data: u16) -> SrRes<()> {
        let next_reg = reg.try_next()?;
        self.write_reg8(reg, (data >> 8) as u8)?;
        self.write_reg8(next_reg, (data & 0xff) as u8)
    }

    fn write_reg8(&mut self, reg: Registers, data: u8) -> SrRes<()> {
        self.wake();

        let result = self
            .i2c
            .write(self.dev_addr, &[reg as u8, data])
            .or(Err(SunriseError::I2CWriteError(self.dev_addr)));

        if reg.is_ee() {
            self.delay.delay_ms(25);
        } else {
            self.delay.delay_ms(1);
        }

        result
    }

    /// Dummy write for waking up the sensor
    fn wake(&mut self) {
        let dummy = [0u8];
        let _ = self.i2c.write(self.dev_addr, &dummy); // It's expected to return an Err(AddressNack)
    }

    fn wait_rdy(&self) -> SrRes<()> {
        defmt::trace!("wait_rdy");
        while !self.is_ready()? {}
        Ok(())
    }

    pub fn is_ready(&self) -> SrRes<bool> {
        self.rdy_pin.is_low().or(Err(SunriseError::ReadyReadFail))
    }
}
