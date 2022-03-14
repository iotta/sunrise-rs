#![no_std]
mod constants;

use constants::{
    ErrorStatusMask::{self, *},
    MeasurementMode,
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
    GpioError,
    IllegalRegister,
    ReadyReadFail,
    NotSupported,
    NeedReset,
    TimedOut,
}

#[derive(Clone, Copy, PartialEq, Debug, Format)]
enum State {
    Initial,       // no saved state (1st startup, no state on host)
    NeedsHostData, // Sensor need copy of state (after enabling/reset of sensor, no state on sensor)
    Valid,         // Sensor has valid data
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
    state: State,
    state_data: [u8; 24],
}

impl<EN, RDY, I2C, D, E> SunriseI2C<EN, RDY, I2C, D>
where
    EN: OutputPin,
    RDY: InputPin,
    I2C: WriteRead<Error = E> + Write<Error = E> + Read<Error = E>,
    E: core::fmt::Debug,
    D: DelayMs<u32>,
{
    pub fn new(i2c: I2C, en_pin: EN, rdy_pin: RDY, delay: D) -> Self {
        Self {
            en_pin: Some(en_pin),
            rdy_pin,
            i2c,
            delay,
            dev_addr: DEFAULT_I2C_ADRESS,
            state_data: [0; 24],
            state: State::Initial,
        }
    }

    pub fn get_error_status(&mut self) -> SrRes<ErrorStatus> {
        let raw = self.read_reg16(ErrorstatusMsb)?;
        Ok(ErrorStatus(raw))
    }

    pub fn set_single_measurement_mode(&mut self) -> SrRes<()> {
        self.set_measurement_mode(MeasurementMode::Single)
    }

    /// Default period is 16 secs
    pub fn set_continuous_measurement_mode(&mut self, period_s: u16) -> SrRes<()> {
        if period_s == 0 {
            self.write_reg16(MeasurementPeriodMsb, 16)?;
        } else {
            self.write_reg16(MeasurementPeriodMsb, period_s)?;
        }

        self.set_measurement_mode(MeasurementMode::Continuous)
    }

    fn set_measurement_mode(&mut self, mode: MeasurementMode) -> SrRes<()> {
        match mode {
            MeasurementMode::Continuous => self.write_reg8(MeasurementMode, 0),
            MeasurementMode::Single => self.write_reg8(MeasurementMode, 1),
        }?;
        self.soft_reset().or(Err(SunriseError::NeedReset))
    }

    pub fn do_single_measurement(&mut self) -> SrRes<()> {
        defmt::debug!("do_single_measurement, state: {}", self.state);

        if self.state == State::NeedsHostData {
            let mut single_with_state = [0u8; 26];
            single_with_state[0] = StartSingleMeasurementMirror as u8;
            single_with_state[1] = 1;
            single_with_state[2..].clone_from_slice(&self.state_data);

            defmt::debug!("Restore state_w: {}", single_with_state);
            self.write_raw(&single_with_state)?;
        } else {
            self.write_reg8(StartSingleMeasurement, 1)?;
        }

        self.state = State::Valid;
        self.wait_rdy()
    }

    pub fn save_state_on_host(&mut self) -> SrRes<()> {
        let mut state_data = [0u8; 24];
        let result = self.read_reg(AbcTimeMsbMirror, &mut state_data[..]); // the filter params trail the AbcTimeMsbMirror register 0xc4
        self.state_data = state_data;

        defmt::debug!("save state({:?}) -> {}", result, self.state_data);

        result
    }

    pub fn delay_ms(&mut self, delay: u32) {
        self.delay.delay_ms(delay);
    }

    pub fn get_measurement_count(&mut self) -> SrRes<u8> {
        self.read_reg8(MeasurementCount)
    }

    pub fn get_co2_filt_pcmp(&mut self) -> SrRes<u16> {
        self.read_reg16(Co2FpMsb)
    }

    /// Default is 8 samples
    pub fn set_number_of_samples(&mut self, samples: u16) -> SrRes<()> {
        self.write_reg16(NumberOfSamplesMsb, samples)
    }

    /// Default is 180 hours
    pub fn set_abc_period(&mut self, period_h: u16) -> SrRes<()> {
        self.write_reg16(AbcTargetMsb, period_h)
    }

    /// Default is 400 ppm
    pub fn set_abc_target(&mut self, target_ppm: u16) -> SrRes<()> {
        self.write_reg16(AbcTargetMsb, target_ppm)
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

    pub fn clear_error_status(&mut self) -> SrRes<()> {
        self.write_reg8(ClearErrorstatus, 1)
    }

    pub fn disable(&mut self) -> SrRes<()> {
        if self.en_pin.is_some() {
            defmt::trace!("Disable sensor (state: {:?})", self.state);

            if self.state != State::Initial {
                self.state = State::NeedsHostData;
                self.save_state_on_host()?; // How to handle(?): not being able to save status to host, prevents sensor from sleeping..
            }
        }

        if let Some(en) = self.en_pin.as_mut() {
            en.set_low().or(Err(SunriseError::GpioError))
        } else {
            Err(SunriseError::NotSupported)
        }
    }

    pub fn enable(&mut self) -> SrRes<()> {
        if let Some(en) = self.en_pin.as_mut() {
            defmt::trace!("Enable sensor");
            let _ = en.set_high();
            self.delay.delay_ms(35);
            Ok(())
        } else {
            Err(SunriseError::NotSupported)
        }
    }

    pub fn soft_reset(&mut self) -> SrRes<()> {
        // if self.state != State::Initial {
        //     self.state = State::NeedsHostData;
        //     self.save_state_on_host()?;
        // }
        // self.write_reg8(SCR, 0xff) // Hmm.. this does not seem to work all too well (i.e. at all).

        self.disable()?;
        self.delay.delay_ms(10);
        self.enable()
    }

    fn read_reg8(&mut self, reg: Registers) -> SrRes<u8> {
        let mut buf = [0u8];
        self.read_reg(reg, &mut buf)?;

        Ok(u8::from_be_bytes(buf))
    }

    fn read_reg16(&mut self, reg: Registers) -> SrRes<u16> {
        let mut buf = [0u8; 2];
        self.read_reg(reg, &mut buf)?;

        Ok(u16::from_be_bytes(buf))
    }

    fn read_reg32(&mut self, reg: Registers) -> SrRes<u32> {
        let mut buf = [0u8; 4];
        self.read_reg(reg, &mut buf)?;

        Ok(u32::from_be_bytes(buf))
    }

    fn read_reg(&mut self, reg: Registers, buffer: &mut [u8]) -> SrRes<()> {
        self.wake();
        self.i2c
            .write_read(self.dev_addr, &[reg as u8], buffer)
            .or(Err(SunriseError::I2CReadError(self.dev_addr)))?;

        defmt::trace!("read_reg {}({:#04x}) -> {:#04x}", reg, reg as u8, buffer);
        self.delay.delay_ms(1); // Fixes (workaround?) wierd I2C io errors that pop up occasionally
                                // Adding the delay in wake() (on err/nack) is not helping
                                // Problem arises when demo is switched from continuous mode to single mode
                                // when trace logging (line above) is disabled.. (cause is not clear for now)
        Ok(())
    }

    fn write_reg16(&mut self, reg: Registers, data: u16) -> SrRes<()> {
        let next_reg = reg.try_next()?;
        self.write_reg8(reg, (data >> 8) as u8)?;
        self.write_reg8(next_reg, (data & 0xff) as u8)
    }

    fn write_reg8(&mut self, reg: Registers, data: u8) -> SrRes<()> {
        defmt::trace!(
            "write_reg {}({:#04x}{}): {:#04x}",
            reg,
            reg as u8,
            if reg.is_ee() { ", EE" } else { "" },
            data
        );

        self.write_raw(&[reg as u8, data])?;

        if reg.is_ee() {
            self.delay.delay_ms(25);
        } else {
            self.delay.delay_ms(1);
        }
        Ok(())
    }

    fn write_raw(&mut self, data: &[u8]) -> SrRes<()> {
        self.wake();

        self.i2c
            .write(self.dev_addr, data)
            .or(Err(SunriseError::I2CWriteError(self.dev_addr)))
    }

    /// Dummy write for waking up the sensor
    fn wake(&mut self) {
        let mut dummy = [0u8];
        // let _ = self.i2c.write(self.dev_addr, &mut dummy);
        if let Err(_) = self.i2c.read(self.dev_addr, &mut dummy) {
            // This does not seem to fix the ocasional read error
            // self.delay.delay_ms(1);
        }
    }

    fn _wake(&mut self) -> SrRes<()> {
        let dummy = [0u8];
        let mut times = 5; // max tries for wakin'
        while self.i2c.write(self.dev_addr, &dummy).is_err() && times > 0 {
            times -= 1;
        }

        if times > 0 {
            Ok(())
        } else {
            defmt::warn!("wake() -> SunriseError::TimedOut");
            Err(SunriseError::TimedOut)
        }
    }

    fn wait_rdy(&self) -> SrRes<()> {
        defmt::trace!("wait_rdy");
        // self.delay.delay_ms(no_samples * 300);
        while !self.is_ready()? {}
        Ok(())
    }

    pub fn is_ready(&self) -> SrRes<bool> {
        self.rdy_pin.is_low().or(Err(SunriseError::ReadyReadFail))
    }
}
