#![no_std]

use core::time::Duration;
use defmt::Format;

use embedded_hal::{
    blocking::i2c::{Read, Write, WriteRead},
    digital::v2::{InputPin, OutputPin},
};

mod constants;
use constants::Registers::{self, *};
pub use constants::{
    CalibrationCommand, CalibrationStatusValue, ErrorStatus, MeasurementMode, MeterControlValue,
};

const DEFAULT_I2C_ADRESS: u8 = 0x68;

#[derive(Clone, Copy, PartialEq, Debug, Format)]
pub enum SunriseError {
    I2CReadError(u8),
    I2CWriteError(u8),
    GpioError,
    IllegalRegister,
    ArgumentError,
    OutOfRange,
    ReadyReadFail,
    NotSupported,
    NeedReset,
    TimedOut,
}

#[derive(Clone, Copy, PartialEq, Debug, Format)]
/// Tracking wether the sensor needs the (ABC and filter) state data
/// written back to it and if the state tracked on the host system is valid
enum State {
    ///  no saved state (1st startup, no state on host)
    Initial,
    /// Sensor need copy of state (after enabling/reset of sensor, no state on sensor)
    NeedsHostData,
    /// Sensor has valid data
    Valid,
}

pub type SrRes<T> = Result<T, SunriseError>;

#[derive(Clone, Copy, PartialEq, Debug, Format)]
pub struct MeasurementResult {
    pub co2_filtered_pc: u16,
    pub co2_filtered: u16,
    pub co2_unfiltered_pc: u16,
    pub co2_unfiltered: u16,
    pub temp_raw: u16,
    pub measurement_count: u8,
    pub error_status: ErrorStatus,
}

pub struct SunriseI2C<EN, RDY, I2C> {
    en_pin: Option<EN>,
    rdy_pin: RDY,
    i2c: I2C,
    dev_addr: u8,
    uptime: fn() -> Duration,
    state: State,
    state_data: [u8; 28],
    last_abc: Duration,
    meter_control: MeterControlValue,
}

impl<EN, RDY, I2C, E> SunriseI2C<EN, RDY, I2C>
where
    EN: OutputPin,
    RDY: InputPin,
    I2C: WriteRead<Error = E> + Write<Error = E> + Read<Error = E>,
    E: core::fmt::Debug,
{
    pub fn new(i2c: I2C, en_pin: EN, rdy_pin: RDY, uptime: fn() -> Duration) -> SrRes<Self> {
        let mut co2_sensor = Self {
            en_pin: Some(en_pin),
            rdy_pin,
            i2c,
            dev_addr: DEFAULT_I2C_ADRESS,
            uptime,
            state_data: [0; 28],
            state: State::Initial,
            last_abc: uptime(),
            meter_control: Default::default(),
        };

        co2_sensor.enable()?;
        co2_sensor.meter_control = co2_sensor.get_meter_control()?;

        Ok(co2_sensor)
    }

    pub fn set_measurement_mode(&mut self, mode: MeasurementMode) -> SrRes<()> {
        match mode {
            MeasurementMode::Continuous(period_s) => {
                self.set_continuous_measurement_period(period_s)?;
                self.write_reg8(MeasurementMode, 0)
            }
            MeasurementMode::Single => self.write_reg8(MeasurementMode, 1),
        }?;
        self.soft_reset().or(Err(SunriseError::NeedReset))
    }

    pub fn do_single_measurement(&mut self) -> SrRes<CalibrationStatusValue> {
        defmt::debug!("do_single_measurement, state: {}", self.state);

        if self.state == State::NeedsHostData {
            let mut single_with_state = [0u8; 30];
            single_with_state[0] = StartSingleMeasurementMirror as u8;
            single_with_state[1] = 1;
            single_with_state[2..].clone_from_slice(&self.state_data);

            let abc_age = self.abc_age_hours();
            single_with_state[2] = (abc_age >> 8) as u8;
            single_with_state[3] = (abc_age & 0xff) as u8;

            // if self.is_pcomp_enabled() && self.abc_pressure != 0 {
            //     self.set_abc_barometric_air_pressure(self.abc_pressure)?;
            // }

            defmt::debug!(
                "do_single_measurement(): restore state: {}",
                self.state_data
            );
            self.write_raw(&single_with_state)?;
        } else {
            self.write_reg8(StartSingleMeasurement, 1)?;
        }

        self.state = State::Valid;

        // Now waits for 3s max (2.4s needed for default settings as per datasheet)
        // TODO: implement actual needed timeout value dependant on settings (e.g. number of samnples)
        self.wait_rdy(3000)?;

        // Read calibration status.
        // This function also resets the `self.last_abc` field when a calibration occurred
        self.get_calibration_status()
    }

    pub fn enable(&mut self) -> SrRes<()> {
        if let Some(en) = self.en_pin.as_mut() {
            defmt::trace!("Enable sensor");
            let _ = en.set_high();
            self.delay_ms(100);
            Ok(())
        } else {
            Err(SunriseError::NotSupported)
        }
    }

    pub fn disable(&mut self) -> SrRes<()> {
        if self.en_pin.is_some() {
            defmt::trace!("Disable sensor (state: {:?})", self.state);

            if self.state == State::Valid {
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

    pub fn save_state_on_host(&mut self) -> SrRes<()> {
        let mut state_data = [0u8; 28];
        let result = self.read_reg(AbcTimeMsbMirror, &mut state_data[..]); // the filter params trail the AbcTimeMsbMirror register 0xc4
        self.state_data = state_data;

        defmt::debug!("save_state_on_host({:?}) -> {}", result, self.state_data);

        result
    }

    fn abc_age_hours(&self) -> u16 {
        let diff = self.now() - self.last_abc;
        let hours = (diff.as_secs() / 3600) as u16;
        defmt::debug!("abc_age_hours(): {}", hours);

        hours
    }

    pub fn get_all_measurements(&mut self) -> SrRes<MeasurementResult> {
        let mut data = [0u8; 22];
        self.read_reg(ErrorstatusMsb, &mut data)?;

        Ok(MeasurementResult {
            error_status: ErrorStatus::from_be_bytes([data[0], data[1]]),
            measurement_count: data[0x0d],
            temp_raw: (data[0x08] as u16) << 8 | data[0x09] as u16,

            co2_filtered_pc: (data[0x06] as u16) << 8 | data[0x07] as u16,
            co2_unfiltered_pc: (data[0x10] as u16) << 8 | data[0x11] as u16,
            co2_filtered: (data[0x12] as u16) << 8 | data[0x13] as u16,
            co2_unfiltered: (data[0x14] as u16) << 8 | data[0x15] as u16,
        })
    }

    pub fn get_error_status(&mut self) -> SrRes<ErrorStatus> {
        let value = self.read_reg16(ErrorstatusMsb)?;
        Ok(value.into())
    }

    pub fn clear_error_status(&mut self) -> SrRes<()> {
        self.write_reg8(ClearErrorstatus, 1)
    }

    /// Default period is 16 secs
    pub fn set_continuous_measurement_period(&mut self, period_s: u16) -> SrRes<()> {
        self.write_reg16(MeasurementPeriodMsb, period_s)
    }

    pub fn get_meter_control(&mut self) -> SrRes<MeterControlValue> {
        let mc = self.read_reg8(MeterControl)?;
        Ok(mc.into())
    }

    pub fn set_meter_control(&mut self, control_value: MeterControlValue) -> SrRes<()> {
        self.meter_control = control_value;
        if self.read_reg8(MeterControl)? != control_value.into() {
            self.write_reg8(MeterControl, control_value.into())
        } else {
            Ok(())
        }
    }

    pub fn get_measurement_cycle_time(&mut self) -> SrRes<u16> {
        self.read_reg16(MeasurementCycleTimeMsb)
    }

    pub fn get_measurement_count(&mut self) -> SrRes<u8> {
        self.read_reg8(MeasurementCount)
    }

    pub fn get_co2_filt_pcmp(&mut self) -> SrRes<u16> {
        self.read_reg16(Co2FpMsb)
    }

    pub fn get_co2_filt(&mut self) -> SrRes<u16> {
        self.read_reg16(Co2FMsb)
    }

    pub fn get_co2_unfilt_pcmp(&mut self) -> SrRes<u16> {
        self.read_reg16(Co2UpMsb)
    }

    pub fn get_co2_unfilt(&mut self) -> SrRes<u16> {
        self.read_reg16(Co2UMsb)
    }

    pub fn get_temp(&mut self) -> SrRes<f32> {
        let temp_raw = self.read_reg16(TemperatureMsb)?;
        Ok(temp_raw as f32 / 100.0)
    }

    /// Default is 8 samples
    pub fn set_number_of_samples(&mut self, samples: u16) -> SrRes<()> {
        if self.read_reg16(NumberOfSamplesMsb)? != samples {
            self.write_reg16(NumberOfSamplesMsb, samples)
        } else {
            Ok(())
        }
    }
    /// Valid range 2-10. Default of this setting is unclear...
    pub fn set_static_iir_filter_parameter(&mut self, param: u8) -> SrRes<()> {
        if (param < 2) || (param > 10) {
            Err(SunriseError::ArgumentError)
        } else {
            self.write_reg8(StaticIirFilterParameter, param)
        }
    }

    /// Set barometric air pressure given in hPa (scaled * 10 by this function)
    pub fn set_barometric_air_pressure(&mut self, hpa: u16) -> SrRes<()> {
        if (300..=1300).contains(&hpa) {
            self.write_reg16(BarometricAirPressureValueMsb, hpa * 10)
        } else {
            Err(SunriseError::OutOfRange)
        }
    }

    /// Set ABC barometric air pressure given in hPa (scaled * 10 by this function)
    pub fn set_abc_barometric_air_pressure(&mut self, hpa: u16) -> SrRes<()> {
        if (300..=1300).contains(&hpa) {
            self.write_reg16(AbcBarometricAirPressureValueMsb, hpa * 10)
        } else {
            Err(SunriseError::OutOfRange)
        }
    }

    /// Get ABC barometric air pressure setting in hPa (scaled / 10 by this function)
    pub fn get_abc_barometric_air_pressure(&mut self) -> SrRes<u16> {
        self.read_reg16(AbcBarometricAirPressureValueMsb)
            .map(|p| p / 10)
    }

    pub fn do_calibration_cycle(
        &mut self,
        cmd: CalibrationCommand,
    ) -> SrRes<CalibrationStatusValue> {
        self.clear_calibration_status()?;

        self.send_calibration_command(cmd)?;
        self.do_single_measurement()?;

        self.get_calibration_status()
    }

    /// Send calibration command, will be 'executed' on next measurement, after which
    /// calibration status can be checked (`get_calibration_status()`)
    pub fn send_calibration_command(&mut self, cmd: CalibrationCommand) -> SrRes<()> {
        self.write_reg16(CalibrationCommandMsb, cmd as u16)
    }

    pub fn get_calibration_status(&mut self) -> SrRes<CalibrationStatusValue> {
        let value = self.read_reg8(CalibrationStatus)?;

        if value > 0 {
            self.last_abc = self.now()
        }

        Ok(value.into())
    }

    pub fn clear_calibration_status(&mut self) -> SrRes<()> {
        self.write_reg8(CalibrationStatus, 0)
    }

    /// Default is 180 hours
    pub fn set_abc_period(&mut self, hours: u16) -> SrRes<()> {
        self.write_reg16(AbcPeriodMsb, hours)
    }

    pub fn get_abc_period(&mut self) -> SrRes<u16> {
        self.read_reg16(AbcPeriodMsb)
    }

    /// Default is 400 ppm
    pub fn set_abc_target(&mut self, target_ppm: u16) -> SrRes<()> {
        self.write_reg16(AbcTargetMsb, target_ppm)
    }

    pub fn get_abc_target(&mut self) -> SrRes<u16> {
        self.read_reg16(AbcTargetMsb)
    }

    pub fn set_calibration_target(&mut self, target_ppm: u16) -> SrRes<()> {
        self.write_reg16(CalibrationTargetMsb, target_ppm)
    }

    pub fn get_calibration_target(&mut self) -> SrRes<u16> {
        self.read_reg16(CalibrationTargetMsb)
    }

    pub fn fw_rev(&mut self) -> SrRes<u16> {
        self.read_reg16(FirmwareRevMsb)
    }

    pub fn device_id(&mut self) -> SrRes<u32> {
        self.read_reg32(SensorIdMmsb)
    }

    pub fn soft_reset(&mut self) -> SrRes<()> {
        if self.disable().is_ok() {
            self.delay_ms(10);
            self.enable()
        } else {
            if self.state == State::Valid {
                self.state = State::NeedsHostData;
                self.save_state_on_host()?;
            }
            self.write_reg8(SCR, 0xff) // Hmm.. this does not seem to work all too well (i.e. at all),
                                       // at least not with older (-0002) revisions of the Sunrise sensor.
        }
    }

    pub fn delay_ms(&self, millis: u64) {
        if millis > 0 {
            let stop_time = self.now() + Duration::from_millis(millis);
            while self.now() < stop_time {}
        }
    }

    pub fn now(&self) -> Duration {
        (self.uptime)()
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

        // Delay not needed in newer sensor types (need some further testing)
        // self.delay_ms(1); // Fixes (workaround?) wierd I2C io errors that pop up occasionally
        // Adding the delay in wake() (on err/nack) is not helping
        // Problem arises when demo is switched from continuous mode to single mode
        // when trace logging (line above) is disabled.. (cause is not clear for now)

        Ok(())
    }

    fn write_reg16(&mut self, reg: Registers, data: u16) -> SrRes<()> {
        if reg.is_ee() {
            // Write as separate bytes with appropriate delay for EE registers
            let next_reg = reg.try_next()?;
            self.write_reg8(reg, (data >> 8) as u8)?;
            self.write_reg8(next_reg, (data & 0xff) as u8)
        } else {
            self.write_raw(&[reg as u8, (data >> 8) as u8, (data & 0xff) as u8])?;
            // self.delay_ms(1);
            Ok(())
        }
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
            self.delay_ms(25);
        } else {
            // self.delay_ms(1);
        }
        Ok(())
    }

    fn write_raw(&mut self, data: &[u8]) -> SrRes<()> {
        defmt::trace!("write_raw {:#04x}", data);
        self.wake();

        self.i2c
            .write(self.dev_addr, data)
            .or(Err(SunriseError::I2CWriteError(self.dev_addr)))
    }

    /// Wake sensor by a dummy read opeation
    ///
    /// Senseair Sunrise/Sunlight spend most of its time in deep sleep mode to minimise power consumption,
    /// this have the effect that it is necessary to wake up the sensors before it is possible to communicate
    /// with them. Sensor will wake up on a falling edge on SDA, it is recommended to send sensors address
    /// to wake it up. When sensors address is used to wake up the sensor, the sensor will not acknowledge
    /// this byte if it in sleep mode and will acknowledge it if sensor is already activated to process a
    /// measurement or a previous communication attempt.
    fn wake(&mut self) {
        let mut dummy = [0u8];

        // When the sensor was asleep this read is not ACK'd
        if let Err(_) = self.i2c.read(self.dev_addr, &mut dummy) {
            // This does not seem to fix the ocasional read error...
            defmt::trace!("Sensor woken from sleep");
            self.delay_ms(1);
        }
    }

    fn wait_rdy(&self, mut timeout_ms: i32) -> SrRes<()> {
        defmt::trace!("wait_rdy({})", timeout_ms);

        while !self.is_ready()? && timeout_ms > 0 {
            self.delay_ms(1);
            timeout_ms -= 1;
        }

        if timeout_ms <= 0 {
            defmt::warn!("wait_rdy, timed out!");
            Err(SunriseError::TimedOut)
        } else {
            defmt::trace!("wait_rdy, time left {}ms", timeout_ms);
            Ok(())
        }
    }

    pub fn is_ready(&self) -> SrRes<bool> {
        // ... removed code for handling inverted `nRDY` pin
        self.rdy_pin.is_low().or(Err(SunriseError::ReadyReadFail))
    }
}
