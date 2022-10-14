#![allow(dead_code)]
use defmt::Format;
pub use num_enum::TryFromPrimitive;

use crate::{SrRes, SunriseError};

#[derive(Clone, Copy, Debug, PartialEq, Format)]
pub enum MeasurementMode {
    Continuous(u16),
    Single,
}

impl Default for MeasurementMode {
    fn default() -> Self {
        Self::Continuous(16)
    }
}

pub enum CalibrationCommand {
    FactoryReset = 0x7C02,
    AbcForced = 0x7C03,
    Target = 0x7C05,
    Abc = 0x7C06,
    Zero = 0x7C07,
}

#[derive(Clone, Copy, PartialEq, Format)]
pub struct MeterControlValue(u8);

impl MeterControlValue {
    pub const NRDY_ENABLED: u8 = (0 << 0);
    pub const NRDY_DISABLED: u8 = (1 << 0);

    pub const ABC_ENABLED: u8 = (0 << 1);
    pub const ABC_DISABLED: u8 = (1 << 1);

    pub const STATIC_IIR_ENABLED: u8 = (0 << 2);
    pub const STATIC_IIR_DISABLED: u8 = (1 << 2);

    pub const DYNAMIC_IIR_ENABLED: u8 = (0 << 3);
    pub const DYNAMIC_IIR_DISABLED: u8 = (1 << 3);

    pub const PCOMP_ENABLED: u8 = (0 << 4);
    pub const PCOMP_DISABLED: u8 = (1 << 4);

    pub const NRDY_INVERT_ENABLED: u8 = (0 << 5);
    pub const NRDY_INVERT_DISABLED: u8 = (1 << 5);

    pub fn is_nrdy_enabled(&self) -> bool {
        self.0 & Self::NRDY_DISABLED == 0
    }

    pub fn is_nrdy_inverted(&self) -> bool {
        self.0 & Self::NRDY_INVERT_DISABLED == 0
    }

    pub fn is_abc_enabled(&self) -> bool {
        self.0 & Self::ABC_DISABLED == 0
    }

    pub fn is_static_iir_enabled(&self) -> bool {
        self.0 & Self::STATIC_IIR_DISABLED == 0
    }

    pub fn is_dynamic_iir_enabled(&self) -> bool {
        self.0 & Self::DYNAMIC_IIR_DISABLED == 0
    }

    pub fn is_pcomp_enabled(&self) -> bool {
        self.0 & Self::PCOMP_DISABLED == 0
    }
}

impl core::fmt::Debug for MeterControlValue {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "MeterControlValue({}) ", self.0)?;

        let mut set = f.debug_set();

        if self.is_abc_enabled() {
            set.entry(&"is_abc_enabled");
        }
        if self.is_dynamic_iir_enabled() {
            set.entry(&"is_dynamic_iir_enabled");
        }
        if self.is_nrdy_enabled() {
            set.entry(&"is_nrdy_enabled");
        }
        if self.is_nrdy_inverted() {
            set.entry(&"is_nrdy_inverted");
        }
        if self.is_pcomp_enabled() {
            set.entry(&"is_pcomp_enabled");
        }
        if self.is_static_iir_enabled() {
            set.entry(&"is_static_iir_enabled");
        }

        set.finish()
    }
}

impl From<u8> for MeterControlValue {
    fn from(num: u8) -> Self {
        Self(num)
    }
}

impl From<MeterControlValue> for u8 {
    fn from(mc: MeterControlValue) -> Self {
        mc.0
    }
}

impl Default for MeterControlValue {
    fn default() -> Self {
        Self(Self::PCOMP_DISABLED | Self::NRDY_INVERT_DISABLED)
    }
}

#[derive(Clone, Copy, PartialEq, Format)]
pub struct CalibrationStatusValue(u8);

impl CalibrationStatusValue {
    pub const FACTORY_CALIBRATION_RESTORED: u8 = (1 << 2);
    pub const ABC_CALIBRATION: u8 = (1 << 3);
    pub const TARGET_CALIBRATION: u8 = (1 << 4);
    pub const BACKGROUND_CALIBRATION: u8 = (1 << 5);
    pub const ZERO_CALIBRATION: u8 = (1 << 6);

    pub fn is_factory_calibration_restored(&self) -> bool {
        self.0 & Self::FACTORY_CALIBRATION_RESTORED != 0
    }

    pub fn is_abc_calibration(&self) -> bool {
        self.0 & Self::ABC_CALIBRATION != 0
    }

    pub fn is_target_calibration(&self) -> bool {
        self.0 & Self::TARGET_CALIBRATION != 0
    }

    pub fn is_background_calibration(&self) -> bool {
        self.0 & Self::BACKGROUND_CALIBRATION != 0
    }

    pub fn is_zero_calibration(&self) -> bool {
        self.0 & Self::ZERO_CALIBRATION != 0
    }
}

impl core::fmt::Debug for CalibrationStatusValue {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "CalibrationStatusValue({}) ", self.0)?;

        let mut set = f.debug_set();

        if self.is_factory_calibration_restored() {
            set.entry(&"factory_calibration_restored");
        }
        if self.is_abc_calibration() {
            set.entry(&"abc_calibration");
        }
        if self.is_background_calibration() {
            set.entry(&"background_calibration");
        }
        if self.is_target_calibration() {
            set.entry(&"target_calibration");
        }
        if self.is_zero_calibration() {
            set.entry(&"zero_calibration");
        }

        set.finish()
    }
}

impl From<u8> for CalibrationStatusValue {
    fn from(num: u8) -> Self {
        Self(num)
    }
}

impl From<CalibrationStatusValue> for u8 {
    fn from(cs: CalibrationStatusValue) -> Self {
        cs.0
    }
}

#[derive(Clone, Copy, PartialEq)]
pub struct ErrorStatus(u16);

impl ErrorStatus {
    pub const FATAL: u16 = 1 << 0;
    pub const I2C_ERROR: u16 = 1 << 1;
    pub const ALGORITHM_ERROR: u16 = 1 << 2;
    pub const CALIBRATION_ERROR: u16 = 1 << 3;
    pub const SELF_DIAGNOSTICS_ERROR: u16 = 1 << 4;
    pub const OUT_OF_RANGE: u16 = 1 << 5;
    pub const MEMORY_ERROR: u16 = 1 << 6;
    pub const MEASUREMENT_NOT_COMPLETED: u16 = 1 << 7;
    pub const LOW_INTERNAL_VOLTAGE: u16 = 1 << 8;
    pub const MEASUREMENT_TIMEOUT: u16 = 1 << 9;
    pub const ABNORMAL_SIGNAL_LEVEL: u16 = 1 << 10;
    pub const RESERVED_01: u16 = 1 << 11;
    pub const RESERVED_02: u16 = 1 << 12;
    pub const RESERVED_03: u16 = 1 << 13;
    pub const RESERVED_04: u16 = 1 << 14;
    pub const RESERVED_05: u16 = 1 << 15;

    pub fn from_be_bytes(bytes: [u8; 2]) -> Self {
        u16::from_be_bytes(bytes).into()
    }

    pub fn fatal(&self) -> bool {
        self.0 & Self::FATAL != 0
    }

    pub fn i2c_error(&self) -> bool {
        self.0 & Self::I2C_ERROR != 0
    }

    pub fn algorithm_error(&self) -> bool {
        self.0 & Self::ALGORITHM_ERROR != 0
    }

    pub fn calibration_error(&self) -> bool {
        self.0 & Self::CALIBRATION_ERROR != 0
    }

    pub fn self_diagnostics_error(&self) -> bool {
        self.0 & Self::SELF_DIAGNOSTICS_ERROR != 0
    }

    pub fn out_of_range(&self) -> bool {
        self.0 & Self::OUT_OF_RANGE != 0
    }

    pub fn memory_error(&self) -> bool {
        self.0 & Self::MEMORY_ERROR != 0
    }

    pub fn measurement_not_completed(&self) -> bool {
        self.0 & Self::MEASUREMENT_NOT_COMPLETED != 0
    }

    pub fn low_internal_voltage(&self) -> bool {
        self.0 & Self::LOW_INTERNAL_VOLTAGE != 0
    }

    pub fn measurement_timeout(&self) -> bool {
        self.0 & Self::MEASUREMENT_TIMEOUT != 0
    }

    pub fn abnormal_signal_level(&self) -> bool {
        self.0 & Self::ABNORMAL_SIGNAL_LEVEL != 0
    }
}

impl core::fmt::Debug for ErrorStatus {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "ErrorStatus({}) ", self.0)?;

        let mut set = f.debug_set();

        if self.fatal() {
            set.entry(&"fatal");
        }
        if self.i2c_error() {
            set.entry(&"i2c_error");
        }
        if self.algorithm_error() {
            set.entry(&"algorithm_error");
        }
        if self.calibration_error() {
            set.entry(&"calibration_error");
        }
        if self.self_diagnostics_error() {
            set.entry(&"self_diagnostics_error");
        }
        if self.out_of_range() {
            set.entry(&"out_of_range");
        }
        if self.memory_error() {
            set.entry(&"memory_error");
        }
        if self.measurement_not_completed() {
            set.entry(&"measurement_not_completed");
        }
        if self.low_internal_voltage() {
            set.entry(&"low_internal_voltage");
        }
        if self.measurement_timeout() {
            set.entry(&"measurement_timeout");
        }
        if self.abnormal_signal_level() {
            set.entry(&"abnormal_signal_level");
        }

        set.finish()
    }
}

impl defmt::Format for ErrorStatus {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{}", defmt::Debug2Format(self));
    }
}

impl From<u16> for ErrorStatus {
    fn from(value: u16) -> Self {
        ErrorStatus(value)
    }
}

impl From<ErrorStatus> for u16 {
    fn from(es: ErrorStatus) -> Self {
        es.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Format, TryFromPrimitive)]
#[repr(u8)]
pub enum Registers {
    ErrorstatusMsb = 0x00,
    ErrorstatusLsb = 0x01,

    Co2FpMsb = 0x06,
    Co2FpLsb = 0x07,

    TemperatureMsb = 0x08,
    TemperatureLsb = 0x09,

    MeasurementCount = 0x0D,
    MeasurementCycleTimeMsb = 0x0E,
    MeasurementCycleTimeLsb = 0x0F,

    Co2UpMsb = 0x10,
    Co2UpLsb = 0x11,
    Co2FMsb = 0x12,
    Co2FLsb = 0x13,
    Co2UMsb = 0x14,
    Co2ULsb = 0x15,

    FirmwareRevMsb = 0x38,
    FirmwareRevLsb = 0x39,

    SensorIdMmsb = 0x3A,
    SensorIdMlsb = 0x3B,
    SensorIdLmsb = 0x3C,
    SensorIdLlsb = 0x3D,

    CalibrationStatus = 0x81,
    CalibrationCommandMsb = 0x82,
    CalibrationCommandLsb = 0x83,
    CalibrationTargetMsb = 0x84,
    CalibrationTargetLsb = 0x85,

    Co2ValueOverrideMsb = 0x86,
    Co2ValueOverrideLsb = 0x87,

    AbcTimeMsb = 0x88,
    AbcTimeLsb = 0x89,
    AbcPar0Msb = 0x8A,
    AbcPar0Lsb = 0x8B,
    AbcPar1Msb = 0x8C,
    AbcPar1Lsb = 0x8D,
    AbcPar2Msb = 0x8E,
    AbcPar2Lsb = 0x8F,
    AbcPar3Msb = 0x90,
    AbcPar3Lsb = 0x91,

    StartSingleMeasurement = 0x93,

    MeasurementMode = 0x95,      // EE
    MeasurementPeriodMsb = 0x96, // EE
    MeasurementPeriodLsb = 0x97, // EE
    NumberOfSamplesMsb = 0x98,   // EE
    NumberOfSamplesLsb = 0x99,   // EE
    AbcPeriodMsb = 0x9A,         // EE
    AbcPeriodLsb = 0x9B,         // EE

    ClearErrorstatus = 0x9D,

    AbcTargetMsb = 0x9E,             // EE
    AbcTargetLsb = 0x9F,             // EE
    StaticIirFilterParameter = 0xA1, // EE

    SCR = 0xA3,

    MeterControl = 0xA5, // EE
    MbI2cAddress = 0xA7, // EE

    // Registers from address 0xC0 to 0xCD are mirrors of registers at addresses 0x80, 0x81, 0x92, 0x93, and 0x88 to 0x91.
    CalibrationStatusMirror = 0xC1,      // 0x81
    StartSingleMeasurementMirror = 0xC3, // 0x93
    AbcTimeMsbMirror = 0xC4,             // 0x88
    AbcTimeLsbMirror = 0xC5,             // 0x89
    AbcPar0MsbMirror = 0xC6,             // 0x8A
    AbcPar0LsbMirror = 0xC7,             // 0x8B
    AbcPar1MsbMirror = 0xC8,             // 0x8C
    AbcPar1LsbMirror = 0xC9,             // 0x8D
    AbcPar2MsbMirror = 0xCA,             // 0x8E
    AbcPar2LsbMirror = 0xCB,             // 0x8F
    AbcPar3MsbMirror = 0xCC,             // 0x90
    AbcPar3LsbMirror = 0xCD,             // 0x91

    FilterPar0Msb = 0xCE,
    FilterPar0Lsb = 0xCF,
    FilterPar1Msb = 0xD0,
    FilterPar1Lsb = 0xD1,
    FilterPar2Msb = 0xD2,
    FilterPar2Lsb = 0xD3,
    FilterPar3Msb = 0xD4,
    FilterPar3Lsb = 0xD5,
    FilterPar4Msb = 0xD6,
    FilterPar4Lsb = 0xD7,
    FilterPar5Msb = 0xD8,
    FilterPar5Lsb = 0xD9,
    FilterPar6Msb = 0xDA,
    FilterPar6Lsb = 0xDB,

    BarometricAirPressureValueMsb = 0xDC,
    BarometricAirPressureValueLsb = 0xDD,

    AbcBarometricAirPressureValueMsb = 0xDE,
    AbcBarometricAirPressureValueLsb = 0xDF,

    Reserved01 = 0x02,
    Reserved02 = 0x03,
    Reserved03 = 0x04,
    Reserved04 = 0x05,
    Reserved05 = 0x0A,
    Reserved06 = 0x0B,
    Reserved07 = 0x0C,
    Reserved08 = 0x3E,
    Reserved09 = 0x3F,
    Reserved10 = 0x80,
    Reserved11 = 0x92,
    Reserved12 = 0x94,
    Reserved13 = 0x9C,
    Reserved14 = 0xA0,
    Reserved15 = 0xA2,
    Reserved16 = 0xA4,
    Reserved17 = 0xA6,
    Reserved10Mirror = 0xC0, // 0x80
    Reserved11Mirror = 0xC2, // 0x92
}

impl Registers {
    pub fn is_ee(self) -> bool {
        let reg: u8 = self as u8;
        (reg >= 0x95 && reg <= 0xA1) || reg == 0xa5 || reg == 0xa7
    }

    pub fn try_next(self) -> SrRes<Self> {
        Self::try_from_primitive((self as u8) + 1).or(Err(SunriseError::IllegalRegister))
    }
}
