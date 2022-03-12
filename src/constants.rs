#![allow(dead_code)]

use defmt::Format;
use num_enum::TryFromPrimitive;

use crate::{SrRes, SunriseError};

#[derive(Clone, Copy, Debug, PartialEq, Format)]
enum MeasurementMode {
    Cnntinuous,
    Single,
}

enum CalibrationType {
    FactoryReset = 0x7C02,
    AbcForced = 0x7C03,
    Target = 0x7C05,
    Abc = 0x7C06,
    Zero = 0x7C07,
}

#[derive(Clone, Copy, Debug, PartialEq, Format)]
enum CalibrationStatus {
    FactoryCalibrationRestored = 2,
    AbcCalibration = 3,
    TargetCalibration = 4,
    BackgroundCalibration = 5,
    ZeroCalibration = 6,
}

#[derive(Clone, Copy, Debug, PartialEq, Format)]
pub enum ErrorStatusMask {
    AlgorithmError = 0x0004,
    CalibrationError = 0x0008,
    Fatal = 0x0001,
    I2cError = 0x0002,
    LowInternalVoltage = 0x0100,
    MeasurementNotCompleted = 0x0080,
    MeasurementTimeout = 0x0200,
    MemoryError = 0x0040,
    OutOfRange = 0x0020,
    SelfDiagnosticsError = 0x0010,
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
    Reserved18 = 0xDE,
    Reserved19 = 0xDF,
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
