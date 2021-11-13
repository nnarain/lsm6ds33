//
// regs.rs
//
// @author Natesh Narain <nnaraindev@gmail.com>
// @date Nov 12 2021
//

// Device registers
#[derive(Clone, Copy)]
pub enum Register {
    FifoCtrl1 = 0x06,
    FifoCtrl2 = 0x07,
    FifoCtrl3 = 0x08,
    FifoCtrl4 = 0x09,
    FifoCtrl5 = 0x0A,

    OrientCfgG = 0x0B,

    Int1Ctrl = 0x0D,
    Int2Ctrl = 0x0E,

    WhoAmI = 0x0F,

    Ctrl1XL = 0x10,
    Ctrl2G = 0x11,
    Ctrl3C = 0x12,
    Ctrl4C = 0x13,
    Ctrl5C = 0x14,
    Ctrl6C = 0x15,
    Ctrl7G = 0x16,
    Ctrl8Xl = 0x18,
    Ctrl10C = 0x19,

    WakeUpSrc = 0x1B,
    TapSrc = 0x1C,
    D6dSrc = 0x1D,

    StatusReg = 0x1E,

    OutTempL = 0x1F,
    OutTempH = 0x20,

    OutXLG = 0x22,
    OutXHG = 0x23,
    OutYLG = 0x24,
    OutYHG = 0x25,
    OutZLG = 0x26,
    OutZHG = 0x27,
    OutXLXL = 0x28,
    OutXHXL = 0x29,
    OutYLXL = 0x2A,
    OutYHXL = 0x2B,
    OutZLXL = 0x2C,
    OutZHXL = 0x2D,

    FifoStatus1 = 0x3A,
    FifoStatus2 = 0x3B,
    FifoStatus3 = 0x3C,
    FifoStatus4 = 0x3D,

    FifoDataOutL = 0x3E,
    FifoDataOutH = 0x3F,

    Timestamp1Reg = 0x41,
    Timestamp2Reg = 0x42,

    StepTimestampL = 0x49,
    StepTimestampH = 0x4A,

    StepCounterL = 0x4B,
    StepCounterH = 0x4C,

    FuncSrc = 0x53,

    TapCfg = 0x58,
    TapThs6d = 0x59,

    IntDur2 = 0x5A,

    WakeUpThs = 0x5B,
    WakeUpDur = 0x5C,

    FreeFall = 0x5D,

    Md1Cfg = 0x5E,
    Md2Cfg = 0x5F,
}

impl From<Register> for u8 {
    fn from(r: Register) -> u8 {
        r as u8
    }
}

pub trait RegisterOption {
    fn value(&self) -> u8;
    fn bit_offset(&self) -> u8;
    fn mask(&self) -> u8;
}

// ---------------------------------------------------------------------------------------------------------------------
// --- CTRL1_XL --------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------

#[derive(Debug, Clone, Copy)]
pub enum AccelerometerOutput {
    PowerDown = 0b0000,
    Rate13    = 0b0001,
    Rate26    = 0b0010,
    Rate52    = 0b0011,
    Rate104   = 0b0100,
    Rate208   = 0b0101,
    Rate416   = 0b0110,
    Rate833   = 0b0111,
    Rate1_66k = 0b1000,
    Rate3_33k = 0b1001,
    Rate6_66k = 0b1010,
}

impl RegisterOption for AccelerometerOutput {
    fn value(&self) -> u8 {
        *self as u8
    }
    fn mask(&self) -> u8 {
        0xF
    }
    fn bit_offset(&self) -> u8 {
        4
    }
}

/// Accelerometer full-scale selection
#[derive(Debug, Clone, Copy)]
pub enum AccelerometerScale {
    G02 = 0b00,
    G16 = 0b01,
    G04 = 0b10,
    G08 = 0b11,
}

impl RegisterOption for AccelerometerScale {
    fn value(&self) -> u8 {
        *self as u8
    }
    fn mask(&self) -> u8 {
        0x03
    }
    fn bit_offset(&self) -> u8 {
        2
    }
}

#[derive(Debug, Clone, Copy)]
pub enum AccelerometerBandwidth {
    Freq400 = 0b00,
    Freq200 = 0b01,
    Freq100 = 0b10,
    Freq50  = 0b11,
}

impl RegisterOption for AccelerometerBandwidth {
    fn value(&self) -> u8 {
        *self as u8
    }
    fn mask(&self) -> u8 {
        0x03
    }
    fn bit_offset(&self) -> u8 {
        0
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// --- CTRL2_G --------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------

#[derive(Debug, Clone, Copy)]
pub enum GyroscopeOutput {
    PowerDown = 0b0000,
    Rate13    = 0b0001,
    Rate26    = 0b0010,
    Rate52    = 0b0011,
    Rate104   = 0b0100,
    Rate208   = 0b0101,
    Rate416   = 0b0110,
    Rate833   = 0b0111,
    Rate1_66k = 0b1000,
    Rate3_33k = 0b1001,
    Rate6_66k = 0b1010,
}

impl RegisterOption for GyroscopeOutput {
    fn value(&self) -> u8 {
        *self as u8
    }
    fn mask(&self) -> u8 {
        0xF
    }
    fn bit_offset(&self) -> u8 {
        4
    }
}

#[derive(Debug, Clone, Copy)]
pub enum GyroscopeFullScale {
    Dps245 = 0b00,
    Dps500 = 0b01,
    Dps1000 = 0b10,
    Dps2000 = 0b11,
}

impl RegisterOption for GyroscopeFullScale {
    fn value(&self) -> u8 {
        *self as u8
    }
    fn mask(&self) -> u8 {
        0b11
    }
    fn bit_offset(&self) -> u8 {
        2
    }
}

#[derive(Debug, Clone, Copy)]
pub enum GyroscopeFullScale125Dps {
    Disabled = 0,
    Enabled = 1,
}

impl RegisterOption for GyroscopeFullScale125Dps {
    fn value(&self) -> u8 {
        *self as u8
    }
    fn mask(&self) -> u8 {
        0b1
    }
    fn bit_offset(&self) -> u8 {
        1
    }
}

// pub fn option_mask<T: RegisterOption>(opt: &T) -> u8 {
//     0x01 << opt.bit_offset()
// }

/// Bit fields for CTRL3_C
pub enum Ctrl3C {
    Boot = 7,
    BlockDataUpdate = 6,
    InterruptActivationLevel = 5,
    InterruptPadOutput = 4,
    SpiSerialInterfaceMode = 3,
    AutoIncrement = 2,
    Endian = 1,
    SoftwareReset = 0,
}
