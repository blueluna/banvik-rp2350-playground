//! NFC-related code, including parsing of NDEF messages and TLV tags.

// NDEF (NFC Data Exchange Format)
pub mod ndef;
// NFC Forum Tag Type 2
pub mod tlvtag;
// Crypto1 stream cipher used in MIFARE Classic
pub mod crypto1;

pub mod mifare;

pub mod vendor;

pub mod nxp;

/// Coding of Answer to Request Type A (ATQA) values for different NFC tag types.
#[derive(Debug)]
#[repr(u16)]
pub enum Atqa {
    MifarePlus1K = 0x0004,
    MifarePlus4K = 0x0002,
    MifareUltralight = 0x0044,
}

impl TryFrom<u16> for Atqa {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        match value {
            0x0004 => Ok(Atqa::MifarePlus1K),
            0x0002 => Ok(Atqa::MifarePlus4K),
            0x0044 => Ok(Atqa::MifareUltralight),
            _ => Err(()),
        }
    }
}

#[derive(Debug)]
pub enum TargetType {
    Unknown,
    MifareUltralightCl2,
    MifareClassic1K,
    MifareClassic1KCl2,
    MifareClassic4K,
    MifareClassic4KCl2,
    MifarePlus2K,
    MifarePlus2KCl2,
    MifarePlus4K,
    MifarePlus4KCl2,
    MifarePlusSe,
    MifarePlusSeCl2,
}

impl TargetType {
    pub fn from_response(sak: u8, uid_length: u8) -> Self {
        match (sak, uid_length) {
            (0x00, 7) => TargetType::MifareUltralightCl2,
            (0x08, 4) => TargetType::MifareClassic1K,
            (0x18, 4) => TargetType::MifareClassic4K,
            (0x08, 7) => TargetType::MifareClassic1KCl2,
            (0x18, 7) => TargetType::MifareClassic4KCl2,
            (0x10, 4) => TargetType::MifarePlus2K,
            (0x11, 4) => TargetType::MifarePlus4K,
            (0x10, 7) => TargetType::MifarePlus2KCl2,
            (0x11, 7) => TargetType::MifarePlus4KCl2,
            (0x20, 4) => TargetType::MifarePlus2K,
            (0x20, 7) => TargetType::MifarePlus2KCl2,
            _ => TargetType::Unknown,
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for TargetType {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            TargetType::Unknown => defmt::write!(fmt, "Unknown"),
            TargetType::MifareUltralightCl2 => defmt::write!(fmt, "Mifare Ultralight C/L2"),
            TargetType::MifareClassic1K => defmt::write!(fmt, "Mifare Classic 1K"),
            TargetType::MifareClassic1KCl2 => defmt::write!(fmt, "Mifare Classic 1K C/L2"),
            TargetType::MifareClassic4K => defmt::write!(fmt, "Mifare Classic 4K"),
            TargetType::MifareClassic4KCl2 => defmt::write!(fmt, "Mifare Classic 4K C/L2"),
            TargetType::MifarePlus2K => defmt::write!(fmt, "Mifare Plus 2K"),
            TargetType::MifarePlus2KCl2 => defmt::write!(fmt, "Mifare Plus 2K C/L2"),
            TargetType::MifarePlus4K => defmt::write!(fmt, "Mifare Plus 4K"),
            TargetType::MifarePlus4KCl2 => defmt::write!(fmt, "Mifare Plus 4K C/L2"),
            TargetType::MifarePlusSe => defmt::write!(fmt, "Mifare Plus SE"),
            TargetType::MifarePlusSeCl2 => defmt::write!(fmt, "Mifare Plus SE C/L2"),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Chip {
    NxpNTAG210,
    NxpNTAG212,
    NxpNTAG213,
    NxpNTAG213F,
    NxpNTAG215,
    NxpNTAG216,
    NxpNTAG216F,
    NxpNT3H1101,
    NxpNT3H1101W0,
    NxpNT3H2111W0,
    NxpNT3H2101,
    NxpNT3H1201W0,
    NxpNT3H2211W0,
    NxpMF0UL1101,
    NxpMF0ULH1101,
    NxpMF0UL2101,
    NxpMF0ULH2101,
}

impl Chip {
    pub fn to_str(&self) -> &'static str {
        match self {
            Chip::NxpNTAG210 => "NXP NTAG210",
            Chip::NxpNTAG212 => "NXP NTAG212",
            Chip::NxpNTAG213 => "NXP NTAG213",
            Chip::NxpNTAG213F => "NXP NTAG213F",
            Chip::NxpNTAG215 => "NXP NTAG215",
            Chip::NxpNTAG216 => "NXP NTAG216",
            Chip::NxpNTAG216F => "NXP NTAG216F",
            Chip::NxpNT3H1101 => "NXP NT3H1101",
            Chip::NxpNT3H1101W0 => "NXP NT3H1101W0",
            Chip::NxpNT3H2111W0 => "NXP NT3H2111W0",
            Chip::NxpNT3H2101 => "NXP NT3H2101",
            Chip::NxpNT3H1201W0 => "NXP NT3H1201W0",
            Chip::NxpNT3H2211W0 => "NXP NT3H2211W0",
            Chip::NxpMF0UL1101 => "NXP MF0UL1101",
            Chip::NxpMF0ULH1101 => "NXP MF0ULH1101",
            Chip::NxpMF0UL2101 => "NXP MF0UL2101",
            Chip::NxpMF0ULH2101 => "NXP MF0ULH2101",
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Chip {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{}", self.to_str());
    }
}