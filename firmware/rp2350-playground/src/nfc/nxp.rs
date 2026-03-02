use crate::nfc::vendor::Vendor;
use crate::nfc::Chip;


pub struct VersionResponse {
    pub vendor_id: u8,
    pub product_type: u8,
    pub product_subtype: u8,
    pub major_version: u8,
    pub minor_version: u8,
    pub storage_size: u8,
    pub protocol_type: u8,
}

impl VersionResponse {
    /// Parses a GET_VERSION response from an 8-byte slice.
    /// Returns None if the slice is not exactly 8 bytes.
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.len() != 8 {
            return None;
        }
        if data[0] != 0x00 {
            // The first byte should be 0x00 for a valid GET_VERSION response
            return None;
        }
        Some(VersionResponse {
            vendor_id: data[1],
            product_type: data[2],
            product_subtype: data[3],
            major_version: data[4],
            minor_version: data[5],
            storage_size: data[6],
            protocol_type: data[7],
        })
    }

    pub fn vendor(&self) -> Option<Vendor> {
        Vendor::try_from(self.vendor_id).ok()
    }

    pub fn chip(&self) -> Option<Chip> {
        match (
            self.product_type,
            self.product_subtype,
            self.major_version,
            self.minor_version,
            self.storage_size,
        ) {
            (0x04, 0x01, 0x01, 0x00, 0x0B) => Some(Chip::NxpNTAG210),
            (0x04, 0x01, 0x01, 0x00, 0x0E) => Some(Chip::NxpNTAG212),
            (0x04, 0x02, 0x01, 0x00, 0x0F) => Some(Chip::NxpNTAG213),
            (0x04, 0x04, 0x01, 0x00, 0x0F) => Some(Chip::NxpNTAG213F),
            (0x04, 0x02, 0x01, 0x00, 0x11) => Some(Chip::NxpNTAG215),
            (0x04, 0x02, 0x01, 0x00, 0x13) => Some(Chip::NxpNTAG216),
            (0x04, 0x04, 0x01, 0x00, 0x13) => Some(Chip::NxpNTAG216F),
            (0x04, 0x02, 0x01, 0x01, 0x13) => Some(Chip::NxpNT3H1101),
            (0x04, 0x05, 0x02, 0x01, 0x13) => Some(Chip::NxpNT3H1101W0),
            (0x04, 0x05, 0x02, 0x02, 0x13) => Some(Chip::NxpNT3H2111W0),
            (0x04, 0x02, 0x01, 0x01, 0x15) => Some(Chip::NxpNT3H2101),
            (0x04, 0x05, 0x02, 0x01, 0x15) => Some(Chip::NxpNT3H1201W0),
            (0x04, 0x05, 0x02, 0x02, 0x15) => Some(Chip::NxpNT3H2211W0),
            (0x03, 0x01, 0x01, 0x00, 0x0B) => Some(Chip::NxpMF0UL1101),
            (0x03, 0x02, 0x01, 0x00, 0x0B) => Some(Chip::NxpMF0ULH1101),
            (0x03, 0x01, 0x01, 0x00, 0x0E) => Some(Chip::NxpMF0UL2101),
            (0x03, 0x02, 0x01, 0x00, 0x0E) => Some(Chip::NxpMF0ULH2101),
            _ => None,
        }
    }
}
