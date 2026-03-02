/// Mifare Classic Value Block layout (16 bytes total):
/// Bytes 0-3:  Value (little-endian i32)
/// Bytes 4-7:  ~Value (bitwise NOT of value)
/// Bytes 8-11: Value (copy, little-endian i32)
/// Byte 12:    Address
/// Byte 13:    ~Address (bitwise NOT of address)
/// Byte 14:    Address (copy)
/// Byte 15:    ~Address (bitwise NOT of address, copy)

#[derive(Debug, Clone, PartialEq)]
pub struct ValueBlock {
    pub value: i32,
    pub address: u8,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ValueBlockError {
    InvalidLength,
    ValueMismatch,
    ValueInvertMismatch,
    AddressMismatch,
    AddressInvertMismatch,
}

impl ValueBlock {
    pub fn from_bytes(data: &[u8]) -> Result<Self, ValueBlockError> {
        if data.len() < 16 {
            return Err(ValueBlockError::InvalidLength);
        }

        // Parse the three value copies
        let value1 = i32::from_le_bytes(data[0..4].try_into().unwrap());
        let value_inv = i32::from_le_bytes(data[4..8].try_into().unwrap());
        let value2 = i32::from_le_bytes(data[8..12].try_into().unwrap());

        // Validate value copies match
        if value1 != value2 {
            return Err(ValueBlockError::ValueMismatch);
        }

        // Validate inverted value
        if value1 != !value_inv {
            return Err(ValueBlockError::ValueInvertMismatch);
        }

        let addr1 = data[12];
        let addr_inv = data[13];
        let addr2 = data[14];
        let addr_inv2 = data[15];

        // Validate address copies match
        if addr1 != addr2 {
            return Err(ValueBlockError::AddressMismatch);
        }

        // Validate inverted addresses
        if addr1 != !addr_inv || addr1 != !addr_inv2 {
            return Err(ValueBlockError::AddressInvertMismatch);
        }

        Ok(ValueBlock {
            value: value1,
            address: addr1,
        })
    }
}

/// Calculate CRC16 for MIFARE Classic
pub fn calc_crc16(data: &[u8]) -> [u8; 2] {
    let mut crc = 0x6363; // ITU-V.41
    
    for &byte in data {
        let b = byte as u16;
        crc ^= b & 0xff;
        
        for _ in 0..8 {
            if (crc & 0x0001) != 0 {
                crc = (crc >> 1) ^ 0x8408;
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    [(crc & 0xff) as u8, (crc >> 8) as u8]
}

