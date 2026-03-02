/// NFC Forum Type 2 Tag TLV tag byte values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TlvTag {
    /// Null TLV — padding, no length or value field follows.
    Null = 0x00,
    /// Lock Control TLV — describes locked areas of memory.
    LockControl = 0x01,
    /// Memory Control TLV — describes reserved memory areas.
    MemoryControl = 0x02,
    /// NDEF Message TLV — contains an NDEF message.
    NdefMessage = 0x03,
    /// Proprietary TLV — vendor-specific data.
    Proprietary = 0xFD,
    /// Terminator TLV — marks the end of the TLV area. No length or value follows.
    Terminator = 0xFE,
}

impl TlvTag {
    fn from_byte(b: u8) -> Option<Self> {
        match b {
            0x00 => Some(Self::Null),
            0x01 => Some(Self::LockControl),
            0x02 => Some(Self::MemoryControl),
            0x03 => Some(Self::NdefMessage),
            0xFD => Some(Self::Proprietary),
            0xFE => Some(Self::Terminator),
            _ => None,
        }
    }

    /// Returns true if this tag type has no length/value fields.
    fn is_single_byte(self) -> bool {
        matches!(self, TlvTag::Null | TlvTag::Terminator)
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for TlvTag {
    fn format(&self, f: defmt::Formatter<'_>) {
        match self {
            TlvTag::Null => defmt::write!(f, "Null"),
            TlvTag::LockControl => defmt::write!(f, "LockControl"),
            TlvTag::MemoryControl => defmt::write!(f, "MemoryControl"),
            TlvTag::NdefMessage => defmt::write!(f, "NdefMessage"),
            TlvTag::Proprietary => defmt::write!(f, "Proprietary"),
            TlvTag::Terminator => defmt::write!(f, "Terminator"),
        }
    }
}

/// A single decoded TLV record.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Tlv<'a> {
    pub tag: TlvTag,
    /// The raw value bytes. Empty for single-byte TLVs (Null, Terminator).
    pub value: &'a [u8],
}

/// Errors that can occur during TLV decoding.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TlvError {
    /// An unrecognised tag byte was encountered.
    UnknownTag(u8),
    /// The buffer ended before the length field could be read.
    TruncatedLength,
    /// The buffer ended before the value field was fully read.
    TruncatedValue,
}

#[cfg(feature = "defmt")]
impl defmt::Format for TlvError {
    fn format(&self, f: defmt::Formatter<'_>) {
        match self {
            TlvError::UnknownTag(b) => defmt::write!(f, "UnknownTag({=u8:#x})", b),
            TlvError::TruncatedLength => defmt::write!(f, "TruncatedLength"),
            TlvError::TruncatedValue => defmt::write!(f, "TruncatedValue"),
        }
    }
}

/// Iterator over TLV records in a NFC Forum Type 2 Tag memory region.
///
/// # Example
///
/// ```rust
/// # use rp2350_playground::nfc::tlvtag::TlvDecoder;
/// let data: &[u8] = &[
///     0x00, 0x03, 0x14, 0xd1, 0x01, 0x10, 0x54, 0x02,
///     0x65, 0x6e, 0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x2c,
///     0x20, 0x57, 0x6f, 0x72, 0x6c, 0x64, 0x21, 0xfe,
/// ];
///
/// let mut decoder = TlvDecoder::new(data);
/// while let Some(result) = decoder.next() {
///     match result {
///         Ok(tlv) => { /* process tlv */ }
///         Err(e)  => { /* handle error */ }
///     }
/// }
/// ```
pub struct TlvDecoder<'a> {
    buf: &'a [u8],
    pos: usize,
    terminated: bool,
}

impl<'a> TlvDecoder<'a> {
    /// Create a new decoder over `buf`.
    pub fn new(buf: &'a [u8]) -> Self {
        Self {
            buf,
            pos: 0,
            terminated: false,
        }
    }

    /// Return the current byte offset into the buffer.
    pub fn position(&self) -> usize {
        self.pos
    }

    /// Advance to the next TLV record.
    ///
    /// Returns `None` when the buffer is exhausted or a Terminator TLV was seen.
    /// Returns `Some(Err(_))` on a malformed record.
    pub fn next(&mut self) -> Option<Result<Tlv<'a>, TlvError>> {
        if self.terminated || self.pos >= self.buf.len() {
            return None;
        }

        let tag_byte = self.buf[self.pos];
        self.pos += 1;

        let tag = match TlvTag::from_byte(tag_byte) {
            Some(t) => t,
            None => return Some(Err(TlvError::UnknownTag(tag_byte))),
        };

        // Single-byte TLVs have no length or value.
        if tag.is_single_byte() {
            if tag == TlvTag::Terminator {
                self.terminated = true;
            }
            return Some(Ok(Tlv { tag, value: &[] }));
        }

        // Read length — supports the three-byte format (0xFF LL LL) for lengths > 254.
        let length: usize = match self.read_byte() {
            None => return Some(Err(TlvError::TruncatedLength)),
            Some(0xFF) => {
                // Three-byte length format: 0xFF followed by two big-endian bytes.
                let hi = match self.read_byte() {
                    Some(b) => b as usize,
                    None => return Some(Err(TlvError::TruncatedLength)),
                };
                let lo = match self.read_byte() {
                    Some(b) => b as usize,
                    None => return Some(Err(TlvError::TruncatedLength)),
                };
                (hi << 8) | lo
            }
            Some(len) => len as usize,
        };

        // Slice out the value bytes.
        let end = self.pos + length;
        if end > self.buf.len() {
            return Some(Err(TlvError::TruncatedValue));
        }
        let value = &self.buf[self.pos..end];
        self.pos = end;

        Some(Ok(Tlv { tag, value }))
    }

    fn read_byte(&mut self) -> Option<u8> {
        if self.pos < self.buf.len() {
            let b = self.buf[self.pos];
            self.pos += 1;
            Some(b)
        } else {
            None
        }
    }
}

// ---------------------------------------------------------------------------
// Convenience: collect all TLVs up to the Terminator into a fixed-size array.
// ---------------------------------------------------------------------------

/// Decode up to `N` TLV records from `buf` into a stack-allocated array.
///
/// Returns `(records, count)` where `count` is the number of records decoded.
/// Stops at the first error, Terminator, or when the array is full.
pub fn decode_tlvs<'a, const N: usize>(
    buf: &'a [u8],
) -> ([Option<Tlv<'a>>; N], usize, Option<TlvError>) {
    // SAFETY: Option<Tlv> is Copy so this MaybeUninit trick is unnecessary;
    // we can initialise with None directly via a const.
    const NONE: Option<Tlv<'static>> = None;
    // Transmute-free workaround for const generics on stable no_std:
    let mut records: [Option<Tlv<'a>>; N] = [NONE; N];
    let mut decoder = TlvDecoder::new(buf);
    let mut count = 0;

    loop {
        if count == N {
            break;
        }
        match decoder.next() {
            None => break,
            Some(Err(e)) => return (records, count, Some(e)),
            Some(Ok(tlv)) => {
                records[count] = Some(tlv);
                count += 1;
                if tlv.tag == TlvTag::Terminator {
                    break;
                }
            }
        }
    }

    (records, count, None)
}

// ---------------------------------------------------------------------------
// Tests (conditionally compiled; use `cargo test` in a std environment)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// The two data blocks from the "Hello, World!" example.
    const HELLO_WORLD: &[u8] = &[
        0x00, 0x03, 0x14, 0xd1, 0x01, 0x10, 0x54, 0x02, 0x65, 0x6e, 0x48, 0x65, 0x6c, 0x6c, 0x6f,
        0x2c, 0x20, 0x57, 0x6f, 0x72, 0x6c, 0x64, 0x21, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00,
    ];

    #[test]
    fn test_hello_world_tlvs() {
        let mut decoder = TlvDecoder::new(HELLO_WORLD);

        // First record: Null TLV (padding)
        let tlv = decoder.next().unwrap().unwrap();
        assert_eq!(tlv.tag, TlvTag::Null);
        assert!(tlv.value.is_empty());

        // Second record: NDEF Message TLV, length 0x14 = 20 bytes
        let tlv = decoder.next().unwrap().unwrap();
        assert_eq!(tlv.tag, TlvTag::NdefMessage);
        assert_eq!(tlv.value.len(), 20);
        // First byte of NDEF payload is the record header 0xD1
        assert_eq!(tlv.value[0], 0xd1);

        // Third record: Terminator
        let tlv = decoder.next().unwrap().unwrap();
        assert_eq!(tlv.tag, TlvTag::Terminator);

        // Iterator should now be exhausted
        assert!(decoder.next().is_none());
    }

    #[test]
    fn test_three_byte_length() {
        // Craft a TLV with a three-byte length field (0xFF, 0x00, 0x05) and 5 value bytes.
        let buf: &[u8] = &[0x03, 0xFF, 0x00, 0x05, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFE];
        let mut decoder = TlvDecoder::new(buf);

        let tlv = decoder.next().unwrap().unwrap();
        assert_eq!(tlv.tag, TlvTag::NdefMessage);
        assert_eq!(tlv.value, &[0xAA, 0xBB, 0xCC, 0xDD, 0xEE]);
    }

    #[test]
    fn test_truncated_value_error() {
        // Length says 10 bytes but buffer only has 3.
        let buf: &[u8] = &[0x03, 0x0A, 0x01, 0x02, 0x03];
        let mut decoder = TlvDecoder::new(buf);
        assert_eq!(decoder.next().unwrap(), Err(TlvError::TruncatedValue));
    }

    #[test]
    fn test_unknown_tag_error() {
        let buf: &[u8] = &[0xAB];
        let mut decoder = TlvDecoder::new(buf);
        assert_eq!(decoder.next().unwrap(), Err(TlvError::UnknownTag(0xAB)));
    }

    #[test]
    fn test_decode_tlvs_helper() {
        let (records, count, err) = decode_tlvs::<8>(HELLO_WORLD);
        assert!(err.is_none());
        // Null, NdefMessage, Terminator → 3 records
        assert_eq!(count, 3);
        assert_eq!(records[0].unwrap().tag, TlvTag::Null);
        assert_eq!(records[1].unwrap().tag, TlvTag::NdefMessage);
        assert_eq!(records[2].unwrap().tag, TlvTag::Terminator);
    }
}
