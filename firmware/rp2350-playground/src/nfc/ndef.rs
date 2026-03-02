/// NFC Data Exchange Format (NDEF) record decoder.
///
/// Decodes NDEF records from a byte slice according to the NFC Forum
/// NDEF specification (NFCForum-TS-NDEF_1.0).
use bitflags::bitflags;

/// Type Name Format field values (bits 2–0 of the header byte).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TypeNameFormat {
    Empty = 0x00,
    WellKnown = 0x01,
    MediaType = 0x02,
    AbsoluteUri = 0x03,
    ExternalType = 0x04,
    Unknown = 0x05,
    Unchanged = 0x06,
    Reserved = 0x07,
}

#[cfg(feature = "defmt")]
impl defmt::Format for TypeNameFormat {
    fn format(&self, f: defmt::Formatter<'_>) {
        match self {
            Self::Empty => defmt::write!(f, "Empty"),
            Self::WellKnown => defmt::write!(f, "WellKnown"),
            Self::MediaType => defmt::write!(f, "MediaType"),
            Self::AbsoluteUri => defmt::write!(f, "AbsoluteUri"),
            Self::ExternalType => defmt::write!(f, "ExternalType"),
            Self::Unknown => defmt::write!(f, "Unknown"),
            Self::Unchanged => defmt::write!(f, "Unchanged"),
            Self::Reserved => defmt::write!(f, "Reserved"),
        }
    }
}

impl TypeNameFormat {
    fn from_byte(b: u8) -> Self {
        match b & 0x07 {
            0x00 => Self::Empty,
            0x01 => Self::WellKnown,
            0x02 => Self::MediaType,
            0x03 => Self::AbsoluteUri,
            0x04 => Self::ExternalType,
            0x05 => Self::Unknown,
            0x06 => Self::Unchanged,
            _ => Self::Reserved,
        }
    }
}

/// Decoded NDEF record type, combining the TNF field with the type bytes.
///
/// Well-known RTD (Record Type Definition) types from TNF=0x01 are resolved
/// to named variants. All other combinations are represented as `Other`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecordType<'a> {
    /// Empty record — TNF=Empty.
    Empty,
    /// NFC RTD Text ("T") — TNF=WellKnown.
    Text,
    /// NFC RTD URI ("U") — TNF=WellKnown.
    Uri,
    /// NFC RTD Smart Poster ("Sp") — TNF=WellKnown.
    SmartPoster,
    /// NFC RTD Alternative Carrier ("ac") — TNF=WellKnown.
    AlternativeCarrier,
    /// NFC RTD Handover Carrier ("Hc") — TNF=WellKnown.
    HandoverCarrier,
    /// NFC RTD Handover Request ("Hr") — TNF=WellKnown.
    HandoverRequest,
    /// NFC RTD Handover Select ("Hs") — TNF=WellKnown.
    HandoverSelect,
    /// Unrecognised or non-well-known type. Contains the raw type bytes.
    Other(&'a [u8]),
}

impl<'a> RecordType<'a> {
    /// Resolve a record type from the TNF and raw type bytes.
    pub fn from_tnf_and_bytes(tnf: TypeNameFormat, type_bytes: &'a [u8]) -> Self {
        if tnf == TypeNameFormat::Empty {
            return Self::Empty;
        }
        if tnf == TypeNameFormat::WellKnown {
            match type_bytes {
                b"T" => Self::Text,
                b"U" => Self::Uri,
                b"Sp" => Self::SmartPoster,
                b"ac" => Self::AlternativeCarrier,
                b"Hc" => Self::HandoverCarrier,
                b"Hr" => Self::HandoverRequest,
                b"Hs" => Self::HandoverSelect,
                _ => Self::Other(type_bytes),
            }
        } else {
            Self::Other(type_bytes)
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for RecordType<'_> {
    fn format(&self, f: defmt::Formatter<'_>) {
        match self {
            Self::Empty => defmt::write!(f, "Empty"),
            Self::Text => defmt::write!(f, "Text"),
            Self::Uri => defmt::write!(f, "Uri"),
            Self::SmartPoster => defmt::write!(f, "SmartPoster"),
            Self::AlternativeCarrier => defmt::write!(f, "AlternativeCarrier"),
            Self::HandoverCarrier => defmt::write!(f, "HandoverCarrier"),
            Self::HandoverRequest => defmt::write!(f, "HandoverRequest"),
            Self::HandoverSelect => defmt::write!(f, "HandoverSelect"),
            Self::Other(bytes) => defmt::write!(f, "Other({=[u8]:#x})", bytes),
        }
    }
}

bitflags! {
    /// Header flags parsed from an NDEF record's first byte (bits 7–3).
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct NdefFlags: u8 {
        /// Message Begin — first record of an NDEF message.
        const MESSAGE_BEGIN = 0x80;
        /// Message End — last record of an NDEF message.
        const MESSAGE_END = 0x40;
        /// Chunk Flag — this record is a chunk of a larger payload.
        const CHUNK = 0x20;
        /// Short Record — payload length is encoded as a single byte.
        const SHORT_RECORD = 0x10;
        /// ID Length present — the ID length and ID fields are present.
        const ID_LENGTH = 0x08;
    }
}

/// A single decoded NDEF record, borrowing from the input slice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NdefRecord<'a> {
    pub flags: NdefFlags,
    /// Type Name Format.
    pub tnf: TypeNameFormat,
    /// Decoded record type.
    pub record_type: RecordType<'a>,
    /// Optional record ID.
    pub id: &'a [u8],
    /// Record payload.
    pub payload: &'a [u8],
}

/// Decoded text content from an NDEF Text record (RTD "T").
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NdefText<'a> {
    /// IANA language code (e.g. "en", "de").
    pub language: &'a str,
    /// The text content.
    pub text: &'a str,
}

/// Errors from decoding an NDEF Text record payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NdefTextError {
    /// The record type is not `RecordType::Text`.
    NotATextRecord,
    /// The payload is too short to contain the status byte.
    TruncatedPayload,
    /// The language code length exceeds the remaining payload.
    TruncatedLanguage,
    /// The text encoding is UTF-16, which is not supported.
    Utf16NotSupported,
    /// The language code is not valid UTF-8.
    InvalidLanguage,
    /// The text content is not valid UTF-8.
    InvalidText,
}

#[cfg(feature = "defmt")]
impl defmt::Format for NdefTextError {
    fn format(&self, f: defmt::Formatter<'_>) {
        match self {
            Self::NotATextRecord => defmt::write!(f, "NotATextRecord"),
            Self::TruncatedPayload => defmt::write!(f, "TruncatedPayload"),
            Self::TruncatedLanguage => defmt::write!(f, "TruncatedLanguage"),
            Self::Utf16NotSupported => defmt::write!(f, "Utf16NotSupported"),
            Self::InvalidLanguage => defmt::write!(f, "InvalidLanguage"),
            Self::InvalidText => defmt::write!(f, "InvalidText"),
        }
    }
}

impl<'a> NdefRecord<'a> {
    /// Decode the payload of an NDEF Text record (RTD "T").
    ///
    /// The text record payload layout is:
    /// - Byte 0: status byte (bit 7 = encoding, bits 5–0 = language code length)
    /// - Bytes 1..1+lang_len: IANA language code (e.g. "en")
    /// - Remaining bytes: the text content (UTF-8)
    ///
    /// Returns an error if this is not a text record, the encoding is UTF-16,
    /// or the payload is malformed.
    pub fn text(&self) -> Result<NdefText<'a>, NdefTextError> {
        if self.record_type != RecordType::Text {
            return Err(NdefTextError::NotATextRecord);
        }

        let payload = self.payload;
        if payload.is_empty() {
            return Err(NdefTextError::TruncatedPayload);
        }

        let status = payload[0];
        let is_utf16 = status & 0x80 != 0;
        if is_utf16 {
            return Err(NdefTextError::Utf16NotSupported);
        }

        let lang_len = (status & 0x3F) as usize;
        if 1 + lang_len > payload.len() {
            return Err(NdefTextError::TruncatedLanguage);
        }

        let language = core::str::from_utf8(&payload[1..1 + lang_len])
            .map_err(|_| NdefTextError::InvalidLanguage)?;
        let text = core::str::from_utf8(&payload[1 + lang_len..])
            .map_err(|_| NdefTextError::InvalidText)?;

        Ok(NdefText { language, text })
    }
}

/// Errors returned by the NDEF decoder.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NdefError {
    /// Input ended before the header byte could be read.
    TruncatedHeader,
    /// Input ended before all header fields could be read.
    TruncatedFields,
    /// Input ended before the full payload could be read.
    TruncatedPayload,
    /// Expected the MB flag on the first record but it was missing.
    MissingMessageBegin,
    /// Expected the ME flag on the last record but it was missing.
    MissingMessageEnd,
}

#[cfg(feature = "defmt")]
impl defmt::Format for NdefError {
    fn format(&self, f: defmt::Formatter<'_>) {
        match self {
            Self::TruncatedHeader => defmt::write!(f, "TruncatedHeader"),
            Self::TruncatedFields => defmt::write!(f, "TruncatedFields"),
            Self::TruncatedPayload => defmt::write!(f, "TruncatedPayload"),
            Self::MissingMessageBegin => defmt::write!(f, "MissingMessageBegin"),
            Self::MissingMessageEnd => defmt::write!(f, "MissingMessageEnd"),
        }
    }
}

/// Iterator over NDEF records within a single NDEF message byte slice.
///
/// # Example
///
/// ```rust
/// # use rp2350_playground::nfc::ndef::{NdefDecoder, RecordType};
/// // "Hello, World!" as an NDEF Text record (well-known type "T").
/// let ndef_bytes: &[u8] = &[
///     0xd1, 0x01, 0x10, 0x54, 0x02, 0x65, 0x6e, 0x48,
///     0x65, 0x6c, 0x6c, 0x6f, 0x2c, 0x20, 0x57, 0x6f,
///     0x72, 0x6c, 0x64, 0x21,
/// ];
///
/// let mut decoder = NdefDecoder::new(ndef_bytes);
/// while let Some(result) = decoder.next() {
///     let record = result.unwrap();
///     assert_eq!(record.record_type, RecordType::Text);
/// }
/// ```
pub struct NdefDecoder<'a> {
    buf: &'a [u8],
    pos: usize,
    finished: bool,
}

impl<'a> NdefDecoder<'a> {
    /// Create a new decoder over an NDEF message contained in `buf`.
    pub fn new(buf: &'a [u8]) -> Self {
        Self {
            buf,
            pos: 0,
            finished: false,
        }
    }

    /// Return the current byte offset into the buffer.
    pub fn position(&self) -> usize {
        self.pos
    }

    /// Advance to the next NDEF record.
    ///
    /// Returns `None` when all records have been consumed.
    /// Returns `Some(Err(_))` on a malformed record.
    pub fn next(&mut self) -> Option<Result<NdefRecord<'a>, NdefError>> {
        if self.finished || self.pos >= self.buf.len() {
            return None;
        }

        // --- Header byte ---
        let header = self.buf[self.pos];
        self.pos += 1;
        let flags = NdefFlags::from_bits_truncate(header & 0xF8);
        let tnf = TypeNameFormat::from_byte(header);

        // --- Type length ---
        let type_length = match self.read_byte() {
            Some(b) => b as usize,
            None => return Some(Err(NdefError::TruncatedFields)),
        };

        // --- Payload length (1 byte if SR, 4 bytes otherwise) ---
        let payload_length = if flags.contains(NdefFlags::SHORT_RECORD) {
            match self.read_byte() {
                Some(b) => b as usize,
                None => return Some(Err(NdefError::TruncatedFields)),
            }
        } else {
            let bytes = self.read_bytes(4);
            if bytes.len() < 4 {
                return Some(Err(NdefError::TruncatedFields));
            }
            u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]) as usize
        };

        // --- ID length (only if IL flag is set) ---
        let id_length = if flags.contains(NdefFlags::ID_LENGTH) {
            match self.read_byte() {
                Some(b) => b as usize,
                None => return Some(Err(NdefError::TruncatedFields)),
            }
        } else {
            0
        };

        // --- Type, ID, and Payload fields ---
        let total_needed = type_length + id_length + payload_length;
        if self.pos + total_needed > self.buf.len() {
            return Some(Err(NdefError::TruncatedPayload));
        }

        let type_bytes = &self.buf[self.pos..self.pos + type_length];
        self.pos += type_length;

        let id = &self.buf[self.pos..self.pos + id_length];
        self.pos += id_length;

        let payload = &self.buf[self.pos..self.pos + payload_length];
        self.pos += payload_length;

        let record_type = RecordType::from_tnf_and_bytes(tnf, type_bytes);

        if flags.contains(NdefFlags::MESSAGE_END) {
            self.finished = true;
        }

        Some(Ok(NdefRecord {
            flags,
            tnf,
            record_type,
            id,
            payload,
        }))
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

    fn read_bytes(&mut self, n: usize) -> &'a [u8] {
        let available = (self.buf.len() - self.pos).min(n);
        let slice = &self.buf[self.pos..self.pos + available];
        self.pos += available;
        slice
    }
}

/// Decode up to `N` NDEF records from `buf` into a stack-allocated array.
///
/// Returns `(records, count, error)` where `count` is the number of records
/// successfully decoded. Stops at the first error, the ME flag, or when
/// the array is full.
pub fn decode_ndef<'a, const N: usize>(
    buf: &'a [u8],
) -> ([Option<NdefRecord<'a>>; N], usize, Option<NdefError>) {
    const NONE: Option<NdefRecord<'static>> = None;
    let mut records: [Option<NdefRecord<'a>>; N] = [NONE; N];
    let mut decoder = NdefDecoder::new(buf);
    let mut count = 0;

    loop {
        if count == N {
            break;
        }
        match decoder.next() {
            None => break,
            Some(Err(e)) => return (records, count, Some(e)),
            Some(Ok(record)) => {
                records[count] = Some(record);
                count += 1;
            }
        }
    }

    (records, count, None)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// "Hello, World!" NDEF Text record:
    ///   MB=1 ME=1 CF=0 SR=1 IL=0 TNF=0x01 (well-known)
    ///   Type = "T"
    ///   Payload = 0x02 "en" "Hello, World!"
    const HELLO_WORLD_NDEF: &[u8] = &[
        0xd1, 0x01, 0x10, 0x54, 0x02, 0x65, 0x6e, 0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x2c, 0x20, 0x57,
        0x6f, 0x72, 0x6c, 0x64, 0x21,
    ];

    #[test]
    fn test_hello_world() {
        let mut decoder = NdefDecoder::new(HELLO_WORLD_NDEF);

        let record = decoder.next().unwrap().unwrap();
        assert!(record.flags.contains(NdefFlags::MESSAGE_BEGIN));
        assert!(record.flags.contains(NdefFlags::MESSAGE_END));
        assert!(record.flags.contains(NdefFlags::SHORT_RECORD));
        assert!(!record.flags.contains(NdefFlags::ID_LENGTH));
        assert!(!record.flags.contains(NdefFlags::CHUNK));
        assert_eq!(record.record_type, RecordType::Text);
        assert!(record.id.is_empty());
        // Payload: status byte (0x02) + "en" + "Hello, World!"
        assert_eq!(record.payload.len(), 16);
        assert_eq!(&record.payload[3..], b"Hello, World!");

        // Should be exhausted (ME was set).
        assert!(decoder.next().is_none());
    }

    #[test]
    fn test_multi_record_message() {
        // Two records: first has MB=1 ME=0, second has MB=0 ME=1.
        // Both are SR=1, TNF=WellKnown, type="T", payload="AB".
        let buf: &[u8] = &[
            // Record 1: MB=1 ME=0 SR=1 TNF=1 → 0b1001_0001 = 0x91
            0x91, 0x01, 0x02, 0x54, 0x41, 0x42,
            // Record 2: MB=0 ME=1 SR=1 TNF=1 → 0b0101_0001 = 0x51
            0x51, 0x01, 0x02, 0x54, 0x43, 0x44,
        ];
        let mut decoder = NdefDecoder::new(buf);

        let r1 = decoder.next().unwrap().unwrap();
        assert!(r1.flags.contains(NdefFlags::MESSAGE_BEGIN));
        assert!(!r1.flags.contains(NdefFlags::MESSAGE_END));
        assert_eq!(r1.payload, b"AB");

        let r2 = decoder.next().unwrap().unwrap();
        assert!(!r2.flags.contains(NdefFlags::MESSAGE_BEGIN));
        assert!(r2.flags.contains(NdefFlags::MESSAGE_END));
        assert_eq!(r2.payload, b"CD");

        assert!(decoder.next().is_none());
    }

    #[test]
    fn test_long_payload() {
        // Non-short record (SR=0): 4-byte payload length.
        // MB=1 ME=1 SR=0 TNF=1 → 0b1100_0001 = 0xC1
        let mut buf = vec![0xC1, 0x01, 0x00, 0x00, 0x01, 0x00, 0x54];
        // Payload of 256 bytes (0x00000100).
        buf.extend_from_slice(&[0xAA; 256]);

        let mut decoder = NdefDecoder::new(&buf);
        let record = decoder.next().unwrap().unwrap();
        assert!(!record.flags.contains(NdefFlags::SHORT_RECORD));
        assert_eq!(record.payload.len(), 256);
        assert!(record.payload.iter().all(|&b| b == 0xAA));
    }

    #[test]
    fn test_id_field() {
        // MB=1 ME=1 SR=1 IL=1 TNF=1 → 0b1101_1001 = 0xD9
        let buf: &[u8] = &[
            0xD9, 0x01, 0x02, 0x03, // header, type_len=1, payload_len=2, id_len=3
            0x54, // type = "T"
            0x01, 0x02, 0x03, // id
            0xAA, 0xBB, // payload
        ];
        let mut decoder = NdefDecoder::new(buf);
        let record = decoder.next().unwrap().unwrap();
        assert!(record.flags.contains(NdefFlags::ID_LENGTH));
        assert_eq!(record.record_type, RecordType::Text);
        assert_eq!(record.id, &[0x01, 0x02, 0x03]);
        assert_eq!(record.payload, &[0xAA, 0xBB]);
    }

    #[test]
    fn test_empty_record() {
        // MB=1 ME=1 SR=1 TNF=0 (empty) → 0b1101_0000 = 0xD0
        // Type length and payload length both 0.
        let buf: &[u8] = &[0xD0, 0x00, 0x00];
        let mut decoder = NdefDecoder::new(buf);
        let record = decoder.next().unwrap().unwrap();
        assert_eq!(record.tnf, TypeNameFormat::Empty);
        assert_eq!(record.record_type, RecordType::Empty);
        assert!(record.payload.is_empty());
    }

    #[test]
    fn test_truncated_header() {
        let buf: &[u8] = &[0xD1];
        let mut decoder = NdefDecoder::new(buf);
        assert_eq!(decoder.next().unwrap(), Err(NdefError::TruncatedFields));
    }

    #[test]
    fn test_truncated_payload() {
        // Claims payload of 10 bytes but only 2 follow.
        let buf: &[u8] = &[0xD1, 0x01, 0x0A, 0x54, 0xAA, 0xBB];
        let mut decoder = NdefDecoder::new(buf);
        assert_eq!(decoder.next().unwrap(), Err(NdefError::TruncatedPayload));
    }

    #[test]
    fn test_text_hello_world() {
        let mut decoder = NdefDecoder::new(HELLO_WORLD_NDEF);
        let record = decoder.next().unwrap().unwrap();
        let text = record.text().unwrap();
        assert_eq!(text.language, "en");
        assert_eq!(text.text, "Hello, World!");
    }

    #[test]
    fn test_text_not_a_text_record() {
        // Empty record
        let buf: &[u8] = &[0xD0, 0x00, 0x00];
        let mut decoder = NdefDecoder::new(buf);
        let record = decoder.next().unwrap().unwrap();
        assert_eq!(record.text(), Err(NdefTextError::NotATextRecord));
    }

    #[test]
    fn test_text_utf16_rejected() {
        // Text record with UTF-16 flag set (status byte = 0x82 → UTF-16, lang_len=2)
        // MB=1 ME=1 SR=1 TNF=1 = 0xD1, type_len=1, payload_len=5
        let buf: &[u8] = &[0xD1, 0x01, 0x05, 0x54, 0x82, 0x65, 0x6e, 0x00, 0x41];
        let mut decoder = NdefDecoder::new(buf);
        let record = decoder.next().unwrap().unwrap();
        assert_eq!(record.text(), Err(NdefTextError::Utf16NotSupported));
    }

    #[test]
    fn test_decode_ndef_helper() {
        let (records, count, err) = decode_ndef::<4>(HELLO_WORLD_NDEF);
        assert!(err.is_none());
        assert_eq!(count, 1);
        let r = records[0].unwrap();
        assert_eq!(r.record_type, RecordType::Text);
    }
}
