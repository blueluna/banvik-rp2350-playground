/// Extracted metadata pointing directly to slices in the original file buffer
#[derive(Default, Debug, Clone)]
pub struct TagInfo<'a> {
    pub title: Option<&'a [u8]>,
    pub artist: Option<&'a [u8]>,
    pub album: Option<&'a [u8]>,
    pub year: Option<&'a [u8]>,
    pub track: Option<&'a [u8]>,
}

pub enum Id3Detection<'a> {
    /// ID3v2 tag detected. Contains the audio offset and extracted tags.
    Id3v2 {
        audio_start_offset: usize,
        tags: TagInfo<'a>,
    },
    /// ID3v1 tag detected at the end of the file.
    Id3v1 { tags: TagInfo<'a> },
    None,
}

#[derive(Debug, Clone, Copy)]
pub struct OwnedTagField<const N: usize> {
    data: [u8; N],
    len: usize,
    present: bool,
    truncated: bool,
}

impl<const N: usize> Default for OwnedTagField<N> {
    fn default() -> Self {
        Self {
            data: [0u8; N],
            len: 0,
            present: false,
            truncated: false,
        }
    }
}

impl<const N: usize> OwnedTagField<N> {
    fn clear(&mut self) {
        self.len = 0;
        self.present = false;
        self.truncated = false;
    }

    fn push(&mut self, byte: u8) {
        self.present = true;
        if self.len < N {
            self.data[self.len] = byte;
            self.len += 1;
        } else {
            self.truncated = true;
        }
    }

    pub fn as_slice(&self) -> Option<&[u8]> {
        if self.present {
            Some(&self.data[..self.len])
        } else {
            None
        }
    }

    pub fn was_truncated(&self) -> bool {
        self.truncated
    }
}

#[derive(Default, Debug, Clone, Copy)]
pub struct StreamingTagInfo<const N: usize = 96> {
    pub title: OwnedTagField<N>,
    pub artist: OwnedTagField<N>,
    pub album: OwnedTagField<N>,
    pub year: OwnedTagField<N>,
    pub track: OwnedTagField<N>,
}

impl<const N: usize> StreamingTagInfo<N> {
    pub fn as_tag_info(&self) -> TagInfo<'_> {
        TagInfo {
            title: self.title.as_slice(),
            artist: self.artist.as_slice(),
            album: self.album.as_slice(),
            year: self.year.as_slice(),
            track: self.track.as_slice(),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Id3v2Header {
    pub version: u8,
    pub flags: u8,
    pub tag_size: usize,
    pub total_size: usize,
}

#[derive(Debug, Clone, Copy)]
enum FrameTarget {
    Title,
    Artist,
    Album,
    Year,
    Track,
}

fn frame_target_from_id(version: u8, frame_id: &[u8]) -> Option<FrameTarget> {
    if version == 2 {
        match frame_id {
            b"TT2" => Some(FrameTarget::Title),
            b"TP1" => Some(FrameTarget::Artist),
            b"TAL" => Some(FrameTarget::Album),
            b"TYE" => Some(FrameTarget::Year),
            b"TRK" => Some(FrameTarget::Track),
            _ => None,
        }
    } else {
        match frame_id {
            b"TIT2" => Some(FrameTarget::Title),
            b"TPE1" => Some(FrameTarget::Artist),
            b"TALB" => Some(FrameTarget::Album),
            b"TYER" | b"TDRC" => Some(FrameTarget::Year),
            b"TRCK" => Some(FrameTarget::Track),
            _ => None,
        }
    }
}

fn assign_id3v2_text_field<'a>(tags: &mut TagInfo<'a>, target: FrameTarget, payload: &'a [u8]) {
    let text = extract_text_payload(payload);
    match target {
        FrameTarget::Title => tags.title = text,
        FrameTarget::Artist => tags.artist = text,
        FrameTarget::Album => tags.album = text,
        FrameTarget::Year => tags.year = text,
        FrameTarget::Track => tags.track = text,
    }
}

/// Parses a 10-byte ID3v2 header.
pub fn parse_id3v2_header(file_start: &[u8]) -> Option<Id3v2Header> {
    if file_start.len() < 10 || &file_start[0..3] != b"ID3" {
        return None;
    }

    let version = file_start[3];
    let flags = file_start[5];

    let mut size_bytes = [0u8; 4];
    size_bytes.copy_from_slice(&file_start[6..10]);
    let tag_size = decode_syncsafe_int(&size_bytes);

    let has_footer = version >= 4 && (flags & 0x10) != 0;
    let total_size = tag_size + 10 + if has_footer { 10 } else { 0 };

    Some(Id3v2Header {
        version,
        flags,
        tag_size,
        total_size,
    })
}

/// Incremental ID3v2 parser that consumes bytes from the start of a file.
///
/// `feed` returns the number of bytes consumed from the provided chunk.
/// If the returned value is smaller than the chunk length, the remaining bytes
/// are audio payload that can be decoded immediately.
pub struct Id3v2StreamParser<const N: usize = 96> {
    version: u8,
    remaining_tag_body: usize,
    parsing_extended_header: bool,
    remaining_extended_header: usize,
    extended_header_size_bytes: [u8; 4],
    extended_header_size_len: usize,
    current_frame_remaining: usize,
    current_frame_header: [u8; 10],
    current_frame_header_len: usize,
    current_target: Option<FrameTarget>,
    skip_encoding_byte: bool,
    complete: bool,
    tags: StreamingTagInfo<N>,
}

impl<const N: usize> Id3v2StreamParser<N> {
    pub fn new(header: Id3v2Header) -> Self {
        Self {
            version: header.version,
            remaining_tag_body: header.tag_size,
            parsing_extended_header: header.version >= 3 && (header.flags & 0x40) != 0,
            remaining_extended_header: 0,
            extended_header_size_bytes: [0u8; 4],
            extended_header_size_len: 0,
            current_frame_remaining: 0,
            current_frame_header: [0u8; 10],
            current_frame_header_len: 0,
            current_target: None,
            skip_encoding_byte: false,
            complete: header.tag_size == 0,
            tags: StreamingTagInfo::default(),
        }
    }

    pub fn is_complete(&self) -> bool {
        self.complete
    }

    pub fn tags(&self) -> &StreamingTagInfo<N> {
        &self.tags
    }

    pub fn has_primary_text_fields(&self) -> bool {
        self.tags.title.as_slice().is_some()
            && self.tags.artist.as_slice().is_some()
            && self.tags.album.as_slice().is_some()
            && self.tags.year.as_slice().is_some()
    }

    pub fn feed(&mut self, chunk: &[u8]) -> usize {
        if self.complete {
            return 0;
        }

        let mut offset = 0;

        while offset < chunk.len() && !self.complete {
            if self.parsing_extended_header {
                offset += self.consume_extended_header(&chunk[offset..]);
                continue;
            }

            if self.current_frame_remaining > 0 {
                offset += self.consume_frame_payload(&chunk[offset..]);
                continue;
            }

            if self.remaining_tag_body == 0 {
                self.complete = true;
                break;
            }

            let frame_header_len = self.frame_header_len();
            let needed = (frame_header_len - self.current_frame_header_len)
                .min(self.remaining_tag_body)
                .min(chunk.len() - offset);

            self.current_frame_header[self.current_frame_header_len..self.current_frame_header_len + needed]
                .copy_from_slice(&chunk[offset..offset + needed]);
            self.current_frame_header_len += needed;
            offset += needed;

            if self.current_frame_header_len < frame_header_len {
                continue;
            }

            self.remaining_tag_body = self.remaining_tag_body.saturating_sub(frame_header_len);

            if self.current_frame_header[0] == 0x00 {
                self.complete = true;
                break;
            }

            let frame_size = self.decode_frame_size();

            if frame_size > self.remaining_tag_body {
                self.complete = true;
                break;
            }

            self.current_frame_remaining = frame_size;
            self.current_target = self.decode_frame_target();
            self.skip_encoding_byte = self.current_target.is_some() && self.current_frame_remaining > 0;

            if let Some(target) = self.current_target {
                self.field_mut(target).clear();
            }

            self.current_frame_header_len = 0;

            if self.current_frame_remaining == 0 {
                self.finish_frame();
            }
        }

        if self.remaining_tag_body == 0 && self.current_frame_remaining == 0 {
            self.complete = true;
        }

        offset
    }

    fn consume_extended_header(&mut self, bytes: &[u8]) -> usize {
        if self.remaining_extended_header > 0 {
            let take = self.remaining_extended_header.min(bytes.len());
            self.remaining_extended_header -= take;
            self.remaining_tag_body = self.remaining_tag_body.saturating_sub(take);

            if self.remaining_extended_header == 0 {
                self.parsing_extended_header = false;
            }
            return take;
        }

        let needed = (4 - self.extended_header_size_len)
            .min(self.remaining_tag_body)
            .min(bytes.len());
        self.extended_header_size_bytes
            [self.extended_header_size_len..self.extended_header_size_len + needed]
            .copy_from_slice(&bytes[..needed]);
        self.extended_header_size_len += needed;

        if self.extended_header_size_len < 4 {
            return needed;
        }

        self.remaining_tag_body = self.remaining_tag_body.saturating_sub(4);

        let ext_total_size = if self.version >= 4 {
            decode_syncsafe_int(&self.extended_header_size_bytes)
        } else {
            decode_u32_be(&self.extended_header_size_bytes).saturating_add(4)
        };

        let ext_payload_size = ext_total_size
            .saturating_sub(4)
            .min(self.remaining_tag_body);

        self.remaining_extended_header = ext_payload_size;

        if self.remaining_extended_header == 0 {
            self.parsing_extended_header = false;
        }

        needed
    }

    fn consume_frame_payload(&mut self, bytes: &[u8]) -> usize {
        let take = self.current_frame_remaining.min(bytes.len());

        if let Some(target) = self.current_target {
            let mut skip_encoding_byte = self.skip_encoding_byte;
            for byte in &bytes[..take] {
                if skip_encoding_byte {
                    skip_encoding_byte = false;
                    continue;
                }
                self.push_target_byte(target, *byte);
            }
            self.skip_encoding_byte = skip_encoding_byte;
        }

        self.current_frame_remaining -= take;
        self.remaining_tag_body = self.remaining_tag_body.saturating_sub(take);

        if self.current_frame_remaining == 0 {
            self.finish_frame();
        }

        take
    }

    fn finish_frame(&mut self) {
        self.current_target = None;
        self.skip_encoding_byte = false;
    }

    fn field_mut(&mut self, target: FrameTarget) -> &mut OwnedTagField<N> {
        match target {
            FrameTarget::Title => &mut self.tags.title,
            FrameTarget::Artist => &mut self.tags.artist,
            FrameTarget::Album => &mut self.tags.album,
            FrameTarget::Year => &mut self.tags.year,
            FrameTarget::Track => &mut self.tags.track,
        }
    }

    fn push_target_byte(&mut self, target: FrameTarget, byte: u8) {
        self.field_mut(target).push(byte);
    }

    fn frame_header_len(&self) -> usize {
        if self.version == 2 {
            6
        } else {
            10
        }
    }

    fn decode_frame_size(&self) -> usize {
        if self.version == 2 {
            decode_u24_be(
                self.current_frame_header[3],
                self.current_frame_header[4],
                self.current_frame_header[5],
            )
        } else {
            let mut size_bytes = [0u8; 4];
            size_bytes.copy_from_slice(&self.current_frame_header[4..8]);
            if self.version >= 4 {
                decode_syncsafe_int(&size_bytes)
            } else {
                decode_u32_be(&size_bytes)
            }
        }
    }

    fn decode_frame_target(&self) -> Option<FrameTarget> {
        if self.version == 2 {
            frame_target_from_id(self.version, &self.current_frame_header[0..3])
        } else {
            frame_target_from_id(self.version, &self.current_frame_header[0..4])
        }
    }
}

/// Decodes an ID3v2 syncsafe integer (used for headers and v2.4 frame sizes)
fn decode_syncsafe_int(bytes: &[u8; 4]) -> usize {
    ((bytes[0] as usize & 0x7F) << 21)
        | ((bytes[1] as usize & 0x7F) << 14)
        | ((bytes[2] as usize & 0x7F) << 7)
        | (bytes[3] as usize & 0x7F)
}

/// Decodes a standard big-endian u32 (used for ID3v2.3 frame sizes)
fn decode_u32_be(bytes: &[u8; 4]) -> usize {
    ((bytes[0] as usize) << 24)
        | ((bytes[1] as usize) << 16)
        | ((bytes[2] as usize) << 8)
        | (bytes[3] as usize)
}

/// Decodes a standard big-endian 24-bit integer (used for ID3v2.2 frame sizes)
fn decode_u24_be(b0: u8, b1: u8, b2: u8) -> usize {
    ((b0 as usize) << 16) | ((b1 as usize) << 8) | (b2 as usize)
}

/// Strips the trailing null bytes or spaces often found in fixed-length buffers
fn trim_padding(bytes: &[u8]) -> &[u8] {
    let mut len = bytes.len();
    while len > 0 && (bytes[len - 1] == 0x00 || bytes[len - 1] == 0x20) {
        len -= 1;
    }
    &bytes[..len]
}

/// Extracts text payload from an ID3v2 frame (skips the encoding byte)
fn extract_text_payload(frame_data: &[u8]) -> Option<&[u8]> {
    if frame_data.len() > 1 {
        // byte 0 is the text encoding (e.g., 0=ISO-8859-1, 1=UTF-16, 3=UTF-8)
        Some(&frame_data[1..])
    } else {
        None
    }
}

/// Parses ID3v2 frames sequentially out of the tag body
fn parse_id3v2_frames(tag_body: &[u8], version: u8) -> TagInfo<'_> {
    let mut tags = TagInfo::default();
    let mut offset = 0;
    let frame_header_len = if version == 2 { 6 } else { 10 };

    while offset + frame_header_len <= tag_body.len() {
        let frame_id = if version == 2 {
            &tag_body[offset..offset + 3]
        } else {
            &tag_body[offset..offset + 4]
        };

        // A null byte frame ID indicates padding. Stop parsing.
        if frame_id[0] == 0x00 {
            break;
        }

        let frame_size = if version == 2 {
            decode_u24_be(
                tag_body[offset + 3],
                tag_body[offset + 4],
                tag_body[offset + 5],
            )
        } else {
            let mut size_bytes = [0u8; 4];
            size_bytes.copy_from_slice(&tag_body[offset + 4..offset + 8]);
            // ID3v2.4 uses syncsafe frame sizes, v2.3 uses standard 32-bit integers
            if version >= 4 {
                decode_syncsafe_int(&size_bytes)
            } else {
                decode_u32_be(&size_bytes)
            }
        };

        let frame_start = offset + frame_header_len;
        let frame_end = frame_start + frame_size;

        if frame_end > tag_body.len() {
            break; // Truncated frame
        }

        let payload = &tag_body[frame_start..frame_end];

        if let Some(target) = frame_target_from_id(version, frame_id) {
            assign_id3v2_text_field(&mut tags, target, payload);
        }

        offset = frame_end;
    }

    tags
}

/// Scans for ID3 tags and extracts metadata slices.
/// Note: To parse ID3v2 tags, `file_start` must contain the full ID3 tag,
/// otherwise frames will be cut off.
pub fn detect_id3_tags<'a>(file_start: &'a [u8], file_end: Option<&'a [u8]>) -> Id3Detection<'a> {
    // 1. Check for ID3v2 at the beginning of the file
    if let Some(header) = parse_id3v2_header(file_start) {
        let version = header.version;
        let tag_size = header.tag_size;

        let mut tags = TagInfo::default();

        // If the buffer contains the full tag, we can parse it
        if file_start.len() >= tag_size + 10 {
            tags = parse_id3v2_frames(&file_start[10..tag_size + 10], version);
        }

        return Id3Detection::Id3v2 {
            audio_start_offset: header.total_size,
            tags,
        };
    }

    // 2. Check for ID3v1 at the end of the file
    if let Some(end_bytes) = file_end {
        if end_bytes.len() >= 128
            && &end_bytes[end_bytes.len() - 128..end_bytes.len() - 125] == b"TAG"
        {
            let base = end_bytes.len() - 128;

            let mut tags = TagInfo {
                title: Some(trim_padding(&end_bytes[base + 3..base + 33])),
                artist: Some(trim_padding(&end_bytes[base + 33..base + 63])),
                album: Some(trim_padding(&end_bytes[base + 63..base + 93])),
                year: Some(trim_padding(&end_bytes[base + 93..base + 97])),
                track: None,
            };

            // ID3v1.1 stores the track number in byte 126 if byte 125 is null
            if end_bytes[base + 125] == 0 && end_bytes[base + 126] != 0 {
                tags.track = Some(&end_bytes[base + 126..base + 127]);
            }

            return Id3Detection::Id3v1 { tags };
        }
    }

    Id3Detection::None
}
