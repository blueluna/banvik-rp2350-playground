#![no_std]

use serde::{Deserialize, Serialize};

pub const UNKNOWN_UID: u64 = 0x0000_0000_0000_0000;

/// 8-bit status codes.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Status {
    Ok = 0,
    UnknownError = 1,
    TitleNotFound = 2,
    HashNotFound = 3,
    UidNotFound = 4,
}

#[cfg(feature = "defmt")]
impl defmt::Format for Status {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            Status::Ok => defmt::write!(fmt, "Ok"),
            Status::UnknownError => defmt::write!(fmt, "UnknownError"),
            Status::TitleNotFound => defmt::write!(fmt, "TitleNotFound"),
            Status::HashNotFound => defmt::write!(fmt, "HashNotFound"),
            Status::UidNotFound => defmt::write!(fmt, "UidNotFound"),
        }
    }
}

/// Client -> Server requests.
#[derive(Serialize, Deserialize, Debug)]
pub enum Request {
    List,
    QueryUid { uid: u64 },
    PlayUid { uid: u64 },
    PlayHash { hash: [u8; 32] },
    Stop,
}

/// A single song entry in the listing.
/// Title is a UTF-8 encoded string, up to 64 bytes.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SongEntry {
    pub uids: heapless::Vec<[u8; 8], 4>,
    pub hash: [u8; 32],
    pub title: heapless::Vec<u8, 256>,
}

impl Default for SongEntry {
    fn default() -> Self {
        Self {
            uids: heapless::Vec::new(),
            hash: [0u8; 32],
            title: heapless::Vec::new(),
        }
    }
}

impl SongEntry {
    /// Create a SongEntry, truncating the title to fit in 64 bytes.
    pub fn new(uids: &[u64], hash: [u8; 32], title: &str) -> Self {
        let mut uids_vec = heapless::Vec::new();
        let len = uids.len().min(4);
        for n in 0..len {
            let uid_bytes = uids[n].to_be_bytes();
            uids_vec.push(uid_bytes).ok();
        }
        let bytes = title.as_bytes();
        let len = bytes.len().min(256);
        let mut title_buf = heapless::Vec::new();
        title_buf.extend_from_slice(&bytes[..len]).ok();
        Self {
            uids: uids_vec,
            hash,
            title: title_buf,
        }
    }
}

/// Server -> Client responses.
#[derive(Serialize, Deserialize, Debug)]
pub enum Response {
    SongList {
        status: Status,
        songs: heapless::Vec<SongEntry, 128>,
    },
    Song {
        status: Status,
        song: SongEntry,
    },
    Play {
        status: Status,
        hash: [u8; 32],
        total_chunks: u32,
    },
    Stop {
        status: Status,
    },
}

/// Streaming chunk sent over UDP.
#[derive(Serialize, Deserialize, Debug)]
pub struct StreamChunk {
    pub status: Status,
    pub hash: [u8; 32],
    pub chunk_index: u32,
    pub total_chunks: u32,
    pub data: heapless::Vec<u8, 4096>,
}

