#![no_std]

use serde::{Deserialize, Serialize};

/// 8-bit status codes.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Status {
    Ok = 0,
    UnknownError = 1,
    TitleNotFound = 2,
}

/// Client -> Server requests.
#[derive(Serialize, Deserialize, Debug)]
pub enum Request {
    List,
    Play { hash: u32 },
    Stop,
}

/// A single song entry in the listing.
/// Title is a UTF-8 encoded string, up to 64 bytes.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SongEntry {
    pub hash: u32,
    pub title: heapless::Vec<u8, 64>,
}

impl SongEntry {
    /// Create a SongEntry, truncating the title to fit in 64 bytes.
    pub fn new(hash: u32, title: &str) -> Self {
        let bytes = title.as_bytes();
        let len = bytes.len().min(64);
        let mut title_buf = heapless::Vec::new();
        title_buf.extend_from_slice(&bytes[..len]).ok();
        Self {
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
    Chunk {
        status: Status,
        hash: u32,
        chunk_index: u32,
        total_chunks: u32,
        data: heapless::Vec<u8, 4096>,
    },
    Stop {
        status: Status,
    },
}
