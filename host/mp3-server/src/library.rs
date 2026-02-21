use std::collections::HashMap;
use std::io;
use std::path::{Path, PathBuf};

use id3::TagLike;

use crate::protocol::SongEntry;
use crate::transcode;

pub struct SongInfo {
    pub hash: u32,
    pub title: String,
    pub source_path: PathBuf,
}

pub struct Library {
    songs: HashMap<u32, SongInfo>,
    source_dir: PathBuf,
    cache_dir: PathBuf,
}

impl Library {
    /// Scan a directory for MP3 files, read ID3 titles, compute CRC32 hashes,
    /// and eagerly transcode all files into the cache.
    pub fn scan_directory(dir: &Path) -> io::Result<Self> {
        let cache_dir = dir.join(".mp3cache");
        std::fs::create_dir_all(&cache_dir)?;

        let mut library = Library {
            songs: HashMap::new(),
            source_dir: dir.to_path_buf(),
            cache_dir,
        };

        library.rescan()?;
        Ok(library)
    }

    /// Rescan the source directory, adding any new MP3 files and transcoding them.
    pub fn rescan(&mut self) -> io::Result<()> {
        for entry in std::fs::read_dir(&self.source_dir)? {
            let entry = entry?;
            let path = entry.path();

            if !path.is_file()
            {
                continue;
            }

            let contents = std::fs::read(&path)?;
            let hash = crc32fast::hash(&contents);

            if self.songs.contains_key(&hash) {
                continue;
            }

            let title = read_title(&path).unwrap_or_else(|| {
                path.file_stem()
                    .unwrap_or_default()
                    .to_string_lossy()
                    .into_owned()
            });

            // Eagerly transcode into cache
            let cached_path = self.cache_dir.join(format!("{:08x}.mp3", hash));
            if !cached_path.exists() {
                println!("  Transcoding: {}", title);
                match transcode::transcode(&path) {
                    Ok(transcoded) => {
                        std::fs::write(&cached_path, &transcoded).map_err(|e| {
                            io::Error::new(
                                io::ErrorKind::Other,
                                format!("failed to write cache for {title}: {e}"),
                            )
                        })?;
                        println!("    -> {} bytes", transcoded.len());
                    }
                    Err(e) => {
                        eprintln!("  Failed to transcode {}: {}", title, e);
                        continue;
                    }
                }
            }

            println!("  {:08x}  {}", hash, title);
            self.songs.insert(
                hash,
                SongInfo {
                    hash,
                    title,
                    source_path: path,
                },
            );
        }

        println!("Library: {} song(s)", self.songs.len());
        Ok(())
    }

    /// Return the list of songs as protocol entries.
    pub fn list_songs(&self) -> Vec<SongEntry> {
        self.songs
            .values()
            .map(|s| SongEntry::new(s.hash, &s.title))
            .collect()
    }

    /// Get the transcoded MP3 data for a song from the cache.
    pub fn get_transcoded(&self, hash: u32) -> Result<Vec<u8>, ()> {
        let song = self.songs.get(&hash).ok_or(())?;
        let cached_path = self.cache_dir.join(format!("{:08x}.mp3", hash));

        if cached_path.exists() {
            return std::fs::read(&cached_path).map_err(|e| {
                eprintln!("Failed to read cache for {:08x}: {}", hash, e);
            });
        }

        // Shouldn't happen since we transcode eagerly, but handle it
        println!("Cache miss for {:08x}, transcoding...", hash);
        match transcode::transcode(&song.source_path) {
            Ok(transcoded) => {
                let _ = std::fs::write(&cached_path, &transcoded);
                Ok(transcoded)
            }
            Err(e) => {
                eprintln!("Failed to transcode {:08x}: {}", hash, e);
                Err(())
            }
        }
    }
}

fn read_title(path: &Path) -> Option<String> {
    let title = match id3::Tag::read_from_path(path) {
        Ok(tag) => {
            let title = tag.title().unwrap_or("<none>");
            let artist = tag.artist().unwrap_or("<none>");
            let album = tag.album().unwrap_or("<none>");
            println!(
                "  Found ID3 tag for {}: title={}, artist={}, album={}",
                path.display(),
                title,
                artist,
                album
            );
            tag.title().map(|s| s.to_owned())
        }
        Err(_) => None,
    };
    if title.is_some() {
        return title;
    }
    match mp4ameta::Tag::read_from_path(path) {
        Ok(meta) => {
            let title = meta.title().unwrap_or("<none>");
            let artist = meta.artist().unwrap_or("<none>");
            let album = meta.album().unwrap_or("<none>");
            println!(
                "  Found MP4 metadata for {}: title={}, artist={}, album={}",
                path.display(),
                title,
                artist,
                album
            );
            Some(title.to_owned())
        }
        Err(_) => None,
    }
}
