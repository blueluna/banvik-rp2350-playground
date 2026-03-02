use std::collections::HashMap;
use std::env;
use std::io;
use std::path::{Path, PathBuf};

use directories::ProjectDirs;
use id3::TagLike;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use crate::protocol::SongEntry;
use crate::transcode;

#[derive(Serialize, Deserialize, Debug, Clone, Copy, Eq, PartialEq, Hash)]
pub struct Sha256Hash(pub [u8; 32]);

impl Sha256Hash {
    pub fn to_hex(&self) -> String {
        self.0.iter().map(|b| format!("{:02x}", b)).collect()
    }
}

impl From<[u8; 32]> for Sha256Hash {
    fn from(bytes: [u8; 32]) -> Self {
        Sha256Hash(bytes)
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SongMapping {
    pub hash: Sha256Hash,
    pub uids: Vec<[u8; 8]>,
    pub title: String,
}

/// A simple container for the TOML file that holds a list of mappings.
#[derive(Serialize, Deserialize, Default, Debug)]
pub struct SongMappingFile {
    pub mappings: HashMap<String, SongMapping>,
}

impl SongMappingFile {
    /// Load the mapping file from `override_path` if provided, otherwise
    /// consult the `SONG_MAPPING_PATH` environment variable and finally
    /// fall back to `$XDG_CONFIG_HOME/mp3-server/song-mappings.toml`.
    pub fn load(override_path: Option<&Path>) -> io::Result<Self> {
        let path = Self::config_path(override_path)?;
        if path.exists() {
            let contents = std::fs::read_to_string(&path)?;
            toml::from_str(&contents)
                .map_err(|e| io::Error::new(io::ErrorKind::Other, e.to_string()))
        } else {
            Ok(Self::default())
        }
    }

    /// Persist the mapping file back to disk using the same path lookup
    /// rules as `load`.
    pub fn save(&self, override_path: Option<&Path>) -> io::Result<()> {
        let path = Self::config_path(override_path)?;
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let toml_str = toml::to_string(self)
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e.to_string()))?;
        std::fs::write(&path, toml_str)?;
        Ok(())
    }

    fn config_path(override_path: Option<&Path>) -> io::Result<PathBuf> {
        if let Some(p) = override_path {
            return Ok(p.to_path_buf());
        }
        if let Ok(env_path) = env::var("SONG_MAPPING_PATH") {
            return Ok(PathBuf::from(env_path));
        }

        let proj = ProjectDirs::from("com", "example", "mp3-server").ok_or_else(|| {
            io::Error::new(io::ErrorKind::Other, "could not determine config directory")
        })?;
        let config_dir = proj.config_dir();
        std::fs::create_dir_all(config_dir)?;
        Ok(config_dir.join("song-mappings.toml"))
    }
}

pub struct SongInfo {
    pub hash: Sha256Hash,
    pub uids: Vec<u64>,
    pub title: String,
    pub source_path: PathBuf,
}

pub struct Library {
    songs: HashMap<Sha256Hash, SongInfo>,
    source_dir: PathBuf,
    cache_dir: PathBuf,
    mappings: SongMappingFile,
}

impl Library {
    /// Create a library from `dir`.
    ///
    /// The mapping file is loaded with the normal lookup logic (no override)
    /// and any hashes contained in it are applied to the entries that are
    /// discovered when the directory is scanned.  This preserves previously
    ///‑assigned UID/title information across restarts.
    pub fn from_path(dir: &Path) -> io::Result<Self> {
        // load whatever mappings we already have; if the file doesn’t exist
        // we just get an empty default.
        let mappings = SongMappingFile::load(None)?;

        let cache_dir = dir.join(".stream-cache");
        std::fs::create_dir_all(&cache_dir)?;

        let mut library = Library {
            songs: HashMap::new(),
            source_dir: dir.to_path_buf(),
            cache_dir,
            mappings,
        };

        library.rescan()?;
        Ok(library)
    }

    pub fn lookup_hash_from_uid(&self, uid: u64) -> Option<Sha256Hash> {
        for (_, mapping) in &self.mappings.mappings {
            if mapping.uids.iter().any(|u| u64::from_be_bytes(*u) == uid) {
                return Some(mapping.hash);
            }
        }
        None
    }

    pub fn lookup_uids_from_hash(&self, hash: Sha256Hash) -> Option<Vec<u64>> {
        let key = hash.to_hex();
        self.mappings
            .mappings
            .get(&key)
            .map(|m| m.uids.iter().map(|uid| u64::from_be_bytes(*uid)).collect())
    }

    /// Rescan the source directory, adding any new MP3 files and transcoding them.
    pub fn rescan(&mut self) -> io::Result<()> {
        self.mappings = SongMappingFile::load(None)?;

        for entry in std::fs::read_dir(&self.source_dir)? {
            let entry = entry?;
            let path = entry.path();

            if !path.is_file() {
                continue;
            }

            let contents = std::fs::read(&path)?;
            let digest = Sha256::digest(&contents);
            let hash: [u8; 32] = digest.into();
            let hash = Sha256Hash::from(hash);

            let title = match read_title(&path) {
                Some(media_information) => {
                    format!(
                        "{} - {} - {}",
                        media_information.artist, media_information.album, media_information.title
                    )
                }
                None => {
                    println!(
                        "  No metadata found for {}, using filename as title",
                        path.display()
                    );
                    path.file_stem()
                        .unwrap_or_default()
                        .to_string_lossy()
                        .into_owned()
                }
            };

            let uids = if let Some(uids) = self.lookup_uids_from_hash(hash) {
                uids
            } else {
                let uids = vec![mp3_protocol::UNKNOWN_UID];
                self.mappings.mappings.insert(
                    hash.to_hex(),
                    SongMapping {
                        hash,
                        uids: uids.iter().map(|uid| uid.to_be_bytes()).collect(),
                        title: title.clone(),
                    },
                );
                uids
            };

            if let Some(entry) = self.songs.get_mut(&hash) {
                // This hash already exists in the library, so just update the UID if needed.
                entry.uids = uids;
                continue;
            }

            // Eagerly transcode into cache
            let cached_path = self.cache_dir.join(format!("{}.mp3", hash.to_hex()));
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

            self.songs.insert(
                hash,
                SongInfo {
                    hash,
                    uids,
                    title,
                    source_path: path,
                },
            );
        }

        self.mappings.save(None)?;

        println!("Library: {} song(s)", self.songs.len());
        Ok(())
    }

    /// Return the list of songs as protocol entries.
    pub fn list_songs(&self) -> Vec<SongEntry> {
        self.songs
            .values()
            .map(|s| SongEntry::new(s.uids.as_ref(), s.hash.0, &s.title))
            .collect()
    }

    pub fn get_song(&self, hash: Sha256Hash) -> Option<SongEntry> {
        self.songs.get(&hash).map(|s| SongEntry::new(s.uids.as_ref(), s.hash.0, &s.title))
    }

    /// Get the transcoded MP3 data for a song from the cache.
    pub fn get_transcoded_by_hash(&self, hash: Sha256Hash) -> Result<(Vec<u8>, Sha256Hash), ()> {
        let hash_str = hash.to_hex();
        let song = self.songs.values().find(|s| s.hash == hash).ok_or(())?;
        let cached_path = self.cache_dir.join(format!("{}.mp3", hash_str));

        if cached_path.exists() {
            match std::fs::read(&cached_path) {
                Ok(d) => Ok((d, hash)),
                Err(e) => {
                    eprintln!("Failed to read cache for {}: {}", hash_str, e);
                    Err(())
                }
            }
        } else {
            // Shouldn't happen since we transcode eagerly, but handle it
            println!("Cache miss for {}, transcoding...", hash_str);
            match transcode::transcode(&song.source_path) {
                Ok(transcoded) => {
                    let _ = std::fs::write(&cached_path, &transcoded);
                    Ok((transcoded, hash))
                }
                Err(e) => {
                    eprintln!("Failed to transcode {}: {}", hash_str, e);
                    Err(())
                }
            }
        }
    }

    pub fn get_transcoded_by_uid(&self, uid: u64) -> Result<(Vec<u8>, Sha256Hash), ()> {
        let hash = self.lookup_hash_from_uid(uid).ok_or(())?;
        self.get_transcoded_by_hash(hash)
    }
}

struct MediaInformation {
    title: String,
    artist: String,
    album: String,
}

fn read_title(path: &Path) -> Option<MediaInformation> {
    // try ID3 first
    if let Ok(tag) = id3::Tag::read_from_path(path) {
        let title = tag.title().unwrap_or("<none>").to_string();
        let artist = tag.artist().unwrap_or("<none>").to_string();
        let album = tag.album().unwrap_or("<none>").to_string();
        return Some(MediaInformation {
            title,
            artist,
            album,
        });
    }

    // fall back to MP4 metadata
    if let Ok(meta) = mp4ameta::Tag::read_from_path(path) {
        let title = meta.title().unwrap_or("<none>").to_string();
        let artist = meta.artist().unwrap_or("<none>").to_string();
        let album = meta.album().unwrap_or("<none>").to_string();
        return Some(MediaInformation {
            title,
            artist,
            album,
        });
    }

    None
}
