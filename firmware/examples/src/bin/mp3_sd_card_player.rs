#![no_std]
#![no_main]

extern crate alloc;

use alloc::vec;
use alloc::{format, string::String, string::ToString};
use cmsis_dsp::transform::FloatRealFft;
use core::fmt::Write as _;
use core::mem;
use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{
    DMA_CH1, DMA_CH2, DMA_CH3, DMA_CH4, DMA_CH5, DMA_CH6, PIO1, PIO2, SPI0, SPI1,
};
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::i2s::{PioI2sOut, PioI2sOutProgram};
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use embassy_rp::pwm::{Pwm, PwmOutput, SetDutyCycle};
use embassy_rp::spi;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Ticker, Timer};
use embedded_alloc::LlffHeap as Heap;
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{
    Directory, Error as SdFsError, LfnBuffer, Mode, ShortFileName, TimeSource, Timestamp,
    VolumeIdx, VolumeManager,
};
use esp_hal_mfrc522::MFRC522;
use esp_hal_mfrc522::consts::UidSize;
use libm::sqrtf;
use serde::de::{self, MapAccess, Visitor};
use serde::ser::{SerializeMap, SerializeStruct};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use smart_leds::RGB8;

use {defmt_rtt as _, panic_probe as _};

/// Zeroth-order modified Bessel function of the first kind, computed via
/// the convergent power-series expansion (truncated at 20 terms).
fn bessel_i0(x: f32) -> f32 {
    let mut sum = 1.0f32;
    let mut term = 1.0f32;
    let half_x = x * 0.5;
    for k in 1..20 {
        term *= (half_x / k as f32) * (half_x / k as f32);
        sum += term;
    }
    sum
}

#[global_allocator]
static HEAP: Heap = Heap::empty();

const SAMPLE_RATE: u32 = 11_025 * 4;
const BIT_DEPTH: u32 = 16;

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"MP3 SD Card Player"),
    embassy_rp::binary_info::rp_program_description!(
        c"An MP3 player that streams files from an SD card."
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

bind_interrupts!(struct Irqs {
    PIO1_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO1>;
    PIO2_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO2>;
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<DMA_CH1>,
            embassy_rp::dma::InterruptHandler<DMA_CH2>,
            embassy_rp::dma::InterruptHandler<DMA_CH3>,
            embassy_rp::dma::InterruptHandler<DMA_CH4>,
            embassy_rp::dma::InterruptHandler<DMA_CH5>,
            embassy_rp::dma::InterruptHandler<DMA_CH6>;
});

const BUTTON_1: u32 = 1 << 0;
const BUTTON_2: u32 = 1 << 1;
const BUTTON_3: u32 = 1 << 2;
const BUTTON_4: u32 = 1 << 3;

const NUM_LEDS: usize = 64;

static BUTTONS_CHANNEL: PubSubChannel<CriticalSectionRawMutex, u32, 4, 4, 1> = PubSubChannel::new();

static SPECTRUM_CHANNEL: PubSubChannel<CriticalSectionRawMutex, [u8; 8], 4, 1, 1> =
    PubSubChannel::new();

static NFC_UID_CHANNEL: Channel<CriticalSectionRawMutex, [u8; 8], 4> = Channel::new();

struct AudioChunk {
    data: [u8; 2048],
    data_len: usize,
    index: u32,
}

impl AudioChunk {
    fn new(data: &[u8], index: u32) -> Self {
        let len: usize = data.len().min(2048);
        let mut chunk_data = [0u8; 2048];
        chunk_data[..len].copy_from_slice(&data[..len]);
        Self {
            data: chunk_data,
            data_len: len,
            index,
        }
    }

    fn payload(&self) -> &[u8] {
        &self.data[..self.data_len]
    }
}

enum AudioOperation {
    Chunk(AudioChunk),
}

/// Pipe for streaming MP3 data from the SD-card task to audio_task.
static AUDIO_CHANNEL: Channel<CriticalSectionRawMutex, AudioOperation, 2> = Channel::new();

fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        return (255 - wheel_pos * 3, 0, wheel_pos * 3).into();
    }
    if wheel_pos < 170 {
        wheel_pos -= 85;
        return (0, wheel_pos * 3, 255 - wheel_pos * 3).into();
    }
    wheel_pos -= 170;
    (wheel_pos * 3, 255 - wheel_pos * 3, 0).into()
}

struct Controls<'a> {
    button_1: Input<'a>,
    button_2: Input<'a>,
    button_3: Input<'a>,
    button_4: Input<'a>,
    led_1: PwmOutput<'a>,
    led_2: PwmOutput<'a>,
    led_3: PwmOutput<'a>,
    led_4: PwmOutput<'a>,
}

struct SdCardTimeSource;

impl TimeSource for SdCardTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 56,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

struct SdDirEntry {
    short_name: ShortFileName,
    display_name: String,
    is_dir: bool,
    size: u32,
}

struct PlaylistEntry {
    path: String,
    dir_stack: vec::Vec<ShortFileName>,
    file_name: ShortFileName,
    size: u32,
}

type SdSpiBus = spi::Spi<'static, SPI1, spi::Blocking>;
type SdSpiDevice =
    ExclusiveDevice<SdSpiBus, Output<'static>, embedded_hal_bus::spi::NoDelay>;
type SdBlockDevice = embedded_sdmmc::SdCard<SdSpiDevice, embassy_time::Delay>;
type SdDirectory<'a> = Directory<'a, SdBlockDevice, SdCardTimeSource, 4, 4, 1>;
type SdError = embedded_sdmmc::Error<<SdBlockDevice as embedded_sdmmc::BlockDevice>::Error>;

const SONGS_JSON_FILE: &str = "songs.json";
const SONGS_JSON_SHORT_FILE: &str = "SONGS.JSO";
const UID_HEX_LEN: usize = 16;
const MAX_SONG_PATH_LEN: usize = 96;
const MAX_SONG_BINDINGS: usize = 64;
const SONGS_JSON_BUFFER_SIZE: usize = 4096;

#[derive(Clone)]
struct SongBinding {
    uid: heapless::String<UID_HEX_LEN>,
    path: heapless::String<MAX_SONG_PATH_LEN>,
}

#[derive(Default)]
struct SongsDb {
    entries: heapless::Vec<SongBinding, MAX_SONG_BINDINGS>,
}

struct SongsMapRef<'a> {
    entries: &'a [SongBinding],
}

struct SongsEntries(SongsDb);

enum StreamSongOutcome {
    Completed,
    InterruptedByUid {
        uid: [u8; 8],
        offset: u32,
        next_chunk_index: u32,
    },
}

impl SongsDb {
    fn upsert(
        &mut self,
        uid: heapless::String<UID_HEX_LEN>,
        path: heapless::String<MAX_SONG_PATH_LEN>,
    ) -> Result<(), ()> {
        if let Some(entry) = self.entries.iter_mut().find(|entry| entry.uid == uid) {
            entry.path = path;
            return Ok(());
        }

        self.entries
            .push(SongBinding { uid, path })
            .map_err(|_| ())
    }

    fn find_path(&self, uid: &str) -> Option<&str> {
        self.entries
            .iter()
            .find(|entry| entry.uid.as_str() == uid)
            .map(|entry| entry.path.as_str())
    }
}

impl Serialize for SongsMapRef<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut map = serializer.serialize_map(Some(self.entries.len()))?;
        for entry in self.entries {
            map.serialize_entry(entry.uid.as_str(), entry.path.as_str())?;
        }
        map.end()
    }
}

impl Serialize for SongsDb {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut state = serializer.serialize_struct("SongsDb", 1)?;
        state.serialize_field(
            "songs",
            &SongsMapRef {
                entries: &self.entries,
            },
        )?;
        state.end()
    }
}

impl<'de> Deserialize<'de> for SongsEntries {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        struct SongsEntriesVisitor;

        impl<'de> Visitor<'de> for SongsEntriesVisitor {
            type Value = SongsEntries;

            fn expecting(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                formatter.write_str("a songs mapping object")
            }

            fn visit_map<A>(self, mut map: A) -> Result<Self::Value, A::Error>
            where
                A: MapAccess<'de>,
            {
                let mut songs_db = SongsDb::default();

                while let Some((uid, path)) = map.next_entry::<
                    heapless::String<UID_HEX_LEN>,
                    heapless::String<MAX_SONG_PATH_LEN>,
                >()? {
                    songs_db
                        .upsert(uid, path)
                        .map_err(|_| de::Error::custom("too many song mappings"))?;
                }

                Ok(SongsEntries(songs_db))
            }
        }

        deserializer.deserialize_map(SongsEntriesVisitor)
    }
}

impl<'de> Deserialize<'de> for SongsDb {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        #[serde(field_identifier, rename_all = "lowercase")]
        enum Field {
            Songs,
        }

        struct SongsDbVisitor;

        impl<'de> Visitor<'de> for SongsDbVisitor {
            type Value = SongsDb;

            fn expecting(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                formatter.write_str("a songs database object")
            }

            fn visit_map<A>(self, mut map: A) -> Result<Self::Value, A::Error>
            where
                A: MapAccess<'de>,
            {
                let mut songs_db = None;

                while let Some(key) = map.next_key::<Field>()? {
                    match key {
                        Field::Songs => {
                            if songs_db.is_some() {
                                return Err(de::Error::duplicate_field("songs"));
                            }
                            songs_db = Some(map.next_value::<SongsEntries>()?.0);
                        }
                    }
                }

                Ok(songs_db.unwrap_or_default())
            }
        }

        deserializer.deserialize_struct("SongsDb", &["songs"], SongsDbVisitor)
    }
}

fn is_mp3_name(name: &str) -> bool {
    name.to_ascii_lowercase().ends_with(".mp3")
}

fn format_uid(uid: [u8; 8]) -> heapless::String<UID_HEX_LEN> {
    let mut formatted = heapless::String::<UID_HEX_LEN>::new();
    for byte in uid {
        let _ = write!(&mut formatted, "{:02x}", byte);
    }
    formatted
}

fn song_matches_path(song: &PlaylistEntry, mapped_path: &str) -> bool {
    let trimmed_path = song.path.trim_start_matches('/');
    let trimmed_mapped = mapped_path.trim_start_matches('/');
    if trimmed_path.eq_ignore_ascii_case(trimmed_mapped) {
        return true;
    }

    let song_name = trimmed_path.rsplit('/').next().unwrap_or(trimmed_path);
    song_name.eq_ignore_ascii_case(trimmed_mapped)
}

fn find_song_index(playlist: &[PlaylistEntry], mapped_path: &str) -> Option<usize> {
    playlist
        .iter()
        .position(|song| song_matches_path(song, mapped_path))
}

fn load_songs_db(
    volume: &embedded_sdmmc::Volume<'_, SdBlockDevice, SdCardTimeSource, 4, 4, 1>,
) -> SongsDb {
    let root_dir = match volume.open_root_dir() {
        Ok(dir) => dir,
        Err(error) => {
            defmt::warn!("Failed to open root dir for songs.json: {}", defmt::Debug2Format(&error));
            return SongsDb::default();
        }
    };

    let songs_file_short_name = match find_root_file_short_name(&root_dir, SONGS_JSON_FILE) {
        Ok(Some(short_name)) => Some(short_name),
        Ok(None) => match find_root_file_short_name(&root_dir, SONGS_JSON_SHORT_FILE) {
            Ok(Some(short_name)) => Some(short_name),
            Ok(None) => None,
            Err(error) => {
                defmt::warn!("Failed to scan root dir for songs.json: {}", defmt::Debug2Format(&error));
                return SongsDb::default();
            }
        },
        Err(error) => {
            defmt::warn!("Failed to scan root dir for songs.json: {}", defmt::Debug2Format(&error));
            return SongsDb::default();
        }
    };

    let Some(songs_file_short_name) = songs_file_short_name else {
        defmt::info!("songs.json not found on SD card");
        return SongsDb::default();
    };

    let file = match root_dir.open_file_in_dir(&songs_file_short_name, Mode::ReadOnly) {
        Ok(file) => file,
        Err(SdFsError::NotFound) => {
            defmt::info!("songs.json not found on SD card");
            return SongsDb::default();
        }
        Err(error) => {
            defmt::warn!("Failed to open songs.json: {}", defmt::Debug2Format(&error));
            return SongsDb::default();
        }
    };

    let mut buffer = [0u8; SONGS_JSON_BUFFER_SIZE];
    let mut used = 0usize;

    loop {
        if used == buffer.len() {
            defmt::warn!("songs.json is too large to load, ignoring contents");
            return SongsDb::default();
        }

        match file.read(&mut buffer[used..]) {
            Ok(0) => break,
            Ok(len) => used += len,
            Err(error) => {
                defmt::warn!("Failed reading songs.json: {}", defmt::Debug2Format(&error));
                return SongsDb::default();
            }
        }
    }

    if used == 0 {
        return SongsDb::default();
    }

    match serde_json_core::from_slice::<SongsDb>(&buffer[..used]) {
        Ok((songs_db, _)) => songs_db,
        Err(error) => {
            defmt::warn!("Failed parsing songs.json: {}", defmt::Debug2Format(&error));
            SongsDb::default()
        }
    }
}

fn save_songs_db(
    volume: &embedded_sdmmc::Volume<'_, SdBlockDevice, SdCardTimeSource, 4, 4, 1>,
    songs_db: &SongsDb,
) {
    let mut json_buffer = [0u8; SONGS_JSON_BUFFER_SIZE];
    let json_len = match serde_json_core::to_slice(songs_db, &mut json_buffer) {
        Ok(len) => len,
        Err(error) => {
            defmt::warn!("Failed serializing songs.json: {}", defmt::Debug2Format(&error));
            return;
        }
    };

    let root_dir = match volume.open_root_dir() {
        Ok(dir) => dir,
        Err(error) => {
            defmt::warn!("Failed to open root dir for songs.json write: {}", defmt::Debug2Format(&error));
            return;
        }
    };

    let songs_file_short_name = match find_root_file_short_name(&root_dir, SONGS_JSON_FILE) {
        Ok(Some(short_name)) => short_name,
        Ok(None) => match find_root_file_short_name(&root_dir, SONGS_JSON_SHORT_FILE) {
            Ok(Some(short_name)) => short_name,
            Ok(None) => match ShortFileName::create_from_str(SONGS_JSON_SHORT_FILE) {
                Ok(short_name) => {
                    defmt::warn!(
                        "songs.json is missing; creating {} because embedded_sdmmc only creates 8.3 filenames",
                        SONGS_JSON_SHORT_FILE
                    );
                    short_name
                }
                Err(error) => {
                    defmt::warn!("Invalid short fallback filename for songs.json: {}", defmt::Debug2Format(&error));
                    return;
                }
            },
            Err(error) => {
                defmt::warn!("Failed to scan root dir for songs.json: {}", defmt::Debug2Format(&error));
                return;
            }
        },
        Err(error) => {
            defmt::warn!("Failed to scan root dir for songs.json: {}", defmt::Debug2Format(&error));
            return;
        }
    };

    let file = match root_dir.open_file_in_dir(&songs_file_short_name, Mode::ReadWriteCreateOrTruncate) {
        Ok(file) => file,
        Err(error) => {
            defmt::warn!("Failed to open songs.json for write: {}", defmt::Debug2Format(&error));
            return;
        }
    };

    if let Err(error) = file.write(&json_buffer[..json_len]) {
        defmt::warn!("Failed writing songs.json: {}", defmt::Debug2Format(&error));
        return;
    }

    if let Err(error) = file.flush() {
        defmt::warn!("Failed flushing songs.json: {}", defmt::Debug2Format(&error));
    }
}

fn find_root_file_short_name(
    root_dir: &SdDirectory<'_>,
    target_name: &str,
) -> Result<Option<ShortFileName>, SdError> {
    let mut lfn_storage = [0u8; 260];
    let mut lfn_buffer = LfnBuffer::new(&mut lfn_storage);
    let mut found = None;

    root_dir.iterate_dir_lfn(&mut lfn_buffer, |entry, lfn_name| {
        let display_name = lfn_name.unwrap_or("");
        if display_name.eq_ignore_ascii_case(target_name)
            || format!("{}", entry.name).eq_ignore_ascii_case(target_name)
        {
            found = Some(entry.name.clone());
        }
    })?;

    Ok(found)
}

fn handle_nfc_uid(
    volume: &embedded_sdmmc::Volume<'_, SdBlockDevice, SdCardTimeSource, 4, 4, 1>,
    songs_db: &mut SongsDb,
    playlist: &[PlaylistEntry],
    uid: [u8; 8],
) -> Option<usize> {
    let uid_string = format_uid(uid);
    defmt::info!("NFC UID {}", uid_string.as_str());

    match songs_db.find_path(uid_string.as_str()) {
        Some(path) if !path.is_empty() => match find_song_index(playlist, path) {
            Some(song_index) => {
                defmt::info!("NFC selected {}", playlist[song_index].path.as_str());
                Some(song_index)
            }
            None => {
                defmt::warn!(
                    "Mapped song '{}' for UID {} was not found on SD card",
                    path,
                    uid_string.as_str()
                );
                None
            }
        },
        Some(_) => {
            defmt::warn!("UID {} has no song assigned in songs.json", uid_string.as_str());
            None
        }
        None => {
            defmt::warn!("Unknown NFC UID {}, adding to songs.json", uid_string.as_str());
            if songs_db
                .upsert(uid_string, heapless::String::new())
                .is_err()
            {
                defmt::warn!("songs.json mapping table is full; could not persist unknown UID");
            } else {
                save_songs_db(volume, songs_db);
            }
            None
        }
    }
}

fn collect_mp3_files_recursive(
    dir: &SdDirectory<'_>,
    current_path: &str,
    dir_stack: &mut vec::Vec<ShortFileName>,
    playlist: &mut vec::Vec<PlaylistEntry>,
) -> Result<(), SdError> {
    let mut entries: alloc::vec::Vec<SdDirEntry> = alloc::vec::Vec::new();
    let mut lfn_storage = [0u8; 260];
    let mut lfn_buffer = LfnBuffer::new(&mut lfn_storage);

    dir.iterate_dir_lfn(&mut lfn_buffer, |entry, lfn_name| {
        entries.push(SdDirEntry {
            short_name: entry.name.clone(),
            display_name: lfn_name
                .map(|name| name.to_string())
                .unwrap_or_else(|| format!("{}", entry.name)),
            is_dir: entry.attributes.is_directory(),
            size: entry.size,
        });
    })?;

    entries.sort_unstable_by(|left, right| left.display_name.cmp(&right.display_name));

    for entry in entries {
        if entry.short_name == ShortFileName::this_dir() || entry.short_name == ShortFileName::parent_dir() {
            continue;
        }

        let full_path = if current_path == "/" {
            format!("/{}", entry.display_name)
        } else {
            format!("{}/{}", current_path, entry.display_name)
        };

        if entry.is_dir {
            match dir.open_dir(&entry.short_name) {
                Ok(subdir) => {
                    dir_stack.push(entry.short_name.clone());
                    let recurse_result = collect_mp3_files_recursive(
                        &subdir,
                        full_path.as_str(),
                        dir_stack,
                        playlist,
                    );
                    dir_stack.pop();
                    if let Err(e) = recurse_result {
                        defmt::warn!(
                            "SD recurse failed on {}: {}",
                            full_path.as_str(),
                            defmt::Debug2Format(&e)
                        );
                    }
                }
                Err(e) => {
                    defmt::warn!(
                        "SD open dir failed on {}: {}",
                        full_path.as_str(),
                        defmt::Debug2Format(&e)
                    );
                }
            }
        } else if is_mp3_name(entry.display_name.as_str()) {
            playlist.push(PlaylistEntry {
                path: full_path,
                dir_stack: dir_stack.clone(),
                file_name: entry.short_name,
                size: entry.size,
            });
        }
    }

    Ok(())
}

fn log_id3_tags(path: &str, tags: &rp2350_playground::id3::TagInfo<'_>, size: u32) {
    defmt::info!("Playing {} ({} bytes)", path, size);
    log_id3_field("Title", tags.title);
    log_id3_field("Artist", tags.artist);
    log_id3_field("Album", tags.album);
    log_id3_field("Year", tags.year);
    log_id3_field("Track", tags.track);
}

fn log_id3_field(label: &str, bytes: Option<&[u8]>) {
    let Some(bytes) = bytes else {
        return;
    };

    if label == "Track" {
        log_track_field(bytes);
        return;
    }

    if let Ok(s) = core::str::from_utf8(bytes) {
        defmt::info!("  {}: {}", label, s);
        return;
    }

    if let Some(decoded) = decode_utf16_heuristic(bytes) {
        defmt::info!("  {}: {}", label, decoded.as_str());
        return;
    }

    defmt::info!("  {}: <unrecognized text encoding>", label);
}

fn log_track_field(bytes: &[u8]) {
    if bytes.len() == 1 {
        let track = bytes[0];
        if track != 0 {
            defmt::info!("  Track: {}", track);
            return;
        }
    }

    if let Ok(s) = core::str::from_utf8(bytes) {
        if let Some(track) = first_track_component(s) {
            defmt::info!("  Track: {}", track);
            return;
        }
    }

    if let Some(decoded) = decode_utf16_heuristic(bytes) {
        if let Some(track) = first_track_component(decoded.as_str()) {
            defmt::info!("  Track: {}", track);
            return;
        }
    }

    defmt::info!("  Track: <unrecognized text encoding>");
}

fn first_track_component(value: &str) -> Option<&str> {
    let trimmed = value.trim_matches('\0').trim();
    if trimmed.is_empty() {
        return None;
    }
    let first = trimmed.split('/').next().unwrap_or(trimmed).trim();
    if first.is_empty() {
        None
    } else {
        Some(first)
    }
}

fn decode_utf16_heuristic(bytes: &[u8]) -> Option<heapless::String<128>> {
    if bytes.len() < 2 {
        return None;
    }

    let (little_endian, start) = if bytes.starts_with(&[0xFF, 0xFE]) {
        (true, 2)
    } else if bytes.starts_with(&[0xFE, 0xFF]) {
        (false, 2)
    } else {
        let mut low_zeroes = 0usize;
        let mut high_zeroes = 0usize;
        let mut i = 0usize;
        while i + 1 < bytes.len() {
            if bytes[i] == 0 {
                low_zeroes += 1;
            }
            if bytes[i + 1] == 0 {
                high_zeroes += 1;
            }
            i += 2;
        }

        if low_zeroes == 0 && high_zeroes == 0 {
            return None;
        }

        (high_zeroes >= low_zeroes, 0)
    };

    let mut units = heapless::Vec::<u16, 128>::new();
    let mut i = start;

    while i + 1 < bytes.len() {
        let code_unit = if little_endian {
            u16::from_le_bytes([bytes[i], bytes[i + 1]])
        } else {
            u16::from_be_bytes([bytes[i], bytes[i + 1]])
        };
        i += 2;

        if code_unit == 0 {
            break;
        }

        if units.push(code_unit).is_err() {
            break;
        }
    }

    if units.is_empty() {
        return None;
    }

    let mut out = heapless::String::<128>::new();
    for decoded in core::char::decode_utf16(units.into_iter()) {
        match decoded {
            Ok(ch) => {
                if out.push(ch).is_err() {
                    break;
                }
            }
            Err(_) => {
                if out.push('\u{FFFD}').is_err() {
                    break;
                }
            }
        }
    }

    if out.is_empty() {
        None
    } else {
        Some(out)
    }
}

async fn stream_sd_song(
    volume: &embedded_sdmmc::Volume<'_, SdBlockDevice, SdCardTimeSource, 4, 4, 1>,
    song: &PlaylistEntry,
    start_offset: u32,
    start_chunk_index: u32,
) -> Result<StreamSongOutcome, SdError> {
    let mut dir = volume.open_root_dir()?;

    for component in &song.dir_stack {
        dir.change_dir(component)?;
    }

    let file = dir.open_file_in_dir(&song.file_name, Mode::ReadOnly)?;
    if start_offset != 0 {
        file.seek_from_start(start_offset)?;
    }

    let mut chunk_index = start_chunk_index;
    let mut chunk_buffer = [0u8; 2048];
    let mut audio_end = song.size;

    if start_offset == 0 && song.size >= 128 {
        let mut tail = [0u8; 128];
        file.seek_from_start(song.size - 128)?;
        if file.read(&mut tail)? == 128 {
            if let rp2350_playground::id3::Id3Detection::Id3v1 { .. } =
                rp2350_playground::id3::detect_id3_tags(&[], Some(&tail))
            {
                audio_end = song.size - 128;
                defmt::debug!(
                    "Detected ID3v1 trailer on {}, trimming 128 bytes",
                    song.path.as_str()
                );
            }
        }
        file.seek_from_start(0)?;
    }

    if start_offset == 0 {
        let first_len = file.read(&mut chunk_buffer)?;
        if first_len == 0 {
            defmt::warn!("Skipping empty MP3 {}", song.path.as_str());
            return Ok(StreamSongOutcome::Completed);
        }

        if let Some(header) = rp2350_playground::id3::parse_id3v2_header(&chunk_buffer[..first_len]) {
            let mut parser = rp2350_playground::id3::Id3v2StreamParser::<96>::new(header);
            let tag_total = header.total_size;
            let tag_body_start = 10usize;
            let tag_body_end = tag_body_start + header.tag_size;
            let mut trailing_audio: Option<(usize, usize)> = None;

            let first_body_end = first_len.min(tag_body_end);
            if first_body_end > tag_body_start {
                parser.feed(&chunk_buffer[tag_body_start..first_body_end]);
            }

            if first_len > tag_total {
                trailing_audio = Some((tag_total, first_len));
            }

            while (file.offset() as usize) < tag_body_end {
                let remaining_tag = tag_body_end - file.offset() as usize;
                let chunk_capacity = chunk_buffer.len();
                let read_len = remaining_tag.min(chunk_capacity);
                let len = file.read(&mut chunk_buffer[..read_len])?;
                if len == 0 {
                    break;
                }
                parser.feed(&chunk_buffer[..len]);
            }

            file.seek_from_start(tag_total as u32)?;

            let parsed_tags = parser.tags().as_tag_info();
            log_id3_tags(song.path.as_str(), &parsed_tags, song.size);

            if let Some((start, end)) = trailing_audio {
                AUDIO_CHANNEL
                    .send(AudioOperation::Chunk(AudioChunk::new(
                        &chunk_buffer[start..end],
                        chunk_index,
                    )))
                    .await;
                chunk_index += 1;
            }
        } else {
            defmt::info!("Playing {} ({} bytes)", song.path.as_str(), song.size);

            AUDIO_CHANNEL
                .send(AudioOperation::Chunk(AudioChunk::new(
                    &chunk_buffer[..first_len],
                    chunk_index,
                )))
                .await;
            chunk_index += 1;
        }
    }

    while file.offset() < audio_end {
        if let Ok(uid) = NFC_UID_CHANNEL.try_receive() {
            return Ok(StreamSongOutcome::InterruptedByUid {
                uid,
                offset: file.offset(),
                next_chunk_index: chunk_index,
            });
        }

        let remaining = audio_end.saturating_sub(file.offset()) as usize;
        let chunk_capacity = chunk_buffer.len();
        let len = file.read(&mut chunk_buffer[..remaining.min(chunk_capacity)])?;
        if len == 0 {
            break;
        }

        AUDIO_CHANNEL
            .send(AudioOperation::Chunk(AudioChunk::new(
                &chunk_buffer[..len],
                chunk_index,
            )))
            .await;
        chunk_index += 1;
    }

    if chunk_index == 0 {
        defmt::warn!("Skipping empty MP3 {}", song.path.as_str());
    }

    Ok(StreamSongOutcome::Completed)
}

#[embassy_executor::task]
async fn sd_card_stream_task(spi_device: SdSpiDevice) -> ! {
    let delay = embassy_time::Delay;
    let sd_card = embedded_sdmmc::SdCard::new(spi_device, delay);

    let card_size = match sd_card.num_bytes() {
        Ok(size) => size,
        Err(e) => {
            defmt::error!("SD card init failed: {}", defmt::Debug2Format(&e));
            loop {
                Timer::after_secs(5).await;
            }
        }
    };

    defmt::info!("SD card ready: {} bytes", card_size);

    let volume_mgr = VolumeManager::new(sd_card, SdCardTimeSource);
    let volume = match volume_mgr.open_volume(VolumeIdx(0)) {
        Ok(volume) => volume,
        Err(e) => {
            defmt::error!("SD open volume 0 failed: {}", defmt::Debug2Format(&e));
            loop {
                Timer::after_secs(5).await;
            }
        }
    };

    let root_dir = match volume.open_root_dir() {
        Ok(dir) => dir,
        Err(e) => {
            defmt::error!("SD open root dir failed: {}", defmt::Debug2Format(&e));
            loop {
                Timer::after_secs(5).await;
            }
        }
    };

    let mut playlist = vec![];
    let mut dir_stack = vec![];
    if let Err(e) = collect_mp3_files_recursive(&root_dir, "/", &mut dir_stack, &mut playlist) {
        defmt::error!("SD scan failed: {}", defmt::Debug2Format(&e));
        loop {
            Timer::after_secs(5).await;
        }
    }

    playlist.sort_unstable_by(|left, right| left.path.cmp(&right.path));

    if playlist.is_empty() {
        defmt::error!("No MP3 files found on SD card");
        loop {
            Timer::after_secs(5).await;
        }
    }

    defmt::info!("Found {} MP3 files on SD card", playlist.len());
    for song in &playlist {
        defmt::info!(" - {}", song.path.as_str());
    }

    let mut songs_db = load_songs_db(&volume);
    let mut playlist_index = 0usize;
    loop {
        let song = &playlist[playlist_index];
        let mut resume_offset = 0u32;
        let mut resume_chunk_index = 0u32;

        match stream_sd_song(&volume, song, resume_offset, resume_chunk_index).await {
            Ok(StreamSongOutcome::Completed) => {
                resume_offset = 0;
                resume_chunk_index = 0;
                playlist_index += 1;
                if playlist_index >= playlist.len() {
                    playlist_index = 0;
                    defmt::info!("Reached end of playlist, restarting from top");
                }
            }
            Ok(StreamSongOutcome::InterruptedByUid {
                uid,
                offset,
                next_chunk_index,
            }) => {
                if let Some(song_index) = handle_nfc_uid(&volume, &mut songs_db, &playlist, uid) {
                    playlist_index = song_index;
                    resume_offset = 0;
                    resume_chunk_index = 0;
                } else {
                    resume_offset = offset;
                    resume_chunk_index = next_chunk_index;
                }
            }
            Err(e) => {
                defmt::warn!("Failed to stream {}: {}", song.path.as_str(), defmt::Debug2Format(&e));
                Timer::after_millis(250).await;
            }
        }

        while resume_offset != 0 {
            match stream_sd_song(&volume, &playlist[playlist_index], resume_offset, resume_chunk_index).await {
                Ok(StreamSongOutcome::Completed) => {
                    resume_offset = 0;
                    resume_chunk_index = 0;
                    playlist_index += 1;
                    if playlist_index >= playlist.len() {
                        playlist_index = 0;
                        defmt::info!("Reached end of playlist, restarting from top");
                    }
                }
                Ok(StreamSongOutcome::InterruptedByUid {
                    uid,
                    offset,
                    next_chunk_index,
                }) => {
                    if let Some(song_index) = handle_nfc_uid(&volume, &mut songs_db, &playlist, uid) {
                        playlist_index = song_index;
                        resume_offset = 0;
                        resume_chunk_index = 0;
                    } else {
                        resume_offset = offset;
                        resume_chunk_index = next_chunk_index;
                    }
                }
                Err(e) => {
                    defmt::warn!(
                        "Failed to resume {}: {}",
                        playlist[playlist_index].path.as_str(),
                        defmt::Debug2Format(&e)
                    );
                    Timer::after_millis(250).await;
                    break;
                }
            }
        }
    }
}

// ── Buttons task (reused from mp3_player) ────────────────────────────────────

#[embassy_executor::task]
async fn buttons_task(controls: &'static mut Controls<'static>) -> ! {
    let mut button_states = 0u32;
    let mut button_level = (0u16, 0u16, 0u16, 0u16);

    let buttons_publisher = BUTTONS_CHANNEL.publisher().unwrap();

    let mut ticker = Ticker::every(Duration::from_millis(10));

    loop {
        let current_button_states = {
            let mut states = 0u32;
            if controls.button_1.is_low() {
                states |= BUTTON_1;
            }
            if controls.button_2.is_low() {
                states |= BUTTON_2;
            }
            if controls.button_3.is_low() {
                states |= BUTTON_3;
            }
            if controls.button_4.is_low() {
                states |= BUTTON_4;
            }
            states
        };
        button_level.0 = button_level.0.saturating_sub(256);
        button_level.1 = button_level.1.saturating_sub(256);
        button_level.2 = button_level.2.saturating_sub(256);
        button_level.3 = button_level.3.saturating_sub(256);

        if current_button_states != button_states {
            button_states = current_button_states;
            if button_states & BUTTON_1 == BUTTON_1 {
                button_level.0 = 32767;
            }
            if button_states & BUTTON_2 == BUTTON_2 {
                button_level.1 = 32767;
            }
            if button_states & BUTTON_3 == BUTTON_3 {
                button_level.2 = 32767;
            }
            if button_states & BUTTON_4 == BUTTON_4 {
                button_level.3 = 32767;
            }
            buttons_publisher.publish_immediate(button_states);
        }

        let _ = controls.led_1.set_duty_cycle(32767 - button_level.0);
        let _ = controls.led_2.set_duty_cycle(32767 - button_level.1);
        let _ = controls.led_3.set_duty_cycle(32767 - button_level.2);
        let _ = controls.led_4.set_duty_cycle(32767 - button_level.3);
        ticker.next().await;
    }
}

// ── Audio task ───────────────────────────────────────────────────────────────

#[embassy_executor::task]
async fn audio_task(i2s: &'static mut PioI2sOut<'static, PIO1, 0>) -> ! {
    let mut decoder = nanomp3::Decoder::new();
    const BUFFER_SIZE: usize = nanomp3::MAX_SAMPLES_PER_FRAME;

    let mut pcm_buffer = [0f32; BUFFER_SIZE];

    static DMA_BUFFER: static_cell::StaticCell<[u32; BUFFER_SIZE * 2]> =
        static_cell::StaticCell::new();
    let dma_buffer = DMA_BUFFER.init_with(|| [0u32; BUFFER_SIZE * 2]);
    let (mut back_buffer, mut front_buffer) = dma_buffer.split_at_mut(BUFFER_SIZE);

    let mut buttons_subscriber = BUTTONS_CHANNEL.subscriber().unwrap();
    let spectrum_publisher = SPECTRUM_CHANNEL.publisher().unwrap();

    const FFT_SIZE: usize = 64;
    let fft: FloatRealFft = FloatRealFft::new(FFT_SIZE as u16).unwrap();
    let mut fft_input = [0f32; FFT_SIZE];
    let mut fft_output = [0f32; FFT_SIZE];

    const KAISER_BETA: f32 = 5.0;
    let mut kaiser_window = [0f32; FFT_SIZE];
    let i0_beta = bessel_i0(KAISER_BETA);
    for n in 0..FFT_SIZE {
        let ratio = 2.0 * n as f32 / FFT_SIZE as f32 - 1.0;
        kaiser_window[n] = bessel_i0(KAISER_BETA * sqrtf(1.0 - ratio * ratio)) / i0_beta;
    }

    i2s.start();

    const MULTIPLIER: f32 = 1024.0;
    let mut stream_buffer_a = vec![0u8; 4 * 1024];
    let mut stream_buffer_b = vec![0u8; 4 * 1024];
    let mut current_stream = stream_buffer_a.as_mut_slice();
    let mut next_stream = stream_buffer_b.as_mut_slice();
    let mut song_change_pending = false; // Whether we've received a new song signal but haven't switched buffers yet
    let mut current_len = 0usize; // How many valid bytes are in in_buffer
    let mut next_len = 0usize; // How many valid bytes are in next_buffer (for the next song)
    let mut mp3_offset = 0usize; // Current read position within in_buffer
    let mut front_sample_count = 0;
    let mut volume = MULTIPLIER;
    let mut volume_effective = volume;

    loop {
        let dma_future = i2s.write(&front_buffer[..front_sample_count]);

        let current_remaining = current_len - mp3_offset;
        // Refill in_buffer from pipe when running low
        let remaining = if song_change_pending {
            next_len
        } else {
            current_remaining
        };
        if remaining < current_stream.len() / 2 {
            // Shift remaining data to front
            if mp3_offset > 0 && current_remaining > 0 {
                current_stream.copy_within(mp3_offset..current_len, 0);
            }
            current_len -= mp3_offset;
            mp3_offset = 0;

            if let Ok(operation) = AUDIO_CHANNEL.try_receive() {
                match operation {
                    AudioOperation::Chunk(chunk) => {
                        if chunk.index == 0 {
                            song_change_pending = true;
                            next_len = 0;
                        }
                        let chunk_data = chunk.payload();
                        if song_change_pending {
                            next_stream[next_len..next_len + chunk_data.len()]
                                .copy_from_slice(chunk_data);
                            next_len += chunk_data.len();
                            defmt::debug!(
                                "Audio: Fill next {} bytes (chunk index {})",
                                chunk.data_len,
                                chunk.index
                            );
                        } else {
                            current_stream[current_len..current_len + chunk_data.len()]
                                .copy_from_slice(chunk_data);
                            current_len += chunk_data.len();
                            defmt::debug!(
                                "Audio: Fill current {} bytes (chunk index {})",
                                chunk.data_len,
                                chunk.index
                            );
                        }
                    }
                }
            }
        }

        if (current_len - mp3_offset) == 0 && song_change_pending {
            // No more data for current song, switch to next song buffer
            decoder = nanomp3::Decoder::new();
            mem::swap(&mut current_stream, &mut next_stream);
            current_len = next_len;
            next_len = 0;
            mp3_offset = 0;
            song_change_pending = false;
        }

        if current_len == 0 {
            // Still no data, output silence
            back_buffer[..BUFFER_SIZE].fill(0);
            front_sample_count = BUFFER_SIZE;
            dma_future.await;
            continue;
        }

        // Volume control from buttons
        while let Some(state) = buttons_subscriber.try_next_message_pure() {
            if (state & BUTTON_1) == BUTTON_1 {
                if volume_effective == 0.0 {
                    volume_effective = volume;
                } else {
                    volume_effective = 0.0;
                }
            }
            if (state & BUTTON_2) == BUTTON_2 {
                if volume_effective == 0.0 {
                    volume_effective = volume;
                } else {
                    volume = volume * 0.5;
                    volume_effective = volume;
                }
            }
            if (state & BUTTON_3) == BUTTON_3 {
                if volume_effective == 0.0 {
                    volume_effective = volume;
                } else {
                    volume = volume * 2.0;
                    volume_effective = volume;
                }
            }
        }

        let (consumed, info) = decoder.decode(
            &current_stream[mp3_offset..current_len],
            &mut pcm_buffer[0..],
        );
        mp3_offset += consumed;

        let sample_count = if let Some(info) = info {
            defmt::debug!(
                "Decoded: consumed {} samples {} ch {} rate {} kbps {}",
                consumed,
                info.samples_produced,
                info.channels.num(),
                info.sample_rate,
                info.bitrate,
            );
            match info.channels {
                nanomp3::Channels::Mono => {
                    for n in 0..info.samples_produced {
                        let s = (pcm_buffer[n] * volume_effective) as i16;
                        back_buffer[n] = (s as u16 as u32) | ((s as u16 as u32) << 16);
                    }
                    info.samples_produced
                }
                nanomp3::Channels::Stereo => {
                    for n in 0..info.samples_produced {
                        let index = n << 1;
                        let left = pcm_buffer[index];
                        let right = pcm_buffer[index | 1];
                        let left_sample = (left * volume_effective) as i16;
                        let right_sample = (right * volume_effective) as i16;
                        back_buffer[n] =
                            (left_sample as u16 as u32) | ((right_sample as u16 as u32) << 16);
                    }
                    info.samples_produced
                }
            }
        } else {
            defmt::warn!(
                "Failed to decode MP3 frame, {} offset {} remaining",
                mp3_offset,
                current_len - mp3_offset
            );
            mp3_offset = current_len; // Skip the problematic data
            back_buffer.fill(0);
            back_buffer.len()
        };

        // FFT spectrum
        if sample_count >= FFT_SIZE {
            for n in 0..FFT_SIZE {
                fft_input[n] = pcm_buffer[n] * kaiser_window[n];
            }

            fft.run(&mut fft_input, &mut fft_output);

            let mut spectrum = [0u8; 8];
            for band in 0..8u8 {
                let mut max_mag = 0.0f32;
                for bin in (band as usize * 4)..((band as usize + 1) * 4) {
                    let mag = if bin == 0 {
                        fft_output[0].abs() / 2.0
                    } else if bin == 32 {
                        fft_output[1].abs()
                    } else {
                        let re = fft_output[2 * bin];
                        let im = fft_output[2 * bin + 1];
                        sqrtf(re * re + im * im)
                    };
                    if mag > max_mag {
                        max_mag = mag;
                    }
                }

                let max_mag_scaled = max_mag * 0.0125;
                const MIN_DB: f32 = -100.0;
                const MAX_DB: f32 = -20.0;
                const DB_RANGE: f32 = MAX_DB - MIN_DB;
                const DIVISOR: f32 = 255.0 / DB_RANGE;

                let db = if max_mag_scaled > 0.0 {
                    let db = 20.0 * libm::log10f(max_mag_scaled);
                    db.clamp(MIN_DB, MAX_DB)
                } else {
                    MIN_DB
                };
                let scaled = ((db - MIN_DB) * DIVISOR) as i32;
                spectrum[band as usize] = scaled.clamp(0, 255) as u8;
            }
            spectrum_publisher.publish_immediate(spectrum);
        }

        dma_future.await;
        mem::swap(&mut back_buffer, &mut front_buffer);
        front_sample_count = sample_count;
    }
}

// ── Heap stats task ──────────────────────────────────────────────────────────

#[embassy_executor::task]
async fn heap_stats_task() {
    let mut ticker = Ticker::every(Duration::from_secs(5));
    let mut last_used_kib = 0;
    let mut last_free_kib = 0;

    loop {
        let used = HEAP.used();
        let free = HEAP.free();
        let used_kib = used / 1024;
        let free_kib = free / 1024;
        if last_used_kib != used_kib || last_free_kib != free_kib {
            defmt::info!("Heap used: {} KiB, free: {} KiB", used_kib, free_kib);
        }

        last_used_kib = used_kib;
        last_free_kib = free_kib;

        ticker.next().await;
    }
}

// ── Main ─────────────────────────────────────────────────────────────────────

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let mut rp_configuration: embassy_rp::config::Config = Default::default();
    rp_configuration.clocks = embassy_rp::clocks::ClockConfig::crystal(12_000_000);
    let peripherals = embassy_rp::init(rp_configuration);

    // ── PSRAM / Heap ──────────────────────────────────────────────────────
    let psram = {
        use embassy_rp::qmi_cs1::QmiCs1;
        let psram_config = embassy_rp::psram::Config::aps6404l();
        embassy_rp::psram::Psram::new(
            QmiCs1::new(peripherals.QMI_CS1, peripherals.PIN_0),
            psram_config,
        )
    };

    if let Ok(psram) = psram {
        unsafe {
            let address = psram.base_address() as usize;
            let size = psram.size() as usize;
            HEAP.init(address, size);
            defmt::info!("Heap in PSRAM at {:08x}, size: {}", address, size);
        }
    } else {
        defmt::warn!("Failed to initialize PSRAM, using internal RAM for heap");
        const HEAP_SIZE: usize = 65535;
        static mut HEAP_MEM: [u8; HEAP_SIZE] = [0xEE; HEAP_SIZE];

        #[allow(static_mut_refs)]
        unsafe {
            let address = HEAP_MEM.as_ptr() as usize;
            HEAP.init(address, HEAP_SIZE);
        }
    }

    // ── Audio shutdown (keep amp off until ready) ─────────────────────────
    let mut audio_shutdown = Output::new(peripherals.PIN_19, Level::Low);

    // ── I2S audio on PIO1 SM0 ─────────────────────────────────────────────
    let mut pio1 = Pio::new(peripherals.PIO1, Irqs);

    let i2s_program = PioI2sOutProgram::new(&mut pio1.common);
    let i2s = PioI2sOut::new(
        &mut pio1.common,
        pio1.sm0,
        peripherals.DMA_CH1,
        Irqs,
        peripherals.PIN_20,
        peripherals.PIN_21,
        peripherals.PIN_22,
        SAMPLE_RATE,
        BIT_DEPTH,
        &i2s_program,
    );

    static I2S: static_cell::StaticCell<PioI2sOut<'static, PIO1, 0>> =
        static_cell::StaticCell::new();

    let mut pio2 = Pio::new(peripherals.PIO2, Irqs);

    // ── WS2812 on PIO2 SM0 ───────────────────────────────────────────────
    let ws2812_program = PioWs2812Program::new(&mut pio2.common);
    let mut ws2812 = PioWs2812::new(
        &mut pio2.common,
        pio2.sm0,
        peripherals.DMA_CH2,
        Irqs,
        peripherals.PIN_31,
        &ws2812_program,
    );

    // ── Buttons + LEDs ────────────────────────────────────────────────────
    static CONTROL: static_cell::StaticCell<Controls> = static_cell::StaticCell::new();

    let mut pwm_config: embassy_rp::pwm::Config = Default::default();
    pwm_config.top = 32_768;
    let pwm_2_1 = Pwm::new_output_ab(
        peripherals.PWM_SLICE9,
        peripherals.PIN_34,
        peripherals.PIN_35,
        pwm_config,
    );
    let (pwm_2, pwm_1) = pwm_2_1.split();

    let mut pwm_config: embassy_rp::pwm::Config = Default::default();
    pwm_config.top = 32_768;
    let pwm_4_3 = Pwm::new_output_ab(
        peripherals.PWM_SLICE8,
        peripherals.PIN_32,
        peripherals.PIN_33,
        pwm_config,
    );
    let (pwm_4, pwm_3) = pwm_4_3.split();

    // ── NFC ───────────────────────────────────────────────────────────────

    let mut mfrc522_reset = Output::new(peripherals.PIN_6, Level::Low);
    let mut _mfrc522_irq = Input::new(peripherals.PIN_5, Pull::Up);

    let mut nfc_spi_config = spi::Config::default();
    nfc_spi_config.frequency = 1_000_000;
    nfc_spi_config.polarity = spi::Polarity::IdleLow;
    nfc_spi_config.phase = spi::Phase::CaptureOnFirstTransition;

    let nfc_spi_bus = spi::Spi::new(
        peripherals.SPI0,
        peripherals.PIN_2,
        peripherals.PIN_3,
        peripherals.PIN_4,
        peripherals.DMA_CH3,
        peripherals.DMA_CH4,
        Irqs,
        nfc_spi_config,
    );

    let nfc_cs = Output::new(peripherals.PIN_1, Level::High);
    let nfc_spi_device = ExclusiveDevice::new_no_delay(nfc_spi_bus, nfc_cs).unwrap();
    let nfc_driver = esp_hal_mfrc522::drivers::SpiDriver::new(nfc_spi_device);
    let mfrc522 = MFRC522::new(nfc_driver);

    // SPI configuration for microSD
    let mut spi_1_config = spi::Config::default();
    spi_1_config.frequency = 16_000_000;
    spi_1_config.polarity = spi::Polarity::IdleLow;
    spi_1_config.phase = spi::Phase::CaptureOnFirstTransition;

    // SPI1 in blocking mode for embedded-sdmmc
    let spi_1_bus = spi::Spi::new_blocking(
        peripherals.SPI1,
        peripherals.PIN_10, // SCK
        peripherals.PIN_11, // MOSI (TX)
        peripherals.PIN_8, // MISO (RX)
        spi_1_config,
    );

    let cs_1 = Output::new(peripherals.PIN_9, Level::High);

    let spi_1_device = ExclusiveDevice::new_no_delay(spi_1_bus, cs_1).unwrap();

    Timer::after_millis(10).await;
    mfrc522_reset.set_high();
    Timer::after_millis(50).await;

    spawner.spawn(unwrap!(buttons_task(CONTROL.init(Controls {
        button_1: Input::new(peripherals.PIN_39, Pull::Up),
        button_2: Input::new(peripherals.PIN_38, Pull::Up),
        button_3: Input::new(peripherals.PIN_37, Pull::Up),
        button_4: Input::new(peripherals.PIN_36, Pull::Up),
        led_1: pwm_1.unwrap(),
        led_2: pwm_2.unwrap(),
        led_3: pwm_3.unwrap(),
        led_4: pwm_4.unwrap(),
    }))));

    // ── Enable audio amp ──────────────────────────────────────────────────
    Timer::after_millis(10).await;
    audio_shutdown.set_high();

    static NFC_DEVICE: static_cell::StaticCell<
        MFRC522<
            esp_hal_mfrc522::drivers::SpiDriver<
                ExclusiveDevice<
                    spi::Spi<'static, SPI0, spi::Async>,
                    Output<'static>,
                    embedded_hal_bus::spi::NoDelay,
                >,
            >,
        >,
    > = static_cell::StaticCell::new();

    // ── Spawn tasks ───────────────────────────────────────────────────────

    spawner.spawn(unwrap!(audio_task(I2S.init(i2s))));
    spawner.spawn(unwrap!(sd_card_stream_task(spi_1_device)));
    spawner.spawn(unwrap!(heap_stats_task()));
    spawner.spawn(unwrap!(nfc_task(NFC_DEVICE.init(mfrc522))));

    // ── LED visualization loop (main task) ────────────────────────────────
    let mut led_colors = [RGB8::default(); NUM_LEDS];
    let mut ticker = Ticker::every(Duration::from_millis(50));
    let mut spectrum_subscriber = SPECTRUM_CHANNEL.subscriber().unwrap();
    let mut display_spectrum = [0.0f32; 8];
    const DECAY: f32 = 0.95;

    loop {
        let mut latest_spectrum = None;
        while let Some(spectrum) = spectrum_subscriber.try_next_message_pure() {
            latest_spectrum = Some(spectrum);
        }

        if let Some(spectrum) = latest_spectrum {
            for col in 0..8 {
                let new_val = spectrum[col] as f32 / 32.0;
                if new_val > display_spectrum[col] {
                    display_spectrum[col] = new_val;
                } else {
                    display_spectrum[col] *= DECAY;
                }
            }
        } else {
            for col in 0..8 {
                display_spectrum[col] *= DECAY;
            }
        }

        let bar_heights: [u8; 8] = display_spectrum.map(|v| (v + 0.5) as u8);
        for col in 0..8usize {
            let bar_height = bar_heights[col] as usize;
            let bar_height = bar_height.min(8);
            let hue = (col as u8) * 25;
            let color = wheel(hue);

            for row in 0..8 {
                let led_index = row * 8 + col;
                if row >= (8 - bar_height) {
                    led_colors[led_index] = RGB8 {
                        r: color.r >> 6,
                        g: color.g >> 6,
                        b: color.b >> 6,
                    };
                } else {
                    led_colors[led_index] = RGB8::default();
                }
            }
        }
        ws2812.write(&led_colors).await;
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn nfc_task(
    nfc: &'static mut MFRC522<
        esp_hal_mfrc522::drivers::SpiDriver<
            ExclusiveDevice<
                spi::Spi<'static, SPI0, spi::Async>,
                Output<'static>,
                embedded_hal_bus::spi::NoDelay,
            >,
        >,
    >,
) -> ! {
    match nfc.pcd_init().await {
        Ok(()) => defmt::info!("MFRC522 initialized successfully"),
        Err(_error) => {
            defmt::error!("Failed to initialize MFRC522");
            loop {
                Timer::after_millis(1000).await;
            }
        }
    }

    match nfc.pcd_get_version().await {
        Ok(version) => defmt::info!("MFRC522 firmware version: {}", version as u8),
        Err(_error) => defmt::error!("Failed to get MFRC522 version"),
    }

    match nfc.pcd_set_antenna_gain(0xff).await {
        Ok(()) => defmt::info!("Antenna gain set to max"),
        Err(_error) => defmt::error!("Failed to set antenna gain"),
    }

    defmt::info!("Scanning for NFC cards...");

    loop {
        if nfc.picc_is_new_card_present().await.is_err() {
            continue;
        }

        let uid = match nfc.get_card(UidSize::Four).await {
            Ok(uid) => uid,
            Err(_) => match nfc.get_card(UidSize::Seven).await {
                Ok(uid) => uid,
                Err(_) => continue,
            },
        };

        let uid_len = uid.size as usize;
        let uid_bytes = &uid.uid_bytes[..uid_len];
        let target_type = rp2350_playground::nfc::TargetType::from_response(uid.sak, uid.size);
        defmt::info!(
            "Found card with UID: {=[u8]:02x} SAK: {=u8:02x} {}",
            uid_bytes,
            uid.sak,
            target_type
        );

        if uid_len <= 8 {
            let mut padded_uid = [0u8; 8];
            padded_uid[8 - uid_len..].copy_from_slice(uid_bytes);
            defmt::info!("NFC: Use {=[u8]:02x}", padded_uid);
            NFC_UID_CHANNEL.send(padded_uid).await;
            Timer::after_millis(500).await;
        }

        let _ = nfc.picc_halta().await;
        let _ = nfc.pcd_stop_crypto1().await;
    }
}
