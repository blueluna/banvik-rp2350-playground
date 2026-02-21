#![no_std]
#![no_main]

extern crate alloc;

use alloc::vec;
use cmsis_dsp::transform::FloatRealFft;
use core::mem;
use core::str::FromStr;
use cyw43::aligned_bytes;
use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};
use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_net::Stack;
use embassy_net::tcp::TcpSocket;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{PIO0, PIO1, PIO2};
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::i2s::{PioI2sOut, PioI2sOutProgram};
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use embassy_rp::pwm::{Pwm, PwmOutput, SetDutyCycle};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Ticker, Timer};
use embedded_alloc::LlffHeap as Heap;
use embedded_io_async::{Read as _, Write as _};
use libm::sqrtf;
use mp3_protocol::{Request, Response, Status};
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

const WIFI_NETWORK: &str = env!("SSID");
const WIFI_PASSWORD: &str = env!("WIRELESS_PSK");
const MP3_SERVER: &str = env!("MP3_SERVER");

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"MP3 Stream Client"),
    embassy_rp::binary_info::rp_program_description!(
        c"A WiFi MP3 streaming client that plays music from a server."
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
    PIO1_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO1>;
    PIO2_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO2>;
});

const BUTTON_1: u32 = 1 << 0;
const BUTTON_2: u32 = 1 << 1;
const BUTTON_3: u32 = 1 << 2;
const BUTTON_4: u32 = 1 << 3;

const NUM_LEDS: usize = 64;

static BUTTONS_CHANNEL: PubSubChannel<CriticalSectionRawMutex, u32, 4, 4, 1> = PubSubChannel::new();

static SPECTRUM_CHANNEL: PubSubChannel<CriticalSectionRawMutex, [u8; 8], 4, 4, 1> =
    PubSubChannel::new();

struct AudioChunk {
    data: [u8; 4096],
    data_len: usize,
    index: u32,
}

impl AudioChunk {
    fn new(data: &[u8], index: u32) -> Self {
        let len: usize = data.len().min(4096);
        let mut chunk_data = [0u8; 4096];
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

/// Pipe for streaming MP3 data from protocol_task to audio_task.
static AUDIO_CHANNEL: Channel<CriticalSectionRawMutex, AudioChunk, 4> = Channel::new();

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

// ── WiFi / Network tasks ────────────────────────────────────────────────────

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<
        'static,
        cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0, embassy_rp::peripherals::DMA_CH0>>,
    >,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
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

// ── Protocol helpers ─────────────────────────────────────────────────────────

async fn write_message<T: serde::Serialize>(
    socket: &mut TcpSocket<'_>,
    msg: &T,
) -> Result<(), embassy_net::tcp::Error> {
    let mut buf = [0u8; 8192];
    let payload = postcard::to_slice(msg, &mut buf).unwrap();
    let len = (payload.len() as u32).to_be_bytes();
    socket.write_all(&len).await?;
    socket.write_all(payload).await?;
    socket.flush().await?;
    Ok(())
}

async fn read_message_into(
    socket: &mut TcpSocket<'_>,
    buf: &mut [u8],
) -> Result<usize, embassy_net::tcp::Error> {
    let mut len_buf = [0u8; 4];
    socket.read_exact(&mut len_buf).await.map_err(|e| match e {
        embedded_io::ReadExactError::UnexpectedEof => embassy_net::tcp::Error::ConnectionReset,
        embedded_io::ReadExactError::Other(e) => e,
    })?;
    let len = u32::from_be_bytes(len_buf) as usize;
    if len > buf.len() {
        defmt::error!("Message too large: {} > {}", len, buf.len());
        return Err(embassy_net::tcp::Error::ConnectionReset);
    }
    socket
        .read_exact(&mut buf[..len])
        .await
        .map_err(|e| match e {
            embedded_io::ReadExactError::UnexpectedEof => embassy_net::tcp::Error::ConnectionReset,
            embedded_io::ReadExactError::Other(e) => e,
        })?;
    Ok(len)
}

// ── Protocol task ────────────────────────────────────────────────────────────

#[embassy_executor::task]
async fn protocol_task(net_stack: Stack<'static>) -> ! {
    // Parse server address
    let (ip_str, port_str) = MP3_SERVER.rsplit_once(':').unwrap();
    let ip = embassy_net::Ipv4Address::from_str(ip_str).unwrap();
    let port: u16 = port_str.parse().unwrap();
    let remote = embassy_net::IpEndpoint::new(embassy_net::IpAddress::Ipv4(ip), port);

    let mut rx_buf = [0u8; 8192];
    let mut tx_buf = [0u8; 8192];

    // Buffer for receiving messages (needs to be large enough for Chunk responses)
    // A Chunk response contains up to 4096 bytes of data + overhead
    let mut msg_buf = [0u8; 8192];

    loop {
        defmt::info!("Connecting to MP3 server at {}:{}", ip_str, port);

        let mut socket = TcpSocket::new(net_stack, &mut rx_buf, &mut tx_buf);
        socket.set_timeout(Some(Duration::from_secs(30)));

        if let Err(e) = socket.connect(remote).await {
            defmt::warn!("TCP connect failed: {:?}", e);
            Timer::after_secs(2).await;
            continue;
        }

        defmt::info!("Connected to server");

        if let Err(e) = run_session(&mut socket, &mut msg_buf).await {
            defmt::warn!("Session error: {:?}", e);
        }

        defmt::info!("Disconnected, reconnecting...");
        Timer::after_secs(1).await;
    }
}

async fn run_session(
    socket: &mut TcpSocket<'_>,
    msg_buf: &mut [u8],
) -> Result<(), embassy_net::tcp::Error> {
    loop {
        // List songs
        defmt::info!("Requesting song list");
        write_message(socket, &Request::List).await?;

        let len = read_message_into(socket, msg_buf).await?;
        let response: Response = postcard::from_bytes(&msg_buf[..len])
            .map_err(|_| embassy_net::tcp::Error::ConnectionReset)?;

        let songs = match response {
            Response::SongList {
                status: Status::Ok,
                songs,
            } => {
                defmt::info!("Received song list with {} songs", songs.len());
                for song in &songs {
                    let title = core::str::from_utf8(&song.title).unwrap_or("<invalid utf-8>");
                    defmt::info!("{=u32:08x} {}", song.hash, title);
                }
                songs
            }
            _ => {
                defmt::warn!("Unexpected response to List request");
                return Err(embassy_net::tcp::Error::ConnectionReset);
            }
        };

        if songs.is_empty() {
            defmt::warn!("No songs available, retrying in 5s");
            Timer::after_secs(5).await;
            continue;
        }

        // Play each song in order
        for song in songs.iter() {
            let title = core::str::from_utf8(&song.title).unwrap_or("<invalid utf-8>");
            defmt::info!("Playing song {} ({:08x})", title, song.hash);

            write_message(socket, &Request::Play { hash: song.hash }).await?;

            // Receive chunks until the song is complete
            loop {
                let len = read_message_into(socket, msg_buf).await?;
                let response: Response = postcard::from_bytes(&msg_buf[..len])
                    .map_err(|_| embassy_net::tcp::Error::ConnectionReset)?;

                match response {
                    Response::Chunk {
                        status: Status::Ok,
                        chunk_index,
                        total_chunks,
                        data,
                        ..
                    } => {
                        defmt::debug!("Received chunk {}/{}", chunk_index + 1, total_chunks);

                        // Write MP3 data into the channel (blocks if channel is full)
                        AUDIO_CHANNEL
                            .send(AudioChunk::new(data.as_slice(), chunk_index))
                            .await;

                        if chunk_index + 1 >= total_chunks {
                            break;
                        }
                    }
                    Response::Chunk {
                        status: Status::TitleNotFound,
                        ..
                    } => {
                        defmt::warn!("Song not found, skipping");
                        break;
                    }
                    _ => {
                        defmt::warn!("Unexpected response during playback");
                        break;
                    }
                }
            }
        }

        // After playing all songs, loop back to listing
        defmt::info!("All songs played, refreshing list");
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

    const MULTIPLIER: f32 = 4095.0;
    // let mut stream_buffer_a = vec![0u8; 32 * 1024];
    // let mut stream_buffer_b = vec![0u8; 32 * 1024];
    let mut stream_buffer_a = [0u8; 32 * 1024];
    let mut stream_buffer_b = [0u8; 32 * 1024];
    let mut current_stream = stream_buffer_a.as_mut_slice();
    let mut next_stream = stream_buffer_b.as_mut_slice();
    let mut song_change_pending = false; // Whether we've received a new song signal but haven't switched buffers yet
    let mut current_len = 0usize; // How many valid bytes are in in_buffer
    let mut next_len = 0usize; // How many valid bytes are in next_buffer (for the next song)
    let mut mp3_offset = 0usize; // Current read position within in_buffer
    let mut front_sample_count = 0;
    let mut volume = MULTIPLIER;
    let mut counter = 0u32;

    let mut time_start = embassy_time::Instant::now();

    loop {
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

            if let Ok(chunk) = AUDIO_CHANNEL.try_receive() {
                if chunk.index == 0 {
                    song_change_pending = true;
                    next_len = 0;
                }
                let chunk_data = chunk.payload();
                if song_change_pending {
                    next_stream[next_len..next_len + chunk_data.len()].copy_from_slice(chunk_data);
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
            front_buffer[..BUFFER_SIZE].fill(0);
            let dma_future = i2s.write(&front_buffer[..BUFFER_SIZE]);
            dma_future.await;
            continue;
        }

        let dma_future = i2s.write(&front_buffer[..front_sample_count]);

        // Volume control from buttons
        while let Some(state) = buttons_subscriber.try_next_message_pure() {
            if (state & BUTTON_1) == BUTTON_1 {
                if volume == 0.0 {
                    volume = MULTIPLIER;
                } else {
                    volume = 0.0;
                }
            }
            if (state & BUTTON_2) == BUTTON_2 {
                if volume == 0.0 {
                    volume = MULTIPLIER;
                } else {
                    volume = volume * 0.5;
                }
            }
            if (state & BUTTON_3) == BUTTON_3 {
                if volume == 0.0 {
                    volume = MULTIPLIER;
                } else {
                    volume = volume * 2.0;
                }
            }
        }

        let mp3_slice = &current_stream[mp3_offset..current_len];
        let (consumed, info) = decoder.decode(mp3_slice, &mut pcm_buffer);
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
                        let s = (pcm_buffer[n] * volume) as i16;
                        back_buffer[n] = (s as u16 as u32) | ((s as u16 as u32) << 16);
                    }
                    info.samples_produced
                }
                nanomp3::Channels::Stereo => {
                    for n in 0..info.samples_produced {
                        let index = n << 1;
                        let left = pcm_buffer[index];
                        let right = pcm_buffer[index | 1];
                        let left_sample = (left * volume) as i16;
                        let right_sample = (right * volume) as i16;
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

        let timer_end = embassy_time::Instant::now();
        let decode_time = timer_end - time_start;
        defmt::debug!("MP3 frame decoded in {}", decode_time.as_micros());
        dma_future.await;
        time_start = embassy_time::Instant::now();
        defmt::debug!("DMA transfer of {} samples took {} micros", front_sample_count, (time_start - timer_end).as_micros());
        mem::swap(&mut back_buffer, &mut front_buffer);
        front_sample_count = sample_count;
        counter = counter.wrapping_add(1);
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
    let peripherals = embassy_rp::init(Default::default());
    let mut rng = embassy_rp::clocks::RoscRng;

    defmt::info!("MP3 Stream Client starting");

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

    // ── WiFi (CYW43 on PIO0) ─────────────────────────────────────────────
    let clm = aligned_bytes!("../../../firmware/43439A0_clm.bin");
    let nvram = aligned_bytes!("../../../firmware/nvram_rp2040.bin");
    let fw = aligned_bytes!("../../../firmware/43439A0.bin");

    static CYW43_STATE: static_cell::StaticCell<cyw43::State> = static_cell::StaticCell::new();
    let state = CYW43_STATE.init(cyw43::State::new());

    let pwr = Output::new(peripherals.PIN_23, Level::Low);
    let cs = Output::new(peripherals.PIN_25, Level::High);
    let mut pio0 = Pio::new(peripherals.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio0.common,
        pio0.sm0,
        RM2_CLOCK_DIVIDER,
        pio0.irq0,
        cs,
        peripherals.PIN_24,
        peripherals.PIN_29,
        peripherals.DMA_CH0,
    );

    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw, nvram).await;
    spawner.spawn(unwrap!(cyw43_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    // ── Network stack ─────────────────────────────────────────────────────
    let config = embassy_net::Config::dhcpv4(Default::default());
    let seed = rng.next_u64();

    static NET_RESOURCES: static_cell::StaticCell<embassy_net::StackResources<5>> =
        static_cell::StaticCell::new();
    let (net_stack, net_runner) = embassy_net::new(
        net_device,
        config,
        NET_RESOURCES.init(embassy_net::StackResources::new()),
        seed,
    );

    spawner.spawn(unwrap!(net_task(net_runner)));

    // ── WiFi connect ──────────────────────────────────────────────────────
    defmt::info!("Joining WiFi network");
    while let Err(err) = control
        .join(
            WIFI_NETWORK,
            cyw43::JoinOptions::new(WIFI_PASSWORD.as_bytes()),
        )
        .await
    {
        defmt::info!("WiFi join failed: {:?}", err);
    }
    defmt::info!("WiFi joined");

    net_stack.wait_link_up().await;
    defmt::info!("Link up, waiting for DHCP...");
    net_stack.wait_config_up().await;

    if let Some(ipv4_config) = net_stack.config_v4() {
        defmt::info!("Got IP: {}", ipv4_config.address);
    }

    // ── Enable audio amp ──────────────────────────────────────────────────
    Timer::after_millis(10).await;
    audio_shutdown.set_high();

    // ── Spawn tasks ───────────────────────────────────────────────────────
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

    spawner.spawn(unwrap!(audio_task(I2S.init(i2s))));
    spawner.spawn(unwrap!(protocol_task(net_stack)));
    spawner.spawn(unwrap!(heap_stats_task()));

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
