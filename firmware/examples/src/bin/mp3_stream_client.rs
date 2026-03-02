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
use embassy_futures::select::{Either, select};
use embassy_net::Stack;
use embassy_net::tcp::TcpSocket;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{
    DMA_CH0, DMA_CH1, DMA_CH2, DMA_CH3, DMA_CH4, PIO0, PIO1, PIO2, SPI0,
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
use embedded_io_async::{Read, Write as _};
use esp_hal_mfrc522::MFRC522;
use esp_hal_mfrc522::consts::UidSize;
use libm::sqrtf;
use mp3_protocol::{Request, Response, Status, StreamChunk};
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
const STREAM_HOST: &str = env!("STREAM_HOST");
const STREAM_PORT: &str = env!("STREAM_PORT");

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
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<DMA_CH0>,
            embassy_rp::dma::InterruptHandler<DMA_CH1>,
            embassy_rp::dma::InterruptHandler<DMA_CH2>,
            embassy_rp::dma::InterruptHandler<DMA_CH3>,
            embassy_rp::dma::InterruptHandler<DMA_CH4>;
});

const BUTTON_1: u32 = 1 << 0;
const BUTTON_2: u32 = 1 << 1;
const BUTTON_3: u32 = 1 << 2;
const BUTTON_4: u32 = 1 << 3;

const NUM_LEDS: usize = 64;

static BUTTONS_CHANNEL: PubSubChannel<CriticalSectionRawMutex, u32, 4, 4, 1> = PubSubChannel::new();

static SPECTRUM_CHANNEL: PubSubChannel<CriticalSectionRawMutex, [u8; 8], 4, 1, 1> =
    PubSubChannel::new();

static NFC_CHANNEL: PubSubChannel<CriticalSectionRawMutex, [u8; 8], 4, 1, 1> = PubSubChannel::new();

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

enum AudioOperation {
    Chunk(AudioChunk),
    NewStream,
    StopStream,
}

enum StreamControl {
    Play { hash: [u8; 32] },
    Stop,
}

/// Pipe for streaming MP3 data from protocol_task to audio_task.
static AUDIO_CHANNEL: Channel<CriticalSectionRawMutex, AudioOperation, 4> = Channel::new();

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
    runner: cyw43::Runner<'static, cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0>>>,
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
    let payload = postcard::to_slice(msg, &mut buf[4..]).unwrap();
    let payload_len = payload.len();
    let len = (payload_len as u32).to_be_bytes();
    buf[..4].copy_from_slice(&len);
    socket.write_all(&buf[..payload_len + 4]).await?;
    socket.flush().await?;
    Ok(())
}

async fn read_message_into(
    socket: &mut TcpSocket<'_>,
    buf: &mut [u8],
) -> Result<usize, u32> {
    let mut len_buf = [0u8; 4];
    socket.read_exact(&mut len_buf).await.map_err(|e| match e {
        embedded_io::ReadExactError::UnexpectedEof => 1u32,
        embedded_io::ReadExactError::Other(_) => 1u32,
    })?;
    let len = u32::from_be_bytes(len_buf) as usize;
    if len > buf.len() {
        defmt::error!("Message too large: {} > {}", len, buf.len());
        // Attempt to clear the socket buffer
        match socket.read(buf).await {
            Ok(len) => defmt::warn!("Socket buffer cleared: {} bytes", len),
            Err(_) => defmt::error!("Failed to clear socket buffer"),
        }
        return Err(2);
    }
    socket
        .read_exact(&mut buf[..len])
        .await
        .map_err(|e| match e {
            embedded_io::ReadExactError::UnexpectedEof => 1u32,
            embedded_io::ReadExactError::Other(_) => 1u32,
        })?;
    Ok(len)
}

// ── Protocol task ────────────────────────────────────────────────────────────

#[embassy_executor::task]
async fn protocol_task(net_stack: Stack<'static>) -> ! {
    // Parse server address
    let ip = embassy_net::Ipv4Address::from_str(STREAM_HOST).unwrap();
    let port: u16 = STREAM_PORT.parse().unwrap();
    let remote = embassy_net::IpEndpoint::new(embassy_net::IpAddress::Ipv4(ip), port);

    let mut rx_buf = [0u8; 1024];
    let mut tx_buf = [0u8; 1024];
    let mut msg_buf = [0u8; 1024];

    // Buffers for the dedicated stream TCP socket
    let mut stream_rx_buf = vec![0u8; 8192];
    let mut stream_tx_buf = vec![0u8; 8192];
    let mut stream_buf = vec![0u8; 8192];

    let stream_remote = embassy_net::IpEndpoint::new(
        embassy_net::IpAddress::Ipv4(ip),
        port + 1,
    );

    loop {
        let mut socket = TcpSocket::new(net_stack, &mut rx_buf, &mut tx_buf);
        socket.set_keep_alive(Some(Duration::from_secs(10)));
        socket.set_timeout(Some(Duration::from_secs(30)));

        if let Err(e) = socket.connect(remote).await {
            defmt::warn!("TCP connect failed: {:?}", e);
            Timer::after_secs(2).await;
            continue;
        }

        defmt::info!("Connected to {}:{} (control)", STREAM_HOST, STREAM_PORT);

        let mut stream_socket = TcpSocket::new(net_stack, &mut stream_rx_buf, &mut stream_tx_buf);
        stream_socket.set_keep_alive(Some(Duration::from_secs(10)));
        stream_socket.set_timeout(Some(Duration::from_secs(30)));

        if let Err(e) = stream_socket.connect(stream_remote).await {
            defmt::warn!("Stream TCP connect failed: {:?}", e);
            Timer::after_secs(2).await;
            continue;
        }

        defmt::info!("Connected to {}:{} (stream)", STREAM_HOST, port + 1);

        if let Err(e) = run_session(&mut socket, &mut stream_socket, &mut msg_buf, &mut stream_buf).await {
            defmt::warn!("Session error: {:?}", e);
        }

        defmt::info!("Disconnected, reconnecting...");
        Timer::after_millis(500).await;
    }
}

// ── Control task (NFC + control TCP → sends StreamControl to stream task) ────

async fn control_task(
    socket: &mut TcpSocket<'_>,
    msg_buf: &mut [u8],
    stream_ctrl: &Channel<CriticalSectionRawMutex, StreamControl, 4>,
) -> Result<(), embassy_net::tcp::Error> {
    let mut nfc_subscriber = NFC_CHANNEL.subscriber().unwrap();
    
    loop {
        match select(
            nfc_subscriber.next_message(),
            read_message_into(socket, msg_buf),
        )
        .await
        {
            Either::First(nfc_result) => match nfc_result {
                embassy_sync::pubsub::WaitResult::Message(uid) => {
                    defmt::info!("Query song from NFC {=[u8]:02x}", uid);
                    write_message(
                        socket,
                        &Request::QueryUid {
                            uid: u64::from_be_bytes(uid),
                        },
                    )
                    .await?;
                }
                embassy_sync::pubsub::WaitResult::Lagged(_) => {
                    defmt::warn!("Lagged while waiting for NFC message, skipping");
                }
            },

            Either::Second(Ok(len)) => {
                let response: Response = postcard::from_bytes(&msg_buf[..len])
                    .map_err(|_| embassy_net::tcp::Error::ConnectionReset)?;

                match response {
                    Response::Play { status, hash, total_chunks: _ } => match status {
                        Status::Ok => {
                            stream_ctrl.send(StreamControl::Play { hash }).await;
                        }
                        _ => defmt::warn!("Failed to start stream, {}", status),
                    },
                    Response::Song { status, song } => {
                        if status == Status::Ok {
                            let title = core::str::from_utf8(&song.title)
                                .unwrap_or("<invalid title>");
                            defmt::info!("Found song {}, Playing", title);
                            write_message(socket, &Request::PlayHash { hash: song.hash })
                                .await?;
                        } else {
                            defmt::warn!("Failed to get song metadata, {}", status);
                        }
                    }
                    Response::SongList { status, songs } => {
                        if status == Status::Ok {
                            defmt::debug!("Received song list, {}", songs.len());
                            for song in songs {
                                let title = core::str::from_utf8(&song.title)
                                    .unwrap_or("<invalid title>");
                                defmt::info!(" - {}", title);
                            }
                        } else {
                            defmt::warn!("Failed to get song list, {}", status);
                        }
                    }
                    Response::Stop { status } => {
                        if status == Status::Ok {
                            defmt::info!("Server stopped the stream");
                            stream_ctrl.send(StreamControl::Stop).await;
                        } else {
                            defmt::warn!("Failed to stop stream, {}", status);
                        }
                    }
                }
            }

            Either::Second(Err(code)) => {
                if code == 1 {
                    defmt::warn!("Control connection closed by peer");
                    return Err(embassy_net::tcp::Error::ConnectionReset);
                }
                // code 2: message too large — already warned in read_message_into, continue
            }
        }
    }
}

// ── Stream task (stream TCP, drained by control messages between reads) ───────

async fn stream_task(
    socket: &mut TcpSocket<'_>,
    stream_buf: &mut [u8],
    stream_ctrl: &Channel<CriticalSectionRawMutex, StreamControl, 4>,
) -> Result<(), embassy_net::tcp::Error> {
    let mut current_song_hash = [0u8; 32];
    let mut next_song_hash = [0u8; 32];
    let mut flush_stream = false;
    loop {
        // Drain pending control signals before blocking on the next read so we
        // never cancel a read_message_into call mid-message (which would
        // desynchronise the length-prefix framing).
        while let Ok(ctrl) = stream_ctrl.try_receive() {
            match ctrl {
                StreamControl::Play { hash } => {
                    defmt::info!("Stream: new stream, {=[u8]:02x}", hash);
                    if current_song_hash == hash {
                        defmt::info!("Stream: already playing this song");
                    } else if current_song_hash != [0u8; 32] {
                        defmt::info!("Stream: switching to new song");
                        AUDIO_CHANNEL.send(AudioOperation::NewStream).await;
                        flush_stream = true;
                    }
                    next_song_hash = hash;
                }
                StreamControl::Stop => {
                    defmt::info!("Stream: stopped by server");
                    current_song_hash = [0u8; 32];
                    AUDIO_CHANNEL.send(AudioOperation::StopStream).await;
                }
            }
        }

        match read_message_into(socket, stream_buf).await {
            Ok(len) => match postcard::from_bytes::<StreamChunk>(&stream_buf[..len]) {
                Ok(chunk) => {
                    defmt::debug!(
                        "TCP chunk {}/{}",
                        chunk.chunk_index,
                        chunk.total_chunks
                    );
                    if chunk.hash != current_song_hash {
                        defmt::info!("Received chunk for new song {} / {}, switching stream", chunk.chunk_index, chunk.total_chunks);
                        current_song_hash = chunk.hash;
                    }
                    if chunk.chunk_index == chunk.total_chunks - 1 {
                        current_song_hash = [0u8; 32];
                        defmt::info!("Received last chunk of the stream");
                    }
                    if flush_stream {
                        if chunk.hash == next_song_hash {
                            defmt::info!("Chunk belongs to the new song, unflushing");
                            flush_stream = false;
                        }
                        else {
                            defmt::info!("Flushing stream chunk (index {})", chunk.chunk_index);
                        }
                    }
                    if flush_stream == false {
                        AUDIO_CHANNEL
                            .send(AudioOperation::Chunk(AudioChunk::new(
                                chunk.data.as_slice(),
                                chunk.chunk_index,
                            )))
                            .await;
                    }
                }
                Err(_) => defmt::warn!("Failed to decode StreamChunk"),
            },
            Err(code) => {
                if code == 1 {
                    defmt::warn!("Stream connection closed by peer");
                    return Err(embassy_net::tcp::Error::ConnectionReset);
                }
                // code 2: message too large — already warned, continue
            }
        }
    }
}

// ── Session (runs control and stream tasks concurrently) ─────────────────────

async fn run_session(
    socket: &mut TcpSocket<'_>,
    stream_socket: &mut TcpSocket<'_>,
    msg_buf: &mut [u8],
    stream_buf: &mut [u8],
) -> Result<(), embassy_net::tcp::Error> {
    let stream_ctrl = Channel::<CriticalSectionRawMutex, StreamControl, 4>::new();

    match select(
        control_task(socket, msg_buf, &stream_ctrl),
        stream_task(stream_socket, stream_buf, &stream_ctrl),
    )
    .await
    {
        Either::First(result) | Either::Second(result) => result,
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
    let mut stream_buffer_a = vec![0u8; 32 * 1024];
    let mut stream_buffer_b = vec![0u8; 32 * 1024];
    // let mut stream_buffer_a = [0u8; 32 * 1024];
    // let mut stream_buffer_b = [0u8; 32 * 1024];
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
                            defmt::info!(
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
                    AudioOperation::NewStream => {
                        defmt::info!("Audio: New stream incoming, {} bytes, {} bytes", current_len, mp3_offset);
                        current_len = 0;
                        mp3_offset = 0;
                    }
                    AudioOperation::StopStream => {
                        defmt::info!("Audio: Stream stopped");
                        current_len = 0;
                        next_len = 0;
                        mp3_offset = 0;
                        song_change_pending = false;
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
    let mut rng = embassy_rp::clocks::RoscRng;

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

    // ── NFC ─────────────────────────────────────────────

    // MFRC522 reset pin
    let mut mfrc522_reset = Output::new(peripherals.PIN_6, Level::Low);

    // SPI configuration for MFRC522 (SPI Mode 0: CPOL=0, CPHA=0)
    let mut spi_config = spi::Config::default();
    spi_config.frequency = 1_000_000; // 1 MHz
    spi_config.polarity = spi::Polarity::IdleLow;
    spi_config.phase = spi::Phase::CaptureOnFirstTransition;

    // SPI0 with DMA for async support
    let spi_bus = spi::Spi::new(
        peripherals.SPI0,
        peripherals.PIN_2, // SCK
        peripherals.PIN_3, // MOSI (TX)
        peripherals.PIN_4, // MISO (RX)
        peripherals.DMA_CH3,
        peripherals.DMA_CH4,
        Irqs,
        spi_config,
    );

    let cs = Output::new(peripherals.PIN_1, Level::High);

    let spi_device = ExclusiveDevice::new_no_delay(spi_bus, cs).unwrap();
    let driver = esp_hal_mfrc522::drivers::SpiDriver::new(spi_device);
    let mfrc522 = MFRC522::new(driver);

    // Hardware reset
    Timer::after_millis(10).await;
    mfrc522_reset.set_high();
    Timer::after_millis(50).await;

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
        embassy_rp::dma::Channel::new(peripherals.DMA_CH0, Irqs),
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
    net_stack.wait_config_up().await;

    if let Some(ipv4_config) = net_stack.config_v4() {
        defmt::info!("Got IP: {}", ipv4_config.address);
    }

    // ── Enable audio amp ──────────────────────────────────────────────────
    Timer::after_millis(10).await;
    audio_shutdown.set_high();

    static NFC_DEVICE: static_cell::StaticCell<
        MFRC522<
            esp_hal_mfrc522::drivers::SpiDriver<
                ExclusiveDevice<
                    spi::Spi<'_, SPI0, spi::Async>,
                    Output<'_>,
                    embedded_hal_bus::spi::NoDelay,
                >,
            >,
        >,
    > = static_cell::StaticCell::new();
    // ── Spawn tasks ───────────────────────────────────────────────────────

    spawner.spawn(unwrap!(audio_task(I2S.init(i2s))));
    spawner.spawn(unwrap!(protocol_task(net_stack)));
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
    let nfc_publisher = NFC_CHANNEL.publisher().unwrap();

    // Initialize MFRC522
    match nfc.pcd_init().await {
        Ok(()) => defmt::info!("MFRC522 initialized successfully"),
        Err(_e) => {
            defmt::error!("Failed to initialize MFRC522");
            loop {
                Timer::after_millis(1000).await;
            }
        }
    }

    match nfc.pcd_get_version().await {
        Ok(version) => defmt::info!("MFRC522 firmware version: {}", version as u8),
        Err(_e) => defmt::error!("Failed to get MFRC522 version"),
    }

    match nfc.pcd_set_antenna_gain(0xff).await {
        Ok(()) => defmt::info!("Antenna gain set to max"),
        Err(_e) => defmt::error!("Failed to set antenna gain"),
    }

    defmt::info!("Scanning for NFC cards...");

    loop {
        // Check for new card
        if nfc.picc_is_new_card_present().await.is_err() {
            continue;
        }

        // Read card UID
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

        if uid_len == 7 {
            // NTAG / Ultralight - NFC Forum Type 2 Tag
            let mut uid = [0u8; 8];
            uid[1..8].copy_from_slice(uid_bytes);
            defmt::info!("NFC: Use {=[u8]:02x}", uid);
            nfc_publisher.publish_immediate(uid);
            Timer::after_millis(500).await;
        } else if uid_len == 4 {
            // MIFARE Classic - authenticate and read sectors
        }

        let _ = nfc.picc_halta().await;
        let _ = nfc.pcd_stop_crypto1().await;
    }
}
