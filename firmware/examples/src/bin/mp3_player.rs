#![no_std]
#![no_main]

extern crate alloc;

use cmsis_dsp::transform::FloatRealFft;
use core::mem;
use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{PIO0, PIO1};
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::i2s::{PioI2sOut, PioI2sOutProgram};
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use embassy_rp::pwm::{Pwm, PwmOutput, SetDutyCycle};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Ticker, Timer};
use embedded_alloc::LlffHeap as Heap;
use libm::{cosf, sqrtf};
use smart_leds::RGB8;

use {defmt_rtt as _, panic_probe as _};

#[global_allocator]
static HEAP: Heap = Heap::empty();

const SAMPLE_RATE: u32 = 11_025 * 4;
const BIT_DEPTH: u32 = 16;

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"MP3 Example"),
    embassy_rp::binary_info::rp_program_description!(c"An example that plays an MP3 music file."),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
    PIO1_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO1>;
});

const BUTTON_1: u32 = 1 << 0;
const BUTTON_2: u32 = 1 << 1;
const BUTTON_3: u32 = 1 << 2;
const BUTTON_4: u32 = 1 << 3;

const NUM_LEDS: usize = 64;

static BUTTONS_CHANNEL: PubSubChannel<CriticalSectionRawMutex, u32, 4, 4, 1> = PubSubChannel::new();

static SPECTRUM_CHANNEL: PubSubChannel<CriticalSectionRawMutex, [u8; 8], 4, 4, 1> =
    PubSubChannel::new();

/// Input value 0 to 255 to get a color value.
/// Colors transition r -> g -> b -> back to r.
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

#[embassy_executor::task]
async fn audio_task(i2s: &'static mut PioI2sOut<'static, PIO0, 0>) -> ! {
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
    let fft = FloatRealFft::new(FFT_SIZE as u16).unwrap();
    let mut fft_input = [0f32; FFT_SIZE];
    let mut fft_output = [0f32; FFT_SIZE];

    // Precompute Hann window
    let mut hann_window = [0f32; FFT_SIZE];
    for n in 0..FFT_SIZE {
        hann_window[n] =
            0.5 * (1.0 - cosf(2.0 * core::f32::consts::PI * n as f32 / FFT_SIZE as f32));
    }

    i2s.start();

    // probe-rs download --probe 2e8a:000c examples/src/bin/menu-screen.mp3 --binary-format bin --chip RP235x --base-address 0x10100000
    let file = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 3231241) };

    const MULTIPLIER: f32 = 4095.0;
    let mut _button_state = 0;
    let mut offset = 0;
    let mut in_buffer = [0u8; 32 * 1024];
    let end = offset + in_buffer.len();
    in_buffer.copy_from_slice(file[offset..end].as_ref());
    let mut mp3_buffer = &in_buffer[..];
    offset = end;
    let mut front_sample_count = 0;
    let mut volume = MULTIPLIER;

    loop {
        // let start_time = embassy_time::Instant::now();
        // trigger transfer of front buffer data to the pio fifo
        // but don't await the returned future, yet
        let dma_future = i2s.write(&front_buffer[..front_sample_count]);

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
            _button_state = state;
        }

        // fill back buffer with fresh audio samples before awaiting the dma future

        let (consumed, info) = decoder.decode(mp3_buffer, &mut pcm_buffer);

        mp3_buffer = &mp3_buffer[consumed..];

        let left = mp3_buffer.len();

        let sample_count = if let Some(info) = info {
            defmt::debug!(
                "Decoded MP3 frame: consumed {} produced {} samples, channels {}, {} Hz, {} kbps offset {}",
                consumed,
                info.samples_produced,
                info.channels.num(),
                info.sample_rate,
                info.bitrate,
                offset
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
            defmt::warn!("Failed to decode MP3 frame");
            back_buffer.fill(0);
            back_buffer.len()
        };

        // Compute FFT spectrum from decoded audio
        if sample_count >= FFT_SIZE {
            // Fill FFT input with mono samples (before volume scaling)
            for n in 0..FFT_SIZE {
                fft_input[n] = pcm_buffer[n] * hann_window[n];
            }

            fft.run(&mut fft_input, &mut fft_output);

            // Compute 8 frequency bands from 32 FFT bins (4 bins per band)
            let mut spectrum = [0u8; 8];
            for band in 0..8u8 {
                let mut max_mag = 0.0f32;
                for bin in (band as usize * 4)..((band as usize + 1) * 4) {
                    let mag = if bin == 0 {
                        // DC component (real only)
                        fft_output[0].abs()
                    } else if bin == 32 {
                        // Nyquist (packed in output[1])
                        fft_output[1].abs()
                    } else {
                        // Complex bin: real at output[2*bin], imag at output[2*bin+1]
                        let re = fft_output[2 * bin];
                        let im = fft_output[2 * bin + 1];
                        sqrtf(re * re + im * im)
                    };
                    if mag > max_mag {
                        max_mag = mag;
                    }
                }

                let max_mag_scaled = max_mag * 0.0125;
                // Convert to dB (logarithmic), map to 0-255
                // 20*log10(mag) gives dB; use a floor of ~-100dB and ceiling of ~0dB
                const MIN_DB: f32 = -100.0;
                const MAX_DB: f32 = 0.0;
                const DB_RANGE: f32 = MAX_DB - MIN_DB;
                const DIVISOR: f32 = 255.0 / DB_RANGE;

                let db = if max_mag_scaled > 0.0 {
                    let db = 20.0 * libm::log10f(max_mag_scaled);
                    db.clamp(MIN_DB, MAX_DB)
                } else {
                    MIN_DB
                };
                // Map -100dB..20dB to 0..255
                let scaled = ((db - MIN_DB) * DIVISOR) as i32;
                spectrum[band as usize] = scaled.clamp(0, 255) as u8;
                defmt::debug!("FFT band {}: {} {} dB {}", band, max_mag, db, scaled);
            }
            spectrum_publisher.publish_immediate(spectrum);
        }

        if left < (in_buffer.len() / 2) {
            let pos = in_buffer.len() - left;
            in_buffer.copy_within(pos.., 0);
            let len = pos;
            let (len, reload) = if (offset + len) > file.len() {
                let len = file.len() - offset;
                (len, true)
            } else {
                (len, false)
            };
            let end = offset + len;
            // defmt::info!("Refill {} {} {} {} {}", offset, left, pos, len, end);
            in_buffer[left..left + len].copy_from_slice(file[offset..end].as_ref());
            if reload {
                offset = in_buffer.len();
                decoder = nanomp3::Decoder::new();
                in_buffer.copy_from_slice(file[..offset].as_ref());
                // defmt::info!("MP3 buffer empty, restarting");
            } else {
                offset = end;
            }
            mp3_buffer = &in_buffer[..];
        }

        // now await the dma future. once the dma finishes, the next buffer needs to be q(music_level >> 16) as u8ueued
        // within DMA_DEPTH / SAMPLE_RATE = 960 / 11025 seconds = 87.1ms
        dma_future.await;
        mem::swap(&mut back_buffer, &mut front_buffer);
        front_sample_count = sample_count;
        // let end_time = embassy_time::Instant::now();
        // defmt::info!("Audio frame time: {} us", (end_time - start_time).as_micros());
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let peripherals = embassy_rp::init(Default::default());

    let psram = {
        use embassy_rp::qmi_cs1::QmiCs1;
        let psram_config = embassy_rp::psram::Config::aps6404l();
        embassy_rp::psram::Psram::new(
            QmiCs1::new(peripherals.QMI_CS1, peripherals.PIN_0),
            psram_config,
        )
    };

    if let Ok(psram) = psram {
        {
            unsafe {
                let address = psram.base_address() as usize;
                let size = psram.size() as usize;
                HEAP.init(address, size);
            }
        }
    } else {
        defmt::warn!("Failed to initialize PSRAM, using internal RAM for heap");

        const HEAP_SIZE: usize = 65535; // 64 KiB heap size
        static mut HEAP_MEM: [u8; HEAP_SIZE] = [0xEE; HEAP_SIZE];

        #[allow(static_mut_refs)]
        unsafe {
            let address = HEAP_MEM.as_ptr() as usize;
            HEAP.init(address, HEAP_SIZE);
        }
    }

    let mut pio1 = Pio::new(peripherals.PIO1, Irqs);

    let mut led_colors = [RGB8::default(); NUM_LEDS];

    let program = PioWs2812Program::new(&mut pio1.common);
    let mut ws2812 = PioWs2812::new(
        &mut pio1.common,
        pio1.sm1,
        peripherals.DMA_CH1,
        peripherals.PIN_31,
        &program,
    );

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

    let mut audio_shutdown = Output::new(peripherals.PIN_19, Level::Low);

    // Setup pio state machine for i2s output
    let mut pio0 = Pio::new(peripherals.PIO0, Irqs);

    let bit_clock_pin = peripherals.PIN_21;
    let left_right_clock_pin = peripherals.PIN_22;
    let data_pin = peripherals.PIN_20;

    let program = PioI2sOutProgram::new(&mut pio0.common);
    let i2s = PioI2sOut::new(
        &mut pio0.common,
        pio0.sm0,
        peripherals.DMA_CH0,
        data_pin,
        bit_clock_pin,
        left_right_clock_pin,
        SAMPLE_RATE,
        BIT_DEPTH,
        &program,
    );

    static I2S: static_cell::StaticCell<PioI2sOut<'static, PIO0, 0>> =
        static_cell::StaticCell::new();

    Timer::after_millis(10).await;
    audio_shutdown.set_high();

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
    spawner.spawn(unwrap!(heap_stats_task()));

    let mut ticker = Ticker::every(Duration::from_millis(50));

    let mut spectrum_subscriber = SPECTRUM_CHANNEL.subscriber().unwrap();
    let mut display_spectrum = [0.0f32; 8];

    loop {
        // Take the latest spectrum data
        let mut latest_spectrum = None;
        while let Some(spectrum) = spectrum_subscriber.try_next_message_pure() {
            latest_spectrum = Some(spectrum);
        }

        if let Some(spectrum) = latest_spectrum {
            for col in 0..8 {
                let new_val = spectrum[col] as f32 / 32.0; // Scale 0-255 to 0-8
                // Fast attack, slow decay for smooth visualization
                if new_val > display_spectrum[col] {
                    display_spectrum[col] = new_val;
                } else {
                    display_spectrum[col] *= 0.7;
                }
            }
        } else {
            // Decay when no new data
            for col in 0..8 {
                display_spectrum[col] *= 0.7;
            }
        }

        // Render spectrum to LED matrix
        for col in 0..8usize {
            let bar_height = display_spectrum[col] as usize;
            let bar_height = bar_height.min(8);
            // Frequency gradient: col 0 (low) = red, col 7 (high) = blue
            let hue = (col as u8) * 25; // 0..175 across the wheel
            let color = wheel(hue);

            for row in 0..8usize {
                let led_index = row * 8 + col;
                if row >= (8 - bar_height) {
                    led_colors[led_index] = RGB8 {
                        r: color.r >> 4,
                        g: color.g >> 4,
                        b: color.b >> 4,
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
