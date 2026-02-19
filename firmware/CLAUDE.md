# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Embedded Rust firmware for the RP2350 microcontroller (dual-core ARM Cortex-M85). Uses the Embassy async runtime. All runnable code lives in `examples/src/bin/` as individual binaries.

## Build & Run Commands

Format the code with `cargo fmt`.

Following describes how to run the examples.

```bash
# Run any example (flashes via probe-rs automatically)
cargo run --release --bin blinky
cargo run --release --bin audio
cargo run --release --bin mp3_player
cargo run --release --bin player
cargo run --release --bin soundboard
cargo run --release --bin bmp390
cargo run --release --bin tmp117
cargo run --release --bin psram_heap

# WiFi example requires env vars
SSID=<ssid> WIRELESS_PSK=<password> cargo run --release --bin wireless_80211

# Upload MP3 data to flash before running mp3_player or soundboard
probe-rs download --probe 2e8a:000c examples/src/bin/<file>.mp3 \
  --binary-format bin --chip RP235x --base-address 0x10100000
```

The `.cargo/config.toml` runner automatically uses `probe-rs` with probe ID `2e8a:000c` and target `thumbv8m.main-none-eabihf`.

## Hardware

**Pinout (from examples):**
- GPIO 19: Audio amp shutdown (active low)
- GPIO 20-22: I2S audio (PIO0)
- GPIO 31: WS2812 LED strip (PIO1)
- GPIO 32-35: PWM LEDs (SLICE8/9)
- GPIO 36-39: Buttons
- QMI_CS1: External PSRAM

**Memory layout** (`examples/memory.x`): 4MB flash at `0x10000000`, 512KB RAM at `0x20000000`. MP3 audio data stored in flash starting at `0x10100000`.

## Architecture

### Embassy Async Patterns
All examples use `#[embassy_executor::main]`. Tasks are spawned with `spawner.spawn()` and communicate via static `PubSubChannel`s. The typical task structure per example:
- **button_task**: 10ms debounce ticker, publishes button state
- **audio_task**: Double-buffered DMA for continuous I2S output
- **led_task**: Controls WS2812 (via PIO+DMA) or PWM LEDs

### Audio Pipeline
I2S output is implemented with PIO for bit-accurate timing. Audio uses double-buffering: DMA drains `front_buffer` while CPU fills `back_buffer`, then swaps. The `audio` example synthesizes tones; `mp3_player` decodes MP3 via `nanomp3`; `player` uses `xmrsplayer` for MOD tracker files.

### Memory Hierarchy
PSRAM is initialized first for the heap (via `embedded-alloc`); falls back to internal SRAM if unavailable. See `psram_heap.rs` for the init pattern.

### Logging
Uses `defmt` + `defmt-rtt` for logging. `DEFMT_LOG=info` is set in `.cargo/config.toml`.

## Key Dependencies

- **embassy-rp / embassy-executor**: Async HAL and runtime (patched to `main` branch of embassy-rs)
- **nanomp3**: MP3 decoding (patched to `master`)
- **xmrs / xmrsplayer**: MOD tracker playback
- **cyw43 / embassy-net**: CYW43439 WiFi chip + networking stack
- **smart-leds**: WS2812 LED protocol
- **embedded-devices**: Sensor drivers (BMP390, TMP117)
- **defmt**: Embedded logging framework
