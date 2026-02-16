# RP2350 Playground Rust Firmware

## Examples

### Blinky

Blink with WS2812 smart LEDs.

```shell
cargo run --release --bin blinky
```

### PSRAM heap

Set up PSRAM as heap for allocator.

```shell
cargo run --release --bin psram_heap
```

### Audio

Play a sine wave on the speaker.

```shell
cargo run --release --bin audio
```

### Mod Player

Play a music mod file.

```shell
cargo run --release --bin player
```

### MP3 Player

Play a music mp3 file. Currently not functional.

```shell
probe-rs download --probe 2e8a:000c music.mp3 --binary-format bin --chip RP235x --base-address 0x10100000
cargo run --release --bin mp3_player
```

### Wireless 802.11

Connect to Wi-Fi and get an IPv4 address.

```shell
SSID=<ssid> WIRELESS_PSK=<Wi-Fi password> cargo run --release --bin wireless_80211
```
