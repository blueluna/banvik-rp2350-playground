# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository layout

Three unrelated build systems live side by side; there is no top-level build.

| Directory | What it is | Build system |
|---|---|---|
| `pcb/` | KiCad 10 hardware design for the board | KiCad / `kicad-cli` |
| `firmware/` | Embedded Rust for the RP2350 (Embassy async) | Cargo workspace |
| `host/` | Desktop Rust — MP3 streaming server the firmware talks to | Cargo workspace |
| `rp2350-music/` | Untracked KiCad spin-off at the repo root, not in git — **not** the same thing as the `rp2350-music` project inside `pcb/` | — |

`README.md` at the repo root holds the **authoritative GPIO pinout and bring-up status**. The
pinout table in `firmware/CLAUDE.md` is a subset derived from the examples; when they disagree,
the root README is the hardware truth.

## Firmware (`firmware/`)

`firmware/CLAUDE.md` covers the Embassy patterns, audio pipeline, PSRAM heap and per-example run
commands. Read it before touching anything under `firmware/`. Points it does not cover:

**The workspace currently does not resolve.** `examples/Cargo.toml` has path dependencies on
`../rnfc/rnfc` and `../rnfc/rnfc-st25r39`, but `firmware/rnfc/` is absent and gitignored. Every
`cargo` command in `firmware/` — including `cargo test -p rp2350-playground` — fails at manifest
load. Fix before anything else:

```bash
git clone https://github.com/embassy-rs/rnfc firmware/rnfc
```

Alternatively drop the two `rnfc*` deps and `examples/src/bin/nfc_st25r3918.rs` together.

**Tests.** All tests live in `firmware/rp2350-playground/` (`nfc/crypto1.rs`, `nfc/ndef.rs`,
`nfc/tlvtag.rs`). `.cargo/config.toml` pins the build target to `thumbv8m.main-none-eabihf`, so a
host target must be given explicitly:

```bash
cd firmware
cargo test -p rp2350-playground --target x86_64-unknown-linux-gnu
cargo test -p rp2350-playground --target x86_64-unknown-linux-gnu ndef::tests::decode_text   # single test
```

**Env vars** come from `firmware/.cargo/config.toml` (`DEFMT_LOG`, `SSID`, `WIRELESS_PSK`,
`STREAM_HOST`, `STREAM_PORT`) and are read with `env!()` at compile time — changing them requires a
rebuild, and an empty `SSID` compiles fine but fails at runtime.

`firmware/CLAUDE.md` predates the NFC stack (`rp2350-playground/src/nfc/`, the three `nfc_*`
examples), `mp3_sd_card_player.rs` and `mp3_stream_client.rs` — worth extending when you work there.

## Host (`host/`)

```bash
cd host
cargo run -p mp3-server -- /path/to/music-dir          # control on 6564, stream on 6565
cargo run -p mp3-server -- /path/to/music-dir -p 7000  # stream port is always port+1
```

`host/mp3-server/specification.md` is the design brief the server was written against and still
describes intended behaviour.

On startup the server scans the directory, hashes each file (SHA-256), reads the ID3 title, and
**eagerly transcodes everything to mono 44.1 kHz VBR MP3** into a cache (`transcode.rs`, via
symphonia + mp3lame). Mono is not a preference — the on-device `nanomp3` decoder only handles mono.
Input may be mp3/aac/m4a; output is always mono MP3.

`library.rs` maps NFC tag UIDs to songs via `song-mappings.toml` in the XDG config dir
(`ProjectDirs("com", "example", "mp3-server")`); unknown files get `UNKNOWN_UID` and the file is
rewritten so tags can be assigned later. This is how `nfc_*` examples and `mp3_stream_client`
connect: scan a tag → `PlayUid` → stream.

## The cross-cutting piece: `mp3-protocol`

`host/mp3-protocol/` is a `no_std`, alloc-free crate (`serde` + `postcard` + `heapless`) used by
**both** sides. The firmware depends on it by relative path
(`firmware/examples/Cargo.toml`: `mp3-protocol = { path = "../../host/mp3-protocol" }`), so editing
it changes firmware and server together, and the `defmt` feature flag exists purely for the firmware
side. Wire format: 4-byte big-endian length prefix + postcard body, on two TCP sockets (control on
`port`, bulk audio on `port + 1`). `heapless::Vec` capacities in `SongEntry`/`StreamChunk` are part
of the wire contract — changing one breaks the other side silently.

## PCB (`pcb/`)

The KiCad project is **`rp2350-music`** (renamed from `rp-lights`), so the root sheet is
`pcb/rp2350-music.kicad_sch`. It instantiates six sub-sheets:

| Sheetname | File | Contents |
|---|---|---|
| RP2350 | `sheets/rp2350.kicad_sch` | MCU, RM2 radio, crystal, debug header |
| Power | `sheets/power.kicad_sch` | USB-C, BQ24074 charger, battery protection, two TPS63070 rails |
| Audio | `sheets/audio.kicad_sch` | Two MAX98357A amps, output filter, speaker terminal |
| Storage | `sheets/storage.kicad_sch` | microSD socket, QSPI flash (U3) and PSRAM (U4) |
| Video & NFC | `sheets/video_nfc.kicad_sch` | RFID-RC522 header (J18), display header (J19) |
| Buttons & Light | `sheets/control-and-lights.kicad_sch` | Button connectors (J20-J23), LED PWM drivers, smart-LED level translator, two rotary encoders |

Flash and PSRAM live on the **Storage** sheet, not the MCU sheet, which is why the QSPI bus
`{SCK,IO0,IO1,IO2,IO3,~{CS0},~{CS1}}` crosses from RP2350 to Storage.

All six sub-sheets live in `pcb/sheets/`; only the root sheet sits at `pcb/`. The MCU sheet was
`rp-lights.kicad_sch` before the hierarchy work — anything referring to that filename is stale.

**Inter-sheet signals travel as bus groups on the sheet pins.** The current set is:

```
{SCK,IO0,IO1,IO2,IO3,~{CS0},~{CS1}}                                  RP2350 <-> Storage (QSPI)
{~{TF_CS},SPI1_SCK,SPI1_RX,SPI1_TX,TF_CD}                            RP2350 <-> Storage (microSD)
{SPI1_SCK,SPI1_TX,SPI1_RX,~{NFC_CS},NFC_IRQ,NFC_RESET}               RP2350 <-> Video & NFC
{SPI0_SCK,SPI0_TX,SPI0_RX,~{DISPLAY_CS},DISPLAY_DC,DISPLAY_RST,DISPLAY_BL}   RP2350 <-> Video & NFC
{I2S_BCK,I2S_FSYNC,I2S_DIN,~{AUDIO_SHUTDOWN}}                        RP2350 <-> Audio
{3V3_GOOD,3V3_POWER_SAVE,5V_GOOD,5V_POWER_SAVE}                      RP2350 <-> Power
{BTN1,BTN2,BTN3,BTN4} / {BTN1_PWM,BTN2_PWM,BTN3_PWM,BTN4_PWM}        RP2350 <-> Buttons & Light
{ENC_1_A,ENC_1_B,ENC_1_SW,ENC_2_A,ENC_2_B,ENC_2_SW}                  RP2350 <-> Buttons & Light
{SMART_LED_CK,SMART_LED_DA}                                          RP2350 <-> Buttons & Light
```

Adding a cross-sheet signal means editing the sheet pin on the root **and** the matching
hierarchical label inside the subsheet — the two strings must agree. Rails cross as power symbols
instead (`+3.3V`, `+5V`, `3V3_AUDIO`, `VBUS`, `GND`), not through sheet pins.

Bus **members** also merge with identically-named plain labels elsewhere on the same sheet, which is
how the NFC bus reaches J18: the RC522 shares the microSD SPI1 lines and only adds `~{NFC_CS}`,
`NFC_IRQ` and `NFC_RESET` on GPIO 26/27/28. Only one of `~{NFC_CS}` and `~{TF_CS}` may be asserted
at a time.

**A name mismatch between a bus member and a wire label is a warning, not an error.** If the bus
says `~{TF_CS}` and the wire on it is labelled `TF_CS`, KiCad reports `net_not_bus_member` and ERC
still shows zero errors — but the signal never crosses the sheet boundary. A comma typo inside a
group (`{BTN1,BTN2.BTN3,BTN4}`) is worse: it is *consistent*, so it produces no error at all while
silently merging two signals into one dead net. Both have bitten this design. After any bus edit,
check the exported netlist, not the ERC error count:

```bash
python3 -c "
import xml.etree.ElementTree as ET
r=ET.parse('/tmp/net.xml').getroot()
print([n.get('name') for n in r.find('nets').findall('net')
       if len(n.findall('node'))==1 and not n.get('name').startswith('unconnected-')])"
```

Expect only `SPI0_RX`, `UART_TX` and `UART_RX` — the three deliberately unrouted pins.

**All KiCad files are Git LFS pointers** (see the root `.gitattributes`: `.kicad_sch`, `.kicad_pcb`,
`.kicad_sym`, `.kicad_mod`, `.kicad_pro`, plus `.pdf`/`.png`/`.zip`/`.bin`/`.mp3`/`.mod`). They are
plain text on disk but `git diff` shows pointer churn, not schematic changes.

**Read connectivity via netlist export, not by parsing s-expressions** — it resolves labels, power
symbols and hierarchy for you:

```bash
cd pcb
kicad-cli sch export netlist --format kicadxml -o /tmp/net.xml rp2350-music.kicad_sch
kicad-cli sch export netlist --format kicadxml -o /tmp/audio.xml sheets/audio.kicad_sch   # sheets export standalone too
```

Note this is read-only but does write the output file; keep exports out of the project directory
(`*.xml`, `*.csv`, `*.net` are gitignored anyway).

**Symbol/footprint libraries** come from two places: project-local ones registered in
`pcb/sym-lib-table` / `pcb/fp-lib-table`, and `PCM_JLCPCB-*` libraries that resolve only through the
user's **global** KiCad tables (`~/.config/kicad/10.0/`). A checkout alone will not open cleanly
without that plugin installed.

Known gap: the schematics reference `lcsc_footprints:` and `lcsc_imported:`, and the files exist
under `pcb/libs/lcsc/`, but **neither nickname is registered in the project tables**. That leaves
eight broken footprint links (RN1, RN2, Card1, U9, U10, L2, L3, Q5) and will block the PCB update
until the two `(lib ...)` entries are added back.

**Rev B was re-annotated after the hierarchy was linked, so every reference designator changed.**
Anything citing pre-rev-B refs — git history and older notes — is keyed to the
old numbering and will mislead. Current numbering is sequential per sheet: RP2350 C1-C27, Power
C28-C50, Audio C51-C58. A few anchors:

| Part | Ref |
|---|---|
| RP2350 / level shifter / flash / PSRAM / RM2 | U1 / U2 / U3 / U4 / U5 |
| USBLC6 / BQ24074 / DW01A | U6 / U7 / U8 |
| TPS63070 +3.3V / +5V | U9 / U10 |
| MAX98357A right / left | U11 / U12 |
| `3V3_AUDIO` 0 ohm link | R48 |
| SD_MODE channel-select | R54 (220k) |
| USB-C / battery JST / speaker terminal | J14 / J16 / J17 |

Channel select: U12's `SD_MODE` sits directly on `~{AUDIO_SHUTDOWN}` (3.3 V, Left band), while U11
reaches it through R54 220k, dividing against the amp's internal 100k pulldown to ≈1.03 V (Right
band). So **U11 is Right and U12 is Left**, matching the text on the Audio sheet.

The MAX98357A amps have **already been moved off +5V**: both run from `3V3_AUDIO`, branched off
`+3.3V` through R48. That leaves the +5V rail (U10) feeding only the smart-LED terminal J8, the
TXB0102 VCCB and J6 pin 3.

Design notes: `pcb/improvements.md` (backlog), with its images under `pcb/improvements/`. The
`_restore_backup_*/` and `.history/` directories are editor artefacts, not design history — ignore
them.

## Codex config detected

A `~/.codex/config.toml` exists. Reply `/import` to scan it and list what can be brought over (MCP
servers, slash commands, subagents, skills, instructions), then `/import --yes=<digest>` using the
digest the scan prints. If `/import` is unavailable here, run `claude import` from a terminal.
