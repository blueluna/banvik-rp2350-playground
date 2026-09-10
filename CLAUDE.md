# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository layout

Three unrelated build systems live side by side; there is no top-level build.

| Directory | What it is | Build system |
|---|---|---|
| `pcb/` | KiCad 10 hardware design for the board | KiCad / `kicad-cli` |
| `firmware/` | Embedded Rust for the RP2350 (Embassy async) | Cargo workspace |
| `host/` | Desktop Rust — MP3 streaming server the firmware talks to | Cargo workspace |
| `rp2350-music/` | Untracked KiCad spin-off of `pcb/`, not in git | — |

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

The KiCad project is named **`rp-lights`**, not after the repo or the board. Root sheet is
`pcb/rp-lights.kicad_sch`; `power`, `audio` and `storage` are separate `.kicad_sch` files.

**All KiCad files are Git LFS pointers** (see the root `.gitattributes`: `.kicad_sch`, `.kicad_pcb`,
`.kicad_sym`, `.kicad_mod`, `.kicad_pro`, plus `.pdf`/`.png`/`.zip`/`.bin`/`.mp3`/`.mod`). They are
plain text on disk but `git diff` shows pointer churn, not schematic changes.

**Read connectivity via netlist export, not by parsing s-expressions** — it resolves labels, power
symbols and hierarchy for you:

```bash
cd pcb
kicad-cli sch export netlist --format kicadxml -o /tmp/net.xml rp-lights.kicad_sch
kicad-cli sch export netlist --format kicadxml -o /tmp/audio.xml audio.kicad_sch   # sheets export standalone too
```

Note this is read-only but does write the output file; keep exports out of the project directory
(`*.xml`, `*.csv`, `*.net` are gitignored anyway).

**Symbol/footprint libraries** come from two places: project-local ones registered in
`pcb/sym-lib-table` / `pcb/fp-lib-table` (`MCU_RaspberryPi_RP2350`, `RPI_RMC20452T`,
`lcsc_imported`, `lcsc_footprints` under `pcb/libs/lcsc/`), and `PCM_JLCPCB-*` libraries that resolve
only through the user's **global** KiCad tables (`~/.config/kicad/10.0/`). A checkout alone will not
open cleanly without that plugin installed.

**Rev B is mid-refactor.** `power`, `audio` and `storage` are not yet instantiated as sheet symbols
in `rp-lights.kicad_sch`, so a netlist export of the root sheet sees only the MCU. Design notes:
`pcb/improvements.md` (backlog) and `pcb/audio_3v3.md` (analysis of moving the MAX98357A amps to
3.3 V, including the current power budget and known schematic bugs). The `_restore_backup_*/` and
`.history/` directories are editor artefacts, not design history — ignore them.

## Codex config detected

A `~/.codex/config.toml` exists. Reply `/import` to scan it and list what can be brought over (MCP
servers, slash commands, subagents, skills, instructions), then `/import --yes=<digest>` using the
digest the scan prints. If `/import` is unavailable here, run `claude import` from a terminal.
