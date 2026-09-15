# Possible Improvements

## PCB

### Smaller screw terminals

2.54 mm screw terminals instead of 5 mm screw terminals

 - LCSC# C918124
 - LCSC# C5188446
 - LCSC# C915915

### Stereo audio

An additional MAX98357A for the other channel

 - LSCS# C910544 replace with C2682619?

379 kohm pull-up on SD_MODE for selecting right channel on one of the MAX98357A.

### I2S microphone

TDK INMP441. **Analysed, not implemented.** Feasible, but it spends the last practically-free GPIO.

The mic needs only **one** new pin. The RP2350 is already I2S controller for the amplifiers, so the
INMP441 can share `I2S_BCK` (GPIO 21) and `I2S_FSYNC` (GPIO 22); its SD line is an input to the MCU,
so nothing contends with `I2S_DIN` (GPIO 20, MCU to amps). Only SD needs a pin of its own.

**GPIO 4 (`SPI0_RX`) is the donor.** All 48 GPIO are allocated; the only pins not routed anywhere
are GPIO 4 and the GPIO 40/41 UART pair. GPIO 4 is free in practice because the display is
write-only and J19 has no MISO pin. It is also the only one that *works*: RP2350B's PIO sees a
32-pin window at a time (GPIOBASE 0 gives GPIO 0-31, 16 gives GPIO 16-47), and GPIO 4 sits in a
different window from GPIO 40/41, so those three could never serve one state machine. GPIO 4, 20,
21 and 22 all fall inside window 0-31, so one PIO instance covers the whole audio path.

Three consequences to accept before building it:

- **32-bit slots.** The INMP441 requires exactly 64 SCK per WS frame (32 per channel). The firmware
  runs `BIT_DEPTH = 16`; it must move to 32. The MAX98357A accepts 64xfs, so this is safe.
- **44.1 kHz becomes a ceiling.** 64 x 44100 = 2.8224 MHz, inside the mic's 0-3 MHz SCK range.
  48 kHz would need 3.072 MHz and exceeds spec. The host already transcodes to mono 44.1 kHz.
- **A custom PIO program is needed.** embassy-rp does ship `PioI2sIn` for `rp235xb`, but it is
  documented as "both the controller (provider of SCK and WS) and receiver" - it drives the clock
  pins itself and would fight `PioI2sOut`. An input-only program treating BCK/WS as inputs, or a
  single duplex program, is required. PIO capacity is not a constraint: PIO0 SM0 is I2S out and
  PIO1 SM1 is WS2812, across three blocks of four state machines.

Supply and mounting: the part runs on 1.8-3.3 V. Prefer a filtered branch off `+3.3V` over
`3V3_AUDIO`, which feeds the class-D amps through R48 and carries their switching noise. A 1x06
2.54 mm header for the usual breakout (matching J18 and J19) keeps a MEMS mic off the board shared
with two amplifiers and lets the port sit against an enclosure opening. Strap L/R to GND to put the
mic in the left slot, and enable the pin's internal pulldown - a lone INMP441 tri-states SD outside
its half-frame.

### Battery power

Li-po battery port (JST-PH) and Li-Po charge circuit.

```
USB-C receptacle
   │
   ├─ CC1, CC2: 5.1 kΩ Rd to GND
   ├─ ESD / TVS protection
   └─ USB_5V ── input protection / eFuse ──┐
                                            │
                                     1S switching charger
                                     + power-path manager
                                            │
                    Li-Po JST ───────── BAT│
                                            │
                                          VSYS
                                            │
              ┌─────────────────────────────┼───────────────────┐
              │                             │                   │
      3.3 V buck-boost                 5 V boost         optional 5 V
              │                             │              load switch /
            3V3                       5V_AUDIO             LC filtering
              │                             │                   │
           RP2350                 MAX98357A × 2              5V rail
```

 - BQ24074, 1S switching charger + power-path manager
 - TPS63070, 5 V boost or Silergy SY7088?
 - TPS63070 or TPS63070, 3V3
 - Li‑Po protection, DW01A + dual MOSFET

### SD-Card

Add an SD-card connector hooked up to the rp2350. For storing MP3-files.

### Components close to the Radio Module 

It is a bit hard to hand solder the RM2 since there are a screw terminal and a 0402 resistor close to the RM2.

### eInk support 

Support for FCP 24 connector and components for eInk displays.

| Pin | Name  | Description                                                                                                                                      |
| --- | ----- | ------------------------------------------------------------------------------------------------------------------------------------------------ |
| 1   | NC    | No connection. Leave open; do not tie to another NC pin.                                                                                         |
| 2   | GDR   | N-channel MOSFET gate-drive control signal used by the panel’s high-voltage power-generation circuitry.                                          |
| 3   | RESE  | Current-sense input used by the high-voltage/control loop.                                                                                       |
| 4   | NC    | No connection. Leave open.                                                                                                                       |
| 5   | VSH2  | Positive source-driver supply rail.                                                                                                              |
| 6   | TSCL  | I²C clock for the optional external digital temperature sensor.                                                                                  |
| 7   | TSDA  | I²C data for the optional external digital temperature sensor.                                                                                   |
| 8   | BS1   | Interface/bus-selection strap input. It selects the controller interface mode according to the panel/controller datasheet.                       |
| 9   | BUSY  | Display busy-status output. The controller asserts this while an update, initialization, power transition, or waveform operation is in progress. |
| 10  | RES#  | Active-low hardware reset input.                                                                                                                 |
| 11  | D/C#  | Data/command selector input: normally distinguishes command bytes from pixel/LUT/data bytes on SPI.                                              |
| 12  | CS#   | Active-low SPI chip-select input.                                                                                                                |
| 13  | SCL   | SPI serial clock input, generally connected to the MCU’s SCK.                                                                                    |
| 14  | SDA   | SPI serial data input, generally connected to the MCU’s MOSI. Despite the name, this is commonly used as SPI data rather than I²C SDA.           |
| 15  | VDDIO | Digital I/O supply for the interface pins; determines valid SPI/control logic levels.                                                            |
| 16  | VCI   | Main supply input for the display controller/analog circuitry.                                                                                   |
| 17  | VSS   | Ground / 0 V return.                                                                                                                             |
| 18  | VDD   | Internal core-logic supply rail. Depending on panel design, this may be regulated or managed by the supporting driver circuitry.                 |
| 19  | VPP   | OTP-programming supply. Usually not needed in ordinary host-MCU operation; follow the display datasheet.                                         |
| 20  | VSH1  | Additional positive source-driver high-voltage rail.                                                                                             |
| 21  | VGH   | Positive gate-driver high-voltage rail.                                                                                                          |
| 22  | VSL   | Negative source-driver supply rail.                                                                                                              |
| 23  | VGL   | Negative gate-driver high-voltage rail.                                                                                                          |
| 24  | VCOM  | Common-electrode bias voltage for the electrophoretic panel.                                                                                     |

#### On the MCU side

| PIN | Function |
| --- | --- |
| BUSY | Display busy status |
| DC | Data / Command |
| CS | Chip Select |
| SCK | Serial clock |
| DIN | Serial data in |
| RST | Reset |

### TFT display support

**Done in rev B** - J19, an 8-pin header on SPI0. See the Display section in the root `README.md`
for the pinout.

Support for display that support MIPI Display Command Set,
https://crates.io/crates/mipidsi

| PIN | Function |
| --- | --- |
| BL | Backlight (PWM) |
| DC | Data / Command |
| CS | Chip Select |
| SCK | Serial clock |
| DIN | Serial data in |
| RST | Reset |

### 5V power improvement

5V power improvement for better audio quality.

![Audio 5V 1](improvements/audio-5v-1.png)
![Audio 5V 2](improvements/audio-5v-1.png)

### Is the PWM Led drivers inverse

Investigate if the PWM LED drivers is inverse / negates the signal.

#### Designa om PWM-drivare

 - AO3400A

![PWM driver](improvements/pwm-driver.png)

### Rotary encoder support 

**Done in rev B** - SW3 and SW4 on GPIO 42-47, each channel RC-filtered.

Footprint and filter for rotary encoder.
