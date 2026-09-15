# Bånvik RP2350 Playground

A Raspberry Pi Pico 2 (RP2350B) development board.

![PCB image](rp2350-playground.png)

 - Raspberry Pi Pico RP2350B
 - 16 Mib of Flash
 - 8 Mib of PSRAM
 - USB-C power, 1S Li-Po charging with power path and cell protection
 - Interface for four arcade buttons with LEDs. Supporting PWM control of LEDs.
 - Interface for smart LEDs (WS2812/SK6812/APA102/SK9822)
 - One Qwiic/STEMMA QT port
 - Stereo speaker interface (I2S, 2 x MAX98357A)
 - microSD card slot (SPI)
 - Raspberry Pi Debug Probe compatible debug port
 - Raspberry Pi Radio Module 2 (RMC20452T)

Pinout below reflects **rev B**.

## Functions

### Buttons

4 x external buttons, one JST-XH 2-pin connector each (J20-J23). There are no switches on the
board; each connector shorts its input to GND. Every channel has a pull-up to +3.3V (R69-R72) and
an RC filter into the RP2350 (R65-R68 with C63-C66).

| Function | Connector | GPIO    |
| -------- | --------- | ------- |
| BUTTON 1 | J20       | GPIO 39 |
| BUTTON 2 | J21       | GPIO 38 |
| BUTTON 3 | J22       | GPIO 37 |
| BUTTON 4 | J23       | GPIO 36 |

### LED PWM

4 x LED PWM driver

| Function | GPIO    |
| -------- | ------- |
|    PWM 1 | GPIO 35 |
|    PWM 2 | GPIO 34 |
|    PWM 3 | GPIO 33 |
|    PWM 4 | GPIO 32 |

### Smart LED Driver

TXB0102DCT, 3.3V to 5V level translator

| Function | GPIO    |
| -------- | ------- |
|      CLK | GPIO 30 |
|      DAT | GPIO 31 |

### Qwiic / Stemma / I2C

1 x port (J2)

| Function | GPIO    |
| -------- | ------- |
|      SCL | GPIO 17 |
|      SDA | GPIO 16 |

### I2S amplifier

2 x MAX98357A I2S class-D amplifier, stereo. Both run from the 3V3 rail
(`3V3_AUDIO`, a filtered branch off +3.3V), gain 12 dB.

| Function | GPIO    |
| -------- | ------- |
|      BCK | GPIO 21 |
|     LRCK | GPIO 22 |
|     DATA | GPIO 20 |
| SHUTDOWN | GPIO 19 |

Both amplifiers share the I2S bus and the SHUTDOWN line. Channel select is by
resistor on SD_MODE: U11 is driven directly (left), U12 through 220 kOhm (right).
Driving SHUTDOWN low shuts down both.

Speaker terminal J17, 4-pos screw terminal:

| Pin | Signal    |
| --- | --------- |
|   1 | RIGHT +   |
|   2 | RIGHT -   |
|   3 | LEFT +    |
|   4 | LEFT -    |

Outputs are BTL. Do not connect either terminal of a speaker to ground.

### microSD

microSD socket (Card1) in SPI mode, on the 3V3 rail.

| Function | Net          | GPIO    |
| -------- | ------------ | ------- |
|       SO | `SPI1_RX`    | GPIO 08 |
|       CS | `~{TF_CS}`   | GPIO 09 |
|      SCK | `SPI1_SCK`   | GPIO 10 |
|       SI | `SPI1_TX`    | GPIO 11 |
|       CD | `TF_CD`      | GPIO 12 |

The SPI lines are named for the peripheral (`SPI1_*`) because the NFC reader shares them; the
card-specific signals keep the `TF_` prefix.

Card detect reaches the RP2350 on GPIO 12, pulled up to +3.3V by one element of RN2 (47k), so
insertion can be detected in firmware.

### NFC

RFID-RC522 module on an 8-pin 2.54 mm header (J18), on the 3V3 rail. It shares the microSD SPI1
bus, so only one of `~{NFC_CS}` and `~{TF_CS}` may be asserted at a time.

| J18 | Function | Net          | GPIO    |
| --- | -------- | ------------ | ------- |
|   1 |      SDA | `~{NFC_CS}`  | GPIO 26 |
|   2 |      SCK | `SPI1_SCK`   | GPIO 10 |
|   3 |     MOSI | `SPI1_TX`    | GPIO 11 |
|   4 |     MISO | `SPI1_RX`    | GPIO 08 |
|   5 |      IRQ | `NFC_IRQ`    | GPIO 27 |
|   6 |      GND |              |         |
|   7 |      RST | `NFC_RESET`  | GPIO 28 |
|   8 |    +3.3V |              |         |

SDA is the module's chip select. SCK, MOSI and MISO are the same pins the microSD socket uses; only
SDA, IRQ and RST are dedicated to the reader.

### Wireless

Raspberry Pi Radio Module 2 (RMC20452T). Same pins as Raspberry Pi Pico 2 W.

| Function | GPIO    |
| -------- | ------- |
|   WL_ON  | GPIO 23 |
|   WL_CLK | GPIO 29 |
|   WL_D   | GPIO 24 |
|   WL_CS  | GPIO 25 |

### Display

8-pin 2.54 mm header (J19) on SPI0, for a display supporting the MIPI Display Command Set.

| J19 | Function | Net              | GPIO    |
| --- | -------- | ---------------- | ------- |
|   1 |    +3.3V |                  |         |
|   2 |      GND |                  |         |
|   3 |      SCK | `SPI0_SCK`       | GPIO 02 |
|   4 |      DIN | `SPI0_TX`        | GPIO 03 |
|   5 |       CS | `~{DISPLAY_CS}`  | GPIO 01 |
|   6 |       DC | `DISPLAY_DC`     | GPIO 05 |
|   7 |      RST | `DISPLAY_RST`    | GPIO 06 |
|   8 |       BL | `DISPLAY_BL`     | GPIO 07 |

The link is write-only - there is no MISO pin on the header. GPIO 04 is reserved as `SPI0_RX` but
is not routed anywhere.

### Rotary encoders

2 x rotary encoders with integrated push switch (SW3, SW4). Each A/B channel is RC-filtered
(R55-R64 with C59-C62); the switches are pulled up and filtered the same way.

| Function   | GPIO    |
| ---------- | ------- |
| ENC 1 A    | GPIO 42 |
| ENC 1 B    | GPIO 43 |
| ENC 1 SW   | GPIO 44 |
| ENC 2 A    | GPIO 45 |
| ENC 2 B    | GPIO 46 |
| ENC 2 SW   | GPIO 47 |

These were ADC2-ADC7 on the rev A ADC header.

### Debug UART

GPIO 40 (`UART_TX`) and GPIO 41 (`UART_RX`) are reserved for a debug UART but are not routed to any
connector. These were ADC0 and ADC1 on the rev A ADC header.

### GPIO allocation

**All 48 GPIO are allocated.** The only pins not routed to anything are GPIO 04 (`SPI0_RX`, the
display has no MISO) and the GPIO 40/41 UART pair - so any new peripheral has to take one of those
or displace an existing function. See `pcb/improvements.md` for how this plays out for the proposed
I2S microphone.

The 2x8 GPIO header (J6) and the ADC header (J7) from rev A have both been removed.

### Power

USB-C (J14) is the only external supply input. It feeds a BQ24074 charger and
power-path manager (U7), which also charges a 1S Li-Po on the JST-PH connector
(J16). The battery has DW01A + FS8205A protection (U8, Q5).

The charger output, VSYS, feeds two TPS63070 buck-boost converters:

| Rail  | Converter | Voltage | Feeds                                            |
| ----- | --------- | ------- | ------------------------------------------------ |
| +3.3V | U9        | 3.30 V  | RP2350, flash, PSRAM, RM2, microSD, NFC and display headers |
| 3V3_AUDIO | (branch off +3.3V via R48) | 3.30 V | Both MAX98357A (U11, U12)   |
| +5V   | U10       | 5.09 V  | Smart LED connector (J8), TXB0102 VCCB           |

Charger settings: input current limit ~1.4 A (R37 1.1 kOhm), fast charge 890 mA
(R39 1 kOhm). VSYS is the system ceiling, not the 2 A rating of either converter.

The RP2350's ADC_AVDD pin is still filtered from +3.3V by a 33 Ohm / 1 uF / 100 nF network
(R6, C17, C18), even though no ADC pin is now used as an analog input.

J15 is a 4-pin 1.00 mm header carrying +3.3V and GND.

### System control

| Function        | GPIO    | Notes                                          |
| --------------- | ------- | ---------------------------------------------- |
| 3V3_POWER_SAVE  | GPIO 14 | U9 PS/SYNC. High = power save (PFM), low = forced PWM. Pulled high by default; drive low while audio plays. |
| 3V3_GOOD        | GPIO 15 | U9 power good, open drain, pulled to +3.3V.    |
| 5V_GOOD         | GPIO 13 | U10 power good, open drain, pulled to +3.3V.   |
| 5V_POWER_SAVE   | GPIO 18 | U10 PS/SYNC. High = power save (PFM), low = forced PWM. Pulled high by default; drive low under heavy smart-LED load. |

## Board bring up

### Rev A

 - [X] RP2350
 - [X] External Oscillator
 - [X] Flash
 - [X] PSRAM
 - [X] PWM leds
 - [X] I2S
 - [X] Buttons
 - [X] Smart led (WS2812)
 - [ ] Smart led (APA102)
 - [X] Wireless 802.11
 - [ ] Wireless Bluetooth
 - [X] I2C / Qwiic / Stemma 1
 - [X] I2C / Qwiic / Stemma 2
 - [ ] ADC input
 - [X] GPIO header
 - [X] SPI
 - [ ] USB

### Rev B

Not built yet.

 - [ ] Battery charging and power path
 - [ ] Battery protection
 - [ ] Buck-boost rails
 - [ ] Stereo audio, both channels
 - [ ] Audio on 3V3 rail
 - [ ] microSD card
 - [ ] Power save control
 - [ ] NFC reader (J18)
 - [ ] Display (J19)
 - [ ] Rotary encoders

## Rust firmware

See [firmware](firmware/README.md).