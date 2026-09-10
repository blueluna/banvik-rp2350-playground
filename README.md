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

4 x buttons with debouncing

| Function | GPIO    |
| -------- | ------- |
| BUTTON 1 | GPIO 39 |
| BUTTON 2 | GPIO 38 |
| BUTTON 3 | GPIO 37 |
| BUTTON 4 | GPIO 36 |

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

| Function | GPIO    |
| -------- | ------- |
|       SO | GPIO 08 |
|       CS | GPIO 09 |
|      SCK | GPIO 10 |
|       SI | GPIO 11 |

Card detect is brought out on the socket but is not connected to the RP2350.

### Wireless

Raspberry Pi Radio Module 2 (RMC20452T). Same pins as Raspberry Pi Pico 2 W.

| Function | GPIO    |
| -------- | ------- |
|   WL_ON  | GPIO 23 |
|   WL_CLK | GPIO 29 |
|   WL_D   | GPIO 24 |
|   WL_CS  | GPIO 25 |

### GPIO header

2x8 header (J6). GPIO 08-11 are no longer on the header; they drive the
microSD socket.

| Pin | Function | GPIO    |
| --- | -------- | ------- |
|   1 |        G |         |
|   2 |      3V3 |         |
|   3 |       5V |         |
|   4 |        1 | GPIO 01 |
|   5 |        2 | GPIO 02 |
|   6 |        3 | GPIO 03 |
|   7 |        4 | GPIO 04 |
|   8 |        5 | GPIO 05 |
|   9 |        6 | GPIO 06 |
|  10 |        7 | GPIO 07 |
|  11 |       NC |         |
|  12 |       NC |         |
|  13 |       NC |         |
|  14 |       NC |         |
|  15 |       12 | GPIO 12 |
|  16 |       13 | GPIO 13 |

### ADC header

Not necessarily for ADC input.

ADC_AVDD is filtered from +3.3V by a 33 Ohm / 1 uF / 100 nF network (R6, C17, C18).

| Function | GPIO    |
| -------- | ------- |
|        G |         |
|      3V3 |         |
|       A0 | GPIO 40 |
|       A1 | GPIO 41 |
|       A2 | GPIO 42 |
|       A3 | GPIO 43 |

### Power

USB-C (J14) is the only external supply input. It feeds a BQ24074 charger and
power-path manager (U7), which also charges a 1S Li-Po on the JST-PH connector
(J16). The battery has DW01A + FS8205A protection (U8, Q5).

The charger output, VSYS, feeds two TPS63070 buck-boost converters:

| Rail  | Converter | Voltage | Feeds                                            |
| ----- | --------- | ------- | ------------------------------------------------ |
| +3.3V | U9        | 3.30 V  | RP2350, flash, PSRAM, RM2, microSD, headers      |
| 3V3_AUDIO | (branch off +3.3V via R48) | 3.30 V | Both MAX98357A (U11, U12)   |
| +5V   | U10       | 5.09 V  | Smart LED connector, TXB0102 VCCB, header pin 3  |

Charger settings: input current limit ~1.4 A (R37 1.1 kOhm), fast charge 890 mA
(R39 1 kOhm). VSYS is the system ceiling, not the 2 A rating of either converter.

J15 is a 4-pin 1.00 mm header carrying +3.3V and GND.

### System control

| Function        | GPIO    | Notes                                          |
| --------------- | ------- | ---------------------------------------------- |
| 3V3_POWER_SAVE  | GPIO 14 | U9 PS/SYNC. High = power save (PFM), low = forced PWM. Pulled high by default; drive low while audio plays. |
| 3V3_GOOD        | GPIO 15 | U9 power good, open drain, pulled to +3.3V.    |
| 5V_GOOD         | GPIO 18 | U10 power good, open drain, pulled to +3.3V.   |

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

## Rust firmware

See [firmware](firmware/README.md).