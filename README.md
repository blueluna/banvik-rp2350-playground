# Bånvik RP2350 Playground

A Raspberry Pi Pico 2 (RP2350B) development board.

![PCB image](rp2350-playground.png)

 - Raspberry Pi Pico RP2350B
 - 16 Mib of Flash
 - 8 Mib of PSRAM
 - Power through USB-C
 - Interface for four arcade buttons with LEDs. Supporting PWM control of LEDs.
 - Interface for smart LEDs (WS2812/SK6812/APA102/SK9822)
 - Two Qwiic/STEMMA QT ports
 - Speaker interface (I2S)
 - Raspberry Pi Debug Probe compatible debug port
 - Raspberry Pi Radio Module 2 (RMC20452T)

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

2 x ports

| Function | GPIO    |
| -------- | ------- |
|    SCL 1 | GPIO 17 |
|    SDA 1 | GPIO 16 |
|    SCL 2 | GPIO 15 |
|    SDA 2 | GPIO 14 |

### I2S amplifier

MAX98357A, I2S mono audio amplifier

Only left channel.

| Function | GPIO    |
| -------- | ------- |
|      BCK | GPIO 21 |
|     LRCK | GPIO 22 |
|     DATA | GPIO 20 |
| SHUTDOWN | GPIO 19 |

### Wireless

Raspberry Pi Radio Module 2 (RMC20452T). Same pins as Raspberry Pi Pico 2 W.

| Function | GPIO    |
| -------- | ------- |
|   WL_ON  | GPIO 23 |
|   WL_CLK | GPIO 29 |
|   WL_D   | GPIO 24 |
|   WL_CS  | GPIO 25 |

### GPIO header

| Function | GPIO    |
| -------- | ------- |
|        G |         |
|      3V3 |         |
|       5V |         |
|        1 | GPIO 01 |
|        2 | GPIO 02 |
|        3 | GPIO 03 |
|        4 | GPIO 04 |
|        5 | GPIO 05 |
|        6 | GPIO 06 |
|        7 | GPIO 07 |
|        8 | GPIO 08 |
|        9 | GPIO 09 |
|       10 | GPIO 10 |
|       11 | GPIO 11 |
|       12 | GPIO 12 |
|       13 | GPIO 13 |

### ADC header

Not necessarily for ADC input.

| Function | GPIO    |
| -------- | ------- |
|        G |         |
|      3V3 |         |
|       A0 | GPIO 40 |
|       A1 | GPIO 41 |
|       A2 | GPIO 42 |
|       A3 | GPIO 43 |

## Board bring up

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
 - [ ] GPIO header
 - [ ] SPI
 - [ ] USB

## Rust firmware

See [firmware](firmware/README.md).