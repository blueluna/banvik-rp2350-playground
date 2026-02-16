# RP2350 Playground Rust Firmware

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

### Smart Led Driver

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

## Verified

 - [X] RP2350
 - [X] Flash
 - [X] PSRAM
 - [X] PWM leds
 - [X] I2S
 - [X] Buttons
 - [X] Smart led (WS2812)
 - [ ] Smart led (AP102)
 - [X] Wireless 802.11
 - [ ] Wireless Bluetooth
 - [ ] I2C / Qwiic / Stemma 1
 - [ ] I2C / Qwiic / Stemma 2
 - [ ] ADC input
 - [ ] GPIO header
 - [ ] SPI
