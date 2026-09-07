# Possible Improvements

## PCB

### Smaller screw terminals

2.54 mm screw terminals instead of 5 mm screw terminals

LCSC# C918124
LCSC# C5188446
LCSC# C915915

### Stereo audio

An additional MAX98357A for the other channel

LSCS# C910544 replace with C2682619?

379 kohm pull-up on SD_MODE for selecting right channel on one of the MAX98357A.

### Battery power

Li-po battery port (JST-PH) and Li-Po charge circuit.

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

### Is the PWM Led drivers inverse

Investigate if the PWM LED drivers is inverse / negates the signal.

#### Designa om PWM-drivare

 - AO3400A

![PWM driver](improvements/pwm-driver.png)

### Rotary encoder support 

Footprint and filter for rotary encoder.
