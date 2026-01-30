# RP2350 Playground Rust Firmware

```shell
probe-rs download --probe 2e8a:000c firmware/43439A0.bin --binary-format bin --chip RP235x --base-address 0x10100000
probe-rs download --probe 2e8a:000c firmware/43439A0_clm.bin --binary-format bin --chip RP235x --base-address 0x10140000
```

## Hookup


| 40-pin  | Pico 2 W | Description |
| ------- | -------- | ------------|
|       1 |      3V3 | Power supply |
|       6 |      GND | Ground |
|      19 |      GP3 | SPI controller output |
|      23 |      GP2 | SPI serial clock |
|      24 |      GP5 | SPI chip select |
|      15 |      GP6 | Data/Command |
|      13 |      GP7 | Display Reset |
|      11 |      GP8 | Display busy |

