#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::pwm::{Pwm, SetDutyCycle};
use embassy_rp::spi;
use embassy_time::{Duration, Timer};
use embedded_alloc::LlffHeap as Heap;
use embedded_hal_bus::spi::ExclusiveDevice;
use rnfc::iso14443a::Poller;
use rnfc::traits::iso14443a::Reader as Iso14443aReader;
use rnfc_st25r39::{DriverResistance, SpiInterface, St25r39};

use {defmt_rtt as _, panic_probe as _};

fn rnfc_st25r39_to_str<T: core::fmt::Debug>(
    error: &rnfc_st25r39::iso14443a::Error<T>,
) -> &'static str {
    match error {
        rnfc_st25r39::iso14443a::Error::Timeout => "Timeout",
        rnfc_st25r39::iso14443a::Error::Framing => "Framing error",
        rnfc_st25r39::iso14443a::Error::FramingLastByteMissingParity => {
            "Framing error (last byte missing parity)"
        }
        rnfc_st25r39::iso14443a::Error::Crc => "CRC error",
        rnfc_st25r39::iso14443a::Error::Collision => "Collision error",
        rnfc_st25r39::iso14443a::Error::Parity => "Parity error",
        rnfc_st25r39::iso14443a::Error::ResponseTooShort => "Response too short",
        rnfc_st25r39::iso14443a::Error::ResponseTooLong => "Response too long",
        rnfc_st25r39::iso14443a::Error::FifoOverflow => "FIFO overflow",
        rnfc_st25r39::iso14443a::Error::FifoUnderflow => "FIFO underflow",
        rnfc_st25r39::iso14443a::Error::Interface(_) => "Interface error",
    }
}

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"ST25R3918 NFC Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"An example that reads NFC cards using the ST25R3918 via SPI."
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let mut rp_configuration: embassy_rp::config::Config = Default::default();
    rp_configuration.clocks = embassy_rp::clocks::ClockConfig::crystal(12_000_000);
    let peripherals = embassy_rp::init(rp_configuration);

    // PWM LED setup
    let mut pwm_config: embassy_rp::pwm::Config = Default::default();
    pwm_config.top = 32_768;
    let pwm_2_1 = Pwm::new_output_ab(
        peripherals.PWM_SLICE9,
        peripherals.PIN_34,
        peripherals.PIN_35,
        pwm_config,
    );
    let (pwm_2, pwm_1) = pwm_2_1.split();
    if let Some(mut pwm_1) = pwm_1 {
        let _ = pwm_1.set_duty_cycle(16_383);
    }
    if let Some(mut pwm_2) = pwm_2 {
        let _ = pwm_2.set_duty_cycle(32_767);
    }

    let mut pwm_config: embassy_rp::pwm::Config = Default::default();
    pwm_config.top = 32_768;
    let pwm_4_3 = Pwm::new_output_ab(
        peripherals.PWM_SLICE8,
        peripherals.PIN_32,
        peripherals.PIN_33,
        pwm_config,
    );
    let (pwm_4, pwm_3) = pwm_4_3.split();
    if let Some(mut pwm_3) = pwm_3 {
        let _ = pwm_3.set_duty_cycle(0);
    }
    if let Some(mut pwm_4) = pwm_4 {
        let _ = pwm_4.set_duty_cycle(32_767);
    }

    let _audio_shutdown = Output::new(peripherals.PIN_19, Level::Low);

    // SPI configuration for ST25R3918 (SPI Mode 1: CPOL=0, CPHA=1)
    let mut spi_config = spi::Config::default();
    spi_config.frequency = 5_000_000; // 5 MHz
    spi_config.polarity = spi::Polarity::IdleLow;
    spi_config.phase = spi::Phase::CaptureOnSecondTransition;

    // SPI0 pins
    let spi_bus = spi::Spi::new_blocking(
        peripherals.SPI0,
        peripherals.PIN_2, // SCK
        peripherals.PIN_3, // MOSI (TX)
        peripherals.PIN_4, // MISO (RX)
        spi_config,
    );

    let cs = Output::new(peripherals.PIN_1, Level::High);
    let spi_device = ExclusiveDevice::new_no_delay(spi_bus, cs).unwrap();
    let iface = SpiInterface::new(spi_device);

    // IRQ pin from ST25R3918
    let irq = Input::new(peripherals.PIN_5, Pull::None);

    defmt::info!("Initializing ST25R3918...");

    let mut st = St25r39::new(iface, irq)
        .await
        .expect("Failed to initialize ST25R3918");

    let mut config = rnfc_st25r39::Config::new();
    config.driver_resistance = DriverResistance::Ohm1; // max power
    st.set_config(config).unwrap();

    defmt::info!("ST25R3918 initialized, scanning for cards...");

    loop {
        Timer::after(Duration::from_millis(500)).await;

        let iso14 = match st.start_iso14443a().await {
            Ok(iso14) => iso14,
            Err(_) => {
                defmt::error!("Failed to start ISO14443A");
                continue;
            }
        };

        let mut poller = Poller::new(iso14);

        let cards = match poller.search::<4>().await {
            Ok(cards) => cards,
            Err(_) => {
                defmt::warn!("Card search failed");
                continue;
            }
        };

        if cards.is_empty() {
            continue;
        }

        defmt::info!("Found {} card(s)", cards.len());

        let mut crypto1 = rp2350_playground::nfc::crypto1::Crypto1State::default();

        for uid in &cards {
            let mut card = match poller.select_by_id(uid).await {
                Ok(card) => card,
                Err(_) => {
                    defmt::warn!("Failed to select card {:02x}", uid.as_slice());
                    continue;
                }
            };

            let atqa = card.atqa();
            let sak = card.sak();
            let uid_len = card.uid().len();
            let mut picc_uid = [0; 10];
            picc_uid[..uid_len].copy_from_slice(card.uid());
            let uid = &picc_uid[..uid_len];

            let target_type = rp2350_playground::nfc::TargetType::from_response(sak, uid_len as u8);
            defmt::info!(
                "Card UID: {:02x} ATQA: {:02x} SAK: {:02x} Target Type: {}",
                uid,
                atqa,
                sak,
                target_type
            );

            if let Ok(vendor) = rp2350_playground::nfc::vendor::Vendor::try_from(uid[0]) {
                defmt::info!("Vendor: {}", vendor);
            }

            let mut response = [0u8; 16];

            match card.transceive(&[0x60], &mut response, 65536).await {
                Ok(len) => {
                    if let Some(version) = rp2350_playground::nfc::nxp::VersionResponse::parse(&response[..len]) {
                        defmt::info!("Chip: {}", version.chip().map_or("", |c| { c.to_str() }));
                    }
                }
                Err(_) => {
                    defmt::error!("Failed to read version 0");
                    continue;
                }
            }


            if uid_len == 7 {
                // NTAG / Ultralight - NFC Forum Type 2 Tag
                // READ command (0x30) returns 4 pages (16 bytes) starting at given page
                // Pages 0-1: UID, Page 2: lock bytes, Page 3: Capability Container (CC)
                let length = match card.transceive(&[0x30, 0x00], &mut response, 65536).await {
                    Ok(len) => {
                        len
                    }
                    Err(_) => {
                        0
                    }
                };

                if length < 16 {
                    defmt::warn!("Unexpectedly short response for page 0: {} bytes", length);
                    continue;
                }
                // CC is at page 3 (bytes 12-15 of the first READ)
                let cc_magic = response[12];
                let cc_version = response[13];
                let cc_size = response[14] as usize * 8; // data area size in bytes
                let cc_access = response[15];
                defmt::info!(
                    "CC: magic={=u8:02x} version={=u8:02x} size={} access={=u8:02x}",
                    cc_magic,
                    cc_version,
                    cc_size,
                    cc_access
                );

                if cc_magic != 0xE1 || cc_size == 0 {
                    defmt::warn!("Not an NDEF tag or empty");
                    continue;
                }

                // Read user data pages (starting at page 4)
                // Each READ returns 4 pages = 16 bytes
                let user_bytes = cc_size.min(888); // cap at NTAG216 max
                let mut memory = [0u8; 888];
                let mut offset = 0;
                let mut page = 4u8;

                while offset < user_bytes {
                    let mut buf = [0u8; 16];
                    match card.transceive(&[0x30, page], &mut buf, 65536).await {
                        Ok(len) => {
                            if len == 16 {
                                let copy_len = len.min(user_bytes - offset);
                                memory[offset..offset + copy_len].copy_from_slice(&buf[..copy_len]);
                            } else if len == 1 {
                                defmt::warn!("NAK? {}: {} bytes {=u8:02x}", page, len, buf[0]);
                            }
                            offset += 16;
                            page = page.saturating_add(4);
                        }
                        Err(_) => {
                            defmt::warn!("Read failed at page {}, got {} bytes", page, offset);
                            break;
                        }
                    }
                }

                let valid_len = offset.min(user_bytes);

                for (i, chunk) in memory[..valid_len].chunks(16).enumerate() {
                    defmt::info!("Memory [{=u8:02x}] {=[u8]:02x}", (i * 4 + 4) as u8, chunk);
                }

                // TLV / NDEF decode
                let mut tlv_tag_decoder =
                    rp2350_playground::nfc::tlvtag::TlvDecoder::new(&memory[..valid_len]);
                loop {
                    match tlv_tag_decoder.next() {
                        Some(Ok(tlv)) => match tlv.tag {
                            rp2350_playground::nfc::tlvtag::TlvTag::NdefMessage => {
                                let mut ndef_decoder =
                                    rp2350_playground::nfc::ndef::NdefDecoder::new(&tlv.value);
                                while let Some(record) = ndef_decoder.next() {
                                    match record {
                                        Ok(record) => {
                                            defmt::info!(
                                                "  NDEF Record {} payload_len={} id={}",
                                                record.record_type,
                                                record.payload.len(),
                                                record.id
                                            );
                                            match record.record_type {
                                                rp2350_playground::nfc::ndef::RecordType::Text => {
                                                    if let Ok(text) = record.text() {
                                                        defmt::info!("    {}", text.text);
                                                    } else {
                                                        defmt::error!(
                                                            "    Text record failed to parse"
                                                        );
                                                    }
                                                }
                                                _ => {
                                                    defmt::info!(
                                                        "    Record type: {:?}",
                                                        record.record_type
                                                    );
                                                }
                                            }
                                        }
                                        Err(e) => {
                                            defmt::error!("  NDEF parse error: {:?}", e);
                                        }
                                    }
                                }
                            }
                            rp2350_playground::nfc::tlvtag::TlvTag::LockControl => {
                                defmt::info!("  Lock Control TLV, length {}", tlv.value.len());
                            }
                            rp2350_playground::nfc::tlvtag::TlvTag::MemoryControl => {
                                defmt::info!("  Memory Control TLV, length {}", tlv.value.len());
                            }
                            rp2350_playground::nfc::tlvtag::TlvTag::Proprietary => {
                                defmt::info!("  Proprietary TLV, length {}", tlv.value.len());
                            }
                            _ => (),
                        },
                        Some(Err(e)) => {
                            defmt::error!("  TLV parse error: {:?}", e);
                            break;
                        }
                        None => break,
                    }
                }
            } else if uid_len == 4 {
                let uid = u32::from_be_bytes(uid.try_into().unwrap());

                let reader_nonce = 0x1234_5678_u32;

                defmt::info!("UID: {=u32:08x}", uid);
                let mut authenticated_sector = false;
                let mut last_sector = u16::MAX;
                let mut command = [0u8; 32];
                let mut encryption_input = [0u8; 16];
                let mut block = [0u8; 16];
                for block_num in 4u8..64u8 {
                    let sector = (block_num as u16) / 4;

                    let authenticated = if sector != last_sector {
                        command[0] = 0x60; // AUTH A
                        command[1] = block_num;
                        let command_offset = 2;
                        let crc =
                            rp2350_playground::nfc::mifare::calc_crc16(&command[..command_offset]);
                        command[command_offset] = crc[0];
                        command[command_offset + 1] = crc[1];
                        let command_offset = command_offset + 2;

                        let cmd = &command[..command_offset];

                        defmt::info!("Sector {}: Sending auth command: {=[u8]:02x}", sector, cmd);
                        crypto1.init(0xFFFF_FFFF_FFFF);

                        let tag_nonce = match card.transceive_no_crc(cmd, &mut block, 65536).await {
                            Ok(len) => {
                                if len >= 4 {
                                    defmt::info!(
                                        "Sector {}: Received auth command response: {=[u8]:02x}",
                                        sector,
                                        &block[..len]
                                    );
                                    let tag_nonce = &block[0..4];
                                    let tag_nonce =
                                        u32::from_be_bytes(tag_nonce.try_into().unwrap());
                                    Some(tag_nonce)
                                } else {
                                    defmt::warn!(
                                        "Sector {}: Auth response too short: {=[u8]:02x} ({})",
                                        sector,
                                        &block[..len],
                                        len
                                    );
                                    None
                                }
                            }
                            Err(e) => {
                                let error_str = rnfc_st25r39_to_str(&e);
                                defmt::warn!(
                                    "Sector {}: Authentication failed: {}",
                                    sector,
                                    error_str
                                );
                                None
                            }
                        };
                        let authenticated = if let Some(tag_nonce) = tag_nonce {
                            let tag_nonce_prim =
                                rp2350_playground::nfc::crypto1::prng_successor(tag_nonce, 64);

                            let _ = crypto1.step_word_fill(uid ^ tag_nonce, false);

                            let fill = reader_nonce.to_be_bytes();
                            encryption_input[..4].copy_from_slice(&fill);
                            encryption_input[4..8].copy_from_slice(&tag_nonce_prim.to_be_bytes());

                            let len = crypto1
                                .encrypt_with_parity(&encryption_input[..8], &fill, &mut command)
                                .unwrap();

                            defmt::info!(
                                "tag nonce: {=u32:08x} tag nonce' {=u32:08x} reader nonce {=u32:08x} UID {=u32:08x} UID + tag nonce {=u32:08x}",
                                tag_nonce,
                                tag_nonce_prim,
                                reader_nonce,
                                uid,
                                uid ^ tag_nonce,
                            );

                            defmt::info!("Sector {}: Request {=[u8]:02x}", sector, command[..len]);

                            let cmd = &command[..len];

                            defmt::info!(
                                "Sector {}: Sending auth response: {=[u8]:02x}",
                                sector,
                                cmd
                            );

                            match card
                                .transceive_no_crc_no_parity(cmd, &mut block, 65536)
                                .await
                            {
                                Ok(len) => {
                                    let response = &block[..len];
                                    if len >= 4 {
                                        let tag_challenge = &response[..4];
                                        let tag_challenge =
                                            u32::from_be_bytes(tag_challenge.try_into().unwrap());
                                        defmt::info!(
                                            "Sector {}: Auth success, data: {=u32:08x}",
                                            sector,
                                            tag_challenge
                                        );
                                        let tag_challenge = tag_challenge ^ crypto1.step_word();
                                        let tag_challenge_expected =
                                            rp2350_playground::nfc::crypto1::prng_successor(
                                                tag_nonce, 96,
                                            );
                                        if tag_challenge == tag_challenge_expected {
                                            defmt::info!(
                                                "Sector {}: Authenticated successfully!",
                                                sector
                                            );
                                            true
                                        } else {
                                            defmt::warn!(
                                                "Sector {}: Auth success, but tag challenge mismatch: got {=u32:08x}, expected {=u32:08x}",
                                                sector,
                                                tag_challenge,
                                                tag_challenge_expected
                                            );
                                            false
                                        }
                                    } else {
                                        defmt::warn!(
                                            "Sector {}: Auth success, but response too short: {=[u8]:02x} ({})",
                                            sector,
                                            response,
                                            len
                                        );
                                        false
                                    }
                                }
                                Err(e) => {
                                    let error_str = rnfc_st25r39_to_str(&e);
                                    defmt::warn!(
                                        "Sector {}: Challenge failed: {}",
                                        sector,
                                        error_str
                                    );
                                    false
                                }
                            }
                        } else {
                            false
                        };
                        last_sector = sector;
                        authenticated_sector = authenticated;
                        authenticated
                    } else {
                        authenticated_sector
                    };
                    if authenticated {
                        defmt::info!("Block {}: Authentication successful!", block_num);
                        // Proceed with the next steps in the NFC communication
                        command[0] = 0x30; // READ
                        command[1] = block_num;
                        let crc = rp2350_playground::nfc::mifare::calc_crc16(&command[..2]);
                        command[2] = crc[0];
                        command[3] = crc[1];

                        match card
                            .transceive_no_crc(&command[..4], &mut block, 65536)
                            .await
                        {
                            Ok(len) => {
                                defmt::info!(
                                    "Block {}: Read success, data: {=[u8]:02x}",
                                    block_num,
                                    &block[..len]
                                );
                            }
                            Err(e) => {
                                let error_str = rnfc_st25r39_to_str(&e);
                                defmt::warn!("Block {}: Read failed: {}", block_num, error_str);
                            }
                        }
                    } else {
                        break;
                    }
                }
            }
        }
    }
}
