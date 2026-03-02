#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1};
use embassy_rp::pwm::{Pwm, SetDutyCycle};
use embassy_rp::spi;
use embassy_time::{Delay, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal_mfrc522::consts::{PICCCommand, UidSize};
use esp_hal_mfrc522::MFRC522;

use embedded_alloc::LlffHeap as Heap;

use {defmt_rtt as _, panic_probe as _};

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"MFRC522 NFC Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"An example that reads NFC cards using the MFRC522 via SPI."
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<DMA_CH0>,
            embassy_rp::dma::InterruptHandler<DMA_CH1>;
});

pub struct DummyCsPin;

impl embedded_hal::digital::ErrorType for DummyCsPin {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::OutputPin for DummyCsPin {
    #[inline(always)]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    #[inline(always)]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Helper to call mifare_read with the required buffer size parameter.
async fn mfrc522_read<D: esp_hal_mfrc522::MfrcDriver>(
    mfrc522: &mut MFRC522<D>,
    block: u8,
    buf: &mut [u8; 18],
) -> Result<(), esp_hal_mfrc522::consts::PCDErrorCode> {
    let mut buf_size = 18u8;
    mfrc522.mifare_read(block, buf, &mut buf_size).await
}

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

    // MFRC522 reset pin
    let mut mfrc522_reset = Output::new(peripherals.PIN_6, Level::Low);

    // SPI configuration for MFRC522 (SPI Mode 0: CPOL=0, CPHA=0)
    let mut spi_config = spi::Config::default();
    spi_config.frequency = 1_000_000; // 1 MHz
    spi_config.polarity = spi::Polarity::IdleLow;
    spi_config.phase = spi::Phase::CaptureOnFirstTransition;

    // SPI0 with DMA for async support
    let spi_bus = spi::Spi::new(
        peripherals.SPI0,
        peripherals.PIN_2, // SCK
        peripherals.PIN_3, // MOSI (TX)
        peripherals.PIN_4, // MISO (RX)
        peripherals.DMA_CH0,
        peripherals.DMA_CH1,
        Irqs,
        spi_config,
    );

    let cs = Output::new(peripherals.PIN_1, Level::High);

    let spi_device = ExclusiveDevice::new(spi_bus, cs, Delay).unwrap();
    let driver = esp_hal_mfrc522::drivers::SpiDriver::new(spi_device);
    let mut mfrc522 = MFRC522::new(driver);

    // Hardware reset
    Timer::after_millis(10).await;
    mfrc522_reset.set_high();
    Timer::after_millis(50).await;

    // Initialize MFRC522
    match mfrc522.pcd_init().await {
        Ok(()) => defmt::info!("MFRC522 initialized successfully"),
        Err(_e) => {
            defmt::error!("Failed to initialize MFRC522");
            loop {
                Timer::after_millis(1000).await;
            }
        }
    }

    match mfrc522.pcd_get_version().await {
        Ok(version) => defmt::info!("MFRC522 firmware version: {}", version as u8),
        Err(_e) => defmt::error!("Failed to get MFRC522 version"),
    }

    match mfrc522.pcd_set_antenna_gain(0xff).await {
        Ok(()) => defmt::info!("Antenna gain set to max"),
        Err(_e) => defmt::error!("Failed to set antenna gain"),
    }

    let mut last_uid_bytes = [0u8; 10];
    let mut last_uid_size = 0u8;

    defmt::info!("Scanning for NFC cards...");

    loop {
        Timer::after_millis(200).await;

        // Check for new card
        if mfrc522.picc_is_new_card_present().await.is_err() {
            continue;
        }

        // Read card UID
        let uid = match mfrc522.get_card(UidSize::Four).await {
            Ok(uid) => uid,
            Err(_) => match mfrc522.get_card(UidSize::Seven).await {
                Ok(uid) => uid,
                Err(_) => continue,
            },
        };

        let uid_len = uid.size as usize;
        let uid_bytes = &uid.uid_bytes[..uid_len];

        let target_type =
            rp2350_playground::nfc::TargetType::from_response(uid.sak, uid.size);
        defmt::info!(
            "Found card with UID: {=[u8]:02x} SAK: {=u8:02x} {}",
            uid_bytes,
            uid.sak,
            target_type
        );

        // Skip if same card as last time
        if uid.size == last_uid_size && uid_bytes == &last_uid_bytes[..uid_len] {
            let _ = mfrc522.picc_halta().await;
            let _ = mfrc522.pcd_stop_crypto1().await;
            continue;
        }

        if uid_len == 7 {
            // NTAG / Ultralight - NFC Forum Type 2 Tag
            // Read capability container (page 3) first via pages 0-3
            let mut buf = [0u8; 18];
            match mfrc522_read(&mut mfrc522, 3, &mut buf).await {
                Ok(()) => {
                    let cc_magic = buf[0];
                    let cc_version = buf[1];
                    let cc_size = buf[2] as usize * 8;
                    let cc_access = buf[3];
                    defmt::info!(
                        "CC: magic={=u8:02x} version={=u8:02x} size={} access={=u8:02x}",
                        cc_magic,
                        cc_version,
                        cc_size,
                        cc_access
                    );

                    if cc_magic == 0xE1 && cc_size > 0 {
                        let user_bytes = cc_size.min(888);
                        let mut memory = [0u8; 888];
                        let mut offset = 0;
                        let mut page = 4u8;

                        while offset < user_bytes {
                            match mfrc522_read(&mut mfrc522, page, &mut buf).await {
                                Ok(()) => {
                                    let copy_len = 16.min(user_bytes - offset);
                                    memory[offset..offset + copy_len]
                                        .copy_from_slice(&buf[..copy_len]);
                                    offset += 16;
                                    page = page.saturating_add(4);
                                }
                                Err(_e) => {
                                    defmt::warn!("Read failed at page {}", page);
                                    break;
                                }
                            }
                        }

                        let valid_len = offset.min(user_bytes);
                        for (i, chunk) in memory[..valid_len].chunks(16).enumerate() {
                            defmt::info!(
                                "Memory [{=u8:02x}] {=[u8]:02x}",
                                (i * 4 + 4) as u8,
                                chunk
                            );
                        }

                        decode_tlv_ndef(&memory[..valid_len]);
                    }
                }
                Err(_e) => {
                    defmt::warn!("Failed to read CC page");
                }
            }
        } else if uid_len == 4 {
            // MIFARE Classic - authenticate and read sectors
            let mut memory = [0u8; 16 * 3 * 15];
            let default_key: [u8; 6] = [0xFF; 6];
            let mut authenticated = false;

            for block in 0u8..64 {
                let sector = block / 4;
                let block_in_sector = block % 4;

                // Authenticate at start of each sector
                if block_in_sector == 0 {
                    match mfrc522
                        .pcd_authenticate(
                            PICCCommand::PICC_CMD_MF_AUTH_KEY_B,
                            block,
                            &default_key,
                            &uid,
                        )
                        .await
                    {
                        Ok(()) => {
                            authenticated = true;
                        }
                        Err(_e) => {
                            defmt::error!(
                                "Authentication failed for sector {}",
                                sector,
                            );
                            authenticated = false;
                        }
                    }
                }

                if authenticated {
                    let mut buf = [0u8; 18];
                    match mfrc522_read(&mut mfrc522, block, &mut buf).await {
                        Ok(()) => {
                            if sector > 0 && block_in_sector < 3 {
                                let memory_offset =
                                    ((sector as usize - 1) * 16 * 3)
                                        + (block_in_sector as usize * 16);
                                memory[memory_offset..memory_offset + 16]
                                    .copy_from_slice(&buf[..16]);
                            }
                        }
                        Err(_e) => {
                            defmt::warn!("Read failed at block {}", block);
                        }
                    }
                }
            }

            // Stop crypto and halt card
            let _ = mfrc522.pcd_stop_crypto1().await;

            for chunk in memory.chunks_exact(16) {
                defmt::info!("Memory {=[u8]:02x}", chunk);
            }

            decode_tlv_ndef(&memory);
        }

        // Remember this card
        last_uid_bytes.fill(0);
        last_uid_bytes[..uid_len].copy_from_slice(uid_bytes);
        last_uid_size = uid.size;

        let _ = mfrc522.picc_halta().await;
        let _ = mfrc522.pcd_stop_crypto1().await;
    }
}

fn decode_tlv_ndef(memory: &[u8]) {
    let mut tlv_tag_decoder = rp2350_playground::nfc::tlvtag::TlvDecoder::new(memory);
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
}
