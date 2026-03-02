#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::pwm::{Pwm, SetDutyCycle};
use embassy_time::Timer;
use embedded_alloc::LlffHeap as Heap;
use pn532::i2c::I2CInterfaceWithIrq;
use pn532::{Pn532, Request};

use {defmt_rtt as _, panic_probe as _};

#[global_allocator]
static HEAP: Heap = Heap::empty();

embassy_rp::bind_interrupts!(struct Irqs {
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<embassy_rp::peripherals::I2C0>;
});

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"TMP117 Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"An example that reads temperature from TMP117 sensor."
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let mut rp_configuration: embassy_rp::config::Config = Default::default();
    rp_configuration.clocks = embassy_rp::clocks::ClockConfig::crystal(12_000_000);
    let peripherals = embassy_rp::init(rp_configuration);

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

    let mut pn532_reset = Output::new(peripherals.PIN_6, Level::Low);

    let pn532_sda = peripherals.PIN_16;
    let pn532_scl = peripherals.PIN_17;
    let pn532_irq = Input::new(peripherals.PIN_7, Pull::Up);

    let mut config = embassy_rp::i2c::Config::default();
    config.frequency = 400_000;
    let _bus = embassy_rp::i2c::I2c::new_blocking(peripherals.I2C0, pn532_scl, pn532_sda, config);
    let mut pn532: Pn532<_, (), 512> = Pn532::new_async(I2CInterfaceWithIrq {
        i2c: _bus,
        irq: pn532_irq,
    });

    Timer::after_millis(100).await;

    let _ = pn532_reset.set_high();

    Timer::after_millis(100).await;

    {
        let mut is_ok = false;

        defmt::info!("Aborting PN532...");

        pn532.abort().expect("Failed to abort PN532");

        defmt::info!("Configure PN532 SAM...");

        let sam_request =
            pn532::Request::sam_configuration(pn532::requests::SAMMode::Normal, false);
        match pn532.process_no_response_async(&sam_request).await {
            Ok(()) => {
                defmt::info!("PN532 SAM Configuration successful");
                is_ok = true;
            }
            Err(_) => {
                defmt::error!("Error during PN532 SAM Configuration");
            }
        }
        if is_ok {
            match pn532.process_async(&Request::GET_FIRMWARE_VERSION, 4).await {
                Ok(response) => {
                    defmt::info!("PN532 Firmware Version: {:?}", response);
                }
                Err(_) => {
                    defmt::error!("Error getting PN532 firmware version");
                }
            }
        }
    }

    const SCAN: pn532::Request<2> = Request::new(
        pn532::requests::Command::InListPassiveTarget,
        [1, pn532::requests::CardType::IsoTypeA as u8],
    );

    let mut nfc_id_buf = [0u8; 8];
    let mut last_nfc_id_buf = [0u8; 8];

    loop {
        let (target, nfc_id) = match pn532.process_async(&SCAN, 32).await {
            Ok(response) => {
                let num_targets = response[0] as usize;

                if num_targets == 0 {
                    (0u8, &[0u8; 4][..])
                } else {
                    let target = response[1];
                    // ATQA (Answer To ReQuest code A)
                    let atqa = u16::from_be_bytes(response[2..4].try_into().unwrap());
                    // SAK (Select Acknowledge)
                    let sak = response[4];
                    let nfc_id_len = response[5];
                    let target_type = rp2350_playground::nfc::TargetType::from_response(sak, nfc_id_len);
                    let nfc_id_len = usize::from(response[5]);
                    nfc_id_buf.fill(0);
                    nfc_id_buf[..nfc_id_len].copy_from_slice(&response[6..6 + nfc_id_len]);
                    if last_nfc_id_buf != nfc_id_buf {
                        defmt::info!(
                            "Found card {} with NFC ID: {=[u8]:02x} ATQA: {=u16:04x} SAK: {=u8:02x} {}",
                            target,
                            &nfc_id_buf[..nfc_id_len],
                            atqa,
                            sak,
                            target_type
                        );
                    }

                    (target, &nfc_id_buf[..nfc_id_len])
                }
            }
            Err(error) => {
                let emsg = match error {
                    pn532::Error::BadAck => "bad acknowledge",
                    pn532::Error::BadResponseFrame => "bad response frame",
                    pn532::Error::Syntax => "syntax error",
                    pn532::Error::CrcError => "CRC error",
                    pn532::Error::BufTooSmall => "buffer too small",
                    pn532::Error::TimeoutAck => "acknowledge timeout",
                    pn532::Error::TimeoutResponse => "response timeout",
                    pn532::Error::InterfaceError(_if_error) => "underlying interface error",
                };
                defmt::error!("No card detected: {}", emsg);
                (0u8, &[0u8; 4][..])
            }
        };
        if last_nfc_id_buf == nfc_id_buf {
            continue;
        }
        let mut sector;
        let mut authenticated = false;
        if nfc_id.len() == 7 {
            let page = pn532
                .process_async(&pn532::Request::ntag_read(0), 262)
                .await
                .unwrap();
            defmt::info!("Page 0: {=[u8]:02x}", page);
        } else if nfc_id.len() == 4 {
            let mut memory = [0u8; 16 * 3 * 15];
            const DEFAULT_KEY: [u8; 6] = [0xFF; 6];
            for block in 0..64 {
                sector = block / 4;
                let block_in_sector = block % 4;
                if block_in_sector == 0 {
                    let command = [
                        target,
                        pn532::requests::MifareCommand::AuthenticationWithKeyB as u8,
                        block, // block number
                        DEFAULT_KEY[0],
                        DEFAULT_KEY[1],
                        DEFAULT_KEY[2],
                        DEFAULT_KEY[3],
                        DEFAULT_KEY[4],
                        DEFAULT_KEY[5],
                        nfc_id[0],
                        nfc_id[1],
                        nfc_id[2],
                        nfc_id[3],
                    ];
                    let auth: pn532::Request<13> =
                        Request::new(pn532::requests::Command::InDataExchange, command);
                    let auth_response = pn532.process_async(&auth, 16).await.unwrap();
                    if auth_response[0] == 0 {
                        authenticated = true;
                    } else {
                        defmt::error!(
                            "Authentication failed for sector {}, {=u8:02x}",
                            sector,
                            auth_response[0]
                        );
                        authenticated = false;
                    }
                }
                if authenticated {
                    let command = [
                        target,
                        pn532::requests::MifareCommand::Read as u8,
                        block, // block number
                    ];
                    let read: pn532::Request<3> =
                        Request::new(pn532::requests::Command::InDataExchange, command);

                    let block_data = pn532.process_async(&read, 32).await.unwrap();
                    if sector > 0 && block_in_sector < 3 {
                        let memory_offset =
                            ((sector as usize - 1) * 16 * 3) + (block_in_sector as usize * 16);
                        let copy_size = 16.min(block_data.len() - 1);
                        memory[memory_offset..memory_offset + copy_size]
                            .copy_from_slice(&block_data[1..1 + copy_size]);
                    }
                }
            }

            for chunk in memory.chunks_exact(16) {
                defmt::info!("Memory {=[u8]:02x}", chunk);
            }
            let mut tlv_tag_decoder = rp2350_playground::nfc::tlvtag::TlvDecoder::new(&memory);
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
                                            "    Found NDEF Record {} with payload length {} identity {}",
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
                                                        "    NDEF Record Type: Text, but failed to parse text"
                                                    );
                                                }
                                            }
                                            _ => {
                                                defmt::info!(
                                                    "    NDEF Record Type: {:?}",
                                                    record.record_type
                                                );
                                            }
                                        }
                                    }
                                    Err(e) => {
                                        defmt::error!("    Error parsing NDEF Record: {:?}", e);
                                    }
                                }
                            }
                        }
                        rp2350_playground::nfc::tlvtag::TlvTag::LockControl => {
                            defmt::info!(
                                "  Found Lock Control TLV with length {}",
                                tlv.value.len()
                            );
                        }
                        rp2350_playground::nfc::tlvtag::TlvTag::MemoryControl => {
                            defmt::info!(
                                "  Found Memory Control TLV with length {}",
                                tlv.value.len()
                            );
                        }
                        rp2350_playground::nfc::tlvtag::TlvTag::Proprietary => {
                            defmt::info!("  Found Proprietary TLV with length {}", tlv.value.len());
                        }
                        _ => (),
                    },
                    Some(Err(e)) => {
                        defmt::error!("  Error parsing TLV: {:?}", e);
                        break;
                    }
                    None => break,
                }
            }

            last_nfc_id_buf.copy_from_slice(&nfc_id_buf[..]);
        }
    }
}
