#![no_std]
#![no_main]

extern crate alloc;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::pwm::Pwm;
use embassy_time::{Duration, Ticker};
use embedded_alloc::LlffHeap as Heap;
use embedded_devices::devices::bosch::bmp390::{BMP390Async, address::Address};
use embedded_devices::sensor::OneshotSensorAsync;
use uom::si::pressure::hectopascal;
use uom::si::thermodynamic_temperature::degree_celsius;

use {defmt_rtt as _, panic_probe as _};

#[global_allocator]
static HEAP: Heap = Heap::empty();

embassy_rp::bind_interrupts!(struct Irqs {
    I2C1_IRQ => embassy_rp::i2c::InterruptHandler<embassy_rp::peripherals::I2C1>;
});

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"BMP390 Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"An example that reads temperature from BMP390 sensor."
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let mut rp_configuration: embassy_rp::config::Config = Default::default();
    rp_configuration.clocks = embassy_rp::clocks::ClockConfig::crystal(12_000_000);
    let peripherals = embassy_rp::init(rp_configuration);

    // ── PSRAM / Heap ──────────────────────────────────────────────────────
    let psram = {
        use embassy_rp::qmi_cs1::QmiCs1;
        let psram_config = embassy_rp::psram::Config::aps6404l();
        embassy_rp::psram::Psram::new(
            QmiCs1::new(peripherals.QMI_CS1, peripherals.PIN_0),
            psram_config,
        )
    };

    if let Ok(psram) = psram {
        unsafe {
            let address = psram.base_address() as usize;
            let size = psram.size() as usize;
            HEAP.init(address, size);
            defmt::info!("Heap in PSRAM at {:08x}, size: {}", address, size);
        }
    } else {
        defmt::warn!("Failed to initialize PSRAM, using internal RAM for heap");
        const HEAP_SIZE: usize = 65535;
        static mut HEAP_MEM: [u8; HEAP_SIZE] = [0xEE; HEAP_SIZE];

        #[allow(static_mut_refs)]
        unsafe {
            let address = HEAP_MEM.as_ptr() as usize;
            HEAP.init(address, HEAP_SIZE);
        }
    }

    let mut pwm_config: embassy_rp::pwm::Config = Default::default();
    pwm_config.top = 32_768;
    let _pwm_2_1 = Pwm::new_output_ab(
        peripherals.PWM_SLICE9,
        peripherals.PIN_34,
        peripherals.PIN_35,
        pwm_config,
    );

    let mut pwm_config: embassy_rp::pwm::Config = Default::default();
    pwm_config.top = 32_768;
    let _pwm_4_3 = Pwm::new_output_ab(
        peripherals.PWM_SLICE8,
        peripherals.PIN_32,
        peripherals.PIN_33,
        pwm_config,
    );

    let _audio_shutdown = Output::new(peripherals.PIN_19, Level::Low);

    let mut ticker = Ticker::every(Duration::from_secs(10));

    let sda = peripherals.PIN_14;
    let scl = peripherals.PIN_15;
    let config = embassy_rp::i2c::Config::default();
    let bus = embassy_rp::i2c::I2c::new_async(peripherals.I2C1, scl, sda, Irqs, config);

    let delay = embassy_time::Delay;

    let mut bmp390 = BMP390Async::new_i2c(delay, bus, Address::Secondary);
    match bmp390.init().await {
        Ok(()) => (),
        Err(_) => defmt::error!("Failed to initialize BMP390"),
    }

    let bmp390_configuration = embedded_devices::devices::bosch::bmp390::Configuration::default();
    match bmp390.configure(bmp390_configuration).await {
        Ok(()) => (),
        Err(_) => defmt::error!("Failed to configure BMP390"),
    }

    loop {
        match bmp390.measure().await {
            Ok(measurement) => {
                if let Some(temperature) = measurement.temperature {
                    let temp = temperature.get::<degree_celsius>();
                    defmt::info!("Oneshot temperature: {:?}°C", temp);
                } else {
                    defmt::info!("Temperature measurement not available");
                }
                if let Some(pressure) = measurement.pressure {
                    let pres = pressure.get::<hectopascal>();
                    defmt::info!("Oneshot pressure: {:?} hPa", pres);
                } else {
                    defmt::info!("Pressure measurement not available");
                }
            }
            Err(_) => defmt::error!("Failed to read temperature from BMP390"),
        }
        ticker.next().await;
    }
}
