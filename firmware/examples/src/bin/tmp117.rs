#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::pwm::{Pwm};
use embassy_time::{Duration, Ticker};
use embedded_devices::devices::texas_instruments::tmp117::{TMP117Async, address::Address};
use embedded_devices::sensor::OneshotSensorAsync;
use uom::si::thermodynamic_temperature::degree_celsius;

use {defmt_rtt as _, panic_probe as _};

embassy_rp::bind_interrupts!(struct Irqs {
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<embassy_rp::peripherals::I2C0>;
});

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"TMP117 Example"),
    embassy_rp::binary_info::rp_program_description!(c"An example that reads temperature from TMP117 sensor."),
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

    let sda = peripherals.PIN_16;
    let scl = peripherals.PIN_17;
    let config = embassy_rp::i2c::Config::default();
    let bus = embassy_rp::i2c::I2c::new_async(peripherals.I2C0, scl, sda, Irqs, config);

    let delay = embassy_time::Delay;

    let mut tmp117 = TMP117Async::new_i2c(delay, bus, Address::Gnd);
    match tmp117.init().await {
        Ok(()) => (),
        Err(_) => defmt::error!("Failed to initialize TMP117"),
    }

    loop {
        match tmp117.measure().await {
            Ok(measurement) => {
                let temp = measurement.temperature.get::<degree_celsius>();
                defmt::info!("Oneshot temperature: {:?}°C", temp);
            }
            Err(_) => defmt::error!("Failed to read temperature from TMP117"),
        }
        ticker.next().await;
    }
}
