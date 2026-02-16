#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::pwm::{Pwm};
use embassy_time::{Duration, Ticker};
use embedded_devices::devices::bosch::bmp390::{BMP390Async, address::Address};
use embedded_devices::sensor::OneshotSensorAsync;
use uom::si::thermodynamic_temperature::degree_celsius;
use uom::si::pressure::hectopascal;

use {defmt_rtt as _, panic_probe as _};

embassy_rp::bind_interrupts!(struct Irqs {
    I2C1_IRQ => embassy_rp::i2c::InterruptHandler<embassy_rp::peripherals::I2C1>;
});

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"BMP390 Example"),
    embassy_rp::binary_info::rp_program_description!(c"An example that reads temperature from BMP390 sensor."),
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
