#![no_std]
#![no_main]

extern crate alloc;

use alloc::vec;
use cyw43::aligned_bytes;
use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};
use defmt::unwrap;
use embedded_alloc::LlffHeap as Heap;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{PIO0};
use embassy_rp::pio::Pio;
use embassy_time::{Duration, Ticker};

use {defmt_rtt as _, panic_probe as _};

const WIFI_NETWORK: &str = env!("SSID");
const WIFI_PASSWORD: &str = env!("WIRELESS_PSK");

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"802.11 Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"An 802.11 test program"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];


bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<
        'static,
        cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0, embassy_rp::peripherals::DMA_CH0>>,
    >,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let peripherals = embassy_rp::init(Default::default());
    let mut rng = embassy_rp::clocks::RoscRng;

    defmt::info!("Entry");

    let psram = {
        use embassy_rp::qmi_cs1::QmiCs1;
        let psram_config = embassy_rp::psram::Config::aps6404l();
        embassy_rp::psram::Psram::new(QmiCs1::new(peripherals.QMI_CS1, peripherals.PIN_0), psram_config)
    };

    if let Ok(psram) = psram {
        defmt::info!("PSRAM initialized successfully, using PSRAM for heap");

        {
            let (address, size) = unsafe {
                let address = psram.base_address() as usize;
                let size = psram.size() as usize;
                HEAP.init(address, size);
                (address, size)
            };
            defmt::info!("Heap initialized in PSRAM at {:08x}, size: {}", address, size);
        }
    } else {
        defmt::warn!("Failed to initialize PSRAM, using internal RAM for heap");

        const HEAP_SIZE: usize = 65535; // 64 KiB heap size
        static mut HEAP_MEM: [u8; HEAP_SIZE] = [0xEE; HEAP_SIZE];

        #[allow(static_mut_refs)]
        let (address, size) = unsafe {
            let address = HEAP_MEM.as_ptr() as usize;
            HEAP.init(address, HEAP_SIZE);
            (address, HEAP_SIZE)
        };
        defmt::info!("Heap initialized at addr: {:08x}, size: {}", address, size);
    }

    let mut _audio_shutdown = Output::new(peripherals.PIN_19, Level::Low);

    let clm = aligned_bytes!("../../../firmware/43439A0_clm.bin");
    let nvram = aligned_bytes!("../../../firmware/nvram_rp2040.bin");
    let fw = aligned_bytes!("../../../firmware/43439A0.bin");

    defmt::info!("Initialize CYW43");

    static STATE: static_cell::StaticCell<cyw43::State> = static_cell::StaticCell::new();
    let state = STATE.init(cyw43::State::new());

    defmt::info!("Configure CYW43 SPI");

    let pwr = Output::new(peripherals.PIN_23, Level::Low);
    let cs = Output::new(peripherals.PIN_25, Level::High);
    let mut pio0 = Pio::new(peripherals.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio0.common,
        pio0.sm0,
        // SPI communication won't work if the speed is too high, so we use a divider larger than `DEFAULT_CLOCK_DIVIDER`.
        // See: https://github.com/embassy-rs/embassy/issues/3960.
        RM2_CLOCK_DIVIDER,
        pio0.irq0,
        cs,
        peripherals.PIN_24,
        peripherals.PIN_29,
        peripherals.DMA_CH0,
    );

    defmt::info!("CYW43 Construction");

    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw, nvram).await;

    defmt::info!("Spawn CYW43 runner task");

    spawner.spawn(unwrap!(cyw43_task(runner)));

    let chonk: alloc::vec::Vec<u8> = vec![];
    defmt::info!("Allocated chonk of size {}", chonk.len());

    defmt::info!("CYW43 Initialization");

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let config = embassy_net::Config::dhcpv4(Default::default());
    let seed = rng.next_u64();

    // Init network stack
    static NET_RESOURCES: static_cell::StaticCell<embassy_net::StackResources<5>> = static_cell::StaticCell::new();
    let (net_stack, runner) = embassy_net::new(net_device, config, NET_RESOURCES.init(embassy_net::StackResources::new()), seed);

    spawner.spawn(unwrap!(net_task(runner)));

    let _wireless_hw_address = net_stack.hardware_address();

    while let Err(err) = control
        .join(WIFI_NETWORK, cyw43::JoinOptions::new(WIFI_PASSWORD.as_bytes()))
        .await
    {
        defmt::info!("join failed: {:?}", err);
    }

    defmt::info!("waiting for link...");
    net_stack.wait_link_up().await;

    defmt::info!("waiting for DHCP...");
    net_stack.wait_config_up().await;

    if let Some(ipv4_config) = net_stack.config_v4() {
        defmt::info!("Got IP address: {}", ipv4_config.address);
    } else {
        defmt::warn!("No IPv4 address assigned");
    }

    let mut ticker = Ticker::every(Duration::from_secs(10));

    defmt::info!("Enter main loop");

    loop {
        defmt::info!("Tick");
        ticker.next().await;
    }
}
