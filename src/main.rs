#![feature(backtrace)]

use anyhow::Result;
use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics::image::Image;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::{Dimensions, DrawTarget, Point, RgbColor, WebColors};
use embedded_graphics::primitives::{
    Polyline, Primitive, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle,
};
use embedded_graphics::text::Text;
use embedded_graphics::Drawable;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::prelude::_embedded_hal_blocking_delay_DelayMs;
use esp_idf_hal::delay::Ets;
use esp_idf_hal::rmt::config::TransmitConfig;
use esp_idf_hal::rmt::{self, FixedLengthSignal, PinState, Pulse, Transmit};
use esp_idf_hal::spi::SPI3;
use esp_idf_hal::{delay, gpio, gpio::Pull, prelude::*, spi};
use esp_idf_sys as _;
use log::*;
use mipidsi::DisplayOptions;
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;
use tinybmp::DynamicBmp;

static LED1: AtomicBool = AtomicBool::new(true);

fn main() -> Result<()> {
    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_sys::link_patches();

    esp_idf_svc::log::EspLogger::initialize_default();

    info!("Initializing");

    let peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;

    let button_up_mutex: Arc<Mutex<()>> = Arc::new(Mutex::new(()));

    let mut led1 = pins.gpio15.into_output()?;

    let button_up_mutex_thread = Arc::clone(&button_up_mutex);
    let thread1 = thread::spawn(move || {
        info!("Hello world, thread 1");
        loop {
            if LED1.load(std::sync::atomic::Ordering::Relaxed) {
                let lock = button_up_mutex_thread.lock().unwrap();
                led1.set_high().unwrap();
                thread::sleep(Duration::from_millis(500));
                led1.set_low().unwrap();
                drop(lock);
            }
            thread::sleep(Duration::from_millis(500));
        }
    });

    let mut led2: gpio::Gpio16<gpio::Output> = pins.gpio16.into_output()?;

    let thread2 = thread::spawn(move || {
        info!("Hello world, thread 2");
        loop {
            led2.set_high().unwrap();
            thread::sleep(Duration::from_millis(500));
            led2.set_low().unwrap();
            thread::sleep(Duration::from_millis(500));
        }
    });

    let mut button_up = pins.gpio10.into_input()?;
    button_up.set_pull_up()?;

    let button1_thread = Arc::clone(&button_up_mutex);
    let button_thread = thread::spawn(move || {
        info!("Hello world, button thread");
        loop {
            let lock = button1_thread.lock().unwrap();
            if button_up.is_low().unwrap() {
                info!("button up");
                if LED1
                    .compare_exchange(
                        true,
                        false,
                        std::sync::atomic::Ordering::SeqCst,
                        std::sync::atomic::Ordering::Acquire,
                    )
                    .is_err()
                {
                    LED1.store(true, std::sync::atomic::Ordering::SeqCst);
                }
            }
            drop(lock);
            thread::sleep(Duration::from_millis(100));
        }
    });

    let backlight: gpio::Gpio9<gpio::Unknown> = pins.gpio9;
    let sclk: gpio::Gpio6<gpio::Unknown> = pins.gpio6;
    let sdo: gpio::Gpio7<gpio::Unknown> = pins.gpio7;
    let cs: gpio::Gpio5<gpio::Unknown> = pins.gpio5;
    let dc: gpio::Gpio4<gpio::Unknown> = pins.gpio4;
    let rst: gpio::Gpio8<gpio::Unknown> = pins.gpio8;
    let spi3 = peripherals.spi3;

    setup_display(backlight, sclk, sdo, cs, dc, rst, spi3)?;

    info!("Hello, world!");

    thread1
        .join()
        .map_err(|e| anyhow::anyhow!("Thread join error: {:?}", e))?;

    thread2
        .join()
        .map_err(|e| anyhow::anyhow!("Thread join error: {:?}", e))?;

    button_thread
        .join()
        .map_err(|e| anyhow::anyhow!("Thread join error: {:?}", e))?;

    Ok(())
}

#[allow(dead_code)]
fn neopixel(led: gpio::Gpio16<gpio::Output>, channel: rmt::CHANNEL0) -> Result<()> {
    let config = TransmitConfig::new().clock_divider(1);
    let mut tx = Transmit::new(led, channel, &config)?;

    let rgbs = [0xff0000, 0xffff00, 0x00ffff, 0x00ff00, 0xa000ff];
    loop {
        for rgb in rgbs {
            let ticks_hz = tx.counter_clock()?;
            let t0h = Pulse::new_with_duration(ticks_hz, PinState::High, &ns(350))?;
            let t0l = Pulse::new_with_duration(ticks_hz, PinState::Low, &ns(800))?;
            let t1h = Pulse::new_with_duration(ticks_hz, PinState::High, &ns(700))?;
            let t1l = Pulse::new_with_duration(ticks_hz, PinState::Low, &ns(600))?;

            let mut signal = FixedLengthSignal::<24>::new();
            for i in 0..24 {
                let bit = 2_u32.pow(i) & rgb != 0;
                let (high_pulse, low_pulse) = if bit { (t1h, t1l) } else { (t0h, t0l) };
                signal.set(i as usize, &(high_pulse, low_pulse))?;
            }
            tx.start_blocking(&signal)?;
            Ets.delay_ms(1000u32);
        }
    }
}

#[allow(dead_code)]
fn ns(nanos: u64) -> Duration {
    Duration::from_nanos(nanos)
}

type Display = mipidsi::Display<
    SPIInterfaceNoCS<
        spi::Master<
            SPI3,
            gpio::Gpio6<gpio::Unknown>,
            gpio::Gpio7<gpio::Unknown>,
            gpio::Gpio21<gpio::Unknown>,
            gpio::Gpio5<gpio::Unknown>,
        >,
        gpio::Gpio4<gpio::Output>,
    >,
    gpio::Gpio8<gpio::Output>,
    mipidsi::models::ST7789,
>;

fn setup_display(
    backlight: gpio::Gpio9<gpio::Unknown>,
    sclk: gpio::Gpio6<gpio::Unknown>,
    sdo: gpio::Gpio7<gpio::Unknown>,
    cs: gpio::Gpio5<gpio::Unknown>,
    dc: gpio::Gpio4<gpio::Unknown>,
    rst: gpio::Gpio8<gpio::Unknown>,
    spi3: SPI3,
) -> Result<()> {
    let config = <spi::config::Config as Default>::default().baudrate(80.MHz().into());
    let mut backlight = backlight.into_output()?;
    backlight.set_high()?;

    let spi: spi::Master<
        spi::SPI3,
        gpio::Gpio6<gpio::Unknown>,
        gpio::Gpio7<gpio::Unknown>,
        gpio::Gpio21<gpio::Unknown>,
        gpio::Gpio5<gpio::Unknown>,
    > = spi::Master::<spi::SPI3, _, _, _, _>::new(
        spi3,
        spi::Pins {
            sclk,
            sdo,
            sdi: Option::<gpio::Gpio21<gpio::Unknown>>::None,
            cs: Some(cs),
        },
        config,
    )?;

    let di = SPIInterfaceNoCS::new(spi, dc.into_output()?);

    let reset: gpio::Gpio8<gpio::Output> = rst.into_output()?;

    let mut display: Display = Display::st7789(di, reset);

    display
        .init(&mut delay::Ets, DisplayOptions::default())
        .map_err(|e| anyhow::anyhow!("Display error: {:?}", e))?;

    display.clear(Rgb565::BLACK).unwrap();

    info!("Screen: {:?}", display.bounding_box());

    Rectangle::new(display.bounding_box().top_left, display.bounding_box().size)
        .into_styled(
            PrimitiveStyleBuilder::new()
                .fill_color(Rgb565::CSS_PURPLE)
                .build(),
        )
        .draw(&mut display)
        .map_err(|e| anyhow::anyhow!("Display error: {:?}", e))?;

    // draw_heartbeat(&mut display)?;

    let bmp = DynamicBmp::from_slice(include_bytes!("../assets/screen.bmp"))
        .map_err(|e| anyhow::anyhow!("Image error: {:?}", e))?;

    let image = Image::new(&bmp, Point::new(0, 0));

    image
        .draw(&mut display)
        .map_err(|e| anyhow::anyhow!("Image draw error: {:?}", e))?;

    Text::new(
        "Hello Rust!",
        Point::new(10, 20),
        MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE),
    )
    .draw(&mut display)
    .map_err(|e| anyhow::anyhow!("Display error: {:?}", e))?;
    Ok(())
}

#[allow(unused)]
fn draw_heartbeat(display: &mut Display) -> Result<()> {
    let points: [Point; 10] = [
        Point::new(10, 84),
        Point::new(50, 84),
        Point::new(60, 64),
        Point::new(70, 84),
        Point::new(80, 84),
        Point::new(90, 94),
        Point::new(100, 30),
        Point::new(110, 104),
        Point::new(120, 84),
        Point::new(300, 84),
    ];

    let line_style = PrimitiveStyle::with_stroke(Rgb565::RED, 5);

    Polyline::new(&points)
        .into_styled(line_style)
        .draw(display)
        .map_err(|e| anyhow::anyhow!("Draw error: {:?}", e))?;

    Ok(())
}
