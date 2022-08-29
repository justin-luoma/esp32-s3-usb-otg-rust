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
use embedded_hal::prelude::*;
use esp_idf_hal::rmt::{
    self, config::TransmitConfig, FixedLengthSignal, PinState, Pulse, Transmit,
};
use esp_idf_hal::{delay, delay::Ets, gpio, prelude::*, spi, spi::SPI3};

use esp32s3_hal::IO;

use log::*;
use mipidsi::DisplayOptions;
use std::sync::{atomic::AtomicBool, Arc, Mutex};
use std::thread;

use std::time::Duration;
use tinybmp::DynamicBmp;

static LED1: AtomicBool = AtomicBool::new(true);
static LED2: AtomicBool = AtomicBool::new(true);

fn main() -> Result<()> {
    esp_idf_sys::link_patches();

    esp_idf_svc::log::EspLogger::initialize_default();

    info!("Initializing");

    let peripherals = esp32s3_hal::pac::Peripherals::take().unwrap();
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let pins = io.pins;

    // let touch = pins.gpio3.into_input()?.into_floating()?;

    // let touch_thread = thread::spawn(move || loop {
    //     info!("Touch: {}", touch.is_high().unwrap());
    //     thread::sleep(Duration::from_millis(100));
    // });

    let button_up_mutex: Arc<Mutex<()>> = Arc::new(Mutex::new(()));

    let led1 = pins.gpio15.into_push_pull_output();

    let button_up_mutex_thread = Arc::clone(&button_up_mutex);

    let thread1 =
        thread::spawn(move || spawn_toggleable_pin_thread(led1, &LED1, button_up_mutex_thread));

    // let button_down_mutex: Arc<Mutex<()>> = Arc::new(Mutex::new(()));

    // let led2: gpio::Gpio16<gpio::Output> = pins.gpio16.into_output()?;

    // let button_down_mutex_thread = Arc::clone(&button_down_mutex);
    // let thread2 =
    //     thread::spawn(move || spawn_toggleable_pin_thread(led2, &LED2, button_down_mutex_thread));

    // let mut button_up = pins.gpio10.into_input()?;
    // button_up.set_pull_up()?;

    // let button_up_mutex_thread = Arc::clone(&button_up_mutex);
    // let button_up_thread =
    //     thread::spawn(move || spawn_button_thread(button_up, button_up_mutex_thread, &LED1));

    // let mut button_down = pins.gpio11.into_input()?;
    // button_down.set_pull_up()?;

    // let button_down_mutex_thread = Arc::clone(&button_down_mutex);
    // let button_down_thread =
    //     thread::spawn(move || spawn_button_thread(button_down, button_down_mutex_thread, &LED2));

    // let backlight: gpio::Gpio9<gpio::Unknown> = pins.gpio9;
    // let sclk: gpio::Gpio6<gpio::Unknown> = pins.gpio6;
    // let sdo: gpio::Gpio7<gpio::Unknown> = pins.gpio7;
    // let cs: gpio::Gpio5<gpio::Unknown> = pins.gpio5;
    // let dc: gpio::Gpio4<gpio::Unknown> = pins.gpio4;
    // let rst: gpio::Gpio8<gpio::Unknown> = pins.gpio8;
    // let spi3 = peripherals.spi3;

    // setup_display(backlight, sclk, sdo, cs, dc, rst, spi3)?;

    info!("Hello, world!");

    thread1
        .join()
        .map_err(|e| anyhow::anyhow!("Thread join error: {:?}", e))?;

    // thread2
    //     .join()
    //     .map_err(|e| anyhow::anyhow!("Thread join error: {:?}", e))?;

    // button_up_thread
    //     .join()
    //     .map_err(|e| anyhow::anyhow!("Thread join error: {:?}", e))?;

    // button_down_thread
    //     .join()
    //     .map_err(|e| anyhow::anyhow!("Thread join error: {:?}", e))?;

    // touch_thread
    //     .join()
    //     .map_err(|e| anyhow::anyhow!("Thread join error: {:?}", e))?;

    Ok(())
}

fn spawn_button_thread<B>(button: B, mutex: Arc<Mutex<()>>, led_state: &AtomicBool)
where
    B: InputPin,
{
    info!("Hello world, button thread");
    loop {
        let lock = mutex.lock().unwrap();
        if button
            .is_low()
            .map_err(|_| anyhow::anyhow!("Button error"))
            .unwrap()
        {
            toggle(led_state);
            thread::sleep(Duration::from_millis(250));
        }
        drop(lock);
        thread::sleep(Duration::from_millis(25));
    }
}

fn spawn_toggleable_pin_thread<P>(mut led: P, enabled: &AtomicBool, button_mutex: Arc<Mutex<()>>)
where
    P: OutputPin,
{
    loop {
        if enabled.load(std::sync::atomic::Ordering::Relaxed) {
            let lock = button_mutex.lock().unwrap();
            led.set_high()
                .map_err(|_| anyhow::anyhow!("Led error"))
                .unwrap();
            thread::sleep(Duration::from_millis(500));
            led.set_low()
                .map_err(|_| anyhow::anyhow!("Led error"))
                .unwrap();
            drop(lock);
        }
        thread::sleep(Duration::from_millis(500));
    }
}

fn toggle(b: &AtomicBool) {
    if b.compare_exchange(
        true,
        false,
        std::sync::atomic::Ordering::SeqCst,
        std::sync::atomic::Ordering::Acquire,
    )
    .is_err()
    {
        b.store(true, std::sync::atomic::Ordering::SeqCst);
    }
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
