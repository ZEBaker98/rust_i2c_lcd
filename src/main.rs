#![no_std]
#![no_main]

#[cfg(feature = "rtt")]
use defmt as _;
#[cfg(feature = "rtt")]
use defmt_rtt as _;

use panic_probe as _;

mod i2c_lcd_backpack;

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT_LOADER: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[rtic::app(
    device = rp2040_hal::pac,
    dispatchers = [SW0_IRQ, SW1_IRQ],
    peripherals = true,
)]
mod app {
    use core::fmt::Write;

    use crate::i2c_lcd_backpack::{LCDBackpack, Mcp23008};
    use embedded_hal::digital::OutputPin;
    use fugit::RateExtU32;
    use hal::gpio::{self, bank0, FunctionSio, PullDown, PullUp, SioOutput};
    use hal::pac;
    use hal::pio::PIOExt;
    use hal::Clock;
    use lcd::Display;
    use rp2040_hal as hal;
    use rtic_monotonics::rp2040::prelude::*;

    const XTAL_FREQ_HZ: u32 = 12_000_000;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        lcd: Display<LCDBackpack<rp2040_hal::I2C<rp2040_pac::I2C1, (gpio::Pin<bank0::Gpio2, gpio::FunctionI2c, PullUp>, gpio::Pin<bank0::Gpio3, gpio::FunctionI2c, PullUp>)>, Mono>>,
        led: gpio::Pin<bank0::Gpio13, FunctionSio<SioOutput>, PullDown>,
        neopixel: ws2812_pio::Ws2812Direct<
            pac::PIO0,
            rp2040_hal::pio::SM0,
            gpio::Pin<bank0::Gpio4, gpio::FunctionPio0, PullDown>,
        >,
    }
    rp2040_timer_monotonic!(Mono);

    #[init()]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        let mut watchdog = hal::Watchdog::new(ctx.device.WATCHDOG);
        let clocks = hal::clocks::init_clocks_and_plls(
            XTAL_FREQ_HZ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut ctx.device.RESETS,
            &mut watchdog,
        )
        .unwrap();

        //let mut delay = cortex_m::delay::Delay::new(ctx.core.SYST, clocks.system_clock.freq().to_Hz()); // Synchronous delay for use in init
        Mono::start(ctx.device.TIMER, &ctx.device.RESETS); // Async delay for use in tasks

        let sio = hal::Sio::new(ctx.device.SIO);
        let pins = gpio::bank0::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );

        // Init LED
        let mut led = pins.gpio13.into_push_pull_output();
        led.set_low().unwrap();

        // Init RGB LED
        let (mut pio0, sm0, _, _, _) = ctx.device.PIO0.split(&mut ctx.device.RESETS);
        let neopixel = ws2812_pio::Ws2812Direct::new(
            pins.gpio4.into_function(),
            &mut pio0,
            sm0,
            clocks.peripheral_clock.freq(),
        );

        // Init I2C
        let i2c = hal::I2C::new_controller(
            ctx.device.I2C1,
            pins.gpio2.reconfigure(),
            pins.gpio3.reconfigure(),
            400.kHz(),
            &mut ctx.device.RESETS,
            clocks.system_clock.freq(),
        );

        // Init LCD
        let mut lcd = Display::new(
            LCDBackpack::new(
                Mcp23008::new(i2c, 0x20),
                Mono
            )
        );
        lcd.init(lcd::FunctionLine::Line2, lcd::FunctionDots::Dots5x8);
        lcd.display(lcd::DisplayMode::DisplayOn, lcd::DisplayCursor::CursorOff, lcd::DisplayBlink::BlinkOff);
        lcd.clear();
        lcd.entry_mode(lcd::EntryModeDirection::EntryRight, lcd::EntryModeShift::NoShift);

        lcd.position(0, 0);
        lcd.write_str("Hello world!").unwrap();

        // Start tasks
        heartbeat::spawn().unwrap();
        rainbow::spawn().unwrap();

        // Enable sleep on exit ISR
        ctx.core.SCB.set_sleeponexit();

        // Return resources and timer
        (Shared {}, Local { lcd, led, neopixel })
    }

    #[idle()]
    fn idle(_: idle::Context) -> ! {
        loop {
            rtic::export::wfi();
        }
    }

    #[task(local = [led], priority = 2)]
    async fn heartbeat(ctx: heartbeat::Context) {
        use embedded_hal::digital::StatefulOutputPin;
        loop {
            ctx.local.led.toggle().unwrap();
            Mono::delay(1000.millis()).await;
        }
    }

    #[task(local = [lcd, neopixel], priority = 1)]
    async fn rainbow(ctx: rainbow::Context) {
        use smart_leds::{
            hsv::{hsv2rgb, Hsv},
            SmartLedsWrite,
        };
        use heapless::String;
        let mut hsv = Hsv {
            hue: 0,
            sat: 230,
            val: 40,
        };
        let mut buffer: String<20> = String::new();
        loop {
            let rgb = hsv2rgb(hsv);
            ctx.local.neopixel.write([rgb].iter().copied()).unwrap();
            
            if hsv.hue % 4 == 0 {
                buffer.clear();
                write!(buffer, "H {:3} S {:3} V {:3}", hsv.hue, hsv.sat, hsv.val).unwrap();
                ctx.local.lcd.position(0, 1);
                ctx.local.lcd.write_str(&buffer).unwrap();

                buffer.clear();
                write!(buffer, "R {:3} G {:3} B {:3}", rgb.r, rgb.g, rgb.b).unwrap();
                ctx.local.lcd.position(0, 2);
                ctx.local.lcd.write_str(&buffer).unwrap();
            }

            Mono::delay(20.millis()).await;
            hsv.hue = hsv.hue.wrapping_add(1);
        }
    }
}
