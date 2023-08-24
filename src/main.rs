//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::{
    entry,
    hal::{
        clocks::ClocksManager,
        rtc::{DateTime, DateTimeFilter, DayOfWeek::Monday, RealTimeClock},
        Clock,
    },
};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use fugit::RateExtU32;
use pac::interrupt;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{pac, sio::Sio};

#[interrupt]
unsafe fn RTC_IRQ() {
    info!("RTC IRQ!");
    let rtc = pac::Peripherals::steal().RTC;
    rtc.irq_setup_0.modify(|_, s| s.match_ena().clear_bit());

    while rtc.irq_setup_0.read().match_active().bit() {
        core::hint::spin_loop();
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let sio = Sio::new(pac.SIO);
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led = pins.led.into_push_pull_output();
    let _ = led.set_high();

    let external_xtal_freq_hz = 12_000_000u32.Hz();

    // enable RTC in sleep. This is not yet supported in the clock manager.
    pac.CLOCKS
        .sleep_en0
        .modify(|_, w| w.clk_sys_rtc().set_bit());
    let mut clocks = ClocksManager::new(pac.CLOCKS);
    // we boot on rosc
    // stabilize xosc
    let xosc = bsp::hal::xosc::setup_xosc_blocking(pac.XOSC, external_xtal_freq_hz).unwrap();
    let rosc = bsp::hal::rosc::RingOscillator::new(pac.ROSC).initialize();
    // use xosc at 12MHz for clk_ref -> clk_sys -> clk_peri
    //
    clocks
        .reference_clock
        .configure_clock(&xosc, external_xtal_freq_hz)
        .unwrap();
    clocks
        .system_clock
        .configure_clock(&clocks.reference_clock, external_xtal_freq_hz)
        .unwrap();
    clocks
        .peripheral_clock
        .configure_clock(&clocks.system_clock, external_xtal_freq_hz)
        .unwrap();
    // use xosc at 12MHz/256 for clk_rtc
    clocks.rtc_clock.configure_clock(&rosc, 46875.Hz()).unwrap();

    // see ARMv6-M Architectural Reference Manual chapter B3.2.7, table B3-9.
    unsafe {
        core.SCB.scr.modify(|w| w | 0x4);
    };

    let initial_date = DateTime {
        year: 0,
        month: 1,
        day: 1,
        day_of_week: Monday,
        hour: 0,
        minute: 0,
        second: 0,
    };
    let mut rtc =
        RealTimeClock::new(pac.RTC, clocks.rtc_clock, &mut pac.RESETS, initial_date).unwrap();
    // wake in 5 seconds
    rtc.schedule_alarm(DateTimeFilter::default().second(5));
    // this is missing from the RTC driver
    unsafe {
        pac::Peripherals::steal()
            .RTC
            .inte
            .modify(|_, w| w.rtc().set_bit());
    }
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::RTC_IRQ);
    }

    // wait 1 second
    cortex_m::asm::delay(external_xtal_freq_hz.to_Hz());
    info!("going to deep sleep at: {}", rtc.now().unwrap().second);
    let _ = led.set_low();

    // Enter deep sleep for 4more seconds.
    cortex_m::asm::wfi();
    let _ = led.set_high();

    // invoke dormant mode
    cortex_m::asm::delay(external_xtal_freq_hz.to_Hz());
    rtc.schedule_alarm(DateTimeFilter::default().second(10));
    info!("going dormant at: {}", rtc.now().unwrap().second);
    let _ = led.set_low();
    unsafe {
        xosc.dormant();
    }

    info!("Yay !!");
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    loop {
        info!("on!");
        led.set_high().unwrap();
        delay.delay_ms(500);
        info!("off!");
        led.set_low().unwrap();
        delay.delay_ms(500);
    }
}

// End of file
