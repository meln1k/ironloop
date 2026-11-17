//! IronLoop on the HDZero Mobula6 ECO 2024 (STM32F411CEU6, embassy).
//!
//! The flight controller itself is `ironloop_fc::step`, the same pure
//! function the simulator and the browser run. This crate only turns
//! hardware into timestamped events and effects back into hardware:
//!
//! - `control::control_task`, on a high-priority interrupt executor: owns
//!   the firmware `State`, reads the IMU on data-ready, generates the 1 kHz
//!   `Tick`, runs `step` on each event and applies the effects (DShot,
//!   status LED, beeper, DFU reboot).
//! - `rc_task`, thread mode: ELRS CRSF bytes on USART2 → `Rc` event
//! - `battery_task`, thread mode: ADC VBAT/current at 50 Hz → `Battery` event
//!
//! Pinout and clocking come from docs/hardware notes for this board; see
//! README for what still needs a bench check.

#![no_std]
#![no_main]

mod bmi270_config;
mod control;
mod dshot;
mod imu;

use core::mem::MaybeUninit;

use control::send;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::Peri;
use embassy_stm32::adc::{Adc, AdcChannel, AnyAdcChannel, SampleTime};
use embassy_stm32::exti::{ExtiInput, TriggerEdge};
use embassy_stm32::gpio::{AnyPin, Level, Output, Pull, Speed};
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::peripherals::{ADC1, DMA1_CH5, DMA2_CH0, DMA2_CH3, USART2};
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{self, RingBufferedUartRx, UartRx};
use embassy_stm32::{bind_interrupts, dma, interrupt};
use embassy_time::{Duration, Ticker};
use ironloop_fc::EventKind;
use ironloop_fc::proto::CrsfParser;
use panic_halt as _;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<USART2>;
    DMA1_STREAM5 => dma::InterruptHandler<DMA1_CH5>;
    DMA2_STREAM0 => dma::InterruptHandler<DMA2_CH0>;
    DMA2_STREAM3 => dma::InterruptHandler<DMA2_CH3>;
});

/// Runs the control task. SPI5 is unused on this board; its interrupt only
/// schedules the executor.
static CONTROL: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn SPI5() {
    // SAFETY: SPI5 is the interrupt `CONTROL` was started on.
    unsafe { CONTROL.on_interrupt() }
}

// ---- DFU bootloader entry -------------------------------------------------
//
// The SWD clock pin doubles as the blackbox flash chip-select on this board,
// so reflashing goes through the ROM USB DFU bootloader. The firmware asks for
// it with `Effect::RebootToBootloader` (a disarmed stick gesture). We set a
// flag in RAM that survives reset, reset, and jump before anything else
// touches the clocks or peripherals.

const BOOT_MAGIC: u32 = 0xB007_10AD;
const SYSTEM_MEMORY: u32 = 0x1FFF_0000;

#[unsafe(link_section = ".uninit.BOOTFLAG")]
static mut BOOT_FLAG: MaybeUninit<u32> = MaybeUninit::uninit();

fn jump_to_bootloader_if_requested() {
    // SAFETY: single-threaded, before any interrupt is enabled.
    unsafe {
        let flag = (&raw mut BOOT_FLAG).cast::<u32>();
        if core::ptr::read_volatile(flag) != BOOT_MAGIC {
            return;
        }
        core::ptr::write_volatile(flag, 0);
        // Map system memory at 0x0 the way the ROM bootloader expects.
        let rcc = embassy_stm32::pac::RCC;
        rcc.apb2enr().modify(|w| w.set_syscfgen(true));
        embassy_stm32::pac::SYSCFG.memrm().modify(|w| w.set_mem_mode(1));
        cortex_m::asm::bootload(SYSTEM_MEMORY as *const u32);
    }
}

pub fn reboot_to_bootloader() -> ! {
    // SAFETY: plain volatile store to a word only this module uses.
    unsafe { core::ptr::write_volatile((&raw mut BOOT_FLAG).cast::<u32>(), BOOT_MAGIC) };
    cortex_m::peripheral::SCB::sys_reset();
}

// ---- Tasks ----------------------------------------------------------------

#[embassy_executor::task]
async fn rc_task(mut rx: RingBufferedUartRx<'static>) {
    let mut parser = CrsfParser::default();
    let mut buf = [0u8; 64];
    loop {
        match rx.read(&mut buf).await {
            Ok(n) => {
                for &b in &buf[..n] {
                    if let Some(channels) = parser.push(b) {
                        send(EventKind::Rc { channels });
                    }
                }
            }
            // Overrun or framing error: drop the partial frame and resync.
            Err(_) => parser = CrsfParser::default(),
        }
    }
}

#[embassy_executor::task]
async fn battery_task(mut adc: Adc<'static, ADC1>, mut vbat: AnyAdcChannel<'static, ADC1>, mut ibat: AnyAdcChannel<'static, ADC1>) {
    const VREF: f32 = 3.3;
    const VBAT_DIVIDER: f32 = 11.0; // Betaflight vbat_scale 110
    const AMPS_PER_VOLT: f32 = 1.0 / 0.047; // ibata_scale 470; reads ~25% high, calibrate
    let mut ticker = Ticker::every(Duration::from_millis(20));
    loop {
        ticker.next().await;
        let raw_v = adc.blocking_read(&mut vbat, SampleTime::CYCLES112);
        let raw_i = adc.blocking_read(&mut ibat, SampleTime::CYCLES112);
        let volts = raw_v as f32 * (VREF / 4095.0) * VBAT_DIVIDER;
        let amps = raw_i as f32 * (VREF / 4095.0) * AMPS_PER_VOLT;
        send(EventKind::Battery { volts, amps });
    }
}

// ---- Board bring-up -------------------------------------------------------

fn clocks() -> embassy_stm32::Config {
    use embassy_stm32::rcc::*;
    let mut config = embassy_stm32::Config::default();
    // 8 MHz crystal → 96 MHz SYSCLK, 48 MHz USB, APB1 48 MHz, APB2 96 MHz.
    config.rcc.hse = Some(Hse { freq: Hertz(8_000_000), mode: HseMode::Oscillator });
    config.rcc.pll_src = PllSource::HSE;
    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV4,
        mul: PllMul::MUL192,
        divp: Some(PllPDiv::DIV4),
        divq: Some(PllQDiv::DIV8),
        divr: None,
    });
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV2;
    config.rcc.apb2_pre = APBPrescaler::DIV1;
    config
}

static CRSF_DMA_BUF: StaticCell<[u8; 256]> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    jump_to_bootloader_if_requested();
    let p = embassy_stm32::init(clocks());

    if let Some(mut core) = cortex_m::Peripherals::take() {
        // Cycle counter for `control::STATS`.
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();
    }

    let led = Output::new(p.PC13, Level::High, Speed::Low); // off
    let beeper = Output::new(p.PC15, Level::Low, Speed::Low);
    let pin = |o: Peri<'static, AnyPin>| Output::new(o, Level::Low, Speed::Medium);
    let motors = dshot::Dshot::new(p.TIM1, p.DMA2_CH5, [pin(p.PB10.into()), pin(p.PB6.into()), pin(p.PB7.into()), pin(p.PB8.into())]);
    // The firmware streams stop commands from the first IMU sample; until
    // then the ESCs see an idle-low line, which they treat as no signal.

    let mut spi_cfg = spi::Config::default();
    spi_cfg.mode = spi::MODE_3;
    spi_cfg.frequency = Hertz(1_000_000);
    let spi = Spi::new(p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA2_CH3, p.DMA2_CH0, Irqs, spi_cfg);
    let cs = Output::new(p.PA4, Level::High, Speed::VeryHigh);
    // Without an IMU no `Imu` events arrive, so the firmware reports the
    // IMU arming blocker and never spins the motors.
    let imu = imu::Imu::init(spi, cs).await.ok();

    // The control loop starts only now, so gyro calibration (timed from the
    // first event) does not run while the IMU is still being set up.
    // Priorities, most urgent first: data-ready stamping and the time
    // driver and DMA (P0, embassy's default), then the control loop (P1),
    // then everything in thread mode.
    let drdy = ExtiInput::new_blocking(p.PA1, p.EXTI1, Pull::Down, TriggerEdge::Rising);
    interrupt::EXTI1.set_priority(Priority::P0);
    // SAFETY: `control::EXTI1` handles this interrupt and shares nothing
    // with code that relies on it being masked.
    unsafe { interrupt::EXTI1.enable() };
    interrupt::SPI5.set_priority(Priority::P1);
    let control = CONTROL.start(interrupt::SPI5);
    control.spawn(control::control_task(imu, drdy, motors, led, beeper).unwrap());

    let mut uart_cfg = usart::Config::default();
    uart_cfg.baudrate = 420_000; // CRSF
    if let Ok(rx) = UartRx::new(p.USART2, p.PA3, p.DMA1_CH5, Irqs, uart_cfg) {
        let rx = rx.into_ring_buffered(CRSF_DMA_BUF.init([0; 256]));
        spawner.spawn(rc_task(rx).unwrap());
    }

    let adc = Adc::new(p.ADC1);
    spawner.spawn(battery_task(adc, p.PB0.degrade_adc(), p.PB1.degrade_adc()).unwrap());
}
