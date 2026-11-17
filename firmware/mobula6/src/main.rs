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
//! A panic or a hung control loop (through the independent watchdog) resets
//! the board. The ESCs stop on the missing DShot signal, and after the reset
//! arming needs the arm switch seen off first, so a reset in flight cannot
//! re-arm by itself.
//!
//! Pinout and clocking come from docs/hardware/mobula6-eco-2024.md; see the
//! README for what still needs a bench check.

#![no_std]
#![no_main]

mod bmi270_config;
mod control;
mod dfu;
mod dshot;
mod imu;

use control::send;
use embassy_executor::{Executor, InterruptExecutor, Spawner};
use embassy_stm32::Peri;
use embassy_stm32::adc::{Adc, AdcChannel, AnyAdcChannel, SampleTime};
use embassy_stm32::exti::{ExtiInput, TriggerEdge};
use embassy_stm32::gpio::{AnyPin, Level, Output, Pull, Speed};
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::peripherals::{ADC1, DMA1_CH5, DMA2_CH0, DMA2_CH3, USART2};
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{self, RingBufferedUartRx, UartRx};
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::{bind_interrupts, dma, interrupt};
use embassy_time::{Duration, Ticker};
use ironloop_fc::proto::CrsfParser;
use ironloop_fc::{Config, EventKind};
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
    const VBAT_DIVIDER: f32 = 11.0; // Betaflight vbat_scale 110
    // Betaflight's ibata_scale 470 is 47 mV/A, but on this quad it reads
    // about 25% high against a meter (QuadMath), so 47 × 1.25 mV/A.
    const AMPS_PER_VOLT: f32 = 1.0 / (0.047 * 1.25);
    // The internal reference's reading at 3.3 V, measured in the factory
    // (STM32F411 datasheet, "Internal reference voltage calibration values").
    // SAFETY: a read-only word in system memory.
    let vrefint_cal = unsafe { core::ptr::read_volatile(0x1FFF_7A2A as *const u16) } as f32;
    let mut vrefint = adc.enable_vrefint();
    let mut ticker = Ticker::every(Duration::from_millis(20));
    loop {
        ticker.next().await;
        // The ADC's reference is the 3.3 V rail, whose own error would
        // otherwise become battery-voltage (and so throttle-gain) error.
        let raw_ref = adc.blocking_read(&mut vrefint, SampleTime::CYCLES480).max(1);
        let volts_per_count = 3.3 * vrefint_cal / raw_ref as f32 / 4095.0;
        let raw_v = adc.blocking_read(&mut vbat, SampleTime::CYCLES112);
        let raw_i = adc.blocking_read(&mut ibat, SampleTime::CYCLES112);
        let volts = raw_v as f32 * volts_per_count * VBAT_DIVIDER;
        let amps = raw_i as f32 * volts_per_count * AMPS_PER_VOLT;
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

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    cortex_m::peripheral::SCB::sys_reset()
}

#[cortex_m_rt::entry]
fn main() -> ! {
    // First, before the clocks or any peripheral are touched.
    dfu::jump_if_requested();
    static EXECUTOR: StaticCell<Executor> = StaticCell::new();
    EXECUTOR.init(Executor::new()).run(|spawner| spawner.spawn(init(spawner).unwrap()))
}

#[embassy_executor::task]
async fn init(spawner: Spawner) {
    let p = embassy_stm32::init(clocks());

    if let Some(mut core) = cortex_m::Peripherals::take() {
        // Cycle counter for `control::STATS`.
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();
    }

    let led = Output::new(p.PC13, Level::High, Speed::Low); // off
    let beeper = Output::new(p.PC15, Level::Low, Speed::Low);
    let idle = Level::from(dshot::Dshot::IDLE_HIGH);
    let pin = |o: Peri<'static, AnyPin>| Output::new(o, idle, Speed::Medium);
    let motors = dshot::Dshot::new(p.TIM1, p.DMA2_CH5, [pin(p.PB10.into()), pin(p.PB6.into()), pin(p.PB7.into()), pin(p.PB8.into())]);
    // The firmware streams stop commands from the first IMU sample; until
    // then the ESCs see an idle line, which they treat as no signal.

    let mut spi_cfg = spi::Config::default();
    spi_cfg.mode = spi::MODE_3;
    spi_cfg.frequency = Hertz(1_000_000);
    let spi = Spi::new(p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA2_CH3, p.DMA2_CH0, Irqs, spi_cfg);
    let cs = Output::new(p.PA4, Level::High, Speed::VeryHigh);
    // Without an IMU no `Imu` events arrive, so the firmware reports the
    // IMU arming blocker and never spins the motors.
    let imu = imu::Imu::init(spi, cs).await.ok();
    // The board knows its sensor; the firmware's defaults do not.
    let config = Config {
        gyro_range: imu::GYRO_RANGE,
        accel_range: imu::ACCEL_RANGE,
        imu_alignment: imu.as_ref().map_or(Config::default().imu_alignment, imu::Imu::alignment),
        ..Config::default()
    };

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
    // Fed by the control loop at least every millisecond. The LSI clock is
    // 17-47 kHz, so 50 ms nominal is 34-94 ms.
    let watchdog = IndependentWatchdog::new(p.IWDG, 50_000);
    // The battery pins become analog inputs before DShot starts: that
    // rewrites GPIOB's mode register, which the DShot interrupt also does
    // around every reply window, without a lock.
    let (vbat, ibat) = (p.PB0.degrade_adc(), p.PB1.degrade_adc());
    let control = CONTROL.start(interrupt::SPI5);
    control.spawn(control::control_task(config, imu, drdy, motors, led, beeper, watchdog).unwrap());

    let mut uart_cfg = usart::Config::default();
    uart_cfg.baudrate = 420_000; // CRSF
    // Without the receiver no `Rc` events arrive: the RC link arming blocker.
    if let Ok(rx) = UartRx::new(p.USART2, p.PA3, p.DMA1_CH5, Irqs, uart_cfg) {
        let rx = rx.into_ring_buffered(CRSF_DMA_BUF.init([0; 256]));
        spawner.spawn(rc_task(rx).unwrap());
    }

    let adc = Adc::new(p.ADC1);
    spawner.spawn(battery_task(adc, vbat, ibat).unwrap());
}
