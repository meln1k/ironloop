//! The control loop: one task on a high-priority interrupt executor owns the
//! firmware `State` and is the only caller of `step`.
//!
//! - IMU data-ready is stamped in the EXTI1 interrupt, so a sample's time is
//!   when the sensor had it, whatever the loop was doing. If the loop has not
//!   read the previous sample yet, that sample is lost and counted.
//! - RC and battery events come from thread-mode tasks through `EVENTS`.
//! - `Tick` comes from the loop's own 1 ms timer, along with the ESCs'
//!   eRPM collected since the last one. Nothing else produces one:
//!   a dead data-ready line gives ticks and no `Imu` events, so the
//!   firmware's IMU timeout blocks arming or disarms.
//!
//! `STATS` counts samples, losses, and the cycle cost of `step`, and keeps
//! the newest RC channels and battery current. Every 100 ticks the loop
//! also publishes the firmware's `Status` in `STATUS`, and it queues the
//! firmware's notices in `NOTICES`; `usb` prints them on the USB serial
//! port.

use core::sync::atomic::{AtomicU8, AtomicU16, AtomicU32, Ordering::Relaxed};

use cortex_m::peripheral::DWT;
use embassy_futures::select::{Either3, select3};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals::IWDG;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::{interrupt, pac};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use ironloop_fc::{Config, Effect, Event, EventKind, Notice, RC_CHANNELS, State, Status, step};

use crate::{dshot, imu};

const TICK: Duration = Duration::from_millis(1);
/// Ticks between `STATUS` updates (10 Hz).
const STATUS_TICKS: u32 = 100;

/// RC and battery events from thread mode, in arrival order.
static EVENTS: Channel<CriticalSectionRawMutex, Event, 32> = Channel::new();

/// Data-ready time (µs) of the newest IMU sample not yet read.
static DRDY: Signal<CriticalSectionRawMutex, u64> = Signal::new();

/// The firmware's latest `Status`, for the USB status line.
pub static STATUS: Signal<CriticalSectionRawMutex, Status> = Signal::new();

/// The firmware's notices (why it disarmed, refused to arm, reported a
/// motor or the battery) with their event time in µs, newest kept when
/// full, for the USB port.
pub static NOTICES: Channel<CriticalSectionRawMutex, (u64, Notice), 16> = Channel::new();

/// Bench counters, printed on the USB serial port by `usb` (SWD is not
/// reachable on this board).
pub struct Stats {
    /// IMU samples read and stepped.
    pub imu_samples: AtomicU32,
    /// Data-ready edges that came before the previous sample was read.
    pub drdy_missed: AtomicU32,
    /// SPI reads that failed (the sample is skipped).
    pub imu_read_errors: AtomicU32,
    /// RC and battery events dropped because `EVENTS` was full.
    pub events_dropped: AtomicU32,
    /// CPU cycles (96 MHz) of the last and the longest `step` plus applying
    /// its effects. The BMI270 period is 312 µs, 30,000 cycles.
    pub step_cycles_last: AtomicU32,
    pub step_cycles_max: AtomicU32,
    /// The newest valid eRPM reply per motor output (0 before the first).
    pub erpm: [AtomicU32; 4],
    /// The newest RC frame's channels, as decoded (µs).
    pub rc: [AtomicU16; RC_CHANNELS],
    /// The newest battery current reading (A, `f32` bits).
    pub amps: AtomicU32,
    /// Which IMU was found: 0 none, 1 BMI270, 2 ICM-42688.
    pub imu_chip: AtomicU8,
    /// Notices dropped (the oldest) because `NOTICES` was full.
    pub notices_dropped: AtomicU32,
}

#[unsafe(no_mangle)]
pub static STATS: Stats = Stats {
    imu_samples: AtomicU32::new(0),
    drdy_missed: AtomicU32::new(0),
    imu_read_errors: AtomicU32::new(0),
    events_dropped: AtomicU32::new(0),
    step_cycles_last: AtomicU32::new(0),
    step_cycles_max: AtomicU32::new(0),
    erpm: [const { AtomicU32::new(0) }; 4],
    rc: [const { AtomicU16::new(0) }; RC_CHANNELS],
    amps: AtomicU32::new(0),
    imu_chip: AtomicU8::new(0),
    notices_dropped: AtomicU32::new(0),
};

fn now_us() -> u64 {
    Instant::now().as_micros()
}

/// Queues an event from a thread-mode task, stamped now.
pub fn send(kind: EventKind) {
    if EVENTS.try_send(Event { t_us: now_us(), kind }).is_err() {
        STATS.events_dropped.fetch_add(1, Relaxed);
    }
}

/// EXTI1 (IMU data-ready on PA1). Runs above the control executor, so it
/// stamps every edge even while a step is running.
#[interrupt]
fn EXTI1() {
    let t = now_us();
    pac::EXTI.pr(0).write(|w| w.set_line(1, true));
    if DRDY.signaled() {
        STATS.drdy_missed.fetch_add(1, Relaxed);
    }
    DRDY.signal(t);
}

#[embassy_executor::task]
pub async fn control_task(
    config: Config,
    mut imu: Option<imu::Imu>,
    _drdy: ExtiInput<'static, Blocking>,
    mut motors: dshot::Dshot,
    mut led: Output<'static>,
    mut beeper: Output<'static>,
    mut watchdog: IndependentWatchdog<'static, IWDG>,
) {
    let mut state = State::new(config);
    watchdog.unleash();
    // Edges during IMU set-up carry no sample worth a step.
    DRDY.reset();
    let mut next_tick = Instant::now() + TICK;
    let mut ticks = 0u32;
    loop {
        // Every pass takes at most about a millisecond (the tick), so a
        // loop that stops passing here is hung.
        watchdog.pet();
        // A motor command held back by a frame still on the wire goes out
        // now; the next pass comes within a tick.
        motors.flush();
        // Queued RC and battery events go first: they were stamped before
        // anything the loop has not yet seen, so time stays in order even
        // after a long step.
        let event = match select3(EVENTS.receive(), DRDY.wait(), Timer::at(next_tick)).await {
            Either3::First(event) => event,
            Either3::Second(t_us) => {
                let Some(imu) = imu.as_mut() else { continue };
                match imu.sample().await {
                    Ok((gyro, accel)) => {
                        STATS.imu_samples.fetch_add(1, Relaxed);
                        Event { t_us, kind: EventKind::Imu { gyro, accel } }
                    }
                    Err(_) => {
                        STATS.imu_read_errors.fetch_add(1, Relaxed);
                        continue;
                    }
                }
            }
            Either3::Third(()) => {
                let now = Instant::now();
                next_tick += TICK;
                if next_tick <= now {
                    next_tick = now + TICK;
                }
                let t_us = now.as_micros();
                // The ESCs' eRPM since the last tick goes in just before it.
                ticks += 1;
                if ticks % STATUS_TICKS == 0 {
                    STATUS.signal(state.status());
                }
                if let Some(erpm) = motors.take_telemetry() {
                    for (stat, reading) in STATS.erpm.iter().zip(erpm) {
                        if let Some(v) = reading {
                            stat.store(v, Relaxed);
                        }
                    }
                    let telemetry = Event { t_us, kind: EventKind::MotorTelemetry { erpm } };
                    run(&mut state, &telemetry, &mut motors, &mut led, &mut beeper);
                }
                Event { t_us, kind: EventKind::Tick }
            }
        };
        run(&mut state, &event, &mut motors, &mut led, &mut beeper);
    }
}

/// Steps the firmware on one event and applies its effects.
fn run(state: &mut State, event: &Event, motors: &mut dshot::Dshot, led: &mut Output<'static>, beeper: &mut Output<'static>) {
    let start = DWT::cycle_count();
    match event.kind {
        EventKind::Rc { channels } => {
            for (stat, c) in STATS.rc.iter().zip(channels) {
                stat.store(c, Relaxed);
            }
        }
        EventKind::Battery { amps, .. } => STATS.amps.store(amps.to_bits(), Relaxed),
        _ => {}
    }
    let effects = step(state, event);
    // Only the last motor command of a step goes out: an earlier one
    // would be superseded within microseconds anyway.
    let mut command = None;
    for effect in effects {
        match effect {
            Effect::Motors(m) => command = Some(m),
            // Status LED on PC13 is active-low.
            Effect::Led(on) => led.set_level(if on { Level::Low } else { Level::High }),
            Effect::Beeper(on) => beeper.set_level(Level::from(on)),
            Effect::Notice(n) => {
                let note = (event.t_us, n);
                if NOTICES.try_send(note).is_err() {
                    // Keep the newest: drop the oldest instead.
                    let _ = NOTICES.try_receive();
                    let _ = NOTICES.try_send(note);
                    STATS.notices_dropped.fetch_add(1, Relaxed);
                }
            }
            Effect::RebootToBootloader => {
                // The stop frame goes out whole before the reset.
                motors.wait_idle();
                motors.write([0; 4]);
                motors.wait_idle();
                crate::dfu::reboot();
            }
        }
    }
    if let Some(m) = command {
        motors.write(m);
    }
    let cycles = DWT::cycle_count().wrapping_sub(start);
    STATS.step_cycles_last.store(cycles, Relaxed);
    STATS.step_cycles_max.fetch_max(cycles, Relaxed);
}
