//! The control loop: one task on a high-priority interrupt executor owns the
//! firmware `State` and is the only caller of `step`.
//!
//! - IMU data-ready is stamped in the EXTI1 interrupt, so a sample's time is
//!   when the sensor had it, whatever the loop was doing. If the loop has not
//!   read the previous sample yet, that sample is lost and counted.
//! - RC and battery events come from thread-mode tasks through `EVENTS`.
//! - `Tick` comes from the loop's own 1 ms timer. Nothing else produces one:
//!   a dead data-ready line gives ticks and no `Imu` events, so the
//!   firmware's IMU timeout blocks arming or disarms.
//!
//! `STATS` counts samples, losses, and the cycle cost of `step`.

use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

use cortex_m::peripheral::DWT;
use embassy_futures::select::{Either3, select3};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output};
use embassy_stm32::mode::Blocking;
use embassy_stm32::{interrupt, pac};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use ironloop_fc::{Effect, Event, EventKind, State, step};

use crate::{dshot, imu};

const TICK: Duration = Duration::from_millis(1);

/// RC and battery events from thread mode, in arrival order.
static EVENTS: Channel<CriticalSectionRawMutex, Event, 32> = Channel::new();

/// Data-ready time (µs) of the newest IMU sample not yet read.
static DRDY: Signal<CriticalSectionRawMutex, u64> = Signal::new();

/// Bench counters, for the planned USB diagnostics (SWD is not reachable on
/// this board).
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
}

#[unsafe(no_mangle)]
pub static STATS: Stats = Stats {
    imu_samples: AtomicU32::new(0),
    drdy_missed: AtomicU32::new(0),
    imu_read_errors: AtomicU32::new(0),
    events_dropped: AtomicU32::new(0),
    step_cycles_last: AtomicU32::new(0),
    step_cycles_max: AtomicU32::new(0),
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
    mut imu: Option<imu::Imu>,
    _drdy: ExtiInput<'static, Blocking>,
    mut motors: dshot::Dshot,
    mut led: Output<'static>,
    mut beeper: Output<'static>,
) {
    let mut state = State::default();
    // Edges during IMU set-up carry no sample worth a step.
    DRDY.reset();
    let mut next_tick = Instant::now() + TICK;
    loop {
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
                Event { t_us: now.as_micros(), kind: EventKind::Tick }
            }
        };

        let start = DWT::cycle_count();
        let (next, effects) = step(state, &event);
        state = next;
        // Only the last motor command of a step goes out: an earlier one
        // would be superseded within microseconds anyway.
        let mut command = None;
        for effect in effects {
            match effect {
                Effect::Motors(m) => command = Some(m),
                // Status LED on PC13 is active-low.
                Effect::Led(on) => led.set_level(if on { Level::Low } else { Level::High }),
                Effect::Beeper(on) => beeper.set_level(Level::from(on)),
                Effect::Notice(_) => {}
                Effect::RebootToBootloader => {
                    motors.write([0; 4]);
                    motors.wait_idle();
                    crate::reboot_to_bootloader();
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
}
