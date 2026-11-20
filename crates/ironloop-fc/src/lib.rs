//! IronLoop flight controller core.
//!
//! The whole firmware is one function, [`step`]: it takes the current
//! [`State`] and one timestamped [`Event`] (an IMU sample, an RC frame, a
//! battery reading, ESC telemetry, a housekeeping tick or a command),
//! advances the state in place, and returns the [`Effect`]s the board must
//! perform (motor outputs, LED, beeper, notices, bootloader jump). It never reads a clock,
//! touches hardware, allocates, or keeps hidden state, so the same code runs
//! on the STM32F411 under embassy, in the deterministic simulator, and in the
//! browser.

#![no_std]

pub mod config;
mod control;
mod dyn_notch;
mod failsafe;
mod health;
mod imu;
pub mod math;
mod protect;
pub mod proto;
pub mod rc;
pub mod rpm;

pub use config::{AxisPid, Config, ImuAlignment, Rates};
use heapless::Vec;
use imu::GyroCal;
use math::{Lpf, Quat, Vec3};
pub use rc::ch;
use rc::RcInput;

pub const RC_CHANNELS: usize = 8;
/// Upper bound on effects produced by a single step.
pub const MAX_EFFECTS: usize = 8;
pub type Effects = Vec<Effect, MAX_EFFECTS>;

/// Effects being collected during one step, and how many did not fit.
#[derive(Default)]
struct Out {
    list: Effects,
    dropped: u32,
}

pub const GRAVITY: f32 = 9.80665;
/// DShot command 0 stops a motor; 48..=2047 is the throttle range.
pub const DSHOT_STOP: u16 = 0;
pub const DSHOT_MIN: u16 = 48;
pub const DSHOT_MAX: u16 = 2047;

/// Throttle (0..1) at or below which the stick counts as down: arming is
/// allowed, the I-term rests, and a link lost there was lost on the ground.
const THROTTLE_LOW: f32 = 0.05;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Event {
    /// Microseconds since an arbitrary epoch, from the board's monotonic clock.
    pub t_us: u64,
    pub kind: EventKind,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EventKind {
    /// One IMU sample in the sensor's own frame: gyro in rad/s, accel in
    /// m/s² (specific force; reads about -9.8 on z when level in FRD).
    Imu { gyro: [f32; 3], accel: [f32; 3] },
    /// One decoded RC frame, channels in µs (1000..2000 nominal).
    Rc { channels: [u16; RC_CHANNELS] },
    /// Battery voltage (V) and current (A).
    Battery { volts: f32, amps: f32 },
    /// Bidirectional DShot eRPM per motor; `None` when a frame was lost.
    MotorTelemetry { erpm: [Option<u32>; 4] },
    /// Housekeeping tick from a free-running timer (about 1 kHz).
    Tick,
    Command(Command),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    /// Jump to the STM32 ROM DFU bootloader (ignored while armed). Also
    /// triggered by holding the bootloader stick gesture for 2 s while
    /// disarmed: arm switch off, throttle low, yaw full left, pitch full
    /// back, roll full right.
    RebootToBootloader,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Effect {
    /// DShot value per motor, in Betaflight order (rear-right, front-right,
    /// rear-left, front-left). 0 stops the motor.
    Motors([u16; 4]),
    Led(bool),
    Beeper(bool),
    Notice(Notice),
    RebootToBootloader,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlightMode {
    Acro,
    Angle,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DisarmReason {
    Switch,
    /// RC lost while on the ground, or failsafe landing finished.
    Failsafe,
    ImuFailure,
    RunawayTakeoff,
    Crash,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BatteryLevel {
    Ok,
    Warning,
    Critical,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FailsafeStage {
    None,
    /// Link lost: sticks centered, self-leveling, throttle held.
    Hold,
    /// Link still lost: self-leveling at landing throttle, then disarm.
    Landing,
}

/// Reasons arming was refused, as bit flags.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ArmBlockers(pub u16);

impl ArmBlockers {
    pub const CALIBRATING: u16 = 1 << 0;
    pub const THROTTLE: u16 = 1 << 1;
    pub const RC_LINK: u16 = 1 << 2;
    pub const IMU: u16 = 1 << 3;
    pub const TILT: u16 = 1 << 4;
    pub const BATTERY: u16 = 1 << 5;

    pub fn contains(self, bit: u16) -> bool {
        self.0 & bit != 0
    }
    pub fn is_empty(self) -> bool {
        self.0 == 0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Notice {
    GyroCalibrated,
    Armed(FlightMode),
    ArmingBlocked(ArmBlockers),
    Disarmed(DisarmReason),
    ModeChanged(FlightMode),
    Failsafe(FailsafeStage),
    FailsafeRecovered,
    RcFrameRejected,
    ImuSampleRejected,
    GyroSaturated,
    Battery(BatteryLevel),
    MotorFault(u8),
    MotorRecovered(u8),
    ClockWentBackwards,
}

/// Read-only summary of the state for OSD, telemetry, and visualization.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Status {
    pub armed: bool,
    pub mode: FlightMode,
    pub failsafe: FailsafeStage,
    pub calibrated: bool,
    pub attitude: Quat,
    pub gyro: Vec3,
    pub rate_setpoint: Vec3,
    pub throttle: f32,
    /// Throttle the firmware asks for (pilot stick or failsafe), before
    /// battery compensation and airmode.
    pub throttle_demand: f32,
    pub battery_volts: Option<f32>,
    pub battery_level: BatteryLevel,
    pub motors: [u16; 4],
    pub motor_faults: [bool; 4],
    pub airmode_active: bool,
    pub blockers: ArmBlockers,
    /// Vertical speed estimate from the accelerometer (m/s, up positive).
    /// Leaky in normal flight; integrated during failsafe.
    pub climb_rate: f32,
    /// Throttle believed to hold altitude (learned in flight), as it would
    /// be with the pack at the sag-compensation reference voltage.
    pub hover_throttle: f32,
    /// The firmware's clock (µs, the newest event time seen).
    pub now_us: u64,
    /// Effects that did not fit in a step's output since boot.
    pub effects_dropped: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Rc {
    last_valid_us: Option<u64>,
    /// Start of the current run of valid frames (no gap above the timeout).
    valid_since_us: u64,
    /// The latest valid frame.
    input: RcInput,
    arm_switch: bool,
}

/// What the firmware knows about the current flight. Arming starts a new
/// one, so nothing from an earlier flight carries over except the vertical
/// speed, which is integrated disarmed too.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Flight {
    /// Leaky integral of vertical acceleration (m/s, down positive). It
    /// forgets steady motion over a few seconds, but shows climbs and
    /// descents as they start and end. Kept across arming.
    vz_est: f32,
    airmode_active: bool,
    integral: Vec3,
    mixer_saturated: bool,
    /// Low-passed setpoint; the setpoint minus this is how fast the pilot
    /// is moving the sticks (I-term relax).
    setpoint_lpf: Lpf,
    setpoint: Vec3,
    last_pid: Vec3,
    throttle_demand: f32,
    runaway_since_us: Option<u64>,
    /// Since when the PIDs have pushed at their limit against a clipped
    /// gyro, and the yaw output against the yaw rate (`runaway_and_crash`).
    clipped_since_us: Option<u64>,
    /// The last step on which the gyro was clipped with the PIDs pushing
    /// against it.
    clipped_last_us: u64,
    yaw_opposed_since_us: Option<u64>,
    /// When the yaw output started pushing hard against the yaw rate, and
    /// how fast it was then (rad/s).
    yaw_pushed_since: Option<(u64, f32)>,
    /// Since when the frame, never lifted off, has not turned the way the
    /// controller asks (`runaway_and_crash`).
    stuck_since_us: Option<u64>,
    /// Since when the frame, never lifted off, has had the ground pushing
    /// on its side, and the last step on which it did.
    on_side_since_us: Option<u64>,
    on_side_last_us: u64,
    runaway_ok_us: u64,
    /// Since arming, the climb estimate has shown the quad rising while
    /// upright: it left the ground under its own thrust.
    lifted_off: bool,
    lying_since_us: Option<u64>,
    /// Time spent blocked (see `runaway_and_crash`), and when it last was.
    blocked_us: u64,
    blocked_last_us: u64,
    last_impact_us: Option<u64>,
    /// Fastest fall (m/s, from `vz_est`) since the frame last tipped past
    /// 60 degrees.
    tipped_fall: f32,
    motor_fault_since_us: [Option<u64>; 4],
    motor_faults: [bool; 4],
}

/// The complete firmware state. Everything the firmware knows lives here.
#[derive(Clone, Debug, PartialEq)]
pub struct State {
    cfg: Config,
    now_us: u64,
    started: bool,
    cal: GyroCal,

    last_imu_us: Option<u64>,
    last_good_imu_us: Option<u64>,
    /// Last raw sample; a bit-exact repeat means the sensor or bus hung.
    last_raw: ([f32; 3], [f32; 3]),
    /// Last gyro sample that passed spike rejection, and how many samples
    /// in a row have been rejected since.
    last_accepted_gyro: Option<Vec3>,
    spike_run: u8,
    gyro_lpf: Lpf,
    rpm_filter: rpm::RpmFilter,
    /// Notches the strongest vibration tone while the RPM filter is off.
    dyn_notch: dyn_notch::DynNotch,
    /// The last gyro sample (body frame, bias removed), to fill a missed
    /// one with.
    prev_gyro_sample: Vec3,
    /// When each motor's eRPM last arrived; the RPM filter drops a motor
    /// whose telemetry goes stale.
    rpm_at_us: [Option<u64>; 4],
    /// Mean IMU sample period (s), where the notches are placed.
    imu_period: f32,
    dterm_lpf: Lpf,
    accel_lpf: Lpf,
    /// Heavily filtered gyro: body motion without prop vibration.
    gyro_slow: Lpf,
    prev_gyro: Vec3,
    /// Filtered angular acceleration (the D term's input), rad/s².
    ang_accel: Vec3,
    gyro_saturated: bool,
    fast_converge_until_us: u64,
    /// After gyro saturation: until then, a clean 1 g reading far from the
    /// estimate is believed at the fast gain (see `estimate_attitude`).
    recovering_until_us: u64,
    /// Disarmed and still (gyro and accelerometer quiet) since then; drives
    /// the attitude reset on the ground.
    still_since_us: Option<u64>,

    attitude: Quat,
    est_integral: Vec3,

    rc: Rc,
    failsafe: FailsafeStage,
    failsafe_since_us: u64,
    throttle_at_loss: f32,
    /// Throttle that holds altitude, learned while flying, as it would be
    /// with the pack at the reference voltage (see `sag_ratio`).
    hover_throttle: f32,
    /// Level flight that has taught `hover_throttle` since power-up.
    hover_learned_us: u64,
    /// Failsafe landing: throttle taken off (fraction of hover) because the
    /// quad is neither sinking nor speeding up. Large and still not
    /// accelerating means on the ground.
    landing_trim: f32,
    /// Failsafe landing: vertical speed estimate a quarter second ago, and
    /// the vertical acceleration (m/s², down positive) over that quarter.
    landing_vz_ref: (u64, f32),
    landing_accel: f32,
    /// Vertical speed estimate when the trim started growing from zero.
    landing_trim_vz: f32,
    /// Failsafe hold: vertical speed estimate half way through, to measure
    /// how far the learned hover throttle is off.
    hold_vz_mark: Option<f32>,

    armed: bool,
    throttle_out: f32,
    motors: [u16; 4],
    motor_frac: [f32; 4],

    flight: Flight,
    runaway_active: bool,
    /// The build has flown properly since power-up (runaway detection
    /// switched itself off once), so motor order and direction are right.
    flight_proven: bool,
    /// Runaway detection disarmed this power-up's build, which has not
    /// shown it flies: re-arming waits until the frame lies still and the
    /// tilt estimate has snapped to the accelerometer, since a tumble on a
    /// build that cannot fly leaves the estimate wrong.
    runaway_tripped: bool,
    /// Runaway detection has disarmed this power-up's build at least once.
    runaway_seen: bool,
    boot_gesture_since_us: Option<u64>,

    battery_volts: Option<f32>,
    battery_level: BatteryLevel,


    led: bool,
    beeper: bool,
    effects_dropped: u32,
}

impl State {
    pub fn new(cfg: Config) -> Self {
        Self {
            cfg,
            now_us: 0,
            started: false,
            cal: GyroCal::new(0),
            last_imu_us: None,
            last_good_imu_us: None,
            last_raw: ([f32::NAN; 3], [f32::NAN; 3]),
            last_accepted_gyro: None,
            spike_run: 0,
            gyro_lpf: Lpf::default(),
            rpm_filter: rpm::RpmFilter::default(),
            dyn_notch: dyn_notch::DynNotch::default(),
            prev_gyro_sample: Vec3::ZERO,
            rpm_at_us: [None; 4],
            imu_period: 0.0,
            dterm_lpf: Lpf::default(),
            accel_lpf: Lpf::default(),
            gyro_slow: Lpf::default(),
            prev_gyro: Vec3::ZERO,
            ang_accel: Vec3::ZERO,
            gyro_saturated: false,
            fast_converge_until_us: 0,
            recovering_until_us: 0,
            still_since_us: None,
            attitude: Quat::IDENTITY,
            est_integral: Vec3::ZERO,
            rc: Rc {
                last_valid_us: None,
                valid_since_us: 0,
                input: RcInput { roll: 0.0, pitch: 0.0, yaw: 0.0, throttle: 0.0, arm: rc::Switch::High, mode: FlightMode::Angle },
                // Assume the switch is on until we see it off, so a switch
                // left on at power-up can never arm.
                arm_switch: true,
            },
            failsafe: FailsafeStage::None,
            failsafe_since_us: 0,
            throttle_at_loss: 0.0,
            hover_throttle: cfg.hover_throttle_guess,
            hover_learned_us: 0,
            landing_trim: 0.0,
            landing_vz_ref: (0, 0.0),
            landing_accel: f32::MAX,
            landing_trim_vz: 0.0,
            hold_vz_mark: None,
            armed: false,
            throttle_out: 0.0,
            motors: [DSHOT_STOP; 4],
            motor_frac: [0.0; 4],
            flight: Flight::default(),
            runaway_active: false,
            flight_proven: false,
            runaway_tripped: false,
            runaway_seen: false,
            boot_gesture_since_us: None,
            battery_volts: None,
            battery_level: BatteryLevel::Ok,
            led: false,
            beeper: false,
            effects_dropped: 0,
        }
    }

    pub fn config(&self) -> &Config {
        &self.cfg
    }

    pub fn status(&self) -> Status {
        Status {
            armed: self.armed,
            mode: self.effective_mode(),
            failsafe: self.failsafe,
            calibrated: self.cal.done,
            attitude: self.attitude,
            gyro: self.gyro_lpf.y,
            rate_setpoint: self.flight.setpoint,
            throttle: self.throttle_out,
            throttle_demand: if self.armed { self.flight.throttle_demand } else { 0.0 },
            battery_volts: self.battery_volts,
            battery_level: self.battery_level,
            motors: self.motors,
            motor_faults: self.flight.motor_faults,
            airmode_active: self.flight.airmode_active,
            blockers: self.arm_blockers(),
            climb_rate: -self.flight.vz_est,
            hover_throttle: self.hover_throttle,
            now_us: self.now_us,
            effects_dropped: self.effects_dropped,
        }
    }

    fn effective_mode(&self) -> FlightMode {
        if self.failsafe != FailsafeStage::None { FlightMode::Angle } else { self.rc.input.mode }
    }

    fn rc_lost(&self) -> bool {
        match self.rc.last_valid_us {
            None => true,
            Some(t) => self.now_us.saturating_sub(t) > self.cfg.rc_timeout_us,
        }
    }

    fn imu_stale(&self) -> bool {
        match self.last_good_imu_us {
            None => true,
            Some(t) => self.now_us.saturating_sub(t) > self.cfg.imu_timeout_us,
        }
    }

    fn arm_blockers(&self) -> ArmBlockers {
        let mut b = 0;
        if !self.cal.done {
            b |= ArmBlockers::CALIBRATING;
        }
        if self.rc.input.throttle > THROTTLE_LOW {
            b |= ArmBlockers::THROTTLE;
        }
        if self.rc_lost() {
            b |= ArmBlockers::RC_LINK;
        }
        if self.imu_stale() {
            b |= ArmBlockers::IMU;
        }
        let settled = self.still_since_us.is_some_and(|t| self.now_us.saturating_sub(t) >= imu::GROUND_SETTLED_US);
        if self.attitude.tilt() > self.cfg.max_arm_tilt || (self.runaway_tripped && !settled) {
            b |= ArmBlockers::TILT;
        }
        if self.battery_volts.is_none_or(|v| v < self.cfg.vbat_min_arm) {
            b |= ArmBlockers::BATTERY;
        }
        ArmBlockers(b)
    }
}

impl Default for State {
    fn default() -> Self {
        State::new(Config::default())
    }
}

/// The firmware: advance `s` by one event. It takes the state by reference
/// rather than by value: moving the 1.6 KB `State` in and out costs more on
/// the Cortex-M4 than most events' work.
pub fn step(s: &mut State, event: &Event) -> Effects {
    let mut fx = Out::default();
    if !s.started {
        s.started = true;
        s.now_us = event.t_us;
        s.cal.start_us = event.t_us;
    } else if event.t_us < s.now_us {
        // Time never runs backwards: keep the newest time and process the
        // event as happening now. The board stamps IMU samples at data-ready
        // and queues them after the SPI read, so events from other tasks can
        // overtake them by a fraction of a millisecond; only a larger step
        // back is a glitching timer worth reporting.
        if s.now_us - event.t_us > 1_000 {
            fx.push(Effect::Notice(Notice::ClockWentBackwards));
        }
    } else {
        s.now_us = event.t_us;
    }

    let motors_before = s.motors;
    match event.kind {
        EventKind::Imu { gyro, accel } => imu::on_imu(s, gyro, accel, &mut fx),
        EventKind::Rc { channels } => on_rc(s, &channels, &mut fx),
        EventKind::Battery { volts, amps: _ } => health::on_battery(s, volts, &mut fx),
        EventKind::MotorTelemetry { erpm } => {
            imu::on_erpm(s, &erpm);
            health::check_motors(s, &erpm, &mut fx);
        }
        EventKind::Tick => {}
        EventKind::Command(Command::RebootToBootloader) => {
            if !s.armed {
                fx.push(Effect::RebootToBootloader);
            }
        }
    }
    housekeeping(s, &mut fx);
    // `s.motors` is the one motor command; send it on every IMU sample,
    // whenever it changed (arming, disarming), and on ticks while IMU
    // samples are late: ESCs need a steady stream, so the last command
    // repeats until the IMU timeout disarms.
    let imu_late = s.last_imu_us.is_none_or(|t| s.now_us.saturating_sub(t) >= 1_000);
    let send = match event.kind {
        EventKind::Imu { .. } => true,
        EventKind::Tick => s.armed && imu_late,
        _ => false,
    };
    if send || s.motors != motors_before {
        fx.send_motors(s.motors);
    }
    s.effects_dropped = s.effects_dropped.saturating_add(fx.dropped);
    fx.list
}

impl Out {
    /// Sends the motor command first. It is never the effect dropped: if
    /// the step's output is full, the last notice makes room.
    fn send_motors(&mut self, m: [u16; 4]) {
        if self.list.is_full() {
            self.list.pop();
            self.dropped += 1;
        }
        let _ = self.list.insert(0, Effect::Motors(m));
    }

    fn push(&mut self, e: Effect) {
        // Capacity is sized for the worst case; dropping a notice is
        // preferable to panicking in flight if that ever proves wrong, but
        // it is counted (`Status::effects_dropped`) so tests can tell.
        if self.list.push(e).is_err() {
            self.dropped += 1;
        }
    }
}

/// How long `on` has held without a break (µs), timing it in `since`:
/// the debounce behind every "for this long" check.
fn held_for(since: &mut Option<u64>, on: bool, now: u64) -> u64 {
    if on {
        now - *since.get_or_insert(now)
    } else {
        *since = None;
        0
    }
}

fn disarm(s: &mut State, reason: DisarmReason, fx: &mut Out) {
    if !s.armed {
        return;
    }
    s.armed = false;
    s.flight.airmode_active = false;
    s.flight.integral = Vec3::ZERO;
    s.failsafe = FailsafeStage::None;
    s.motors = [DSHOT_STOP; 4];
    s.motor_frac = [0.0; 4];
    s.throttle_out = 0.0;
    fx.push(Effect::Notice(Notice::Disarmed(reason)));
}

fn on_rc(s: &mut State, channels: &[u16; RC_CHANNELS], fx: &mut Out) {
    // A corrupted frame is dropped rather than acted on.
    let Some(input) = RcInput::decode(channels) else {
        fx.push(Effect::Notice(Notice::RcFrameRejected));
        return;
    };
    let now = s.now_us;
    if s.rc_lost() {
        s.rc.valid_since_us = now;
    }
    s.rc.last_valid_us = Some(now);
    if input.mode != s.rc.input.mode {
        fx.push(Effect::Notice(Notice::ModeChanged(input.mode)));
    }
    s.rc.input = input;

    if held_for(&mut s.boot_gesture_since_us, input.boot_gesture() && !s.armed, now) >= 2_000_000 {
        s.boot_gesture_since_us = None;
        fx.push(Effect::RebootToBootloader);
    }

    let was_on = s.rc.arm_switch;
    let on = input.arm.arm_on(was_on);
    s.rc.arm_switch = on;
    if !on && s.armed {
        disarm(s, DisarmReason::Switch, fx);
    } else if on && !was_on && !s.armed {
        let blockers = s.arm_blockers();
        if blockers.is_empty() {
            s.armed = true;
            // The vertical speed is the quad's, not the flight's: it may
            // be re-armed falling.
            s.flight = Flight { vz_est: s.flight.vz_est, ..Flight::default() };
            // Runaway detection catches a build that cannot fly. Once this
            // power-up has shown it flies, a re-arm (even mid-air, to save a
            // fall) must not mistake the fight back from a crash for one;
            // crash detection handles that.
            s.runaway_active = !s.flight_proven;
            s.runaway_tripped = false;
            // Spin up to idle now. Until the next IMU sample runs the
            // controller, the keep-alive tick repeats this command; left at
            // the disarmed zeros it would hold the motors stopped while armed.
            s.motors = [control::dshot(s.cfg.motor_idle); 4];
            fx.push(Effect::Notice(Notice::Armed(s.rc.input.mode)));
        } else {
            fx.push(Effect::Notice(Notice::ArmingBlocked(blockers)));
        }
    }
}

/// Timeouts, failsafe stages and annunciators, run after every event.
fn housekeeping(s: &mut State, fx: &mut Out) {
    let now = s.now_us;

    if s.armed && s.imu_stale() {
        disarm(s, DisarmReason::ImuFailure, fx);
    }

    failsafe::advance(s, fx);

    // LED: solid when armed, slow blink when ready, fast blink otherwise.
    let led = if s.armed {
        true
    } else {
        let period = if s.arm_blockers().is_empty() { 1_000_000 } else { 200_000 };
        now % period < period / 2
    };
    if led != s.led {
        s.led = led;
        fx.push(Effect::Led(led));
    }
    let beeper = if s.failsafe != FailsafeStage::None {
        now % 400_000 < 200_000
    } else {
        match s.battery_level {
            BatteryLevel::Ok => false,
            BatteryLevel::Warning => now % 2_000_000 < 100_000,
            BatteryLevel::Critical => now % 500_000 < 150_000,
        }
    };
    if beeper != s.beeper {
        s.beeper = beeper;
        fx.push(Effect::Beeper(beeper));
    }
}

#[cfg(test)]
mod tests;
