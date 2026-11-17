//! IronLoop flight controller core.
//!
//! The whole firmware is one pure function, [`step`]: it takes the current
//! [`State`] and one timestamped [`Event`] (an IMU sample, an RC frame, a
//! battery reading, ESC telemetry, a housekeeping tick or a command) and
//! returns the next state plus the [`Effect`]s the board must perform (motor
//! outputs, LED, beeper, notices, bootloader jump). It never reads a clock,
//! touches hardware, allocates, or keeps hidden state, so the same code runs
//! on the STM32F411 under embassy, in the deterministic simulator, and in the
//! browser.

#![no_std]

pub mod config;
pub mod math;
pub mod proto;

pub use config::{AxisPid, Config, ImuAlignment, Rates};
use heapless::Vec;
use libm::roundf;
use math::{Quat, Vec3, tilt_error, up_in_body};

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

/// Channel order (AETR, as sent by ExpressLRS over CRSF).
pub mod ch {
    pub const ROLL: usize = 0;
    pub const PITCH: usize = 1;
    pub const THROTTLE: usize = 2;
    pub const YAW: usize = 3;
    /// High (>1700 µs) arms.
    pub const ARM: usize = 4;
    /// Low or centered selects Angle, high (>1700 µs) selects Acro.
    pub const MODE: usize = 5;
}

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
    /// Throttle believed to hold altitude (learned in flight).
    pub hover_throttle: f32,
    /// The firmware's clock (µs, the newest event time seen).
    pub now_us: u64,
    /// Effects that did not fit in a step's output since boot.
    pub effects_dropped: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Pt1 {
    y: f32,
}

impl Pt1 {
    fn update(&mut self, x: f32, cutoff_hz: f32, dt: f32) -> f32 {
        let rc = 1.0 / (2.0 * core::f32::consts::PI * cutoff_hz);
        let k = dt / (rc + dt);
        self.y += k * (x - self.y);
        self.y
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
struct Pt1v {
    x: Pt1,
    y: Pt1,
    z: Pt1,
}

impl Default for Pt1 {
    fn default() -> Self {
        Pt1 { y: 0.0 }
    }
}

impl Pt1v {
    fn update(&mut self, v: Vec3, hz: f32, dt: f32) -> Vec3 {
        Vec3::new(self.x.update(v.x, hz, dt), self.y.update(v.y, hz, dt), self.z.update(v.z, hz, dt))
    }
    fn reset(&mut self, v: Vec3) {
        self.x.y = v.x;
        self.y.y = v.y;
        self.z.y = v.z;
    }
    fn get(&self) -> Vec3 {
        Vec3::new(self.x.y, self.y.y, self.z.y)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Rc {
    last_valid_us: Option<u64>,
    /// Start of the current run of valid frames (no gap above the timeout).
    valid_since_us: u64,
    roll: f32,
    pitch: f32,
    yaw: f32,
    throttle: f32,
    arm_switch: bool,
    mode: FlightMode,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct GyroCal {
    done: bool,
    start_us: u64,
    sum: Vec3,
    min: Vec3,
    max: Vec3,
    count: u32,
    bias: Vec3,
    /// Sum of accelerometer magnitudes, and their mean once done: what
    /// this sensor reads for 1 g at rest.
    accel_sum: f32,
    accel_1g: f32,
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
    imu_failed: bool,
    gyro_lpf: Pt1v,
    dterm_lpf: Pt1v,
    /// Low-passed setpoint; the setpoint minus this is how fast the pilot
    /// is moving the sticks (I-term relax).
    setpoint_lpf: Pt1v,
    accel_lpf: Pt1v,
    /// Heavily filtered gyro: body motion without prop vibration.
    gyro_slow: Pt1v,
    prev_gyro: Vec3,
    gyro_saturated: bool,
    fast_converge_until_us: u64,

    attitude: Quat,
    est_integral: Vec3,

    rc: Rc,
    failsafe: FailsafeStage,
    failsafe_since_us: u64,
    throttle_at_loss: f32,
    /// Throttle that holds altitude, learned while flying.
    hover_throttle: f32,
    /// Leaky integral of vertical acceleration (m/s, down positive). It
    /// forgets steady motion over a few seconds, but shows climbs and
    /// descents as they start and end.
    vz_est: f32,
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
    airmode_active: bool,
    integral: Vec3,
    mixer_saturated: bool,
    setpoint: Vec3,
    last_pid: Vec3,
    throttle_out: f32,
    throttle_demand: f32,
    motors: [u16; 4],
    motor_frac: [f32; 4],

    runaway_active: bool,
    /// The build has flown properly since power-up (runaway detection
    /// switched itself off once), so motor order and direction are right.
    flight_proven: bool,
    runaway_since_us: Option<u64>,
    runaway_ok_us: u64,
    lying_since_us: Option<u64>,
    blocked_since_us: Option<u64>,
    last_impact_us: Option<u64>,
    boot_gesture_since_us: Option<u64>,

    battery_volts: Option<f32>,
    battery_level: BatteryLevel,

    motor_fault_since_us: [Option<u64>; 4],
    motor_faults: [bool; 4],

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
            cal: GyroCal {
                done: false,
                start_us: 0,
                sum: Vec3::ZERO,
                min: Vec3::new(f32::MAX, f32::MAX, f32::MAX),
                max: Vec3::new(f32::MIN, f32::MIN, f32::MIN),
                count: 0,
                bias: Vec3::ZERO,
                accel_sum: 0.0,
                accel_1g: GRAVITY,
            },
            last_imu_us: None,
            last_good_imu_us: None,
            last_raw: ([f32::NAN; 3], [f32::NAN; 3]),
            last_accepted_gyro: None,
            spike_run: 0,
            imu_failed: false,
            gyro_lpf: Pt1v::default(),
            dterm_lpf: Pt1v::default(),
            setpoint_lpf: Pt1v::default(),
            accel_lpf: Pt1v::default(),
            gyro_slow: Pt1v::default(),
            prev_gyro: Vec3::ZERO,
            gyro_saturated: false,
            fast_converge_until_us: 0,
            attitude: Quat::IDENTITY,
            est_integral: Vec3::ZERO,
            rc: Rc {
                last_valid_us: None,
                valid_since_us: 0,
                roll: 0.0,
                pitch: 0.0,
                yaw: 0.0,
                throttle: 0.0,
                // Assume the switch is on until we see it off, so a switch
                // left on at power-up can never arm.
                arm_switch: true,
                mode: FlightMode::Angle,
            },
            failsafe: FailsafeStage::None,
            failsafe_since_us: 0,
            throttle_at_loss: 0.0,
            hover_throttle: cfg.hover_throttle_guess,
            vz_est: 0.0,
            landing_trim: 0.0,
            landing_vz_ref: (0, 0.0),
            landing_accel: f32::MAX,
            landing_trim_vz: 0.0,
            hold_vz_mark: None,
            armed: false,
            airmode_active: false,
            integral: Vec3::ZERO,
            mixer_saturated: false,
            setpoint: Vec3::ZERO,
            last_pid: Vec3::ZERO,
            throttle_out: 0.0,
            throttle_demand: 0.0,
            motors: [DSHOT_STOP; 4],
            motor_frac: [0.0; 4],
            runaway_active: false,
            flight_proven: false,
            runaway_since_us: None,
            runaway_ok_us: 0,
            lying_since_us: None,
            blocked_since_us: None,
            last_impact_us: None,
            boot_gesture_since_us: None,
            battery_volts: None,
            battery_level: BatteryLevel::Ok,
            motor_fault_since_us: [None; 4],
            motor_faults: [false; 4],
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
            gyro: self.gyro_lpf.get(),
            rate_setpoint: self.setpoint,
            throttle: self.throttle_out,
            throttle_demand: if self.armed { self.throttle_demand } else { 0.0 },
            battery_volts: self.battery_volts,
            battery_level: self.battery_level,
            motors: self.motors,
            motor_faults: self.motor_faults,
            airmode_active: self.airmode_active,
            blockers: self.arm_blockers(),
            climb_rate: -self.vz_est,
            hover_throttle: self.hover_throttle,
            now_us: self.now_us,
            effects_dropped: self.effects_dropped,
        }
    }

    fn effective_mode(&self) -> FlightMode {
        if self.failsafe != FailsafeStage::None { FlightMode::Angle } else { self.rc.mode }
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
        if self.rc.throttle > 0.05 {
            b |= ArmBlockers::THROTTLE;
        }
        if self.rc_lost() {
            b |= ArmBlockers::RC_LINK;
        }
        if self.imu_failed || self.imu_stale() {
            b |= ArmBlockers::IMU;
        }
        if self.attitude.tilt() > self.cfg.max_arm_tilt {
            b |= ArmBlockers::TILT;
        }
        let min = self.cfg.vbat_min_arm_cell * self.cfg.battery_cells as f32;
        if self.battery_volts.is_none_or(|v| v < min) {
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

/// The firmware: advance `state` by one event.
pub fn step(mut s: State, event: &Event) -> (State, Effects) {
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
            push(&mut fx, Effect::Notice(Notice::ClockWentBackwards));
        }
    } else {
        s.now_us = event.t_us;
    }

    match event.kind {
        EventKind::Imu { gyro, accel } => on_imu(&mut s, gyro, accel, &mut fx),
        EventKind::Rc { channels } => on_rc(&mut s, &channels, &mut fx),
        EventKind::Battery { volts, amps: _ } => on_battery(&mut s, volts, &mut fx),
        EventKind::MotorTelemetry { erpm } => on_telemetry(&mut s, &erpm, &mut fx),
        EventKind::Tick => {
            // ESCs need a steady command stream; if IMU samples (which
            // normally drive the motors) are late, repeat the last command
            // until the IMU timeout decides.
            if s.armed && s.last_imu_us.is_none_or(|t| s.now_us.saturating_sub(t) >= 1_000) {
                push(&mut fx, Effect::Motors(s.motors));
            }
        }
        EventKind::Command(Command::RebootToBootloader) => {
            if !s.armed {
                push(&mut fx, Effect::RebootToBootloader);
            }
        }
    }
    housekeeping(&mut s, &mut fx);
    s.effects_dropped = s.effects_dropped.saturating_add(fx.dropped);
    (s, fx.list)
}

fn push(fx: &mut Out, e: Effect) {
    // Capacity is sized for the worst case; dropping a notice is preferable
    // to panicking in flight if that ever proves wrong, but it is counted
    // (`Status::effects_dropped`) so tests can tell.
    if fx.list.push(e).is_err() {
        fx.dropped += 1;
    }
}

fn disarm(s: &mut State, reason: DisarmReason, fx: &mut Out) {
    if !s.armed {
        return;
    }
    s.armed = false;
    s.airmode_active = false;
    s.integral = Vec3::ZERO;
    s.failsafe = FailsafeStage::None;
    s.motors = [DSHOT_STOP; 4];
    s.motor_frac = [0.0; 4];
    s.throttle_out = 0.0;
    push(fx, Effect::Motors(s.motors));
    push(fx, Effect::Notice(Notice::Disarmed(reason)));
}

fn align(a: ImuAlignment, v: Vec3) -> Vec3 {
    // Sensor axes rotated clockwise (seen from above, about body z down).
    match a {
        ImuAlignment::Cw0 => v,
        ImuAlignment::Cw90 => Vec3::new(-v.y, v.x, v.z),
        ImuAlignment::Cw180 => Vec3::new(-v.x, -v.y, v.z),
        ImuAlignment::Cw270 => Vec3::new(v.y, -v.x, v.z),
    }
}

fn on_imu(s: &mut State, gyro: [f32; 3], accel: [f32; 3], fx: &mut Out) {
    let now = s.now_us;
    let g_raw = Vec3::from_array(gyro);
    let a_raw = Vec3::from_array(accel);
    let in_range = g_raw.is_finite()
        && a_raw.is_finite()
        && g_raw.max_abs() <= s.cfg.gyro_range * 1.5
        && a_raw.max_abs() <= s.cfg.accel_range * 1.5;
    // A quad cannot change its rate by more than about 1 rad/s between
    // samples (torque / inertia), plus a few rad/s of prop vibration. A
    // bigger jump is an SPI glitch or EMI spike. Rejecting it keeps it out of
    // the attitude estimate; after a few rejects in a row the sensor is
    // believed again, so a real step cannot lock it out.
    let spike = in_range
        && s.spike_run < 4
        && s.last_accepted_gyro.is_some_and(|p| (g_raw - p).max_abs() > s.cfg.gyro_spike);
    if spike {
        s.spike_run += 1;
    } else if in_range {
        s.spike_run = 0;
        s.last_accepted_gyro = Some(g_raw);
    }
    let valid = in_range && !spike;
    let dt_us = s.last_imu_us.map_or(0, |t| now.saturating_sub(t));
    s.last_imu_us = Some(now);
    if !valid {
        push(fx, Effect::Notice(Notice::ImuSampleRejected));
        if s.armed {
            // Keep the ESCs fed with the last command until the IMU timeout.
            push(fx, Effect::Motors(s.motors));
        }
        return;
    }
    let gap = s.last_good_imu_us.is_none_or(|t| now.saturating_sub(t) > s.cfg.imu_timeout_us);
    // Sensor noise makes consecutive real samples differ; an exact repeat is
    // a hung sensor or bus. Keep flying on it, but it does not count as
    // fresh data, so a hang longer than the IMU timeout stops the motors.
    if (gyro, accel) != s.last_raw {
        s.last_good_imu_us = Some(now);
    }
    s.last_raw = (gyro, accel);
    if s.imu_failed && !s.armed {
        // Data is back; the estimator restarts from the accelerometer.
        s.imu_failed = false;
    }

    let saturated = g_raw.max_abs() >= s.cfg.gyro_range * 0.97;
    if saturated && !s.gyro_saturated {
        push(fx, Effect::Notice(Notice::GyroSaturated));
    }
    if s.gyro_saturated && !saturated {
        // The attitude estimate integrated clipped rates; lean on the
        // accelerometer until it has re-converged.
        s.fast_converge_until_us = now + 1_500_000;
    }
    s.gyro_saturated = saturated;

    let g = align(s.cfg.imu_alignment, g_raw);
    let a = align(s.cfg.imu_alignment, a_raw);
    // Nominal sample spacing is 125..500 µs; clamp so one late or early
    // sample cannot blow up the integrators or the derivative.
    let reset_dynamics = gap || dt_us == 0 || dt_us > 5_000;
    let dt = (dt_us as f32 * 1e-6).clamp(50e-6, 2e-3);

    if !s.cal.done {
        calibrate(s, g, a, fx);
    }
    let g = g - s.cal.bias;
    let gyro_f = if reset_dynamics {
        s.gyro_lpf.reset(g);
        g
    } else {
        s.gyro_lpf.update(g, s.cfg.gyro_lpf_hz, dt)
    };
    let accel_f = if reset_dynamics {
        s.accel_lpf.reset(a);
        s.gyro_slow.reset(g);
        a
    } else {
        s.gyro_slow.update(g, 10.0, dt);
        s.accel_lpf.update(a, 30.0, dt)
    };

    estimate_attitude(s, g, accel_f, dt, reset_dynamics);
    if s.armed {
        control(s, gyro_f, accel_f, dt, reset_dynamics, fx);
    } else {
        s.prev_gyro = gyro_f;
        s.dterm_lpf.reset(Vec3::ZERO);
        push(fx, Effect::Motors([DSHOT_STOP; 4]));
    }
}

fn calibrate(s: &mut State, g: Vec3, a: Vec3, fx: &mut Out) {
    let c = &mut s.cal;
    c.sum += g;
    c.accel_sum += a.norm();
    c.count += 1;
    c.min = Vec3::new(c.min.x.min(g.x), c.min.y.min(g.y), c.min.z.min(g.z));
    c.max = Vec3::new(c.max.x.max(g.x), c.max.y.max(g.y), c.max.z.max(g.z));
    let spread = (c.max - c.min).max_abs();
    if spread > s.cfg.gyro_cal_motion * 2.0 {
        // Someone is moving the quad: start over.
        *c = GyroCal { start_us: s.now_us, ..State::new(s.cfg).cal };
        return;
    }
    if s.now_us.saturating_sub(c.start_us) >= s.cfg.gyro_cal_us && c.count >= 100 {
        c.bias = c.sum * (1.0 / c.count as f32);
        c.accel_1g = c.accel_sum / c.count as f32;
        c.done = true;
        s.fast_converge_until_us = s.now_us + 1_000_000;
        push(fx, Effect::Notice(Notice::GyroCalibrated));
    }
}

/// Mahony complementary filter.
fn estimate_attitude(s: &mut State, gyro: Vec3, accel: Vec3, dt: f32, reset: bool) {
    let a_norm = accel.norm();
    let up_meas = accel * (1.0 / a_norm.max(1e-6));
    if reset && s.imu_failed {
        s.attitude = Quat::IDENTITY;
    }
    // Specific force is only a gravity reference when the quad is not
    // accelerating; trust it less the further it is from 1 g.
    let dev = (a_norm / GRAVITY - 1.0).abs();
    let fast = s.now_us < s.fast_converge_until_us || !s.cal.done;
    // After calibration or gyro saturation the estimate may be far off, so
    // any roughly-1 g reading beats the gyro integral.
    let band = if fast { 0.5 } else { 0.1 };
    let weight = (1.0 - dev / band).clamp(0.0, 1.0);
    let kp = if fast {
        6.0
    } else if s.armed {
        0.5
    } else {
        2.0
    };
    let mut omega = gyro;
    if weight > 0.0 && !s.gyro_saturated {
        let up_est = s.attitude.inverse_rotate(Vec3::new(0.0, 0.0, -1.0));
        let err = up_meas.cross(up_est);
        if s.cal.done && !fast {
            s.est_integral = s.est_integral + err * (0.3 * weight * dt);
            // Residual gyro bias from warm-up drift after calibration.
            let lim = 0.1;
            s.est_integral = Vec3::new(
                s.est_integral.x.clamp(-lim, lim),
                s.est_integral.y.clamp(-lim, lim),
                s.est_integral.z.clamp(-lim, lim),
            );
        }
        omega = omega + err * (kp * weight) + s.est_integral;
    }
    s.attitude = s.attitude.integrate(omega, dt);
    if !s.attitude.is_finite() {
        s.attitude = Quat::IDENTITY;
    }
}

/// Motor mix (roll, pitch, yaw per motor) for QuadX in Betaflight motor
/// order, props out: M1 rear-right, M2 front-right, M3 rear-left, M4
/// front-left. FRD axes: positive roll is right side down, positive pitch
/// nose up, positive yaw nose right.
const MIX: [[f32; 3]; 4] = [
    [-1.0, -1.0, 1.0],
    [-1.0, 1.0, -1.0],
    [1.0, -1.0, -1.0],
    [1.0, 1.0, 1.0],
];

fn rate_curve(r: &Rates, stick: f32) -> f32 {
    let x = stick.clamp(-1.0, 1.0);
    let ax = x.abs();
    let x = x * ax * ax * ax * r.expo + x * (1.0 - r.expo);
    let super_factor = 1.0 / (1.0 - ax * r.super_rate).clamp(0.01, 1.0);
    let dps = (200.0 * r.rc_rate * x * super_factor).clamp(-1998.0, 1998.0);
    dps.to_radians()
}

fn control(s: &mut State, gyro: Vec3, accel: Vec3, dt: f32, reset: bool, fx: &mut Out) {
    let cfg = s.cfg;
    // Vertical motion from the accelerometer, against what it read for 1 g
    // on the ground. In normal flight the integral leaks, forgetting steady
    // motion over a few seconds (without a barometer a steady climb looks
    // like hover) but showing climbs and descents as they start and end.
    // It reads g * (1 - cos e) too low for an attitude estimate off by e,
    // which in hard maneuvering adds up to a m/s or two. So failsafe starts
    // it from zero when the link is lost and integrates without leaking.
    let down_accel = s.attitude.rotate(accel).z + s.cal.accel_1g;
    let leak = if s.failsafe == FailsafeStage::None { s.vz_est / 4.0 } else { 0.0 };
    s.vz_est += (down_accel - leak) * dt;
    if s.failsafe == FailsafeStage::Hold && s.hold_vz_mark.is_none() && s.now_us.saturating_sub(s.failsafe_since_us) >= 500_000 {
        s.hold_vz_mark = Some(s.vz_est);
    }
    let (roll, pitch, yaw, throttle) = match s.failsafe {
        FailsafeStage::None => (s.rc.roll, s.rc.pitch, s.rc.yaw, s.rc.throttle),
        // Hold altitude at the learned hover throttle, then sink at about
        // 1-2 m/s at 96% of it (thrust grows with throttle squared, so about
        // 92% of hover thrust). The hold also measures how far the learned
        // hover throttle is off (see housekeeping). During the landing a
        // fast sink brakes; the integrated accelerometer drifts (vibration),
        // so the brake's ceiling falls from 115% to 96% of hover over the
        // first 6 s and a drifting estimate cannot hold the quad up.
        // Throttle at the bottom when the link went means the quad is on the
        // ground (or the pilot cut it): never spin up from there.
        FailsafeStage::Hold if s.throttle_at_loss < 0.05 => (0.0, 0.0, 0.0, 0.0),
        FailsafeStage::Hold => (0.0, 0.0, 0.0, s.hover_throttle),
        FailsafeStage::Landing => {
            let landing_s = s.now_us.saturating_sub(s.failsafe_since_us) as f32 * 1e-6;
            let ceiling = (1.15 - 0.04 * landing_s).max(0.98);
            if s.now_us.saturating_sub(s.landing_vz_ref.0) >= 250_000 {
                s.landing_accel = (s.vz_est - s.landing_vz_ref.1) / 0.25;
                s.landing_vz_ref = (s.now_us, s.vz_est);
            }
            // Not sinking, or not speeding up although throttle is being
            // taken off: take more off, steadily. In the air that soon
            // starts a descent (a learned hover throttle far too high cannot
            // keep it climbing) and then speeds it up, which stops the trim;
            // on the ground nothing changes, and once thrust is well below
            // hover the housekeeping calls it landed. The speed estimate
            // itself drifts by a m/s or more over a long landing, so these
            // tests use only how it changes.
            if s.landing_trim == 0.0 {
                s.landing_trim_vz = s.vz_est;
            }
            let unmoved = s.landing_accel.abs() < 0.6 && s.vz_est - s.landing_trim_vz < 0.5;
            s.landing_trim = if s.vz_est < 0.25 || unmoved {
                (s.landing_trim + 0.25 * dt).min(0.35)
            } else {
                (s.landing_trim - 2.0 * dt).max(0.0)
            };
            let brake = 0.15 * (s.vz_est - 1.2).max(0.0);
            let k = (0.96 + brake).min(ceiling) - s.landing_trim;
            (0.0, 0.0, 0.0, s.hover_throttle * k)
        }
    };
    s.throttle_demand = throttle;
    if throttle > cfg.airmode_throttle {
        s.airmode_active = true;
    }
    // Over a second or two of level flight the pilot's average throttle is
    // the one that holds altitude. Thrust grows with throttle squared and
    // only its vertical part lifts, so scale by sqrt(cos(tilt)). A climbing
    // or sinking quad would teach the wrong value (and the failsafe would
    // then climb instead of land), so learning pauses while moving up or
    // down.
    let tilt = s.attitude.tilt();
    let level_flight = s.vz_est.abs() < 0.5;
    if s.failsafe == FailsafeStage::None && s.airmode_active && level_flight && tilt < 0.5 && (0.1..0.95).contains(&throttle) {
        let level = throttle * libm::sqrtf(libm::cosf(tilt));
        s.hover_throttle += (level - s.hover_throttle) * (dt / 1.5);
    }

    let yaw_rate = rate_curve(&cfg.rates[2], yaw);
    let setpoint = match s.effective_mode() {
        FlightMode::Acro => Vec3::new(
            rate_curve(&cfg.rates[0], roll),
            // Stick forward pitches the nose down.
            -rate_curve(&cfg.rates[1], pitch),
            yaw_rate,
        ),
        FlightMode::Angle => {
            let target = up_in_body(roll * cfg.angle_limit, -pitch * cfg.angle_limit);
            let up = s.attitude.inverse_rotate(Vec3::new(0.0, 0.0, -1.0));
            let e = tilt_error(up, target) * cfg.angle_p;
            let lim = cfg.angle_rate_limit;
            Vec3::new(e.x.clamp(-lim, lim), e.y.clamp(-lim, lim), yaw_rate)
        }
    };

    if reset {
        s.prev_gyro = gyro;
        s.dterm_lpf.reset(Vec3::ZERO);
    }
    let d_gyro = (gyro - s.prev_gyro) * (1.0 / dt);
    s.prev_gyro = gyro;
    let d_gyro = s.dterm_lpf.update(d_gyro, cfg.dterm_lpf_hz, dt);

    let err = setpoint - gyro;
    // No I-term at zero throttle: resting on the ground (or a wall) the
    // frame cannot move, the error never closes, and an integrator would
    // wind up and push the motors through airmode.
    let integrate = s.airmode_active && throttle > 0.05 && !s.mixer_saturated && !s.gyro_saturated && !reset;
    if !s.airmode_active {
        // On the ground before takeoff: no I-term windup.
        s.integral = Vec3::ZERO;
    }
    // I-term relax (as in Betaflight): while the roll or pitch setpoint is
    // moving fast the error is the frame catching up, not a steady offset;
    // integrating it overshoots the stick by half again and then unwinds
    // for a couple of hundred ms.
    let sp_moving = setpoint - s.setpoint_lpf.update(setpoint, 15.0, dt);
    let relax = |x: f32| (1.0 - x.abs() / 40f32.to_radians()).clamp(0.0, 1.0);
    let relax = [relax(sp_moving.x), relax(sp_moving.y), 1.0];
    let e = [err.x, err.y, err.z];
    let d = [d_gyro.x, d_gyro.y, d_gyro.z];
    let sp = [setpoint.x, setpoint.y, setpoint.z];
    let mut integral = [s.integral.x, s.integral.y, s.integral.z];
    let mut out = [0.0f32; 3];
    for i in 0..3 {
        let k = cfg.pid[i];
        if integrate {
            integral[i] = (integral[i] + k.i * e[i] * relax[i] * dt).clamp(-cfg.iterm_limit, cfg.iterm_limit);
        }
        let lim = if i == 2 { cfg.pid_sum_limit * 0.7 } else { cfg.pid_sum_limit };
        out[i] = (k.p * e[i] + integral[i] - k.d * d[i] + k.ff * sp[i]).clamp(-lim, lim);
    }
    s.integral = Vec3::from_array(integral);
    s.setpoint = setpoint;
    s.last_pid = Vec3::from_array(out);

    let mut mix = [0.0f32; 4];
    for (m, row) in mix.iter_mut().zip(MIX) {
        *m = row[0] * out[0] + row[1] * out[1] + row[2] * out[2];
    }
    let lo = mix.iter().copied().fold(f32::MAX, f32::min);
    let hi = mix.iter().copied().fold(f32::MIN, f32::max);
    let span = hi - lo;
    s.mixer_saturated = span > 1.0;
    if span > 1.0 {
        for m in &mut mix {
            *m /= span;
        }
    }
    let (lo, hi) = (lo / span.max(1.0), hi / span.max(1.0));
    let mut thr = throttle.clamp(0.0, 1.0);
    let comp = match (cfg.vbat_sag_comp, s.battery_volts) {
        (true, Some(v)) if v > 0.5 => {
            (cfg.vbat_ref_cell * cfg.battery_cells as f32 / v).clamp(1.0, 1.25)
        }
        _ => 1.0,
    };
    thr = (thr * comp).min(1.0);
    if s.airmode_active {
        // Airmode: shift throttle so every motor stays in range and the
        // attitude correction is never clipped.
        thr = thr.clamp(-lo, (1.0 - hi).max(-lo));
    }
    s.throttle_out = thr;
    for i in 0..4 {
        let m = (thr + mix[i]).clamp(0.0, 1.0);
        let f = cfg.motor_idle + (1.0 - cfg.motor_idle) * m;
        s.motor_frac[i] = f;
        let v = DSHOT_MIN as f32 + roundf(f * (DSHOT_MAX - DSHOT_MIN) as f32);
        s.motors[i] = (v as u16).clamp(DSHOT_MIN, DSHOT_MAX);
    }

    runaway_and_crash(s, gyro, accel, throttle, dt, fx);
    if s.armed {
        push(fx, Effect::Motors(s.motors));
    }
}

fn runaway_and_crash(s: &mut State, gyro: Vec3, accel: Vec3, throttle: f32, dt: f32, fx: &mut Out) {
    let cfg = s.cfg;
    let now = s.now_us;
    if s.runaway_active {
        let lim = cfg.pid_sum_limit * cfg.runaway_pid_frac;
        let pid_high = s.last_pid.x.abs() >= lim || s.last_pid.y.abs() >= lim;
        let spinning = gyro.norm() > cfg.runaway_gyro;
        if pid_high && spinning {
            let since = *s.runaway_since_us.get_or_insert(now);
            if now - since >= cfg.runaway_us {
                disarm(s, DisarmReason::RunawayTakeoff, fx);
                return;
            }
        } else {
            s.runaway_since_us = None;
            // Calm, level flight shows the build works, and so does a
            // pilot flying it with the sticks (as Betaflight counts it).
            // Throttle alone does not: a quad that cannot fly may sit on
            // the ground at part throttle until a gust tips it.
            let calm = !spinning && s.attitude.tilt() < 30f32.to_radians();
            let flown = s.rc.roll.abs().max(s.rc.pitch.abs()).max(s.rc.yaw.abs()) > 0.15;
            if throttle > cfg.airmode_throttle && (calm || flown) {
                s.runaway_ok_us += (dt * 1e6) as u64;
                if s.runaway_ok_us >= cfg.runaway_deactivate_us {
                    s.runaway_active = false;
                    s.flight_proven = true;
                }
            }
        }
    }

    // Crashed: tilted far over and not righting itself, in one of two ways.
    // Stop the motors before they burn out or scoot the quad along the floor.
    //
    // Only rotation that changes the tilt counts as motion: on the ground
    // nothing stops a quad spinning flat (around the vertical, whichever
    // body axis that is), and that never rights it.
    //
    // Blocked: the sticks (or angle mode) demand a fast roll or pitch and
    // the frame does not follow. In the air it would within milliseconds,
    // so something (the ground, a wall, a tree) holds it.
    //
    // Lying: still and, by the accelerometer, propped up by something solid
    // (about 1 g pushing against its side or canopy), shortly after an
    // impact. This is the hands-off case, so it must not catch acro. An
    // inverted plunge at terminal speed reads exactly like the ground (drag
    // balances gravity), at any throttle and for as long as the fall lasts,
    // but it never hit anything: getting from flight to resting on the
    // ground takes a jolt that thrust cannot produce.
    let tilt = s.attitude.tilt();
    let slow = s.gyro_slow.get();
    let up = up_in_body_frame(s.attitude);
    let rp_motion = (slow - up * slow.dot(up)).norm();
    let a = accel.norm();
    if a > cfg.crash_impact {
        s.last_impact_us = Some(now);
    }
    let propped = (0.7 * GRAVITY..1.3 * GRAVITY).contains(&a) && -accel.z / a < 0.64;
    let lying = tilt > 60f32.to_radians() && rp_motion < 0.5 && propped;
    // Blocked means not turning the way the sticks ask, whatever else it
    // does: a corkscrew dive rolls about the vertical, like a quad spinning
    // flat, but it follows the stick.
    let demand = Vec3::new(s.setpoint.x, s.setpoint.y, 0.0);
    let wants = demand.norm();
    let blocked = tilt > 60f32.to_radians() && wants > 2.0 && slow.dot(demand) < wants;
    // Each condition times itself: a pilot rolling out of a slow inverted
    // dive turns `lying` into `blocked` for a moment, and must get the full
    // blocked time for the frame to follow.
    let since = |t: &mut Option<u64>, on: bool| if on { now - *t.get_or_insert(now) } else { *t = None; 0 };
    let lying_for = since(&mut s.lying_since_us, lying);
    let blocked_for = since(&mut s.blocked_since_us, blocked);
    let hit = s.lying_since_us.is_some_and(|l| s.last_impact_us.is_some_and(|t| t + cfg.crash_impact_window_us >= l));
    if (hit && lying_for >= cfg.crash_still_us) || blocked_for >= cfg.crash_blocked_us {
        disarm(s, DisarmReason::Crash, fx);
    }
}

/// The world's up direction in the body frame.
fn up_in_body_frame(q: Quat) -> Vec3 {
    q.inverse_rotate(Vec3::new(0.0, 0.0, -1.0))
}

fn stick(us: u16) -> f32 {
    ((us as f32 - 1500.0) / 500.0).clamp(-1.0, 1.0)
}

fn on_rc(s: &mut State, channels: &[u16; RC_CHANNELS], fx: &mut Out) {
    // A frame with any channel outside the plausible pulse range is
    // corrupted; drop it rather than act on it.
    if channels.iter().any(|&c| !(880..=2120).contains(&c)) {
        push(fx, Effect::Notice(Notice::RcFrameRejected));
        return;
    }
    let now = s.now_us;
    if s.rc_lost() {
        s.rc.valid_since_us = now;
    }
    s.rc.last_valid_us = Some(now);
    let deadband = |x: f32| if x.abs() < 0.01 { 0.0 } else { x };
    s.rc.roll = deadband(stick(channels[ch::ROLL]));
    s.rc.pitch = deadband(stick(channels[ch::PITCH]));
    s.rc.yaw = deadband(stick(channels[ch::YAW]));
    s.rc.throttle = ((channels[ch::THROTTLE] as f32 - 1000.0) / 1000.0).clamp(0.0, 1.0);

    let mode = if channels[ch::MODE] > 1700 { FlightMode::Acro } else { FlightMode::Angle };
    if mode != s.rc.mode {
        s.rc.mode = mode;
        push(fx, Effect::Notice(Notice::ModeChanged(mode)));
    }

    let gesture = channels[ch::ARM] < 1300
        && channels[ch::THROTTLE] < 1050
        && channels[ch::YAW] < 1100
        && channels[ch::PITCH] < 1100
        && channels[ch::ROLL] > 1900;
    if gesture && !s.armed {
        let since = *s.boot_gesture_since_us.get_or_insert(now);
        if now - since >= 2_000_000 {
            s.boot_gesture_since_us = None;
            push(fx, Effect::RebootToBootloader);
        }
    } else {
        s.boot_gesture_since_us = None;
    }

    let arm = channels[ch::ARM];
    let was_on = s.rc.arm_switch;
    // Hysteresis so a noisy switch channel cannot chatter.
    let on = if was_on { arm > 1300 } else { arm > 1700 };
    s.rc.arm_switch = on;
    if !on && s.armed {
        disarm(s, DisarmReason::Switch, fx);
    } else if on && !was_on && !s.armed {
        let blockers = s.arm_blockers();
        if blockers.is_empty() {
            s.armed = true;
            s.airmode_active = false;
            s.vz_est = 0.0;
            s.integral = Vec3::ZERO;
            // Runaway detection catches a build that cannot fly. Once this
            // power-up has shown it flies, a re-arm (even mid-air, to save a
            // fall) must not mistake the fight back from a crash for one;
            // crash detection handles that.
            s.runaway_active = !s.flight_proven;
            s.runaway_since_us = None;
            s.runaway_ok_us = 0;
            s.lying_since_us = None;
            s.blocked_since_us = None;
            s.last_impact_us = None;
            s.motor_faults = [false; 4];
            s.motor_fault_since_us = [None; 4];
            // Spin up to idle now. Until the next IMU sample runs the
            // controller, the keep-alive tick repeats this command; left at
            // the disarmed zeros it would hold the motors stopped while armed.
            let idle = DSHOT_MIN as f32 + roundf(s.cfg.motor_idle * (DSHOT_MAX - DSHOT_MIN) as f32);
            s.motors = [(idle as u16).clamp(DSHOT_MIN, DSHOT_MAX); 4];
            push(fx, Effect::Notice(Notice::Armed(s.rc.mode)));
            push(fx, Effect::Motors(s.motors));
        } else {
            push(fx, Effect::Notice(Notice::ArmingBlocked(blockers)));
        }
    }
}

fn on_battery(s: &mut State, volts: f32, fx: &mut Out) {
    if !volts.is_finite() || volts < 0.0 {
        return;
    }
    let v = match s.battery_volts {
        // About 0.5 s time constant at the usual 50..100 Hz sample rate.
        Some(prev) => prev + 0.05 * (volts - prev),
        None => volts,
    };
    s.battery_volts = Some(v);
    let cells = s.cfg.battery_cells as f32;
    let per_cell = v / cells;
    let hyst = 0.05;
    let level = match s.battery_level {
        BatteryLevel::Ok if per_cell < s.cfg.vbat_crit_cell => BatteryLevel::Critical,
        BatteryLevel::Ok if per_cell < s.cfg.vbat_warn_cell => BatteryLevel::Warning,
        BatteryLevel::Warning if per_cell < s.cfg.vbat_crit_cell => BatteryLevel::Critical,
        BatteryLevel::Warning if per_cell > s.cfg.vbat_warn_cell + hyst => BatteryLevel::Ok,
        BatteryLevel::Critical if per_cell > s.cfg.vbat_warn_cell + hyst => BatteryLevel::Ok,
        BatteryLevel::Critical if per_cell > s.cfg.vbat_crit_cell + hyst => BatteryLevel::Warning,
        l => l,
    };
    if level != s.battery_level {
        s.battery_level = level;
        push(fx, Effect::Notice(Notice::Battery(level)));
    }
}

fn on_telemetry(s: &mut State, erpm: &[Option<u32>; 4], fx: &mut Out) {
    if !s.armed {
        return;
    }
    let vcomp = s.battery_volts.map_or(1.0, |v| {
        (v / (s.cfg.vbat_ref_cell * s.cfg.battery_cells as f32)).clamp(0.6, 1.2)
    });
    for i in 0..4 {
        let Some(rpm) = erpm[i] else { continue };
        let expected = s.motor_frac[i] * s.cfg.motor_max_erpm * vcomp;
        let low = s.motor_frac[i] > 0.25 && (rpm as f32) < expected * s.cfg.motor_fault_frac;
        if low {
            let since = *s.motor_fault_since_us[i].get_or_insert(s.now_us);
            if !s.motor_faults[i] && s.now_us - since >= s.cfg.motor_fault_us {
                s.motor_faults[i] = true;
                push(fx, Effect::Notice(Notice::MotorFault(i as u8)));
            }
        } else if (rpm as f32) >= expected * 0.6 || s.motor_frac[i] <= 0.25 {
            s.motor_fault_since_us[i] = None;
            if s.motor_faults[i] {
                s.motor_faults[i] = false;
                push(fx, Effect::Notice(Notice::MotorRecovered(i as u8)));
            }
        }
    }
}

/// Timeouts and annunciators, run after every event.
fn housekeeping(s: &mut State, fx: &mut Out) {
    let now = s.now_us;
    let cfg = s.cfg;

    if s.armed && s.imu_stale() {
        s.imu_failed = true;
        disarm(s, DisarmReason::ImuFailure, fx);
    }

    let lost = s.rc_lost();
    let recovered = !lost
        && s.rc.last_valid_us.is_some()
        && now.saturating_sub(s.rc.valid_since_us) >= cfg.failsafe_recovery_us;
    match s.failsafe {
        FailsafeStage::None if lost && s.armed => {
            s.failsafe = FailsafeStage::Hold;
            s.failsafe_since_us = now;
            s.vz_est = 0.0;
            s.hold_vz_mark = None;
            s.throttle_at_loss = s.rc.throttle;
            push(fx, Effect::Notice(Notice::Failsafe(FailsafeStage::Hold)));
        }
        FailsafeStage::Hold | FailsafeStage::Landing if recovered => {
            s.failsafe = FailsafeStage::None;
            push(fx, Effect::Notice(Notice::FailsafeRecovered));
            if !s.rc.arm_switch {
                disarm(s, DisarmReason::Switch, fx);
            }
        }
        FailsafeStage::Hold if now - s.failsafe_since_us >= cfg.failsafe_stage1_us => {
            if !s.airmode_active || s.throttle_at_loss < 0.05 {
                // Still on the ground: just stop.
                disarm(s, DisarmReason::Failsafe, fx);
            } else {
                // The hold ran at the learned hover throttle: an upward
                // acceleration a in its second half means thrust is a/g too
                // high, so throttle is about a/2g too high.
                if let Some(mark) = s.hold_vz_mark {
                    let span = now.saturating_sub(s.failsafe_since_us + 500_000).max(100_000) as f32 * 1e-6;
                    let down_accel = (s.vz_est - mark) / span;
                    s.hover_throttle *= (1.0 + down_accel / (2.0 * GRAVITY)).clamp(0.85, 1.15);
                }
                s.failsafe = FailsafeStage::Landing;
                s.failsafe_since_us = now;
                // Keep the vertical speed the hold measured: the quad may
                // already be sinking.
                s.landing_trim = 0.0;
                s.landing_vz_ref = (now, s.vz_est);
                s.landing_accel = f32::MAX;
                push(fx, Effect::Notice(Notice::Failsafe(FailsafeStage::Landing)));
            }
        }
        FailsafeStage::Landing => {
            // Touchdown: not accelerating with the throttle trimmed to about
            // half of hover thrust, which in the air would drop the quad at
            // 5 m/s². The accelerometer magnitude is no use here: with a
            // bent prop it reads several g of vibration in flight.
            let touchdown = s.landing_trim >= 0.3 && s.landing_accel.abs() < 0.6 && s.vz_est - s.landing_trim_vz < 0.5;
            if touchdown || now - s.failsafe_since_us >= cfg.failsafe_landing_us {
                disarm(s, DisarmReason::Failsafe, fx);
            }
        }
        _ => {}
    }

    // LED: solid when armed, slow blink when ready, fast blink otherwise.
    let led = if s.armed {
        true
    } else {
        let period = if s.arm_blockers().is_empty() { 1_000_000 } else { 200_000 };
        now % period < period / 2
    };
    if led != s.led {
        s.led = led;
        push(fx, Effect::Led(led));
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
        push(fx, Effect::Beeper(beeper));
    }
}

#[cfg(test)]
mod tests;
