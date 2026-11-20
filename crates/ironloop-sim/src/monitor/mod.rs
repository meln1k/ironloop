//! Properties checked on every firmware step of every simulated run.
//!
//! Two kinds:
//! - **Safety properties** must hold in every scenario, including broken
//!   builds and hardware faults. They are about what the firmware does with
//!   the motors: never spin disarmed, never arm unsafely, stop in bounded
//!   time when the link, the IMU, or the airframe fails, never disarm a
//!   flying quad without cause.
//! - **Flight expectations** (marked `[flight]`) must hold when the airframe
//!   is flyable and no fault severe enough to excuse them is active: the
//!   attitude estimate tracks the truth, angle mode stays level, landings
//!   are soft.
//!
//! The limits are requirements written in [`req`], not read from the
//! firmware's `Config`. Each group of properties lives in its own module
//! with the state it needs; [`PROPERTIES`] lists them all.
//!
//! Every property also counts how often its precondition was live (the
//! situation it guards actually happened), so a sweep can tell a property
//! that held from one that was never exercised.

use ironloop_fc::{DisarmReason, Effect, Event, EventKind, Notice, RC_CHANNELS, Status, ch};

use crate::physics::World;
use crate::scenario::{Excuse, Scenario};

mod arming;
mod failsafe;
mod flight;
mod outputs;
mod protect;
pub mod req;
mod script;

use req::{REQ_NO_FAILSAFE_BEFORE_US, REQ_OFF_SCRIPT_RESPONSE_US, REQ_RECONVERGE_US};

/// A property the monitor checks.
#[derive(Debug)]
pub struct Property {
    pub name: &'static str,
    /// One line; `[flight]` marks a flight expectation.
    pub doc: &'static str,
}

pub static NO_PANIC: Property = Property {
    name: "no_panic",
    doc: "The firmware never panics, with overflow checks on.",
};

/// Every property the monitor checks.
pub static PROPERTIES: &[&Property] = &[
    &outputs::MOTOR_RANGE,
    &outputs::NO_SPIN_DISARMED,
    &outputs::ARMED_ABOVE_IDLE,
    &outputs::NO_BOOTLOADER_ARMED,
    &outputs::DISARM_STOPS_MOTORS,
    &arming::DISARM_JUSTIFIED,
    &arming::ARM_THROTTLE_LOW,
    &arming::ARM_SWITCH_CYCLED,
    &arming::ARM_CALIBRATED,
    &arming::ARM_BATTERY,
    &arming::ARM_LEVEL,
    &arming::SWITCH_DISARMS,
    &failsafe::FAILSAFE_ENTERS,
    &failsafe::FAILSAFE_NO_SPINUP,
    &failsafe::FAILSAFE_BOUNDED,
    &failsafe::FAILSAFE_LEVELS,
    &failsafe::FAILSAFE_DESCENDS,
    &failsafe::FAILSAFE_DISARMS_ON_TOUCHDOWN,
    &protect::IMU_LOSS_STOPS,
    &outputs::ESC_FED,
    &protect::CRASH_STOPS,
    &protect::NO_FLYAWAY,
    &outputs::FINITE_STATE,
    &outputs::NO_FALSE_CLOCK_ALARM,
    &protect::BATTERY_CRITICAL_REPORTED,
    &protect::MOTOR_FAULT_REPORTED,
    &flight::ESTIMATE_TRACKS,
    &flight::ANGLE_STAYS_LEVEL,
    &failsafe::FAILSAFE_LANDS_SOFTLY,
    &flight::NO_CRASH,
    &arming::PILOT_CAN_ARM,
    &arming::ARM_IMU_FRESH,
    &outputs::CLOCK_MONOTONIC,
    &outputs::ATTITUDE_UNIT,
    &outputs::EFFECTS_FIT,
    &NO_PANIC,
    &protect::RUNAWAY_STOPS,
    &protect::BATTERY_NOTICE_TRUE,
    &protect::MOTOR_FAULT_TRUE,
    &failsafe::LINK_RECOVERS,
    &failsafe::FAILSAFE_HOLDS,
    &flight::ESTIMATE_RECONVERGES,
    &script::RATE_TRACKS,
    &script::SETTLES,
    &flight::SAG_COMPENSATED,
    &script::ANGLE_TRACKS,
    &flight::MOTORS_QUIET,
    &protect::NO_ESC_STALL,
];

#[derive(Clone, Debug, PartialEq)]
pub struct Violation {
    pub t_us: u64,
    pub property: &'static str,
    pub detail: String,
}

#[derive(Clone, Debug, Default)]
pub struct Stats {
    pub max_alt: f32,
    pub max_dist: f32,
    pub max_est_err_deg: f32,
    pub max_impact_g: f32,
    pub max_touchdown: f32,
    pub max_tilt_angle_mode_deg: f32,
    /// Worst motor-command jitter in steady flight (RMS fraction of full).
    pub max_motor_noise: f32,
    pub armed_us: u64,
    /// Armed and more than 0.3 m off the ground.
    pub airborne_us: u64,
    pub arms: u32,
    pub disarms: u32,
    pub notices: u32,
    /// Steps on which each property's precondition was live.
    pub live: Live,
    /// Kinds of the faults that started while the quad was airborne.
    pub faults_in_flight: Vec<&'static str>,
}

/// Counts by property name, read and printed like a `BTreeMap`. The monitor
/// counts dozens of properties on every step, so a count is found by the
/// address of the name, without comparing strings.
#[derive(Clone, Default, PartialEq)]
pub struct Live(Vec<(&'static str, u64)>);

impl Live {
    pub fn get(&self, name: &str) -> Option<&u64> {
        self.0.iter().find(|(n, _)| *n == name).map(|(_, c)| c)
    }

    pub fn iter(&self) -> impl Iterator<Item = (&&'static str, &u64)> {
        self.0.iter().map(|(n, c)| (n, c))
    }

    fn bump(&mut self, name: &'static str) {
        if let Some((_, c)) = self.0.iter_mut().find(|(n, _)| core::ptr::eq(*n, name)) {
            *c += 1;
        } else if let Some((_, c)) = self.0.iter_mut().find(|(n, _)| *n == name) {
            *c += 1;
        } else {
            let at = self.0.partition_point(|(n, _)| *n < name);
            self.0.insert(at, (name, 1));
        }
    }
}

impl core::fmt::Debug for Live {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        f.debug_map().entries(self.iter()).finish()
    }
}

/// One firmware step as the simulator saw it.
#[derive(Clone, Copy)]
pub struct Step<'a> {
    /// Simulated time (the firmware's clock may differ, see `event.t_us`).
    pub t: u64,
    pub event: &'a Event,
    pub fx: &'a [Effect],
    pub before: &'a Status,
    pub after: &'a Status,
    pub world: &'a World,
    pub sc: &'a Scenario,
    /// When the sim last sent an RC frame corrupted in a way the CRC
    /// passes: the firmware cannot tell it from the pilot's sticks.
    pub rc_corrupted_us: Option<u64>,
}

/// A step plus the truth several properties judge it by.
struct Cx<'a> {
    t: u64,
    event: &'a Event,
    fx: &'a [Effect],
    before: &'a Status,
    after: &'a Status,
    world: &'a World,
    sc: &'a Scenario,
    alt: f32,
    true_tilt: f32,
    /// Off the ground and above 0.3 m, armed or not.
    airborne: bool,
    /// Armed, off the ground, and above 0.15 m.
    flying: bool,
    /// The flight expectations are off: an unflyable build or a fault.
    excused: bool,
    /// Inside a deliberate wild maneuver or the 3 s after it.
    wild: bool,
    /// Within `REQ_RECONVERGE_US` of the true rate going beyond the
    /// gyro's range: that loses the attitude, and in flight the
    /// accelerometer reads mostly thrust, so the estimate takes a few
    /// seconds to come back (Betaflight's too). Until then
    /// `estimate_reconverges` judges it, and nothing that needs the
    /// estimate right (`estimate_tracks`, `angle_tracks`) is judged.
    reconverging: bool,
    /// Within `REQ_OFF_SCRIPT_RESPONSE_US` of a corrupted RC frame that
    /// passed the CRC: the quad is following a stick the pilot never sent.
    rc_corrupted: bool,
    imu_step: bool,
    /// Time since the previous IMU step (at most 5 ms), or 0 on other steps.
    imu_dt: f32,
    /// On the step that touches down: (speed, vertical speed downward) on
    /// the last step in the air.
    touchdown: Option<(f32, f32)>,
}

/// Where a property's verdicts go.
struct Verdicts<'a> {
    violations: &'a mut Vec<Violation>,
    stats: &'a mut Stats,
}

impl Verdicts<'_> {
    fn live(&mut self, p: &Property) {
        self.stats.live.bump(p.name);
    }

    fn fail(&mut self, t_us: u64, p: &Property, detail: String) {
        // One report per property keeps the output readable; the first
        // occurrence is the interesting one.
        if self.violations.iter().any(|v| v.property == p.name) {
            return;
        }
        self.violations.push(Violation { t_us, property: p.name, detail });
    }
}

/// What the RC link really delivered, judged independently of the firmware.
#[derive(Clone, Debug)]
struct Link {
    /// Arm switch position after the last valid frame (firmware hysteresis).
    arm_on: bool,
    switch_seen_off: bool,
    last_arm_ch: u16,
    last_rc_throttle: u16,
    /// Roll, pitch, yaw channels of the last valid frame.
    rc_sticks: [u16; 3],
    last_rc_valid_us: u64,
    /// Start of the current run of frames without a failsafe-length gap.
    rc_run_since: u64,
}

impl Default for Link {
    fn default() -> Self {
        Link {
            arm_on: true,
            switch_seen_off: false,
            last_arm_ch: 1000,
            last_rc_throttle: 1000,
            rc_sticks: [1500; 3],
            last_rc_valid_us: 0,
            rc_run_since: 0,
        }
    }
}

/// This step's RC frame, if it is a valid one.
struct Frame {
    rc: Option<[u16; RC_CHANNELS]>,
    /// How long the link had been silent before this step.
    silent_before: u64,
    /// The arm switch went on with this frame.
    arm_edge: bool,
}

impl Link {
    /// Reads the arm switch. The rest of the frame is taken in by `settle`,
    /// after arming has been judged against the link as it was.
    fn observe(&mut self, cx: &Cx) -> Frame {
        let rc = match cx.event.kind {
            EventKind::Rc { channels } if channels.iter().all(|c| (880..=2120).contains(c)) => Some(channels),
            _ => None,
        };
        let silent_before = cx.t.saturating_sub(self.last_rc_valid_us);
        let mut arm_edge = false;
        if let Some(ch) = rc {
            let a = ch[ch::ARM];
            let on = if self.arm_on { a > 1300 } else { a > 1700 };
            arm_edge = on && !self.arm_on;
            self.arm_on = on;
            self.last_arm_ch = a;
        }
        Frame { rc, silent_before, arm_edge }
    }

    fn settle(&mut self, cx: &Cx, frame: &Frame) {
        let Some(ch) = frame.rc else { return };
        if ch[ch::ARM] < 1300 {
            self.switch_seen_off = true;
        }
        if cx.t.saturating_sub(self.last_rc_valid_us) > REQ_NO_FAILSAFE_BEFORE_US {
            self.rc_run_since = cx.t;
        }
        self.last_rc_valid_us = cx.t;
        self.last_rc_throttle = ch[ch::THROTTLE];
        self.rc_sticks = [ch[ch::ROLL], ch[ch::PITCH], ch[ch::YAW]];
    }
}

/// What the IMU really delivered: a sample is good when it is finite,
/// within the sensor's range, and not a bit-exact repeat.
#[derive(Clone, Debug, Default)]
struct Imu {
    last_good_us: u64,
    last_us: u64,
    last_sample: Option<([f32; 3], [f32; 3])>,
}

impl Imu {
    fn observe(&mut self, cx: &Cx) {
        let EventKind::Imu { gyro, accel } = cx.event.kind else { return };
        let finite = gyro.iter().chain(accel.iter()).all(|v| v.is_finite());
        let in_range = gyro.iter().all(|v| v.abs() <= cx.sc.imu.gyro_range * 1.5)
            && accel.iter().all(|v| v.abs() <= cx.sc.imu.accel_range * 1.5);
        let repeat = self.last_sample == Some((gyro, accel));
        self.last_sample = Some((gyro, accel));
        if finite && in_range && !repeat {
            self.last_good_us = cx.t;
        }
        self.last_us = cx.t;
    }
}

/// A fault excuses what it excuses from just before it starts until 3 s
/// after it ends.
fn excuses(sc: &Scenario, t: u64, pred: impl Fn(Excuse) -> bool) -> bool {
    sc.faults.iter().any(|f| {
        let window = f.at_us.saturating_sub(100_000)..f.end_us().saturating_add(3_000_000);
        window.contains(&t) && pred(f.kind.excuse())
    })
}

/// Faults that excuse the failsafe's flight too: not a lost link or wind,
/// which the failsafe must cope with on its own.
fn flight_fault(sc: &Scenario, t: u64) -> bool {
    excuses(sc, t, |e| e == Excuse::Flight)
}

/// Inside a deliberate wild maneuver (flip, stick slam, drop) or the 3 s
/// after it.
fn wild(sc: &Scenario, t: u64) -> bool {
    sc.segments.iter().enumerate().any(|(i, s)| {
        let end = sc.segments.get(i + 1).map_or(u64::MAX, |n| n.start_us);
        s.action.is_wild() && t >= s.start_us && t < end.saturating_add(3_000_000)
    })
}

fn disarm_reason(fx: &[Effect]) -> Option<DisarmReason> {
    fx.iter().find_map(|e| match e {
        Effect::Notice(Notice::Disarmed(r)) => Some(*r),
        _ => None,
    })
}

#[derive(Clone, Debug, Default)]
pub struct Monitor {
    pub violations: Vec<Violation>,
    pub stats: Stats,
    link: Link,
    imu: Imu,
    outputs: outputs::Outputs,
    failsafe: failsafe::Failsafe,
    protect: protect::Protect,
    flight: flight::Flight,
    script: script::Script,
    last_t: u64,
    /// (speed, vertical speed downward) in the last airborne step.
    airborne_speed: Option<(f32, f32)>,
    /// Last time the true rotation rate was beyond the gyro's range.
    gyro_saturated_us: Option<u64>,
}

impl Monitor {
    pub fn on_step(&mut self, s: Step) {
        let Step { t, event, fx, before, after, world, sc, rc_corrupted_us } = s;
        let alt = world.altitude();
        let on_ground = world.body.on_ground;
        let imu_step = matches!(event.kind, EventKind::Imu { .. });
        let touchdown = if on_ground {
            self.airborne_speed.take()
        } else {
            self.airborne_speed = Some((world.body.vel.norm(), world.body.vel.z));
            None
        };
        if world.body.omega.max_abs() >= sc.imu.gyro_range * 0.97 {
            self.gyro_saturated_us = Some(t);
        }
        let cx = Cx {
            t,
            event,
            fx,
            before,
            after,
            world,
            sc,
            alt,
            true_tilt: world.body.att.tilt(),
            airborne: !on_ground && alt > 0.3,
            flying: after.armed && !on_ground && alt > 0.15,
            excused: sc.unflyable() || excuses(sc, t, |e| e != Excuse::Nothing),
            wild: wild(sc, t),
            reconverging: self.gyro_saturated_us.is_some_and(|s| t - s < REQ_RECONVERGE_US),
            rc_corrupted: rc_corrupted_us.is_some_and(|c| t.saturating_sub(c) < REQ_OFF_SCRIPT_RESPONSE_US),
            imu_step,
            imu_dt: if imu_step { t.saturating_sub(self.imu.last_us).min(5_000) as f32 * 1e-6 } else { 0.0 },
            touchdown,
        };
        let mut v = Verdicts { violations: &mut self.violations, stats: &mut self.stats };

        // Faults that started since the last step while flying (coverage).
        if after.armed && cx.airborne {
            let started = sc.faults.iter().filter(|f| f.at_us > self.last_t && f.at_us <= t);
            v.stats.faults_in_flight.extend(started.map(|f| f.kind.name()));
        }

        // Arming and disarming are judged against the link and the IMU as
        // they were before this step; everything after sees this step's
        // frame and sample.
        let frame = self.link.observe(&cx);
        arming::step(&cx, &self.link, &frame, &self.imu, &mut v);
        self.link.settle(&cx, &frame);
        self.failsafe.step(&cx, &self.link, &frame, &mut v);
        self.imu.observe(&cx);
        self.outputs.step(&cx, &self.imu, &mut v);
        self.protect.step(&cx, &self.link, &self.imu, &mut v);
        self.flight.step(&cx, &self.link, &mut v);
        self.script.step(&cx, &self.link, &mut v);

        let dt = t.saturating_sub(self.last_t);
        if before.armed {
            v.stats.armed_us += dt;
            if cx.airborne {
                v.stats.airborne_us += dt;
            }
        }
        self.last_t = t;
    }

    /// The run ended at `t_us`, normally or by a firmware panic.
    pub fn on_end(&mut self, t_us: u64, panic: Option<String>) {
        let mut v = Verdicts { violations: &mut self.violations, stats: &mut self.stats };
        v.live(&NO_PANIC);
        if let Some(msg) = panic {
            v.fail(t_us, &NO_PANIC, msg);
        }
    }
}
