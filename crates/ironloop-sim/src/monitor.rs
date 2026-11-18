//! Properties checked on every firmware step of every simulated run.
//!
//! Two kinds:
//! - **Safety properties** must hold in every scenario, including broken
//!   builds and hardware faults. They are about what the firmware does with
//!   the motors: never spin disarmed, never arm unsafely, stop in bounded
//!   time when the link, the IMU, or the airframe fails, never disarm a
//!   flying quad without cause.
//! - **Flight expectations** must hold when the airframe is flyable and no
//!   fault severe enough to excuse them is active: the attitude estimate
//!   tracks the truth, angle mode stays level, landings are soft.
//!
//! The limits are requirements written here, not read from the firmware's
//! `Config`: a wrong value in the configuration under test must fail the
//! check instead of moving it.
//!
//! Every property also counts how often its precondition was live (the
//! situation it guards actually happened), so a sweep can tell a property
//! that held from one that was never exercised.

use std::collections::BTreeMap;

use ironloop_fc::math::Vec3;
use ironloop_fc::{
    DSHOT_MAX, DSHOT_MIN, DisarmReason, Effect, Event, EventKind, FailsafeStage, FlightMode, Notice, Status, ch,
};

use crate::physics::{MotorHealth, World};
use crate::scenario::{Action, Excuse, FaultKind, Scenario};

/// Armed with the link silent this long must be failsafe.
pub const REQ_FAILSAFE_AFTER_US: u64 = 150_000;
/// A link silent for less than this must not trigger failsafe.
pub const REQ_NO_FAILSAFE_BEFORE_US: u64 = 50_000;
/// RC lost while armed must end disarmed within this long.
pub const REQ_FAILSAFE_END_US: u64 = 35_000_000;
/// Failsafe must level the quad within this long...
pub const REQ_FAILSAFE_LEVEL_US: u64 = 1_000_000;
/// ...to within this tilt (rad).
pub const REQ_FAILSAFE_LEVEL_TILT: f32 = 20.0 * core::f32::consts::PI / 180.0;
/// A failsafe landing must be sinking at least this fast (m/s)...
pub const REQ_FAILSAFE_MIN_SINK: f32 = 0.2;
/// ...after this long in the landing stage.
pub const REQ_FAILSAFE_SINK_AFTER_US: u64 = 6_000_000;
/// A failsafe landing must disarm within this long of touching down.
pub const REQ_FAILSAFE_TOUCHDOWN_DISARM_US: u64 = 2_000_000;
/// Touchdown faster than this (m/s) is a crash.
pub const REQ_SOFT_TOUCHDOWN: f32 = 2.5;
/// Armed with no fresh IMU data this long must stop the motors.
pub const REQ_IMU_STOP_US: u64 = 30_000;
/// An IMU gap shorter than this must not disarm.
pub const REQ_IMU_TOLERATE_US: u64 = 10_000;
/// While armed the ESCs must get a command at least this often.
pub const REQ_ESC_FEED_US: u64 = 5_000;
/// Arming is never allowed tilted further than this (rad).
pub const REQ_ARM_TILT: f32 = 33.0 * core::f32::consts::PI / 180.0;
/// Tilted past this (rad) on the ground, the quad has tipped over: lying
/// after a crash, not standing on its ducts.
pub const REQ_TIPPED_OVER: f32 = 60f32.to_radians();
/// Angle mode never tilts further than this (rad).
pub const REQ_ANGLE_TILT: f32 = 75.0 * core::f32::consts::PI / 180.0;
/// Estimated tilt stays within this of the truth in flight (deg).
pub const REQ_ESTIMATE_DEG: f32 = 15.0;
/// A crashed quad must stop its motors within this long. Hands off it may
/// take a few seconds (an inverted plunge must not look like a crash).
pub const REQ_CRASH_STOP_US: u64 = 4_000_000;
/// The 1S pack below this many volts for 5 s must raise a critical notice.
pub const REQ_CRITICAL_VOLTS: f32 = 3.15;
/// A stalled motor must be reported within this long (with telemetry).
pub const REQ_MOTOR_FAULT_US: u64 = 200_000;
/// While armed every motor runs at least this DShot value (3% idle).
pub const REQ_ARMED_MIN_DSHOT: u16 = DSHOT_MIN + 60;
/// Motor idle (fraction of full command) the throttle curve starts from.
pub const REQ_IDLE: f32 = 0.055;
/// Arming never happens without good IMU data for this long.
pub const REQ_ARM_IMU_FRESH_US: u64 = 30_000;
/// The attitude quaternion stays this close to unit length.
pub const REQ_UNIT_QUAT: f32 = 1e-3;
/// An estimate off by more than `REQ_ESTIMATE_DEG` must be back within
/// `REQ_ESTIMATE_OK_DEG` of the truth this soon, even after a flip.
pub const REQ_RECONVERGE_US: u64 = 4_000_000;
pub const REQ_ESTIMATE_OK_DEG: f32 = 10.0;
/// A quad that cannot fly (motors swapped) stops this soon after the pilot
/// throttles up, and never climbs above `REQ_RUNAWAY_ALT` meters.
pub const REQ_RUNAWAY_STOP_US: u64 = 5_000_000;
pub const REQ_RUNAWAY_ALT: f32 = 1.0;
/// Acro rates the pilot expects (Betaflight defaults: rc_rate 1.0,
/// super_rate 0.7, no expo).
pub const REQ_RC_RATE: f32 = 1.0;
pub const REQ_SUPER_RATE: f32 = 0.7;
/// Rate tracking: the true rate follows the stick's rate, through a
/// first-order response of this time constant (s)...
pub const REQ_RATE_TAU: f32 = 0.03;
/// ...with an RMS error at most this fraction of the RMS commanded rate
/// (roll and pitch; yaw has a quarter of the torque authority).
pub const REQ_RATE_ERR: f32 = 0.35;
pub const REQ_YAW_RATE_ERR: f32 = 0.5;
/// Angle mode: full stick commands this tilt (Betaflight's default
/// angle limit is 55 deg)...
pub const REQ_ANGLE_LIMIT: f32 = 55.0 * core::f32::consts::PI / 180.0;
/// ...and in an angle-mode stick script the furthest each 0.3 s stick hold
/// tilts the quad, averaged over the four roll and pitch holds, is between
/// these fractions of what the stick commands. (One hold alone varies too
/// much with wind and small sticks: 0.4-1.4 in seeds 0-299.)
pub const REQ_ANGLE_REACH: (f32, f32) = (0.75, 1.15);
/// Hands off in acro the roll and pitch rates settle: RMS below this (rad/s).
pub const REQ_SETTLED_RMS: f32 = 0.4;
/// Failsafe hold: thrust within 30% of weight (vertical acceleration m/s²).
pub const REQ_HOLD_ACCEL: f32 = 3.0;
/// Battery notices need the pack truly below these within the last 2 s (V):
/// the 3.30 / 3.45 V thresholds plus 3% voltage-sensor gain error.
pub const REQ_CRIT_TRUE_V: f32 = 3.45;
pub const REQ_WARN_TRUE_V: f32 = 3.62;
pub const REQ_BATTERY_NOTICE_WINDOW_US: u64 = 2_000_000;
/// A motor fault notice needs the motor truly impaired within this long.
pub const REQ_MOTOR_FAULT_TRUE_US: u64 = 300_000;
/// With the link back without a gap for this long and the switch on, the
/// pilot is in control again.
pub const REQ_RECOVER_US: u64 = 1_000_000;
/// Sag compensation: averaged motor speed matches what the throttle stick
/// asks for at the reference voltage within this fraction, on a sagging pack
/// (the firmware filters the voltage over about 0.5 s, and a 1S pack
/// recovers faster, so a few percent of lag is expected, plus up to 3%
/// voltage-sensor gain error). Below about
/// 3.15 V compensation is capped at 1.25x and not judged.
pub const REQ_SAG_TOL: f32 = 0.15;
pub const REQ_SAG_ABOVE_V: f32 = 3.15;
pub const REQ_SAG_BELOW_V: f32 = 3.65;
pub const REQ_SAG_REF_V: f32 = 3.9;

#[derive(Clone, Debug, PartialEq)]
pub struct Violation {
    pub t_us: u64,
    pub property: &'static str,
    pub detail: String,
}

/// Every property the monitor checks, with a one-line description.
pub const PROPERTIES: &[(&str, &str)] = &[
    ("motor_range", "Every motor command is 0 (stop) or in the DShot throttle range 48..=2047."),
    ("no_spin_disarmed", "While disarmed, the last motor command of every step is all zeros."),
    ("armed_above_idle", "While armed, no motor is ever commanded below 3% idle (a stopped motor in flight)."),
    ("no_bootloader_armed", "The firmware never jumps to the bootloader while armed."),
    ("disarm_stops_motors", "A step that disarms emits an all-zero motor command."),
    ("disarm_justified", "Every disarm has a cause the truth supports (switch, real link loss, stale IMU, crash, unflyable build)."),
    ("arm_throttle_low", "Arming only happens on an RC frame with throttle at the bottom."),
    ("arm_switch_cycled", "Arming needs the arm switch seen off since boot (switch on at power-up never arms)."),
    ("arm_calibrated", "Arming only happens after gyro calibration finished."),
    ("arm_battery", "Arming never happens on USB power (no battery)."),
    ("arm_level", "Arming never happens on the ground with the quad tilted past 33 deg (on its side, upside down)."),
    ("switch_disarms", "A valid RC frame with the arm switch off always leaves the quad disarmed."),
    ("failsafe_enters", "Armed with RC silent 150 ms is failsafe; silent under 50 ms never is."),
    ("failsafe_no_spinup", "If the throttle was at the bottom when the link was lost, failsafe adds none."),
    ("failsafe_bounded", "RC lost while armed ends with motors stopped within 35 s."),
    ("failsafe_levels", "[flight] One second into failsafe the quad is within 20 deg of level."),
    ("failsafe_descends", "[flight] Six seconds into a failsafe landing the quad sinks at 0.2 m/s or more."),
    ("failsafe_disarms_on_touchdown", "A failsafe landing disarms within 2 s of touching down upright."),
    ("imu_loss_stops", "No fresh IMU data for 30 ms while armed stops the motors."),
    ("esc_fed", "While armed the ESCs receive a command at least every 5 ms (unless the firmware task is stalled)."),
    ("crash_stops", "A crashed quad (on the ground, tilted past 60 deg, not rotating toward upright) stops its motors within 4 s."),
    ("no_flyaway", "The quad never goes above 60 m or further than 120 m from home."),
    ("finite_state", "Attitude, rates, and setpoints in the firmware status are always finite."),
    ("no_false_clock_alarm", "The firmware reports the clock going backwards only when it really jumped back."),
    ("battery_critical_reported", "A 1S pack below 3.15 V for 5 s raises a critical battery notice."),
    ("motor_fault_reported", "A stalled motor that is commanded hard with telemetry available is reported within 200 ms."),
    ("estimate_tracks", "[flight] Estimated tilt stays within 15 deg of the truth while flying, except within 4 s of the gyro saturating."),
    ("angle_stays_level", "[flight] In angle mode the true tilt stays under 75 deg."),
    ("failsafe_lands_softly", "[flight] A failsafe landing touches down below 2.5 m/s vertical and never disarms above 1 m."),
    ("no_crash", "[flight] Armed touchdowns are slower than 2.5 m/s, outside deliberate drops, flips, and stick slams."),
    ("pilot_can_arm", "[flight] In a flyable scenario, flipping the arm switch on with everything ready (level, still, pack above 3.4 V) arms."),
    ("arm_imu_fresh", "Arming never happens without good IMU data in the last 30 ms."),
    ("clock_monotonic", "The firmware's clock never goes backwards, whatever order events arrive in."),
    ("attitude_unit", "The attitude quaternion stays unit length (within 1e-3)."),
    ("effects_fit", "No step ever produces more effects than its output buffer holds."),
    ("no_panic", "The firmware never panics, with overflow checks on."),
    ("runaway_stops", "A quad with its motors swapped never climbs above 1 m and stops within 5 s of throttle-up."),
    ("battery_notice_true", "Battery warning and critical notices only when the pack really was below 3.62 / 3.45 V in the last 2 s."),
    ("motor_fault_true", "A motor fault notice only for a motor really stalled or weakened in the last 300 ms."),
    ("link_recovers", "Armed in failsafe, with the link back without a gap for 1 s and the switch on, failsafe ends."),
    ("failsafe_holds", "[flight] In the failsafe hold the quad neither climbs nor sinks faster than 3 m/s² of acceleration."),
    ("estimate_reconverges", "[flight] A tilt estimate off by over 15 deg (after a flip, say) is back within 10 deg within 4 s."),
    ("rate_tracks", "[flight] In an acro stick script each axis tracks the Betaflight-rate setpoint (30 ms lag) within 35% RMS."),
    ("settles", "[flight] Hands off in an acro stick script, roll and pitch rates settle below 0.4 rad/s RMS (no oscillation)."),
    ("sag_compensated", "[flight] On a sagging pack the averaged motor speed matches the throttle stick within 15%."),
    ("angle_tracks", "[flight] In an angle-mode stick script roll and pitch stick holds tilt the quad 75-115% of the commanded angle on average (55 deg at full stick)."),
];

/// Pilot rate (rad/s) for a stick deflection, with the required rates.
pub fn req_rate(stick: f32) -> f32 {
    let x = stick.clamp(-1.0, 1.0);
    let k = 1.0 / (1.0 - x.abs() * REQ_SUPER_RATE).clamp(0.01, 1.0);
    (200.0 * REQ_RC_RATE * x * k).to_radians()
}

/// Running checks over one stick-script segment.
#[derive(Clone, Debug)]
struct ScriptCheck {
    start_us: u64,
    mode: FlightMode,
    valid: bool,
    /// Angle mode: per axis (roll, pitch) and stick hold (first, second),
    /// the furthest the quad tilted the commanded way, as a fraction of the
    /// commanded angle.
    reach: [[f32; 2]; 2],
    reference: [f32; 3],
    err2: [f32; 3],
    ref2: [f32; 3],
    settle2: f32,
    settle_n: u32,
    /// Time the sticks disagreed with the script (the pilot was landing a
    /// low pack, or a frame was corrupted), s.
    off_script: f32,
}

#[derive(Clone, Debug, Default)]
pub struct Stats {
    pub max_alt: f32,
    pub max_dist: f32,
    pub max_est_err_deg: f32,
    pub max_impact_g: f32,
    pub max_touchdown: f32,
    pub max_tilt_angle_mode_deg: f32,
    pub armed_us: u64,
    /// Armed and more than 0.3 m off the ground.
    pub airborne_us: u64,
    pub arms: u32,
    pub disarms: u32,
    pub notices: u32,
    /// Steps on which each property's precondition was live.
    pub live: BTreeMap<&'static str, u64>,
    /// Kinds of the faults that started while the quad was airborne.
    pub faults_in_flight: Vec<&'static str>,
}

#[derive(Clone, Debug)]
pub struct Monitor {
    pub violations: Vec<Violation>,
    pub stats: Stats,
    switch_seen_off: bool,
    /// Arm switch position after the last valid frame (firmware hysteresis).
    arm_on: bool,
    last_arm_ch: u16,
    last_rc_throttle: u16,
    /// Roll, pitch, yaw channels of the last valid frame.
    rc_sticks: [u16; 3],
    idle_at_loss: bool,
    last_rc_valid_us: u64,
    rc_lost_since: Option<u64>,
    last_good_imu_us: u64,
    last_imu_us: u64,
    last_imu: Option<([f32; 3], [f32; 3])>,
    last_motor_cmd_us: Option<u64>,
    inverted_since: Option<u64>,
    crit_since: Option<u64>,
    crit_reported: bool,
    /// Indexed by motor output (as the firmware numbers them), not by the
    /// physical motor: with the wires swapped, output 0 drives motor 1.
    stall_since: [Option<u64>; 4],
    fault_reported: [bool; 4],
    failsafe_since: Option<u64>,
    landing_since: Option<u64>,
    not_sinking_since: Option<u64>,
    landed_since: Option<u64>,
    max_event_t: u64,
    faults_started: usize,
    last_t: u64,
    /// (speed, vertical speed downward) in the last airborne step.
    airborne_speed: Option<(f32, f32)>,
    est_bad_since: Option<u64>,
    /// Last time the true rotation rate was beyond the gyro's range.
    gyro_saturated_us: Option<u64>,
    runaway_since: Option<u64>,
    below_crit_v_us: Option<u64>,
    below_warn_v_us: Option<u64>,
    /// Per motor output, like `fault_reported`.
    motor_bad_us: [Option<u64>; 4],
    rc_run_since: u64,
    hold_vz: Option<f32>,
    script: Option<ScriptCheck>,
    last_imu_step_us: u64,
    /// Sag check window: seconds, summed motor speed and stick demand.
    sag: (f32, f32, f32),
    /// True pack voltage averaged over about 0.5 s.
    volts_avg: f32,
}

impl Monitor {
    pub fn new(_sc: &Scenario) -> Self {
        Monitor {
            violations: Vec::new(),
            stats: Stats::default(),
            switch_seen_off: false,
            arm_on: true,
            last_arm_ch: 1000,
            last_rc_throttle: 1000,
            rc_sticks: [1500; 3],
            idle_at_loss: false,
            last_rc_valid_us: 0,
            rc_lost_since: None,
            last_good_imu_us: 0,
            last_imu_us: 0,
            last_imu: None,
            last_motor_cmd_us: None,
            inverted_since: None,
            crit_since: None,
            crit_reported: false,
            stall_since: [None; 4],
            fault_reported: [false; 4],
            failsafe_since: None,
            landing_since: None,
            not_sinking_since: None,
            landed_since: None,
            max_event_t: 0,
            faults_started: 0,
            last_t: 0,
            airborne_speed: None,
            est_bad_since: None,
            gyro_saturated_us: None,
            runaway_since: None,
            below_crit_v_us: None,
            below_warn_v_us: None,
            motor_bad_us: [None; 4],
            rc_run_since: 0,
            hold_vz: None,
            script: None,
            last_imu_step_us: 0,
            sag: (0.0, 0.0, 0.0),
            volts_avg: 0.0,
        }
    }

    fn fail(&mut self, t_us: u64, property: &'static str, detail: String) {
        // One report per property keeps the output readable; the first
        // occurrence is the interesting one.
        if self.violations.iter().any(|v| v.property == property) {
            return;
        }
        self.violations.push(Violation { t_us, property, detail });
    }

    fn live(&mut self, property: &'static str) {
        *self.stats.live.entry(property).or_default() += 1;
    }

    /// A fault excuses what it excuses from just before it starts until 3 s
    /// after it ends.
    fn excuses(sc: &Scenario, t: u64, pred: impl Fn(Excuse) -> bool) -> bool {
        sc.faults.iter().any(|f| {
            let window = f.at_us.saturating_sub(100_000)..f.at_us.saturating_add(f.dur_us).saturating_add(3_000_000);
            window.contains(&t) && pred(f.kind.excuse())
        })
    }

    /// Faults that excuse the flight expectations.
    fn excusing_fault(sc: &Scenario, t: u64) -> bool {
        Self::excuses(sc, t, |e| e != Excuse::Nothing)
    }

    /// Faults that excuse the failsafe's flight too: not a lost link or wind,
    /// which the failsafe must cope with on its own.
    fn flight_fault(sc: &Scenario, t: u64) -> bool {
        Self::excuses(sc, t, |e| e == Excuse::Flight)
    }

    /// Inside a deliberate wild maneuver (flip, stick slam, drop) or the
    /// 3 s after it.
    fn wild(sc: &Scenario, t: u64) -> bool {
        sc.segments.iter().enumerate().any(|(i, s)| {
            let end = sc.segments.get(i + 1).map_or(u64::MAX, |n| n.start_us);
            s.action.is_wild() && t >= s.start_us
                && t < end.saturating_add(3_000_000)
        })
    }

    #[allow(clippy::too_many_arguments)]
    pub fn on_step(
        &mut self,
        t: u64,
        event: &Event,
        fx: &[Effect],
        before: &Status,
        after: &Status,
        world: &World,
        sc: &Scenario,
    ) {
        let alt = world.altitude();
        let true_tilt = world.body.att.tilt();
        let airborne = !world.body.on_ground && alt > 0.3;

        // Faults that start while flying (coverage).
        while let Some(f) = sc.faults.get(self.faults_started) {
            if f.at_us > t {
                break;
            }
            if after.armed && airborne {
                self.stats.faults_in_flight.push(f.kind.name());
            }
            self.faults_started += 1;
        }

        // Truth the notice checks compare against.
        if world.battery.volts < REQ_CRIT_TRUE_V {
            self.below_crit_v_us = Some(t);
        }
        if world.battery.volts < REQ_WARN_TRUE_V {
            self.below_warn_v_us = Some(t);
        }
        for i in 0..4 {
            if world.motors[sc.wired(i)].health != MotorHealth::Ok {
                self.motor_bad_us[i] = Some(t);
            }
        }
        if sc.esc_telemetry && before.armed && matches!(event.kind, EventKind::MotorTelemetry { .. }) {
            self.live("motor_fault_true");
        }

        // The firmware's own clock, its state, and its output buffer.
        self.live("clock_monotonic");
        if after.now_us < before.now_us && before.calibrated {
            self.fail(t, "clock_monotonic", format!("clock went from {} to {} us", before.now_us, after.now_us));
        }
        self.live("attitude_unit");
        let q = after.attitude;
        let norm = libm::sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        if (norm - 1.0).abs() > REQ_UNIT_QUAT {
            self.fail(t, "attitude_unit", format!("|q| = {norm:.5}"));
        }
        if fx.len() >= 4 {
            self.live("effects_fit");
        }
        if after.effects_dropped > 0 {
            self.fail(t, "effects_fit", format!("{} effects dropped", after.effects_dropped));
        }

        // Clock: only a real jump back may raise the alarm. The firmware's
        // inputs can arrive slightly out of order (board queueing), which
        // it must absorb silently.
        let jumped = sc.faults.iter().any(|f| matches!(f.kind, FaultKind::ClockGlitch { .. }) && t >= f.at_us && t < f.at_us + 50_000);
        if event.t_us < self.max_event_t {
            self.live("no_false_clock_alarm");
        }
        self.max_event_t = self.max_event_t.max(event.t_us);
        if fx.contains(&Effect::Notice(Notice::ClockWentBackwards)) && !jumped {
            self.fail(t, "no_false_clock_alarm", "ClockWentBackwards without a clock jump".into());
        }

        // Motor commands.
        let mut last_motors = None;
        for e in fx {
            if let Effect::Motors(m) = e {
                self.live("motor_range");
                if m.iter().any(|&v| v != 0 && !(DSHOT_MIN..=DSHOT_MAX).contains(&v)) {
                    self.fail(t, "motor_range", format!("{m:?}"));
                }
                if after.armed {
                    self.live("armed_above_idle");
                    if m.iter().any(|&v| v < REQ_ARMED_MIN_DSHOT) {
                        self.fail(t, "armed_above_idle", format!("{m:?}"));
                    }
                }
                last_motors = Some(*m);
                self.last_motor_cmd_us = Some(t);
            }
            if *e == Effect::RebootToBootloader && (before.armed || after.armed) {
                self.fail(t, "no_bootloader_armed", String::new());
            }
            if let Effect::Notice(n) = e {
                self.stats.notices += 1;
                let recent = |since: Option<u64>| since.is_some_and(|s| t.saturating_sub(s) <= REQ_BATTERY_NOTICE_WINDOW_US);
                match n {
                    // A Warning after Critical reports the pack recovering.
                    Notice::Battery(level) if *level != ironloop_fc::BatteryLevel::Ok && !(*level == ironloop_fc::BatteryLevel::Warning && before.battery_level == ironloop_fc::BatteryLevel::Critical) => {
                        self.live("battery_notice_true");
                        let crit = *level == ironloop_fc::BatteryLevel::Critical;
                        if crit {
                            self.crit_reported = true;
                        }
                        let (seen, limit) = if crit { (self.below_crit_v_us, REQ_CRIT_TRUE_V) } else { (self.below_warn_v_us, REQ_WARN_TRUE_V) };
                        if !recent(seen) {
                            self.fail(t, "battery_notice_true", format!("{level:?} with the pack at {:.2} V, not below {limit} V in 2 s", world.battery.volts));
                        }
                    }
                    Notice::MotorFault(i) => {
                        let i = *i as usize;
                        self.fault_reported[i] = true;
                        let bad = self.motor_bad_us[i].is_some_and(|b| t.saturating_sub(b) <= REQ_MOTOR_FAULT_TRUE_US);
                        if !bad {
                            self.fail(t, "motor_fault_true", format!("motor {i} reported faulty but healthy"));
                        }
                    }
                    _ => {}
                }
            }
        }
        if matches!(event.kind, EventKind::Command(_)) && before.armed {
            self.live("no_bootloader_armed");
        }
        if !after.armed {
            if let Some(m) = last_motors {
                self.live("no_spin_disarmed");
                if m != [0; 4] {
                    self.fail(t, "no_spin_disarmed", format!("{m:?}"));
                }
            }
        }

        // RC bookkeeping, before judging arm and disarm.
        let rc_valid = match event.kind {
            EventKind::Rc { channels } if channels.iter().all(|c| (880..=2120).contains(c)) => Some(channels),
            _ => None,
        };
        let silent_before = t.saturating_sub(self.last_rc_valid_us);
        let mut arm_edge = false;
        if let Some(ch) = rc_valid {
            let a = ch[ch::ARM];
            let on = if self.arm_on { a > 1300 } else { a > 1700 };
            arm_edge = on && !self.arm_on;
            if on && !self.switch_seen_off {
                self.live("arm_switch_cycled");
            }
            self.arm_on = on;
            self.last_arm_ch = a;
        }

        // Disarming.
        if before.armed && !after.armed {
            self.stats.disarms += 1;
            self.live("disarm_stops_motors");
            if last_motors != Some([0; 4]) {
                self.fail(t, "disarm_stops_motors", format!("last motors {last_motors:?}"));
            }
            let reason = fx.iter().find_map(|e| match e {
                Effect::Notice(Notice::Disarmed(r)) => Some(*r),
                _ => None,
            });
            if airborne {
                self.live("disarm_justified");
            }
            let imu_stale = t.saturating_sub(self.last_good_imu_us) >= REQ_IMU_TOLERATE_US;
            let justified = match reason {
                Some(DisarmReason::Switch) => self.last_arm_ch <= 1300,
                Some(DisarmReason::Failsafe) => before.failsafe != FailsafeStage::None,
                Some(DisarmReason::ImuFailure) => imu_stale,
                // Stopping a quad that cannot fly (dead motor) is no false alarm.
                Some(DisarmReason::Crash) => (alt < 0.3 && true_tilt > REQ_TIPPED_OVER) || sc.unflyable(),
                Some(DisarmReason::RunawayTakeoff) => sc.unflyable(),
                None => false,
            };
            if !justified {
                self.fail(
                    t,
                    "disarm_justified",
                    format!("{reason:?} at {alt:.1} m, tilt {:.0} deg, IMU fresh {} ms ago", true_tilt.to_degrees(), t.saturating_sub(self.last_good_imu_us) / 1000),
                );
            }
            if reason == Some(DisarmReason::Failsafe) && alt > 1.0 && !sc.unflyable() && !Self::flight_fault(sc, t) {
                self.fail(t, "failsafe_lands_softly", format!("failsafe disarmed at {alt:.1} m"));
            }
        }

        // Arming.
        if !before.armed && after.armed {
            self.stats.arms += 1;
            match rc_valid {
                None => self.fail(t, "arm_throttle_low", format!("armed on non-RC event {:?}", event.kind)),
                Some(ch) => {
                    if ch[ch::THROTTLE] > 1050 {
                        self.fail(t, "arm_throttle_low", format!("throttle {}", ch[ch::THROTTLE]));
                    }
                }
            }
            if !self.switch_seen_off {
                self.fail(t, "arm_switch_cycled", "armed without the switch ever seen off".into());
            }
            if !after.calibrated {
                self.fail(t, "arm_calibrated", String::new());
            }
            if t.saturating_sub(self.last_good_imu_us) > REQ_ARM_IMU_FRESH_US {
                self.fail(t, "arm_imu_fresh", format!("armed with no good IMU data for {} ms", t.saturating_sub(self.last_good_imu_us) / 1000));
            }
            if world.battery.absent {
                self.fail(t, "arm_battery", "armed on USB power".into());
            }
            // On the ground: re-arming in the air (after an IMU fault
            // disarm) is a recovery, not a tipped-over quad spinning up.
            if world.body.on_ground && true_tilt > REQ_ARM_TILT {
                self.fail(t, "arm_level", format!("true tilt {:.0} deg", true_tilt.to_degrees()));
            }
        }
        if let Some(ch) = rc_valid {
            if arm_edge && !before.armed {
                if ch[ch::THROTTLE] > 1050 {
                    self.live("arm_throttle_low");
                }
                if !before.calibrated {
                    self.live("arm_calibrated");
                }
                if t.saturating_sub(self.last_good_imu_us) > REQ_ARM_IMU_FRESH_US {
                    self.live("arm_imu_fresh");
                }
                if world.battery.absent {
                    self.live("arm_battery");
                }
                if world.body.on_ground && true_tilt > REQ_ARM_TILT {
                    self.live("arm_level");
                }
                // Everything the firmware needs is really there: the pilot
                // must be able to arm.
                let ready = ch[ch::THROTTLE] < 1050
                    && before.calibrated
                    && world.battery.volts > 3.4
                    && true_tilt < 15f32.to_radians()
                    && world.body.omega.norm() < 0.05
                    && t.saturating_sub(self.last_good_imu_us) < 2_000
                    && !sc.unflyable()
                    && !Self::excusing_fault(sc, t)
                    && !(sc.moved_at_boot && t < 4_000_000);
                if ready {
                    self.live("pilot_can_arm");
                    if !after.armed {
                        self.fail(t, "pilot_can_arm", format!("arm refused, blockers {:?}", after.blockers));
                    }
                }
            }
            if ch[ch::ARM] < 1300 {
                self.switch_seen_off = true;
                if before.armed {
                    self.live("switch_disarms");
                }
                if after.armed {
                    self.fail(t, "switch_disarms", "still armed after switch-off frame".into());
                }
            }
            if t.saturating_sub(self.last_rc_valid_us) > REQ_NO_FAILSAFE_BEFORE_US {
                self.rc_run_since = t;
            }
            self.last_rc_valid_us = t;
            self.last_rc_throttle = ch[ch::THROTTLE];
            self.rc_sticks = [ch[ch::ROLL], ch[ch::PITCH], ch[ch::YAW]];
        }
        let link_back = t.saturating_sub(self.last_rc_valid_us) <= REQ_NO_FAILSAFE_BEFORE_US && self.last_arm_ch > 1700;
        if before.armed && before.failsafe != FailsafeStage::None && link_back {
            self.live("link_recovers");
            if after.armed && after.failsafe != FailsafeStage::None && t - self.rc_run_since >= REQ_RECOVER_US {
                self.fail(t, "link_recovers", format!("still in {:?} with the link back for {} ms", after.failsafe, (t - self.rc_run_since) / 1000));
            }
        }

        // RC loss.
        // Airmode may lift total throttle to keep corrections in range; what
        // the failsafe must not do is ask for throttle of its own.
        // Judge by the throttle when the link went: frames that arrive
        // during the recovery window do not change what failsafe may do.
        if before.failsafe == FailsafeStage::None && after.failsafe != FailsafeStage::None {
            self.idle_at_loss = self.last_rc_throttle < 1050;
            if silent_before < REQ_NO_FAILSAFE_BEFORE_US && rc_valid.is_none() {
                self.fail(t, "failsafe_enters", format!("failsafe after only {} ms of silence", silent_before / 1000));
            }
        }
        if after.failsafe != FailsafeStage::None && self.idle_at_loss {
            self.live("failsafe_no_spinup");
            if after.throttle_demand > 0.0 {
                self.fail(t, "failsafe_no_spinup", format!("failsafe demands {:.2} after link lost at idle", after.throttle_demand));
            }
        }
        let silent = t.saturating_sub(self.last_rc_valid_us);
        if after.armed && silent > REQ_FAILSAFE_AFTER_US {
            self.live("failsafe_enters");
            self.live("failsafe_bounded");
            if after.failsafe == FailsafeStage::None {
                self.fail(t, "failsafe_enters", format!("RC silent {} ms, no failsafe", silent / 1000));
            }
            let since = *self.rc_lost_since.get_or_insert(self.last_rc_valid_us);
            if t - since > REQ_FAILSAFE_END_US {
                self.fail(t, "failsafe_bounded", format!("still armed {} ms after RC loss", (t - since) / 1000));
            }
        } else if silent <= REQ_NO_FAILSAFE_BEFORE_US {
            self.rc_lost_since = None;
        }

        // Failsafe flight: level, sink, disarm on touchdown.
        let flyable_now = !sc.unflyable() && !Self::flight_fault(sc, t);
        if after.armed && after.failsafe != FailsafeStage::None {
            let since = *self.failsafe_since.get_or_insert(t);
            if airborne && flyable_now && t - since > REQ_FAILSAFE_LEVEL_US {
                self.live("failsafe_levels");
                if true_tilt > REQ_FAILSAFE_LEVEL_TILT {
                    self.fail(t, "failsafe_levels", format!("tilt {:.0} deg {} ms into failsafe", true_tilt.to_degrees(), (t - since) / 1000));
                }
            }
        } else {
            self.failsafe_since = None;
        }
        if after.armed && after.failsafe == FailsafeStage::Hold && !self.idle_at_loss {
            // The hold flies at the learned hover throttle: measure the
            // vertical acceleration over its second half.
            let into = self.failsafe_since.map_or(0, |s| t - s);
            if into < 300_000 {
                self.hold_vz = None;
            } else if self.hold_vz.is_none() {
                self.hold_vz = Some(world.body.vel.z);
            } else if into >= 900_000 && alt > 1.0 && flyable_now {
                if let Some(vz0) = self.hold_vz.take() {
                    self.live("failsafe_holds");
                    let a = (world.body.vel.z - vz0) / (into as f32 * 1e-6 - 0.3);
                    if a.abs() > REQ_HOLD_ACCEL {
                        self.fail(t, "failsafe_holds", format!("vertical acceleration {a:.1} m/s2 in the hold"));
                    }
                    self.hold_vz = Some(f32::NAN);
                }
            }
        }
        if after.armed && after.failsafe == FailsafeStage::Landing {
            let since = *self.landing_since.get_or_insert(t);
            let sink = world.body.vel.z;
            if alt > 1.0 && flyable_now && t - since > REQ_FAILSAFE_SINK_AFTER_US {
                self.live("failsafe_descends");
                if sink < REQ_FAILSAFE_MIN_SINK {
                    let ns = *self.not_sinking_since.get_or_insert(t);
                    if t - ns > 1_000_000 {
                        self.fail(t, "failsafe_descends", format!("sinking {sink:.2} m/s at {alt:.1} m"));
                    }
                } else {
                    self.not_sinking_since = None;
                }
            }
            // Upright; a quad that tipped over landing is crash_stops' case.
            if world.body.on_ground && alt < 0.1 && true_tilt < REQ_TIPPED_OVER {
                self.live("failsafe_disarms_on_touchdown");
                let landed = *self.landed_since.get_or_insert(t);
                if t - landed > REQ_FAILSAFE_TOUCHDOWN_DISARM_US {
                    self.fail(t, "failsafe_disarms_on_touchdown", format!("armed on the ground {} ms after touchdown", (t - landed) / 1000));
                }
            } else {
                self.landed_since = None;
            }
        } else {
            self.landing_since = None;
            self.not_sinking_since = None;
            self.landed_since = None;
        }

        // IMU freshness, judged independently of the firmware: a sample is
        // good when it is finite, within the sensor's range, and not a
        // bit-exact repeat.
        if let EventKind::Imu { gyro, accel } = event.kind {
            let finite = gyro.iter().chain(accel.iter()).all(|v| v.is_finite());
            let in_range = gyro.iter().all(|v| v.abs() <= sc.imu.gyro_range * 1.5)
                && accel.iter().all(|v| v.abs() <= sc.imu.accel_range * 1.5);
            let repeat = self.last_imu == Some((gyro, accel));
            self.last_imu = Some((gyro, accel));
            if finite && in_range && !repeat {
                self.last_good_imu_us = t;
            }
            self.last_imu_us = t;
        }
        let imu_stale = t.saturating_sub(self.last_good_imu_us);
        if before.armed && imu_stale > REQ_IMU_TOLERATE_US {
            self.live("imu_loss_stops");
        }
        if after.armed && imu_stale > REQ_IMU_STOP_US {
            self.fail(t, "imu_loss_stops", format!("{} ms without good IMU data", imu_stale / 1000));
        }

        if after.armed {
            if t.saturating_sub(self.last_imu_us) > 2_000 {
                self.live("esc_fed");
            }
            if let Some(last) = self.last_motor_cmd_us {
                let stalled = sc
                    .faults
                    .iter()
                    .any(|f| matches!(f.kind, FaultKind::FcStall) && f.at_us <= t && f.end_us().saturating_add(1_000) >= last);
                if t - last > REQ_ESC_FEED_US && !stalled {
                    self.fail(t, "esc_fed", format!("{} ms without a motor command", (t - last) / 1000));
                }
            }
        }

        // Crash: tipped over on the ground and not righting itself (it may
        // still spin flat or skid).
        let w_world = world.body.att.rotate(world.body.omega);
        let stuck = Vec3::new(w_world.x, w_world.y, 0.0).norm() < 1.0;
        if after.armed && world.body.on_ground && true_tilt > REQ_TIPPED_OVER && stuck {
            self.live("crash_stops");
            let since = *self.inverted_since.get_or_insert(t);
            if t - since > REQ_CRASH_STOP_US {
                self.fail(t, "crash_stops", format!("armed, crashed at {:.0} deg for {} ms", true_tilt.to_degrees(), (t - since) / 1000));
            }
        } else {
            self.inverted_since = None;
        }

        // Runaway: a quad that cannot fly must not be allowed to keep trying.
        if sc.motors_swapped && after.armed && self.last_rc_throttle > 1300 {
            self.live("runaway_stops");
            let since = *self.runaway_since.get_or_insert(t);
            if t - since > REQ_RUNAWAY_STOP_US || alt > REQ_RUNAWAY_ALT {
                self.fail(t, "runaway_stops", format!("still armed {} ms after throttle-up, alt {alt:.1} m", (t - since) / 1000));
            }
        } else if !after.armed {
            self.runaway_since = None;
        }

        // Fly-away.
        let dist = Vec3::new(world.body.pos.x, world.body.pos.y, 0.0).norm();
        self.stats.max_alt = self.stats.max_alt.max(alt);
        self.stats.max_dist = self.stats.max_dist.max(dist);
        if after.armed && airborne {
            self.live("no_flyaway");
        }
        if alt > 60.0 || dist > 120.0 {
            self.fail(t, "no_flyaway", format!("alt {alt:.1} m, dist {dist:.1} m"));
        }

        self.live("finite_state");
        let finite = after.attitude.is_finite() && after.gyro.is_finite() && after.rate_setpoint.is_finite() && after.throttle.is_finite();
        if !finite {
            self.fail(t, "finite_state", format!("{after:?}"));
        }

        // Battery critical must be reported.
        if !world.battery.absent && world.battery.volts < REQ_CRITICAL_VOLTS && world.battery.volts > 0.5 {
            self.live("battery_critical_reported");
            let since = *self.crit_since.get_or_insert(t);
            if t - since > 5_000_000 && !self.crit_reported {
                self.fail(t, "battery_critical_reported", format!("{:.2} V for 5 s", world.battery.volts));
            }
        } else {
            self.crit_since = None;
        }

        // Stalled motor detection (needs bidirectional DShot telemetry).
        let telemetry_ok =
            sc.esc_telemetry && !sc.faults.iter().any(|f| f.active(t) && matches!(f.kind, FaultKind::TelemetryLoss));
        for i in 0..4 {
            let stalled = matches!(world.motors[sc.wired(i)].health, MotorHealth::Dead | MotorHealth::Desync);
            let hard = after.motors[i] > DSHOT_MIN + 800;
            if after.armed && stalled && hard && telemetry_ok {
                self.live("motor_fault_reported");
                let since = *self.stall_since[i].get_or_insert(t);
                if t - since > REQ_MOTOR_FAULT_US && !self.fault_reported[i] {
                    self.fail(t, "motor_fault_reported", format!("motor {i} stalled {} ms", (t - since) / 1000));
                }
            } else {
                self.stall_since[i] = None;
            }
        }

        // Flight expectations.
        let dt = t.saturating_sub(self.last_t);
        if before.armed {
            self.stats.armed_us += dt;
            if airborne {
                self.stats.airborne_us += dt;
            }
        }
        let flying = after.armed && !world.body.on_ground && alt > 0.15;
        let excused = sc.unflyable() || Self::excusing_fault(sc, t);
        let wild = Self::wild(sc, t);
        // A rotation beyond the gyro's range loses the attitude, and in
        // flight the accelerometer reads mostly thrust, so the estimate
        // takes a few seconds to come back (Betaflight's too); until then
        // `estimate_reconverges` judges it instead.
        if world.body.omega.max_abs() >= sc.imu.gyro_range * 0.97 {
            self.gyro_saturated_us = Some(t);
        }
        let recovering = self.gyro_saturated_us.is_some_and(|s| t - s < REQ_RECONVERGE_US);
        if flying && !excused && !wild && !recovering && matches!(event.kind, EventKind::Imu { .. }) {
            let est_up = after.attitude.inverse_rotate(Vec3::new(0.0, 0.0, -1.0));
            let true_up = world.body.att.inverse_rotate(Vec3::new(0.0, 0.0, -1.0));
            let err = libm::acosf(est_up.dot(true_up).clamp(-1.0, 1.0)).to_degrees();
            self.live("estimate_tracks");
            self.stats.max_est_err_deg = self.stats.max_est_err_deg.max(err);
            if err > REQ_ESTIMATE_DEG {
                self.fail(t, "estimate_tracks", format!("tilt estimate off by {err:.1} deg"));
            }
            if after.mode == FlightMode::Angle {
                self.live("angle_stays_level");
                let tilt = true_tilt.to_degrees();
                self.stats.max_tilt_angle_mode_deg = self.stats.max_tilt_angle_mode_deg.max(tilt);
                if true_tilt > REQ_ANGLE_TILT {
                    self.fail(t, "angle_stays_level", format!("tilt {tilt:.0} deg in angle mode"));
                }
            }
        }
        let imu_step = matches!(event.kind, EventKind::Imu { .. });
        let imu_dt = if imu_step { (t.saturating_sub(self.last_imu_step_us)).min(5_000) as f32 * 1e-6 } else { 0.0 };
        if imu_step {
            self.last_imu_step_us = t;
        }
        // Re-convergence, including during flips and slams.
        if flying && !excused && imu_step {
            let est_up = after.attitude.inverse_rotate(Vec3::new(0.0, 0.0, -1.0));
            let true_up = world.body.att.inverse_rotate(Vec3::new(0.0, 0.0, -1.0));
            let err = libm::acosf(est_up.dot(true_up).clamp(-1.0, 1.0)).to_degrees();
            if err > REQ_ESTIMATE_DEG {
                self.est_bad_since.get_or_insert(t);
            } else if err < REQ_ESTIMATE_OK_DEG {
                self.est_bad_since = None;
            }
            if let Some(b) = self.est_bad_since {
                self.live("estimate_reconverges");
                if t - b > REQ_RECONVERGE_US {
                    self.fail(t, "estimate_reconverges", format!("tilt estimate off by {err:.1} deg for {} ms", (t - b) / 1000));
                }
            }
        } else {
            self.est_bad_since = None;
        }

        // Stick script: open-loop rate tracking and settling in acro, how
        // far each stick tilts the quad in angle mode.
        let seg = sc.segment_at(t).copied();
        let in_script = seg.filter(|g| matches!(g.action, Action::StickScript { .. }));
        if self.script.as_ref().is_some_and(|c| in_script.is_none_or(|g| g.start_us != c.start_us)) {
            let c = self.script.take().unwrap();
            if c.valid && c.off_script < 0.1 && c.mode == FlightMode::Angle {
                self.live("angle_tracks");
                let r = c.reach.iter().flatten().sum::<f32>() / 4.0;
                if !(REQ_ANGLE_REACH.0..=REQ_ANGLE_REACH.1).contains(&r) {
                    self.fail(t, "angle_tracks", format!("stick holds reached {:.0}% of the commanded angle on average", r * 100.0));
                }
            }
            if c.valid && c.off_script < 0.1 && c.mode == FlightMode::Acro {
                for (i, axis) in ["roll", "pitch", "yaw"].iter().enumerate() {
                    if c.ref2[i] > 0.0 {
                        self.live("rate_tracks");
                        let rel = libm::sqrtf(c.err2[i] / c.ref2[i]);
                        if rel > if i == 2 { REQ_YAW_RATE_ERR } else { REQ_RATE_ERR } {
                            self.fail(t, "rate_tracks", format!("{axis} rate off by {:.0}% RMS", rel * 100.0));
                        }
                    }
                }
                if c.settle_n > 0 {
                    self.live("settles");
                    let rms = libm::sqrtf(c.settle2 / c.settle_n as f32);
                    if rms > REQ_SETTLED_RMS {
                        self.fail(t, "settles", format!("hands-off roll/pitch rate {rms:.2} rad/s RMS"));
                    }
                }
            }
        }
        if let Some(g) = in_script {
            let c = self.script.get_or_insert(ScriptCheck {
                start_us: g.start_us,
                mode: after.mode,
                valid: true,
                reach: [[f32::MIN; 2]; 2],
                reference: [0.0; 3],
                err2: [0.0; 3],
                ref2: [0.0; 3],
                settle2: 0.0,
                settle_n: 0,
                off_script: 0.0,
            });
            if !flying || excused || after.failsafe != FailsafeStage::None || after.mode != c.mode || alt < 1.0 {
                c.valid = false;
            }
            if imu_step && c.valid {
                let into = t - g.start_us;
                let ch = |v: u16| {
                    let x = (v as f32 - 1500.0) / 500.0;
                    if x.abs() < 0.01 { 0.0 } else { x }
                };
                let Action::StickScript { amp } = g.action else { unreachable!() };
                let mut script = [0.0f32; 3];
                if let Some((axis, k)) = crate::scenario::stick_script(into) {
                    script[axis] = amp * k;
                }
                if (0..3).any(|i| (ch(self.rc_sticks[i]) - script[i]).abs() > 0.02) {
                    c.off_script += imu_dt;
                }
                let sp = [req_rate(ch(self.rc_sticks[0])), -req_rate(ch(self.rc_sticks[1])), req_rate(ch(self.rc_sticks[2]))];
                let w = [world.body.omega.x, world.body.omega.y, world.body.omega.z];
                let k = (imu_dt / REQ_RATE_TAU).min(1.0);
                for i in 0..3 {
                    c.reference[i] += (sp[i] - c.reference[i]) * k;
                }
                if let Some((axis @ 0..=1, k)) = crate::scenario::stick_script(into) {
                    // Stick right rolls right; stick forward pitches the
                    // nose down.
                    let (roll, pitch, _) = world.body.att.to_euler();
                    let angle = if axis == 0 { roll } else { -pitch };
                    let r = &mut c.reach[axis][if k > 0.0 { 0 } else { 1 }];
                    *r = r.max(angle / (amp * k * REQ_ANGLE_LIMIT));
                }
                if let Some((axis, _)) = crate::scenario::stick_script(into) {
                    let e = w[axis] - c.reference[axis];
                    c.err2[axis] += e * e * imu_dt;
                    c.ref2[axis] += c.reference[axis] * c.reference[axis] * imu_dt;
                }
                let ms = into / 1000;
                if (150..400).contains(&ms) || (3_300..3_800).contains(&ms) {
                    c.settle2 += w[0] * w[0] + w[1] * w[1];
                    c.settle_n += 1;
                }
            }
        }

        // Sag compensation: on a sagging pack, the motors spin as fast on
        // average as the stick asks for at the reference voltage.
        if imu_step {
            self.volts_avg += (world.battery.volts - self.volts_avg) * (imu_dt / 0.5).min(1.0);
        }
        // Only on a steady pack: right after a punch-out it rebounds faster
        // than any reasonable voltage filter follows.
        let steady = (world.battery.volts - self.volts_avg).abs() < 0.05;
        let stick_thr = (self.last_rc_throttle as f32 - 1000.0) / 1000.0;
        let cmd_ok = after.motors.iter().all(|&m| m > DSHOT_MIN + 170 && m < DSHOT_MAX - 60);
        let sag_ok = flying
            && !excused
            && !wild
            && after.failsafe == FailsafeStage::None
            && (0.15..0.85).contains(&stick_thr)
            && cmd_ok
            && world.body.omega.norm() < 3.0
            && !world.battery.absent
            && world.battery.volts < REQ_SAG_BELOW_V
            && world.battery.volts > REQ_SAG_ABOVE_V
            && steady;
        if imu_step {
            if sag_ok {
                let spin = world.motors.iter().map(|m| m.omega).sum::<f32>() / (4.0 * world.frame.omega_max);
                let want = REQ_IDLE * world.battery.volts / REQ_SAG_REF_V + (1.0 - REQ_IDLE) * stick_thr;
                self.sag.0 += imu_dt;
                self.sag.1 += spin * imu_dt;
                self.sag.2 += want * imu_dt;
                if self.sag.0 >= 0.5 {
                    self.live("sag_compensated");
                    let ratio = self.sag.1 / self.sag.2;
                    if (ratio - 1.0).abs() > REQ_SAG_TOL {
                        self.fail(t, "sag_compensated", format!("motors at {:.0}% of what the stick asks at {:.2} V", ratio * 100.0, world.battery.volts));
                    }
                    self.sag = (0.0, 0.0, 0.0);
                }
            } else {
                self.sag = (0.0, 0.0, 0.0);
            }
        }

        let impact_g = world.body.impact / 9.80665;
        self.stats.max_impact_g = self.stats.max_impact_g.max(impact_g);
        if world.body.on_ground {
            if let Some((speed, vz)) = self.airborne_speed.take() {
                if before.failsafe == FailsafeStage::Landing || after.failsafe == FailsafeStage::Landing {
                    if !sc.unflyable() && !Self::flight_fault(sc, t) {
                        self.live("failsafe_lands_softly");
                        if vz > REQ_SOFT_TOUCHDOWN {
                            self.fail(t, "failsafe_lands_softly", format!("touched down at {vz:.1} m/s vertical"));
                        }
                    }
                }
                let dropping = sc.segments.iter().any(|s| s.action == Action::Drop && t >= s.start_us);
                self.stats.max_touchdown = self.stats.max_touchdown.max(speed);
                // A fall after a disarm is judged by disarm_justified; a
                // justified one (frozen IMU, say) falls however long it takes.
                let armed = before.armed || after.armed;
                if armed && !excused && !dropping && !wild && sc.segment_at(t).is_some() {
                    self.live("no_crash");
                    if speed > REQ_SOFT_TOUCHDOWN {
                        self.fail(t, "no_crash", format!("touched down at {speed:.1} m/s"));
                    }
                }
            }
        } else {
            self.airborne_speed = Some((world.body.vel.norm(), world.body.vel.z));
        }
        self.last_t = t;
    }

    pub fn finish(&mut self, _world: &World, _sc: &Scenario, _t: u64) {}
}
