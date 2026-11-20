//! Protections: stop the motors when the IMU, the airframe, or the build
//! fails; never fly away; report a low pack and a stalled motor, and only
//! when they are real.

use ironloop_fc::math::Vec3;
use ironloop_fc::{BatteryLevel, DSHOT_MIN, Effect, EventKind, Notice};

use super::req::*;
use super::{Cx, Imu, Link, Property, Verdicts};
use crate::physics::{Esc, MotorHealth};
use crate::scenario::FaultKind;

pub static IMU_LOSS_STOPS: Property = Property {
    name: "imu_loss_stops",
    doc: "No fresh IMU data for 30 ms while armed stops the motors.",
};
pub static CRASH_STOPS: Property = Property {
    name: "crash_stops",
    doc: "A crashed quad (on the ground, tilted past 60 deg, not rotating toward upright) stops its motors within 4 s.",
};
pub static RUNAWAY_STOPS: Property = Property {
    name: "runaway_stops",
    doc: "A quad with its motors swapped or an IMU axis flipped never climbs above 1 m and stops within 5 s of throttle-up.",
};
pub static NO_ESC_STALL: Property = Property {
    name: "no_esc_stall",
    doc: "While armed, no ESC loses sync on a sound motor: idle keeps every motor fast enough (a stall stops it for at least 100 ms).",
};
pub static NO_FLYAWAY: Property = Property {
    name: "no_flyaway",
    doc: "The quad never goes above 60 m or further than 120 m from home.",
};
pub static BATTERY_CRITICAL_REPORTED: Property = Property {
    name: "battery_critical_reported",
    doc: "A 1S pack below 3.15 V for 5 s raises a critical battery notice.",
};
pub static BATTERY_NOTICE_TRUE: Property = Property {
    name: "battery_notice_true",
    doc: "Battery warning and critical notices only when the pack really was below 3.62 / 3.45 V in the last 2 s.",
};
pub static MOTOR_FAULT_REPORTED: Property = Property {
    name: "motor_fault_reported",
    doc: "A stalled motor that is commanded hard with telemetry available is reported within 200 ms.",
};
pub static MOTOR_FAULT_TRUE: Property = Property {
    name: "motor_fault_true",
    doc: "A motor fault notice only for a motor really stalled or weakened in the last 300 ms.",
};

#[derive(Clone, Debug, Default)]
pub struct Protect {
    inverted_since: Option<u64>,
    runaway_since: Option<u64>,
    crit_since: Option<u64>,
    crit_reported: bool,
    below_crit_v_us: Option<u64>,
    below_warn_v_us: Option<u64>,
    /// Per motor output (as the firmware numbers them), not per physical
    /// motor: with the wires swapped, output 0 drives motor 1.
    motor_bad_us: [Option<u64>; 4],
    stall_since: [Option<u64>; 4],
    fault_reported: [bool; 4],
    /// ESC stalls seen so far, per physical motor.
    esc_stalls: [u32; 4],
}

impl Protect {
    pub fn step(&mut self, cx: &Cx, link: &Link, imu: &Imu, v: &mut Verdicts) {
        let (t, before, after, world, sc, alt) = (cx.t, cx.before, cx.after, cx.world, cx.sc, cx.alt);
        self.notices(cx, v);

        let imu_stale = t.saturating_sub(imu.last_good_us);
        if before.armed && imu_stale > REQ_IMU_TOLERATE_US {
            v.live(&IMU_LOSS_STOPS);
        }
        if after.armed && imu_stale > REQ_IMU_STOP_US {
            v.fail(t, &IMU_LOSS_STOPS, format!("{} ms without good IMU data", imu_stale / 1000));
        }

        // Crash: tipped over on the ground and not righting itself (it may
        // still spin flat or skid).
        let w_world = world.body.att.rotate(world.body.omega);
        let stuck = Vec3::new(w_world.x, w_world.y, 0.0).norm() < 1.0;
        if after.armed && world.body.on_ground && cx.true_tilt > REQ_TIPPED_OVER && stuck {
            v.live(&CRASH_STOPS);
            let since = *self.inverted_since.get_or_insert(t);
            if t - since > REQ_CRASH_STOP_US {
                v.fail(t, &CRASH_STOPS, format!("armed, crashed at {:.0} deg for {} ms", cx.true_tilt.to_degrees(), (t - since) / 1000));
            }
        } else {
            self.inverted_since = None;
        }

        // Runaway: a quad that cannot fly must not be allowed to keep trying.
        if sc.runaway_build() && after.armed && link.last_rc_throttle > 1300 {
            v.live(&RUNAWAY_STOPS);
            let since = *self.runaway_since.get_or_insert(t);
            if t - since > REQ_RUNAWAY_STOP_US || alt > REQ_RUNAWAY_ALT {
                v.fail(t, &RUNAWAY_STOPS, format!("still armed {} ms after throttle-up, alt {alt:.1} m", (t - since) / 1000));
            }
        } else if !after.armed {
            self.runaway_since = None;
        }

        let dist = Vec3::new(world.body.pos.x, world.body.pos.y, 0.0).norm();
        v.stats.max_alt = v.stats.max_alt.max(alt);
        v.stats.max_dist = v.stats.max_dist.max(dist);
        if after.armed && cx.airborne {
            v.live(&NO_FLYAWAY);
        }
        if alt > 60.0 || dist > 120.0 {
            v.fail(t, &NO_FLYAWAY, format!("alt {alt:.1} m, dist {dist:.1} m"));
        }

        let volts = world.battery.volts;
        if !world.battery.absent && volts < REQ_CRITICAL_VOLTS && volts > 0.5 {
            v.live(&BATTERY_CRITICAL_REPORTED);
            let since = *self.crit_since.get_or_insert(t);
            if t - since > 5_000_000 && !self.crit_reported {
                v.fail(t, &BATTERY_CRITICAL_REPORTED, format!("{volts:.2} V for 5 s"));
            }
        } else {
            self.crit_since = None;
        }

        // A weakened motor (`MotorDegraded`) spins slower than idle asks
        // for and may lose sync; only a sound motor's stall is the idle's
        // fault (seeds 1324, 4285, 5303 and 7 more in 0-29999, each on the
        // degraded motor).
        let stalls = world.motors.each_ref().map(|m| m.stalls);
        if before.armed {
            v.live(&NO_ESC_STALL);
            for (i, m) in world.motors.iter().enumerate() {
                if stalls[i] > self.esc_stalls[i] && m.health == MotorHealth::Ok {
                    v.fail(t, &NO_ESC_STALL, format!("motor {i} lost sync at {:.2} V", world.battery.volts));
                }
            }
        }
        self.esc_stalls = stalls;

        // Stalled motor detection (needs bidirectional DShot telemetry).
        let telemetry_ok = sc.esc_telemetry && !sc.faults.iter().any(|f| f.active(t) && f.kind == FaultKind::TelemetryLoss);
        for i in 0..4 {
            let stalled = matches!(world.motors[sc.wired(i)].health, MotorHealth::Dead | MotorHealth::Desync);
            let hard = after.motors[i] > DSHOT_MIN + 800;
            if after.armed && stalled && hard && telemetry_ok {
                v.live(&MOTOR_FAULT_REPORTED);
                let since = *self.stall_since[i].get_or_insert(t);
                if t - since > REQ_MOTOR_FAULT_US && !self.fault_reported[i] {
                    v.fail(t, &MOTOR_FAULT_REPORTED, format!("motor {i} stalled {} ms", (t - since) / 1000));
                }
            } else {
                self.stall_since[i] = None;
            }
        }
    }

    /// Battery and motor-fault notices, judged against the truth just
    /// before them.
    fn notices(&mut self, cx: &Cx, v: &mut Verdicts) {
        let (t, world) = (cx.t, cx.world);
        if world.battery.volts < REQ_CRIT_TRUE_V {
            self.below_crit_v_us = Some(t);
        }
        if world.battery.volts < REQ_WARN_TRUE_V {
            self.below_warn_v_us = Some(t);
        }
        for i in 0..4 {
            // A fault, or the ESC starting or restarting the motor: either
            // way it is not spinning as commanded.
            let m = &world.motors[cx.sc.wired(i)];
            if m.health != MotorHealth::Ok || !matches!(m.esc, Esc::Running { .. }) {
                self.motor_bad_us[i] = Some(t);
            }
        }
        if cx.sc.esc_telemetry && cx.before.armed && matches!(cx.event.kind, EventKind::MotorTelemetry { .. }) {
            v.live(&MOTOR_FAULT_TRUE);
        }
        let recent = |since: Option<u64>| since.is_some_and(|s| t.saturating_sub(s) <= REQ_BATTERY_NOTICE_WINDOW_US);
        for e in cx.fx {
            let Effect::Notice(n) = e else { continue };
            v.stats.notices += 1;
            match *n {
                // A Warning after Critical reports the pack recovering.
                Notice::Battery(level)
                    if level != BatteryLevel::Ok && !(level == BatteryLevel::Warning && cx.before.battery_level == BatteryLevel::Critical) =>
                {
                    v.live(&BATTERY_NOTICE_TRUE);
                    let crit = level == BatteryLevel::Critical;
                    self.crit_reported |= crit;
                    let (seen, limit) = if crit { (self.below_crit_v_us, REQ_CRIT_TRUE_V) } else { (self.below_warn_v_us, REQ_WARN_TRUE_V) };
                    if !recent(seen) {
                        v.fail(t, &BATTERY_NOTICE_TRUE, format!("{level:?} with the pack at {:.2} V, not below {limit} V in 2 s", world.battery.volts));
                    }
                }
                Notice::MotorFault(i) => {
                    let i = i as usize;
                    self.fault_reported[i] = true;
                    if !self.motor_bad_us[i].is_some_and(|b| t.saturating_sub(b) <= REQ_MOTOR_FAULT_TRUE_US) {
                        v.fail(t, &MOTOR_FAULT_TRUE, format!("motor {i} reported faulty but healthy"));
                    }
                }
                _ => {}
            }
        }
    }
}
