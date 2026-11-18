//! Flight expectations in normal flight: the attitude estimate tracks the
//! truth and comes back after losing it, angle mode stays level, landings
//! are soft, and sag compensation keeps the motors where the stick asks.

use ironloop_fc::math::Vec3;
use ironloop_fc::{DSHOT_MAX, DSHOT_MIN, FailsafeStage, FlightMode};

use super::req::*;
use super::{Cx, Link, Property, Verdicts};
use crate::scenario::Action;

pub static ESTIMATE_TRACKS: Property = Property {
    name: "estimate_tracks",
    doc: "[flight] Estimated tilt stays within 15 deg of the truth while flying, except within 4 s of the gyro saturating.",
};
pub static ESTIMATE_RECONVERGES: Property = Property {
    name: "estimate_reconverges",
    doc: "[flight] A tilt estimate off by over 15 deg (after a flip, say) is back within 10 deg within 4 s.",
};
pub static ANGLE_STAYS_LEVEL: Property = Property {
    name: "angle_stays_level",
    doc: "[flight] In angle mode the true tilt stays under 75 deg.",
};
pub static NO_CRASH: Property = Property {
    name: "no_crash",
    doc: "[flight] Armed touchdowns are slower than 2.5 m/s, outside deliberate drops, flips, and stick slams.",
};
pub static SAG_COMPENSATED: Property = Property {
    name: "sag_compensated",
    doc: "[flight] On a sagging pack the averaged motor speed matches the throttle stick within 15%.",
};

#[derive(Clone, Debug, Default)]
pub struct Flight {
    est_bad_since: Option<u64>,
    /// Last time the true rotation rate was beyond the gyro's range.
    gyro_saturated_us: Option<u64>,
    /// True pack voltage averaged over about 0.5 s.
    volts_avg: f32,
    sag: SagWindow,
}

/// The sag check's window, integrated over IMU steps.
#[derive(Clone, Copy, Debug, Default)]
struct SagWindow {
    secs: f32,
    /// Motor speed as a fraction of full.
    spin: f32,
    /// What the stick asks for at the reference voltage.
    want: f32,
}

impl Flight {
    pub fn step(&mut self, cx: &Cx, link: &Link, v: &mut Verdicts) {
        let (t, after, world) = (cx.t, cx.after, cx.world);

        // How far the estimated tilt is off the truth, on IMU steps in
        // flight with the expectations on.
        let est_err = (cx.flying && !cx.excused && cx.imu_step).then(|| {
            let up = Vec3::new(0.0, 0.0, -1.0);
            let est_up = after.attitude.inverse_rotate(up);
            let true_up = world.body.att.inverse_rotate(up);
            libm::acosf(est_up.dot(true_up).clamp(-1.0, 1.0)).to_degrees()
        });

        // A rotation beyond the gyro's range loses the attitude, and in
        // flight the accelerometer reads mostly thrust, so the estimate
        // takes a few seconds to come back (Betaflight's too); until then
        // `estimate_reconverges` judges it instead.
        if world.body.omega.max_abs() >= cx.sc.imu.gyro_range * 0.97 {
            self.gyro_saturated_us = Some(t);
        }
        let recovering = self.gyro_saturated_us.is_some_and(|s| t - s < REQ_RECONVERGE_US);
        if let Some(err) = est_err.filter(|_| !cx.wild && !recovering) {
            v.live(&ESTIMATE_TRACKS);
            v.stats.max_est_err_deg = v.stats.max_est_err_deg.max(err);
            if err > REQ_ESTIMATE_DEG {
                v.fail(t, &ESTIMATE_TRACKS, format!("tilt estimate off by {err:.1} deg"));
            }
            if after.mode == FlightMode::Angle {
                v.live(&ANGLE_STAYS_LEVEL);
                let tilt = cx.true_tilt.to_degrees();
                v.stats.max_tilt_angle_mode_deg = v.stats.max_tilt_angle_mode_deg.max(tilt);
                if cx.true_tilt > REQ_ANGLE_TILT {
                    v.fail(t, &ANGLE_STAYS_LEVEL, format!("tilt {tilt:.0} deg in angle mode"));
                }
            }
        }

        // Re-convergence, including during flips and slams.
        match est_err {
            Some(err) => {
                if err > REQ_ESTIMATE_DEG {
                    self.est_bad_since.get_or_insert(t);
                } else if err < REQ_ESTIMATE_OK_DEG {
                    self.est_bad_since = None;
                }
                if let Some(b) = self.est_bad_since {
                    v.live(&ESTIMATE_RECONVERGES);
                    if t - b > REQ_RECONVERGE_US {
                        v.fail(t, &ESTIMATE_RECONVERGES, format!("tilt estimate off by {err:.1} deg for {} ms", (t - b) / 1000));
                    }
                }
            }
            None => self.est_bad_since = None,
        }

        self.sag(cx, link, v);

        v.stats.max_impact_g = v.stats.max_impact_g.max(world.body.impact / 9.80665);
        if let Some((speed, _)) = cx.touchdown {
            v.stats.max_touchdown = v.stats.max_touchdown.max(speed);
            let dropping = cx.sc.segments.iter().any(|s| s.action == Action::Drop && t >= s.start_us);
            // A fall after a disarm is judged by disarm_justified; a
            // justified one (frozen IMU, say) falls however long it takes.
            let armed = cx.before.armed || after.armed;
            if armed && !cx.excused && !dropping && !cx.wild && cx.sc.segment_at(t).is_some() {
                v.live(&NO_CRASH);
                if speed > REQ_SOFT_TOUCHDOWN {
                    v.fail(t, &NO_CRASH, format!("touched down at {speed:.1} m/s"));
                }
            }
        }
    }

    /// On a sagging pack, the motors spin as fast on average as the stick
    /// asks for at the reference voltage.
    fn sag(&mut self, cx: &Cx, link: &Link, v: &mut Verdicts) {
        if !cx.imu_step {
            return;
        }
        let (world, after) = (cx.world, cx.after);
        let volts = world.battery.volts;
        self.volts_avg += (volts - self.volts_avg) * (cx.imu_dt / 0.5).min(1.0);
        // Only on a steady pack: right after a punch-out it rebounds faster
        // than any reasonable voltage filter follows.
        let steady = (volts - self.volts_avg).abs() < 0.05;
        let stick_thr = (link.last_rc_throttle as f32 - 1000.0) / 1000.0;
        let cmd_ok = after.motors.iter().all(|&m| m > DSHOT_MIN + 170 && m < DSHOT_MAX - 60);
        let judged = cx.flying
            && !cx.excused
            && !cx.wild
            && after.failsafe == FailsafeStage::None
            && (0.15..0.85).contains(&stick_thr)
            && cmd_ok
            && world.body.omega.norm() < 3.0
            && !world.battery.absent
            && volts < REQ_SAG_BELOW_V
            && volts > REQ_SAG_ABOVE_V
            && steady;
        if !judged {
            self.sag = SagWindow::default();
            return;
        }
        let s = &mut self.sag;
        s.secs += cx.imu_dt;
        s.spin += world.motors.iter().map(|m| m.omega).sum::<f32>() / (4.0 * world.frame.omega_max) * cx.imu_dt;
        s.want += (REQ_IDLE * volts / REQ_SAG_REF_V + (1.0 - REQ_IDLE) * stick_thr) * cx.imu_dt;
        if s.secs >= 0.5 {
            v.live(&SAG_COMPENSATED);
            let ratio = s.spin / s.want;
            if (ratio - 1.0).abs() > REQ_SAG_TOL {
                v.fail(cx.t, &SAG_COMPENSATED, format!("motors at {:.0}% of what the stick asks at {volts:.2} V", ratio * 100.0));
            }
            self.sag = SagWindow::default();
        }
    }
}
