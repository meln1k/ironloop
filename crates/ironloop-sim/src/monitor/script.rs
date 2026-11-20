//! Stick scripts, flown open loop: in acro the rates track the sticks and
//! settle hands off; in angle mode each stick tilts the quad as far as the
//! angle limit says. Judged once the script segment ends.

use ironloop_fc::{FailsafeStage, FlightMode};

use super::req::*;
use super::{Cx, Link, Property, Verdicts};
use crate::scenario::{Action, stick_script};

pub static RATE_TRACKS: Property = Property {
    name: "rate_tracks",
    doc: "[flight] In an acro stick script each axis tracks the Betaflight-rate setpoint (30 ms lag) within 35% RMS.",
};
pub static SETTLES: Property = Property {
    name: "settles",
    doc: "[flight] Hands off in an acro stick script, roll and pitch rates settle below 0.4 rad/s RMS (no oscillation).",
};
pub static ANGLE_TRACKS: Property = Property {
    name: "angle_tracks",
    doc: "[flight] In an angle-mode stick script roll and pitch stick holds tilt the quad 75-115% of the commanded angle on average (55 deg at full stick).",
};

/// Running checks over one stick-script segment.
#[derive(Clone, Debug)]
struct Run {
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
    /// Until then the quad is still responding to sticks off the script,
    /// which is not scored.
    unscored_until_us: u64,
}

impl Run {
    fn new(start_us: u64, mode: FlightMode) -> Self {
        Run {
            start_us,
            mode,
            valid: true,
            reach: [[f32::MIN; 2]; 2],
            reference: [0.0; 3],
            err2: [0.0; 3],
            ref2: [0.0; 3],
            settle2: 0.0,
            settle_n: 0,
            off_script: 0.0,
            unscored_until_us: 0,
        }
    }

    fn judge(&self, t: u64, v: &mut Verdicts) {
        if !self.valid || self.off_script >= 0.1 {
            return;
        }
        match self.mode {
            // A hold spent entirely responding to corrupted frames has
            // nothing to judge (seed 2407).
            FlightMode::Angle if self.reach.iter().flatten().any(|&r| r == f32::MIN) => {}
            FlightMode::Angle => {
                v.live(&ANGLE_TRACKS);
                let r = self.reach.iter().flatten().sum::<f32>() / 4.0;
                if !(REQ_ANGLE_REACH.0..=REQ_ANGLE_REACH.1).contains(&r) {
                    v.fail(t, &ANGLE_TRACKS, format!("stick holds reached {:.0}% of the commanded angle on average", r * 100.0));
                }
            }
            FlightMode::Acro => {
                for (i, axis) in ["roll", "pitch", "yaw"].iter().enumerate() {
                    if self.ref2[i] > 0.0 {
                        v.live(&RATE_TRACKS);
                        let rel = libm::sqrtf(self.err2[i] / self.ref2[i]);
                        if rel > if i == 2 { REQ_YAW_RATE_ERR } else { REQ_RATE_ERR } {
                            v.fail(t, &RATE_TRACKS, format!("{axis} rate off by {:.0}% RMS", rel * 100.0));
                        }
                    }
                }
                if self.settle_n > 0 {
                    v.live(&SETTLES);
                    let rms = libm::sqrtf(self.settle2 / self.settle_n as f32);
                    if rms > REQ_SETTLED_RMS {
                        v.fail(t, &SETTLES, format!("hands-off roll/pitch rate {rms:.2} rad/s RMS"));
                    }
                }
            }
        }
    }

    fn sample(&mut self, cx: &Cx, link: &Link, into: u64, amp: f32) {
        let ch = |v: u16| {
            let x = (v as f32 - 1500.0) / 500.0;
            if x.abs() < 0.01 { 0.0 } else { x }
        };
        let held = stick_script(into);
        let mut script = [0.0f32; 3];
        if let Some((axis, k)) = held {
            script[axis] = amp * k;
        }
        let sticks = link.rc_sticks.map(ch);
        if (0..3).any(|i| (sticks[i] - script[i]).abs() > 0.02) {
            self.off_script += cx.imu_dt;
            self.unscored_until_us = cx.t + REQ_OFF_SCRIPT_RESPONSE_US;
        }
        let sp = [req_rate(sticks[0]), -req_rate(sticks[1]), req_rate(sticks[2])];
        let omega = cx.world.body.omega;
        let w = [omega.x, omega.y, omega.z];
        let k = (cx.imu_dt / REQ_RATE_TAU).min(1.0);
        for i in 0..3 {
            self.reference[i] += (sp[i] - self.reference[i]) * k;
        }
        if cx.t < self.unscored_until_us {
            return;
        }
        if let Some((axis, k)) = held {
            if axis <= 1 {
                // Stick right rolls right; stick forward pitches the nose
                // down.
                let (roll, pitch, _) = cx.world.body.att.to_euler();
                let angle = if axis == 0 { roll } else { -pitch };
                let r = &mut self.reach[axis][if k > 0.0 { 0 } else { 1 }];
                *r = r.max(angle / (amp * k * REQ_ANGLE_LIMIT));
            }
            let e = w[axis] - self.reference[axis];
            self.err2[axis] += e * e * cx.imu_dt;
            self.ref2[axis] += self.reference[axis] * self.reference[axis] * cx.imu_dt;
        }
        let ms = into / 1000;
        if (150..400).contains(&ms) || (3_300..3_800).contains(&ms) {
            self.settle2 += w[0] * w[0] + w[1] * w[1];
            self.settle_n += 1;
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct Script {
    run: Option<Run>,
}

impl Script {
    pub fn step(&mut self, cx: &Cx, link: &Link, v: &mut Verdicts) {
        let seg = cx.sc.segment_at(cx.t).copied();
        let script = seg.and_then(|g| match g.action {
            Action::StickScript { amp } => Some((g.start_us, amp)),
            _ => None,
        });
        if self.run.as_ref().is_some_and(|r| script.is_none_or(|(start, _)| start != r.start_us)) {
            self.run.take().unwrap().judge(cx.t, v);
        }
        let Some((start_us, amp)) = script else { return };
        let after = cx.after;
        let run = self.run.get_or_insert_with(|| Run::new(start_us, after.mode));
        // Angle mode tilts the quad as far as the estimate says; while the
        // estimate may be off after a gyro saturation, so may the tilt.
        let estimate_off = after.mode == FlightMode::Angle && cx.reconverging;
        if !cx.flying || cx.excused || estimate_off || after.failsafe != FailsafeStage::None || after.mode != run.mode || cx.alt < 1.0 {
            run.valid = false;
        }
        if cx.imu_step && run.valid {
            run.sample(cx, link, cx.t - start_us, amp);
        }
    }
}
