//! The virtual pilot: flies a scenario's plan of segments the way a person
//! would, from the true state of the quad, and reacts to what the firmware
//! does (a refused arm, a critical battery, a failsafe descent).

use ironloop_fc::math::{Vec3, tilt_error, up_in_body};
use ironloop_fc::rc::{RcInput, Switch};
use ironloop_fc::{FlightMode, RC_CHANNELS, Status};
use libm::{atanf, cosf, sinf};

use crate::physics::World;
use crate::rng::Rng;
use crate::scenario::{Action, Scenario};

/// Stick positions: roll, pitch, yaw in -1..1 and throttle in 0..1.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Sticks {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub throttle: f32,
    pub arm: bool,
    pub acro: bool,
}

impl Sticks {
    pub fn channels(&self) -> [u16; RC_CHANNELS] {
        RcInput {
            roll: self.roll,
            pitch: self.pitch,
            yaw: self.yaw,
            throttle: self.throttle,
            arm: if self.arm { Switch::High } else { Switch::Low },
            mode: if self.acro { FlightMode::Acro } else { FlightMode::Angle },
        }
        .channels()
    }
}

/// The virtual pilot: a human-like controller that flies the scenario's
/// plan and sees the true state.
#[derive(Clone, Debug, Default)]
pub struct Pilot {
    seg_start: u64,
    anchor: Vec3,
    alt_i: f32,
    hand: Sticks,
    slam: Sticks,
    slam_next: u64,
    flip_angle: f32,
    hang_since: Option<u64>,
    hang_done: bool,
    land_alt: f32,
    low_batt_land: bool,
    finishing_landing: bool,
    crit_since: Option<u64>,
}

impl Pilot {
    /// The sticks at `t`, for an RC frame every `dt` seconds.
    pub fn sticks(&mut self, sc: &Scenario, world: &World, status: &Status, rng: &mut Rng, t: u64, dt: f32) -> Sticks {
        let Some(seg) = sc.segment_at(t).copied() else {
            return Sticks::default();
        };
        let next_start = sc.segments.iter().map(|s| s.start_us).find(|&s| s > t).unwrap_or(u64::MAX);
        let w = world;
        let p = self;
        if p.seg_start != seg.start_us {
            p.seg_start = seg.start_us;
            p.anchor = w.body.pos;
            p.flip_angle = 0.0;
            p.hang_since = None;
            p.hang_done = false;
            p.land_alt = w.altitude();
        }
        // A pilot lands once the critical battery warning keeps sounding
        // (a punch-out alone can sag a 1S pack to critical for a moment).
        let critical = status.battery_level == ironloop_fc::BatteryLevel::Critical;
        p.crit_since = if critical && status.armed { Some(p.crit_since.unwrap_or(t)) } else { None };
        if p.crit_since.is_some_and(|c| t - c >= 3_000_000) && !p.low_batt_land {
            p.low_batt_land = true;
            p.land_alt = w.altitude();
            p.anchor = w.body.pos;
        }
        let mut seg = seg;
        if p.low_batt_land && !matches!(seg.action, Action::Disarmed { .. }) {
            seg.action = Action::Land;
        }
        // A pilot never flips the arm switch off while still in the air
        // (the plan's timeline can run out during a slow failsafe descent).
        if matches!(seg.action, Action::Disarmed { .. }) && status.armed && !w.body.on_ground && w.altitude() > 0.15 {
            if !p.finishing_landing {
                p.finishing_landing = true;
                p.land_alt = w.altitude();
                p.anchor = w.body.pos;
            }
            seg.action = Action::Land;
        }
        let acro = seg.mode == FlightMode::Acro;
        let alt = w.altitude();
        let vel = w.body.vel;
        let up = w.body.att.inverse_rotate(Vec3::new(0.0, 0.0, -1.0));
        let (_, _, yaw) = w.body.att.to_euler();
        let fc_cfg = &sc.fc;

        let hold_alt = |p: &mut Pilot, target: f32| {
            let e = (target - alt).clamp(-1.5, 1.5);
            // On the way down the trim only unwinds what a climb wound up:
            // integrating through a long descent winds up and turns a 15 m
            // descent into a 4 m/s dive, but trim left from a long climb
            // must go, or it holds the quad climbing above the target.
            if e > -0.5 {
                p.alt_i = (p.alt_i + 0.08 * e * dt).clamp(-0.25, 0.25);
            } else if p.alt_i > 0.0 {
                p.alt_i = (p.alt_i + 0.08 * e * dt).max(0.0);
            }
            let thr = 0.5 + p.alt_i + 0.14 * e + 0.09 * vel.z;
            let tilt = up.z.abs().max(0.5);
            (thr / tilt).clamp(0.05, 0.95)
        };
        // Desired tilt (roll, pitch) to drive horizontal velocity/position.
        let lean = |anchor: Option<Vec3>, roll0: f32, pitch0: f32| {
            let mut acc = -Vec3::new(vel.x, vel.y, 0.0) * 1.8;
            if let Some(a) = anchor {
                let e = a - w.body.pos;
                acc += Vec3::new(e.x, e.y, 0.0) * 1.2;
            }
            let lim = 5.0;
            let n = acc.norm();
            if n > lim {
                acc = acc * (lim / n);
            }
            let (c, s) = (cosf(yaw), sinf(yaw));
            let fwd = acc.x * c + acc.y * s;
            let right = -acc.x * s + acc.y * c;
            (roll0 + atanf(right / 9.81), pitch0 - atanf(fwd / 9.81))
        };
        // Convert desired attitude to sticks for the current mode.
        let attitude_sticks = |roll: f32, pitch: f32| -> (f32, f32) {
            if acro {
                let rate = tilt_error(up, up_in_body(roll, pitch)) * 5.0 - w.body.omega * 0.08;
                (inverse_rate(&fc_cfg.rates[0], rate.x), -inverse_rate(&fc_cfg.rates[1], rate.y))
            } else {
                (roll / fc_cfg.angle_limit, -pitch / fc_cfg.angle_limit)
            }
        };

        // A refused or lost arm (still calibrating, glitchy link, crash)
        // makes the pilot flip the switch off and on again.
        let rearm = status.armed || (t - seg.start_us) % 1_000_000 < 700_000;
        let mut out = Sticks { arm: rearm, acro, ..Sticks::default() };
        // Disarmed when it should be flying (arming refused while still
        // calibrating, or a safety disarm): throttle down and cycle the
        // switch until it arms, then carry on with the plan.
        let action = if seg.action.is_flight() && !status.armed { Action::Arm { throttle: 0.0 } } else { seg.action };
        match action {
            Action::Disarmed { throttle } => {
                out.arm = sc.arm_switch_at_boot && t + 500_000 < next_start && seg.start_us == 0;
                out.throttle = throttle;
                p.alt_i = 0.0;
            }
            Action::Arm { throttle } => {
                out.throttle = throttle;
            }
            Action::Hover { alt: target } => {
                out.throttle = if w.body.on_ground && t - seg.start_us < 300_000 {
                    0.6
                } else {
                    hold_alt(p, target)
                };
                let (r, pt) = lean(Some(p.anchor), 0.0, 0.0);
                (out.roll, out.pitch) = attitude_sticks(r, pt);
            }
            Action::Cruise { alt: target, roll, pitch } => {
                out.throttle = hold_alt(p, target);
                (out.roll, out.pitch) = attitude_sticks(roll, pitch);
                p.anchor = w.body.pos;
            }
            Action::Flip { axis, dir } => {
                let rate = if axis == 0 { w.body.omega.x } else { w.body.omega.y };
                p.flip_angle += (rate * dir).abs() * dt;
                if t - seg.start_us < 150_000 {
                    out.throttle = 0.8;
                } else if p.flip_angle < 5.4 {
                    out.throttle = 0.15;
                    if axis == 0 { out.roll = dir } else { out.pitch = dir }
                } else {
                    out.throttle = hold_alt(p, 2.5);
                    let (r, pt) = lean(None, 0.0, 0.0);
                    (out.roll, out.pitch) = attitude_sticks(r, pt);
                    p.anchor = w.body.pos;
                }
            }
            Action::InvertedHang { secs, yaw } => {
                if !p.hang_done {
                    match p.hang_since {
                        None if up.z > 0.95 => p.hang_since = Some(t),
                        None => {
                            out.roll = 1.0;
                            out.throttle = 0.3;
                        }
                        Some(h) if t - h >= (secs * 1e6) as u64 => p.hang_done = true,
                        Some(_) => {
                            // A little throttle keeps control authority.
                            out.yaw = yaw;
                            out.throttle = 0.12;
                        }
                    }
                }
                if p.hang_done {
                    if up.z > -0.7 {
                        // Keep rolling the same way, back upright.
                        out.roll = 1.0;
                        out.throttle = 0.3;
                    } else {
                        // Recover above where the longest hang ends.
                        out.throttle = hold_alt(p, 15.0);
                        let (r, pt) = lean(None, 0.0, 0.0);
                        (out.roll, out.pitch) = attitude_sticks(r, pt);
                        p.anchor = w.body.pos;
                    }
                }
            }
            Action::StickScript { amp } => {
                // Open loop on attitude: only the throttle reacts (to hold
                // altitude); the sticks follow the script.
                out.throttle = hold_alt(p, p.land_alt.max(3.0));
                if let Some((axis, k)) = crate::scenario::stick_script(t - seg.start_us) {
                    match axis {
                        0 => out.roll = amp * k,
                        1 => out.pitch = amp * k,
                        _ => out.yaw = amp * k,
                    }
                }
                p.anchor = w.body.pos;
            }
            Action::Punch => {
                out.throttle = 1.0;
                p.anchor = w.body.pos;
            }
            Action::StickSlam => {
                if t >= p.slam_next {
                    let r = &mut *rng;
                    p.slam = Sticks {
                        roll: r.range(-1.0, 1.0),
                        pitch: r.range(-1.0, 1.0),
                        yaw: r.range(-1.0, 1.0),
                        throttle: r.range(0.25, 0.9),
                        ..out
                    };
                    p.slam_next = t + 150_000;
                }
                out = p.slam;
                if alt < 1.0 {
                    out.throttle = 0.8;
                }
                p.anchor = w.body.pos;
            }
            Action::Land => {
                p.land_alt = (p.land_alt - 0.6 * dt).max(-0.5);
                out.throttle = if w.body.on_ground && alt < 0.05 { 0.0 } else { hold_alt(p, p.land_alt) };
                let (r, pt) = lean(Some(p.anchor), 0.0, 0.0);
                (out.roll, out.pitch) = attitude_sticks(r, pt);
            }
            Action::Drop => out.throttle = 0.0,
        }
        // The pilot's hands move fast but not instantly.
        let k = (dt / 0.03).min(1.0);
        let h = &mut p.hand;
        h.roll += (out.roll - h.roll) * k;
        h.pitch += (out.pitch - h.pitch) * k;
        h.yaw += (out.yaw - h.yaw) * k;
        h.throttle = out.throttle;
        h.arm = out.arm;
        h.acro = out.acro;
        if seg.action.snaps_sticks() {
            *h = out;
        }
        *h
    }
}


/// Stick deflection that produces `rate` (rad/s) with Betaflight rates.
fn inverse_rate(r: &ironloop_fc::Rates, rate: f32) -> f32 {
    let target = rate.to_degrees().abs();
    let (mut lo, mut hi) = (0.0f32, 1.0f32);
    for _ in 0..20 {
        let mid = 0.5 * (lo + hi);
        let x = mid * mid * mid * mid * r.expo + mid * (1.0 - r.expo);
        let v = 200.0 * r.rc_rate * x / (1.0 - mid * r.super_rate).clamp(0.01, 1.0);
        if v < target { lo = mid } else { hi = mid }
    }
    0.5 * (lo + hi) * rate.signum()
}
