//! Failsafe after the RC link is lost: hold altitude at the learned hover
//! throttle, then land open loop and disarm on touchdown.

use crate::control::sag_ratio;
use crate::{DisarmReason, Effect, FailsafeStage, GRAVITY, Notice, Out, State, THROTTLE_LOW, disarm};

/// Throttle the failsafe flies at (sticks centered, self-leveling), or
/// `None` while the pilot flies. Runs on every control step.
pub(crate) fn throttle(s: &mut State, dt: f32) -> Option<f32> {
    if s.failsafe == FailsafeStage::Hold && s.hold_vz_mark.is_none() && s.now_us.saturating_sub(s.failsafe_since_us) >= 500_000 {
        s.hold_vz_mark = Some(s.vz_est);
    }
    match s.failsafe {
        FailsafeStage::None => None,
        // Hold altitude at the learned hover throttle, then sink at about
        // 1-2 m/s at 96% of it (thrust grows with throttle squared, so about
        // 92% of hover thrust). The hold also measures how far the learned
        // hover throttle is off (see `advance`). During the landing a
        // fast sink brakes; the integrated accelerometer drifts (vibration),
        // so the brake's ceiling falls from 115% of hover to hover itself
        // over the first 4 s: a drifting estimate can then at most hold the
        // quad, and the trim below still takes it down. Below hover nothing
        // could stop a long descent speeding up.
        // Throttle at the bottom when the link went means the quad is on the
        // ground (or the pilot cut it): never spin up from there.
        FailsafeStage::Hold if s.throttle_at_loss < THROTTLE_LOW => Some(0.0),
        FailsafeStage::Hold => Some(s.hover_throttle / sag_ratio(s)),
        FailsafeStage::Landing => {
            let landing_s = s.now_us.saturating_sub(s.failsafe_since_us) as f32 * 1e-6;
            let ceiling = (1.15 - 0.04 * landing_s).max(1.0);
            if s.now_us.saturating_sub(s.landing_vz_ref.0) >= 250_000 {
                s.landing_accel = (s.vz_est - s.landing_vz_ref.1) / 0.25;
                s.landing_vz_ref = (s.now_us, s.vz_est);
            }
            // Not sinking, or not speeding up although throttle is being
            // taken off: take more off, steadily. In the air that soon
            // starts a descent (a learned hover throttle far too high cannot
            // keep it climbing) and then speeds it up, which stops the trim;
            // on the ground nothing changes, and once thrust is well below
            // hover `advance` calls it landed.
            if s.landing_trim == 0.0 {
                s.landing_trim_vz = s.vz_est;
            }
            s.landing_trim = if s.vz_est < 0.25 || unmoved(s) {
                (s.landing_trim + 0.25 * dt).min(0.35)
            } else {
                (s.landing_trim - 2.0 * dt).max(0.0)
            };
            let brake = 0.15 * (s.vz_est - 1.2).max(0.0);
            // A descent still speeding up gets up to 8% over the ceiling.
            // This brakes on acceleration over a quarter second, which the
            // estimate's slow drift cannot fake, so it cannot hold the quad
            // up; it stops the trim's probes from adding up, and a learned
            // hover throttle that is too low from running away. Only upright:
            // a tumbling quad's estimate is meaningless.
            let speeding = if s.attitude.tilt() < 0.5 { (0.04 * (s.landing_accel - 0.3)).clamp(0.0, 0.08) } else { 0.0 };
            let k = (0.96 + brake).min(ceiling) + speeding - s.landing_trim;
            Some(s.hover_throttle * k / sag_ratio(s))
        }
    }
}

/// Landing and neither speeding up nor sinking faster since the trim
/// started. The speed estimate itself drifts by a m/s or more over a long
/// landing, so this uses only how it changes.
fn unmoved(s: &State) -> bool {
    s.landing_accel.abs() < 0.6 && s.vz_est - s.landing_trim_vz < 0.5
}

/// Stage changes, run after every event.
pub(crate) fn advance(s: &mut State, fx: &mut Out) {
    let now = s.now_us;
    let cfg = s.cfg;
    let lost = s.rc_lost();
    let recovered = !lost
        && s.rc.last_valid_us.is_some()
        && now.saturating_sub(s.rc.valid_since_us) >= cfg.failsafe_recovery_us;
    match s.failsafe {
        FailsafeStage::None if lost && s.armed => {
            s.failsafe = FailsafeStage::Hold;
            s.failsafe_since_us = now;
            s.hold_vz_mark = None;
            s.throttle_at_loss = s.rc.throttle;
            fx.push(Effect::Notice(Notice::Failsafe(FailsafeStage::Hold)));
        }
        FailsafeStage::Hold | FailsafeStage::Landing if recovered => {
            s.failsafe = FailsafeStage::None;
            fx.push(Effect::Notice(Notice::FailsafeRecovered));
            if !s.rc.arm_switch {
                disarm(s, DisarmReason::Switch, fx);
            }
        }
        FailsafeStage::Hold if now - s.failsafe_since_us >= cfg.failsafe_stage1_us => {
            if !s.airmode_active || s.throttle_at_loss < THROTTLE_LOW {
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
                fx.push(Effect::Notice(Notice::Failsafe(FailsafeStage::Landing)));
            }
        }
        FailsafeStage::Landing => {
            // Touchdown: not accelerating with the throttle trimmed to about
            // half of hover thrust, which in the air would drop the quad at
            // 5 m/s². The accelerometer magnitude is no use here: with a
            // bent prop it reads several g of vibration in flight.
            if (s.landing_trim >= 0.3 && unmoved(s)) || now - s.failsafe_since_us >= cfg.failsafe_landing_us {
                disarm(s, DisarmReason::Failsafe, fx);
            }
        }
        _ => {}
    }
}
