//! Automatic disarms while flying: runaway takeoff (a build that cannot
//! fly) and crashes (tipped over and not righting itself).

use crate::math::Vec3;
use crate::{DisarmReason, GRAVITY, Out, State, disarm, held_for};

/// How long a frame must stay unblocked before its blocked time starts
/// over (µs).
const CRASH_BLOCKED_GAP_US: u64 = 500_000;

/// A frame that tipped over without the vertical-speed estimate showing a
/// fall faster than this (m/s) rolled over on the ground.
const TIPPED_FALL_MAX: f32 = 2.0;

/// How long (µs) a gyro clipped at its range, with the PIDs pushing
/// against the clipped rate, counts as a runaway.
const CLIPPED_RUNAWAY_US: u64 = 500_000;

/// How long (µs) the yaw output at its limit against the yaw rate counts as
/// a runaway.
const YAW_RUNAWAY_US: u64 = 250_000;

/// Climb (m/s, from `vz_est`) that shows an upright quad left the ground.
const LIFTOFF_CLIMB: f32 = 0.5;

/// Tilted further than this (rad), a quad is over: no flight mode holds it
/// there while it lies or is blocked.
const TIPPED_OVER: f32 = 60f32.to_radians();

pub(crate) fn runaway_and_crash(s: &mut State, gyro: Vec3, accel: Vec3, throttle: f32, dt: f32, fx: &mut Out) {
    let cfg = s.cfg;
    let now = s.now_us;
    if s.runaway_active {
        let lim = cfg.pid_sum_limit * cfg.runaway_pid_frac;
        let pid_high = s.flight.last_pid.x.abs() >= lim || s.flight.last_pid.y.abs() >= lim;
        let spinning = gyro.norm() > cfg.runaway_gyro;
        // A pilot's stick reversal saturates the PIDs while the frame still
        // turns the old way, but it slows down in the direction pushed. A
        // build that cannot fly speeds up away from the push. With the
        // sticks moved, only that counts; with them centred, any saturated
        // spin does (a tumble on the ground need not speed up).
        let flown = s.rc.input.roll.abs().max(s.rc.input.pitch.abs()).max(s.rc.input.yaw.abs()) > 0.15;
        let p = s.flight.last_pid;
        let pushed_back = s.ang_accel.x * p.x + s.ang_accel.y * p.y < 0.0;
        let runaway = pid_high && spinning && (pushed_back || !flown);
        let runaway_for = held_for(&mut s.flight.runaway_since_us, runaway, now);
        // Yaw on its own, at its lower output limit: all motors spinning the
        // wrong way (props in on a props-out mixer), or two motors of
        // opposite spin swapped, push yaw the wrong way while roll and pitch
        // may tumble every which way. Its acceleration is too noisy to judge
        // (vibration at full throttle, every bounce of a tumble), but a
        // build that flies stops even a full-stick yaw within about 0.1 s
        // once the yaw output is at its limit against it.
        let yaw_lim = cfg.pid_sum_limit * crate::control::YAW_PID_SHARE * cfg.runaway_pid_frac;
        let yaw_high = p.z.abs() >= yaw_lim;
        let yaw_opposed = yaw_high && gyro.z * p.z < 0.0 && gyro.z.abs() > cfg.runaway_gyro;
        let yaw_opposed_for = held_for(&mut s.flight.yaw_opposed_since_us, yaw_opposed, now);
        // A gyro clipped at its range reads a flat rate, so whether the
        // frame follows the push cannot be seen (an IMU axis with the wrong
        // sign spins it up past the range at once). Full stick asks for a
        // third of the range, so a build that flies turns back out of the
        // clip quickly; a clip the PIDs push against for longer than a
        // tumble on the ground holds it is a runaway.
        let clipped = (pid_high || yaw_high) && s.gyro_saturated && gyro.dot(p) < 0.0;
        let clipped_for = held_for(&mut s.flight.clipped_since_us, clipped, now);
        if (runaway && runaway_for >= cfg.runaway_us)
            || (yaw_opposed && yaw_opposed_for >= YAW_RUNAWAY_US)
            || (clipped && clipped_for >= CLIPPED_RUNAWAY_US)
        {
            disarm(s, DisarmReason::RunawayTakeoff, fx);
            s.runaway_tripped = true;
            return;
        }
        if !runaway {
            // Calm, level flight shows the build works, and so does a
            // pilot flying it with the sticks (as Betaflight counts it),
            // as long as it stays the right way up with the PIDs in range.
            // Throttle alone does not: a quad that cannot fly may sit on
            // the ground at part throttle until a gust tips it, and then
            // tumble about on it while the pilot works the sticks. Sitting
            // still and upright is calm too, so none of it counts until the
            // quad has lifted off: climbed upright, which the ground cannot
            // fake.
            let upright = s.attitude.tilt() < 30f32.to_radians();
            if upright && -s.flight.vz_est > LIFTOFF_CLIMB {
                s.flight.lifted_off = true;
            }
            if throttle > cfg.airmode_throttle && s.flight.lifted_off && upright && !pid_high && (!spinning || flown) {
                s.flight.runaway_ok_us += (dt * 1e6) as u64;
                if s.flight.runaway_ok_us >= cfg.runaway_deactivate_us {
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
    let slow = s.gyro_slow.y;
    let up = s.attitude.up_in_body();
    let rp_motion = (slow - up * slow.dot(up)).norm();
    let a = accel.norm();
    // An impact is a jolt no flight can produce. Thrust is at most
    // thrust/weight (about 1 / hover throttle squared) in g along the thrust
    // axis; drag reaches thrust/weight + 1 g after a power dive (cutting
    // the throttle at the bottom of an inverted dive brakes at that), and
    // adds to thrust along the thrust axis when pulling out of one.
    let h = s.hover_throttle.clamp(0.15, 0.9);
    let tw = 1.0 / (h * h);
    let margin = cfg.crash_impact_margin;
    let along_thrust = -accel.z;
    let across = Vec3::new(accel.x, accel.y, accel.z.max(0.0)).norm();
    let jolt = across > (tw + 1.0) * GRAVITY * margin || along_thrust > (2.0 * tw + 1.0) * GRAVITY * margin;
    if a > cfg.crash_impact && jolt {
        s.flight.last_impact_us = Some(now);
    }
    let propped = (0.7 * GRAVITY..1.3 * GRAVITY).contains(&a) && -accel.z / a < 0.64;
    // Spinning flat on the ground with the thrust and friction pulling
    // the attitude estimate a few degrees off, part of the spin reads as
    // roll and pitch; allow for about 15 degrees of that.
    let lying = tilt > TIPPED_OVER && rp_motion < 0.5 + 0.25 * slow.norm() && propped;
    // Blocked means not turning the way the sticks ask, whatever else it
    // does: a corkscrew dive rolls about the vertical, like a quad spinning
    // flat, but it follows the stick.
    let demand = Vec3::new(s.flight.setpoint.x, s.flight.setpoint.y, 0.0);
    let wants = demand.norm();
    let blocked = tilt > TIPPED_OVER && wants > 2.0 && slow.dot(demand) < wants;
    // Each condition times itself: a pilot rolling out of a slow inverted
    // dive turns `lying` into `blocked` for a moment, and must get the full
    // blocked time for the frame to follow.
    let lying_for = held_for(&mut s.flight.lying_since_us, lying, now);
    // Blocked adds up: a pilot slamming the sticks from side to side rocks
    // a stuck frame towards each new demand for a moment, which is not
    // getting free. Only righting itself or half a second unblocked is.
    if blocked {
        s.flight.blocked_us += (dt * 1e6) as u64;
        s.flight.blocked_last_us = now;
    } else if tilt <= TIPPED_OVER || now.saturating_sub(s.flight.blocked_last_us) >= CRASH_BLOCKED_GAP_US {
        s.flight.blocked_us = 0;
    }
    let blocked_for = if blocked { s.flight.blocked_us } else { 0 };
    // Or it rolled over on the ground: an inverted plunge only reads like
    // lying once drag holds it up, so it must have fallen several m/s
    // first, which the vertical-speed estimate sees. Tipping over where it
    // stood needs no jolt (a pilot flipping a quad that cannot take off).
    s.flight.tipped_fall = if tilt > TIPPED_OVER { s.flight.tipped_fall.max(s.flight.vz_est) } else { 0.0 };
    let rolled_over = s.flight.tipped_fall < TIPPED_FALL_MAX;
    let hit = s.flight.lying_since_us.is_some_and(|l| rolled_over || s.flight.last_impact_us.is_some_and(|t| t + cfg.crash_impact_window_us >= l));
    if (hit && lying_for >= cfg.crash_still_us) || blocked_for >= cfg.crash_blocked_us {
        disarm(s, DisarmReason::Crash, fx);
    }
}
