//! The controller, run on every IMU sample while armed: vertical speed,
//! hover learning, setpoint, PID with I-term relax, mixer and airmode.

use crate::math::{Vec3, tilt_error, up_in_body};
use crate::{DSHOT_MAX, DSHOT_MIN, Effect, FailsafeStage, FlightMode, Out, Rates, State, THROTTLE_LOW, failsafe, protect};
use libm::roundf;

/// Motor mix (roll, pitch, yaw per motor) for QuadX in Betaflight motor
/// order, props out: M1 rear-right, M2 front-right, M3 rear-left, M4
/// front-left. FRD axes: positive roll is right side down, positive pitch
/// nose up, positive yaw nose right.
pub(crate) const MIX: [[f32; 3]; 4] = [
    [-1.0, -1.0, 1.0],
    [-1.0, 1.0, -1.0],
    [1.0, -1.0, -1.0],
    [1.0, 1.0, 1.0],
];

pub(crate) fn rate_curve(r: &Rates, stick: f32) -> f32 {
    let x = stick.clamp(-1.0, 1.0);
    let ax = x.abs();
    let x = x * ax * ax * ax * r.expo + x * (1.0 - r.expo);
    let super_factor = 1.0 / (1.0 - ax * r.super_rate).clamp(0.01, 1.0);
    let dps = (200.0 * r.rc_rate * x * super_factor).clamp(-1998.0, 1998.0);
    dps.to_radians()
}

/// DShot throttle value for a motor output fraction (0..1 of the range).
pub(crate) fn dshot(frac: f32) -> u16 {
    let v = DSHOT_MIN as f32 + roundf(frac * (DSHOT_MAX - DSHOT_MIN) as f32);
    (v as u16).clamp(DSHOT_MIN, DSHOT_MAX)
}

/// Motor speed for a throttle at the pack's voltage, relative to the same
/// throttle at the sag-compensation reference voltage: 1 while compensation
/// keeps up, less below about 3.1 V a cell where it is capped at 1.25x (a
/// sagging pack then needs more throttle to hover), more above the
/// reference (a fresh pack needs less).
pub(crate) fn sag_ratio(s: &State) -> f32 {
    let cfg = &s.cfg;
    match (cfg.vbat_sag_comp, s.battery_volts) {
        (true, Some(v)) if v > 0.5 => {
            let vref = cfg.vbat_ref_cell * cfg.battery_cells as f32;
            (vref / v).clamp(1.0, 1.25) * v / vref
        }
        _ => 1.0,
    }
}

pub(crate) fn control(s: &mut State, gyro: Vec3, accel: Vec3, dt: f32, reset: bool, fx: &mut Out) {
    let cfg = s.cfg;
    // Vertical motion from the accelerometer, against what it read for 1 g
    // on the ground. In normal flight the integral leaks, forgetting steady
    // motion over a few seconds (without a barometer a steady climb looks
    // like hover) but showing climbs and descents as they start and end.
    // It reads g * (1 - cos e) too low for an attitude estimate off by e,
    // which in hard maneuvering adds up to a m/s or two, but the leak
    // forgets that within a few seconds. Failsafe keeps the estimate it had
    // when the link was lost (the quad is often already climbing or
    // sinking; starting from zero would read that error for the whole
    // landing) and integrates without leaking.
    let down_accel = s.attitude.rotate(accel).z + s.cal.accel_1g;
    let leak = if s.failsafe == FailsafeStage::None { s.vz_est / 4.0 } else { 0.0 };
    s.vz_est += (down_accel - leak) * dt;
    let (roll, pitch, yaw, throttle) = match failsafe::throttle(s, dt) {
        None => (s.rc.roll, s.rc.pitch, s.rc.yaw, s.rc.throttle),
        Some(throttle) => (0.0, 0.0, 0.0, throttle),
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
    // In reference-voltage terms, so the failsafe flies the same hover on a
    // sagging pack.
    let at_ref = sag_ratio(s);
    if s.failsafe == FailsafeStage::None && s.airmode_active && level_flight && tilt < 0.5 && (0.1..0.95).contains(&throttle) {
        let level = throttle * at_ref * libm::sqrtf(libm::cosf(tilt));
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
            let e = tilt_error(s.attitude.up_in_body(), target) * cfg.angle_p;
            let lim = cfg.angle_rate_limit;
            Vec3::new(e.x.clamp(-lim, lim), e.y.clamp(-lim, lim), yaw_rate)
        }
    };

    if reset {
        s.prev_gyro = gyro;
        s.dterm_lpf.y = Vec3::ZERO;
    }
    let d_gyro = (gyro - s.prev_gyro) * (1.0 / dt);
    s.prev_gyro = gyro;
    let d_gyro = s.dterm_lpf.update(d_gyro, cfg.dterm_lpf_hz, dt);
    s.ang_accel = d_gyro;

    let err = setpoint - gyro;
    // No I-term at zero throttle: resting on the ground (or a wall) the
    // frame cannot move, the error never closes, and an integrator would
    // wind up and push the motors through airmode.
    let integrate = s.airmode_active && throttle > THROTTLE_LOW && !s.mixer_saturated && !s.gyro_saturated && !reset;
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
    let e = err.to_array();
    let d = d_gyro.to_array();
    let sp = setpoint.to_array();
    let mut integral = s.integral.to_array();
    let mut out = [0.0f32; 3];
    for i in 0..3 {
        let k = cfg.pid[i];
        if integrate {
            integral[i] = (integral[i] + k.i * e[i] * relax[i] * dt).clamp(-cfg.iterm_limit, cfg.iterm_limit);
        }
        // Yaw gets less: its authority comes out of roll and pitch's.
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
        (true, Some(v)) if v > 0.5 => (cfg.vbat_ref_cell * cfg.battery_cells as f32 / v).clamp(1.0, 1.25),
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
        s.motors[i] = dshot(f);
    }

    protect::runaway_and_crash(s, gyro, accel, throttle, dt, fx);
    if s.armed {
        fx.push(Effect::Motors(s.motors));
    }
}
