//! IMU samples in: validation, calibration, filtering, and the attitude
//! estimate. Drives the controller when armed.

use crate::math::{Quat, Vec3};
use crate::{Effect, GRAVITY, ImuAlignment, Notice, Out, State, control, dyn_notch, rpm};

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct GyroCal {
    pub(crate) done: bool,
    pub(crate) start_us: u64,
    sum: Vec3,
    min: Vec3,
    max: Vec3,
    count: u32,
    pub(crate) bias: Vec3,
    /// Sum of accelerometer magnitudes, and their mean once done: what
    /// this sensor reads for 1 g at rest.
    accel_sum: f32,
    pub(crate) accel_1g: f32,
}

impl GyroCal {
    pub(crate) fn new(start_us: u64) -> Self {
        GyroCal {
            done: false,
            start_us,
            sum: Vec3::ZERO,
            min: Vec3::new(f32::MAX, f32::MAX, f32::MAX),
            max: Vec3::new(f32::MIN, f32::MIN, f32::MIN),
            count: 0,
            bias: Vec3::ZERO,
            accel_sum: 0.0,
            accel_1g: GRAVITY,
        }
    }
}

impl ImuAlignment {
    fn to_body(self, v: Vec3) -> Vec3 {
        // Sensor axes rotated clockwise (seen from above, about body z down).
        match self {
            ImuAlignment::Cw0 => v,
            ImuAlignment::Cw90 => Vec3::new(-v.y, v.x, v.z),
            ImuAlignment::Cw180 => Vec3::new(-v.x, -v.y, v.z),
            ImuAlignment::Cw270 => Vec3::new(v.y, -v.x, v.z),
        }
    }
}

/// eRPM older than this no longer places a motor's notches (µs).
const RPM_STALE_US: u64 = 20_000;

pub(crate) fn on_imu(s: &mut State, gyro: [f32; 3], accel: [f32; 3], fx: &mut Out) {
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
        // `step` keeps the ESCs fed with the last command until the IMU
        // timeout.
        fx.push(Effect::Notice(Notice::ImuSampleRejected));
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

    let saturated = g_raw.max_abs() >= s.cfg.gyro_range * 0.97;
    if saturated && !s.gyro_saturated {
        fx.push(Effect::Notice(Notice::GyroSaturated));
    }
    if s.gyro_saturated && !saturated {
        // The attitude estimate integrated clipped rates; lean on the
        // accelerometer until it has re-converged.
        s.fast_converge_until_us = now + 1_500_000;
        s.recovering_until_us = now + 5_000_000;
    }
    s.gyro_saturated = saturated;

    let g = s.cfg.imu_alignment.to_body(g_raw);
    let a = s.cfg.imu_alignment.to_body(a_raw);
    // Nominal sample spacing is 125..500 µs; clamp so one late or early
    // sample cannot blow up the integrators or the derivative.
    let reset_dynamics = gap || dt_us == 0 || dt_us > 5_000;
    let dt = (dt_us as f32 * 1e-6).clamp(50e-6, 2e-3);

    if !s.cal.done {
        calibrate(s, g, a, fx);
    }
    let g = g - s.cal.bias;
    let gyro_f = if reset_dynamics {
        s.rpm_filter.reset(g);
        s.dyn_notch.reset(g);
        s.gyro_lpf.y = g;
        g
    } else {
        s.imu_period = if s.imu_period == 0.0 { dt } else { s.imu_period + (dt - s.imu_period) * 0.01 };
        // A motor whose eRPM stopped coming loses its notches.
        for i in 0..4 {
            if s.rpm_at_us[i].is_some_and(|t| now.saturating_sub(t) > RPM_STALE_US) {
                s.rpm_at_us[i] = None;
                s.rpm_filter.set_motor(i, None, 0.0, 0, 0.0, 0.0);
            }
        }
        let rpm_on = s.cfg.rpm_filter_harmonics > 0 && s.rpm_at_us.iter().all(Option::is_some);
        let hz = if rpm_on { s.cfg.gyro_lpf_rpm_hz } else { s.cfg.gyro_lpf_hz };
        let notched = s.rpm_filter.apply(g);
        let min_bin = s.cfg.rpm_filter_min_hz * dyn_notch::N as f32 * s.imu_period;
        let notched = s.dyn_notch.apply(notched, !rpm_on && s.cfg.dyn_notch_q > 0.0, s.cfg.dyn_notch_q, min_bin);
        s.gyro_lpf.update(notched, hz, dt)
    };
    let accel_f = if reset_dynamics {
        s.accel_lpf.y = a;
        s.gyro_slow.y = g;
        a
    } else {
        s.gyro_slow.update(g, 10.0, dt);
        s.accel_lpf.update(a, 30.0, dt)
    };

    estimate_attitude(s, g, accel_f, dt);
    if s.armed {
        control::control(s, gyro_f, accel_f, dt, reset_dynamics, fx);
    } else {
        s.prev_gyro = gyro_f;
        s.dterm_lpf.y = Vec3::ZERO;
    }
}

/// Places each motor's RPM notches from its eRPM.
pub(crate) fn on_erpm(s: &mut State, erpm: &[Option<u32>; 4]) {
    let cfg = s.cfg;
    for (i, e) in erpm.iter().enumerate() {
        let Some(e) = e else { continue };
        s.rpm_at_us[i] = Some(s.now_us);
        if s.imu_period > 0.0 {
            let rotation_hz = *e as f32 / cfg.motor_pole_pairs / 60.0;
            let harmonics = (cfg.rpm_filter_harmonics as usize).min(rpm::MAX_HARMONICS);
            s.rpm_filter.set_motor(i, Some(rotation_hz), 1.0 / s.imu_period, harmonics, cfg.rpm_filter_q, cfg.rpm_filter_min_hz);
        }
    }
}

fn calibrate(s: &mut State, g: Vec3, a: Vec3, fx: &mut Out) {
    let c = &mut s.cal;
    c.sum += g;
    c.accel_sum += a.norm();
    c.count += 1;
    c.min = c.min.min(g);
    c.max = c.max.max(g);
    let spread = (c.max - c.min).max_abs();
    if spread > s.cfg.gyro_cal_motion * 2.0 {
        // Someone is moving the quad: start over.
        *c = GyroCal::new(s.now_us);
        return;
    }
    if s.now_us.saturating_sub(c.start_us) >= s.cfg.gyro_cal_us && c.count >= 100 {
        c.bias = c.sum * (1.0 / c.count as f32);
        c.accel_1g = c.accel_sum / c.count as f32;
        c.done = true;
        s.fast_converge_until_us = s.now_us + 1_000_000;
        fx.push(Effect::Notice(Notice::GyroCalibrated));
    }
}

/// Attitude reset on the ground (as in Betaflight): disarmed and still for
/// this long (µs), the estimate snaps to the accelerometer for
/// `GROUND_RESET_US`. Any movement starts it over, so righting a crashed
/// quad by hand or with turtle mode resets it again.
const GROUND_RESET_QUIET_US: u64 = 250_000;
const GROUND_RESET_US: u64 = 500_000;
/// Still this long (µs), the ground reset has finished.
pub(crate) const GROUND_SETTLED_US: u64 = GROUND_RESET_QUIET_US + GROUND_RESET_US;

/// Mahony complementary filter.
fn estimate_attitude(s: &mut State, gyro: Vec3, accel: Vec3, dt: f32) {
    let a_norm = accel.norm();
    let up_meas = accel * (1.0 / a_norm.max(1e-6));
    // Specific force is only a gravity reference when the quad is not
    // accelerating; trust it less the further it is from 1 g.
    let dev = (a_norm / GRAVITY - 1.0).abs();
    // In flight the accelerometer reads mostly thrust, which looks level
    // whatever the tilt while the quad speeds up or slows down, so a crash
    // can leave the estimate off for seconds. Sitting still on the ground
    // it reads pure gravity: trust it fully then.
    let still = s.cal.done && !s.armed && gyro.max_abs() < 15f32.to_radians() && dev < 0.2;
    let still_for = crate::held_for(&mut s.still_since_us, still, s.now_us);
    let ground_reset = (GROUND_RESET_QUIET_US..GROUND_RESET_QUIET_US + GROUND_RESET_US).contains(&still_for);
    let fast = s.now_us < s.fast_converge_until_us || !s.cal.done || ground_reset;
    // After calibration or gyro saturation the estimate may be far off, so
    // any roughly-1 g reading beats the gyro integral.
    let band = if fast { 0.5 } else { 0.1 };
    let weight = (1.0 - dev / band).clamp(0.0, 1.0);
    let kp: f32 = if ground_reset {
        25.0
    } else if fast {
        6.0
    } else if s.armed {
        0.5
    } else {
        2.0
    };
    let mut omega = gyro;
    if weight > 0.0 && !s.gyro_saturated {
        let up_est = s.attitude.up_in_body();
        let err = up_meas.cross(up_est);
        // After a saturation the fast window often passes while the pilot
        // punches out and the accelerometer reads thrust, far from 1 g.
        // For a few seconds more, a clean 1 g reading more than 10 degrees
        // from the estimate while the quad hardly turns is the estimate's
        // fault: pull at the fast gain. Only then: a punch at 1.3 g, or a
        // fast roll at 1 g, still reads level while tilted, and believing
        // that fast tips angle mode over.
        let recovering = s.now_us < s.recovering_until_us && dev < 0.1 && gyro.norm() < 1.0 && err.norm() > 0.17;
        let kp = if recovering { kp.max(6.0) } else { kp };
        if s.cal.done && !fast && !recovering {
            // Residual gyro bias from warm-up drift after calibration, which
            // builds over a minute. A push or a cruise reads level-looking
            // thrust near 1 g for a second or two; learning faster than the
            // drift took that error for bias (0.07 rad/s after one push
            // where the real residual was 0.003, seed 6018), which then kept
            // turning the estimate after the push.
            s.est_integral += err * (0.15 * weight * dt);
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
