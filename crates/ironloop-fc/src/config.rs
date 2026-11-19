//! Tunable parameters. Defaults target the HDZero Mobula6 ECO 2024
//! (31 g, 1S, 0702 28000KV, 31 mm props).

/// How the IMU chip is mounted relative to the body frame, Betaflight style
/// (clockwise rotation about the body z axis, seen from above).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImuAlignment {
    Cw0,
    Cw90,
    Cw180,
    Cw270,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rates {
    /// Betaflight `rc_rate` (center sensitivity, 1.0 = 200 deg/s).
    pub rc_rate: f32,
    /// Betaflight `super_rate`.
    pub super_rate: f32,
    /// Betaflight `rc_expo`.
    pub expo: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AxisPid {
    /// Output per rad/s of rate error.
    pub p: f32,
    /// Output per rad of integrated rate error.
    pub i: f32,
    /// Output per rad/s² of gyro change (derivative on measurement).
    pub d: f32,
    /// Output per rad/s of setpoint (static feedforward).
    pub ff: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Config {
    pub imu_alignment: ImuAlignment,
    /// Gyro full-scale range in rad/s; samples at or beyond 97% count as
    /// saturated.
    pub gyro_range: f32,
    /// Accelerometer full-scale range in m/s².
    pub accel_range: f32,
    pub gyro_lpf_hz: f32,
    /// Gyro low-pass cutoff while the RPM filter has fresh eRPM from all
    /// four motors: the notches take out prop vibration, so the low-pass
    /// can let more through and lag less.
    pub gyro_lpf_rpm_hz: f32,
    /// RPM filter: harmonics notched per motor (0 turns it off), the
    /// notches' Q, and the lowest frequency notched (Hz).
    pub rpm_filter_harmonics: u8,
    pub rpm_filter_q: f32,
    pub rpm_filter_min_hz: f32,
    /// Motor magnet pole pairs (eRPM / pole pairs = RPM).
    pub motor_pole_pairs: f32,
    /// Largest believable change between consecutive gyro samples (rad/s).
    pub gyro_spike: f32,
    pub dterm_lpf_hz: f32,

    pub rates: [Rates; 3],
    pub pid: [AxisPid; 3],
    pub iterm_limit: f32,
    pub pid_sum_limit: f32,
    /// Angle mode: maximum commanded tilt (rad).
    pub angle_limit: f32,
    /// Angle mode: rate command per rad of tilt error.
    pub angle_p: f32,
    /// Angle mode: rate command cap (rad/s).
    pub angle_rate_limit: f32,

    /// Motor idle as a fraction of the DShot range while armed.
    pub motor_idle: f32,
    /// Throttle above which airmode (full PID authority, I-term) engages.
    pub airmode_throttle: f32,
    // Battery voltages are for the whole pack, which on this quad is 1S.
    /// Throttle scaling reference voltage (sag compensation).
    pub vbat_ref: f32,
    pub vbat_warn: f32,
    pub vbat_crit: f32,
    /// Below this a battery counts as absent (USB power) and blocks arming.
    pub vbat_min_arm: f32,

    /// RC is lost after this long without a valid frame (µs).
    pub rc_timeout_us: u64,
    /// Failsafe stage 1 length before landing (µs).
    pub failsafe_stage1_us: u64,
    /// Longest failsafe landing (stage 2) before disarming (µs); touchdown
    /// disarms sooner.
    pub failsafe_landing_us: u64,
    /// Hover throttle assumed until one is learned in flight. Failsafe
    /// holds at the hover throttle, then lands at about 96% of it,
    /// corrected to sink at about 1 m/s.
    pub hover_throttle_guess: f32,
    /// Valid RC needed to leave failsafe (µs).
    pub failsafe_recovery_us: u64,
    /// IMU silence or invalid data longer than this stops the motors (µs).
    pub imu_timeout_us: u64,
    /// Arming is blocked while tilted more than this (rad).
    pub max_arm_tilt: f32,
    /// Time after boot during which the gyro is calibrated (µs).
    pub gyro_cal_us: u64,
    /// Maximum gyro motion allowed during calibration (rad/s).
    pub gyro_cal_motion: f32,

    /// Runaway takeoff: PID sum fraction that counts as runaway.
    pub runaway_pid_frac: f32,
    /// Runaway takeoff: gyro magnitude that counts as runaway (rad/s).
    pub runaway_gyro: f32,
    /// Runaway takeoff: how long both must hold (µs).
    pub runaway_us: u64,
    /// Runaway detection turns off after this much calm, level flight
    /// above `airmode_throttle` with PIDs in range (µs).
    pub runaway_deactivate_us: u64,
    /// Lying tilted over and still, propped up by the ground, for this long
    /// while armed disarms (µs), if an impact came first (see below).
    pub crash_still_us: u64,
    /// Filtered acceleration above this (m/s^2) counts as an impact. Thrust
    /// alone stays below it.
    pub crash_impact: f32,
    /// An impact must also exceed what thrust and drag can produce in
    /// flight (from the learned hover throttle) by this factor.
    pub crash_impact_margin: f32,
    /// Lying only counts as a crash if an impact came at most this long
    /// before it began (µs).
    pub crash_impact_window_us: u64,
    /// Tilted over and not rotating although the sticks or angle mode demand
    /// it, for this long while armed, disarms (µs).
    pub crash_blocked_us: u64,

    /// Motor considered failed if its eRPM stays below this fraction of the
    /// expected value for `motor_fault_us`.
    pub motor_fault_frac: f32,
    pub motor_fault_us: u64,
    /// eRPM expected at full command on a full battery.
    pub motor_max_erpm: f32,
}

impl Default for Config {
    fn default() -> Self {
        // Tuned against open-loop stick doublets in the simulator (the
        // `rate_tracks` property): feedforward overshot the stick by half
        // again, so roll and pitch run on P and D alone.
        let roll_pitch = AxisPid { p: 0.0286, i: 0.14, d: 0.000525, ff: 0.0 };
        Self {
            imu_alignment: ImuAlignment::Cw0,
            gyro_range: 2000.0f32.to_radians(),
            accel_range: 16.0 * 9.80665,
            gyro_lpf_hz: 150.0,
            // In the sim, the notches plus 200 Hz leave less motor noise
            // than 150 Hz alone, and lag the gyro no more.
            gyro_lpf_rpm_hz: 200.0,
            rpm_filter_harmonics: 3,
            rpm_filter_q: 8.0,
            rpm_filter_min_hz: 100.0,
            // SE0702: 12 poles.
            motor_pole_pairs: 6.0,
            gyro_spike: 10.0,
            dterm_lpf_hz: 90.0,
            rates: [
                Rates { rc_rate: 1.0, super_rate: 0.7, expo: 0.0 },
                Rates { rc_rate: 1.0, super_rate: 0.7, expo: 0.0 },
                Rates { rc_rate: 1.0, super_rate: 0.7, expo: 0.0 },
            ],
            pid: [roll_pitch, roll_pitch, AxisPid { p: 0.1, i: 0.2, d: 0.0, ff: 0.005 }],
            iterm_limit: 0.25,
            pid_sum_limit: 0.6,
            angle_limit: 55.0f32.to_radians(),
            angle_p: 8.0,
            angle_rate_limit: 600.0f32.to_radians(),
            // Bluejay hands the motor to the FC once it runs, and too low an
            // idle then stalls it. The factory Betaflight setup uses 10%.
            motor_idle: 0.10,
            airmode_throttle: 0.25,
            vbat_ref: 3.9,
            vbat_warn: 3.45,
            vbat_crit: 3.3,
            vbat_min_arm: 3.3,
            rc_timeout_us: 100_000,
            failsafe_stage1_us: 1_000_000,
            // No barometer: the descent is open loop at about 1-2 m/s, so
            // allow enough time to come down from ~25 m.
            failsafe_landing_us: 30_000_000,
            hover_throttle_guess: 0.45,
            failsafe_recovery_us: 300_000,
            imu_timeout_us: 20_000,
            max_arm_tilt: 25.0f32.to_radians(),
            gyro_cal_us: 1_000_000,
            gyro_cal_motion: 6.0f32.to_radians(),
            runaway_pid_frac: 0.9,
            runaway_gyro: 20.0f32.to_radians(),
            runaway_us: 100_000,
            runaway_deactivate_us: 1_000_000,
            crash_still_us: 3_000_000,
            crash_impact: 4.0 * 9.80665,
            crash_impact_margin: 1.15,
            crash_impact_window_us: 2_000_000,
            crash_blocked_us: 1_500_000,
            motor_fault_frac: 0.3,
            motor_fault_us: 60_000,
            motor_max_erpm: 72_000.0 * 6.0, // 72,000 RPM × 6 pole pairs
        }
    }
}
