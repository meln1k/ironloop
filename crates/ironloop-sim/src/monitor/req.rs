//! The requirements the properties check, written here rather than read
//! from the firmware's `Config`: a wrong value in the configuration under
//! test must fail a check instead of moving it.

use ironloop_fc::DSHOT_MIN;

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
/// In steady flight, motor commands jitter above this frequency (Hz)...
pub const REQ_QUIET_ABOVE_HZ: f32 = 30.0;
/// ...by at most this RMS fraction of full range.
pub const REQ_QUIET_RMS: f32 = 0.075;
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
pub const REQ_IDLE: f32 = 0.10;
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
/// A stick script is not scored on the response to a frame off the
/// script (a corrupted one) for this long after it (µs): one frame at full
/// stick asks for more than the motors can give, which the first-order
/// reference does not model, and the quad takes about this long to come
/// back (seeds 48969, 136715, 143014, 445159, 34731, 271939).
pub const REQ_OFF_SCRIPT_RESPONSE_US: u64 = 100_000;
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

/// Pilot rate (rad/s) for a stick deflection, with the required rates.
pub fn req_rate(stick: f32) -> f32 {
    let x = stick.clamp(-1.0, 1.0);
    let k = 1.0 / (1.0 - x.abs() * REQ_SUPER_RATE).clamp(0.01, 1.0);
    (200.0 * REQ_RC_RATE * x * k).to_radians()
}
