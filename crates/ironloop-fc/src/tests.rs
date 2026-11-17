use super::*;

const LEVEL_ACCEL: [f32; 3] = [0.0, 0.0, -GRAVITY];

fn rc(thr: u16, arm: u16, mode: u16, roll: u16) -> EventKind {
    EventKind::Rc { channels: [roll, 1500, thr, 1500, arm, mode, 1500, 1500] }
}

struct Rig {
    s: State,
    t: u64,
    fx: std::vec::Vec<Effect>,
}

extern crate std;

impl Rig {
    fn new() -> Self {
        Rig { s: State::default(), t: 0, fx: std::vec::Vec::new() }
    }
    fn ev(&mut self, kind: EventKind) {
        let s = core::mem::take(&mut self.s);
        let (s, fx) = step(s, &Event { t_us: self.t, kind });
        self.s = s;
        self.fx.extend(fx);
    }
    /// Runs `ms` of level, still IMU samples at 2 kHz with RC at 100 Hz.
    fn run(&mut self, ms: u64, rc_frame: Option<EventKind>) {
        for i in 0..ms * 2 {
            self.t += 500;
            // Real sensors never repeat a sample bit for bit; the firmware
            // treats exact repeats as a hung IMU.
            let d = if i % 2 == 0 { 1e-4 } else { -1e-4 };
            self.ev(EventKind::Imu { gyro: [d, 0.0, 0.0], accel: LEVEL_ACCEL });
            if i % 20 == 0 {
                self.ev(EventKind::Battery { volts: 4.2, amps: 0.0 });
                if let Some(f) = rc_frame {
                    self.ev(f);
                }
            }
        }
    }
    fn last_motors(&self) -> [u16; 4] {
        self.fx
            .iter()
            .rev()
            .find_map(|e| if let Effect::Motors(m) = e { Some(*m) } else { None })
            .unwrap()
    }
    fn notices(&self) -> std::vec::Vec<Notice> {
        self.fx.iter().filter_map(|e| if let Effect::Notice(n) = e { Some(*n) } else { None }).collect()
    }
}

#[test]
fn arm_switch_on_at_power_up_never_arms() {
    let mut r = Rig::new();
    r.run(3000, Some(rc(1000, 2000, 1000, 1500)));
    assert!(!r.s.status().armed);
    assert_eq!(r.last_motors(), [0; 4]);
    // Cycling the switch arms.
    r.run(50, Some(rc(1000, 1000, 1000, 1500)));
    r.run(50, Some(rc(1000, 2000, 1000, 1500)));
    assert!(r.s.status().armed, "{:?}", r.notices());
}

#[test]
fn arming_refused_with_throttle_up_or_before_calibration() {
    let mut r = Rig::new();
    r.run(100, Some(rc(1000, 1000, 1000, 1500)));
    r.run(50, Some(rc(1000, 2000, 1000, 1500)));
    assert!(!r.s.status().armed);
    assert!(r.notices().iter().any(|n| matches!(n, Notice::ArmingBlocked(b) if b.contains(ArmBlockers::CALIBRATING))));
    r.run(1500, Some(rc(1000, 1000, 1000, 1500)));
    r.run(50, Some(rc(1300, 2000, 1000, 1500)));
    assert!(!r.s.status().armed);
    assert!(r.notices().iter().any(|n| matches!(n, Notice::ArmingBlocked(b) if b.contains(ArmBlockers::THROTTLE))));
}

#[test]
fn roll_right_in_acro_speeds_up_left_motors() {
    let mut r = Rig::new();
    r.run(1500, Some(rc(1000, 1000, 2000, 1500)));
    r.run(50, Some(rc(1000, 2000, 2000, 1500)));
    assert!(r.s.status().armed);
    r.run(20, Some(rc(1400, 2000, 2000, 1800)));
    let m = r.last_motors();
    // M3 rear-left and M4 front-left push harder than the right side.
    assert!(m[2] > m[0] && m[3] > m[1], "{m:?}");
}

#[test]
fn rc_loss_while_flying_lands_then_disarms() {
    let mut r = Rig::new();
    r.run(1500, Some(rc(1000, 1000, 1000, 1500)));
    r.run(50, Some(rc(1000, 2000, 1000, 1500)));
    r.run(500, Some(rc(1500, 2000, 1000, 1500)));
    r.run(1500, None);
    assert_eq!(r.s.status().failsafe, FailsafeStage::Landing);
    r.run(30_500, None);
    assert!(!r.s.status().armed);
    assert!(r.notices().contains(&Notice::Disarmed(DisarmReason::Failsafe)));
    assert_eq!(r.last_motors(), [0; 4]);
}

#[test]
fn rate_curve_matches_betaflight() {
    let r = Rates { rc_rate: 1.0, super_rate: 0.7, expo: 0.0 };
    assert!((control::rate_curve(&r, 1.0).to_degrees() - 666.67).abs() < 0.1);
    assert!((control::rate_curve(&r, 0.5).to_degrees() - 153.85).abs() < 0.1);
}

#[test]
fn bootloader_gesture_only_when_held_and_disarmed() {
    let gesture = EventKind::Rc { channels: [2000, 1000, 1000, 1000, 1000, 1000, 1500, 1500] };
    let mut r = Rig::new();
    r.run(1500, Some(gesture));
    assert!(!r.fx.contains(&Effect::RebootToBootloader), "fired before 2 s");
    r.run(700, Some(gesture));
    assert!(r.fx.contains(&Effect::RebootToBootloader));
    // Armed: never.
    let mut r = Rig::new();
    r.run(1500, Some(rc(1000, 1000, 1000, 1500)));
    r.run(50, Some(rc(1000, 2000, 1000, 1500)));
    assert!(r.s.status().armed);
    r.ev(EventKind::Command(Command::RebootToBootloader));
    assert!(!r.fx.contains(&Effect::RebootToBootloader));
}

#[test]
fn hung_imu_repeating_one_sample_disarms() {
    let mut r = Rig::new();
    r.run(1500, Some(rc(1000, 1000, 1000, 1500)));
    r.run(50, Some(rc(1000, 2000, 1000, 1500)));
    assert!(r.s.status().armed);
    for _ in 0..100 {
        r.t += 500;
        r.ev(EventKind::Imu { gyro: [0.0; 3], accel: LEVEL_ACCEL });
    }
    assert!(!r.s.status().armed);
    assert!(r.notices().contains(&Notice::Disarmed(DisarmReason::ImuFailure)));
}

#[test]
fn single_gyro_spike_does_not_tilt_estimate_but_real_step_is_accepted() {
    let mut r = Rig::new();
    r.run(1500, Some(rc(1000, 1000, 1000, 1500)));
    // One full-scale spike on roll between clean samples.
    r.t += 500;
    r.ev(EventKind::Imu { gyro: [34.0, 0.0, 0.0], accel: LEVEL_ACCEL });
    r.run(1, None);
    assert!(r.s.status().attitude.tilt().to_degrees() < 0.5, "{:?}", r.s.status().attitude);
    // A sustained step is believed after a few samples.
    for i in 0..20 {
        r.t += 500;
        r.ev(EventKind::Imu { gyro: [12.0 + i as f32 * 1e-3, 0.0, 0.0], accel: LEVEL_ACCEL });
    }
    assert!(r.s.status().gyro.x > 5.0, "{:?}", r.s.status().gyro);
}

#[test]
fn no_iterm_windup_while_throttle_is_down() {
    // Airmode is on after a takeoff, then the quad sits on the ground at
    // zero throttle while the pilot holds roll: the frame cannot rotate, so
    // an integrator would wind up and leave the motors uneven afterwards.
    let mut r = Rig::new();
    r.run(1500, Some(rc(1000, 1000, 2000, 1500)));
    r.run(50, Some(rc(1000, 2000, 2000, 1500)));
    r.run(100, Some(rc(1400, 2000, 2000, 1500)));
    assert!(r.s.status().airmode_active);
    r.run(2000, Some(rc(1000, 2000, 2000, 1800)));
    r.run(20, Some(rc(1000, 2000, 2000, 1500)));
    let m = r.last_motors();
    let spread = m.iter().max().unwrap() - m.iter().min().unwrap();
    assert!(spread < 20, "motors uneven after stick released: {m:?}");
}

/// Metamorphic check: mirroring every input across a body plane must mirror
/// the motor outputs exactly (up to rounding), with no oracle needed. Any
/// bug that treats left and right (or front and back) differently, such as
/// one wrong mixer entry, an asymmetric clamp, or two axes mixed up, breaks
/// the symmetry. A sign flipped consistently on a whole axis does not; the
/// DST catches those.
fn mirrored_flight_mirrors_motors(
    gyro_m: fn([f32; 3]) -> [f32; 3],
    accel_m: fn([f32; 3]) -> [f32; 3],
    sticks_m: fn([u16; 3]) -> [u16; 3],
    motors_m: fn([u16; 4]) -> [u16; 4],
    mode: u16,
) {
    let mut a = State::default();
    let mut b = State::default();
    let mut seed = 0x2545_f491_4f6c_dd1du64;
    let mut rnd = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 40) as f32 / (1u64 << 24) as f32 * 2.0 - 1.0
    };
    let mut t = 0u64;
    let mut compared = 0;
    let mut sticks = [1500u16; 3];
    for i in 0..12_000u32 {
        t += 500;
        let flying = i > 4_000;
        let (gyro, accel) = if flying {
            let k = i as f32 * 0.003;
            (
                [2.0 * libm::sinf(k * 1.3) + 0.01 * rnd(), 1.5 * libm::cosf(k * 0.7) + 0.01 * rnd(), 0.8 * libm::sinf(k * 0.4) + 0.01 * rnd()],
                [3.0 * libm::sinf(k * 0.9) + 0.1 * rnd(), 2.0 * libm::cosf(k * 1.1) + 0.1 * rnd(), -GRAVITY + 0.5 * libm::sinf(k) + 0.1 * rnd()],
            )
        } else {
            ([1e-4 * rnd(), 1e-4 * rnd(), 1e-4 * rnd()], [0.01 * rnd(), 0.01 * rnd(), -GRAVITY + 0.01 * rnd()])
        };
        let mut events = std::vec![(
            EventKind::Imu { gyro, accel },
            EventKind::Imu { gyro: gyro_m(gyro), accel: accel_m(accel) },
        )];
        if i % 20 == 0 {
            if flying && i % 200 == 0 {
                sticks = [(1500.0 + 450.0 * rnd()) as u16, (1500.0 + 450.0 * rnd()) as u16, (1500.0 + 450.0 * rnd()) as u16];
            }
            let arm = if i < 3_000 { 1000 } else { 2000 };
            let thr = if flying { 1500 } else { 1000 };
            let ch = |s: [u16; 3]| EventKind::Rc { channels: [s[0], s[1], thr, s[2], arm, mode, 1500, 1500] };
            events.push((ch(sticks), ch(sticks_m(sticks))));
            events.push((EventKind::Battery { volts: 3.8, amps: 2.0 }, EventKind::Battery { volts: 3.8, amps: 2.0 }));
        }
        for (ea, eb) in events {
            let (sa, fa) = step(core::mem::take(&mut a), &Event { t_us: t, kind: ea });
            let (sb, fb) = step(core::mem::take(&mut b), &Event { t_us: t, kind: eb });
            a = sa;
            b = sb;
            let ma = fa.iter().filter_map(|e| if let Effect::Motors(m) = e { Some(*m) } else { None });
            let mb = fb.iter().filter_map(|e| if let Effect::Motors(m) = e { Some(*m) } else { None });
            for (ma, mb) in ma.zip(mb) {
                let want = motors_m(ma);
                assert!(
                    want.iter().zip(mb).all(|(&x, y)| x.abs_diff(y) <= 2),
                    "step {i}: motors {ma:?} mirror to {want:?}, mirrored run gave {mb:?}"
                );
                compared += 1;
            }
        }
    }
    assert!(a.status().armed && b.status().armed, "both runs must fly");
    assert!(compared > 10_000);
}

#[test]
fn left_right_mirrored_inputs_mirror_the_motors() {
    // Reflect across the body x-z plane (y -> -y): rates are pseudovectors.
    for mode in [1000, 2000] {
        mirrored_flight_mirrors_motors(
            |g| [-g[0], g[1], -g[2]],
            |a| [a[0], -a[1], a[2]],
            |s| [3000 - s[0], s[1], 3000 - s[2]],
            |m| [m[2], m[3], m[0], m[1]],
            mode,
        );
    }
}

#[test]
fn front_back_mirrored_inputs_mirror_the_motors() {
    // Reflect across the body y-z plane (x -> -x).
    for mode in [1000, 2000] {
        mirrored_flight_mirrors_motors(
            |g| [g[0], -g[1], -g[2]],
            |a| [-a[0], a[1], a[2]],
            |s| [s[0], 3000 - s[1], 3000 - s[2]],
            |m| [m[1], m[0], m[3], m[2]],
            mode,
        );
    }
}

#[test]
fn board_reordering_is_silent_but_a_clock_jump_is_reported() {
    let mut r = Rig::new();
    r.run(100, None);
    let t = r.t;
    // An IMU sample stamped 150 µs before an already processed RC frame
    // (stamped at data-ready, queued after the SPI read).
    r.t = t + 1_000;
    r.ev(rc(1000, 1000, 1000, 1500));
    r.t = t + 850;
    r.ev(EventKind::Imu { gyro: [3e-4, 0.0, 0.0], accel: LEVEL_ACCEL });
    assert!(!r.notices().contains(&Notice::ClockWentBackwards));
    r.t = t - 5_000;
    r.ev(EventKind::Tick);
    assert!(r.notices().contains(&Notice::ClockWentBackwards));
}

#[test]
fn mixer_matches_betaflight_quad_x() {
    // Betaflight's mixerQuadX (roll, pitch, yaw per motor, same motor order).
    // Betaflight's pitch and yaw outputs have the opposite sign of the FRD
    // axes used here.
    const BF: [[f32; 3]; 4] = [[-1.0, 1.0, -1.0], [-1.0, -1.0, 1.0], [1.0, 1.0, 1.0], [1.0, -1.0, -1.0]];
    for (m, bf) in control::MIX.iter().zip(BF) {
        assert_eq!(*m, [bf[0], -bf[1], -bf[2]]);
    }
}

/// A full-stick reversal right after the first arm saturates the PIDs while
/// the frame still turns the old way. Slowing down toward the stick is a
/// pilot flying; speeding up away from it is a build that cannot fly.
#[test]
fn a_stick_reversal_is_no_runaway_but_turning_away_from_the_stick_is() {
    for (accel, runaway) in [(60.0, false), (-60.0, true)] {
        let mut r = Rig::new();
        r.run(1500, Some(rc(1000, 1000, 2000, 1500)));
        r.run(50, Some(rc(1000, 2000, 2000, 1500)));
        assert!(r.s.status().armed);
        // Rolling left at 15 rad/s, full right stick: 150 ms of the frame
        // changing its rate by `accel` rad/s².
        for i in 0..300 {
            r.t += 500;
            let g = -15.0 + accel * i as f32 * 500e-6 + if i % 2 == 0 { 1e-4 } else { -1e-4 };
            r.ev(EventKind::Imu { gyro: [g, 0.0, 0.0], accel: LEVEL_ACCEL });
            if i % 20 == 0 {
                r.ev(EventKind::Battery { volts: 4.2, amps: 0.0 });
                r.ev(rc(1500, 2000, 2000, 2000));
            }
        }
        let disarmed = r.notices().contains(&Notice::Disarmed(DisarmReason::RunawayTakeoff));
        assert_eq!(disarmed, runaway, "angular acceleration {accel}: {:?}", r.notices());
    }
}

#[test]
fn a_still_quad_on_the_ground_resets_its_attitude_estimate() {
    // A crash leaves the estimate wrong (here: the quad lies on its side
    // but the gyro never saw it roll, as when it saturates). Once it has
    // sat still for a moment while disarmed, the estimate must match the
    // accelerometer within a second, not over several.
    let mut r = Rig::new();
    r.run(2500, Some(rc(1000, 1000, 1000, 1500)));
    assert!(r.s.status().attitude.tilt().to_degrees() < 1.0);
    let side = [0.0, -GRAVITY * 0.866, -GRAVITY * 0.5]; // 60 degrees
    // It comes to rest spinning about its own yaw axis, which does not
    // change the estimated tilt but is the movement a crash always has.
    for i in 0..1700 {
        r.t += 500;
        let d = if i % 2 == 0 { 1e-4 } else { -1e-4 };
        let yaw = if i < 100 { 1.0 } else { 0.0 };
        r.ev(EventKind::Imu { gyro: [d, 0.0, yaw], accel: side });
    }
    let tilt = r.s.status().attitude.tilt().to_degrees();
    assert!((tilt - 60.0).abs() < 2.0, "tilt estimate {tilt} deg after 0.8 s still");
}
