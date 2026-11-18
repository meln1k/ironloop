//! A quick DST sweep as part of `cargo test`. The `dst` binary runs larger
//! sweeps (`cargo run --release -p ironloop-sim --bin dst -- --seeds 1000`).

use ironloop_fc::FlightMode;
use ironloop_sim::{Action, Scenario, run_scenario, run_seed};

#[test]
fn generated_scenarios_hold_every_property_and_replay_identically() {
    for seed in 0..24 {
        let r = run_seed(seed);
        assert!(r.violations.is_empty(), "seed {seed} [{}]: {:?}", r.name, r.violations);
        assert_eq!(run_seed(seed).trace_hash, r.trace_hash, "seed {seed} is not deterministic");
    }
}

/// Seeds the default sweep misses that once broke a property. Seed 1531
/// has its motors swapped and sits on the ground at part throttle for half a
/// second, which once switched runaway detection off before it tumbled.
/// Seed 822 re-arms after a drop and slams the sticks at once, which a
/// runaway check that waits for calm flight mistook for a runaway.
/// Seed 3007 arms during an IMU dropout, which once left the motors at
/// the disarmed zeros while armed until the next sample.
/// Seed 6875 is re-armed mid-fall after flying fine and hits the ground,
/// which runaway detection once took for a build that cannot fly.
/// Seed 3934 has its motor wires swapped and motor 1 losing sync, which
/// the firmware reports on output 0; the monitor once judged that report
/// against physical motor 0.
/// Seed 838 has its motors swapped and tumbles upside down on the ground
/// while the pilot slams the sticks, which once counted as proven flight
/// and reset the blocked-crash timer at every stick reversal.
/// Seed 88 crashes onto its nose and spins flat with the roll stick held,
/// which the attitude estimate's error once made look like rolling.
/// Seed 445 loses a motor, sits on the ground, and is flipped over where it
/// stands by the pilot with no impact, which once never counted as lying.
/// Seeds 519 and 415 land by failsafe from 10-13 m, which once sped up past
/// 2.5 m/s: the brake stopped at 98% of hover, and the speed estimate
/// started from zero at link loss although the quad was already moving.
/// Seeds 109 and 257 saturate the gyro hitting the ground and punch out,
/// which once left the tilt estimate 20-27 degrees off for seconds. Seed
/// 209 rolls hard at 1 g soon after one, which a recovery that believed
/// any 1 g reading once took for level.
/// Seeds 741, 1360 and 1979 land by failsafe from 13-16 m and once touched
/// down at 2.5-2.8 m/s: each descent probe of the trim left the descent
/// faster with nothing above hover to slow it, and 1360's sagging pack,
/// below where sag compensation is capped, could not hold even that.
#[test]
fn regression_seeds_hold_every_property() {
    for seed in [1531, 822, 3007, 6875, 3934, 838, 88, 445, 519, 415, 109, 257, 209, 741, 1360, 1979] {
        let r = run_seed(seed);
        assert!(r.violations.is_empty(), "seed {seed} [{}]: {:?}", r.name, r.violations);
    }
}

/// Seed 1269 loses the link at altitude and touches down by failsafe while
/// one of its faults that excuse nothing (a clock jump, RC corruption) is
/// still in its window. The monitor once let unrelated faults like these switch the
/// failsafe checks off, which hid a failsafe touchdown at 2.6 m/s; this
/// landing must be judged, and soft.
#[test]
fn failsafe_landing_is_judged_through_an_unrelated_fault() {
    let r = run_seed(1269);
    assert!(r.violations.is_empty(), "seed 1269 [{}]: {:?}", r.name, r.violations);
    assert!(r.stats.live.get("failsafe_lands_softly").is_some_and(|&n| n > 0), "seed 1269: touchdown not judged: {:?}", r.stats.live);
}

#[test]
fn nominal_flights_take_off_and_land_in_both_modes() {
    for mode in [FlightMode::Angle, FlightMode::Acro] {
        let r = run_scenario(Scenario::nominal(7, mode));
        assert!(r.violations.is_empty(), "{mode:?}: {:?}", r.violations);
        assert_eq!(r.stats.arms, 1, "{mode:?}");
        assert!(r.stats.max_alt > 1.0, "{mode:?} never climbed: {:?}", r.stats);
        assert!(r.stats.max_touchdown < 1.5, "{mode:?} landed hard: {:?}", r.stats);
    }
}

// Paired (metamorphic) runs: the same flight with one thing changed that the
// firmware must not care about. They need no oracle for what the right
// flight is, only that the two agree.

fn seeds_where(pred: impl Fn(&Scenario) -> bool, n: usize) -> Vec<Scenario> {
    (0..400).map(Scenario::generate).filter(|s| pred(s)).take(n).collect()
}

/// Hash of every motor command, and whether the IMU may have clipped: the
/// gyro reported saturating, or the true specific force plus the most prop
/// vibration can add came within reach of the accelerometer's range (a
/// crash can hit the ground at over 100 g).
fn motors(sc: Scenario) -> (u64, bool) {
    let accel_reach = sc.imu.accel_range - 2.0 * sc.imu.vib_accel;
    let mut sim = ironloop_sim::Sim::new(sc);
    let mut clipped = false;
    while !sim.done() {
        sim.next_event();
        clipped |= sim.world.specific_force().max_abs() >= accel_reach;
    }
    clipped |= sim.notices.iter().any(|(_, n)| *n == ironloop_fc::Notice::GyroSaturated);
    (sim.motor_hash, clipped)
}

/// No deliberate violent maneuvers, which can end in a chaotic tumble.
fn calm(s: &Scenario) -> bool {
    s.faults.is_empty() && !s.unflyable() && !s.segments.iter().any(|g| g.action.is_wild() || g.action == Action::Punch)
}

#[test]
fn clock_epoch_does_not_change_the_motors() {
    // Time is only ever used as a difference, so starting the clock just
    // below 2^32 or 2^40 µs must give bit-identical motor commands.
    for sc in seeds_where(|s| s.clock_offset_us == 0 && !s.faults.iter().any(|f| matches!(f.kind, ironloop_sim::FaultKind::ClockGlitch { .. })), 4) {
        let base = motors(sc.clone()).0;
        for offset in [(1u64 << 32) - 7_000_000, 1u64 << 40] {
            let mut shifted = sc.clone();
            shifted.clock_offset_us = offset;
            assert_eq!(motors(shifted).0, base, "seed {} at epoch {offset}", sc.seed);
        }
    }
}

#[test]
fn matched_imu_mounting_does_not_change_the_motors() {
    // The board's sensor axes are a permutation of the body axes; with the
    // configuration matching, the firmware must see exactly the same data.
    use ironloop_fc::ImuAlignment::*;
    // IMU faults corrupt one sensor axis, which differs between mountings.
    let imu_fault = |s: &Scenario| s.faults.iter().any(|f| matches!(f.kind, ironloop_sim::FaultKind::ImuGarbage { .. }));
    // The sensor clips asymmetrically (-32768..32767 LSB), so mountings
    // differ by one LSB once it saturates; compare flights that don't.
    let mut checked = 0;
    for sc in seeds_where(|s| s.imu.mount == Cw0 && s.fc.imu_alignment == Cw0 && !imu_fault(s) && s.imu.axis_sign == [1.0; 3], 12) {
        let (base, clipped) = motors(sc.clone());
        if clipped || checked == 3 {
            continue;
        }
        checked += 1;
        for a in [Cw90, Cw180, Cw270] {
            let mut turned = sc.clone();
            turned.imu.mount = a;
            turned.fc.imu_alignment = a;
            assert_eq!(motors(turned).0, base, "seed {} mounted {a:?}", sc.seed);
        }
    }
    assert_eq!(checked, 3);
}

/// The outcome of a flight in terms that should not depend on heading or
/// sensor rate: the properties, whether it flew, and how high.
fn outcome(r: &ironloop_sim::RunReport) -> (Vec<&'static str>, u32, bool) {
    let mut v: Vec<_> = r.violations.iter().map(|v| v.property).collect();
    v.sort();
    (v, r.stats.arms, r.stats.max_alt > 0.5)
}

#[test]
fn heading_and_imu_rate_do_not_change_the_outcome() {
    use core::f32::consts::PI;
    let seeds = seeds_where(calm, 4);
    assert_eq!(seeds.len(), 4);
    for sc in seeds {
        let base = run_scenario(sc.clone());
        for yaw in [sc.initial_yaw + PI / 2.0, sc.initial_yaw - 2.5] {
            let mut turned = sc.clone();
            turned.initial_yaw = yaw;
            let r = run_scenario(turned);
            assert_eq!(outcome(&r), outcome(&base), "seed {} [{}] heading {yaw}: {:?} vs {:?}", sc.seed, sc.name, r.violations, base.violations);
            assert!((r.stats.max_alt - base.stats.max_alt).abs() < 0.15 * base.stats.max_alt + 0.3, "seed {} heading {yaw}: {} vs {} m", sc.seed, r.stats.max_alt, base.stats.max_alt);
        }
        for hz in [1600.0, 3200.0, 8000.0] {
            let mut rate = sc.clone();
            rate.imu.odr_hz = hz;
            let r = run_scenario(rate);
            assert_eq!(outcome(&r), outcome(&base), "seed {} at {hz} Hz", sc.seed);
        }
    }
}

#[test]
fn a_snapshot_resumes_identically() {
    // Cloning the whole simulation mid-flight and running both copies on
    // must give the same trace: no hidden state outside `State` and `Sim`.
    for seed in [3, 11] {
        let mut a = ironloop_sim::Sim::new(Scenario::generate(seed));
        a.run_until(a.sc.duration_us / 2);
        let mut b = a.clone();
        a.run();
        b.run();
        assert_eq!(a.trace_hash, b.trace_hash, "seed {seed}");
        assert_eq!(a.trace_hash, run_seed(seed).trace_hash, "seed {seed}");
    }
}

/// A build with its motor wires swapped sits upright on the ground at part
/// throttle before the pilot throttles up. Sitting still, upright, with the
/// PIDs in range once counted as proof that the build flies and switched
/// runaway detection off, leaving crash detection to stop it seconds later.
#[test]
fn sitting_at_part_throttle_does_not_prove_a_build_flies() {
    use ironloop_sim::{Sim, Sticks};
    for thr in [0.3, 0.4] {
        for sit in [0.7, 1.2, 2.0] {
            let mut sc = Scenario::manual(0);
            sc.motor_wiring = [1, 0, 2, 3];
            let mut sim = Sim::new(sc);
            let mut armed_at = None;
            while sim.t_us < 12_000_000 {
                let t = sim.t_us as f64 * 1e-6;
                let mut s = Sticks { arm: t > 1.5, ..Default::default() };
                if let Some(a) = armed_at {
                    s.throttle = if t - a < sit { thr } else { 0.7 };
                }
                sim.manual = Some(s);
                sim.next_event();
                if sim.status.armed && armed_at.is_none() {
                    armed_at = Some(t + 0.2);
                }
                if armed_at.is_some() && !sim.status.armed {
                    break;
                }
            }
            assert!(!sim.status.armed, "still armed after sitting {sit} s at {thr}");
            assert!(sim.monitor.violations.is_empty(), "sit {sit} s at {thr}: {:?} {:?}", sim.monitor.violations, sim.notices.last());
        }
    }
}

/// Acro freestyle on a powerful build (twice the Mobula6's thrust): climb,
/// roll inverted, power-dive, cut the throttle and fall inverted hands off,
/// then roll back and catch it. Cutting the throttle at the bottom of the
/// dive brakes at over 4 g on drag alone, and the fall then reads like lying
/// on the ground; no automatic disarm may happen in the air.
#[test]
fn an_inverted_power_dive_and_hands_off_fall_never_disarms_in_the_air() {
    use ironloop_sim::{Sim, Sticks};
    let mut sc = Scenario::manual(0);
    sc.frame.k_t *= 2.0;
    let mut sim = Sim::new(sc);
    let (mut phase, mut since) = (0, 0.0);
    while sim.t_us < 45_000_000 {
        let t = sim.t_us as f64 * 1e-6;
        let (tilt, alt) = (sim.world.body.att.tilt().to_degrees(), sim.world.altitude());
        let mut s = Sticks { arm: t > 1.5, acro: true, ..Default::default() };
        let done = match phase {
            0 => { s.throttle = if t > 2.0 { 1.0 } else { 0.0 }; alt > 150.0 }
            1 => { s.throttle = 0.3; s.roll = 1.0; tilt > 170.0 }
            2 => { s.throttle = 1.0; t - since > 3.0 }
            3 => { s.throttle = 0.0; t - since > 6.0 }
            4 => { s.throttle = 0.5; s.roll = 1.0; tilt < 20.0 }
            _ => { s.throttle = 0.4; false }
        };
        if done {
            phase += 1;
            since = t;
        }
        sim.manual = Some(s);
        sim.next_event();
        if !sim.status.armed && t > 2.0 {
            assert!(sim.world.altitude() < 1.0, "disarmed at {:.1} m in phase {phase}: {:?}", sim.world.altitude(), sim.notices.last());
        }
    }
    assert!(phase >= 5, "the flight plan did not finish (phase {phase})");
}

/// An IMU axis read with the wrong sign is a mirror image, which no
/// `ImuAlignment` describes, so a mount check cannot catch it. A flipped x
/// or y axis arms and pushes the wrong way on roll or pitch; runaway
/// detection must stop it. A flipped z axis (alone, or with y: the chip
/// upside down) reads upside down and must never arm.
#[test]
fn a_flipped_imu_axis_never_flies() {
    let mut failures = Vec::new();
    for mode in [FlightMode::Angle, FlightMode::Acro] {
        for (signs, arms) in [([-1.0, 1.0, 1.0], true), ([1.0, -1.0, 1.0], true), ([1.0, 1.0, -1.0], false), ([1.0, -1.0, -1.0], false)] {
            let mut sc = Scenario::nominal(7, mode);
            sc.imu.axis_sign = signs;
            let r = run_scenario(sc);
            let judged = r.stats.live.get("runaway_stops").is_some_and(|&n| n > 0);
            if !r.violations.is_empty() || (r.stats.arms > 0) != arms || (arms && !judged) || r.stats.max_alt >= 1.0 {
                failures.push(format!("{mode:?} {signs:?}: arms {} judged {judged} alt {:.1} {:?}", r.stats.arms, r.stats.max_alt, r.violations));
            }
        }
    }
    assert!(failures.is_empty(), "{failures:#?}");
}

/// Motors in the wrong order push roll or pitch the wrong way; motors that
/// all spin the wrong way push yaw the wrong way. Neither may fly off:
/// runaway detection must stop them.
#[test]
fn miswired_or_reversed_motors_never_fly() {
    let mut failures = Vec::new();
    let wirings = [[1, 0, 2, 3], [2, 1, 0, 3], [3, 1, 2, 0], [0, 2, 1, 3], [0, 3, 2, 1], [0, 1, 3, 2], [2, 0, 3, 1], [0, 1, 2, 3]];
    for mode in [FlightMode::Angle, FlightMode::Acro] {
        for wiring in wirings {
            let mut sc = Scenario::nominal(7, mode);
            sc.motor_wiring = wiring;
            sc.spin_reversed = wiring == [0, 1, 2, 3];
            let r = run_scenario(sc);
            let judged = r.stats.live.get("runaway_stops").is_some_and(|&n| n > 0);
            if !r.violations.is_empty() || !judged {
                failures.push(format!("{mode:?} {wiring:?} reversed {}: arms {} judged {judged} alt {:.1} {:?}", wiring == [0, 1, 2, 3], r.stats.arms, r.stats.max_alt, r.violations));
            }
        }
    }
    assert!(failures.is_empty(), "{failures:#?}");
}
