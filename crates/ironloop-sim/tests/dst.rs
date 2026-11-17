//! A quick DST sweep as part of `cargo test`. The `dst` binary runs larger
//! sweeps (`cargo run --release -p ironloop-sim --bin dst -- --seeds 1000`).

use ironloop_fc::FlightMode;
use ironloop_sim::{Scenario, run_scenario, run_seed};

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
#[test]
fn regression_seeds_hold_every_property() {
    for seed in [1531, 822, 3007, 6875] {
        let r = run_seed(seed);
        assert!(r.violations.is_empty(), "seed {seed} [{}]: {:?}", r.name, r.violations);
    }
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

/// Hash of every motor command, and whether the gyro ever clipped.
fn motors(sc: Scenario) -> (u64, bool) {
    let mut sim = ironloop_sim::Sim::new(sc);
    sim.run();
    let clipped = sim.notices.iter().any(|(_, n)| *n == ironloop_fc::Notice::GyroSaturated);
    (sim.motor_hash, clipped)
}

/// No deliberate violent maneuvers, which can end in a chaotic tumble.
fn calm(s: &Scenario) -> bool {
    s.faults.is_empty() && !s.unflyable() && !["stick-slam", "flip", "inverted-hang", "drop", "punch"].iter().any(|t| s.name.contains(t))
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
    // differ by one LSB once the gyro saturates; compare flights that don't.
    let mut checked = 0;
    for sc in seeds_where(|s| s.imu.mount == Cw0 && s.fc.imu_alignment == Cw0 && !imu_fault(s), 12) {
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
