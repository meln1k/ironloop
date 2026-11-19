#!/usr/bin/env python3
"""Mutation check for the DST properties.

Each mutant reintroduces a real flight-controller bug into the firmware
core. The checks must catch every one of them; a mutant that survives
means the workload or the properties miss a real-world case. The
"monitor:" mutants instead break the checker (the simulator's monitor) in
ways that once hid real firmware failures.

Usage: scripts/mutants.py [seeds] [name filter]   (default 300; run from anywhere)

For the unmodified firmware and then each mutant this runs the DST sweep,
the firmware unit tests, and the simulator tests, and builds the Mobula6
board firmware (a core change it no longer compiles against). A mutant is caught when
any of them fails where the unmodified firmware passed: a new failing test,
or a property failing on a seed where it held before. The
source files are restored after every mutant, even on Ctrl-C or SIGTERM.
"""

import os
import re
import signal
import subprocess
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
LIB = "crates/ironloop-fc/src/lib.rs"
CFG = "crates/ironloop-fc/src/config.rs"
MATH = "crates/ironloop-fc/src/math.rs"
IMU = "crates/ironloop-fc/src/imu.rs"
CONTROL = "crates/ironloop-fc/src/control.rs"
FAILSAFE = "crates/ironloop-fc/src/failsafe.rs"
PROTECT = "crates/ironloop-fc/src/protect.rs"
MON = "crates/ironloop-sim/src/monitor/mod.rs"

MIX_OK = """    [-1.0, -1.0, 1.0],
    [-1.0, 1.0, -1.0],
    [1.0, -1.0, -1.0],
    [1.0, 1.0, 1.0],"""
MIX_YAW_INVERTED = """    [-1.0, -1.0, -1.0],
    [-1.0, 1.0, 1.0],
    [1.0, -1.0, 1.0],
    [1.0, 1.0, -1.0],"""

# name -> list of (file, original text, mutated text)
MUTANTS = {
    # Arming and disarming.
    "arms with throttle up": [(LIB, "if self.rc.input.throttle > THROTTLE_LOW {", "if self.rc.input.throttle > 0.95 {")],
    "arm switch assumed off at boot": [(LIB, "arm_switch: true,\n", "arm_switch: false,\n")],
    "level-triggered arming": [(LIB, "} else if on && !was_on && !s.armed {", "} else if on && !s.armed {")],
    "disarm keeps last motor values": [(LIB, "    s.motors = [DSHOT_STOP; 4];\n    s.motor_frac", "    s.motor_frac")],
    "reboot command obeyed while armed": [(LIB, "Command(Command::RebootToBootloader) => {\n            if !s.armed {", "Command(Command::RebootToBootloader) => {\n            if true {")],
    # Link loss.
    "no RC-loss failsafe": [(FAILSAFE, "FailsafeStage::None if lost && s.armed =>", "FailsafeStage::None if false =>")],
    "rc_timeout 10 s (ms/us mixup)": [(CFG, "rc_timeout_us: 100_000,", "rc_timeout_us: 10_000_000,")],
    "rc_timeout 10 ms (false failsafe)": [(CFG, "rc_timeout_us: 100_000,", "rc_timeout_us: 10_000,")],
    "failsafe spins up on the ground": [(FAILSAFE, "FailsafeStage::Hold if s.throttle_at_loss < THROTTLE_LOW => Some(0.0),\n", "")],
    "failsafe lands at fixed 28% for 4 s": [
        (FAILSAFE, "Some(s.hover_throttle * k / sag_ratio(s))", "Some(0.28 + 0.0 * k)"),
        (CFG, "failsafe_landing_us: 30_000_000,", "failsafe_landing_us: 4_000_000,"),
    ],
    "failsafe lands at 100% hover (no descent)": [(FAILSAFE, "Some(s.hover_throttle * k / sag_ratio(s))", "Some(s.hover_throttle / sag_ratio(s) + 0.0 * k)")],
    "failsafe touchdown detection never fires": [(FAILSAFE, "if (s.landing_trim >= 0.3 && unmoved(s)) ||", "if false ||")],
    "hover learning also runs in failsafe": [(CONTROL, "if s.failsafe == FailsafeStage::None && s.flight.airmode_active && level_flight", "if s.flight.airmode_active && level_flight")],
    "hover learning during climbs": [(CONTROL, "let level_flight = s.flight.vz_est.abs() < 0.5;", "let level_flight = true;")],
    # IMU handling.
    "no IMU timeout": [(LIB, "if s.armed && s.imu_stale() {", "if false {")],
    "imu_timeout 2 s": [(CFG, "imu_timeout_us: 20_000,", "imu_timeout_us: 2_000_000,")],
    "frozen-IMU (repeat) check removed": [(IMU, "if (gyro, accel) != s.last_raw {", "if true {")],
    "no gyro spike rejection": [(IMU, "let valid = in_range && !spike;", "let valid = in_range;")],
    "spike filter never re-trusts the sensor": [(IMU, "&& s.spike_run < 4", "&& s.spike_run < 255")],
    "dt clamp removed": [(IMU, "let dt = (dt_us as f32 * 1e-6).clamp(50e-6, 2e-3);", "let dt = (dt_us as f32 * 1e-6).max(1e-6);")],
    "IMU Cw90/Cw270 swapped in align()": [(IMU, "ImuAlignment::Cw90 => Vec3::new(-v.y, v.x, v.z),", "ImuAlignment::Cw90 => Vec3::new(v.y, -v.x, v.z),")],
    "gyro calibration accepts motion": [(IMU, "if spread > s.cfg.gyro_cal_motion * 2.0 {", "if false {")],
    "estimator trusts accel during acceleration": [(IMU, "let weight = (1.0 - dev / band).clamp(0.0, 1.0);", "let weight = 1.0f32;")],
    "estimator bias integral sign flipped": [(IMU, "err * (0.3 * weight * dt)", "err * (-0.3 * weight * dt)")],
    # Control.
    "yaw mixer sign inverted (props-in)": [(CONTROL, MIX_OK, MIX_YAW_INVERTED)],
    "D-term sign flipped": [(CONTROL, "+ integral[i] - k.d * d[i]", "+ integral[i] + k.d * d[i]")],
    "acro pitch stick inverted": [(CONTROL, "-rate_curve(&cfg.rates[1], pitch),", "rate_curve(&cfg.rates[1], pitch),")],
    "angle pitch target inverted": [(CONTROL, "up_in_body(roll * cfg.angle_limit, -pitch * cfg.angle_limit)", "up_in_body(roll * cfg.angle_limit, pitch * cfg.angle_limit)")],
    "angle_limit 80 deg": [(CFG, "angle_limit: 55.0f32.to_radians(),", "angle_limit: 80.0f32.to_radians(),")],
    "I-term winds up at zero throttle": [(CONTROL, "s.flight.airmode_active && throttle > THROTTLE_LOW && !s.flight.mixer_saturated", "s.flight.airmode_active && !s.flight.mixer_saturated")],
    "vbat sag compensation inverted": [(CONTROL, "(cfg.vbat_ref / v).clamp(1.0, 1.25),", "(v / cfg.vbat_ref).clamp(0.8, 1.0),")],
    "armed motor drops to 0 when mix is negative": [(CONTROL, "let f = cfg.motor_idle + (1.0 - cfg.motor_idle) * m;", "let f = if m <= 0.0 { 0.0 } else { cfg.motor_idle + (1.0 - cfg.motor_idle) * m };")],
    # Crash and runaway detection.
    "no crash detection": [(PROTECT, "if (hit && lying_for >= cfg.crash_still_us) || blocked_for >= cfg.crash_blocked_us {", "if false {")],
    "crash lying needs no impact (inverted dive disarms)": [(PROTECT, "let hit = s.flight.lying_since_us.is_some_and(", "let hit = true || s.flight.lying_since_us.is_some_and(")],
    "crash rules share one timer (roll-out of a dive disarms)": [(
        PROTECT,
        "let blocked_for = if blocked { s.flight.blocked_us } else { 0 };",
        "let blocked_for = if blocked { s.flight.blocked_us.max(lying_for) } else { 0 };",
    )],
    "crash detect fires at 10 deg tilt": [(PROTECT, "let lying = tilt > TIPPED_OVER && rp_motion", "let lying = tilt > 10f32.to_radians() && rp_motion")],
    "crash motion counts spinning flat": [(PROTECT, "let rp_motion = (slow - up * slow.dot(up)).norm();", "let rp_motion = slow.norm();")],
    "crash detect ignores the accelerometer, fires after 1.5 s": [
        (PROTECT, "rp_motion < 0.5 + 0.25 * slow.norm() && propped;", "rp_motion < 0.5 + 0.25 * slow.norm();"),
        (CFG, "crash_still_us: 3_000_000,", "crash_still_us: 1_500_000,"),
    ],
    "arming leaves the motors at zero until the next IMU sample": [(LIB, "s.motors = [control::dshot(s.cfg.motor_idle); 4];", "s.motors = [0; 4];")],
    "runaway detection rearms after the build flew": [(LIB, "s.runaway_active = !s.flight_proven;", "s.runaway_active = true;")],
    "runaway fires on a pilot's stick reversal": [(PROTECT, "let runaway = pid_high && spinning && (pushed_back || !flown);", "let runaway = pid_high && spinning;")],
    "crash impact counts drag braking after a power dive": [(PROTECT, "if a > cfg.crash_impact && jolt {", "if a > cfg.crash_impact {")],
    "runaway fires on gyro alone": [(PROTECT, "let runaway = pid_high && spinning && (pushed_back || !flown);", "let runaway = spinning;")],
    "runaway detection ends after 0.5 s of throttle alone": [
        (PROTECT, "if throttle > cfg.airmode_throttle && s.flight.lifted_off && upright && !pid_high && (!spinning || flown) {", "if throttle > cfg.airmode_throttle {"),
        (CFG, "runaway_deactivate_us: 1_000_000,", "runaway_deactivate_us: 500_000,"),
    ],
    # Time.
    "time truncated to u32": [(LIB, "    } else {\n        s.now_us = event.t_us;\n    }", "    } else {\n        s.now_us = event.t_us as u32 as u64;\n    }")],
    "reports every reordered event as a clock glitch": [(LIB, "if s.now_us - event.t_us > 1_000 {", "if s.now_us > event.t_us {")],
    "clock follows reordered events back": [(LIB, "} else if event.t_us < s.now_us {", "} else if event.t_us < s.now_us && false {")],
    "clock check underflows (panics)": [(LIB, "if s.now_us - event.t_us > 1_000 {", "if event.t_us - s.now_us > 1_000 {")],
    # Arming, output buffer, attitude math.
    "arms on a stale IMU": [(LIB, "if self.imu_stale() {", "if false {")],
    "effects buffer too small": [(LIB, "pub const MAX_EFFECTS: usize = 8;", "pub const MAX_EFFECTS: usize = 3;")],
    "small-angle attitude update, never renormalized": [(
        MATH,
        "(self * Quat::from_rotation_vector(omega_body * dt)).normalized()",
        "{ let h = omega_body * (0.5 * dt); self * Quat { w: 1.0, x: h.x, y: h.y, z: h.z } }",
    )],
    "no re-convergence after gyro saturation": [(IMU, "s.fast_converge_until_us = now + 1_500_000;", "s.fast_converge_until_us = now;")],
    # Runaway, notices, link recovery.
    "no runaway detection": [(PROTECT, "let runaway = pid_high && spinning && (pushed_back || !flown);", "let runaway = false;")],
    "battery warning at 3.75 V": [(CFG, "vbat_warn: 3.45,", "vbat_warn: 3.75,")],
    "motor fault below 95% of expected eRPM": [(CFG, "motor_fault_frac: 0.3,", "motor_fault_frac: 0.95,")],
    "failsafe needs 3 s of link to recover": [(CFG, "failsafe_recovery_us: 300_000,", "failsafe_recovery_us: 3_000_000,")],
    "failsafe hold at 130% hover": [(FAILSAFE, "FailsafeStage::Hold => Some(s.hover_throttle / sag_ratio(s)),", "FailsafeStage::Hold => Some(s.hover_throttle * 1.3 / sag_ratio(s)),")],
    # Control quality.
    "rate curve 40% low": [(CONTROL, "let dps = (200.0 * r.rc_rate * x * super_factor)", "let dps = (120.0 * r.rc_rate * x * super_factor)")],
    "roll/pitch D ten times too high (oscillates)": [(CFG, "d: 0.000525, ff: 0.0 }", "d: 0.00525, ff: 0.0 }")],
    # Filtering: noise the filters let through jitters the motors.
    "RPM filter off": [(CFG, "rpm_filter_harmonics: 3,", "rpm_filter_harmonics: 0,")],
    "D-term low-pass at 250 Hz": [(CFG, "dterm_lpf_hz: 90.0,", "dterm_lpf_hz: 250.0,")],
    # The checker itself: a monitor bug that hides real firmware failures.
    "monitor: any fault during a link loss excuses the failsafe checks": [(
        MON,
        "excuses(sc, t, |e| e == Excuse::Flight)",
        "excuses(sc, t, |e| e != Excuse::LinkOrWeather) && excuses(sc, t, |e| e != Excuse::Nothing)",
    )],
}


def run(cmd):
    return subprocess.run(cmd, capture_output=True, text=True)


def check(seeds):
    """Builds and runs every check; returns the set of failures: a property
    on a seed as "property@seed", anything else by name."""
    b = run(["cargo", "build", "-q", "--release", "-p", "ironloop-sim", "--bin", "dst"])
    if b.returncode != 0:
        return {"build: " + b.stderr[-300:]}
    out = run(["./target/release/dst", "--seeds", seeds, "--list-failures"]).stdout
    fails = set()
    for line in out.splitlines():
        if line.startswith("failure "):
            _, seed, prop = line.split()[:3]
            fails.add(f"{prop}@{seed}")
        elif line.startswith("  FAIL  deterministic_replay") or line.startswith("  THIN  "):
            fails.add(line.strip())
    # Type-checking is enough to see a core change break the board, and
    # skips the firmware's fat-LTO link on every mutant.
    fw = subprocess.run(["cargo", "check", "-q"], cwd="firmware/mobula6", capture_output=True, text=True)
    if fw.returncode != 0:
        fails.add("firmware build")
    for pkg, extra in (("ironloop-fc", []), ("ironloop-sim", ["--test", "dst"])):
        t = run(["cargo", "test", "-q", "--release", "-p", pkg] + extra)
        # `-q` reports a failure as "name --- FAILED".
        failed = set(re.findall(r"^(\S+) --- FAILED$", t.stdout, re.M))
        if t.returncode != 0 and not failed:
            failed = {f"{pkg} tests failed or did not build"}
        fails |= failed
    return fails


def write(path, text):
    """Replaces the file in one step, so a kill mid-write cannot leave it
    empty or half written."""
    tmp = path + ".mutant-tmp"
    with open(tmp, "w") as f:
        f.write(text)
    os.replace(tmp, path)


def new_failures(base, got):
    """What fails now and did not before, with a property's seeds grouped:
    "name(seed, seed)"."""
    new = got - base
    seeds = {}
    for f in new:
        prop, _, seed = f.partition("@")
        seeds.setdefault(prop, []).append(seed)
    return sorted(f"{p}({', '.join(sorted(s, key=int))})" if s[0] else p for p, s in seeds.items())


def main():
    os.chdir(ROOT)
    # Restore the sources when killed too, not only on Ctrl-C.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
    seeds = sys.argv[1] if len(sys.argv) > 1 else "300"
    only = sys.argv[2] if len(sys.argv) > 2 else ""
    originals = {f: open(f).read() for f in {f for m in MUTANTS.values() for f, _, _ in m}}
    base = check(seeds)
    print(f"baseline: {', '.join(new_failures(set(), base)) or 'clean'}", flush=True)
    survived = []
    try:
        for name, edits in MUTANTS.items():
            if only not in name:
                continue
            text = dict(originals)
            for f, a, b in edits:
                assert text[f].count(a) == 1, f"{name}: pattern not unique in {f}: {a!r}"
                text[f] = text[f].replace(a, b)
            for f, t in text.items():
                write(f, t)
            fails = new_failures(base, check(seeds))
            print(f"{'caught' if fails else 'SURVIVED':9} {name:48} {', '.join(fails)}", flush=True)
            if not fails:
                survived.append(name)
            for f, t in originals.items():
                write(f, t)
    finally:
        for f, t in originals.items():
            write(f, t)
    print(f"{len(survived)} of {sum(only in n for n in MUTANTS)} mutants survived")
    sys.exit(1 if survived else 0)


if __name__ == "__main__":
    main()
