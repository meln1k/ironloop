# IronLoop

A simulation-first quadrotor flight controller for the HDZero Mobula6 ECO 2024.

The whole firmware is one pure function:

```rust
pub fn step(state: State, event: &Event) -> (State, heapless::Vec<Effect, 8>)
```

Events are timestamped IMU samples, RC frames, battery readings, ESC telemetry, 1 kHz ticks, and commands. Effects are DShot motor values, LED, beeper, notices, and "reboot to DFU". `step` never reads a clock, touches hardware, or allocates. The same code runs on the STM32F411, in the deterministic simulator, and in the browser.

| Path | What it is |
| --- | --- |
| `crates/ironloop-fc` | `no_std` firmware core: estimator, acro/angle rate PID, mixer, RPM filter, arming, failsafe, crash/runaway detection, motor-fault detection from eRPM, CRSF and DShot codecs |
| `crates/ironloop-sim` | Deterministic simulation testing: physics, ESCs, battery, wind, noisy IMU, RC link, virtual pilot, seeded workload generator, property monitor, `dst` runner |
| `crates/ironloop-viz` | Bevy app: fly the firmware by hand, inject faults live, or replay a DST seed. Builds natively and for wasm/WebGL2 |
| `firmware/mobula6` | Embassy board target (STM32F411CEU6): IMU on SPI1 (BMI270 or ICM-42688-P), ELRS CRSF on USART2, ADC battery, bit-banged bidirectional DShot300 with eRPM via TIM1 + DMA2 |
| `scripts/` | `build-web.sh`, `serve-web.py`, `mutants.py` |

## Flight modes

- **Angle** (mode switch, channel 6, low): sticks command tilt up to 55°, self-levels.
- **Acro** (channel 6 high): sticks command body rates with Betaflight rates (rc_rate 1.0, super 0.7 → 667 °/s at full stick).

Airmode engages above 25% throttle. Arming (channel 5 high) needs: gyro calibrated, throttle low, RC link up, IMU healthy, tilt under 25°, a battery (not USB power), and the switch seen off since power-up.

## Deterministic simulation testing

```sh
cargo test -p ironloop-fc -p ironloop-sim                   # unit tests + 24-seed DST sweep
(cd firmware/mobula6 && cargo build --release)              # board firmware (needs thumbv7em-none-eabihf)
cargo run --release -p ironloop-sim --bin dst -- --seeds 1000
cargo run --release -p ironloop-sim --bin dst -- --seed 626 --trace   # replay one seed with a 20 Hz flight log
cargo run --release -p ironloop-sim --bin dst -- --properties
scripts/mutants.py 300                                      # every reintroduced bug must fail a check
```

Every event is on one time-ordered queue; physics advances on a fixed 250 µs grid; each subsystem draws from its own seeded RNG stream. A seed replays bit for bit (the runner re-runs a sample and compares full input/output trace hashes).

**Workload generator** (per seed): mass, inertia, thrust and motor-lag tolerances; battery charge and internal resistance (0.03–0.15 Ω, fresh to worn 1S packs); IMU rate, jitter, noise, bias, the sensor's own two-pole low-pass filter (300–750 Hz, as the BMI270 and ICM-42688 are set up; the loop delay that makes high gains ring), warm-up drift, scale error, prop-imbalance vibration, bent props, and the IMU mounted in any of four orientations with a matching configuration; RC rate and jitter; wind and gusts; ESCs with or without bidirectional-DShot telemetry (eRPM goes through the same GCR reply codec the board decodes, so it carries the ESC's period resolution, and bit errors fail its checksum); a clock that starts just below 2³² µs so a 32-bit microsecond counter would wrap mid-flight; build mistakes (IMU configured a quarter turn off, motor wires swapped, USB-only power); handling (arm switch on at boot, receiver binding late, quad moved during calibration, sitting on its side); pilot plans (hover, cruise, flips, punch-outs, stick slams, inverted hangs of up to 5 s that fall at terminal speed, drops, landings, mode switches, arming with throttle up); and 0–3 faults placed while the quad is flying (IMU dropouts from 1 to 60 ms, stuck or garbage samples, collision torque, hard impacts that step the gyro by tens of rad/s, motor desync/death/degradation, short or permanent RC loss, RC corruption, telemetry loss, wind shifts, clock going backwards, scheduler jitter, dropped events, a stalled firmware task whose queue bursts afterwards, a reboot command mid-flight).

The simulated board behaves like the real one: IMU samples are stamped at data-ready and stepped after the SPI read. One control task reads the IMU and steps every event, so RC and battery events that arrive during a read wait for it, a data-ready edge that comes while the previous sample is still unread replaces it, and a stalled task makes no ticks and keeps only the newest IMU sample.

The virtual pilot sees the true state. It lands when the critical battery warning has sounded for 3 s (a punch-out can sag a 1S pack to critical for a moment), and re-arms when a plan segment expects it to be flying but it is not. Its altitude hold trims only while climbing or holding, so a long descent does not wind up into a dive.

**Properties** are checked on every firmware step. Safety properties hold in every scenario; `[flight]` expectations hold when the airframe is flyable and no excusing fault is active. The limits are requirements written in `monitor.rs`, not read from the firmware's `Config`, so a wrong configuration value fails a check instead of moving it:

```
motor_range                    Every motor command is 0 (stop) or in the DShot throttle range 48..=2047.
no_spin_disarmed               While disarmed, the last motor command of every step is all zeros.
armed_above_idle               While armed, no motor is ever commanded below 3% idle.
no_bootloader_armed            The firmware never jumps to the bootloader while armed.
disarm_stops_motors            A step that disarms emits an all-zero motor command.
disarm_justified               Every disarm has a cause the truth supports (switch, link loss, stale IMU, crash, unflyable build).
arm_throttle_low               Arming only happens on an RC frame with throttle at the bottom.
arm_switch_cycled              Arming needs the arm switch seen off since boot.
arm_calibrated                 Arming only happens after gyro calibration finished.
arm_battery                    Arming never happens on USB power (no battery).
arm_level                      Arming never happens on the ground tilted past 33°.
switch_disarms                 A valid RC frame with the arm switch off always leaves the quad disarmed.
failsafe_enters                Armed with RC silent 150 ms is failsafe; silent under 50 ms never is.
failsafe_no_spinup             If the throttle was at the bottom when the link was lost, failsafe adds none.
failsafe_bounded               RC lost while armed ends with motors stopped within 35 s.
failsafe_levels                [flight] One second into failsafe the quad is within 20° of level.
failsafe_descends              [flight] Six seconds into a failsafe landing the quad sinks at 0.2 m/s or more.
failsafe_disarms_on_touchdown  A failsafe landing disarms within 2 s of touching down upright.
imu_loss_stops                 No fresh IMU data for 30 ms while armed stops the motors.
esc_fed                        While armed the ESCs receive a command at least every 5 ms (unless the firmware task stalls).
crash_stops                    A crashed quad (on the ground, tilted past 60°, not rotating toward upright) stops within 4 s.
no_flyaway                     The quad never goes above 60 m or further than 120 m from home.
finite_state                   Attitude, rates, and setpoints in the firmware status are always finite.
no_false_clock_alarm           The firmware reports the clock going backwards only when it really jumped back.
battery_critical_reported      A 1S pack below 3.15 V for 5 s raises a critical battery notice.
motor_fault_reported           A stalled motor commanded hard with telemetry available is reported within 200 ms.
estimate_tracks                [flight] Estimated tilt stays within 15° of the truth while flying.
angle_stays_level              [flight] In angle mode the true tilt stays under 75°.
failsafe_lands_softly          [flight] Failsafe touches down below 2.5 m/s vertical and never disarms above 1 m.
no_crash                       [flight] Armed touchdowns are slower than 2.5 m/s, outside drops, flips, stick slams.
pilot_can_arm                  [flight] Flipping the switch on with everything ready (level, still, pack above 3.4 V) arms.
arm_imu_fresh                  Arming never happens without good IMU data in the last 30 ms.
clock_monotonic                The firmware's clock never goes backwards, whatever order events arrive in.
attitude_unit                  The attitude quaternion stays unit length (within 1e-3).
effects_fit                    No step produces more effects than its output buffer holds.
no_panic                       The firmware never panics; its overflow checks are on in the DST build.
runaway_stops                  A quad with its motors swapped never climbs above 1 m and stops within 5 s of throttle-up.
battery_notice_true            Battery warning/critical only when the pack really was below 3.62 / 3.45 V in the last 2 s.
motor_fault_true               A motor fault notice only for a motor really stalled or weakened in the last 300 ms.
link_recovers                  In failsafe with the link back without a gap for 1 s and the switch on, failsafe ends.
failsafe_holds                 [flight] The failsafe hold neither climbs nor sinks faster than 3 m/s² of acceleration.
estimate_reconverges           [flight] A tilt estimate off by over 15° (after a flip, say) is back within 10° within 4 s.
rate_tracks                    [flight] In an acro stick script, rates track the Betaflight-rate setpoint (30 ms lag) within 35% RMS (yaw 50%).
settles                        [flight] Hands off in an acro stick script, roll and pitch rates settle below 0.4 rad/s RMS.
sag_compensated                [flight] On a sagging pack the averaged motor speed matches the throttle stick within 15%.
```

**Stick scripts.** The virtual pilot sees the truth and closes the loop, which hides poor control. In `stick-script` segments it flies open loop instead: hands off, then a doublet on roll, pitch, and yaw in turn, then hands off. `rate_tracks` and `settles` judge those segments against Betaflight rates, which are written in the monitor and not read from the config.

**Paired runs** in `crates/ironloop-sim/tests/dst.rs` run the same flight twice with one thing changed that the firmware must not care about. Starting the clock below 2³² or at 2⁴⁰ µs, and mounting the IMU in another orientation with a matching configuration, must give bit-identical motor commands. The mount pairing only uses flights where the gyro never clips, because the sensor's range is asymmetric by one LSB. A different heading must give the same properties, arming, and altitude within 15%; an IMU at 1.6, 3.2, or 8 kHz the same properties, arming, and whether it flew. A snapshot of the whole simulation taken mid-flight must resume to the same trace.

**Coverage.** A property that never sees its situation passes without testing anything. Each property counts the steps on which its precondition was live, and the sweep prints in how many seeds that happened, how many seeds had each fault kind start in flight, and the share of simulated time spent airborne. Sweeps of 200 seeds or more fail (`THIN`) when a property is live in fewer than two seeds, a fault kind never starts in flight, or less than a quarter of the time is airborne.

**Metamorphic tests** in `crates/ironloop-fc/src/tests.rs` check the mixer against Betaflight's QuadX table and mirror every input left-right and front-back and require mirrored motor outputs. They need no oracle and catch anything that treats the two sides differently, such as one wrong mixer entry or two axes mixed up; a sign flipped consistently on a whole axis stays symmetric, and the DST catches those.

**Mutants.** `scripts/mutants.py` reintroduces 59 real flight-controller bugs and config mistakes, one at a time, and runs the DST sweep, the firmware unit tests, and the simulator tests against each, and type-checks the board firmware. It first records what fails on the unmodified firmware, and counts a mutant as caught only when something new fails or more seeds fail a property.

**Known limits.**

- RC corruption that passes the CRC (about 1 in 256 corrupted frames on a real link; the sim makes half of its corrupted frames this kind) is injected on sticks and the mode switch but not on the arm channel. A single frame with the arm switch read as off disarms in flight, and the firmware cannot tell it from a real switch-off without delaying every disarm.
- A switch moved to on while the link is down arms on the first frame after the link returns. This is intended.
- The motor-fault properties need bidirectional-DShot eRPM; seeds without telemetry (40%) skip them, and the rest of the properties must hold either way. The core also uses eRPM for an RPM filter (notches at each motor's first three harmonics, then a 200 Hz rather than 150 Hz gyro low-pass); without telemetry it keeps the plain 150 Hz low-pass. The sim's vibration is only at 1× and 2× shaft speed, so it can't judge the third harmonic.
- The CRSF parser is unit-tested but not in the DST loop; the simulator delivers decoded channels.
- No automatic disarm may come from how the quad is flown. Crash detection's hands-off rule (lying still, propped by about 1 g, for 3 s) also needs a jolt within 2 s before it or a roll-over on the ground, because an inverted fall at terminal speed reads exactly like lying on the ground. The jolt must be over 4 g and exceed what thrust and drag can produce in flight by 15%, worked out from the learned hover throttle (thrust/weight ≈ 1/hover²): over thrust/weight + 1 g across or against the thrust axis (cutting the throttle after an inverted power dive brakes at that), and over 2 × thrust/weight + 1 g along it (pulling out of a dive). A roll-over is a frame that tipped past 60° without the vertical-speed estimate showing more than 2 m/s of fall: a plunge only reads like lying after falling several m/s. A crash with neither, such as landing on its back after a fall faster than 2 m/s without a hard enough hit, stops only when the sticks or angle mode demand a roll or pitch the frame does not follow. Runaway detection runs only until the build has flown once per power-up, and while the sticks are moved it counts only a frame speeding up away from the controller's push, never a stick reversal.
- `crates/ironloop-sim/FAILING_SEEDS.md` lists the seeds that failed a 200,000-seed overnight sweep, by property, on the first commit (2079703). The firmware and the generator's IMU filter range have changed since, so its seeds and counts are out of date.
- The tilt estimate drifts past 15° after some stick slams and long aggressive sequences (20 of seeds 300–1799).
- The open-loop failsafe landing touches down above 2.5 m/s in about 0.1% of seeds (seed 741 at 2.5 m/s in seeds 0–999, seed 1360 at 2.8 m/s in seeds 1000–1799). Without a barometer the vertical speed estimate can be a few tenths of a m/s off for the whole landing, so the landing probes for descent by taking throttle off until it sinks 0.5 m/s faster, and each probe can leave the descent that much faster.
- One failsafe case is seen in seeds 300–1799: a bent prop climbing about 1 m/s six seconds into the failsafe landing (1257, `failsafe_descends`).
- `settles` catches ten times the D gain but not five times the P gain: the simulator's motors still lag less than real ones, so high P alone rarely rings.
- Six of the 59 mutants survive a 300-seed sweep. Three change nothing reachable: the dt clamp removed (gaps over 5 ms already reset the filters, and the D-term low-pass cancels dt below 2 ms), lying counted from 10° of tilt (the propped check already needs about 50°), and arming on a stale IMU (the same step disarms for `ImuFailure` before any motor command). Three are gaps: an 80° angle limit (the virtual pilot converts the tilt it wants into stick with the firmware's own angle limit, so it compensates), lying without the accelerometer check (the impact and roll-over rules rule out the same cases in the flights the sim makes), and runaway detection switched off after 0.5 s of throttle alone. The last one points at a real weakness: a motors-swapped quad sitting upright on the ground at part throttle for about a second also switches runaway detection off, and crash detection then stops it up to 7 s after throttle-up. The DST pilot always takes off at once, so it never sits there.
- In a sustained push (about 2.5 m/s² for over a second) the accelerometer's apparent vertical leans about 14° at almost exactly 1 g, so the tilt estimate follows it (seed 89 fails `estimate_tracks` by a hair). Without a velocity sensor there is no clean fix; a lower accelerometer gain made other seeds worse.

## Fly it in the browser

```sh
rustup target add wasm32-unknown-unknown
cargo install wasm-bindgen-cli --version 0.2.130
scripts/build-web.sh && scripts/serve-web.py 8080
```

On-screen transmitter (mouse or touch, two fingers fly both sticks): Mode 2 by default, with throttle/yaw on the left and pitch/roll on the right. The **Mode 2** button switches to Mode 1, which swaps pitch and throttle; the choice is remembered. Throttle stays where you leave it; the other axes spring back to center. Each stick moves relative to where you grab it, so touching never makes it jump. **ARM** is the arm switch: grey when off, amber when on but blocked (with the reason above it), red when armed. **ANGLE/ACRO** switches flight mode. **FPV** switches to the drone's own camera, mounted on the canopy with 25° uptilt and a 90° vertical field of view, and back to the previous third-person camera. The sticks also show keyboard, gamepad, and virtual-pilot input.

Keyboard: Space arms, W/S throttle, A/D yaw, arrows roll/pitch (Shift for full stick), M toggles angle/acro, C cycles cameras, H shows the keyboard help. Keys 1–8 inject faults (1 toggles RC loss). N replays DST seeds with the virtual pilot. A gamepad works too (mode 2).

## Flashing the Mobula6

```sh
cd firmware/mobula6
cargo build --release
llvm-objcopy -O binary target/thumbv7em-none-eabihf/release/ironloop-mobula6 ironloop.bin
dfu-util -a 0 -s 0x08000000:leave -D ironloop.bin
```

Enter DFU by holding the BOOT pad while plugging in USB, or from IronLoop itself: disarmed, arm switch off, throttle low, yaw full left, pitch full back, roll full right, held for 2 s. SWD is unavailable on this board (SWCLK doubles as the flash chip-select). To go back to Betaflight, flash target CRAZYBEEF4DX and restore Happymodel's CLI dump.

CI (`.github/workflows/ci.yml`) runs those tests and builds the board firmware with warnings as errors; `scripts/mutants.py` also counts a core change that breaks the firmware build. Hardware facts and sources for this board are in `docs/hardware/mobula6-eco-2024.md`.

The board builds the firmware's `Config` from the IMU it finds: the gyro and accelerometer ranges it programs, and the chip's mounting (CW0 for both chips, from Happymodel's factory dump). A panic, or the control loop missing the independent watchdog for about 50 ms (34–94 ms over the LSI clock's tolerance), resets the board; the ESCs stop on the missing signal, and arming afterwards needs the arm switch seen off first.

**Not yet verified on hardware.** Before props go on, check on the bench: which IMU is fitted and its axis orientation, on a BMI270 board and on an ICM-42688 board (tilt the quad, watch attitude), motor order and direction, DShot reaching the ESCs, battery voltage scaling, and LED/beeper polarity. Then the timing checks, from the counters in `control::STATS` (IMU samples, missed data-ready edges, SPI read errors, dropped events, and `step` cycles from the DWT counter). SWD is unavailable, so they need the planned USB diagnostics to read them.

- IMU rate: samples grow at 3.2 kHz with a BMI270 or 2 kHz with an ICM-42688, and missed data-ready edges stay at zero.
- Step time: the longest `step` plus applying its effects (`step_cycles_max`, at 96 MHz) stays well under the sample period (312 µs, 30,000 cycles, for the BMI270), including during stick moves, arming, and disarming.
- Dead data-ready line: with PA1 disconnected or held low, the quad must refuse to arm (IMU blocker). If it is disconnected while armed, the motors must stop within about 20 ms.

- eRPM: with the quad strapped down and props off, spin each motor up from idle. Its eRPM should rise with throttle, reaching about 72,000 RPM × 6 pole pairs at full throttle on a full pack, and a motor held still while commanded hard must be reported as faulty within about 60 ms. The board sends inverted (bidirectional) DShot by default, which the stock Bluejay 0.19 ESCs answer; ESCs flashed with BLHeli_S need `--features no-erpm`.
- RPM filter: with eRPM coming in, `step_cycles_max` must still fit the sample period (the filter runs 12 notches per sample and retunes them at every eRPM reply). In a hover, motor outputs should be visibly smoother than with `rpm_filter_harmonics = 0`.

OSD over MSP DisplayPort and USB are not implemented yet.
