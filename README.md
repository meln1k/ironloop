# IronLoop

A simulation-first quadrotor flight controller for the HDZero Mobula6 ECO 2024.

The whole firmware is one pure function:

```rust
pub fn step(state: State, event: &Event) -> (State, heapless::Vec<Effect, 8>)
```

Events are timestamped IMU samples, RC frames, battery readings, ESC telemetry, 1 kHz ticks, and commands. Effects are DShot motor values, LED, beeper, notices, and "reboot to DFU". `step` never reads a clock, touches hardware, or allocates. The same code runs on the STM32F411, in the deterministic simulator, and in the browser.

| Path | What it is |
| --- | --- |
| `crates/ironloop-fc` | `no_std` firmware core: estimator, acro/angle rate PID, mixer, arming, failsafe, crash/runaway detection, CRSF and DShot codecs |
| `crates/ironloop-sim` | Deterministic simulation testing: physics, ESCs, battery, wind, noisy IMU, RC link, virtual pilot, seeded workload generator, property monitor, `dst` runner |
| `crates/ironloop-viz` | Bevy app: fly the firmware by hand, inject faults live, or replay a DST seed. Builds natively and for wasm/WebGL2 |
| `firmware/mobula6` | Embassy board target (STM32F411CEU6): IMU on SPI1 (BMI270 or ICM-42688-P), ELRS CRSF on USART2, ADC battery, bit-banged DShot300 via TIM1 + DMA2 |
| `scripts/` | `build-web.sh`, `serve-web.py`, `mutants.py` |

## Flight modes

- **Angle** (mode switch, channel 6, low): sticks command tilt up to 55°, self-levels.
- **Acro** (channel 6 high): sticks command body rates with Betaflight rates (rc_rate 1.0, super 0.7 → 667 °/s at full stick).

Airmode engages above 25% throttle. Arming (channel 5 high) needs: gyro calibrated, throttle low, RC link up, IMU healthy, tilt under 25°, a battery (not USB power), and the switch seen off since power-up.

## Deterministic simulation testing

```sh
cargo test -p ironloop-fc -p ironloop-sim                   # unit tests + 24-seed DST sweep
cargo run --release -p ironloop-sim --bin dst -- --seeds 1000
cargo run --release -p ironloop-sim --bin dst -- --seed 626 --trace   # replay one seed with a 20 Hz flight log
cargo run --release -p ironloop-sim --bin dst -- --properties
scripts/mutants.py 300                                      # every reintroduced bug must fail a check
```

Every event is on one time-ordered queue; physics advances on a fixed 250 µs grid; each subsystem draws from its own seeded RNG stream. A seed replays bit for bit (the runner re-runs a sample and compares full input/output trace hashes).

**Workload generator** (per seed): mass, inertia, thrust and motor-lag tolerances; battery charge and internal resistance (0.03–0.15 Ω, fresh to worn 1S packs); IMU rate, jitter, noise, bias, the sensor's own two-pole low-pass filter (100–500 Hz, the loop delay that makes high gains ring), warm-up drift, scale error, prop-imbalance vibration, bent props, and the IMU mounted in any of four orientations with a matching configuration; RC rate and jitter; wind and gusts; ESCs with or without bidirectional-DShot telemetry (the Mobula6 target has none yet); a clock that starts just below 2³² µs so a 32-bit microsecond counter would wrap mid-flight; build mistakes (IMU configured a quarter turn off, motor wires swapped, USB-only power); handling (arm switch on at boot, receiver binding late, quad moved during calibration, sitting on its side); pilot plans (hover, cruise, flips, punch-outs, stick slams, inverted hangs of up to 5 s that fall at terminal speed, drops, landings, mode switches, arming with throttle up); and 0–3 faults placed while the quad is flying (IMU dropouts from 1 to 60 ms, stuck or garbage samples, collision torque, hard impacts that step the gyro by tens of rad/s, motor desync/death/degradation, short or permanent RC loss, RC corruption, telemetry loss, wind shifts, clock going backwards, scheduler jitter, dropped events, a stalled firmware task whose queue bursts afterwards, a reboot command mid-flight).

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
failsafe_disarms_on_touchdown  A failsafe landing disarms within 2 s of touching down.
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
failsafe_lands_softly          [flight] Failsafe touches down below 2.5 m/s and never disarms above 1 m.
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
link_recovers                  In failsafe with the link back and the switch on, failsafe ends within 1 s.
failsafe_holds                 [flight] The failsafe hold neither climbs nor sinks faster than 3 m/s² of acceleration.
estimate_reconverges           [flight] A tilt estimate off by over 15° (after a flip, say) is back within 10° within 4 s.
rate_tracks                    [flight] In an acro stick script, rates track the Betaflight-rate setpoint (30 ms lag) within 35% RMS (yaw 50%).
settles                        [flight] Hands off in an acro stick script, roll and pitch rates settle below 0.4 rad/s RMS.
sag_compensated                [flight] On a sagging pack the averaged motor speed matches the throttle stick within 15%.
```

**Stick scripts.** The virtual pilot sees the truth and closes the loop, which hides poor control. In `stick-script` segments it flies open loop instead: hands off, then a doublet on roll, pitch, and yaw in turn, then hands off. `rate_tracks` and `settles` judge those segments against Betaflight rates, which are written in the monitor and not read from the config.

**Paired runs** in `crates/ironloop-sim/tests/dst.rs` run the same flight twice with one thing changed that the firmware must not care about. Starting the clock below 2³² or at 2⁴⁰ µs, and mounting the IMU in another orientation with a matching configuration, must give bit-identical motor commands. The mount pairing only uses flights where the gyro never clips, because the sensor's range is asymmetric by one LSB. A different heading or an IMU at 1.6, 3.2, or 8 kHz must give the same properties, arming, and altitude within 15%. A snapshot of the whole simulation taken mid-flight must resume to the same trace.

**Coverage.** A property that never sees its situation passes without testing anything. Each property counts the steps on which its precondition was live, and the sweep prints in how many seeds that happened, how many seeds had each fault kind start in flight, and the share of simulated time spent airborne. Sweeps of 200 seeds or more fail (`THIN`) when a property is live in fewer than two seeds, a fault kind never starts in flight, or less than a quarter of the time is airborne.

**Metamorphic tests** in `crates/ironloop-fc/src/tests.rs` check the mixer against Betaflight's QuadX table and mirror every input left-right and front-back and require mirrored motor outputs. They need no oracle and catch anything that treats the two sides differently, such as one wrong mixer entry or two axes mixed up; a sign flipped consistently on a whole axis stays symmetric, and the DST catches those.

**Mutants.** `scripts/mutants.py` reintroduces 55 real flight-controller bugs and config mistakes, one at a time, and runs the DST sweep, the firmware unit tests, and the simulator tests against each. It first records what fails on the unmodified firmware, and counts a mutant as caught only when something new fails or more seeds fail a property.

**Known limits.**

- RC corruption that passes the CRC (about 1 in 256 corrupted frames) is injected on sticks and the mode switch but not on the arm channel. A single frame with the arm switch read as off disarms in flight, and the firmware cannot tell it from a real switch-off without delaying every disarm.
- A switch moved to on while the link is down arms on the first frame after the link returns. This is intended.
- The motor-fault properties need bidirectional-DShot eRPM, which the Mobula6 target does not implement yet; seeds without telemetry skip them.
- The CRSF parser is unit-tested but not in the DST loop; the simulator delivers decoded channels.
- No automatic disarm may come from how the quad is flown. Crash detection's hands-off rule (lying still, propped by about 1 g) also needs a jolt within 2 s before it, because an inverted fall at terminal speed reads exactly like lying on the ground. That jolt must exceed what thrust and drag can produce in flight, worked out from the learned hover throttle (thrust/weight ≈ 1/hover²): over thrust/weight + 1 g across or against the thrust axis (cutting the throttle after an inverted power dive brakes at that), and over 2 × thrust/weight + 1 g along it (pulling out of a dive). A crash without such a jolt, such as a slow tip-over, stops only when the sticks or angle mode demand a roll or pitch the frame does not follow. Runaway detection runs only until the build has flown once per power-up, and while the sticks are moved it counts only a frame speeding up away from the controller's push, never a stick reversal.
- `crates/ironloop-sim/FAILING_SEEDS.md` lists the seeds that failed a 200,000-seed overnight sweep, by property.
- The tilt estimate drifts past 15° after some stick slams and long aggressive sequences (22 of seeds 300–1799).
- The open-loop failsafe landing touches down at 2.5–2.7 m/s in about 1% of seeds (13 of seeds 300–1799), just over the 2.5 m/s limit. The vertical speed estimate from the accelerometer drifts by a m/s or more over a long landing.
- A few crash and failsafe cases are each seen once in seeds 300–1799: a dead motor not stopped within 4 s (445, `crash_stops`), a bent prop holding altitude in failsafe (1257, `failsafe_descends`), and a touchdown not recognised within 2 s (1762).
- `settles` catches ten times the D gain but not five times the P gain: the simulator's motors still lag less than real ones, so high P alone rarely rings.
- Eight of the 54 mutants survive a 300-seed sweep: hover learning during climbs, no gyro spike rejection, the dt clamp removed, I-term windup at zero throttle, three older crash-detection mutants that the 4 g impact gate now masks, and arming on a stale IMU (harmless here: the same step disarms for `ImuFailure` before any motor command).
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

**Not yet verified on hardware.** Before props go on, check on the bench: which IMU is fitted and its axis orientation (tilt the quad, watch attitude), motor order and direction, DShot reaching the ESCs, battery voltage scaling, and LED/beeper polarity. Then the timing checks, from the counters in `control::STATS` (IMU samples, missed data-ready edges, SPI read errors, dropped events, and `step` cycles from the DWT counter). SWD is unavailable, so they need the planned USB diagnostics to read them.

- IMU rate: samples grow at 3.2 kHz with a BMI270 or 2 kHz with an ICM-42688, and missed data-ready edges stay at zero.
- Step time: the longest `step` (`step_cycles_max`, at 96 MHz) stays well under the sample period (312 µs, 30,000 cycles, for the BMI270), including during stick moves, arming, and disarming.
- Dead data-ready line: with PA1 disconnected or held low, the quad must refuse to arm (IMU blocker). If it is disconnected while armed, the motors must stop within about 20 ms.

Bidirectional DShot telemetry, OSD over MSP DisplayPort, and USB are not implemented yet, so motor-fault detection has no input on hardware.
