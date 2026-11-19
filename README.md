# IronLoop

A simulation-first quadrotor flight controller for the HDZero Mobula6 ECO 2024.

The whole firmware is one function of its state and an event:

```rust
pub fn step(state: &mut State, event: &Event) -> heapless::Vec<Effect, 8>
```

Events are timestamped IMU samples, RC frames, battery readings, ESC telemetry, 1 kHz ticks, and commands. Effects are DShot motor values, LED, beeper, notices, and "reboot to DFU". `step` never reads a clock, touches hardware, or allocates. The same code runs on the STM32F411, in the deterministic simulator, and in the browser.

| Path | What it is |
| --- | --- |
| `crates/ironloop-fc` | `no_std` firmware core: estimator, acro/angle rate PID, mixer, RPM filter, arming, failsafe, crash/runaway detection, motor-fault detection from eRPM, CRSF and DShot codecs |
| `crates/ironloop-sim` | Deterministic simulation testing: physics, ESCs, battery, wind, noisy IMU, RC link, virtual pilot, seeded workload generator, property monitor, `dst` runner |
| `crates/ironloop-viz` | Bevy app: fly the firmware by hand, inject faults live, or replay a DST seed. Builds natively and for wasm/WebGL2 |
| `firmware/mobula6` | Embassy board target (STM32F411CEU6): IMU on SPI1 (BMI270 or ICM-42688-P), ELRS CRSF on USART2, ADC battery, bit-banged bidirectional DShot300 with eRPM via TIM1 + DMA2 |
| `scripts/` | `build-web.sh`, `serve-web.py`, `mutants.py`, `failing-seeds.py` |

## Flight modes

- **Angle** (mode switch, channel 6, low): sticks command tilt up to 55°, self-levels.
- **Acro** (channel 6 high): sticks command body rates with Betaflight rates (rc_rate 1.0, super 0.7 → 667 °/s at full stick).

Airmode engages above 25% throttle. Arming (channel 5 high) needs: gyro calibrated, throttle low, RC link up, IMU healthy, tilt under 25°, a battery (not USB power), and the switch seen off since power-up.

## Deterministic simulation testing

```sh
cargo test -p ironloop-fc -p ironloop-sim                   # unit tests + 24-seed DST sweep
(cd firmware/mobula6 && cargo build --release)              # board firmware (needs thumbv7em-none-eabihf)
cargo run --release -p ironloop-sim --bin dst -- --seeds 1000
cargo run --profile dst -p ironloop-sim --bin dst -- --seeds 100000   # long sweeps: fat LTO, ~6% faster, slower to build
cargo run --release -p ironloop-sim --bin dst -- --seed 626 --trace   # replay one seed with a 20 Hz flight log
cargo run --release -p ironloop-sim --bin dst -- --properties
scripts/mutants.py 300                                      # every reintroduced bug must fail a check (--quick: stop each at its first catch)
scripts/failing-seeds.py 4000                               # regenerate crates/ironloop-sim/FAILING_SEEDS.md
```

Every event is on one time-ordered queue; physics advances on a fixed 250 µs grid; each subsystem draws from its own seeded RNG stream. A seed replays bit for bit (the runner re-runs a sample and compares full input/output trace hashes).

**Workload generator** (per seed): mass, inertia, thrust and motor-lag tolerances; battery charge, internal resistance (0.03–0.15 Ω, fresh to worn 1S packs), LiHV (4.35 V full, as the quad ships with) or LiPo, and the electronics' draw (1.2–2 A, most of it the HDZero VTX), and the board's ±3% voltage reading error; how many cycles each instruction of the flight code takes on the board (1.1–1.8); IMU rate, jitter, noise, bias, which chip it is (BMI270 at 3.2 or 1.6 kHz, ICM-42688 at 2 or 4 kHz), the chip's own low-pass delay (300–750 Hz; the loop delay that makes high gains ring), warm-up drift, scale error, vibration (prop imbalance at shaft speed, blade pass at three times it, and a frame resonance at 150–600 Hz, each passed through the chip's anti-alias filters at its true frequency and then aliased), bent props, and the IMU mounted in any of four orientations with a matching configuration; RC rate and jitter; wind and gusts; ESCs that start a motor open-loop for 50–250 ms before reporting eRPM, limit acceleration at low speed (Bluejay's RPM power protection), and lose sync below 3–6% of full speed, restarting 100 ms later up to three times; ESCs with or without bidirectional-DShot telemetry (eRPM goes through the same GCR reply codec the board decodes, so it carries the ESC's period resolution, and bit errors fail its checksum); a clock that starts just below 2³² µs so a 32-bit microsecond counter would wrap mid-flight; build mistakes (IMU configured a quarter turn off, one IMU axis read with the wrong sign or the chip upside down, two motor wires swapped or the motor connector turned a quarter, every motor spinning the wrong way, USB-only power); handling (arm switch on at boot, receiver binding late, quad moved during calibration, sitting on its side); pilot plans (hover, cruise, flips, punch-outs, stick slams, inverted hangs of up to 5 s that fall at terminal speed, drops, landings, mode switches, arming with throttle up); and 0–3 faults placed while the quad is flying (IMU dropouts from 1 to 60 ms, stuck or garbage samples, collision torque, hard impacts that step the gyro by tens of rad/s, motor desync/death/degradation, short or permanent RC loss, RC corruption, telemetry loss, wind shifts, clock going backwards, scheduler jitter, dropped events, a stalled firmware task whose queue bursts afterwards, a reboot command mid-flight).

The simulated board behaves like the real one: IMU samples are stamped at data-ready and stepped after the SPI read (15–45 µs). One control task reads the IMU and steps every event, and each step takes CPU time: the instructions `step` was counted to need per event on a Cortex-M4 under QEMU (IMU sample 3.5–7.1k, eRPM 1–2k, others 0.3–0.5k), plus 1k for the loop and the DShot frame, at the scenario's cycles per instruction and 96 MHz. The motor command goes out when its step is done. While the task is busy, RC, battery and telemetry events wait in its 32-deep queue, a data-ready edge replaces an unread one (the sample is lost, as the board's `drdy_missed` counts), and its 1 ms tick waits and is stamped when the task gets to it; a stalled task does the same for as long as it stalls. RC frames reach the firmware as bytes: the sim encodes each one as an ExpressLRS receiver sends it, with link-statistics frames in between, and the board's own `CrsfParser` decodes them. A sag below the receiver's brownout voltage (2.6–2.85 V; an ESP8285 behind the board's regulator) reboots the receiver, and no RC frames come for 0.5–1.5 s.

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
arm_level                      Arming never happens on the ground tilted past 33°, as the IMU reads gravity.
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
estimate_tracks                [flight] Estimated tilt stays within 15° of the truth while flying, except within 4 s of the gyro saturating.
angle_stays_level              [flight] In angle mode the true tilt stays under 75°.
failsafe_lands_softly          [flight] Failsafe touches down below 2.5 m/s vertical and never disarms above 1 m.
no_crash                       [flight] Armed touchdowns are slower than 2.5 m/s, outside drops, flips, stick slams.
pilot_can_arm                  [flight] Flipping the switch on with everything ready (level, still, pack above 3.4 V) arms.
arm_imu_fresh                  Arming never happens without good IMU data in the last 30 ms.
clock_monotonic                The firmware's clock never goes backwards, whatever order events arrive in.
attitude_unit                  The attitude quaternion stays unit length (within 1e-3).
effects_fit                    No step produces more effects than its output buffer holds.
no_panic                       The firmware never panics; its overflow checks are on in the DST build.
runaway_stops                  A quad whose rate loop pushes an axis the wrong way (motors in the wrong order or spinning the wrong way, an IMU axis flipped) never climbs above 1 m and stops within 5 s of throttle-up.
battery_notice_true            Battery warning/critical only when the pack really was below 3.62 / 3.45 V in the last 2 s.
motor_fault_true               A motor fault notice only for a motor really stalled or weakened in the last 300 ms.
link_recovers                  In failsafe with the link back without a gap for 1 s and the switch on, failsafe ends.
failsafe_holds                 [flight] The failsafe hold neither climbs nor sinks faster than 3 m/s² of acceleration.
estimate_reconverges           [flight] A tilt estimate off by over 15° (after a flip, say) is back within 10° within 4 s.
rate_tracks                    [flight] In an acro stick script, rates track the Betaflight-rate setpoint (30 ms lag) within 35% RMS (yaw 50%).
settles                        [flight] Hands off in an acro stick script, roll and pitch rates settle below 0.4 rad/s RMS.
sag_compensated                [flight] On a sagging pack the averaged motor speed matches the throttle stick within 15%.
angle_tracks                   [flight] In an angle-mode stick script, roll and pitch stick holds tilt the quad 75–115% of the commanded angle on average (55° at full stick).
```

**Stick scripts.** The virtual pilot sees the truth and closes the loop, which hides poor control. In `stick-script` segments it flies open loop instead: hands off, then a doublet on roll, pitch, and yaw in turn, then hands off. In acro, `rate_tracks` and `settles` judge those segments against Betaflight rates; in angle mode, `angle_tracks` judges how far each roll and pitch stick tilts the quad against a 55° angle limit. Both are written in the monitor and not read from the config, which the closed-loop pilot uses and so compensates for.

**Paired runs** in `crates/ironloop-sim/tests/dst.rs` run the same flight twice with one thing changed that the firmware must not care about. Starting the clock below 2³² or at 2⁴⁰ µs, and mounting the IMU in another orientation with a matching configuration, must give bit-identical motor commands. The mount pairing only uses flights where the gyro never clips, because the sensor's range is asymmetric by one LSB. A different heading must give the same properties, arming, and altitude within 15%; an IMU at 1.6, 3.2, or 8 kHz the same properties, arming, and whether it flew. A snapshot of the whole simulation taken mid-flight must resume to the same trace.

**Coverage.** A property that never sees its situation passes without testing anything. Each property counts the steps on which its precondition was live, and the sweep prints in how many seeds that happened, how many seeds had each fault kind start in flight, and the share of simulated time spent airborne. Sweeps of 200 seeds or more fail (`THIN`) when a property is live in fewer than two seeds, a fault kind never starts in flight, or less than a quarter of the time is airborne.

**Metamorphic tests** in `crates/ironloop-fc/src/tests.rs` check the mixer against Betaflight's QuadX table and mirror every input left-right and front-back and require mirrored motor outputs. They need no oracle and catch anything that treats the two sides differently, such as one wrong mixer entry or two axes mixed up; a sign flipped consistently on a whole axis stays symmetric, and the DST catches those.

**Mutants.** `scripts/mutants.py` reintroduces 67 real flight-controller bugs and config mistakes, one at a time, and runs the DST sweep, the firmware unit tests, and the simulator tests against each, and type-checks the board firmware. It first records what fails on the unmodified firmware, and counts a mutant as caught only when something new fails: a test, or a property on a seed where it held before (`dst --list-failures` lists every failing seed).

**Known limits.**

- RC corruption that passes the CRC (about 1 in 256 corrupted frames on a real link; the sim makes half of its corrupted frames this kind) is injected on sticks and the mode switch but not on the arm channel. A single frame with the arm switch read as off disarms in flight, and the firmware cannot tell it from a real switch-off without delaying every disarm.
- A switch moved to on while the link is down arms on the first frame after the link returns. This is intended.
- The motor-fault properties need bidirectional-DShot eRPM; seeds without telemetry (40%) skip them, and the rest of the properties must hold either way. The core also uses eRPM for an RPM filter (notches at each motor's first three harmonics, then a 200 Hz rather than 150 Hz gyro low-pass); without telemetry it keeps the plain 150 Hz low-pass, behind a dynamic notch that finds the strongest roll and pitch tone above 100 Hz with a 64-sample sliding DFT and follows it, as Betaflight's does. Without it, a bent prop on ESCs without telemetry jittered the motors past `motors_quiet` (seed 7861). The sim vibrates at 1× and 2× shaft speed (prop imbalance) and 3× (blade pass of the tri-blade props), plus a frame resonance, and `motors_quiet` bounds the motor-command jitter they cause in steady flight. It catches the dynamic notch switched off (seed 7861). Before the notch it also caught the RPM filter switched off, a 500 Hz gyro low-pass, and a 250 Hz D-term low-pass; it no longer does (not in 300 seeds, 1,500 for the D-term), because the seeds that caught them were bent props without eRPM just under the limit, which the notch now keeps well under it, and without the RPM filter the notch and the 150 Hz low-pass take its place. Nor does it catch the third notch removed: in steady flight the blade pass aliases either high, where the low-passes take it out, or low on the ICM-42688, where the chip's own anti-alias filter has already cut it by about 45 dB. The amplitudes and the resonance are guesses until a raw-gyro capture from the real quad replaces them.
- The ESC model (startup time, slowest speed it keeps sync at, rampup cap) is drawn from guessed ranges, with the slowest speed set so the factory idle (6–10%) runs on a charged pack. At the 10% idle no seed stalls (`no_esc_stall`); at 5.5% a motor stalls in 68 of seeds 0–299, mostly at idle on a sagging pack, which the `motor idle 5.5%` mutant checks. The firmware's motor-fault check only judges motors commanded above 25%, so a motor stalled at idle is not reported until the throttle comes up.
- RC frames go through the board's CRSF parser, but not through its UART: the sim hands the parser whole frames on time, so a UART overrun (which resets the parser) is not modelled. Noise on the wire flips bits, which the CRC rejects; corruption that passes the CRC is drawn as a valid 11-bit value (880–2159 µs) on a stick or the mode switch.
- The CPU cost of each step is drawn from QEMU instruction counts, not measured on the board, and is not tied to the firmware's code: a change that makes `step` slower does not make the sim's step slower. At the drawn costs the control task is busy for at most about 200 µs at a time and loses about one IMU sample in 70,000, mostly on the ICM-42688 at 4 kHz; at five times the cost the quad still flies, losing most samples at that rate. Interrupts (the DShot DMA, data-ready, USB) take no time in the sim.
- Only the receiver browns out in the sim: the MCU (1.7 V minimum), the IMU, and the ESCs (2.2 V) hold on below any sag the simulated pack reaches (2.6 V worst in seeds 0–299). A board reset in flight is not simulated. The brownout voltage and the receiver's reboot time are estimates until the bench-supply sweep measures them.
- No automatic disarm may come from how the quad is flown. Crash detection's hands-off rule (lying still, propped by about 1 g, for 3 s) also needs a jolt within 2 s before it or a roll-over on the ground, because an inverted fall at terminal speed reads exactly like lying on the ground. The jolt must be over 4 g and exceed what thrust and drag can produce in flight by 15%, worked out from the learned hover throttle (thrust/weight ≈ 1/hover²): over thrust/weight + 1 g across or against the thrust axis (cutting the throttle after an inverted power dive brakes at that), and over 2 × thrust/weight + 1 g along it (pulling out of a dive). A roll-over is a frame that tipped past 60° without the vertical-speed estimate showing more than 2 m/s of fall: a plunge only reads like lying after falling several m/s. A crash with neither, such as landing on its back after a fall faster than 2 m/s without a hard enough hit, stops only when the sticks or angle mode demand a roll or pitch the frame does not follow. Runaway detection runs only until the build has flown once per power-up: it must lift off (the climb estimate shows it rising at 0.5 m/s while upright), then fly 1 s upright above 25% throttle with the PIDs in range. Sitting on the ground at part throttle does not count, because the ground holds even a build that cannot fly still and level. While the sticks are moved it counts only a frame speeding up away from the controller's push, never a stick reversal. A gyro clipped at its range shows no acceleration either way, so a clip the controller pushes against for 0.5 s also counts (dips under the range shorter than 0.1 s, from vibration or a bounce, do not end it): full stick asks for a third of the range, and a tumble on the ground clips it for about 0.1 s. Yaw is judged on its own, at its lower output limit: the yaw output held at its limit against the yaw rate for 0.25 s is a runaway, since a build that flies stops even a full-stick yaw within about 0.1 s that way. Sooner, a yaw rate that doubles within 50 ms while the yaw output pushes against it at over half its limit is one too: a build that flies slows whatever it pushes against, and comparing across the window keeps out the vibration and bounces that make its acceleration too noisy to judge. At full throttle a reversed build otherwise climbs about a meter in the half second its yaw takes to spin up. Motors that all spin the wrong way, or two of opposite spin swapped, show up only there.
- `crates/ironloop-sim/FAILING_SEEDS.md` lists every seed of 0–3999 that fails a property, by property, with the commit it was generated on. `scripts/failing-seeds.py` regenerates it; seed numbers are only valid for that commit. Its last section, kept by hand, lists rare cases from a 1M-seed sweep with a status each, to pick up and fix.
- The open-loop failsafe landing touches down at about 1.45 m/s (median) and 1.85 m/s (90th percentile) in seeds 0–3999. One fails `failsafe_lands_softly` (seed 171, 2.7 m/s): its pack is nearly empty and sags to about 2.8 V under load, so there is too little thrust left to brake. Without a barometer the vertical speed estimate can be up to a m/s or so off for the whole landing, so the landing probes for descent by taking throttle off until it sinks 0.5 m/s faster; braking above hover only while the descent is still speeding up keeps those probes from adding up. The learned hover throttle is kept in reference-voltage terms, so the failsafe also holds on a pack sagged below where sag compensation is capped.
- When the link drops during a climb, the speed estimate can read over a m/s of sink too much for the whole landing, and the brake then holds the quad at under 0.2 m/s of sink (927 at 18 m, `failsafe_descends`). Seed 1257 is worse: the link drops 17 m up in a 3.4 m/s climb, and the failsafe landing keeps climbing at about 2 m/s for over 8 s, to 37 m, until the link returns. This climb-rate bias is the failsafe's largest open gap.
- `settles` catches ten times the D gain but not five times the P gain: the simulator's motors still lag less than real ones, so high P alone rarely rings.
- Four of the 62 mutants survived a 300-seed sweep before the dynamic notch, which adds "RPM filter off" and "D-term low-pass at 250 Hz" (see above). Three change nothing reachable: the dt clamp removed (gaps over 5 ms already reset the filters, and the D-term low-pass cancels dt below 2 ms), lying counted from 10° of tilt (the propped check already needs about 50°), and arming on a stale IMU (the same step disarms for `ImuFailure` before any motor command). One is a gap: lying without the accelerometer check, because in the flights the sim makes the impact and roll-over rules already rule out the same cases. Hover learning during climbs is caught by a single seed (`failsafe_descends`), so a smaller or different sweep can miss it.
- In flight the accelerometer reads mostly thrust, which looks level whatever the tilt while the quad speeds up or slows down. In a sustained push in angle mode (about 2.5 m/s² for over a second) its apparent vertical leans about 14° at almost exactly 1 g, the tilt estimate follows it, and angle mode tilts further (seeds 42, 89 and 283 of 0–999 fail `estimate_tracks`, each just past 15°). Betaflight has the same limit without GPS; without a velocity sensor there is no clean fix, and a lower accelerometer gain made other seeds worse. The estimator's gyro-bias integral learns slowly (0.15) for the same reason: at 0.3 one push taught it 0.07 rad/s of bias that was not there, which kept turning the estimate after the push (seed 6018); halving it cut `estimate_tracks` failures in seeds 0-29999 from 87 to 54. After the gyro saturates, the estimate also takes a few seconds to come back while the pilot punches out; `estimate_tracks` leaves those 4 s to `estimate_reconverges`, and the estimator pulls hard on clean 1 g readings while the quad hardly turns. Disarmed and still for 250 ms, it snaps to the accelerometer, as Betaflight does after a crash.
- A nearly empty, worn pack with the electronics' 1.2–2 A on top sags to about 2.8 V at 80% throttle and cannot hold a hover. Seed 1324 then loses sync on a motor at 3.33 V (`no_esc_stall`). A real pack in that state is past its critical warning, which the pilot was already given. (Such a quad never proves it flies, so runaway detection stays on while it skids and bounces along the ground; the yaw-growth check once took a bounce's spin-up for a runaway, and now starts its window over after a yaw acceleration no motor can produce.)
- Arming checks the estimated tilt but not that the quad is still, so a pilot can re-arm a frame that is still tumbling while the estimate is off. Seed 146 (IMU misaligned, so it runs away) was once re-armed while bouncing at 10 rad/s with the estimate reading 21° against a true 35°, and failed `arm_level`; since the virtual pilot got its own random stream no seed of 0–3999 hits it, but the gap remains. Requiring stillness would also block a deliberate re-arm in mid-air to save a fall. The exception is a runaway disarm: that build has not shown it flies, so there is no fall to save, and arming waits until it lies still and the estimate has snapped to the accelerometer (a flipped IMU axis tumbles with the estimate mirrored).

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

The board gives the firmware's `Config` the gyro and accelerometer ranges it programs into the IMU; the mounting is the default CW0 for both chips, from Happymodel's factory dump. A panic, or the control loop missing the independent watchdog for about 50 ms (34–94 ms over the LSI clock's tolerance), resets the board; the ESCs stop on the missing signal, and arming afterwards needs the arm switch seen off first.

**Not yet verified on hardware.** The step-by-step list, with what to look for in the USB status line, is in [`docs/hardware/mobula6-bench-checklist.md`](docs/hardware/mobula6-bench-checklist.md). Before props go on, check on the bench: which IMU is fitted and its axis orientation, on a BMI270 board and on an ICM-42688 board (tilt the quad, watch attitude), motor order and direction, DShot reaching the ESCs, battery voltage scaling, and LED/beeper polarity. Then the timing checks, from the counters in `control::STATS` (IMU samples, missed data-ready edges, SPI read errors, dropped events, and `step` cycles from the DWT counter). SWD is unavailable, so the board prints them on a USB serial port (CDC-ACM, any baud) as one line per 100 ms, along with eRPM per motor, arming blockers, battery voltage and current, attitude, gyro, motor commands and faults, the RC channels, which IMU was found, and why the board last reset (power-on, watchdog, brownout, panic). The firmware's notices (every disarm and refused arm with its reason, motor faults, battery warnings) come out as their own lines between them; see `firmware/mobula6/src/usb.rs` for the format.

- IMU rate: samples grow at 3.2 kHz with a BMI270 or 2 kHz with an ICM-42688, and missed data-ready edges stay at zero.
- Step time: the longest `step` plus applying its effects (`step_cycles_max`, at 96 MHz) stays well under the sample period (312 µs, 30,000 cycles, for the BMI270), including during stick moves, arming, and disarming.
- Dead data-ready line: with PA1 disconnected or held low, the quad must refuse to arm (IMU blocker). If it is disconnected while armed, the motors must stop within about 20 ms.

- eRPM: with the quad strapped down and props off, spin each motor up from idle. Its eRPM should rise with throttle, reaching about 72,000 RPM × 6 pole pairs at full throttle on a full pack, and a motor held still while commanded hard must be reported as faulty within about 60 ms. The board always sends inverted (bidirectional) DShot, which the stock Bluejay 0.19 ESCs answer; ESCs reflashed to BLHeli_S will not spin until Bluejay is flashed back.
- RPM filter: with eRPM coming in, `step_cycles_max` must still fit the sample period (the filter runs 12 notches per sample and retunes them at every eRPM reply). Without eRPM, the same for the dynamic notch, which updates 28 DFT bins on two axes per sample. In a hover, motor outputs should be visibly smoother than with `rpm_filter_harmonics = 0`.

OSD over MSP DisplayPort is not implemented yet, and the USB port only prints (the status line and notices); it takes no commands.

## Future work

- [ ] **Browser setup and calibration guide over WebUSB, assisted by an LLM.** Plug a stock quad into the browser and the page does the rest:
  1. Back up the stock firmware first: read the whole flash over the ROM DFU bootloader and keep it, plus the Betaflight CLI dump, so the quad can always go back.
  2. Flash IronLoop over WebUSB DFU.
  3. Walk the builder through setup and calibration with an LLM that reads live telemetry and explains each step. That means the bench checks above: IMU orientation (tilt the quad and watch the attitude; a quarter-turn mismatch is the build mistake DST seed 146 flies with), motor order and direction, battery voltage scaling, LED and beeper, and eRPM. It should end with a config the builder has confirmed.

  - [ ] **Commands over USB.** Everything below needs the board to take commands, not just print: a small line protocol on the existing CDC-ACM port (or a WebUSB interface beside it). At least `motor N value` (one motor at a time, disarmed only, refused while the arm switch is on or the RC link is up with throttle raised, with a timeout that stops the motor if the browser goes quiet), `dfu` (reboot into the ROM bootloader, disarmed only, so the browser can flash without the BOOT pad or the stick gesture), `gyro N` (stream N raw samples at the full IMU rate for the vibration checks), and the config values the guide sets (`motor_idle`, battery scales, IMU alignment). Commands reach the core as `Event::Command`, so the DST can run them.

  - [ ] **Motor order and direction test.** Disarmed and props off, spin one motor at a time (a `motor N value` command over USB), and have the builder confirm which corner moved and which way it turned. Check that the eRPM on that output rises, so each pin's ESC answers. A wrong order or a reversed spin is a build mistake the DST flies (`motors-swapped-*`, `motors-rotated`, `spin-reversed`), and runaway detection only stops it after takeoff.

  - [ ] **Idle and startup check.** Props on and the quad strapped down, arm and hold idle on a full pack and again near 3.5 V. Have the builder confirm that no prop spins and stops, check that every motor's eRPM is steady at idle and reaches it within about 0.3 s of arming, and raise `motor_idle` if one stalls. The DST's ESC model guesses the slowest speed Bluejay keeps sync at (3–6% of full); this measures it.

  - [ ] **Frame resonance: tap test and motor sweep.** Disarmed, props off, battery strapped on, ask the builder to flick the frame and record 1 s of raw gyro at the full IMU rate; fit the ring-down for the resonance's frequency and Q. Then ramp each motor slowly, props off, and mark where the gyro jumps as a motor harmonic crosses the resonance. Feed both into the DST's frame model, which today draws them from guessed ranges (150–600 Hz, Q 5–20).

  - [ ] **CPU budget.** Props off, strapped down, with the USB port open: spin all four motors through the throttle range and read `cyc` (longest step, of 30,000 cycles at 3.2 kHz) and `miss` (lost samples). Check that eRPM keeps arriving on all four, which also shows the USB interrupt does not starve the DShot reply. Fail the setup if `cyc` is near the budget or `miss` climbs; feed the measured cycles into the DST's cost model, which today uses QEMU instruction counts at a guessed 1.1–1.8 cycles each.

  - [ ] **Battery calibration.** Compare `vbat` and `amps` with a meter at plug-in and with the motors spinning, and set the voltage and current scales from the difference. Ask which pack is used (LiHV or LiPo) and its capacity.

  - [ ] **Brownout sweep.** On a bench supply instead of a pack, with the motors spinning props off, step the voltage down from 4.2 V to 2.5 V. Record where the receiver drops the link (`blk` 0x04 and a lost-link note), how long it takes to come back, and where the board itself resets (`rst brownout`). Feed both into the DST, which guesses the receiver browns out at 2.6–2.85 V and is back in 0.5–1.5 s.

  - [ ] **Radio check.** Have the builder move each stick and switch in turn and confirm on screen: the channel order (AETR), the direction of each stick, throttle low reading under about 1050 µs, and the arm and mode switches crossing 1700 µs and 1300 µs. Then switch the transmitter off with the quad armed on the bench (props off): the failsafe must start and then disarm, with its notes, and arming must need the switch cycled after the link comes back.

  - [ ] **Raw-gyro capture in a tethered hover.** Stream 1–2 s of raw gyro and accelerometer samples at the full IMU rate (about 40 kB/s) with each motor's eRPM, and show the spectrum: the 1×, 2× and 3× (blade pass) lines per motor, any frame resonance, and what the RPM filter leaves. Feed the measured amplitudes and resonance back into the DST's vibration model, which today are guesses.

  This needs the commands above; today the board only prints the status line.

- [ ] **USB interrupt priority.** `usb.rs` binds OTG_FS without setting a priority, so it probably runs at P0, the NVIC default, beside the DShot DMA and data-ready interrupts. Its handler can then delay the DShot ISR's switch to input and clip the ESC's eRPM reply. Set it below the control loop (P1) and check on the bench that eRPM keeps arriving on all four motors with the port open (the CPU budget check above).

- [ ] **CRSF telemetry for the first flights.** USB needs a cable, so a tethered hover is blind except for the LED and the beeper. Send telemetry back to the radio on PA2 (USART2 TX; DMA1 stream 6 channel 4 is free): battery (0x08), attitude (0x1E), and a flight-mode string (0x21) carrying the arming blocker, the last disarm reason, the longest step, and lost IMU samples. ELRS sends telemetry at a few frames per second, so keep it to what the pilot needs to read on the radio.
