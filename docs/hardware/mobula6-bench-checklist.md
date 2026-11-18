# Mobula6 bench checklist

Checks to run on the real quad before props go on and before the first hover. They cover the README's "Not yet verified on hardware" list and the open items in [mobula6-eco-2024.md](mobula6-eco-2024.md). Tick a box by editing this file once the check passes on hardware.

**Setup:** flash IronLoop with `dfu-util` (README, "Flashing the Mobula6"). Open the USB serial port (`/dev/ttyACM0` on Linux, any baud). It prints one status line every 100 ms ([`usb.rs`](../../firmware/mobula6/src/usb.rs) has the format):

```text
imu 32004 miss 0 rderr 0 drop 0 cyc 4211/9876 erpm 0 0 0 0 armed 0 blk 0x24 vbat 4.08 rpy 1.2 -0.4 87.0 gyro 0.00 0.01 -0.00 mot 0 0 0 0
```

Arming blocker bits in `blk`: 0x01 calibrating, 0x02 throttle, 0x04 RC link, 0x08 IMU, 0x10 tilt, 0x20 battery.

## 1. Board comes up (props off, USB only)

- [ ] The USB serial port enumerates, and lines arrive every 100 ms.
- [ ] Note which IMU is fitted. `imu` should grow at about 3,200/s for a BMI270 or 2,000/s for an ICM-42688.
- [ ] `miss`, `rderr` and `drop` stay at 0.
- [ ] Status LED (PC13) blinks fast with blockers, slowly when ready, and is solid when armed. Beeper (PC15) sounds on failsafe and low battery. If either polarity is inverted, fix it in `firmware/mobula6/src/control.rs`.
- [ ] Flash chip JEDEC ID and size. IronLoop doesn't read it, so check it from Betaflight before flashing; nothing depends on it yet.

## 2. IMU orientation (on each IMU variant)

- [ ] On a **BMI270** quad: tilt nose down, then right side down, then rotate clockwise seen from above. `rpy` should show pitch going negative, roll going positive, and yaw increasing, and the matching `gyro` axis should move with each motion. Confirm these signs against the simulator before trusting them.
- [ ] Repeat on an **ICM-42688** quad. If the two chips disagree, add a per-chip alignment in `firmware/mobula6/src/imu.rs` (CW0 is assumed for both, from Happymodel's factory dump).
- [ ] With the quad level and still, `blk` loses 0x01 after about 1 s (gyro calibrated) and never shows 0x10 (tilt).

## 3. Timing budget

- [ ] The longest `step` (second number of `cyc`) stays well under 30,000 cycles, the BMI270's 312 µs sample period at 96 MHz: idle, while moving the sticks, and through arming and disarming.
- [ ] Check it again with eRPM coming in and motors spinning, since the RPM filter runs 12 notches per sample.
- [ ] Dead data-ready line: with PA1 disconnected or held low, `imu` stops growing, `blk` shows 0x08, and the quad refuses to arm. If PA1 is disconnected while armed, the motors stop within about 20 ms.

## 4. Battery

- [ ] On a battery, `vbat` matches a multimeter on the pack within about 0.05 V. If not, adjust `VBAT_DIVIDER` in `firmware/mobula6/src/main.rs`.
- [ ] The current reading matches a meter. Low priority: the firmware doesn't use current yet.
- [ ] On USB power only, `blk` shows 0x20 and the quad refuses to arm.

## 5. Motors and DShot (quad strapped down, props off)

- [ ] Arming spins all four motors at idle, and `mot` shows about 158 on each.
- [ ] Motor order: M1 rear-right, M2 front-right, M3 rear-left, M4 front-left. With the quad held and armed, tilting it right speeds up the right motors (M1, M2) as it fights back, and tilting it nose down speeds up the front motors (M2, M4).
- [ ] Spin directions match the props-out layout (`yaw_motors_reversed = ON` in the factory dump).
- [ ] `erpm` rises with throttle on every motor, toward about 72,000 RPM × 6 pole pairs at full throttle on a full pack.
- [ ] Holding one motor still while it is commanded hard reports a motor fault within about 60 ms.
- [ ] Disarming stops all motors at once.
- [ ] Record the Bluejay settings and whether Extended DShot Telemetry is available.

## 6. Before the first hover

- [ ] RC link loss with the quad armed on the bench: failsafe starts (beeper), then the quad disarms.
- [ ] The DFU stick gesture is ignored while armed, and reboots into DFU when disarmed.
- [ ] In a hover, motor outputs are visibly smoother with the RPM filter than with `rpm_filter_harmonics = 0`.
- [ ] Measure thrust and the eRPM curve with the stock 1208 props, to fit the simulator.
