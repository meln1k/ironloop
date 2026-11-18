//! What the firmware puts out and keeps on every step: motor commands in
//! range, stopped when disarmed, and sent often enough; a sane clock and
//! attitude; an output buffer big enough.

use ironloop_fc::{DSHOT_MAX, DSHOT_MIN, Effect, EventKind, Notice};

use super::req::*;
use super::{Cx, Imu, Property, Verdicts};
use crate::scenario::FaultKind;

pub static MOTOR_RANGE: Property = Property {
    name: "motor_range",
    doc: "Every motor command is 0 (stop) or in the DShot throttle range 48..=2047.",
};
pub static NO_SPIN_DISARMED: Property = Property {
    name: "no_spin_disarmed",
    doc: "While disarmed, the last motor command of every step is all zeros.",
};
pub static ARMED_ABOVE_IDLE: Property = Property {
    name: "armed_above_idle",
    doc: "While armed, no motor is ever commanded below 3% idle (a stopped motor in flight).",
};
pub static NO_BOOTLOADER_ARMED: Property = Property {
    name: "no_bootloader_armed",
    doc: "The firmware never jumps to the bootloader while armed.",
};
pub static DISARM_STOPS_MOTORS: Property = Property {
    name: "disarm_stops_motors",
    doc: "A step that disarms emits an all-zero motor command.",
};
pub static ESC_FED: Property = Property {
    name: "esc_fed",
    doc: "While armed the ESCs receive a command at least every 5 ms (unless the firmware task is stalled).",
};
pub static FINITE_STATE: Property = Property {
    name: "finite_state",
    doc: "Attitude, rates, and setpoints in the firmware status are always finite.",
};
pub static NO_FALSE_CLOCK_ALARM: Property = Property {
    name: "no_false_clock_alarm",
    doc: "The firmware reports the clock going backwards only when it really jumped back.",
};
pub static CLOCK_MONOTONIC: Property = Property {
    name: "clock_monotonic",
    doc: "The firmware's clock never goes backwards, whatever order events arrive in.",
};
pub static ATTITUDE_UNIT: Property = Property {
    name: "attitude_unit",
    doc: "The attitude quaternion stays unit length (within 1e-3).",
};
pub static EFFECTS_FIT: Property = Property {
    name: "effects_fit",
    doc: "No step ever produces more effects than its output buffer holds.",
};

#[derive(Clone, Debug, Default)]
pub struct Outputs {
    max_event_t: u64,
    last_motor_cmd_us: Option<u64>,
}

impl Outputs {
    pub fn step(&mut self, cx: &Cx, imu: &Imu, v: &mut Verdicts) {
        let (t, before, after) = (cx.t, cx.before, cx.after);

        v.live(&CLOCK_MONOTONIC);
        if after.now_us < before.now_us && before.calibrated {
            v.fail(t, &CLOCK_MONOTONIC, format!("clock went from {} to {} us", before.now_us, after.now_us));
        }
        v.live(&ATTITUDE_UNIT);
        let q = after.attitude;
        let norm = libm::sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        if (norm - 1.0).abs() > REQ_UNIT_QUAT {
            v.fail(t, &ATTITUDE_UNIT, format!("|q| = {norm:.5}"));
        }
        if cx.fx.len() >= 4 {
            v.live(&EFFECTS_FIT);
        }
        if after.effects_dropped > 0 {
            v.fail(t, &EFFECTS_FIT, format!("{} effects dropped", after.effects_dropped));
        }
        v.live(&FINITE_STATE);
        if !(after.attitude.is_finite() && after.gyro.is_finite() && after.rate_setpoint.is_finite() && after.throttle.is_finite()) {
            v.fail(t, &FINITE_STATE, format!("{after:?}"));
        }

        // Only a real jump back may raise the clock alarm. The firmware's
        // inputs can arrive slightly out of order (board queueing), which it
        // must absorb silently.
        let jumped = cx.sc.faults.iter().any(|f| matches!(f.kind, FaultKind::ClockGlitch { .. }) && t >= f.at_us && t < f.at_us.saturating_add(50_000));
        if cx.event.t_us < self.max_event_t {
            v.live(&NO_FALSE_CLOCK_ALARM);
        }
        self.max_event_t = self.max_event_t.max(cx.event.t_us);
        if cx.fx.contains(&Effect::Notice(Notice::ClockWentBackwards)) && !jumped {
            v.fail(t, &NO_FALSE_CLOCK_ALARM, "ClockWentBackwards without a clock jump".into());
        }

        let mut last_motors = None;
        for e in cx.fx {
            match e {
                Effect::Motors(m) => {
                    v.live(&MOTOR_RANGE);
                    if m.iter().any(|&x| x != 0 && !(DSHOT_MIN..=DSHOT_MAX).contains(&x)) {
                        v.fail(t, &MOTOR_RANGE, format!("{m:?}"));
                    }
                    if after.armed {
                        v.live(&ARMED_ABOVE_IDLE);
                        if m.iter().any(|&x| x < REQ_ARMED_MIN_DSHOT) {
                            v.fail(t, &ARMED_ABOVE_IDLE, format!("{m:?}"));
                        }
                    }
                    last_motors = Some(*m);
                    self.last_motor_cmd_us = Some(t);
                }
                Effect::RebootToBootloader if before.armed || after.armed => v.fail(t, &NO_BOOTLOADER_ARMED, String::new()),
                _ => {}
            }
        }
        if matches!(cx.event.kind, EventKind::Command(_)) && before.armed {
            v.live(&NO_BOOTLOADER_ARMED);
        }
        if !after.armed {
            if let Some(m) = last_motors {
                v.live(&NO_SPIN_DISARMED);
                if m != [0; 4] {
                    v.fail(t, &NO_SPIN_DISARMED, format!("{m:?}"));
                }
            }
        }
        if before.armed && !after.armed {
            v.live(&DISARM_STOPS_MOTORS);
            if last_motors != Some([0; 4]) {
                v.fail(t, &DISARM_STOPS_MOTORS, format!("last motors {last_motors:?}"));
            }
        }

        if after.armed {
            if t.saturating_sub(imu.last_us) > 2_000 {
                v.live(&ESC_FED);
            }
            if let Some(last) = self.last_motor_cmd_us {
                let stalled = cx
                    .sc
                    .faults
                    .iter()
                    .any(|f| matches!(f.kind, FaultKind::FcStall) && f.at_us <= t && f.end_us().saturating_add(1_000) >= last);
                if t - last > REQ_ESC_FEED_US && !stalled {
                    v.fail(t, &ESC_FED, format!("{} ms without a motor command", (t - last) / 1000));
                }
            }
        }
    }
}
