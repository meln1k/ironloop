//! Arming and disarming: arm only when it is safe, always when the pilot
//! can, and disarm only for a cause the truth supports. Judged against the
//! link and the IMU as they were before this step.

use ironloop_fc::{DisarmReason, FailsafeStage, ch};

use super::req::*;
use super::{Cx, Frame, Imu, Link, Property, Verdicts, disarm_reason};

pub static ARM_THROTTLE_LOW: Property = Property {
    name: "arm_throttle_low",
    doc: "Arming only happens on an RC frame with throttle at the bottom.",
};
pub static ARM_SWITCH_CYCLED: Property = Property {
    name: "arm_switch_cycled",
    doc: "Arming needs the arm switch seen off since boot (switch on at power-up never arms).",
};
pub static ARM_CALIBRATED: Property = Property {
    name: "arm_calibrated",
    doc: "Arming only happens after gyro calibration finished.",
};
pub static ARM_BATTERY: Property = Property {
    name: "arm_battery",
    doc: "Arming never happens on USB power (no battery).",
};
pub static ARM_LEVEL: Property = Property {
    name: "arm_level",
    doc: "Arming never happens on the ground with the quad tilted past 33 deg (on its side, upside down).",
};
pub static ARM_IMU_FRESH: Property = Property {
    name: "arm_imu_fresh",
    doc: "Arming never happens without good IMU data in the last 30 ms.",
};
pub static PILOT_CAN_ARM: Property = Property {
    name: "pilot_can_arm",
    doc: "[flight] In a flyable scenario, flipping the arm switch on with everything ready (level, still, pack above 3.4 V) arms.",
};
pub static SWITCH_DISARMS: Property = Property {
    name: "switch_disarms",
    doc: "A valid RC frame with the arm switch off always leaves the quad disarmed.",
};
pub static DISARM_JUSTIFIED: Property = Property {
    name: "disarm_justified",
    doc: "Every disarm has a cause the truth supports (switch, real link loss, stale IMU, crash, unflyable build).",
};

pub fn step(cx: &Cx, link: &Link, frame: &Frame, imu: &Imu, v: &mut Verdicts) {
    let (t, before, after, world) = (cx.t, cx.before, cx.after, cx.world);
    let imu_age = t.saturating_sub(imu.last_good_us);
    if let Some(ch) = frame.rc {
        if link.arm_on && !link.switch_seen_off {
            v.live(&ARM_SWITCH_CYCLED);
        }
        if ch[ch::ARM] < 1300 && before.armed {
            v.live(&SWITCH_DISARMS);
        }
    }

    if before.armed && !after.armed {
        v.stats.disarms += 1;
        if cx.airborne {
            v.live(&DISARM_JUSTIFIED);
        }
        let reason = disarm_reason(cx.fx);
        let justified = match reason {
            Some(DisarmReason::Switch) => link.last_arm_ch <= 1300,
            Some(DisarmReason::Failsafe) => before.failsafe != FailsafeStage::None,
            Some(DisarmReason::ImuFailure) => imu_age >= REQ_IMU_TOLERATE_US,
            // Stopping a quad that cannot fly (dead motor) is no false alarm.
            Some(DisarmReason::Crash) => (cx.alt < 0.3 && cx.true_tilt > REQ_TIPPED_OVER) || cx.sc.unflyable(),
            Some(DisarmReason::RunawayTakeoff) => cx.sc.unflyable(),
            None => false,
        };
        if !justified {
            v.fail(
                t,
                &DISARM_JUSTIFIED,
                format!("{reason:?} at {:.1} m, tilt {:.0} deg, IMU fresh {} ms ago", cx.alt, cx.true_tilt.to_degrees(), imu_age / 1000),
            );
        }
    }

    if !before.armed && after.armed {
        v.stats.arms += 1;
        match frame.rc {
            None => v.fail(t, &ARM_THROTTLE_LOW, format!("armed on non-RC event {:?}", cx.event.kind)),
            Some(ch) if ch[ch::THROTTLE] > 1050 => v.fail(t, &ARM_THROTTLE_LOW, format!("throttle {}", ch[ch::THROTTLE])),
            Some(_) => {}
        }
        if !link.switch_seen_off {
            v.fail(t, &ARM_SWITCH_CYCLED, "armed without the switch ever seen off".into());
        }
        if !after.calibrated {
            v.fail(t, &ARM_CALIBRATED, String::new());
        }
        if imu_age > REQ_ARM_IMU_FRESH_US {
            v.fail(t, &ARM_IMU_FRESH, format!("armed with no good IMU data for {} ms", imu_age / 1000));
        }
        if world.battery.absent {
            v.fail(t, &ARM_BATTERY, "armed on USB power".into());
        }
        // On the ground: re-arming in the air (after an IMU fault disarm)
        // is a recovery, not a tipped-over quad spinning up.
        if world.body.on_ground && cx.true_tilt > REQ_ARM_TILT {
            v.fail(t, &ARM_LEVEL, format!("true tilt {:.0} deg", cx.true_tilt.to_degrees()));
        }
    }

    let Some(ch) = frame.rc else { return };
    if frame.arm_edge && !before.armed {
        if ch[ch::THROTTLE] > 1050 {
            v.live(&ARM_THROTTLE_LOW);
        }
        if !before.calibrated {
            v.live(&ARM_CALIBRATED);
        }
        if imu_age > REQ_ARM_IMU_FRESH_US {
            v.live(&ARM_IMU_FRESH);
        }
        if world.battery.absent {
            v.live(&ARM_BATTERY);
        }
        if world.body.on_ground && cx.true_tilt > REQ_ARM_TILT {
            v.live(&ARM_LEVEL);
        }
        // Everything the firmware needs is really there: the pilot must be
        // able to arm.
        let ready = ch[ch::THROTTLE] < 1050
            && before.calibrated
            && world.battery.volts > 3.4
            && cx.true_tilt < 15f32.to_radians()
            && world.body.omega.norm() < 0.05
            && imu_age < 2_000
            && !cx.excused
            && !(cx.sc.moved_at_boot && t < 4_000_000);
        if ready {
            v.live(&PILOT_CAN_ARM);
            if !after.armed {
                v.fail(t, &PILOT_CAN_ARM, format!("arm refused, blockers {:?}", after.blockers));
            }
        }
    }
    if ch[ch::ARM] < 1300 && after.armed {
        v.fail(t, &SWITCH_DISARMS, "still armed after switch-off frame".into());
    }
}
