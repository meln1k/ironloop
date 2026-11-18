//! A lost RC link: failsafe starts in time and not before, adds no throttle
//! on the ground, levels, holds, descends, lands softly, disarms, and gives
//! control back when the link returns.

use ironloop_fc::{DisarmReason, FailsafeStage};

use super::req::*;
use super::{Cx, Frame, Link, Property, Verdicts, disarm_reason, flight_fault};

pub static FAILSAFE_ENTERS: Property = Property {
    name: "failsafe_enters",
    doc: "Armed with RC silent 150 ms is failsafe; silent under 50 ms never is.",
};
pub static FAILSAFE_NO_SPINUP: Property = Property {
    name: "failsafe_no_spinup",
    doc: "If the throttle was at the bottom when the link was lost, failsafe adds none.",
};
pub static FAILSAFE_BOUNDED: Property = Property {
    name: "failsafe_bounded",
    doc: "RC lost while armed ends with motors stopped within 35 s.",
};
pub static FAILSAFE_LEVELS: Property = Property {
    name: "failsafe_levels",
    doc: "[flight] One second into failsafe the quad is within 20 deg of level.",
};
pub static FAILSAFE_HOLDS: Property = Property {
    name: "failsafe_holds",
    doc: "[flight] In the failsafe hold the quad neither climbs nor sinks faster than 3 m/s² of acceleration.",
};
pub static FAILSAFE_DESCENDS: Property = Property {
    name: "failsafe_descends",
    doc: "[flight] Six seconds into a failsafe landing the quad sinks at 0.2 m/s or more.",
};
pub static FAILSAFE_DISARMS_ON_TOUCHDOWN: Property = Property {
    name: "failsafe_disarms_on_touchdown",
    doc: "A failsafe landing disarms within 2 s of touching down upright.",
};
pub static FAILSAFE_LANDS_SOFTLY: Property = Property {
    name: "failsafe_lands_softly",
    doc: "[flight] A failsafe landing touches down below 2.5 m/s vertical and never disarms above 1 m.",
};
pub static LINK_RECOVERS: Property = Property {
    name: "link_recovers",
    doc: "Armed in failsafe, with the link back without a gap for 1 s and the switch on, failsafe ends.",
};

/// The vertical-acceleration probe over the failsafe hold's second half.
#[derive(Clone, Copy, Debug, Default)]
enum HoldProbe {
    /// Not yet 0.3 s into the hold.
    #[default]
    Waiting,
    /// Vertical speed when the measurement started.
    Started(f32),
    /// Measured once this hold.
    Done,
}

#[derive(Clone, Debug, Default)]
pub struct Failsafe {
    /// The throttle was at the bottom when the link went.
    idle_at_loss: bool,
    rc_lost_since: Option<u64>,
    failsafe_since: Option<u64>,
    hold: HoldProbe,
    landing_since: Option<u64>,
    not_sinking_since: Option<u64>,
    landed_since: Option<u64>,
}

impl Failsafe {
    pub fn step(&mut self, cx: &Cx, link: &Link, frame: &Frame, v: &mut Verdicts) {
        let (t, before, after, world, alt) = (cx.t, cx.before, cx.after, cx.world, cx.alt);
        let flyable_now = !cx.sc.unflyable() && !flight_fault(cx.sc, t);

        if before.armed && !after.armed && disarm_reason(cx.fx) == Some(DisarmReason::Failsafe) && alt > 1.0 && flyable_now {
            v.fail(t, &FAILSAFE_LANDS_SOFTLY, format!("failsafe disarmed at {alt:.1} m"));
        }

        let link_back = t.saturating_sub(link.last_rc_valid_us) <= REQ_NO_FAILSAFE_BEFORE_US && link.last_arm_ch > 1700;
        if before.armed && before.failsafe != FailsafeStage::None && link_back {
            v.live(&LINK_RECOVERS);
            if after.armed && after.failsafe != FailsafeStage::None && t - link.rc_run_since >= REQ_RECOVER_US {
                v.fail(t, &LINK_RECOVERS, format!("still in {:?} with the link back for {} ms", after.failsafe, (t - link.rc_run_since) / 1000));
            }
        }

        // Airmode may lift total throttle to keep corrections in range; what
        // the failsafe must not do is ask for throttle of its own. Judge by
        // the throttle when the link went: frames that arrive during the
        // recovery window do not change what failsafe may do.
        if before.failsafe == FailsafeStage::None && after.failsafe != FailsafeStage::None {
            self.idle_at_loss = link.last_rc_throttle < 1050;
            if frame.silent_before < REQ_NO_FAILSAFE_BEFORE_US && frame.rc.is_none() {
                v.fail(t, &FAILSAFE_ENTERS, format!("failsafe after only {} ms of silence", frame.silent_before / 1000));
            }
        }
        if after.failsafe != FailsafeStage::None && self.idle_at_loss {
            v.live(&FAILSAFE_NO_SPINUP);
            if after.throttle_demand > 0.0 {
                v.fail(t, &FAILSAFE_NO_SPINUP, format!("failsafe demands {:.2} after link lost at idle", after.throttle_demand));
            }
        }
        let silent = t.saturating_sub(link.last_rc_valid_us);
        if after.armed && silent > REQ_FAILSAFE_AFTER_US {
            v.live(&FAILSAFE_ENTERS);
            v.live(&FAILSAFE_BOUNDED);
            if after.failsafe == FailsafeStage::None {
                v.fail(t, &FAILSAFE_ENTERS, format!("RC silent {} ms, no failsafe", silent / 1000));
            }
            let since = *self.rc_lost_since.get_or_insert(link.last_rc_valid_us);
            if t - since > REQ_FAILSAFE_END_US {
                v.fail(t, &FAILSAFE_BOUNDED, format!("still armed {} ms after RC loss", (t - since) / 1000));
            }
        } else if silent <= REQ_NO_FAILSAFE_BEFORE_US {
            self.rc_lost_since = None;
        }

        if after.armed && after.failsafe != FailsafeStage::None {
            let since = *self.failsafe_since.get_or_insert(t);
            if cx.airborne && flyable_now && t - since > REQ_FAILSAFE_LEVEL_US {
                v.live(&FAILSAFE_LEVELS);
                if cx.true_tilt > REQ_FAILSAFE_LEVEL_TILT {
                    v.fail(t, &FAILSAFE_LEVELS, format!("tilt {:.0} deg {} ms into failsafe", cx.true_tilt.to_degrees(), (t - since) / 1000));
                }
            }
        } else {
            self.failsafe_since = None;
        }

        if after.armed && after.failsafe == FailsafeStage::Hold && !self.idle_at_loss {
            // The hold flies at the learned hover throttle: measure the
            // vertical acceleration over its second half.
            let into = self.failsafe_since.map_or(0, |s| t - s);
            self.hold = match self.hold {
                _ if into < 300_000 => HoldProbe::Waiting,
                HoldProbe::Waiting => HoldProbe::Started(world.body.vel.z),
                HoldProbe::Started(vz0) if into >= 900_000 && alt > 1.0 && flyable_now => {
                    v.live(&FAILSAFE_HOLDS);
                    let a = (world.body.vel.z - vz0) / (into as f32 * 1e-6 - 0.3);
                    if a.abs() > REQ_HOLD_ACCEL {
                        v.fail(t, &FAILSAFE_HOLDS, format!("vertical acceleration {a:.1} m/s2 in the hold"));
                    }
                    HoldProbe::Done
                }
                probe => probe,
            };
        }

        if after.armed && after.failsafe == FailsafeStage::Landing {
            let since = *self.landing_since.get_or_insert(t);
            let sink = world.body.vel.z;
            if alt > 1.0 && flyable_now && t - since > REQ_FAILSAFE_SINK_AFTER_US {
                v.live(&FAILSAFE_DESCENDS);
                if sink < REQ_FAILSAFE_MIN_SINK {
                    let ns = *self.not_sinking_since.get_or_insert(t);
                    if t - ns > 1_000_000 {
                        v.fail(t, &FAILSAFE_DESCENDS, format!("sinking {sink:.2} m/s at {alt:.1} m"));
                    }
                } else {
                    self.not_sinking_since = None;
                }
            }
            // Upright; a quad that tipped over landing is crash_stops' case.
            if world.body.on_ground && alt < 0.1 && cx.true_tilt < REQ_TIPPED_OVER {
                v.live(&FAILSAFE_DISARMS_ON_TOUCHDOWN);
                let landed = *self.landed_since.get_or_insert(t);
                if t - landed > REQ_FAILSAFE_TOUCHDOWN_DISARM_US {
                    v.fail(t, &FAILSAFE_DISARMS_ON_TOUCHDOWN, format!("armed on the ground {} ms after touchdown", (t - landed) / 1000));
                }
            } else {
                self.landed_since = None;
            }
        } else {
            self.landing_since = None;
            self.not_sinking_since = None;
            self.landed_since = None;
        }

        if let Some((_, vz)) = cx.touchdown {
            let landing = before.failsafe == FailsafeStage::Landing || after.failsafe == FailsafeStage::Landing;
            if landing && flyable_now {
                v.live(&FAILSAFE_LANDS_SOFTLY);
                if vz > REQ_SOFT_TOUCHDOWN {
                    v.fail(t, &FAILSAFE_LANDS_SOFTLY, format!("touched down at {vz:.1} m/s vertical"));
                }
            }
        }
    }
}
