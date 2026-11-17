//! RC channels in µs to pilot inputs, and back. `step` decodes every RC
//! frame here; the board only parses the wire format (`proto`), and the
//! simulator encodes its pilot's sticks with [`RcInput::channels`].

use crate::{FlightMode, RC_CHANNELS};

/// Channel order (AETR, as sent by ExpressLRS over CRSF).
pub mod ch {
    pub const ROLL: usize = 0;
    pub const PITCH: usize = 1;
    pub const THROTTLE: usize = 2;
    pub const YAW: usize = 3;
    /// High arms (see [`super::Switch::arm_on`]).
    pub const ARM: usize = 4;
    /// Low or centered selects Angle, high selects Acro.
    pub const MODE: usize = 5;
}

/// Any channel outside this range (µs) marks the frame as corrupted.
pub const VALID_US: core::ops::RangeInclusive<u16> = 880..=2120;
/// Stick center and half travel (µs).
pub const STICK_CENTER_US: f32 = 1500.0;
pub const STICK_HALF_SPAN_US: f32 = 500.0;
/// Throttle bottom and full travel (µs).
pub const THROTTLE_MIN_US: f32 = 1000.0;
pub const THROTTLE_SPAN_US: f32 = 1000.0;
/// Stick deflection (fraction of half travel) read as centered.
pub const DEADBAND: f32 = 0.01;
/// A switch is high above this and low at or below `SWITCH_LOW_US` (µs).
pub const SWITCH_HIGH_US: u16 = 1700;
pub const SWITCH_LOW_US: u16 = 1300;
/// Stick deflection that counts as full for the bootloader gesture.
const GESTURE_STICK: f32 = 0.8;

/// A switch channel's position.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Switch {
    Low,
    Mid,
    High,
}

impl Switch {
    fn decode(us: u16) -> Switch {
        if us > SWITCH_HIGH_US {
            Switch::High
        } else if us > SWITCH_LOW_US {
            Switch::Mid
        } else {
            Switch::Low
        }
    }

    fn us(self) -> u16 {
        match self {
            Switch::Low => 1000,
            Switch::Mid => 1500,
            Switch::High => 2000,
        }
    }

    /// The arm switch with hysteresis, so a noisy channel cannot chatter:
    /// it turns on only high and off only low.
    pub fn arm_on(self, was_on: bool) -> bool {
        if was_on { self != Switch::Low } else { self == Switch::High }
    }
}

/// What the pilot asks for in one RC frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RcInput {
    /// Roll, pitch and yaw sticks in -1..1 (right, forward, right).
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    /// Throttle in 0..1.
    pub throttle: f32,
    pub arm: Switch,
    pub mode: FlightMode,
}

impl RcInput {
    /// Decodes a frame, or `None` when any channel is out of range.
    pub fn decode(channels: &[u16; RC_CHANNELS]) -> Option<RcInput> {
        if channels.iter().any(|c| !VALID_US.contains(c)) {
            return None;
        }
        let stick = |us: u16| {
            let x = ((us as f32 - STICK_CENTER_US) / STICK_HALF_SPAN_US).clamp(-1.0, 1.0);
            if x.abs() < DEADBAND { 0.0 } else { x }
        };
        Some(RcInput {
            roll: stick(channels[ch::ROLL]),
            pitch: stick(channels[ch::PITCH]),
            yaw: stick(channels[ch::YAW]),
            throttle: ((channels[ch::THROTTLE] as f32 - THROTTLE_MIN_US) / THROTTLE_SPAN_US).clamp(0.0, 1.0),
            arm: Switch::decode(channels[ch::ARM]),
            mode: if Switch::decode(channels[ch::MODE]) == Switch::High { FlightMode::Acro } else { FlightMode::Angle },
        })
    }

    /// The channels (µs) that decode to these inputs, unused ones centered.
    pub fn channels(&self) -> [u16; RC_CHANNELS] {
        let stick = |x: f32| (STICK_CENTER_US + STICK_HALF_SPAN_US * x.clamp(-1.0, 1.0)) as u16;
        let mut c = [STICK_CENTER_US as u16; RC_CHANNELS];
        c[ch::ROLL] = stick(self.roll);
        c[ch::PITCH] = stick(self.pitch);
        c[ch::YAW] = stick(self.yaw);
        c[ch::THROTTLE] = (THROTTLE_MIN_US + THROTTLE_SPAN_US * self.throttle.clamp(0.0, 1.0)) as u16;
        c[ch::ARM] = self.arm.us();
        c[ch::MODE] = match self.mode {
            FlightMode::Acro => Switch::High.us(),
            FlightMode::Angle => Switch::Low.us(),
        };
        c
    }

    /// The bootloader stick gesture: arm switch off, throttle low, yaw
    /// full left, pitch full back, roll full right.
    pub fn boot_gesture(&self) -> bool {
        self.arm == Switch::Low
            && self.throttle < crate::THROTTLE_LOW
            && self.yaw < -GESTURE_STICK
            && self.pitch < -GESTURE_STICK
            && self.roll > GESTURE_STICK
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn channels_decode_back_to_the_same_inputs() {
        for (roll, pitch, yaw, throttle, arm, mode) in [
            (0.0, 0.0, 0.0, 0.0, Switch::Low, FlightMode::Angle),
            (1.0, -1.0, 0.5, 1.0, Switch::High, FlightMode::Acro),
            (-0.3, 0.7, -0.9, 0.42, Switch::Mid, FlightMode::Angle),
        ] {
            let input = RcInput { roll, pitch, yaw, throttle, arm, mode };
            let back = RcInput::decode(&input.channels()).unwrap();
            assert_eq!((back.arm, back.mode), (arm, mode));
            for (a, b) in [(back.roll, roll), (back.pitch, pitch), (back.yaw, yaw), (back.throttle, throttle)] {
                assert!((a - b).abs() <= 0.0025, "{input:?} decoded as {back:?}");
            }
        }
    }

    #[test]
    fn decoding_matches_the_pulse_widths() {
        let mut c = [1500u16; RC_CHANNELS];
        c[ch::ROLL] = 1750;
        c[ch::PITCH] = 1504; // inside the deadband
        c[ch::YAW] = 988;
        c[ch::THROTTLE] = 1250;
        c[ch::ARM] = 1300;
        c[ch::MODE] = 1701;
        let i = RcInput::decode(&c).unwrap();
        assert_eq!((i.roll, i.pitch, i.yaw, i.throttle), (0.5, 0.0, -1.0, 0.25));
        assert_eq!((i.arm, i.mode), (Switch::Low, FlightMode::Acro));
        c[ch::MODE] = 1700;
        assert_eq!(RcInput::decode(&c).unwrap().mode, FlightMode::Angle);
        c[ch::YAW] = 879;
        assert_eq!(RcInput::decode(&c), None);
    }

    #[test]
    fn arm_switch_turns_on_high_and_off_low() {
        assert!(!Switch::Mid.arm_on(false));
        assert!(Switch::High.arm_on(false));
        assert!(Switch::Mid.arm_on(true));
        assert!(!Switch::Low.arm_on(true));
    }
}
