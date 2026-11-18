//! Battery level and motor-fault notices.

use crate::{BatteryLevel, Effect, Notice, Out, State};

pub(crate) fn on_battery(s: &mut State, volts: f32, fx: &mut Out) {
    if !volts.is_finite() || volts < 0.0 {
        return;
    }
    let v = match s.battery_volts {
        // About 0.5 s time constant at the usual 50..100 Hz sample rate.
        Some(prev) => prev + 0.05 * (volts - prev),
        None => volts,
    };
    s.battery_volts = Some(v);
    let hyst = 0.05;
    let level = match s.battery_level {
        BatteryLevel::Ok if v < s.cfg.vbat_crit => BatteryLevel::Critical,
        BatteryLevel::Ok if v < s.cfg.vbat_warn => BatteryLevel::Warning,
        BatteryLevel::Warning if v < s.cfg.vbat_crit => BatteryLevel::Critical,
        BatteryLevel::Warning if v > s.cfg.vbat_warn + hyst => BatteryLevel::Ok,
        BatteryLevel::Critical if v > s.cfg.vbat_warn + hyst => BatteryLevel::Ok,
        BatteryLevel::Critical if v > s.cfg.vbat_crit + hyst => BatteryLevel::Warning,
        l => l,
    };
    if level != s.battery_level {
        s.battery_level = level;
        fx.push(Effect::Notice(Notice::Battery(level)));
    }
}

/// Judges each motor's eRPM against the speed its command should give.
pub(crate) fn check_motors(s: &mut State, erpm: &[Option<u32>; 4], fx: &mut Out) {
    if !s.armed {
        return;
    }
    let vcomp = s.battery_volts.map_or(1.0, |v| {
        (v / s.cfg.vbat_ref).clamp(0.6, 1.2)
    });
    for i in 0..4 {
        let Some(rpm) = erpm[i] else { continue };
        let expected = s.motor_frac[i] * s.cfg.motor_max_erpm * vcomp;
        let low = s.motor_frac[i] > 0.25 && (rpm as f32) < expected * s.cfg.motor_fault_frac;
        if low {
            let since = *s.flight.motor_fault_since_us[i].get_or_insert(s.now_us);
            if !s.flight.motor_faults[i] && s.now_us - since >= s.cfg.motor_fault_us {
                s.flight.motor_faults[i] = true;
                fx.push(Effect::Notice(Notice::MotorFault(i as u8)));
            }
        } else if (rpm as f32) >= expected * 0.6 || s.motor_frac[i] <= 0.25 {
            s.flight.motor_fault_since_us[i] = None;
            if s.flight.motor_faults[i] {
                s.flight.motor_faults[i] = false;
                fx.push(Effect::Notice(Notice::MotorRecovered(i as u8)));
            }
        }
    }
}
