//! RPM filter: a notch on the gyro at each motor's rotation frequency and
//! its harmonics, from the ESCs' eRPM telemetry, as in Betaflight. Prop
//! imbalance shakes the frame at exactly those frequencies, so narrow
//! notches remove it with far less delay than a lower low-pass cutoff.
//!
//! The IMU samples at 1.6-8 kHz while a motor's harmonics reach several
//! kHz, so a harmonic above half the sample rate shows up aliased; the notch
//! goes where it actually appears.

use libm::{cosf, sinf};

use crate::math::Vec3;

pub const MAX_HARMONICS: usize = 3;

/// A second-order notch (RBJ cookbook) on all three axes, direct form II
/// transposed.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Notch {
    /// b0, b1, b2, a1, a2 (a0 = 1); `None` passes the input through.
    c: Option<[f32; 5]>,
    z1: Vec3,
    z2: Vec3,
}

impl Notch {
    const OFF: Notch = Notch { c: None, z1: Vec3::ZERO, z2: Vec3::ZERO };

    /// `input` is the latest gyro sample: a notch switching on starts as if
    /// it had been passing it all along, not from zero (which would kick the
    /// output by the whole gyro rate).
    fn tune(&mut self, hz: f32, sample_hz: f32, q: f32, min_hz: f32, input: Vec3) {
        let f = alias(hz, sample_hz);
        if !(min_hz..0.45 * sample_hz).contains(&f) {
            *self = Notch::OFF;
            return;
        }
        let w = 2.0 * core::f32::consts::PI * f / sample_hz;
        let (s, c) = (sinf(w), cosf(w));
        let alpha = s / (2.0 * q);
        let a0 = 1.0 + alpha;
        let was_off = self.c.is_none();
        self.c = Some([1.0 / a0, -2.0 * c / a0, 1.0 / a0, -2.0 * c / a0, (1.0 - alpha) / a0]);
        if was_off {
            self.reset(input);
        }
    }

    fn apply(&mut self, x: Vec3) -> Vec3 {
        let Some([b0, b1, b2, a1, a2]) = self.c else { return x };
        let y = x * b0 + self.z1;
        self.z1 = x * b1 - y * a1 + self.z2;
        self.z2 = x * b2 - y * a2;
        y
    }

    /// Steady state for a constant input `x` (the notch passes DC).
    fn reset(&mut self, x: Vec3) {
        if let Some([_, b1, b2, a1, a2]) = self.c {
            self.z2 = x * (b2 - a2);
            self.z1 = x * (b1 - a1) + self.z2;
        }
    }
}

/// Where a tone at `hz` appears when sampled at `sample_hz`.
fn alias(hz: f32, sample_hz: f32) -> f32 {
    let f = hz % sample_hz;
    if f > sample_hz / 2.0 { sample_hz - f } else { f }
}

/// Notches for four motors, `MAX_HARMONICS` each.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RpmFilter {
    notches: [[Notch; MAX_HARMONICS]; 4],
    last_input: Vec3,
}

impl Default for RpmFilter {
    fn default() -> Self {
        RpmFilter { notches: [[Notch::OFF; MAX_HARMONICS]; 4], last_input: Vec3::ZERO }
    }
}

impl RpmFilter {
    /// Places motor `i`'s notches for its rotation at `rotation_hz` (`None`:
    /// unknown, so off), the first `harmonics` of them.
    pub fn set_motor(&mut self, i: usize, rotation_hz: Option<f32>, sample_hz: f32, harmonics: usize, q: f32, min_hz: f32) {
        for (h, n) in self.notches[i].iter_mut().enumerate() {
            match rotation_hz {
                Some(f) if h < harmonics => n.tune(f * (h + 1) as f32, sample_hz, q, min_hz, self.last_input),
                _ => *n = Notch::OFF,
            }
        }
    }

    pub fn apply(&mut self, mut v: Vec3) -> Vec3 {
        self.last_input = v;
        for n in self.notches.iter_mut().flatten() {
            v = n.apply(v);
        }
        v
    }

    pub fn reset(&mut self, v: Vec3) {
        self.last_input = v;
        for n in self.notches.iter_mut().flatten() {
            n.reset(v);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// RMS output of the filter for a sine on the x axis after it settles.
    fn gain(f: &mut RpmFilter, hz: f32, sample_hz: f32) -> f32 {
        let (mut sum, n) = (0.0, 4000);
        for k in 0..2 * n {
            let x = sinf(2.0 * core::f32::consts::PI * hz * k as f32 / sample_hz);
            let y = f.apply(Vec3::new(x, 0.0, 0.0)).x;
            if k >= n {
                sum += y * y;
            }
        }
        libm::sqrtf(sum / n as f32) * core::f32::consts::SQRT_2
    }

    #[test]
    fn notches_each_harmonic_and_passes_flight_frequencies() {
        let fs = 3200.0;
        let mut f = RpmFilter::default();
        // 400 Hz rotation: harmonics at 400, 800, and 1200 Hz.
        f.set_motor(2, Some(400.0), fs, 3, 3.0, 100.0);
        for hz in [400.0, 800.0, 1200.0] {
            assert!(gain(&mut f.clone(), hz, fs) < 0.05, "{hz} Hz passes");
        }
        // Control frequencies pass almost untouched.
        assert!(gain(&mut f.clone(), 20.0, fs) > 0.99);
        // Only two harmonics asked for: the third passes.
        f.set_motor(2, Some(400.0), fs, 2, 3.0, 100.0);
        assert!(gain(&mut f.clone(), 1200.0, fs) > 0.9);
    }

    #[test]
    fn notch_goes_where_an_aliased_harmonic_appears() {
        // At 1.6 kHz a 1,100 Hz tone appears at 500 Hz.
        let fs = 1600.0;
        let mut f = RpmFilter::default();
        f.set_motor(0, Some(1100.0), fs, 1, 3.0, 100.0);
        assert!(gain(&mut f.clone(), 1100.0, fs) < 0.05);
        assert!(gain(&mut f, 500.0, fs) < 0.05);
    }

    #[test]
    fn unknown_slow_or_near_nyquist_motors_are_not_filtered() {
        let fs = 3200.0;
        let mut f = RpmFilter::default();
        f.set_motor(1, None, fs, 3, 3.0, 100.0);
        f.set_motor(3, Some(60.0), fs, 1, 3.0, 100.0);
        assert_eq!(f, RpmFilter::default());
        // 1,500 Hz is too close to the 1,600 Hz Nyquist limit to notch.
        f.set_motor(0, Some(1500.0), fs, 1, 3.0, 100.0);
        assert_eq!(f, RpmFilter::default());
    }

    #[test]
    fn reset_starts_from_a_steady_output() {
        let mut f = RpmFilter::default();
        f.set_motor(0, Some(300.0), 3200.0, 3, 3.0, 100.0);
        let v = Vec3::new(1.0, -2.0, 0.5);
        f.reset(v);
        for _ in 0..10 {
            let y = f.apply(v);
            assert!((y - v).max_abs() < 1e-5, "{y:?}");
        }
    }
}
