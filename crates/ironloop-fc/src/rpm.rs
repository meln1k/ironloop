//! RPM filter: a notch on the gyro at each motor's rotation frequency and
//! its harmonics, from the ESCs' eRPM telemetry, as in Betaflight. Prop
//! imbalance shakes the frame at exactly those frequencies, so narrow
//! notches remove it with far less delay than a lower low-pass cutoff.
//!
//! The IMU samples at 1.6-8 kHz while a motor's harmonics reach several
//! kHz, so a harmonic above half the sample rate shows up aliased; the notch
//! goes where it actually appears.

use crate::math::Vec3;

pub const MAX_HARMONICS: usize = 3;

/// A second-order notch (RBJ cookbook) on all three axes, direct form II
/// transposed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Notch {
    /// b0, b1, b2, a1, a2 (a0 = 1); `None` passes the input through.
    c: Option<[f32; 5]>,
    z1: Vec3,
    z2: Vec3,
}

impl Notch {
    pub(crate) const OFF: Notch = Notch { c: None, z1: Vec3::ZERO, z2: Vec3::ZERO };

    /// `input` is the latest gyro sample: a notch switching on starts as if
    /// it had been passing it all along, not from zero (which would kick the
    /// output by the whole gyro rate).
    /// `(s, c)` are the sine and cosine of the notch frequency in radians
    /// per sample, 0..pi.
    pub(crate) fn tune(&mut self, (s, c): (f32, f32), q: f32, input: Vec3) {
        let alpha = s / (2.0 * q);
        let a0 = 1.0 + alpha;
        let was_off = self.c.is_none();
        self.c = Some([1.0 / a0, -2.0 * c / a0, 1.0 / a0, -2.0 * c / a0, (1.0 - alpha) / a0]);
        if was_off {
            self.reset(input);
        }
    }

    pub(crate) fn apply(&mut self, x: Vec3) -> Vec3 {
        let Some([b0, b1, b2, a1, a2]) = self.c else { return x };
        let y = x * b0 + self.z1;
        self.z1 = x * b1 - y * a1 + self.z2;
        self.z2 = x * b2 - y * a2;
        y
    }

    /// Steady state for a constant input `x` (the notch passes DC).
    pub(crate) fn reset(&mut self, x: Vec3) {
        if let Some([_, b1, b2, a1, a2]) = self.c {
            self.z2 = x * (b2 - a2);
            self.z1 = x * (b1 - a1) + self.z2;
        }
    }
}

/// Sine and cosine of `x` in 0..=pi, as single-precision polynomials. The
/// Cortex-M4's FPU has no double precision, and `libm`'s `sinf` and `cosf`
/// work in `f64`, so each call there is a few hundred emulated operations.
/// The error here is below 1e-6.
pub(crate) fn sin_cos(x: f32) -> (f32, f32) {
    // sin x = cos y and cos x = -sin y for y = x - pi/2 in -pi/2..=pi/2,
    // where the Taylor series to y^12 converges well within f32 precision.
    let y = x - core::f32::consts::FRAC_PI_2;
    let y2 = y * y;
    let sin_y = y * (1.0 + y2 * (-1.0 / 6.0 + y2 * (1.0 / 120.0 + y2 * (-1.0 / 5040.0 + y2 * (1.0 / 362_880.0 + y2 * (-1.0 / 39_916_800.0))))));
    let cos_y = 1.0 + y2 * (-0.5 + y2 * (1.0 / 24.0 + y2 * (-1.0 / 720.0 + y2 * (1.0 / 40_320.0 + y2 * (-1.0 / 3_628_800.0 + y2 * (1.0 / 479_001_600.0))))));
    (cos_y, -sin_y)
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
        let Some(f) = rotation_hz else {
            self.notches[i] = [Notch::OFF; MAX_HARMONICS];
            return;
        };
        // One sine and cosine per motor: harmonic h + 1 is h + 1 times the
        // fundamental's angle per sample, so angle addition gives the rest.
        // Aliasing mirrors an angle to 2 pi minus it, which keeps its cosine
        // and flips its sine, so the notch takes the sine's magnitude.
        let w1 = sin_cos(2.0 * core::f32::consts::PI * alias(f, sample_hz) / sample_hz);
        let mut w = w1;
        for (h, n) in self.notches[i].iter_mut().enumerate() {
            let at = alias(f * (h + 1) as f32, sample_hz);
            if h < harmonics && (min_hz..0.45 * sample_hz).contains(&at) {
                n.tune((w.0.abs(), w.1), q, self.last_input);
            } else {
                *n = Notch::OFF;
            }
            w = (w.0 * w1.1 + w.1 * w1.0, w.1 * w1.1 - w.0 * w1.0);
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
    use libm::sinf;

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
        // Harmonics fold too: at 3.2 kHz a 680 Hz rotation's third harmonic,
        // 2,040 Hz, appears at 1,160 Hz; at 1.6 kHz a 1,100 Hz rotation's
        // second, 2,200 Hz, appears at 600 Hz.
        let mut f = RpmFilter::default();
        f.set_motor(0, Some(680.0), 3200.0, 3, 3.0, 100.0);
        for hz in [680.0, 1360.0, 2040.0, 1160.0] {
            assert!(gain(&mut f.clone(), hz, 3200.0) < 0.05, "{hz} Hz passes");
        }
        f.set_motor(0, Some(1100.0), 1600.0, 2, 3.0, 100.0);
        assert!(gain(&mut f.clone(), 2200.0, 1600.0) < 0.05);
        assert!(gain(&mut f, 200.0, 1600.0) > 0.9);
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
