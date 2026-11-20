//! Dynamic notch: without eRPM the RPM filter has nothing to place its
//! notches by, so this finds the strongest vibration tone in the roll and
//! pitch gyro and notches it, as Betaflight's dynamic notch does. A bent or
//! unbalanced prop shakes the frame at shaft rate, which a 150 Hz
//! first-order low-pass barely cuts.
//!
//! A sliding DFT over the last `N` samples tracks each bin's amplitude at a
//! fixed cost per sample. The bins are in cycles per window, so the notch is
//! placed in radians per sample with no need for the sample rate, and an
//! aliased tone is notched where it appears, as the RPM filter does.

use crate::math::Vec3;
use crate::rpm::{Notch, sin_cos};

/// Samples per window: bins 25-62.5 Hz wide at 1.6-4 kHz.
pub(crate) const N: usize = 64;
/// Highest bin searched: 0.45 of the sample rate, as for the RPM filter.
const TOP: usize = N * 45 / 100;
/// Damps the sliding DFT so rounding cannot build up (`R` per sample).
const R: f32 = 0.9999;
/// Samples between peak searches.
const SEARCH_EVERY: u8 = 8;
/// A peak this many times the mean of the searched bins counts as a tone;
/// broadband noise moves no notch.
const PEAK_OVER_MEAN: f32 = 6.0;
/// How fast `avg` follows each search's bin powers.
const AVG_GAIN: f32 = 0.125;

#[derive(Clone, Debug, PartialEq)]
pub struct DynNotch {
    /// `e^(i 2 pi k / N)` for each bin, `(cos, sin)`.
    twiddle: [(f32, f32); TOP + 1],
    /// The window's roll and pitch samples, oldest at `head`.
    window: [[f32; 2]; N],
    head: usize,
    /// Each bin's running DFT for roll and pitch, `(re, im)`.
    bins: [[(f32, f32); 2]; TOP + 1],
    /// Each bin's power averaged over searches, so a noise bin that is
    /// briefly strong does not move the notch.
    avg: [f32; TOP + 1],
    since_search: u8,
    /// Smoothed peak, in bins; 0 before the first tone is found.
    peak: f32,
    notch: Notch,
    last_input: Vec3,
}

impl Default for DynNotch {
    fn default() -> Self {
        let mut twiddle = [(1.0, 0.0); TOP + 1];
        for (k, t) in twiddle.iter_mut().enumerate() {
            let (s, c) = sin_cos(2.0 * core::f32::consts::PI * k as f32 / N as f32);
            *t = (c, s);
        }
        DynNotch {
            twiddle,
            window: [[0.0; 2]; N],
            head: 0,
            bins: [[(0.0, 0.0); 2]; TOP + 1],
            avg: [0.0; TOP + 1],
            since_search: 0,
            peak: 0.0,
            notch: Notch::OFF,
            last_input: Vec3::ZERO,
        }
    }
}

impl DynNotch {
    /// Filters `v`. `on` is false while the RPM filter has the motors'
    /// speeds; the analysis then stops and starts afresh when it is next
    /// needed. `min_bin` is the lowest bin notched (the minimum frequency
    /// times `N` times the sample period).
    pub fn apply(&mut self, v: Vec3, on: bool, q: f32, min_bin: f32) -> Vec3 {
        if !on {
            if self.peak != 0.0 || self.notch != Notch::OFF {
                *self = DynNotch { twiddle: self.twiddle, ..DynNotch::default() };
            }
            self.last_input = v;
            return v;
        }
        let rn = const { pow(R, N) };
        let old = self.window[self.head];
        self.window[self.head] = [v.x, v.y];
        self.head = (self.head + 1) % N;
        for (bin, &(c, s)) in self.bins.iter_mut().zip(&self.twiddle).skip(1) {
            for a in 0..2 {
                let re = bin[a].0 + [v.x, v.y][a] - rn * old[a];
                let im = bin[a].1;
                bin[a] = (R * (re * c - im * s), R * (re * s + im * c));
            }
        }
        self.since_search += 1;
        if self.since_search >= SEARCH_EVERY {
            self.since_search = 0;
            self.search(q, min_bin);
        }
        self.last_input = v;
        self.notch.apply(v)
    }

    /// Moves the notch to the strongest bin from `min_bin` up, if it
    /// stands out.
    fn search(&mut self, q: f32, min_bin: f32) {
        let lo = (libm::ceilf(min_bin) as usize).max(1);
        if lo + 2 > TOP {
            return;
        }
        let power = |b: &[(f32, f32); 2]| b[0].0 * b[0].0 + b[0].1 * b[0].1 + b[1].0 * b[1].0 + b[1].1 * b[1].1;
        let (mut best, mut best_p, mut sum) = (lo, 0.0, 0.0);
        for k in lo..=TOP {
            self.avg[k] += (power(&self.bins[k]) - self.avg[k]) * AVG_GAIN;
            let p = self.avg[k];
            sum += p;
            if p > best_p {
                (best, best_p) = (k, p);
            }
        }
        if best_p <= PEAK_OVER_MEAN * sum / (TOP + 1 - lo) as f32 {
            return;
        }
        // Where between the bins, from the complex values of the peak and
        // its neighbours (Jacobsen's estimator, for an unwindowed DFT), on
        // the axis where the tone is strongest.
        let a = usize::from(power(&[self.bins[best][1], (0.0, 0.0)]) > power(&[self.bins[best][0], (0.0, 0.0)]));
        let (l, m, r) = (self.bins[best - 1][a], self.bins[best][a], self.bins[(best + 1).min(TOP)][a]);
        let num = (l.0 - r.0, l.1 - r.1);
        let den = (2.0 * m.0 - l.0 - r.0, 2.0 * m.1 - l.1 - r.1);
        let dd = den.0 * den.0 + den.1 * den.1;
        let d = if dd > 0.0 { ((num.0 * den.0 + num.1 * den.1) / dd).clamp(-0.5, 0.5) } else { 0.0 };
        let at = best as f32 + d;
        self.peak = if self.peak == 0.0 { at } else { self.peak + (at - self.peak) * 0.5 };
        let (s, c) = sin_cos(2.0 * core::f32::consts::PI * self.peak / N as f32);
        self.notch.tune((s, c), q, self.last_input);
    }

    pub fn reset(&mut self, v: Vec3) {
        self.last_input = v;
        self.notch.reset(v);
    }
}

const fn pow(x: f32, n: usize) -> f32 {
    let mut y = 1.0;
    let mut i = 0;
    while i < n {
        y *= x;
        i += 1;
    }
    y
}

#[cfg(test)]
mod tests {
    use super::*;
    use libm::sinf;

    /// RMS of the roll output over the second half of `secs` of a sine at
    /// `hz` plus a slow 5 Hz move, sampled at `fs`, relative to the tone's.
    fn residual(f: &mut DynNotch, hz: f32, fs: f32) -> f32 {
        let n = (fs * 0.5) as usize;
        let (mut sum, tau) = (0.0, 2.0 * core::f32::consts::PI);
        for k in 0..2 * n {
            let t = k as f32 / fs;
            let slow = 0.5 * sinf(tau * 5.0 * t);
            let y = f.apply(Vec3::new(sinf(tau * hz * t) + slow, 0.3 * sinf(tau * hz * t + 1.0), 0.0), true, 3.0, 100.0 * N as f32 / fs);
            if k >= n {
                sum += (y.x - slow) * (y.x - slow);
            }
        }
        libm::sqrtf(sum / n as f32) * core::f32::consts::SQRT_2
    }

    #[test]
    fn finds_and_notches_a_tone_between_bins() {
        // 330 Hz at 3.2 kHz is bin 6.6; 1,100 Hz at 1.6 kHz appears at
        // 500 Hz.
        for (hz, fs) in [(330.0, 3200.0), (1100.0, 1600.0), (700.0, 4000.0)] {
            let r = residual(&mut DynNotch::default(), hz, fs);
            assert!(r < 0.15, "{hz} Hz at {fs}: {r}");
        }
    }

    #[test]
    fn follows_a_tone_that_moves() {
        let mut f = DynNotch::default();
        residual(&mut f, 250.0, 3200.0);
        let r = residual(&mut f, 420.0, 3200.0);
        assert!(r < 0.15, "{r}");
    }

    #[test]
    fn noise_or_off_places_no_notch() {
        let mut f = DynNotch::default();
        let mut x = 12345u32;
        for _ in 0..4000 {
            x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let n = (x >> 8) as f32 / (1u32 << 24) as f32 - 0.5;
            f.apply(Vec3::new(n, -n, 0.0), true, 3.0, 2.0);
        }
        assert_eq!(f.notch, Notch::OFF);
        let mut f = DynNotch::default();
        residual(&mut f, 330.0, 3200.0);
        let v = Vec3::new(0.3, 0.2, 0.1);
        assert_eq!(f.apply(v, false, 3.0, 2.0), v);
        assert_eq!(f.notch, Notch::OFF);
    }
}
