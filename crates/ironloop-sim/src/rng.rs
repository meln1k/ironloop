//! Small deterministic PRNG (SplitMix64). Pure integer math, so a seed gives
//! the same stream on every platform, including wasm.

use libm::{cosf, expf, logf, sqrtf};

#[derive(Clone, Debug)]
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Rng(seed ^ 0x9E37_79B9_7F4A_7C15)
    }

    /// An independent stream derived from this seed and a label, so adding
    /// draws to one subsystem does not shift the others.
    pub fn fork(seed: u64, label: &str) -> Self {
        let mut h = 0xcbf2_9ce4_8422_2325u64 ^ seed;
        for b in label.bytes() {
            h = (h ^ b as u64).wrapping_mul(0x100_0000_01b3);
        }
        let mut r = Rng(h);
        r.next_u64();
        r
    }

    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in [0, 1).
    pub fn f32(&mut self) -> f32 {
        (self.next_u64() >> 40) as f32 / (1u64 << 24) as f32
    }

    pub fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * self.f32()
    }

    pub fn range_u64(&mut self, lo: u64, hi: u64) -> u64 {
        if hi <= lo { lo } else { lo + self.next_u64() % (hi - lo) }
    }

    pub fn chance(&mut self, p: f32) -> bool {
        self.f32() < p
    }

    pub fn sign(&mut self) -> f32 {
        if self.chance(0.5) { 1.0 } else { -1.0 }
    }

    /// Standard normal (Box-Muller).
    pub fn normal(&mut self) -> f32 {
        let u1 = self.f32().max(1e-7);
        let u2 = self.f32();
        sqrtf(-2.0 * logf(u1)) * cosf(2.0 * core::f32::consts::PI * u2)
    }

    /// Standard normal by Marsaglia and Tsang's ziggurat: a few times
    /// cheaper than `normal`, which the scenario generator keeps using so
    /// scenarios do not change. A different stream from `normal` for the
    /// same state.
    pub fn gaussian(&mut self) -> f32 {
        let z = &*ZIGGURAT;
        loop {
            // The low 7 bits pick the layer, the top 32 the signed position.
            let u = self.next_u64();
            let i = (u & 127) as usize;
            let j = (u >> 32) as u32 as i32;
            let x = j as f32 * z.w[i];
            if j.unsigned_abs() < z.k[i] {
                return x;
            }
            if i == 0 {
                // The tail beyond R, by Marsaglia's method.
                loop {
                    let x = -logf(1.0 - self.f32()) / ZIG_R as f32;
                    let y = -logf(1.0 - self.f32());
                    if y + y >= x * x {
                        return if j > 0 { ZIG_R as f32 + x } else { -(ZIG_R as f32) - x };
                    }
                }
            }
            if z.f[i] + self.f32() * (z.f[i - 1] - z.f[i]) < expf(-0.5 * x * x) {
                return x;
            }
        }
    }

    pub fn pick<'a, T>(&mut self, items: &'a [T]) -> &'a T {
        &items[(self.next_u64() % items.len() as u64) as usize]
    }
}

/// Where the ziggurat's base layer's tail starts, and each layer's area.
const ZIG_R: f64 = 3.442_619_855_899;
const ZIG_V: f64 = 9.912_563_035_262_17e-3;

/// The ziggurat's 128 layers: `k` the position below which a sample is
/// inside the curve, `w` the scale from a 32-bit position to x, `f` the
/// density at each layer's edge. Built with libm, so the same everywhere.
struct Ziggurat {
    k: [u32; 128],
    w: [f32; 128],
    f: [f32; 128],
}

static ZIGGURAT: std::sync::LazyLock<Ziggurat> = std::sync::LazyLock::new(|| {
    let m = 2147483648.0f64;
    let (mut k, mut w, mut f) = ([0u32; 128], [0f32; 128], [0f32; 128]);
    let mut d = ZIG_R;
    let mut t = d;
    let q = ZIG_V / libm::exp(-0.5 * d * d);
    k[0] = ((d / q) * m) as u32;
    k[1] = 0;
    w[0] = (q / m) as f32;
    w[127] = (d / m) as f32;
    f[0] = 1.0;
    f[127] = libm::exp(-0.5 * d * d) as f32;
    for i in (1..=126).rev() {
        d = libm::sqrt(-2.0 * libm::log(ZIG_V / d + libm::exp(-0.5 * d * d)));
        k[i + 1] = ((d / t) * m) as u32;
        t = d;
        f[i] = libm::exp(-0.5 * d * d) as f32;
        w[i] = (d / m) as f32;
    }
    Ziggurat { k, w, f }
});

#[cfg(test)]
mod tests {
    use super::*;

    /// The ziggurat must give a standard normal: its moments, and the
    /// share of samples past each of several points, including far out in
    /// the tail it draws separately.
    #[test]
    fn gaussian_is_standard_normal() {
        let n = 4_000_000;
        let mut r = Rng::new(9);
        let (mut m1, mut m2, mut m4) = (0f64, 0f64, 0f64);
        let edges = [0.5f64, 1.0, 2.0, 3.0, 3.442_62, 4.0];
        let mut past = [0u32; 6];
        for _ in 0..n {
            let x = r.gaussian() as f64;
            m1 += x;
            m2 += x * x;
            m4 += x * x * x * x;
            for (p, e) in past.iter_mut().zip(edges) {
                *p += (x.abs() > e) as u32;
            }
        }
        let n = n as f64;
        assert!((m1 / n).abs() < 0.002, "mean {}", m1 / n);
        assert!((m2 / n - 1.0).abs() < 0.003, "variance {}", m2 / n);
        assert!((m4 / n - 3.0).abs() < 0.03, "kurtosis {}", m4 / n);
        // P(|X| > e) for a standard normal.
        let want = [0.617_075_1, 0.317_310_5, 0.045_500_26, 0.002_699_796, 5.759e-4, 6.334e-5];
        for ((p, w), e) in past.iter().zip(want).zip(edges) {
            let got = *p as f64 / n;
            let sd = (w * (1.0 - w) / n).sqrt();
            assert!((got - w).abs() < 5.0 * sd, "P(|x| > {e}) = {got}, want {w}");
        }
    }
}
