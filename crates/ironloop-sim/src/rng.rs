//! Small deterministic PRNG (SplitMix64). Pure integer math, so a seed gives
//! the same stream on every platform, including wasm.

use libm::{cosf, logf, sqrtf};

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

    pub fn pick<'a, T>(&mut self, items: &'a [T]) -> &'a T {
        &items[(self.next_u64() % items.len() as u64) as usize]
    }
}
