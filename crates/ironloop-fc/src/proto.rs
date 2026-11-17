//! Wire-protocol codecs used by the board target: CRSF (ExpressLRS RC
//! link) and DShot (ESC commands). Pure byte/bit manipulation, so they are
//! tested on the host like the rest of the core.

use crate::RC_CHANNELS;

/// CRSF frame type carrying 16 packed 11-bit RC channels.
pub const CRSF_RC_CHANNELS_PACKED: u8 = 0x16;
const CRSF_MAX_FRAME: usize = 64;

/// CRSF CRC-8 (polynomial 0xD5, DVB-S2) over the type byte and payload.
pub fn crsf_crc8(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 { (crc << 1) ^ 0xD5 } else { crc << 1 };
        }
    }
    crc
}

/// CRSF channel value (172..1811, center 992) to pulse width in µs
/// (988..2012, center 1500), as ExpressLRS and Betaflight map it.
pub fn crsf_to_us(v: u16) -> u16 {
    (1500 + ((v as i32 - 992) * 5) / 8).clamp(0, u16::MAX as i32) as u16
}

/// Incremental CRSF parser: feed it every received byte; it returns the
/// first [`RC_CHANNELS`] channels in µs for each valid RC channels frame.
/// Bad lengths and CRC failures drop the frame and resynchronize.
#[derive(Clone, Debug)]
pub struct CrsfParser {
    buf: [u8; CRSF_MAX_FRAME],
    len: usize,
    pub crc_errors: u32,
}

impl Default for CrsfParser {
    fn default() -> Self {
        Self { buf: [0; CRSF_MAX_FRAME], len: 0, crc_errors: 0 }
    }
}

impl CrsfParser {
    pub fn push(&mut self, b: u8) -> Option<[u16; RC_CHANNELS]> {
        // Sync byte: flight controller address (0xC8); some receivers use
        // the broadcast (0x00) or legacy sync (0xEE) address.
        if self.len == 0 && !matches!(b, 0xC8 | 0xEE | 0x00) {
            return None;
        }
        self.buf[self.len] = b;
        self.len += 1;
        if self.len < 2 {
            return None;
        }
        let frame_len = self.buf[1] as usize; // type + payload + crc
        if !(2..=CRSF_MAX_FRAME - 2).contains(&frame_len) {
            self.len = 0;
            return None;
        }
        if self.len < frame_len + 2 {
            return None;
        }
        self.len = 0;
        let body = &self.buf[2..2 + frame_len];
        let (data, crc) = body.split_at(frame_len - 1);
        if crsf_crc8(data) != crc[0] {
            self.crc_errors += 1;
            return None;
        }
        if data[0] != CRSF_RC_CHANNELS_PACKED || data.len() != 23 {
            return None;
        }
        let payload = &data[1..];
        let mut out = [0u16; RC_CHANNELS];
        for (i, ch) in out.iter_mut().enumerate() {
            let bit = i * 11;
            let (byte, shift) = (bit / 8, bit % 8);
            let raw = payload[byte] as u32
                | (payload[byte + 1] as u32) << 8
                | (payload.get(byte + 2).copied().unwrap_or(0) as u32) << 16;
            *ch = crsf_to_us(((raw >> shift) & 0x7FF) as u16);
        }
        Some(out)
    }
}

/// 16-bit DShot frame: 11-bit value, telemetry request bit, 4-bit checksum.
/// `inverted` selects the bidirectional-DShot checksum.
pub fn dshot_frame(value: u16, telemetry: bool, inverted: bool) -> u16 {
    let v = ((value & 0x7FF) << 1) | telemetry as u16;
    let mut crc = (v ^ (v >> 4) ^ (v >> 8)) & 0xF;
    if inverted {
        crc = !crc & 0xF;
    }
    (v << 4) | crc
}

/// DMA slots per DShot bit in [`dshot_bitbang`].
pub const BITBANG_SLOTS_PER_BIT: usize = 9;
/// Words in one [`dshot_bitbang`] frame.
pub const BITBANG_WORDS: usize = 16 * BITBANG_SLOTS_PER_BIT;

/// One DShot frame for several motors on one GPIO port, as words a timer
/// paces into the port's BSRR (bit set/reset register: the low 16 bits set
/// pins, the high 16 reset them, 0 leaves them alone). `pins` are the port
/// bit masks, one per frame. Each bit is a pulse of three slots (a 0) or six
/// (a 1) out of nine: high pulses on a low line, or, `inverted` for
/// bidirectional DShot, low pulses on a line that idles high.
pub fn dshot_bitbang(frames: [u16; 4], pins: [u32; 4], inverted: bool) -> [u32; BITBANG_WORDS] {
    let all = pins.iter().fold(0, |a, p| a | p);
    let (pulse, idle) = if inverted { (16, 0) } else { (0, 16) };
    let mut table = [0; BITBANG_WORDS];
    for bit in 0..16 {
        let mask = 0x8000u16 >> bit;
        let zeros = frames.iter().zip(pins).filter(|(f, _)| **f & mask == 0).fold(0, |a, (_, p)| a | p);
        let slots = &mut table[bit * BITBANG_SLOTS_PER_BIT..];
        slots[0] = all << pulse;
        slots[3] = zeros << idle;
        slots[6] = all << idle;
    }
    table
}

/// GCR code for each 4-bit nibble of a bidirectional-DShot reply.
const GCR: [u8; 16] = [
    0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17, 0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F,
];
/// The 12-bit value an ESC sends for a stopped motor.
const ERPM_STOPPED: u32 = 0xFFF;

/// An ESC's bidirectional-DShot reply carrying `erpm` (electrical RPM), as
/// the 21 bits it sends: a 1 for every edge on the line, starting with the
/// start bit, then four GCR quintets of `eeem mmmm mmmm cccc`, where the
/// electrical period is `m << e` µs and `c` makes the nibbles XOR to 0xF.
/// The period's resolution is what limits the reading's precision.
pub fn erpm_reply(erpm: u32) -> u32 {
    let period = if erpm == 0 { u32::MAX } else { (60_000_000 + erpm / 2) / erpm };
    let v12 = (0..8u32)
        .find(|&e| period >> e <= 0x1FF)
        .map_or(ERPM_STOPPED, |e| e << 9 | period >> e);
    let crc = 0xF ^ (v12 ^ v12 >> 4 ^ v12 >> 8) & 0xF;
    let v16 = v12 << 4 | crc;
    let gcr = (0..4).fold(0u32, |acc, i| acc << 5 | GCR[(v16 >> (12 - 4 * i) & 0xF) as usize] as u32);
    1 << 20 | gcr
}

/// The electrical RPM in a reply (as from [`erpm_reply`]), 0 for a stopped
/// motor, or `None` when it is corrupted.
pub fn decode_erpm_reply(word: u32) -> Option<u32> {
    let mut v16 = 0u32;
    for i in 0..4 {
        let q = (word >> (15 - 5 * i) & 0x1F) as u8;
        v16 = v16 << 4 | GCR.iter().position(|&g| g == q)? as u32;
    }
    if (v16 ^ v16 >> 4 ^ v16 >> 8 ^ v16 >> 12) & 0xF != 0xF {
        return None;
    }
    let v12 = v16 >> 4;
    if v12 == ERPM_STOPPED {
        return Some(0);
    }
    let period = (v12 & 0x1FF) << (v12 >> 9);
    if period == 0 {
        return None;
    }
    Some((60_000_000 + period / 2) / period)
}

/// The 21-bit reply word in a run of line samples taken `samples_per_bit`
/// per reply bit, the way Betaflight's bit-bang decoder reads it: the line
/// idles high, the start bit is the first falling edge, every edge is a 1
/// and the bits between edges are 0s. The line may end high before the
/// 21st bit, so the last run is inferred. `None` when no reply or a
/// malformed one is found.
pub fn reply_word(samples: impl IntoIterator<Item = bool>, samples_per_bit: f32) -> Option<u32> {
    let mut samples = samples.into_iter().skip_while(|&high| high);
    samples.next()?; // the start edge
    let mut level = false;
    let mut run = 1u32;
    let mut word = 0u32;
    let mut bits = 0u32;
    // A reply lasts 21 bits; stop looking two bits after that.
    let limit = (23.0 * samples_per_bit) as u32;
    let mut seen = 1;
    for high in samples {
        seen += 1;
        if seen > limit {
            break;
        }
        if high == level {
            run += 1;
            continue;
        }
        let len = ((run as f32 / samples_per_bit + 0.5) as u32).max(1);
        bits += len;
        if bits > 21 {
            return None;
        }
        word = word << len | 1 << (len - 1);
        level = high;
        run = 1;
    }
    if bits < 18 {
        return None;
    }
    let last = 21 - bits;
    if last > 0 {
        word = word << last | 1 << (last - 1);
    }
    Some(word)
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    /// Line levels for a reply word, `spb` samples per bit, idle high
    /// before and after.
    fn line(word: u32, spb: f32, lead: usize) -> std::vec::Vec<bool> {
        let mut out = std::vec![true; lead];
        let mut level = true;
        for bit in (0..21).rev() {
            if word >> bit & 1 != 0 {
                level = !level;
            }
            let from = ((20 - bit) as f32 * spb) as usize;
            let to = ((21 - bit) as f32 * spb) as usize;
            out.extend(std::iter::repeat_n(level, to - from));
        }
        out.extend(std::iter::repeat_n(true, 40));
        out
    }

    #[test]
    fn erpm_reply_matches_betaflight_layout() {
        // 4,000 eRPM: period 15,000 µs, sent as e = 5, m = 15,000 >> 5 = 468:
        // 0xBD4, nibbles B^D^4 = 2, checksum 0xD: 0xBD4D.
        let v16 = 0xBD4Du32;
        let gcr = [0xB, 0xD, 0x4, 0xD].iter().fold(0u32, |a, &n| a << 5 | GCR[n] as u32);
        assert_eq!((v16 ^ v16 >> 4 ^ v16 >> 8 ^ v16 >> 12) & 0xF, 0xF);
        assert_eq!(erpm_reply(4_000), 1 << 20 | gcr);
        // It reads back as 468 << 5 = 14,976 µs: 4,006 eRPM.
        assert_eq!(decode_erpm_reply(erpm_reply(4_000)), Some(4_006));
    }

    /// Plays a bit-bang table into a port that starts at `idle` and returns
    /// each pin's 16 bits, read as pulse lengths (3 slots a 0, 6 a 1).
    fn play(table: &[u32], pins: [u32; 4], idle: bool) -> [u16; 4] {
        let mut port: u32 = if idle { 0xFFFF } else { 0 };
        let mut levels = std::vec::Vec::new();
        for &w in table {
            port = (port | (w & 0xFFFF)) & !(w >> 16);
            levels.push(port);
        }
        // The line must end at its idle level.
        assert!(pins.iter().all(|&p| (port & p != 0) == idle));
        pins.map(|p| {
            let mut v = 0u16;
            for chunk in levels.chunks(BITBANG_SLOTS_PER_BIT) {
                let active = chunk.iter().filter(|&&l| (l & p != 0) != idle).count();
                assert!(active == 3 || active == 6, "pulse of {active} slots");
                v = v << 1 | (active == 6) as u16;
            }
            v
        })
    }

    #[test]
    fn bitbang_sends_each_motors_frame_in_both_polarities() {
        let pins = [1 << 10, 1 << 6, 1 << 7, 1 << 8];
        let frames = [0x8001, 0x7FFE, 0xA5C3, dshot_frame(1046, false, true)];
        assert_eq!(play(&dshot_bitbang(frames, pins, false), pins, false), frames);
        assert_eq!(play(&dshot_bitbang(frames, pins, true), pins, true), frames);
        // Pins outside the motor set are never touched.
        let others = !(pins.iter().fold(0u32, |a, p| a | p) * 0x1_0001);
        assert!(dshot_bitbang(frames, pins, true).iter().all(|w| w & others == 0));
    }

    #[test]
    fn gcr_table_matches_betaflight_decoder() {
        // `decode[]` from Betaflight's dshot_bitbang_decode.c (99 = invalid).
        const BF: [u8; 32] = [
            99, 99, 99, 99, 99, 99, 99, 99, 99, 9, 10, 11, 99, 13, 14, 15, 99, 99, 2, 3, 99, 5, 6, 7, 99, 0, 8, 1, 99, 4,
            12, 99,
        ];
        for (n, &q) in GCR.iter().enumerate() {
            assert_eq!(BF[q as usize] as usize, n);
        }
    }

    #[test]
    fn erpm_round_trips_within_the_period_resolution() {
        for erpm in [1_000u32, 9_999, 60_000, 123_457, 250_000, 432_000, 600_000] {
            let got = decode_erpm_reply(erpm_reply(erpm)).unwrap();
            // The period has 9 significant bits (and 1 µs at most).
            let tol = (erpm as f32 * (1.0 / 256.0)).max(erpm as f32 * erpm as f32 / 60e6);
            assert!((got as f32 - erpm as f32).abs() <= tol + 1.0, "{erpm} -> {got}");
        }
        assert_eq!(decode_erpm_reply(erpm_reply(0)), Some(0));
        // Slower than the longest period (65,408 µs, about 917 eRPM) reads stopped.
        assert_eq!(decode_erpm_reply(erpm_reply(800)), Some(0));
    }

    #[test]
    fn every_single_bit_error_is_rejected() {
        let word = erpm_reply(87_654);
        for bit in 0..20 {
            assert_eq!(decode_erpm_reply(word ^ 1 << bit), None, "bit {bit}");
        }
    }

    #[test]
    fn reply_word_reads_line_samples() {
        for erpm in [0u32, 4_000, 87_654, 432_000] {
            let word = erpm_reply(erpm);
            for spb in [3.0f32, 7.3] {
                for lead in [0, 5, 31] {
                    assert_eq!(reply_word(line(word, spb, lead), spb), Some(word), "{erpm} spb {spb} lead {lead}");
                }
            }
        }
        // The ESC replies at its own 375 kbit/s while the board samples at
        // 7.2 per bit (9 slots per 304.8 kbit/s DShot bit, / 1.25).
        for erpm in [0u32, 4_000, 87_654, 432_000] {
            for actual in [6.9f32, 7.31, 7.5] {
                assert_eq!(reply_word(line(erpm_reply(erpm), actual, 11), 7.2), Some(erpm_reply(erpm)), "{erpm} at {actual}");
            }
        }
        // An ESC that never answers: the line stays high.
        assert_eq!(reply_word(std::vec![true; 200], 7.3), None);
        // A frame cut short after a few bits.
        let mut cut = line(erpm_reply(4_000), 7.3, 3);
        cut.truncate(3 + 40);
        assert_eq!(reply_word(cut, 7.3), None);
    }

    #[test]
    fn dshot_matches_published_example() {
        // Throttle 1046, no telemetry: 10000010110 0 0110.
        assert_eq!(dshot_frame(1046, false, false), 0b1000_0010_1100_0110);
    }

    fn encode(channels_ticks: [u16; 16]) -> [u8; 26] {
        let mut f = [0u8; 26];
        f[0] = 0xC8;
        f[1] = 24;
        f[2] = CRSF_RC_CHANNELS_PACKED;
        for (i, &v) in channels_ticks.iter().enumerate() {
            for b in 0..11 {
                if v >> b & 1 != 0 {
                    let bit = i * 11 + b;
                    f[3 + bit / 8] |= 1 << (bit % 8);
                }
            }
        }
        f[25] = crsf_crc8(&f[2..25]);
        f
    }

    #[test]
    fn crsf_decodes_channels_and_rejects_corruption() {
        let mut ticks = [992u16; 16];
        ticks[0] = 172;
        ticks[1] = 1811;
        ticks[2] = 191; // throttle low (1000 µs)
        ticks[4] = 1792; // arm switch high (2000 µs)
        let frame = encode(ticks);
        let mut p = CrsfParser::default();
        let mut got = None;
        // Leading garbage and a link-statistics-like frame must be skipped.
        for &b in [0x12u8, 0x34].iter().chain(frame.iter()) {
            if let Some(c) = p.push(b) {
                got = Some(c);
            }
        }
        let c = got.expect("frame decoded");
        assert_eq!(c[0], 988);
        assert_eq!(c[1], 2011);
        assert_eq!(c[2], 1000);
        assert_eq!(c[3], 1500);
        assert_eq!(c[4], 2000);

        let mut bad = frame;
        bad[10] ^= 0x40;
        let mut p = CrsfParser::default();
        assert!(bad.iter().all(|&b| p.push(b).is_none()));
        assert_eq!(p.crc_errors, 1);
        // The parser resynchronizes on the next good frame.
        assert!(frame.iter().filter_map(|&b| p.push(b)).next().is_some());
    }
}
