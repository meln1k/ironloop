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

#[cfg(test)]
mod tests {
    use super::*;

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
