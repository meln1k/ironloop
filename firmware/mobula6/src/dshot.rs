//! DShot300 to all four ESC inputs by bit-banging GPIOB.
//!
//! All motor pins are on GPIOB (M1 PB10, M2 PB6, M3 PB7, M4 PB8), so one DMA
//! stream can drive all four at once: TIM1 update events pace DMA2 stream 5
//! (channel 6, TIM1_UP), which copies a precomputed table of 32-bit words into
//! GPIOB->BSRR. Each DShot bit takes three equal slots: all pins high, then
//! the pins sending a 0 go low, then all pins low. That gives 33% / 67% duty
//! for 0 / 1 bits, the same scheme Betaflight's bit-bang driver uses. Only
//! DMA2 can reach the GPIO bus on the F4, which is why TIM1 is used.
//!
//! This sends normal (non-inverted) DShot without telemetry. Bluejay detects
//! the polarity at startup. Bidirectional eRPM telemetry is not implemented
//! yet, so the firmware's motor-fault detection gets no input on hardware.

use core::sync::atomic::{Ordering, compiler_fence};

use embassy_stm32::Peri;
use embassy_stm32::gpio::Output;
use embassy_stm32::pac;
use embassy_stm32::pac::dma::vals;
use embassy_stm32::peripherals::{DMA2_CH5, TIM1};
use embassy_stm32::timer::low_level::{RoundTo, Timer};
use ironloop_fc::proto::dshot_frame;

const SLOTS_PER_BIT: usize = 3;
/// 16 bits; each bit's last slot drives every pin low, so the frame ends low.
const WORDS: usize = 16 * SLOTS_PER_BIT;
/// GPIOB bit per motor, Betaflight order (RR, FR, RL, FL).
const PINS: [u32; 4] = [1 << 10, 1 << 6, 1 << 7, 1 << 8];
const ALL: u32 = PINS[0] | PINS[1] | PINS[2] | PINS[3];
/// DShot300 bit time is 3.333 µs, so one slot is 1.111 µs.
const SLOT_HZ: u32 = 900_000;
/// DMA2 stream 5 sits in the high flag registers (streams 4..7) at index 1.
const STREAM: usize = 5;

static mut TABLE: [u32; WORDS] = [0; WORDS];

pub struct Dshot {
    timer: Timer<'static, TIM1>,
    _dma: Peri<'static, DMA2_CH5>,
    _pins: [Output<'static>; 4],
}

impl Dshot {
    pub fn new(tim: Peri<'static, TIM1>, dma: Peri<'static, DMA2_CH5>, pins: [Output<'static>; 4]) -> Self {
        pac::RCC.ahb1enr().modify(|w| w.set_dma2en(true));
        let timer = Timer::new(tim);
        timer.stop();
        timer.set_frequency(embassy_stm32::time::Hertz(SLOT_HZ), RoundTo::Faster);
        Dshot { timer, _dma: dma, _pins: pins }
    }

    /// Sends one frame per motor. Values are DShot commands: 0 stops,
    /// 48..=2047 is throttle. Takes about 55 µs on the wire and returns
    /// as soon as it starts. A call during a frame first waits for that
    /// frame to finish, so a frame is never cut short.
    pub fn write(&mut self, values: [u16; 4]) {
        let frames = values.map(|v| dshot_frame(v, false, false));
        self.wait_idle();
        // SAFETY: the DMA stream has finished (above) before the table
        // changes, and only this driver (a single owner) touches it.
        let table = unsafe { &mut *(&raw mut TABLE) };
        let st = pac::DMA2.st(STREAM);

        self.timer.stop();
        self.timer.enable_update_dma(false);

        for bit in 0..16 {
            let mask = 0x8000u16 >> bit;
            let zeros = frames
                .iter()
                .zip(PINS)
                .filter(|(f, _)| **f & mask == 0)
                .fold(0, |acc, (_, pin)| acc | pin);
            table[bit * SLOTS_PER_BIT] = ALL; // set: all high
            table[bit * SLOTS_PER_BIT + 1] = zeros << 16; // reset: 0-bits low
            table[bit * SLOTS_PER_BIT + 2] = ALL << 16; // reset: all low
        }

        pac::DMA2.ifcr(STREAM / 4).write(|w| {
            let i = STREAM % 4;
            w.set_tcif(i, true);
            w.set_htif(i, true);
            w.set_teif(i, true);
            w.set_dmeif(i, true);
            w.set_feif(i, true);
        });
        st.par().write_value(pac::GPIOB.bsrr().as_ptr() as u32);
        st.m0ar().write_value(table.as_ptr() as u32);
        st.ndtr().write(|w| w.set_ndt(WORDS as u16));
        compiler_fence(Ordering::SeqCst);
        st.cr().write(|w| {
            w.set_chsel(6);
            w.set_dir(vals::Dir::MEMORY_TO_PERIPHERAL);
            w.set_minc(true);
            w.set_psize(vals::Size::BITS32);
            w.set_msize(vals::Size::BITS32);
            w.set_pl(vals::Pl::VERY_HIGH);
            w.set_en(true);
        });
        self.timer.reset();
        self.timer.enable_update_dma(true);
        self.timer.start();
    }

    /// Waits until the frame on the wire, if any, has finished (at most
    /// about 55 µs). The stream disables itself after its last word.
    pub fn wait_idle(&self) {
        while pac::DMA2.st(STREAM).cr().read().en() {}
    }
}
