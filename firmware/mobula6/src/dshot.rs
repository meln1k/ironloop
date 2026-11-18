//! Bidirectional DShot300 to all four ESCs by bit-banging GPIOB, with their
//! eRPM replies read back on the same pins.
//!
//! All motor pins are on GPIOB (M1 PB10, M2 PB6, M3 PB7, M4 PB8), so one DMA
//! stream serves all four at once: TIM1 update events pace DMA2 stream 5
//! (channel 6, TIM1_UP). On the F4 only DMA2 can reach the GPIO bus, which is
//! why TIM1 is used. This is the scheme Betaflight's bit-bang driver uses.
//!
//! - Sending: the stream copies `ironloop_fc::proto::dshot_bitbang`'s words
//!   into GPIOB->BSRR. Each DShot bit takes nine slots; the line idles high
//!   and each bit is a low pulse of three slots (a 0) or six (a 1), the
//!   inverted signal that tells Bluejay to answer with telemetry.
//! - Receiving: when the frame's last word is out, the stream's interrupt
//!   turns the pins into pulled-up inputs and points the stream at
//!   GPIOB->IDR. It samples every pin at the slot rate for long enough to
//!   hold the reply, which starts about 30 µs after the frame. Then the
//!   pins become outputs again, idling high.
//! - Decoding (`ironloop_fc::proto::reply_word`, host-tested): one motor
//!   per frame in turn, so the cost stays at one reply per frame.
//! - A DMA error ends the frame or window early, the same as completion, so
//!   the driver never waits on a stream that stopped.
//!
//! The Mobula6's stock ESCs run Bluejay 0.19, which answers inverted DShot
//! with eRPM. ESCs reflashed to BLHeli_S do not understand inverted DShot
//! and will not spin; flash Bluejay back.

use core::sync::atomic::{AtomicU8, Ordering, compiler_fence};

use embassy_stm32::Peri;
use embassy_stm32::gpio::Output;
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::pac;
use embassy_stm32::pac::dma::vals;
use embassy_stm32::pac::gpio::vals::{Moder, Pupdr};
use embassy_stm32::peripherals::{DMA2_CH5, TIM1};
use embassy_stm32::timer::low_level::{RoundTo, Timer};
use ironloop_fc::proto::{BITBANG_SLOTS_PER_BIT, BITBANG_WORDS, decode_erpm_reply, dshot_bitbang, dshot_frame, reply_word};

/// GPIOB pin per motor, Betaflight order (RR, FR, RL, FL).
const PIN_NUMBERS: [usize; 4] = [10, 6, 7, 8];
const PINS: [u32; 4] = [1 << PIN_NUMBERS[0], 1 << PIN_NUMBERS[1], 1 << PIN_NUMBERS[2], 1 << PIN_NUMBERS[3]];
const ALL: u32 = PINS[0] | PINS[1] | PINS[2] | PINS[3];
/// Nine slots per DShot300 bit (3.333 µs). The timer rounds this up to
/// 96 MHz / 35, so bits run at 304.8 kbit/s, well inside what ESCs accept.
const SLOT_HZ: u32 = 2_700_000;
/// Replies run at 5/4 of the DShot bit rate.
const SAMPLES_PER_REPLY_BIT: f32 = BITBANG_SLOTS_PER_BIT as f32 / 1.25;
/// Reply samples: about 30 µs of turnaround, 21 bits (69 µs), and margin;
/// 320 slots are 117 µs.
const SAMPLES: usize = 320;
/// DMA2 stream 5 sits in the high flag registers (streams 4..7) at index 1.
const STREAM: usize = 5;

const IDLE: u8 = 0;
const SENDING: u8 = 1;
const RECEIVING: u8 = 2;
static PHASE: AtomicU8 = AtomicU8::new(IDLE);

static mut TABLE: [u32; BITBANG_WORDS] = [0; BITBANG_WORDS];
static mut REPLY: [u16; SAMPLES] = [0; SAMPLES];

pub struct Dshot {
    timer: Timer<'static, TIM1>,
    _dma: Peri<'static, DMA2_CH5>,
    _pins: [Output<'static>; 4],
    /// The motor whose reply is decoded next.
    next: usize,
    readings: [Option<u32>; 4],
}

impl Dshot {
    /// `pins` must start at the idle level, high.
    pub fn new(tim: Peri<'static, TIM1>, dma: Peri<'static, DMA2_CH5>, pins: [Output<'static>; 4]) -> Self {
        pac::RCC.ahb1enr().modify(|w| w.set_dma2en(true));
        for n in PIN_NUMBERS {
            // Only matters while the pins are inputs between frames.
            pac::GPIOB.pupdr().modify(|w| w.set_pupdr(n, Pupdr::PULL_UP));
        }
        let timer = Timer::new(tim);
        timer.stop();
        timer.set_frequency(embassy_stm32::time::Hertz(SLOT_HZ), RoundTo::Faster);
        interrupt::DMA2_STREAM5.set_priority(Priority::P0);
        // SAFETY: `DMA2_STREAM5` below handles it and touches only this
        // driver's stream, timer, and pins.
        unsafe { interrupt::DMA2_STREAM5.enable() };
        Dshot { timer, _dma: dma, _pins: pins, next: 0, readings: [None; 4] }
    }

    /// Sends one frame per motor. Values are DShot commands: 0 stops,
    /// 48..=2047 is throttle. Returns as soon as it starts. A call while a
    /// frame or its reply window is in progress (about 55 µs and 117 µs)
    /// first waits for it to finish, so a frame is never cut short.
    pub fn write(&mut self, values: [u16; 4]) {
        let frames = values.map(|v| dshot_frame(v, false, true));
        self.wait_idle();
        self.read_reply();
        // SAFETY: the DMA stream has finished (above) before the table
        // changes, and only this driver (a single owner) touches it.
        let table = unsafe { &mut *(&raw mut TABLE) };
        *table = dshot_bitbang(frames, PINS, true);

        self.timer.stop();
        self.timer.enable_update_dma(false);
        PHASE.store(SENDING, Ordering::Relaxed);
        start_stream(
            vals::Dir::MEMORY_TO_PERIPHERAL,
            pac::GPIOB.bsrr().as_ptr() as u32,
            table.as_ptr() as u32,
            BITBANG_WORDS,
            vals::Size::BITS32,
        );
        self.timer.reset();
        self.timer.enable_update_dma(true);
        self.timer.start();
    }

    /// Waits until the frame on the wire and its reply window, if any, have
    /// finished.
    pub fn wait_idle(&self) {
        while PHASE.load(Ordering::Acquire) != IDLE {}
    }

    /// The eRPM readings collected since the last call, per motor output
    /// (`None` for a motor without a valid reply), or `None` if none came.
    pub fn take_telemetry(&mut self) -> Option<[Option<u32>; 4]> {
        let readings = core::mem::replace(&mut self.readings, [None; 4]);
        readings.iter().any(Option::is_some).then_some(readings)
    }

    /// Decodes the last reply window for the next motor in turn. Before the
    /// first frame the buffer is all low, which holds no reply.
    fn read_reply(&mut self) {
        let i = self.next;
        self.next = (i + 1) % 4;
        // SAFETY: the stream is idle (`wait_idle`), so nothing writes it.
        let samples = unsafe { &*(&raw const REPLY) };
        let pin = PINS[i] as u16;
        let reading = reply_word(samples.iter().map(|&s| s & pin != 0), SAMPLES_PER_REPLY_BIT).and_then(decode_erpm_reply);
        if reading.is_some() {
            self.readings[i] = reading;
        }
    }
}

fn start_stream(dir: vals::Dir, par: u32, mar: u32, count: usize, size: vals::Size) {
    let st = pac::DMA2.st(STREAM);
    pac::DMA2.ifcr(STREAM / 4).write(|w| {
        let i = STREAM % 4;
        w.set_tcif(i, true);
        w.set_htif(i, true);
        w.set_teif(i, true);
        w.set_dmeif(i, true);
        w.set_feif(i, true);
    });
    st.par().write_value(par);
    st.m0ar().write_value(mar);
    st.ndtr().write(|w| w.set_ndt(count as u16));
    compiler_fence(Ordering::SeqCst);
    st.cr().write(|w| {
        w.set_chsel(6);
        w.set_dir(dir);
        w.set_minc(true);
        w.set_psize(size);
        w.set_msize(size);
        w.set_pl(vals::Pl::VERY_HIGH);
        w.set_tcie(true);
        w.set_teie(true);
        w.set_dmeie(true);
        w.set_en(true);
    });
}

/// The stream finished: a frame went out (start the reply window),
/// or a reply window ended (pins back to outputs, idling high). A transfer
/// or direct-mode error stops the stream without completing it; it ends the
/// frame or window the same way, skipping the reply window.
#[interrupt]
fn DMA2_STREAM5() {
    let i = STREAM % 4;
    let flags = pac::DMA2.isr(STREAM / 4).read();
    let failed = flags.teif(i) || flags.dmeif(i);
    pac::DMA2.ifcr(STREAM / 4).write(|w| {
        w.set_tcif(i, true);
        w.set_teif(i, true);
        w.set_dmeif(i, true);
    });
    if !failed && PHASE.load(Ordering::Relaxed) == SENDING {
        for n in PIN_NUMBERS {
            pac::GPIOB.moder().modify(|w| w.set_moder(n, Moder::INPUT));
        }
        // Only this interrupt and `Dshot::read_reply` (after the window,
        // while the stream is idle) use the buffer.
        let buf = &raw mut REPLY as u32;
        start_stream(vals::Dir::PERIPHERAL_TO_MEMORY, pac::GPIOB.idr().as_ptr() as u32, buf, SAMPLES, vals::Size::BITS16);
        PHASE.store(RECEIVING, Ordering::Relaxed);
        return;
    }
    pac::TIM1.cr1().modify(|w| w.set_cen(false));
    pac::GPIOB.bsrr().write_value(pac::gpio::regs::Bsrr(ALL));
    for n in PIN_NUMBERS {
        pac::GPIOB.moder().modify(|w| w.set_moder(n, Moder::OUTPUT));
    }
    PHASE.store(IDLE, Ordering::Release);
}
