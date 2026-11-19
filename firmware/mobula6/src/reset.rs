//! Why the board last reset, for the USB status line: a watchdog, brownout,
//! or panic reset otherwise looks just like plugging the battery in.
//!
//! `RCC_CSR` holds a flag per reset source until cleared. One reset sets
//! several (a power-on also sets the brownout flag, and almost any reset the
//! pin flag, since the chip drives NRST itself), so `latch` keeps the most
//! specific. A panic resets through software like the DFU reboot does, so it
//! leaves its own mark in RAM that survives the reset.

use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicU8, Ordering::Relaxed};

use embassy_stm32::pac;

const PANIC_MAGIC: u32 = 0x9A41_C0DE;

#[unsafe(link_section = ".uninit.PANICFLAG")]
static mut PANIC_FLAG: MaybeUninit<u32> = MaybeUninit::uninit();

/// The cause `latch` found, as an index into `NAMES`.
static CAUSE: AtomicU8 = AtomicU8::new(0);

const NAMES: [&str; 9] = ["unknown", "panic", "watchdog", "window-watchdog", "low-power", "software", "power-on", "brownout", "pin"];

/// Reads and clears the reset flags. Call once at boot.
pub fn latch() {
    // SAFETY: plain volatile accesses to a word only this module uses, at
    // boot before anything can panic.
    let panicked = unsafe {
        let flag = (&raw mut PANIC_FLAG).cast::<u32>();
        let p = core::ptr::read_volatile(flag) == PANIC_MAGIC;
        core::ptr::write_volatile(flag, 0);
        p
    };
    let csr = pac::RCC.csr().read();
    let cause = if panicked {
        1
    } else if csr.wdgrstf() {
        2
    } else if csr.wwdgrstf() {
        3
    } else if csr.lpwrrstf() {
        4
    } else if csr.sftrstf() {
        5
    } else if csr.porrstf() {
        6
    } else if csr.borrstf() {
        7
    } else if csr.padrstf() {
        8
    } else {
        0
    };
    pac::RCC.csr().modify(|w| w.set_rmvf(true));
    CAUSE.store(cause, Relaxed);
}

/// The last reset's cause, such as "power-on", "watchdog", or "brownout".
pub fn cause() -> &'static str {
    NAMES[CAUSE.load(Relaxed) as usize]
}

/// Resets after a panic, leaving the mark `latch` reads.
pub fn panic_reset() -> ! {
    // SAFETY: plain volatile store to a word only this module uses.
    unsafe { core::ptr::write_volatile((&raw mut PANIC_FLAG).cast::<u32>(), PANIC_MAGIC) };
    cortex_m::peripheral::SCB::sys_reset()
}
