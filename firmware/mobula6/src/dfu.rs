//! DFU bootloader entry.
//!
//! The SWD clock pin doubles as the blackbox flash chip-select on this board,
//! so reflashing goes through the ROM USB DFU bootloader. The firmware asks for
//! it with `Effect::RebootToBootloader` (a disarmed stick gesture): `reboot`
//! sets a flag in RAM that survives reset and resets, and the next boot calls
//! `jump_if_requested` first thing, before the clocks or any peripheral are
//! touched.

use core::mem::MaybeUninit;

const BOOT_MAGIC: u32 = 0xB007_10AD;
const SYSTEM_MEMORY: u32 = 0x1FFF_0000;

#[unsafe(link_section = ".uninit.BOOTFLAG")]
static mut BOOT_FLAG: MaybeUninit<u32> = MaybeUninit::uninit();

/// Jumps to the ROM bootloader if the last reset asked for it. Must run
/// before anything else, while no interrupt is enabled in the NVIC.
pub fn jump_if_requested() {
    // SAFETY: single-threaded, before any interrupt is enabled.
    unsafe {
        let flag = (&raw mut BOOT_FLAG).cast::<u32>();
        if core::ptr::read_volatile(flag) != BOOT_MAGIC {
            return;
        }
        core::ptr::write_volatile(flag, 0);
        // Map system memory at 0x0 the way the ROM bootloader expects.
        let rcc = embassy_stm32::pac::RCC;
        rcc.apb2enr().modify(|w| w.set_syscfgen(true));
        embassy_stm32::pac::SYSCFG.memrm().modify(|w| w.set_mem_mode(1));
        cortex_m::asm::bootload(SYSTEM_MEMORY as *const u32);
    }
}

/// Resets into the ROM bootloader.
pub fn reboot() -> ! {
    // SAFETY: plain volatile store to a word only this module uses.
    unsafe { core::ptr::write_volatile((&raw mut BOOT_FLAG).cast::<u32>(), BOOT_MAGIC) };
    cortex_m::peripheral::SCB::sys_reset();
}
