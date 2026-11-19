//! USB serial (CDC-ACM) status line for bench checks. SWD is not reachable
//! on this board, so this is how `control::STATS` and the firmware's
//! `Status` get off the quad: open the port (`/dev/ttyACM0`, any baud) and
//! read one status line per 100 ms:
//!
//! ```text
//! imu 32004 miss 0 rderr 0 drop 0 cyc 4211/9876 erpm 0 0 0 0 armed 0 blk 0x24 vbat 4.08 amps 0.31 rpy 1.2 -0.4 87.0 gyro 0.00 0.01 -0.00 mot 0 0 0 0 flt 0000 rc 1500 1500 988 1500 1000 1000 1000 1000 chip bmi270 rst power-on ndrop 0
//! ```
//!
//! `cyc` is the last and longest `step` in CPU cycles at 96 MHz (the BMI270
//! sample period is 30,000), `blk` the arming blockers, `amps` the battery
//! current, `rpy` the attitude in degrees, `gyro` rad/s, `flt` the motor
//! faults per output, `rc` the newest RC frame's channels in µs (roll,
//! pitch, throttle, yaw, arm, mode, then two spare), `chip` the IMU found (or
//! `none`), `rst` why the board last reset (`reset::cause`), and `ndrop` the
//! notices lost to a full queue.
//!
//! Before each status line come the firmware's notices since the last one,
//! one per line with the event time in ms:
//!
//! ```text
//! note 4021 Disarmed(RunawayTakeoff)
//! ```
//!
//! The port is output only.
//!
//! VBUS sensing is off: PA9, the F411's VBUS pin, carries the VTX UART here.
//! On battery power the device therefore stays attached after the cable is
//! pulled, which only matters to the host.

use core::fmt::Write;
use core::sync::atomic::Ordering::Relaxed;

use embassy_futures::select::{Either, select};
use embassy_stm32::peripherals::{PA11, PA12, USB_OTG_FS};
use embassy_stm32::usb::Driver;
use embassy_stm32::{Peri, bind_interrupts, usb};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config};
use ironloop_fc::Notice;
use static_cell::StaticCell;

use crate::control::{NOTICES, STATS, STATUS};

bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<USB_OTG_FS>;
});

const PACKET: usize = 64;

/// A line of text in a fixed buffer; anything past the end is dropped.
struct Line {
    buf: [u8; 512],
    len: usize,
}

impl Line {
    fn new() -> Self {
        Line { buf: [0; 512], len: 0 }
    }
}

impl Write for Line {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let n = s.len().min(self.buf.len() - self.len);
        self.buf[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
        self.len += n;
        Ok(())
    }
}

#[embassy_executor::task]
pub async fn usb_task(otg: Peri<'static, USB_OTG_FS>, dp: Peri<'static, PA12>, dm: Peri<'static, PA11>) {
    static EP_OUT: StaticCell<[u8; 256]> = StaticCell::new();
    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL: StaticCell<[u8; 64]> = StaticCell::new();
    static CDC: StaticCell<State> = StaticCell::new();

    let driver = Driver::new_fs(otg, Irqs, dp, dm, EP_OUT.init([0; 256]), usb::Config::default());
    // pid.codes test PID; fine for a bench tool, not for distribution.
    let mut config = Config::new(0x1209, 0x0001);
    config.manufacturer = Some("IronLoop");
    config.product = Some("IronLoop Mobula6");
    let mut builder = Builder::new(driver, config, CONFIG_DESC.init([0; 256]), BOS_DESC.init([0; 256]), &mut [], CONTROL.init([0; 64]));
    let mut class = CdcAcmClass::new(&mut builder, CDC.init(State::new()), PACKET as u16);
    let mut device = builder.build();

    let report = async {
        loop {
            class.wait_connection().await;
            // A write error means the host went away; wait for it again.
            while send_line(&mut class).await.is_ok() {}
        }
    };
    match select(device.run(), report).await {
        Either::First(never) | Either::Second(never) => never,
    }
}

type Class = CdcAcmClass<'static, Driver<'static, USB_OTG_FS>>;

async fn send_line(class: &mut Class) -> Result<(), embassy_usb::driver::EndpointError> {
    let status = STATUS.wait().await;
    while let Ok((t_us, note)) = NOTICES.try_receive() {
        let mut line = Line::new();
        let _ = match note {
            // In hex, like `blk`.
            Notice::ArmingBlocked(b) => write!(line, "note {} ArmingBlocked({:#x})\r\n", t_us / 1000, b.0),
            _ => write!(line, "note {} {:?}\r\n", t_us / 1000, note),
        };
        send(class, &line).await?;
    }
    let s = &STATS;
    let mut line = Line::new();
    let (roll, pitch, yaw) = status.attitude.to_euler();
    let deg = 180.0 / core::f32::consts::PI;
    let erpm = s.erpm.each_ref().map(|e| e.load(Relaxed));
    let m = status.motors;
    let g = status.gyro;
    let _ = write!(
        line,
        "imu {} miss {} rderr {} drop {} cyc {}/{} erpm {} {} {} {} armed {} blk {:#x} vbat {:.2} amps {:.2} rpy {:.1} {:.1} {:.1} gyro {:.2} {:.2} {:.2} mot {} {} {} {} flt ",
        s.imu_samples.load(Relaxed),
        s.drdy_missed.load(Relaxed),
        s.imu_read_errors.load(Relaxed),
        s.events_dropped.load(Relaxed),
        s.step_cycles_last.load(Relaxed),
        s.step_cycles_max.load(Relaxed),
        erpm[0],
        erpm[1],
        erpm[2],
        erpm[3],
        status.armed as u8,
        status.blockers.0,
        status.battery_volts.unwrap_or(0.0),
        f32::from_bits(s.amps.load(Relaxed)),
        roll * deg,
        pitch * deg,
        yaw * deg,
        g.x,
        g.y,
        g.z,
        m[0],
        m[1],
        m[2],
        m[3],
    );
    for f in status.motor_faults {
        let _ = line.write_str(if f { "1" } else { "0" });
    }
    let _ = line.write_str(" rc");
    for c in &s.rc {
        let _ = write!(line, " {}", c.load(Relaxed));
    }
    let chip = ["none", "bmi270", "icm42688"].get(s.imu_chip.load(Relaxed) as usize).copied().unwrap_or("none");
    let _ = write!(line, " chip {chip} rst {} ndrop {}\r\n", crate::reset::cause(), s.notices_dropped.load(Relaxed));
    send(class, &line).await
}

async fn send(class: &mut Class, line: &Line) -> Result<(), embassy_usb::driver::EndpointError> {
    let bytes = &line.buf[..line.len];
    for packet in bytes.chunks(PACKET) {
        class.write_packet(packet).await?;
    }
    // A full last packet needs a zero-length one to end the transfer.
    if bytes.len() % PACKET == 0 {
        class.write_packet(&[]).await?;
    }
    Ok(())
}
