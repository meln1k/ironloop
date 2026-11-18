//! The deterministic discrete-event simulator. Every sensor sample, RC frame,
//! battery reading, ESC telemetry frame, and timer tick is an event on one
//! time-ordered queue; physics advances in fixed substeps between events,
//! and the firmware's `step` runs on each delivered event. Given a scenario,
//! the run is a pure function of its seed.

use std::cmp::Reverse;
use std::collections::{BinaryHeap, VecDeque};

use ironloop_fc::math::Vec3;
use ironloop_fc::proto::{decode_erpm_reply, erpm_reply};
use ironloop_fc::{
    Effect, Event, EventKind, ImuAlignment, Notice, RC_CHANNELS, State, Status, step,
};
use libm::{cosf, sinf};

use crate::monitor::Monitor;
use crate::pilot::{Pilot, Sticks};
use crate::physics::{MotorHealth, World};
use crate::rng::Rng;
use crate::scenario::{Fault, FaultKind, Scenario};

const PHYSICS_DT_US: u64 = 250;
const MOTOR_LATENCY_US: u64 = 115;
const TICK_US: u64 = 1_000;
const TELEMETRY_US: u64 = 1_000;
const BATTERY_US: u64 = 20_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Ev {
    Imu,
    Rc,
    Battery,
    Telemetry,
    Tick,
    ApplyMotors,
    /// An IMU sample finishes its SPI read and reaches the event queue.
    ImuDeliver,
}

#[derive(Clone, Debug)]
pub struct Sim {
    pub sc: Scenario,
    pub world: World,
    fc: Option<State>,
    pub t_us: u64,
    physics_us: u64,
    queue: BinaryHeap<Reverse<(u64, u64, Ev)>>,
    seq: u64,
    pending_motors: VecDeque<[u16; 4]>,
    /// The board's control task reads one IMU sample at a time: the sample
    /// being read (stamped at data-ready), and the newest data-ready that
    /// came during that read. An older unread one is lost.
    imu_read: Option<(u64, EventKind)>,
    imu_waiting: Option<(u64, EventKind)>,
    /// Events that arrived during an IMU read; the control task steps them
    /// after the sample.
    held: Vec<(u64, EventKind)>,
    /// Events held in the board's 32-deep queue while the firmware stalls,
    /// and the newest data-ready from the stall.
    stalled: Vec<(u64, EventKind)>,
    stalled_imu: Option<(u64, EventKind)>,
    rng_board: Rng,
    imu_next_ideal: f64,
    rng_imu: Rng,
    rng_rc: Rng,
    rng_env: Rng,
    rng_esc: Rng,
    pilot: Pilot,
    last_imu_raw: Option<([f32; 3], [f32; 3])>,
    /// The IMU chip's low-pass state: two stages each for gyro and accel,
    /// and when it last ran.
    chip_lpf: Option<([Vec3; 2], [Vec3; 2], u64)>,
    one_shot_done: Vec<bool>,
    /// Manual control from the visualizer replaces the virtual pilot.
    pub manual: Option<Sticks>,
    pub sticks: Sticks,
    pub monitor: Monitor,
    /// Hash over every firmware input and output (determinism check).
    pub trace_hash: u64,
    /// Hash over the motor commands alone, in order (metamorphic checks:
    /// a clock offset may shift LED blinks but never the motors).
    pub motor_hash: u64,
    pub notices: Vec<(u64, Notice)>,
    pub status: Status,
    pub led: bool,
    pub beeper: bool,
    pub events_delivered: u64,
    pub rebooted_to_bootloader: bool,
}

impl Sim {
    pub fn new(sc: Scenario) -> Self {
        let mut world = World::new(sc.frame.clone(), sc.battery.clone(), sc.wind.clone());
        let (r, p) = sc.ground_tilt;
        world.body.att = ironloop_fc::math::Quat::from_euler(r, p, sc.initial_yaw);
        if r != 0.0 || p != 0.0 {
            world.body.pos.z = -0.03;
        }
        let fc = State::new(sc.fc);
        let status = fc.status();
        let seed = sc.seed;
        let n_faults = sc.faults.len();
        let mut sim = Sim {
            world,
            fc: Some(fc),
            t_us: 0,
            physics_us: 0,
            queue: BinaryHeap::new(),
            seq: 0,
            pending_motors: VecDeque::new(),
            imu_read: None,
            imu_waiting: None,
            held: Vec::new(),
            stalled: Vec::new(),
            stalled_imu: None,
            rng_board: Rng::fork(seed, "board"),
            imu_next_ideal: 0.0,
            rng_imu: Rng::fork(seed, "imu"),
            rng_rc: Rng::fork(seed, "rc"),
            rng_env: Rng::fork(seed, "env"),
            rng_esc: Rng::fork(seed, "esc"),
            pilot: Pilot::default(),
            last_imu_raw: None,
            chip_lpf: None,
            one_shot_done: vec![false; n_faults],
            manual: None,
            sticks: Sticks::default(),
            monitor: Monitor::new(&sc),
            trace_hash: 0xcbf2_9ce4_8422_2325,
            motor_hash: 0xcbf2_9ce4_8422_2325,
            notices: Vec::new(),
            status,
            led: false,
            beeper: false,
            events_delivered: 0,
            rebooted_to_bootloader: false,
            sc,
        };
        // Boot order is staggered like real peripherals coming up.
        sim.schedule(1_000, Ev::Imu);
        sim.imu_next_ideal = 1_000.0;
        sim.schedule(5_000, Ev::Rc);
        sim.schedule(2_000, Ev::Battery);
        if sim.sc.esc_telemetry {
            sim.schedule(1_500, Ev::Telemetry);
        }
        sim.schedule(500, Ev::Tick);
        sim
    }

    fn schedule(&mut self, t: u64, ev: Ev) {
        self.seq += 1;
        self.queue.push(Reverse((t, self.seq, ev)));
    }

    pub fn fc_state(&self) -> &State {
        self.fc.as_ref().expect("fc state present between events")
    }

    pub fn done(&self) -> bool {
        self.t_us >= self.sc.duration_us || self.rebooted_to_bootloader
    }

    /// Adds a fault at the current time (used by the interactive visualizer).
    pub fn inject(&mut self, kind: FaultKind, dur_us: u64) {
        self.sc.faults.push(Fault { at_us: self.t_us, dur_us, kind });
        self.one_shot_done.push(false);
    }

    /// Clears injected faults that are still running and repairs motors.
    pub fn clear_faults(&mut self) {
        let t = self.t_us;
        for f in &mut self.sc.faults {
            if f.active(t) {
                f.dur_us = t.saturating_sub(f.at_us);
            }
        }
        for m in &mut self.world.motors {
            m.health = MotorHealth::Ok;
        }
        self.world.wind.mean = self.sc.wind.mean;
    }

    /// The first active fault `pick` accepts, as what `pick` takes from it.
    fn active_fault<T>(&self, pick: impl Fn(&FaultKind) -> Option<T>) -> Option<T> {
        self.sc.faults.iter().filter(|f| f.active(self.t_us)).find_map(|f| pick(&f.kind))
    }

    fn fault_active(&self, kind: FaultKind) -> bool {
        self.active_fault(|k| (*k == kind).then_some(())).is_some()
    }

    /// Runs the whole scenario.
    pub fn run(&mut self) {
        while !self.done() {
            self.next_event();
        }
        self.monitor.finish(&self.world, &self.sc, self.t_us);
    }

    /// Processes events up to and including time `t_us`.
    pub fn run_until(&mut self, t_us: u64) {
        while let Some(Reverse((t, _, _))) = self.queue.peek() {
            if *t > t_us || self.rebooted_to_bootloader {
                break;
            }
            self.next_event();
        }
        self.advance_physics(t_us);
        self.t_us = self.t_us.max(t_us);
    }

    /// Steps physics on a fixed 250 µs grid up to `t`. Sensors read the
    /// newest grid state, so results do not depend on how a caller slices
    /// time (batch run versus frame-by-frame in the visualizer).
    fn advance_physics(&mut self, t: u64) {
        while self.physics_us + PHYSICS_DT_US <= t {
            self.physics_us += PHYSICS_DT_US;
            self.apply_world_faults();
            self.world.step(PHYSICS_DT_US as f32 * 1e-6, self.physics_us, &mut self.rng_env);
        }
    }

    fn apply_world_faults(&mut self) {
        let t = self.physics_us;
        let mut wind = self.sc.wind.mean;
        if self.sc.moved_at_boot && t < 1_400_000 {
            // Picked up and turned over in the hand while booting, then set
            // back down level.
            let b = &mut self.world.body;
            if t + PHYSICS_DT_US >= 1_400_000 {
                b.att = ironloop_fc::math::Quat::from_euler(0.0, 0.0, self.sc.initial_yaw);
                b.omega = Vec3::ZERO;
            } else {
                let ts = t as f32 * 1e-6;
                b.omega = Vec3::new(1.2 * sinf(7.0 * ts), 0.9 * cosf(5.0 * ts), 0.5 * sinf(3.0 * ts));
            }
            b.vel = Vec3::ZERO;
        }
        for f in &self.sc.faults {
            let active = f.active(t);
            match f.kind {
                FaultKind::Kick { torque } | FaultKind::Impact { torque } if active => {
                    self.world.kick = Some((f.end_us(), torque));
                }
                FaultKind::MotorDesync { motor } => {
                    let m = &mut self.world.motors[motor as usize];
                    if active {
                        m.health = MotorHealth::Desync;
                    } else if m.health == MotorHealth::Desync && t >= f.at_us {
                        m.health = MotorHealth::Ok;
                    }
                }
                FaultKind::MotorDead { motor } if active => {
                    self.world.motors[motor as usize].health = MotorHealth::Dead;
                }
                FaultKind::MotorDegraded { motor, factor } if active => {
                    let m = &mut self.world.motors[motor as usize];
                    if m.health == MotorHealth::Ok {
                        m.health = MotorHealth::Degraded(factor);
                    }
                }
                FaultKind::WindShift { wind: w } if active => wind = w,
                _ => {}
            }
        }
        self.world.wind.mean = wind;
    }

    pub fn next_event(&mut self) {
        let Some(Reverse((t, _, ev))) = self.queue.pop() else {
            return;
        };
        self.advance_physics(t);
        self.t_us = t;
        match ev {
            Ev::Imu => self.on_imu(),
            Ev::Rc => self.on_rc(),
            Ev::Battery => {
                let b = &self.world.battery;
                let gain = 1.0 + 0.01 * ((self.sc.seed % 7) as f32 - 3.0);
                let volts = if b.absent {
                    0.05 + 0.02 * self.rng_esc.f32()
                } else {
                    ((b.volts + 0.01 * self.rng_esc.normal()) * gain * 100.0).round() / 100.0
                };
                // The board corrects the Mobula6 current sensor's 25% error.
                let amps = b.amps;
                self.deliver(EventKind::Battery { volts, amps });
                self.schedule(t + BATTERY_US, Ev::Battery);
            }
            Ev::Telemetry => {
                let lost_all = self.fault_active(FaultKind::TelemetryLoss);
                let mut erpm = [None; 4];
                for (i, e) in erpm.iter_mut().enumerate() {
                    // Each ESC reports the motor it drives, wherever that is.
                    let m = self.sc.wired(i);
                    let lost = self.rng_esc.chance(self.sc.telemetry_loss);
                    if !lost_all && !lost && self.world.motors[m].command != 0 {
                        // The ESC measures the commutation period and sends
                        // it as a GCR reply. A glitch on the wire adds two
                        // edges; the checksum rejects most of those, but
                        // about 1% read as a wrong eRPM.
                        let noise = 1.0 + 0.01 * self.rng_esc.normal();
                        let mut reply = erpm_reply((self.world.erpm(m) as f32 * noise) as u32);
                        if self.rng_esc.chance(self.sc.telemetry_loss) {
                            reply ^= 0b11 << self.rng_esc.range_u64(0, 19);
                        }
                        *e = decode_erpm_reply(reply);
                    }
                }
                if erpm.iter().any(Option::is_some) {
                    self.deliver(EventKind::MotorTelemetry { erpm });
                }
                self.schedule(t + TELEMETRY_US, Ev::Telemetry);
            }
            Ev::Tick => {
                self.deliver(EventKind::Tick);
                self.schedule(t + TICK_US, Ev::Tick);
            }
            Ev::ImuDeliver => {
                if let Some((stamp, kind)) = self.imu_read.take() {
                    self.deliver_at(stamp, kind);
                }
                for (stamp, kind) in std::mem::take(&mut self.held) {
                    self.deliver_at(stamp, kind);
                }
                if let Some((stamp, kind)) = self.imu_waiting.take() {
                    self.read_imu(stamp, kind);
                }
            }
            Ev::ApplyMotors => {
                if let Some(cmd) = self.pending_motors.pop_front() {
                    for (i, c) in cmd.into_iter().enumerate() {
                        let m = &mut self.world.motors[self.sc.wired(i)];
                        m.command = c;
                        m.last_frame_us = t;
                    }
                }
            }
        }
    }

    fn on_imu(&mut self) {
        let t = self.t_us;
        let imu = self.sc.imu.clone();
        let period = 1e6 / imu.odr_hz as f64;
        self.imu_next_ideal += period;
        let jitter = self
            .active_fault(|k| match *k {
                FaultKind::TimingJitter { jitter_us } => Some(jitter_us),
                _ => None,
            })
            .unwrap_or(imu.jitter_us);
        let next = (self.imu_next_ideal as u64 + self.rng_imu.range_u64(0, jitter + 1)).max(t + 1);
        self.schedule(next, Ev::Imu);

        if self.fault_active(FaultKind::ImuDropout) {
            return;
        }
        if self.fault_active(FaultKind::ImuStuck) {
            if let Some((g, a)) = self.last_imu_raw {
                self.deliver_imu(EventKind::Imu { gyro: g, accel: a });
                return;
            }
        }

        let w = &self.world;
        let ts = t as f32 * 1e-6;
        let drift = imu.gyro_bias_drift * ts.min(60.0);
        let mut gyro = w.body.omega.mul_elem(imu.gyro_scale) + imu.gyro_bias + drift;
        let mut accel = w.specific_force() + imu.accel_bias;
        // Prop imbalance vibration at each motor's shaft frequency, at the
        // shaft angle at the sample time. (Reading the angle from the 250 µs
        // physics grid instead makes the IMU see the same few angles over
        // and over, which turns vibration into a fake constant bias.)
        let since_grid = (t - self.physics_us) as f32 * 1e-6;
        for (i, m) in w.motors.iter().enumerate() {
            let r = m.omega / w.frame.omega_max;
            let k = r * r;
            let ph = m.phase + m.omega * since_grid + i as f32;
            gyro += Vec3::new(sinf(ph), cosf(ph), 0.3 * sinf(2.0 * ph)) * (imu.vib_gyro * k);
            accel += Vec3::new(cosf(ph), sinf(ph), 1.5 * sinf(ph + 0.7)) * (imu.vib_accel * k);
        }
        let r = &mut self.rng_imu;
        gyro += Vec3::new(r.normal(), r.normal(), r.normal()) * imu.gyro_noise;
        accel += Vec3::new(r.normal(), r.normal(), r.normal()) * imu.accel_noise;
        // The chip filters what it samples before the firmware reads it.
        if imu.chip_lpf_hz > 0.0 {
            let (g, a, last) = self.chip_lpf.get_or_insert(([gyro; 2], [accel; 2], t));
            let dt = (t - *last) as f32 * 1e-6;
            *last = t;
            let k = dt / (dt + 1.0 / (2.0 * core::f32::consts::PI * imu.chip_lpf_hz));
            g[0] += (gyro - g[0]) * k;
            g[1] += (g[0] - g[1]) * k;
            a[0] += (accel - a[0]) * k;
            a[1] += (a[0] - a[1]) * k;
            (gyro, accel) = (g[1], a[1]);
        }

        let to_sensor = |v: Vec3| match imu.mount {
            ImuAlignment::Cw0 => v,
            ImuAlignment::Cw90 => Vec3::new(v.y, -v.x, v.z),
            ImuAlignment::Cw180 => Vec3::new(-v.x, -v.y, v.z),
            ImuAlignment::Cw270 => Vec3::new(-v.y, v.x, v.z),
        };
        let quant = |v: Vec3, range: f32| {
            let lsb = range / 32768.0;
            let q = |x: f32| (x.clamp(-range, range - lsb) / lsb).round() * lsb;
            [q(v.x), q(v.y), q(v.z)]
        };
        let mut g = quant(to_sensor(gyro), imu.gyro_range);
        let mut a = quant(to_sensor(accel), imu.accel_range);

        if let Some(p) = self.active_fault(|k| match *k {
            FaultKind::ImuGarbage { p } => Some(p),
            _ => None,
        }) {
            let r = &mut self.rng_imu;
            if r.chance(p) {
                match r.range_u64(0, 10) {
                    0 => g[r.range_u64(0, 3) as usize] = f32::NAN,
                    1..=4 => g[r.range_u64(0, 3) as usize] = imu.gyro_range * r.sign(),
                    5..=7 => {
                        for v in g.iter_mut() {
                            *v = r.range(-imu.gyro_range, imu.gyro_range);
                        }
                    }
                    _ => a = [0.0; 3],
                }
            }
        }
        self.last_imu_raw = Some((g, a));
        self.deliver_imu(EventKind::Imu { gyro: g, accel: a });
    }

    /// The board stamps a sample at data-ready (in the interrupt) and steps
    /// it after the SPI read.
    fn deliver_imu(&mut self, kind: EventKind) {
        self.data_ready(self.t_us, kind);
    }

    fn data_ready(&mut self, stamp: u64, kind: EventKind) {
        if self.fault_active(FaultKind::FcStall) {
            self.stalled_imu = Some((stamp, kind));
        } else if self.imu_read.is_some() {
            self.imu_waiting = Some((stamp, kind));
        } else {
            self.read_imu(stamp, kind);
        }
    }

    fn read_imu(&mut self, stamp: u64, kind: EventKind) {
        self.imu_read = Some((stamp, kind));
        self.schedule(self.t_us + self.sc.imu.read_delay_us, Ev::ImuDeliver);
    }

    fn on_rc(&mut self) {
        let t = self.t_us;
        let period = (1e6 / self.sc.rc.rate_hz) as u64;
        let next = t + period + self.rng_rc.range_u64(0, self.sc.rc.jitter_us + 1);
        self.schedule(next, Ev::Rc);

        let sticks = match self.manual {
            Some(s) => s,
            None => self.pilot.sticks(&self.sc, &self.world, &self.status, &mut self.rng_rc, t, period as f32 * 1e-6),
        };
        self.sticks = sticks;
        for i in 0..self.sc.faults.len() {
            let f = self.sc.faults[i];
            if f.kind == FaultKind::RebootRequest && t >= f.at_us && !self.one_shot_done[i] {
                self.one_shot_done[i] = true;
                self.deliver(EventKind::Command(ironloop_fc::Command::RebootToBootloader));
            }
        }
        if self.fault_active(FaultKind::RcLoss) {
            return;
        }
        let mut channels = sticks.channels();
        if let Some(p) = self.active_fault(|k| match *k {
            FaultKind::RcGlitch { p } => Some(p),
            _ => None,
        }) {
            if self.rng_rc.chance(p) {
                if self.rng_rc.chance(0.5) {
                    let i = self.rng_rc.range_u64(0, RC_CHANNELS as u64) as usize;
                    channels[i] = *self.rng_rc.pick(&[0u16, 172, 3000, 65535, 2200, 700]);
                } else {
                    // Corruption that passes the CRC: a plausible value on a
                    // stick or the mode switch (the arm channel is left
                    // alone; see the README).
                    let i = *self.rng_rc.pick(&[0usize, 1, 2, 3, 5]);
                    channels[i] = self.rng_rc.range_u64(988, 2013) as u16;
                }
            }
        }
        self.deliver(EventKind::Rc { channels });
    }

    fn deliver(&mut self, kind: EventKind) {
        self.deliver_at(self.t_us, kind);
    }

    /// Passes an event stamped at `stamp` to the board's control task.
    fn deliver_at(&mut self, stamp: u64, kind: EventKind) {
        let is_imu = matches!(kind, EventKind::Imu { .. });
        if self.fault_active(FaultKind::FcStall) {
            match kind {
                EventKind::Imu { .. } => self.stalled_imu = Some((stamp, kind)),
                // The control task makes its own ticks; a stalled one makes none.
                EventKind::Tick => {}
                _ if self.stalled.len() < 32 => self.stalled.push((stamp, kind)),
                _ => {}
            }
            return;
        }
        // After a stall the task steps the queue first, then reads the newest
        // data-ready.
        for (stamp, kind) in std::mem::take(&mut self.stalled) {
            self.step_fc(stamp, kind);
        }
        if let Some((stamp, kind)) = self.stalled_imu.take() {
            self.data_ready(stamp, kind);
        }
        if !is_imu && self.imu_read.is_some() {
            self.held.push((stamp, kind));
            return;
        }
        if let Some(p) = self.active_fault(|k| match *k {
            FaultKind::EventDrop { p } => Some(p),
            _ => None,
        }) {
            if self.rng_board.chance(p) {
                return;
            }
        }
        self.step_fc(stamp, kind);
    }

    fn step_fc(&mut self, stamp: u64, kind: EventKind) {
        let mut t_fc = stamp + self.sc.clock_offset_us;
        for (i, f) in self.sc.faults.iter().enumerate() {
            if let FaultKind::ClockGlitch { back_us } = f.kind {
                if self.t_us >= f.at_us && !self.one_shot_done[i] {
                    self.one_shot_done[i] = true;
                    t_fc = t_fc.saturating_sub(back_us);
                }
            }
        }
        let event = Event { t_us: t_fc, kind };
        let before = self.status;
        let state = self.fc.take().expect("fc state present");
        let (state, fx) = step(state, &event);
        let status = state.status();
        self.fc = Some(state);
        self.status = status;
        self.events_delivered += 1;

        hash_debug(&mut self.trace_hash, &event);
        hash_debug(&mut self.trace_hash, &fx);

        for e in &fx {
            match *e {
                Effect::Motors(m) => {
                    hash_debug(&mut self.motor_hash, &m);
                    self.pending_motors.push_back(m);
                    self.schedule(self.t_us + MOTOR_LATENCY_US, Ev::ApplyMotors);
                }
                Effect::Led(on) => self.led = on,
                Effect::Beeper(on) => self.beeper = on,
                Effect::Notice(n) => {
                    if self.notices.len() < 10_000 {
                        self.notices.push((self.t_us, n));
                    }
                }
                Effect::RebootToBootloader => self.rebooted_to_bootloader = true,
            }
        }
        self.monitor.on_step(self.t_us, &event, &fx, &before, &status, &self.world, &self.sc);
    }
}

fn hash_debug<T: core::fmt::Debug>(h: &mut u64, v: &T) {
    use core::fmt::Write;
    struct Fnv<'a>(&'a mut u64);
    impl Write for Fnv<'_> {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            for b in s.bytes() {
                *self.0 = (*self.0 ^ b as u64).wrapping_mul(0x100_0000_01b3);
            }
            Ok(())
        }
    }
    let _ = write!(Fnv(h), "{v:?}");
}
