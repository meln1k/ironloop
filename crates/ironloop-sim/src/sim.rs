//! The deterministic discrete-event simulator. Every sensor sample, RC frame,
//! battery reading, ESC telemetry frame, and timer tick is an event on one
//! time-ordered queue; physics advances in fixed substeps between events,
//! and the firmware's `step` runs on each delivered event. Given a scenario,
//! the run is a pure function of its seed.

use std::cmp::Reverse;
use std::collections::{BinaryHeap, VecDeque};

use ironloop_fc::math::{Vec3, tilt_error, up_in_body};
use ironloop_fc::{
    Effect, Event, EventKind, FlightMode, ImuAlignment, Notice, RC_CHANNELS, State, Status, step,
};
use libm::{atanf, cosf, sinf};

use crate::monitor::Monitor;
use crate::physics::{MotorHealth, World};
use crate::rng::Rng;
use crate::scenario::{Action, Fault, FaultKind, Scenario};

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

/// Stick positions: roll, pitch, yaw in -1..1 and throttle in 0..1.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Sticks {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub throttle: f32,
    pub arm: bool,
    pub acro: bool,
}

impl Sticks {
    pub fn channels(&self) -> [u16; RC_CHANNELS] {
        let us = |x: f32| (1500.0 + 500.0 * x.clamp(-1.0, 1.0)) as u16;
        [
            us(self.roll),
            us(self.pitch),
            (1000.0 + 1000.0 * self.throttle.clamp(0.0, 1.0)) as u16,
            us(self.yaw),
            if self.arm { 2000 } else { 1000 },
            if self.acro { 2000 } else { 1000 },
            1500,
            1500,
        ]
    }
}

#[derive(Clone, Debug, Default)]
struct Pilot {
    seg_start: u64,
    anchor: Vec3,
    alt_i: f32,
    hand: Sticks,
    slam: Sticks,
    slam_next: u64,
    flip_angle: f32,
    hang_since: Option<u64>,
    hang_done: bool,
    land_alt: f32,
    low_batt_land: bool,
    finishing_landing: bool,
    crit_since: Option<u64>,
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
    /// IMU samples stamped at data-ready, waiting for their SPI read.
    pending_imu: VecDeque<(u64, EventKind)>,
    /// Events held in the board's 32-deep queue while the firmware stalls.
    stalled: Vec<(u64, EventKind)>,
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
            pending_imu: VecDeque::new(),
            stalled: Vec::new(),
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

    fn fault_active(&self, pred: impl Fn(&FaultKind) -> bool) -> Option<Fault> {
        self.sc.faults.iter().copied().find(|f| f.active(self.t_us) && pred(&f.kind))
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
        for (i, f) in self.sc.faults.iter().enumerate() {
            let active = f.active(t);
            match f.kind {
                FaultKind::Kick { torque } | FaultKind::Impact { torque } if active => {
                    self.world.kick = Some((f.at_us + f.dur_us, torque));
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
            let _ = i;
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
                // The Mobula6 current sensor reads about 25% high.
                let amps = b.amps * 1.25;
                self.deliver(EventKind::Battery { volts, amps });
                self.schedule(t + BATTERY_US, Ev::Battery);
            }
            Ev::Telemetry => {
                let lost_all = self.fault_active(|k| matches!(k, FaultKind::TelemetryLoss)).is_some();
                let mut erpm = [None; 4];
                for (i, e) in erpm.iter_mut().enumerate() {
                    // Each ESC reports the motor it drives, wherever that is.
                    let m = if self.sc.motors_swapped { [1, 0, 2, 3][i] } else { i };
                    let lost = self.rng_esc.chance(self.sc.telemetry_loss);
                    if !lost_all && !lost && self.world.motors[m].command != 0 {
                        let noise = 1.0 + 0.01 * self.rng_esc.normal();
                        *e = Some((self.world.erpm(m) as f32 * noise) as u32);
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
                if let Some((stamp, kind)) = self.pending_imu.pop_front() {
                    self.deliver_at(stamp, kind);
                }
            }
            Ev::ApplyMotors => {
                if let Some(cmd) = self.pending_motors.pop_front() {
                    let cmd = if self.sc.motors_swapped { [cmd[1], cmd[0], cmd[2], cmd[3]] } else { cmd };
                    for (m, c) in self.world.motors.iter_mut().zip(cmd) {
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
            .fault_active(|k| matches!(k, FaultKind::TimingJitter { .. }))
            .map_or(imu.jitter_us, |f| match f.kind {
                FaultKind::TimingJitter { jitter_us } => jitter_us,
                _ => unreachable!(),
            });
        let next = (self.imu_next_ideal as u64 + self.rng_imu.range_u64(0, jitter + 1)).max(t + 1);
        self.schedule(next, Ev::Imu);

        if self.fault_active(|k| matches!(k, FaultKind::ImuDropout)).is_some() {
            return;
        }
        if self.fault_active(|k| matches!(k, FaultKind::ImuStuck)).is_some() {
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

        if let Some(f) = self.fault_active(|k| matches!(k, FaultKind::ImuGarbage { .. })) {
            let FaultKind::ImuGarbage { p } = f.kind else { unreachable!() };
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

    /// The board stamps a sample at data-ready and queues it after the SPI
    /// read, so events other tasks stamp in between reach the firmware first.
    fn deliver_imu(&mut self, kind: EventKind) {
        self.pending_imu.push_back((self.t_us, kind));
        self.schedule(self.t_us + self.sc.imu.read_delay_us, Ev::ImuDeliver);
    }

    fn on_rc(&mut self) {
        let t = self.t_us;
        let period = (1e6 / self.sc.rc.rate_hz) as u64;
        let next = t + period + self.rng_rc.range_u64(0, self.sc.rc.jitter_us + 1);
        self.schedule(next, Ev::Rc);

        let sticks = match self.manual {
            Some(s) => s,
            None => self.pilot_sticks(period as f32 * 1e-6),
        };
        self.sticks = sticks;
        for i in 0..self.sc.faults.len() {
            let f = self.sc.faults[i];
            if f.kind == FaultKind::RebootRequest && t >= f.at_us && !self.one_shot_done[i] {
                self.one_shot_done[i] = true;
                self.deliver(EventKind::Command(ironloop_fc::Command::RebootToBootloader));
            }
        }
        if self.fault_active(|k| matches!(k, FaultKind::RcLoss)).is_some() {
            return;
        }
        let mut channels = sticks.channels();
        if let Some(f) = self.fault_active(|k| matches!(k, FaultKind::RcGlitch { .. })) {
            let FaultKind::RcGlitch { p } = f.kind else { unreachable!() };
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

    /// Passes an event stamped at `stamp` through the board's event queue.
    fn deliver_at(&mut self, stamp: u64, kind: EventKind) {
        if self.fault_active(|k| matches!(k, FaultKind::FcStall)).is_some() {
            if self.stalled.len() < 32 {
                self.stalled.push((stamp, kind));
            }
            return;
        }
        for (stamp, kind) in std::mem::take(&mut self.stalled) {
            self.step_fc(stamp, kind);
        }
        if let Some(f) = self.fault_active(|k| matches!(k, FaultKind::EventDrop { .. })) {
            let FaultKind::EventDrop { p } = f.kind else { unreachable!() };
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

    /// The virtual pilot: a human-like controller that sees the true state.
    fn pilot_sticks(&mut self, dt: f32) -> Sticks {
        let t = self.t_us;
        let Some(seg) = self.sc.segment_at(t).copied() else {
            return Sticks::default();
        };
        let next_start = self.sc.segments.iter().map(|s| s.start_us).find(|&s| s > t).unwrap_or(u64::MAX);
        let w = &self.world;
        let p = &mut self.pilot;
        if p.seg_start != seg.start_us {
            p.seg_start = seg.start_us;
            p.anchor = w.body.pos;
            p.flip_angle = 0.0;
            p.hang_since = None;
            p.hang_done = false;
            p.land_alt = w.altitude();
        }
        // A pilot lands once the critical battery warning keeps sounding
        // (a punch-out alone can sag a 1S pack to critical for a moment).
        let critical = self.status.battery_level == ironloop_fc::BatteryLevel::Critical;
        p.crit_since = if critical && self.status.armed { Some(p.crit_since.unwrap_or(t)) } else { None };
        if p.crit_since.is_some_and(|c| t - c >= 3_000_000) && !p.low_batt_land {
            p.low_batt_land = true;
            p.land_alt = w.altitude();
            p.anchor = w.body.pos;
        }
        let mut seg = seg;
        if p.low_batt_land && !matches!(seg.action, Action::Disarmed { .. }) {
            seg.action = Action::Land;
        }
        // A pilot never flips the arm switch off while still in the air
        // (the plan's timeline can run out during a slow failsafe descent).
        if matches!(seg.action, Action::Disarmed { .. }) && self.status.armed && !w.body.on_ground && w.altitude() > 0.15 {
            if !p.finishing_landing {
                p.finishing_landing = true;
                p.land_alt = w.altitude();
                p.anchor = w.body.pos;
            }
            seg.action = Action::Land;
        }
        let acro = seg.mode == FlightMode::Acro;
        let alt = w.altitude();
        let vel = w.body.vel;
        let up = w.body.att.inverse_rotate(Vec3::new(0.0, 0.0, -1.0));
        let (_, _, yaw) = w.body.att.to_euler();
        let fc_cfg = &self.sc.fc;

        let hold_alt = |p: &mut Pilot, target: f32| {
            let e = (target - alt).clamp(-1.5, 1.5);
            // Trim except on the way down: integrating through a long
            // descent winds up and turns a 15 m descent into a 4 m/s dive.
            if e > -0.5 {
                p.alt_i = (p.alt_i + 0.08 * e * dt).clamp(-0.25, 0.25);
            }
            let thr = 0.5 + p.alt_i + 0.14 * e + 0.09 * vel.z;
            let tilt = up.z.abs().max(0.5);
            (thr / tilt).clamp(0.05, 0.95)
        };
        // Desired tilt (roll, pitch) to drive horizontal velocity/position.
        let lean = |anchor: Option<Vec3>, roll0: f32, pitch0: f32| {
            let mut acc = -Vec3::new(vel.x, vel.y, 0.0) * 1.8;
            if let Some(a) = anchor {
                let e = a - w.body.pos;
                acc += Vec3::new(e.x, e.y, 0.0) * 1.2;
            }
            let lim = 5.0;
            let n = acc.norm();
            if n > lim {
                acc = acc * (lim / n);
            }
            let (c, s) = (cosf(yaw), sinf(yaw));
            let fwd = acc.x * c + acc.y * s;
            let right = -acc.x * s + acc.y * c;
            (roll0 + atanf(right / 9.81), pitch0 - atanf(fwd / 9.81))
        };
        // Convert desired attitude to sticks for the current mode.
        let attitude_sticks = |roll: f32, pitch: f32| -> (f32, f32) {
            if acro {
                let rate = tilt_error(up, up_in_body(roll, pitch)) * 5.0 - w.body.omega * 0.08;
                (inverse_rate(&fc_cfg.rates[0], rate.x), -inverse_rate(&fc_cfg.rates[1], rate.y))
            } else {
                (roll / fc_cfg.angle_limit, -pitch / fc_cfg.angle_limit)
            }
        };

        // A refused or lost arm (still calibrating, glitchy link, crash)
        // makes the pilot flip the switch off and on again.
        let rearm = self.status.armed || (t - seg.start_us) % 1_000_000 < 700_000;
        let mut out = Sticks { arm: rearm, acro, ..Sticks::default() };
        // Disarmed when it should be flying (arming refused while still
        // calibrating, or a safety disarm): throttle down and cycle the
        // switch until it arms, then carry on with the plan.
        let flight = matches!(seg.action, Action::Hover { .. } | Action::Cruise { .. } | Action::Flip { .. } | Action::Punch | Action::StickSlam | Action::StickScript { .. });
        let action = if flight && !self.status.armed { Action::Arm { throttle: 0.0 } } else { seg.action };
        match action {
            Action::Disarmed { throttle } => {
                out.arm = self.sc.arm_switch_at_boot && t + 500_000 < next_start && seg.start_us == 0;
                out.throttle = throttle;
                p.alt_i = 0.0;
            }
            Action::Arm { throttle } => {
                out.throttle = throttle;
            }
            Action::Hover { alt: target } => {
                out.throttle = if w.body.on_ground && t - seg.start_us < 300_000 {
                    0.6
                } else {
                    hold_alt(p, target)
                };
                let (r, pt) = lean(Some(p.anchor), 0.0, 0.0);
                (out.roll, out.pitch) = attitude_sticks(r, pt);
            }
            Action::Cruise { alt: target, roll, pitch } => {
                out.throttle = hold_alt(p, target);
                (out.roll, out.pitch) = attitude_sticks(roll, pitch);
                p.anchor = w.body.pos;
            }
            Action::Flip { axis, dir } => {
                let rate = if axis == 0 { w.body.omega.x } else { w.body.omega.y };
                p.flip_angle += (rate * dir).abs() * dt;
                if t - seg.start_us < 150_000 {
                    out.throttle = 0.8;
                } else if p.flip_angle < 5.4 {
                    out.throttle = 0.15;
                    if axis == 0 { out.roll = dir } else { out.pitch = dir }
                } else {
                    out.throttle = hold_alt(p, 2.5);
                    let (r, pt) = lean(None, 0.0, 0.0);
                    (out.roll, out.pitch) = attitude_sticks(r, pt);
                    p.anchor = w.body.pos;
                }
            }
            Action::InvertedHang { secs, yaw } => {
                if !p.hang_done {
                    match p.hang_since {
                        None if up.z > 0.95 => p.hang_since = Some(t),
                        None => {
                            out.roll = 1.0;
                            out.throttle = 0.3;
                        }
                        Some(h) if t - h >= (secs * 1e6) as u64 => p.hang_done = true,
                        Some(_) => {
                            // A little throttle keeps control authority.
                            out.yaw = yaw;
                            out.throttle = 0.12;
                        }
                    }
                }
                if p.hang_done {
                    if up.z > -0.7 {
                        // Keep rolling the same way, back upright.
                        out.roll = 1.0;
                        out.throttle = 0.3;
                    } else {
                        // Recover above where the longest hang ends.
                        out.throttle = hold_alt(p, 15.0);
                        let (r, pt) = lean(None, 0.0, 0.0);
                        (out.roll, out.pitch) = attitude_sticks(r, pt);
                        p.anchor = w.body.pos;
                    }
                }
            }
            Action::StickScript { amp } => {
                // Open loop on attitude: only the throttle reacts (to hold
                // altitude); the sticks follow the script.
                out.throttle = hold_alt(p, p.land_alt.max(3.0));
                if let Some((axis, k)) = crate::scenario::stick_script(t - seg.start_us) {
                    match axis {
                        0 => out.roll = amp * k,
                        1 => out.pitch = amp * k,
                        _ => out.yaw = amp * k,
                    }
                }
                p.anchor = w.body.pos;
            }
            Action::Punch => {
                out.throttle = 1.0;
                p.anchor = w.body.pos;
            }
            Action::StickSlam => {
                if t >= p.slam_next {
                    let r = &mut self.rng_rc;
                    p.slam = Sticks {
                        roll: r.range(-1.0, 1.0),
                        pitch: r.range(-1.0, 1.0),
                        yaw: r.range(-1.0, 1.0),
                        throttle: r.range(0.25, 0.9),
                        ..out
                    };
                    p.slam_next = t + 150_000;
                }
                out = p.slam;
                if alt < 1.0 {
                    out.throttle = 0.8;
                }
                p.anchor = w.body.pos;
            }
            Action::Land => {
                p.land_alt = (p.land_alt - 0.6 * dt).max(-0.5);
                out.throttle = if w.body.on_ground && alt < 0.05 { 0.0 } else { hold_alt(p, p.land_alt) };
                let (r, pt) = lean(Some(p.anchor), 0.0, 0.0);
                (out.roll, out.pitch) = attitude_sticks(r, pt);
            }
            Action::Drop => out.throttle = 0.0,
        }
        // The pilot's hands move fast but not instantly.
        let k = (dt / 0.03).min(1.0);
        let h = &mut p.hand;
        h.roll += (out.roll - h.roll) * k;
        h.pitch += (out.pitch - h.pitch) * k;
        h.yaw += (out.yaw - h.yaw) * k;
        h.throttle = out.throttle;
        h.arm = out.arm;
        h.acro = out.acro;
        if matches!(seg.action, Action::StickSlam | Action::Flip { .. } | Action::InvertedHang { .. } | Action::StickScript { .. }) {
            *h = out;
        }
        *h
    }
}

/// Stick deflection that produces `rate` (rad/s) with Betaflight rates.
fn inverse_rate(r: &ironloop_fc::Rates, rate: f32) -> f32 {
    let target = rate.to_degrees().abs();
    let (mut lo, mut hi) = (0.0f32, 1.0f32);
    for _ in 0..20 {
        let mid = 0.5 * (lo + hi);
        let x = mid * mid * mid * mid * r.expo + mid * (1.0 - r.expo);
        let v = 200.0 * r.rc_rate * x / (1.0 - mid * r.super_rate).clamp(0.01, 1.0);
        if v < target { lo = mid } else { hi = mid }
    }
    0.5 * (lo + hi) * rate.signum()
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
