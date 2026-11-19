//! The deterministic discrete-event simulator. Every sensor sample, RC frame,
//! battery reading, ESC telemetry frame, and timer tick is an event on one
//! time-ordered queue; physics advances in fixed substeps between events,
//! and the firmware's `step` runs on each delivered event. Given a scenario,
//! the run is a pure function of its seed.

use core::f32::consts::{SQRT_2, TAU};
use std::cmp::Reverse;
use std::collections::{BinaryHeap, VecDeque};

use ironloop_fc::math::Vec3;
use ironloop_fc::proto::{CRSF_RC_CHANNELS_PACKED, CrsfParser, crsf_crc8, crsf_to_us, decode_erpm_reply, erpm_reply};
use ironloop_fc::{
    Effect, Event, EventKind, ImuAlignment, Notice, RC_CHANNELS, State, Status, step,
};
use libm::{cosf, sinf};

use crate::monitor::{Monitor, Step};
use crate::pilot::{Pilot, Sticks};
use crate::physics::World;
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
    /// The control task finishes what it was doing.
    TaskFree,
    /// The one-shot fault at this index in the scenario happens.
    Fault(usize),
}

#[derive(Clone, Debug)]
pub struct Sim {
    pub sc: Scenario,
    pub world: World,
    fc: State,
    pub t_us: u64,
    physics_us: u64,
    queue: BinaryHeap<Reverse<(u64, u64, Ev)>>,
    seq: u64,
    pending_motors: VecDeque<[u16; 4]>,
    task: Task,
    /// IMU samples lost because the control task had not read the one
    /// before (the board's `drdy_missed`).
    pub imu_missed: u64,
    /// The longest the control task was busy with one event, µs (the
    /// board's `step_cycles_max`, plus the SPI read for a sample).
    pub task_busy_max_us: u64,
    rng_board: Rng,
    /// Its own stream for the CPU time each step takes.
    rng_cpu: Rng,
    imu_next_ideal: f64,
    rng_imu: Rng,
    rng_rc: Rng,
    /// The board's CRSF parser, fed the receiver's bytes.
    crsf: CrsfParser,
    /// The virtual pilot's own stream, so modelling the link differently
    /// does not change how the pilot flies.
    rng_pilot: Rng,
    rng_env: Rng,
    rng_esc: Rng,
    pilot: Pilot,
    last_imu_raw: Option<([f32; 3], [f32; 3])>,
    /// When the IMU last sampled.
    imu_sampled_us: Option<u64>,
    /// The IMU chip's low-pass state: two stages each for gyro and accel.
    chip_lpf: Option<([Vec3; 2], [Vec3; 2])>,
    /// The frame resonance's oscillators for gyro and accel.
    frame: [Resonance; 2],
    /// Its own stream, so the frame's noise does not shift the sensor's.
    rng_vib: Rng,
    /// A clock jump back the firmware's next step will see.
    clock_back_us: u64,
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
    /// Times the receiver browned out.
    pub rx_brownouts: u32,
}

impl Sim {
    pub fn new(sc: Scenario) -> Self {
        let mut frame = sc.frame.clone();
        if sc.spin_reversed {
            frame.spin = frame.spin.map(|d| -d);
        }
        let mut world = World::new(frame, sc.battery.clone(), sc.wind.clone());
        let (r, p) = sc.ground_tilt;
        world.body.att = ironloop_fc::math::Quat::from_euler(r, p, sc.initial_yaw);
        if r != 0.0 || p != 0.0 {
            world.body.pos.z = -0.03;
        }
        let fc = State::new(sc.fc);
        let status = fc.status();
        let seed = sc.seed;
        let mut sim = Sim {
            world,
            fc,
            t_us: 0,
            physics_us: 0,
            queue: BinaryHeap::new(),
            seq: 0,
            pending_motors: VecDeque::new(),
            task: Task::default(),
            imu_missed: 0,
            task_busy_max_us: 0,
            rng_board: Rng::fork(seed, "board"),
            rng_cpu: Rng::fork(seed, "cpu"),
            imu_next_ideal: 0.0,
            rng_imu: Rng::fork(seed, "imu"),
            rng_rc: Rng::fork(seed, "rc"),
            crsf: CrsfParser::default(),
            rng_pilot: Rng::fork(seed, "pilot"),
            rng_env: Rng::fork(seed, "env"),
            rng_esc: Rng::fork(seed, "esc"),
            pilot: Pilot::default(),
            last_imu_raw: None,
            imu_sampled_us: None,
            chip_lpf: None,
            frame: [Resonance::default(); 2],
            rng_vib: Rng::fork(seed, "vib"),
            clock_back_us: 0,
            manual: None,
            sticks: Sticks::default(),
            monitor: Monitor::default(),
            trace_hash: 0xcbf2_9ce4_8422_2325,
            motor_hash: 0xcbf2_9ce4_8422_2325,
            notices: Vec::new(),
            status,
            led: false,
            beeper: false,
            events_delivered: 0,
            rebooted_to_bootloader: false,
            rx_brownouts: 0,
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
        for i in 0..sim.sc.faults.len() {
            sim.schedule_fault(i);
        }
        sim
    }

    fn schedule(&mut self, t: u64, ev: Ev) {
        self.seq += 1;
        self.queue.push(Reverse((t, self.seq, ev)));
    }

    /// A one-shot fault is an event at its start; the others are read off
    /// the scenario while they are active.
    fn schedule_fault(&mut self, i: usize) {
        let f = self.sc.faults[i];
        if f.kind.one_shot() {
            self.schedule(f.at_us, Ev::Fault(i));
        }
    }

    pub fn fc_state(&self) -> &State {
        &self.fc
    }

    pub fn done(&self) -> bool {
        self.t_us >= self.sc.duration_us || self.rebooted_to_bootloader
    }

    /// Adds a fault at the current time (used by the interactive visualizer).
    pub fn inject(&mut self, kind: FaultKind, dur_us: u64) {
        self.sc.faults.push(Fault { at_us: self.t_us, dur_us, kind });
        self.schedule_fault(self.sc.faults.len() - 1);
    }

    /// Ends every fault that is still running (motors recover, the wind
    /// settles back to its mean).
    pub fn clear_faults(&mut self) {
        let t = self.t_us;
        for f in &mut self.sc.faults {
            if f.active(t) {
                f.dur_us = t.saturating_sub(f.at_us);
            }
        }
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
            self.check_brownout();
        }
    }

    /// A sag under the receiver's brownout voltage reboots it: no RC frames
    /// until it has booted and found the transmitter again, which takes
    /// 0.5-1.5 s (estimated). The firmware sees a lost link.
    fn check_brownout(&mut self) {
        let (t, b) = (self.physics_us, &self.world.battery);
        if b.absent || b.volts >= self.sc.rx_brownout_v || self.fault_active_at(FaultKind::RcLoss, t) {
            return;
        }
        let dur_us = self.rng_env.range_u64(500_000, 1_500_000);
        self.sc.faults.push(Fault { at_us: t, dur_us, kind: FaultKind::RcLoss });
        self.rx_brownouts += 1;
    }

    fn fault_active_at(&self, kind: FaultKind, t: u64) -> bool {
        self.sc.faults.iter().any(|f| f.kind == kind && f.active(t))
    }

    fn apply_world_faults(&mut self) {
        let t = self.physics_us;
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
        // The world's faulted parts follow the faults active now, so a
        // fault that ends (or is cleared) leaves nothing to repair.
        let active = || self.sc.faults.iter().filter(move |f| f.active(t)).map(|f| f.kind);
        self.world.wind.mean = active()
            .filter_map(|k| match k {
                FaultKind::WindShift { wind } => Some(wind),
                _ => None,
            })
            .last()
            .unwrap_or(self.sc.wind.mean);
        self.world.kick = active()
            .filter_map(|k| match k {
                FaultKind::Kick { torque } | FaultKind::Impact { torque } => Some(torque),
                _ => None,
            })
            .last();
        for (i, m) in self.world.motors.iter_mut().enumerate() {
            m.health = self.sc.motor_health(i, t);
        }
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
                let volts = if b.absent {
                    0.05 + 0.02 * self.rng_esc.f32()
                } else {
                    ((b.volts + 0.01 * self.rng_esc.normal()) * self.sc.vbat_gain * 100.0).round() / 100.0
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
            Ev::TaskFree => self.run_task(),
            Ev::Fault(i) => match self.sc.faults[i].kind {
                FaultKind::ClockGlitch { back_us } => self.clock_back_us += back_us,
                // The configurator asks over USB, whatever the RC link does.
                FaultKind::RebootRequest => self.deliver(EventKind::Command(ironloop_fc::Command::RebootToBootloader)),
                _ => {}
            },
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
        let r = &mut self.rng_imu;
        gyro += Vec3::new(r.normal(), r.normal(), r.normal()) * imu.gyro_noise;
        accel += Vec3::new(r.normal(), r.normal(), r.normal()) * imu.accel_noise;
        // The chip filters what it samples before the firmware reads it.
        let dt = self.imu_sampled_us.replace(t).map_or(0.0, |last| (t - last) as f32 * 1e-6);
        if imu.chip_lpf_hz > 0.0 {
            let (g, a) = self.chip_lpf.get_or_insert(([gyro; 2], [accel; 2]));
            let k = dt / (dt + 1.0 / (TAU * imu.chip_lpf_hz));
            g[0] += (gyro - g[0]) * k;
            g[1] += (g[0] - g[1]) * k;
            a[0] += (accel - a[0]) * k;
            a[1] += (a[0] - a[1]) * k;
            (gyro, accel) = (g[1], a[1]);
        }

        // Vibration, as tones at each motor's shaft speed and its
        // harmonics, at the shaft angle at the sample time. (Reading the
        // angle from the 250 µs physics grid instead makes the IMU see the
        // same few angles over and over, which turns vibration into a fake
        // constant bias.) Each tone passes the chip's filters at its true
        // frequency before it is sampled, so a tone above half the sample
        // rate arrives cut and then aliased, as on the chip.
        let chip = |hz: f32| {
            let mut gain = 1.0;
            let mut lag = 0.0;
            for fc in imu.aaf_hz.into_iter().filter(|&fc| fc > 0.0) {
                let (re, im) = (1.0 - (hz / fc) * (hz / fc), SQRT_2 * hz / fc);
                gain /= libm::hypotf(re, im);
                lag += libm::atan2f(im, re);
            }
            (gain, lag)
        };
        let since_grid = (t - self.physics_us) as f32 * 1e-6;
        let mut thrust = 0.0;
        for (i, m) in w.motors.iter().enumerate() {
            let r = m.omega / w.frame.omega_max;
            let k = r * r;
            thrust += k / 4.0;
            let ph = m.phase + m.omega * since_grid + i as f32;
            let tone = |h: f32| {
                let (gain, lag) = chip(h * m.omega / TAU);
                (h * ph - lag, gain * k)
            };
            // Prop imbalance at shaft speed, twice it on yaw.
            let (p1, k1) = tone(1.0);
            let (p2, k2) = tone(2.0);
            gyro += Vec3::new(sinf(p1) * k1, cosf(p1) * k1, 0.3 * sinf(p2) * k2) * imu.vib_gyro;
            accel += Vec3::new(cosf(p1), sinf(p1), 1.5 * sinf(p1 + 0.7)) * (imu.vib_accel * k1);
            // Blade pass: mostly thrust pulses, so along z on the
            // accelerometer, and roll and pitch from the arm they act on.
            let (p3, k3) = tone(3.0);
            gyro += Vec3::new(sinf(p3), cosf(p3), 0.5 * sinf(p3 + 1.1)) * (imu.blade_gyro * k3);
            accel += Vec3::new(0.3 * cosf(p3), 0.3 * sinf(p3), sinf(p3 + 0.4)) * (imu.blade_accel * k3);
        }
        // A frame resonance rung by the airflow, scaled to its size at this
        // thrust.
        if (imu.frame_gyro > 0.0 || imu.frame_accel > 0.0) && dt > 0.0 {
            let r = &mut self.rng_vib;
            let g = self.frame[0].step(imu.frame_hz, imu.frame_q, dt, r);
            let a = self.frame[1].step(imu.frame_hz, imu.frame_q, dt, r);
            let size = thrust * chip(imu.frame_hz).0;
            gyro += g * (imu.frame_gyro * size);
            accel += a * (imu.frame_accel * size);
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
        let [sx, sy, sz] = imu.axis_sign;
        let flip = |v: Vec3| Vec3::new(v.x * sx, v.y * sy, v.z * sz);
        let mut g = quant(flip(to_sensor(gyro)), imu.gyro_range);
        let mut a = quant(flip(to_sensor(accel)), imu.accel_range);

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
        self.deliver(kind);
    }

    fn on_rc(&mut self) {
        let t = self.t_us;
        let period = (1e6 / self.sc.rc.rate_hz) as u64;
        let next = t + period + self.rng_rc.range_u64(0, self.sc.rc.jitter_us + 1);
        self.schedule(next, Ev::Rc);

        let sticks = match self.manual {
            Some(s) => s,
            None => self.pilot.sticks(&self.sc, &self.world, &self.status, &mut self.rng_pilot, t, period as f32 * 1e-6),
        };
        self.sticks = sticks;
        if self.fault_active(FaultKind::RcLoss) {
            return;
        }
        let mut channels = sticks.channels();
        let mut noise = false;
        if let Some(p) = self.active_fault(|k| match *k {
            FaultKind::RcGlitch { p } => Some(p),
            _ => None,
        }) {
            if self.rng_rc.chance(p) {
                if self.rng_rc.chance(0.5) {
                    noise = true;
                } else {
                    // Corruption that passes the CRC: any 11-bit value on a
                    // stick or the mode switch (the arm channel is left
                    // alone; see the README).
                    let i = *self.rng_rc.pick(&[0usize, 1, 2, 3, 5]);
                    channels[i] = crsf_to_us(self.rng_rc.range_u64(0, 2048) as u16);
                }
            }
        }
        // The receiver's bytes, as the board's UART hands them to the
        // parser: now and then a link-statistics frame first, which the
        // parser skips.
        let mut bytes = Vec::with_capacity(40);
        if self.rng_rc.chance(0.2) {
            bytes.extend_from_slice(&crsf_frame(0x14, &[0x5A, 0x60, 100, 8, 0, 4, 2, 0x58, 100, 7]));
        }
        bytes.extend_from_slice(&crsf_rc_frame(&channels));
        if noise {
            // Noise on the wire flips a few bits. The CRC rejects the frame;
            // a hit on the length byte can cost the next frame too.
            for _ in 0..self.rng_rc.range_u64(1, 4) {
                let i = self.rng_rc.range_u64(0, bytes.len() as u64) as usize;
                bytes[i] ^= 1 << self.rng_rc.range_u64(0, 8);
            }
        }
        for b in bytes {
            if let Some(channels) = self.crsf.push(b) {
                self.deliver(EventKind::Rc { channels });
            }
        }
    }

    fn deliver(&mut self, kind: EventKind) {
        self.deliver_at(self.t_us, kind);
    }

    /// Passes an event stamped at `stamp` to the board's control task.
    fn deliver_at(&mut self, stamp: u64, kind: EventKind) {
        if let Some(p) = self.active_fault(|k| match *k {
            FaultKind::EventDrop { p } => Some(p),
            _ => None,
        }) {
            if self.rng_board.chance(p) {
                return;
            }
        }
        let task = &mut self.task;
        match kind {
            // Data-ready only keeps the newest sample.
            EventKind::Imu { .. } => {
                if task.drdy.replace((stamp, kind)).is_some() {
                    self.imu_missed += 1;
                }
            }
            // The task's own timer: one tick however late it gets to it.
            EventKind::Tick => task.tick_due = true,
            _ if task.queue.len() < EVENT_QUEUE => task.queue.push_back((stamp, kind)),
            _ => {}
        }
        self.run_task();
    }

    /// Runs the control task until it is busy or has nothing to do. Like
    /// the board's loop, it takes queued events first, then the newest
    /// data-ready (reading it over SPI, then stepping it), then the tick,
    /// which it stamps when it gets to it.
    fn run_task(&mut self) {
        let t = self.t_us;
        if self.task.busy_until > t {
            return;
        }
        // A stalled task does nothing until the stall ends.
        if let Some(end) = self.sc.faults.iter().find(|f| f.kind == FaultKind::FcStall && f.active(t)).map(Fault::end_us) {
            self.busy(end);
            return;
        }
        let (stamp, kind, read_us) = if let Some((stamp, kind)) = self.task.reading.take() {
            (stamp, kind, self.sc.imu.read_delay_us)
        } else if let Some((stamp, kind)) = self.task.queue.pop_front() {
            (stamp, kind, 0)
        } else if let Some(sample) = self.task.drdy.take() {
            self.task.reading = Some(sample);
            self.busy(t + self.sc.imu.read_delay_us);
            return;
        } else if std::mem::take(&mut self.task.tick_due) {
            (t, EventKind::Tick, 0)
        } else {
            return;
        };
        let cost = self.cpu_us(&kind);
        self.task_busy_max_us = self.task_busy_max_us.max(read_us + cost);
        self.step_fc(stamp, kind, cost);
        self.busy(t + cost);
    }

    fn busy(&mut self, until: u64) {
        self.task.busy_until = until;
        self.schedule(until, Ev::TaskFree);
    }

    /// CPU time to step `kind` and apply its effects on the board, µs:
    /// instructions per event as counted for `step` on a Cortex-M4 under
    /// QEMU (`cd0b6d3`), plus the loop, the executor, and building the
    /// DShot frame, at the scenario's cycles per instruction and 96 MHz.
    fn cpu_us(&mut self, kind: &EventKind) -> u64 {
        let (lo, hi) = match kind {
            EventKind::Imu { .. } => (3_500.0, 7_100.0),
            EventKind::MotorTelemetry { .. } => (1_000.0, 2_000.0),
            _ => (300.0, 500.0),
        };
        let instructions = self.rng_cpu.range(lo, hi) + 1_000.0;
        (instructions * self.sc.cycles_per_instruction / 96.0) as u64 + 1
    }

    /// Steps the firmware on one event; its motor command goes out when the
    /// step is done, `cost_us` later.
    fn step_fc(&mut self, stamp: u64, kind: EventKind, cost_us: u64) {
        let t_fc = (stamp + self.sc.clock_offset_us).saturating_sub(std::mem::take(&mut self.clock_back_us));
        let event = Event { t_us: t_fc, kind };
        let before = self.status;
        let fx = step(&mut self.fc, &event);
        let status = self.fc.status();
        self.status = status;
        self.events_delivered += 1;

        hash_debug(&mut self.trace_hash, &event);
        hash_debug(&mut self.trace_hash, &fx);

        for e in &fx {
            match *e {
                Effect::Motors(m) => {
                    hash_debug(&mut self.motor_hash, &m);
                    self.pending_motors.push_back(m);
                    self.schedule(self.t_us + cost_us + MOTOR_LATENCY_US, Ev::ApplyMotors);
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
        self.monitor.on_step(Step { t: self.t_us, event: &event, fx: &fx, before: &before, after: &status, world: &self.world, sc: &self.sc });
    }
}

/// A CRSF frame to the flight controller: address, length, type, payload,
/// CRC.
fn crsf_frame(kind: u8, payload: &[u8]) -> Vec<u8> {
    let mut f = vec![0xC8, payload.len() as u8 + 2, kind];
    f.extend_from_slice(payload);
    f.push(crsf_crc8(&f[2..]));
    f
}

/// An RC channels frame as an ExpressLRS receiver sends it: 16 channels of
/// 11 bits (172..1811 for 988..2012 µs), the ones past ours centered.
fn crsf_rc_frame(us: &[u16; RC_CHANNELS]) -> Vec<u8> {
    let mut payload = [0u8; 22];
    for i in 0..16 {
        let v = us.get(i).map_or(992, |&us| (992 + ((us as i32 - 1500) * 8) / 5).clamp(0, 2047) as u32);
        for b in 0..11 {
            if v >> b & 1 != 0 {
                let bit = i * 11 + b;
                payload[bit / 8] |= 1 << (bit % 8);
            }
        }
    }
    crsf_frame(CRSF_RC_CHANNELS_PACKED, &payload)
}

/// Depth of the board's `EVENTS` queue; a full one drops what arrives.
const EVENT_QUEUE: usize = 32;

/// The board's control task (`control.rs`): one loop steps every event.
/// While it is busy, reading a sample over SPI or stepping, RC, battery and
/// telemetry wait in its queue, the newest data-ready waits (an older
/// unread one is lost), and so does its 1 ms timer.
#[derive(Clone, Debug, Default)]
struct Task {
    busy_until: u64,
    /// The sample being read, stamped at data-ready.
    reading: Option<(u64, EventKind)>,
    queue: VecDeque<(u64, EventKind)>,
    drdy: Option<(u64, EventKind)>,
    tick_due: bool,
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

/// A resonance rung by noise: a damped oscillator at `hz` with quality
/// factor `q`, kicked by white noise. Each kick only stands in for the noise
/// over its interval when that is short against the period, so an interval
/// is split into steps of at most a sixteenth of one, each solved exactly:
/// it stays stable and rings the same whatever the sample timing. Its output
/// has unit RMS on each axis.
#[derive(Clone, Copy, Debug, Default)]
struct Resonance {
    x: Vec3,
    v: Vec3,
}

impl Resonance {
    fn step(&mut self, hz: f32, q: f32, dt: f32, r: &mut Rng) -> Vec3 {
        let n = libm::ceilf(dt * hz * 16.0).max(1.0);
        for _ in 0..n as u32 {
            self.substep(hz, q, dt / n, r);
        }
        self.x
    }

    fn substep(&mut self, hz: f32, q: f32, dt: f32, r: &mut Rng) {
        let w = TAU * hz;
        let sigma = w / (2.0 * q);
        let wd = libm::sqrtf(w * w - sigma * sigma);
        let decay = libm::expf(-sigma * dt);
        let (s, c) = (sinf(wd * dt), cosf(wd * dt));
        // Half its energy is in the velocity, so this leaves x at unit RMS.
        let kick = w * libm::sqrtf(2.0 * (1.0 - decay * decay));
        let (x, v) = (self.x, self.v);
        self.x = (x * c + (v + x * sigma) * (s / wd)) * decay;
        self.v = (v * c - (v * sigma + x * (w * w)) * (s / wd)) * decay;
        self.v += Vec3::new(r.normal(), r.normal(), r.normal()) * kick;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Steps a resonance at random intervals of `min_us..max_us` for
    /// `secs`, and returns the RMS of its x output and its power at each of
    /// `freqs` (averaged over 0.5 s stretches, as a DFT over the uneven
    /// sample times).
    fn ring(hz: f32, q: f32, min_us: u64, max_us: u64, secs: f64, freqs: &[f64]) -> (f64, Vec<f64>) {
        let (mut res, mut noise, mut timing) = (Resonance::default(), Rng::new(1), Rng::new(2));
        let (mut t, mut sq, mut n) = (0.0f64, 0.0f64, 0u64);
        let mut power = vec![0.0; freqs.len()];
        let mut bins = vec![(0.0f64, 0.0f64); freqs.len()];
        let mut stretch_end = 0.5;
        while t < secs {
            let dt = timing.range_u64(min_us, max_us + 1) as f64 * 1e-6;
            t += dt;
            let x = res.step(hz, q, dt as f32, &mut noise).x as f64;
            assert!(x.is_finite(), "diverged at {t:.3} s");
            if t < 1.0 {
                continue; // settling
            }
            sq += x * x;
            n += 1;
            for (b, f) in bins.iter_mut().zip(freqs) {
                let ph = core::f64::consts::TAU * f * t;
                b.0 += x * ph.cos() * dt;
                b.1 -= x * ph.sin() * dt;
            }
            if t >= stretch_end {
                for (p, b) in power.iter_mut().zip(bins.iter_mut()) {
                    *p += b.0 * b.0 + b.1 * b.1;
                    *b = (0.0, 0.0);
                }
                stretch_end += 0.5;
            }
        }
        ((sq / n as f64).sqrt(), power)
    }

    /// The frame resonance once grew without bound when the sample interval
    /// jittered, which the DST read as the gyro clipping on a quad sitting
    /// still. It must ring at its frequency, as sharply as its Q says, at
    /// unit RMS, at the IMU's rate with jitter and at intervals from 50 µs
    /// to far longer than its period.
    #[test]
    fn frame_resonance_rings_at_its_frequency_and_size() {
        for (hz, q) in [(150.0f32, 5.0f32), (300.0, 10.0), (600.0, 20.0)] {
            let f0 = hz as f64;
            // Half-power points are about f0 * (1 +- 1 / 2Q).
            let half = 1.0 / (2.0 * q as f64);
            let freqs = [0.5 * f0, f0 * (1.0 - half), f0, f0 * (1.0 + half), 1.5 * f0];
            let (rms, p) = ring(hz, q, 250, 400, 200.0, &freqs);
            assert!((0.93..1.07).contains(&rms), "{hz} Hz: RMS {rms}");
            let rel: Vec<f64> = p.iter().map(|x| x / p[2]).collect();
            assert!(rel[0] < 0.1 && rel[4] < 0.1, "{hz} Hz: not narrow: {rel:?}");
            assert!((0.3..0.75).contains(&rel[1]) && (0.3..0.75).contains(&rel[3]), "{hz} Hz: wrong width for Q {q}: {rel:?}");

            let (rms, _) = ring(hz, q, 50, 5000, 200.0, &[]);
            assert!((0.93..1.07).contains(&rms), "{hz} Hz: RMS {rms} with intervals of 50 us to 5 ms");
        }
    }
}
