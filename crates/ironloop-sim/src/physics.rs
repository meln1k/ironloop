//! Rigid-body quadrotor, motors, battery, wind, and ground contact.
//!
//! World frame is NED with the ground plane at z = 0 (z grows downward), the
//! body frame is FRD. All math goes through `libm` so results are identical
//! on native and wasm targets.

use ironloop_fc::math::{Quat, Vec3};
use ironloop_fc::{DSHOT_MAX, DSHOT_MIN, GRAVITY};
use libm::{expf, sqrtf};

use crate::rng::Rng;

/// Physical parameters. Defaults follow the HDZero Mobula6 ECO 2024 with a
/// 300 mAh 1S pack.
#[derive(Clone, Debug)]
pub struct Airframe {
    pub mass: f32,
    pub inertia: Vec3,
    /// Motor positions in the body frame, Betaflight order (RR, FR, RL, FL).
    pub motor_pos: [Vec3; 4],
    /// +1 when the prop spins clockwise seen from above.
    pub spin: [f32; 4],
    /// Thrust per (rad/s)².
    pub k_t: f32,
    /// Prop drag torque per unit thrust (m).
    pub k_q_ratio: f32,
    /// Prop speed at full command on `v_ref` volts (rad/s). With `kv` it
    /// sets the motor's winding resistance (see `winding_r`).
    pub omega_max: f32,
    pub v_ref: f32,
    /// Motor KV (RPM per volt, unloaded).
    pub kv: f32,
    /// Rotor inertia, bell and prop (kg·m²). With the winding resistance
    /// and the prop's drag it sets how fast a motor changes speed: about
    /// 40 ms at hover for 3e-8.
    pub rotor_inertia: f32,
    /// Bearing and iron losses as a constant torque (N·m): the no-load
    /// current times `k_e`.
    pub motor_friction: f32,
    /// Linear drag per axis (N per m/s), body frame.
    pub drag_lin: Vec3,
    /// Quadratic drag per axis (N per (m/s)²), body frame.
    pub drag_quad: Vec3,
    /// Rotor drag (N per m/s per rad/s of summed prop speed), horizontal.
    pub rotor_drag: f32,
    /// Rotational damping (N·m per rad/s).
    pub rot_damping: f32,
    pub pole_pairs: f32,
    /// How long the ESC takes to start a motor from standstill (µs): open
    /// loop at its startup power until it has synchronised.
    pub esc_start_us: u64,
    /// The speed (fraction of full) the ESC drives while starting.
    pub esc_start_power: f32,
    /// Below this speed (fraction of full) for `ESC_STALL_US` the back-EMF
    /// is too weak to follow and the ESC loses sync.
    pub esc_min_speed: f32,
    /// Bluejay's RPM power protection ("rampup"): the command is capped at
    /// `esc_start_power + esc_rampup * speed`, so the motor accelerates more
    /// slowly from low speed.
    pub esc_rampup: f32,
}

impl Default for Airframe {
    fn default() -> Self {
        let d = 0.02298 * core::f32::consts::FRAC_1_SQRT_2;
        Self {
            mass: 0.0314,
            inertia: Vec3::new(1.0e-5, 1.0e-5, 1.6e-5),
            motor_pos: [
                Vec3::new(-d, d, 0.0),
                Vec3::new(d, d, 0.0),
                Vec3::new(-d, -d, 0.0),
                Vec3::new(d, -d, 0.0),
            ],
            spin: [-1.0, 1.0, 1.0, -1.0],
            k_t: 4.2e-9,
            k_q_ratio: 0.0075,
            omega_max: 7576.0,
            v_ref: 3.9,
            kv: 28000.0,
            // Not measured: a 0702 bell (about 0.8 g of steel and magnets
            // at 4-5 mm) and a 31 mm tri-blade (about 0.2 g) come to
            // 2.5-3.5e-8.
            rotor_inertia: 3.0e-8,
            // Not measured: about 0.15 A of no-load current.
            motor_friction: 5.0e-5,
            drag_lin: Vec3::new(0.004, 0.004, 0.006),
            drag_quad: Vec3::new(0.0035, 0.0035, 0.006),
            rotor_drag: 3.0e-7,
            rot_damping: 2.0e-6,
            pole_pairs: 6.0,
            esc_start_us: 120_000,
            esc_start_power: 0.1,
            // The speed 4.5% duty settles at on `v_ref` (see `vary_hardware`).
            esc_min_speed: 0.053,
            esc_rampup: 3.0,
        }
    }
}

/// The motors are DC motors driven by the ESC's PWM (inductance left out:
/// its time constant is microseconds). Winding current is
/// `(applied − k_e·ω) / R`, its torque `k_e·i`, against the prop's drag
/// torque `k_q·ω²` and the motor's own friction.
impl Airframe {
    /// Back-EMF constant (V·s/rad), also the torque constant (N·m/A).
    pub fn k_e(&self) -> f32 {
        60.0 / (core::f32::consts::TAU * self.kv)
    }

    /// Prop drag torque per (rad/s)².
    pub fn k_q(&self) -> f32 {
        self.k_q_ratio * self.k_t
    }

    /// Winding resistance, with the ESC's and the wiring's (Ω): the one at
    /// which full command on `v_ref` volts runs the prop at `omega_max`.
    pub fn winding_r(&self) -> f32 {
        let k_e = self.k_e();
        k_e * (self.v_ref - k_e * self.omega_max) / (self.k_q() * self.omega_max * self.omega_max + self.motor_friction)
    }

    /// The speed a sound motor settles at with `volts` applied on average
    /// (duty × pack voltage): where `k_e·(v − k_e·ω)/R = k_q·ω² + friction`.
    pub fn steady_speed(&self, volts: f32) -> f32 {
        let (k_e, r) = (self.k_e(), self.winding_r());
        let c = r * self.k_q() / k_e;
        let v = volts - r * self.motor_friction / k_e;
        if v <= 0.0 {
            return 0.0;
        }
        (sqrtf(k_e * k_e + 4.0 * c * v) - k_e) / (2.0 * c)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MotorHealth {
    Ok,
    /// Driven at this fraction of the voltage the ESC applies (a weak or
    /// shorted winding): it runs at about this fraction of its normal speed.
    Degraded(f32),
    /// ESC lost sync: the motor stalls and produces nothing.
    Desync,
    /// Burnt or disconnected: never spins again.
    Dead,
}

/// What the ESC is doing with its motor, as in Bluejay.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Esc {
    /// Zero command: the motor coasts to a stop.
    Stopped,
    /// Starting open loop since `since_us`; `tries` stalls so far.
    Starting { since_us: u64, tries: u8 },
    /// Synchronised and following the command. `slow_since_us` is when the
    /// speed fell below the ESC's minimum.
    Running { since_us: u64, slow_since_us: Option<u64>, tries: u8 },
    /// Lost sync at `since_us`: no drive for `ESC_RESTART_WAIT_US`, then it
    /// starts again.
    Stalled { since_us: u64, tries: u8 },
    /// Stalled `ESC_MAX_STALLS` times in a row; it waits for a zero command
    /// before it tries again.
    GaveUp,
}

#[derive(Clone, Debug)]
pub struct Motor {
    /// Last DShot value received by the ESC.
    pub command: u16,
    pub last_frame_us: u64,
    /// Prop speed (rad/s).
    pub omega: f32,
    /// Shaft angle (rad), drives prop-imbalance vibration.
    pub phase: f32,
    pub health: MotorHealth,
    pub esc: Esc,
    /// How often the ESC has lost sync on its own (not by a fault).
    pub stalls: u32,
}

#[derive(Clone, Debug)]
pub struct Battery {
    pub capacity_ah: f32,
    pub soc: f32,
    pub r_int: f32,
    /// Terminal voltage at the last update.
    pub volts: f32,
    pub amps: f32,
    /// Electronics draw (FC, receiver, VTX) in amps.
    pub base_amps: f32,
    /// A LiHV pack (4.35 V full), as the quad ships with, instead of LiPo.
    pub lihv: bool,
    /// No pack plugged in (USB power only): motors cannot spin.
    pub absent: bool,
}

impl Battery {
    /// Open-circuit voltage of this pack at `soc`: a LiHV cell sits up to
    /// 0.15 V above a LiPo one when full, and about the same when empty.
    pub fn open_circuit(&self, soc: f32) -> f32 {
        Battery::ocv(soc) + if self.lihv { 0.15 * soc.clamp(0.0, 1.0) } else { 0.0 }
    }

    /// Open-circuit voltage of a 1S LiPo against state of charge.
    pub fn ocv(soc: f32) -> f32 {
        const PTS: [(f32, f32); 9] = [
            (0.0, 3.0),
            (0.05, 3.3),
            (0.1, 3.55),
            (0.2, 3.68),
            (0.4, 3.78),
            (0.6, 3.88),
            (0.8, 4.0),
            (0.95, 4.13),
            (1.0, 4.2),
        ];
        let s = soc.clamp(0.0, 1.0);
        for w in PTS.windows(2) {
            let ((s0, v0), (s1, v1)) = (w[0], w[1]);
            if s <= s1 {
                return v0 + (v1 - v0) * (s - s0) / (s1 - s0);
            }
        }
        4.2
    }
}

#[derive(Clone, Debug)]
pub struct Wind {
    pub mean: Vec3,
    pub gust_sigma: f32,
    pub gust_tau: f32,
    pub gust: Vec3,
}

impl Wind {
    pub fn calm() -> Self {
        Wind { mean: Vec3::ZERO, gust_sigma: 0.0, gust_tau: 1.0, gust: Vec3::ZERO }
    }
    pub fn velocity(&self) -> Vec3 {
        self.mean + self.gust
    }
    /// Ornstein–Uhlenbeck gusts.
    pub fn advance(&mut self, dt: f32, rng: &mut Rng) {
        if self.gust_sigma <= 0.0 {
            return;
        }
        let a = expf(-dt / self.gust_tau);
        let k = self.gust_sigma * sqrtf(1.0 - a * a);
        self.gust = Vec3::new(
            a * self.gust.x + k * rng.gaussian(),
            a * self.gust.y + k * rng.gaussian(),
            a * self.gust.z + 0.3 * k * rng.gaussian(),
        );
    }
}

#[derive(Clone, Debug)]
pub struct Body {
    pub pos: Vec3,
    pub vel: Vec3,
    pub att: Quat,
    /// Body-frame angular rate (rad/s).
    pub omega: Vec3,
    /// World-frame acceleration from the last step, including contact.
    pub accel: Vec3,
    /// Body-frame angular acceleration from the last step.
    pub alpha: Vec3,
    pub on_ground: bool,
    /// Contact force per unit mass in the last step (m/s²).
    pub impact: f32,
}

#[derive(Clone, Debug)]
pub struct World {
    pub frame: Airframe,
    pub body: Body,
    pub motors: [Motor; 4],
    pub battery: Battery,
    pub wind: Wind,
    /// Extra body torque from a prop strike or collision, while it lasts.
    pub kick: Option<Vec3>,
}

/// ESCs stop the motors when DShot frames stop arriving.
pub const ESC_SIGNAL_TIMEOUT_US: u64 = 250_000;
/// Too slow for this long loses sync.
pub const ESC_STALL_US: u64 = 30_000;
/// Bluejay waits this long after a stall before it starts again...
pub const ESC_RESTART_WAIT_US: u64 = 100_000;
/// ...and gives up after this many stalls in a row.
pub const ESC_MAX_STALLS: u8 = 3;
/// Running this long without a stall clears the count.
pub const ESC_STALLS_FORGOTTEN_US: u64 = 1_000_000;

/// Contact points (body frame): duct bottoms, duct tops, canopy.
fn contact_points(f: &Airframe) -> [Vec3; 9] {
    let mut pts = [Vec3::ZERO; 9];
    for i in 0..4 {
        pts[i] = f.motor_pos[i] * 1.35 + Vec3::new(0.0, 0.0, 0.010);
        pts[i + 4] = f.motor_pos[i] * 1.35 + Vec3::new(0.0, 0.0, -0.014);
    }
    pts[8] = Vec3::new(0.0, 0.0, -0.024);
    pts
}

impl World {
    pub fn new(frame: Airframe, battery: Battery, wind: Wind) -> Self {
        // Resting on the duct bottoms.
        let pos = Vec3::new(0.0, 0.0, -0.010 + frame.mass * GRAVITY / 4.0 / CONTACT_K);
        World {
            frame,
            body: Body {
                pos,
                vel: Vec3::ZERO,
                att: Quat::IDENTITY,
                omega: Vec3::ZERO,
                accel: Vec3::ZERO,
                alpha: Vec3::ZERO,
                on_ground: true,
                impact: 0.0,
            },
            motors: core::array::from_fn(|_| Motor {
                command: 0,
                last_frame_us: 0,
                omega: 0.0,
                phase: 0.0,
                health: MotorHealth::Ok,
                esc: Esc::Stopped,
                stalls: 0,
            }),
            battery,
            wind,
            kick: None,
        }
    }

    pub fn thrust(&self, i: usize) -> f32 {
        self.frame.k_t * self.motors[i].omega * self.motors[i].omega
    }

    /// eRPM as a bidirectional-DShot ESC reports it: none until it has
    /// synchronised.
    pub fn erpm(&self, i: usize) -> u32 {
        if !matches!(self.motors[i].esc, Esc::Running { .. }) {
            return 0;
        }
        let rpm = self.motors[i].omega * 60.0 / (2.0 * core::f32::consts::PI);
        (rpm * self.frame.pole_pairs) as u32
    }

    /// Advances the world by `dt` seconds ending at `now_us`.
    pub fn step(&mut self, dt: f32, now_us: u64, rng: &mut Rng) {
        let f = &self.frame;
        let b = &mut self.battery;

        // Motors: the ESC applies duty × pack voltage. Bluejay drives
        // complementary PWM ("damped light"), so a motor turning faster than
        // that voltage drives it brakes through negative current instead of
        // coasting on its prop's drag. Only a stopped, starting-over or
        // given-up ESC lets it coast. A weakened motor gets only part of the
        // duty.
        let (k_e, r, k_q) = (f.k_e(), f.winding_r(), f.k_q());
        let mut duty = [None; 4];
        for (m, duty) in self.motors.iter_mut().zip(&mut duty) {
            let signal = now_us.saturating_sub(m.last_frame_us) < ESC_SIGNAL_TIMEOUT_US;
            let cmd = if !signal || m.command < DSHOT_MIN || b.absent {
                0.0
            } else {
                (m.command.min(DSHOT_MAX) - DSHOT_MIN) as f32 / (DSHOT_MAX - DSHOT_MIN) as f32
            };
            let health = match m.health {
                MotorHealth::Ok => 1.0,
                MotorHealth::Degraded(k) => k,
                MotorHealth::Desync | MotorHealth::Dead => 0.0,
            };
            let drive = esc(m, f, cmd, now_us, health > 0.0);
            if matches!(m.esc, Esc::Starting { .. } | Esc::Running { .. }) {
                *duty = Some(drive * health);
            }
        }
        // The pack draws the winding current of each driven motor for its
        // PWM's on-time (a fraction `d` of it), and its terminal voltage
        // drops by that current times its internal resistance. Both depend
        // on each other, so solve them together: taking the voltage from
        // the step before, a hard punch on a worn pack overshoots and
        // oscillates (the loop gain 4·d²·r_int/R passes 1).
        let (mut d2, mut back) = (0.0, 0.0);
        for (m, d) in self.motors.iter().zip(duty) {
            if let Some(d) = d {
                d2 += d * d / r;
                back += d * k_e * m.omega / r;
            }
        }
        b.volts = if b.absent { 0.0 } else { ((b.open_circuit(b.soc) - b.r_int * (b.base_amps - back)) / (1.0 + b.r_int * d2)).max(0.0) };
        let mut amps = b.base_amps;
        for (m, d) in self.motors.iter_mut().zip(duty) {
            let winding = d.map_or(0.0, |d| (d * b.volts - k_e * m.omega) / r);
            let d = d.unwrap_or(0.0);
            let alpha = (k_e * winding - k_q * m.omega * m.omega - f.motor_friction) / f.rotor_inertia;
            m.omega = (m.omega + alpha * dt).max(0.0);
            m.phase = wrap_phase(m.phase + m.omega * dt);
            amps += d * winding;
        }
        b.amps = amps;
        if !b.absent {
            // Braking motors can push a little charge back.
            b.soc = (b.soc - amps * dt / 3600.0 / b.capacity_ah).clamp(0.0, 1.0);
        }

        self.wind.advance(dt, rng);
        let body = &mut self.body;
        let q = body.att;

        // Forces in the body frame.
        let mut force_b = Vec3::ZERO;
        let mut torque = Vec3::ZERO;
        let mut omega_sum = 0.0;
        for i in 0..4 {
            let w = self.motors[i].omega;
            let t = f.k_t * w * w;
            omega_sum += w;
            let fz = Vec3::new(0.0, 0.0, -t);
            force_b += fz;
            torque += f.motor_pos[i].cross(fz);
            torque += Vec3::new(0.0, 0.0, -f.spin[i] * f.k_q_ratio * t);
        }
        let air_b = q.inverse_rotate(body.vel - self.wind.velocity());
        let speed = air_b.norm();
        force_b += -(f.drag_lin.mul_elem(air_b) + f.drag_quad.mul_elem(air_b) * speed);
        force_b += -Vec3::new(air_b.x, air_b.y, 0.0) * (f.rotor_drag * omega_sum);
        torque += -body.omega * f.rot_damping;
        if let Some(k) = self.kick {
            torque += k;
        }

        let mut force_w = q.rotate(force_b) + Vec3::new(0.0, 0.0, f.mass * GRAVITY);

        // Ground: spring-damper contacts with Coulomb friction.
        let mut contact_w = Vec3::ZERO;
        body.on_ground = false;
        for p in contact_points(f) {
            let r_w = q.rotate(p);
            let world = body.pos + r_w;
            if world.z <= 0.0 {
                continue;
            }
            body.on_ground = true;
            let v = body.vel + q.rotate(body.omega.cross(p));
            let n = (CONTACT_K * world.z + CONTACT_C * v.z).max(0.0);
            let vt = Vec3::new(v.x, v.y, 0.0);
            let vt_n = vt.norm();
            let fric = if vt_n > 1e-4 { vt * (-0.6 * n / vt_n.max(0.05)) } else { Vec3::ZERO };
            let fc = Vec3::new(fric.x, fric.y, -n);
            contact_w += fc;
            torque += p.cross(q.inverse_rotate(fc));
        }
        if body.on_ground {
            // Rolling/scrubbing losses so the quad settles instead of rocking.
            torque += -body.omega * 4.0e-5;
        }
        force_w += contact_w;
        body.impact = contact_w.norm() / f.mass;

        let accel = force_w * (1.0 / f.mass);
        let i = f.inertia;
        let iw = i.mul_elem(body.omega);
        let gyro_t = body.omega.cross(iw);
        let alpha = Vec3::new(
            (torque.x - gyro_t.x) / i.x,
            (torque.y - gyro_t.y) / i.y,
            (torque.z - gyro_t.z) / i.z,
        );
        body.vel += accel * dt;
        body.pos += body.vel * dt;
        body.omega += alpha * dt;
        body.att = body.att.integrate(body.omega, dt);
        body.accel = accel;
        body.alpha = alpha;
    }

    /// Specific force in the body frame (what an ideal accelerometer reads).
    pub fn specific_force(&self) -> Vec3 {
        let b = &self.body;
        b.att.inverse_rotate(b.accel - Vec3::new(0.0, 0.0, GRAVITY))
    }

    pub fn altitude(&self) -> f32 {
        -self.body.pos.z
    }
}

/// `phase % TAU`, without the slow `fmodf` for the usual phase less than
/// two turns: below TAU the remainder is the phase itself, and up to 2 TAU
/// subtracting TAU is exact (Sterbenz), as `fmodf` is.
fn wrap_phase(phase: f32) -> f32 {
    const TAU: f32 = 2.0 * core::f32::consts::PI;
    if (0.0..TAU).contains(&phase) {
        phase
    } else if (TAU..2.0 * TAU).contains(&phase) {
        phase - TAU
    } else {
        phase % TAU
    }
}

const CONTACT_K: f32 = 400.0;
const CONTACT_C: f32 = 1.2;

/// Steps the ESC's state and returns the command it drives the motor at.
/// `sound` is false while a fault stops the motor, which is judged
/// separately.
fn esc(m: &mut Motor, f: &Airframe, cmd: f32, now: u64, sound: bool) -> f32 {
    // A fault that stops the motor leaves the ESC to start it again once it
    // clears, not to find it already stopped and count a stall of its own
    // (seed 42067).
    if cmd <= 0.0 || !sound {
        m.esc = Esc::Stopped;
        return 0.0;
    }
    let speed = m.omega / f.omega_max;
    m.esc = match m.esc {
        Esc::Stopped => Esc::Starting { since_us: now, tries: 0 },
        Esc::Starting { since_us, tries } if now - since_us >= f.esc_start_us => Esc::Running { since_us: now, slow_since_us: None, tries },
        Esc::Running { since_us, slow_since_us, tries } => {
            let tries = if now - since_us >= ESC_STALLS_FORGOTTEN_US { 0 } else { tries };
            let slow = (sound && speed < f.esc_min_speed).then(|| slow_since_us.unwrap_or(now));
            if slow.is_some_and(|s| now - s >= ESC_STALL_US) {
                m.stalls += 1;
                Esc::Stalled { since_us: now, tries: tries + 1 }
            } else {
                Esc::Running { since_us, slow_since_us: slow, tries }
            }
        }
        Esc::Stalled { since_us, tries } if now - since_us >= ESC_RESTART_WAIT_US => {
            if tries >= ESC_MAX_STALLS { Esc::GaveUp } else { Esc::Starting { since_us: now, tries } }
        }
        state => state,
    };
    match m.esc {
        Esc::Starting { .. } => f.esc_start_power,
        Esc::Running { .. } => cmd.min(f.esc_start_power + f.esc_rampup * speed),
        _ => 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT_US: u64 = 250;

    /// A default airframe on a pack that holds 3.9 V (the reference)
    /// however much it draws, with ESCs that start at once.
    fn bench() -> World {
        let frame = Airframe { esc_start_us: 0, ..Airframe::default() };
        // 3.9 V open circuit, between the 0.6 and 0.8 points of the curve.
        let soc = 0.6 + 0.2 * (3.9 - 3.88) / (4.0 - 3.88);
        let battery = Battery { capacity_ah: 1e6, soc, r_int: 0.0, volts: 3.9, amps: 0.0, base_amps: 0.0, lihv: false, absent: false };
        World::new(frame, battery, Wind::calm())
    }

    /// Runs every motor at `command` for `secs`, from `t_us`; returns the
    /// time reached and the first motor's speed after each step.
    fn run(w: &mut World, command: u16, t_us: u64, secs: f32) -> (u64, Vec<f32>) {
        let mut rng = Rng::new(1);
        let mut t = t_us;
        let mut speeds = Vec::new();
        for _ in 0..(secs * 1e6) as u64 / DT_US {
            t += DT_US;
            for m in &mut w.motors {
                m.command = command;
                m.last_frame_us = t;
            }
            w.step(DT_US as f32 * 1e-6, t, &mut rng);
            speeds.push(w.motors[0].omega);
        }
        (t, speeds)
    }

    fn command(duty: f32) -> u16 {
        DSHOT_MIN + libm::roundf(duty * (DSHOT_MAX - DSHOT_MIN) as f32) as u16
    }

    #[test]
    fn full_command_reaches_the_rated_speed_and_current() {
        let mut w = bench();
        run(&mut w, DSHOT_MAX, 0, 1.0);
        let omega = w.motors[0].omega;
        assert!((omega - 7576.0).abs() < 0.01 * 7576.0, "{omega}");
        // Per motor (k_q·ω² + friction) / k_e = (1.808e-3 + 5e-5) / 3.41e-4.
        let amps = w.battery.amps / 4.0;
        assert!((amps - 5.45).abs() < 0.1, "{amps}");
    }

    #[test]
    fn hover_needs_less_duty_than_its_share_of_full_speed() {
        // Hover is 0.0314 kg × g / 4 = 0.077 N a motor, 4281 rad/s: 57% of
        // full speed, but 48.8% duty, since the back-EMF takes most of the
        // voltage and the drag torque grows with the speed squared.
        let mut w = bench();
        run(&mut w, command(0.488), 0, 1.0);
        let thrust = w.thrust(0);
        let hover = 0.0314 * GRAVITY / 4.0;
        assert!((thrust - hover).abs() < 0.02 * hover, "{thrust} N against {hover} N");
    }

    #[test]
    fn a_small_step_at_hover_settles_with_the_rotor_time_constant() {
        // J / (k_e²/R + 2·k_q·ω) = 3e-8 / 7.5e-7 at hover: 40 ms, both up
        // and down (the ESC brakes as hard as it drives).
        for (from, to) in [(0.488, 0.508), (0.508, 0.488)] {
            let mut w = bench();
            let (t, _) = run(&mut w, command(from), 0, 1.0);
            let start = w.motors[0].omega;
            let (_, speeds) = run(&mut w, command(to), t, 0.5);
            let end = *speeds.last().unwrap();
            let reached = speeds.iter().position(|&s| (s - start) / (end - start) >= 1.0 - (-1.0f32).exp()).unwrap();
            let tau_ms = (reached + 1) as f32 * DT_US as f32 * 1e-3;
            assert!((tau_ms - 40.0).abs() < 4.0, "{from} -> {to}: {tau_ms} ms");
        }
    }

    #[test]
    fn steady_speed_matches_the_simulated_motor() {
        let mut w = bench();
        run(&mut w, command(0.3), 0, 1.0);
        let want = w.frame.steady_speed(command_duty(command(0.3)) * 3.9);
        let omega = w.motors[0].omega;
        assert!((omega - want).abs() < 0.005 * want, "{omega} against {want}");
    }

    fn command_duty(c: u16) -> f32 {
        (c - DSHOT_MIN) as f32 / (DSHOT_MAX - DSHOT_MIN) as f32
    }
}
