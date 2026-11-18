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
    /// Prop speed at full command on `v_ref` volts (rad/s).
    pub omega_max: f32,
    pub v_ref: f32,
    pub tau_up: f32,
    pub tau_down: f32,
    /// Current per motor at full command (A).
    pub motor_max_amps: f32,
    /// Linear drag per axis (N per m/s), body frame.
    pub drag_lin: Vec3,
    /// Quadratic drag per axis (N per (m/s)²), body frame.
    pub drag_quad: Vec3,
    /// Rotor drag (N per m/s per rad/s of summed prop speed), horizontal.
    pub rotor_drag: f32,
    /// Rotational damping (N·m per rad/s).
    pub rot_damping: f32,
    pub pole_pairs: f32,
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
            tau_up: 0.020,
            tau_down: 0.030,
            motor_max_amps: 4.74,
            drag_lin: Vec3::new(0.004, 0.004, 0.006),
            drag_quad: Vec3::new(0.0035, 0.0035, 0.006),
            rotor_drag: 3.0e-7,
            rot_damping: 2.0e-6,
            pole_pairs: 6.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MotorHealth {
    Ok,
    /// Producing this fraction of its normal speed (bent prop, worn bearing).
    Degraded(f32),
    /// ESC lost sync: the motor stalls and produces nothing.
    Desync,
    /// Burnt or disconnected: never spins again.
    Dead,
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
    /// No pack plugged in (USB power only): motors cannot spin.
    pub absent: bool,
}

impl Battery {
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
            a * self.gust.x + k * rng.normal(),
            a * self.gust.y + k * rng.normal(),
            a * self.gust.z + 0.3 * k * rng.normal(),
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
            }),
            battery,
            wind,
            kick: None,
        }
    }

    pub fn thrust(&self, i: usize) -> f32 {
        self.frame.k_t * self.motors[i].omega * self.motors[i].omega
    }

    /// eRPM as a bidirectional-DShot ESC reports it.
    pub fn erpm(&self, i: usize) -> u32 {
        let rpm = self.motors[i].omega * 60.0 / (2.0 * core::f32::consts::PI);
        (rpm * self.frame.pole_pairs) as u32
    }

    /// Advances the world by `dt` seconds ending at `now_us`.
    pub fn step(&mut self, dt: f32, now_us: u64, rng: &mut Rng) {
        let f = &self.frame;
        let b = &mut self.battery;

        // Motors: first-order speed response toward the commanded speed,
        // which scales with battery voltage (KV × volts).
        let mut amps = b.base_amps;
        for m in &mut self.motors {
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
            let target = cmd * f.omega_max * (b.volts / f.v_ref).max(0.0) * health;
            let tau = if target > m.omega { f.tau_up } else { f.tau_down };
            m.omega += (target - m.omega) * (1.0 - expf(-dt / tau));
            m.phase = (m.phase + m.omega * dt) % (2.0 * core::f32::consts::PI);
            let r = m.omega / f.omega_max;
            amps += f.motor_max_amps * r * r * r;
        }
        b.amps = amps;
        if !b.absent {
            b.soc = (b.soc - amps * dt / 3600.0 / b.capacity_ah).max(0.0);
            b.volts = (Battery::ocv(b.soc) - amps * b.r_int).max(0.0);
        } else {
            b.volts = 0.0;
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

const CONTACT_K: f32 = 400.0;
const CONTACT_C: f32 = 1.2;
