//! Minimal `no_std` vector and quaternion math.
//!
//! Frames: the body frame is FRD (x forward, y right, z down) and the world
//! frame is NED (x north, y east, z down). A [`Quat`] rotates body vectors into
//! the world frame.

use core::ops::{Add, AddAssign, Mul, Neg, Sub};
use libm::{acosf, asinf, atan2f, cosf, sinf, sqrtf};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3::new(0.0, 0.0, 0.0);

    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
    pub fn from_array(a: [f32; 3]) -> Self {
        Self::new(a[0], a[1], a[2])
    }
    pub fn to_array(self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }
    pub fn dot(self, o: Vec3) -> f32 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    pub fn cross(self, o: Vec3) -> Vec3 {
        Vec3::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }
    pub fn norm(self) -> f32 {
        sqrtf(self.dot(self))
    }
    pub fn normalized(self) -> Vec3 {
        let n = self.norm();
        if n > 1e-9 { self * (1.0 / n) } else { Vec3::ZERO }
    }
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
    pub fn max_abs(self) -> f32 {
        self.x.abs().max(self.y.abs()).max(self.z.abs())
    }
    pub fn mul_elem(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x * o.x, self.y * o.y, self.z * o.z)
    }
}

impl Add for Vec3 {
    type Output = Vec3;
    fn add(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}
impl AddAssign for Vec3 {
    fn add_assign(&mut self, o: Vec3) {
        *self = *self + o;
    }
}
impl Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl Mul<f32> for Vec3 {
    type Output = Vec3;
    fn mul(self, k: f32) -> Vec3 {
        Vec3::new(self.x * k, self.y * k, self.z * k)
    }
}
impl Neg for Vec3 {
    type Output = Vec3;
    fn neg(self) -> Vec3 {
        Vec3::new(-self.x, -self.y, -self.z)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Default for Quat {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Quat {
    pub const IDENTITY: Quat = Quat { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };

    pub fn from_axis_angle(axis: Vec3, angle: f32) -> Quat {
        let a = axis.normalized();
        let (s, c) = (sinf(angle * 0.5), cosf(angle * 0.5));
        Quat { w: c, x: a.x * s, y: a.y * s, z: a.z * s }
    }

    /// Rotation by the rotation vector `v` (axis times angle in radians).
    pub fn from_rotation_vector(v: Vec3) -> Quat {
        let angle = v.norm();
        if angle < 1e-9 {
            Quat { w: 1.0, x: v.x * 0.5, y: v.y * 0.5, z: v.z * 0.5 }.normalized()
        } else {
            Quat::from_axis_angle(v, angle)
        }
    }

    /// Z-Y-X Euler angles (yaw, then pitch, then roll), in radians.
    pub fn from_euler(roll: f32, pitch: f32, yaw: f32) -> Quat {
        let qz = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), yaw);
        let qy = Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), pitch);
        let qx = Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), roll);
        qz.mul(qy).mul(qx)
    }

    pub fn mul(self, o: Quat) -> Quat {
        Quat {
            w: self.w * o.w - self.x * o.x - self.y * o.y - self.z * o.z,
            x: self.w * o.x + self.x * o.w + self.y * o.z - self.z * o.y,
            y: self.w * o.y - self.x * o.z + self.y * o.w + self.z * o.x,
            z: self.w * o.z + self.x * o.y - self.y * o.x + self.z * o.w,
        }
    }

    pub fn conj(self) -> Quat {
        Quat { w: self.w, x: -self.x, y: -self.y, z: -self.z }
    }

    pub fn normalized(self) -> Quat {
        let n = sqrtf(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z);
        if n < 1e-9 || !n.is_finite() {
            return Quat::IDENTITY;
        }
        let k = 1.0 / n;
        Quat { w: self.w * k, x: self.x * k, y: self.y * k, z: self.z * k }
    }

    /// Rotates a body-frame vector into the world frame.
    pub fn rotate(self, v: Vec3) -> Vec3 {
        let u = Vec3::new(self.x, self.y, self.z);
        let t = u.cross(v) * 2.0;
        v + t * self.w + u.cross(t)
    }

    /// Rotates a world-frame vector into the body frame.
    pub fn inverse_rotate(self, v: Vec3) -> Vec3 {
        self.conj().rotate(v)
    }

    /// Integrates a body-frame angular rate (rad/s) over `dt` seconds.
    pub fn integrate(self, omega_body: Vec3, dt: f32) -> Quat {
        self.mul(Quat::from_rotation_vector(omega_body * dt)).normalized()
    }

    /// Returns (roll, pitch, yaw) in radians.
    pub fn to_euler(self) -> (f32, f32, f32) {
        let Quat { w, x, y, z } = self;
        let roll = atan2f(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
        let pitch = asinf((2.0 * (w * y - z * x)).clamp(-1.0, 1.0));
        let yaw = atan2f(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
        (roll, pitch, yaw)
    }

    /// Angle in radians between the body z axis and the world z axis.
    pub fn tilt(self) -> f32 {
        acosf((1.0 - 2.0 * (self.x * self.x + self.y * self.y)).clamp(-1.0, 1.0))
    }

    pub fn is_finite(self) -> bool {
        self.w.is_finite() && self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

/// World "up" direction (NED) expressed in the body frame for the given
/// roll and pitch (yaw does not matter).
pub fn up_in_body(roll: f32, pitch: f32) -> Vec3 {
    Vec3::new(sinf(pitch), -sinf(roll) * cosf(pitch), -cosf(roll) * cosf(pitch))
}

/// Body angular rate that rotates the measured up vector `up` toward
/// `target_up`, scaled by the angle between them (radians).
pub fn tilt_error(up: Vec3, target_up: Vec3) -> Vec3 {
    let axis = target_up.cross(up);
    let s = axis.norm();
    let c = target_up.dot(up);
    let angle = atan2f(s, c);
    if s > 1e-6 {
        axis * (angle / s)
    } else if c < 0.0 {
        // Exactly inverted: any horizontal axis works; roll back over.
        Vec3::new(angle, 0.0, 0.0)
    } else {
        Vec3::ZERO
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn up_vector_matches_quaternion() {
        for (r, p) in [(0.3f32, -0.2f32), (-1.0, 0.5), (2.5, 0.1)] {
            let q = Quat::from_euler(r, p, 0.7);
            let up = q.inverse_rotate(Vec3::new(0.0, 0.0, -1.0));
            let expect = up_in_body(r, p);
            assert!((up - expect).norm() < 1e-5, "{up:?} vs {expect:?}");
        }
    }

    #[test]
    fn tilt_error_rotates_toward_target() {
        // Body rolled right 0.4 rad; target level. Rolling left (negative x)
        // must reduce the error.
        let up = up_in_body(0.4, 0.0);
        let e = tilt_error(up, up_in_body(0.0, 0.0));
        assert!(e.x < -0.39 && e.x > -0.41, "{e:?}");
        assert!(e.y.abs() < 1e-5);
        // Nose up 0.3 rad; target level → negative pitch rate.
        let e = tilt_error(up_in_body(0.0, 0.3), up_in_body(0.0, 0.0));
        assert!((e.y + 0.3).abs() < 1e-4, "{e:?}");
    }
}
