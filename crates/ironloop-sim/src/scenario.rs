//! Seeded workload generation: airframe and sensor variation, environment,
//! pilot behavior, and fault injection.

use ironloop_fc::math::Vec3;
use ironloop_fc::{Config, FlightMode, ImuAlignment};

use crate::physics::{Airframe, Battery, Wind};
use crate::rng::Rng;

#[derive(Clone, Debug)]
pub struct ImuModel {
    pub odr_hz: f32,
    /// Uniform jitter on delivery time (µs).
    pub jitter_us: u64,
    /// How the chip is really mounted (the firmware may be configured wrong).
    pub mount: ImuAlignment,
    pub gyro_noise: f32,
    pub accel_noise: f32,
    pub gyro_bias: Vec3,
    /// Bias drift from warm-up (rad/s per s), applies for the first minute.
    pub gyro_bias_drift: Vec3,
    pub accel_bias: Vec3,
    pub gyro_scale: Vec3,
    /// Prop-imbalance vibration at full speed (rad/s and m/s²).
    pub vib_gyro: f32,
    pub vib_accel: f32,
    pub gyro_range: f32,
    pub accel_range: f32,
    /// Time from the data-ready timestamp to the sample's step: the SPI
    /// read plus waiting for the control task, µs. Events that arrive in
    /// between wait for it, since the same task steps them.
    pub read_delay_us: u64,
    /// The chip's own digital low-pass (Hz, two first-order stages): gyro
    /// and accelerometer data lag the motion by about 1/(pi * f) seconds
    /// before the firmware sees them. 0 disables it.
    pub chip_lpf_hz: f32,
}

impl Default for ImuModel {
    fn default() -> Self {
        Self {
            odr_hz: 3200.0,
            jitter_us: 10,
            mount: ImuAlignment::Cw0,
            gyro_noise: 0.006,
            accel_noise: 0.08,
            gyro_bias: Vec3::ZERO,
            gyro_bias_drift: Vec3::ZERO,
            accel_bias: Vec3::ZERO,
            gyro_scale: Vec3::new(1.0, 1.0, 1.0),
            vib_gyro: 0.15,
            vib_accel: 6.0,
            gyro_range: 2000f32.to_radians(),
            accel_range: 16.0 * 9.80665,
            read_delay_us: 110,
            chip_lpf_hz: 250.0,
        }
    }
}

#[derive(Clone, Debug)]
pub struct RcModel {
    pub rate_hz: f32,
    pub jitter_us: u64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FaultKind {
    /// No IMU samples arrive.
    ImuDropout,
    /// The IMU repeats its last sample bit for bit.
    ImuStuck,
    /// Each sample is garbage (NaN, out of range, or full-scale spike) with
    /// probability `p`.
    ImuGarbage { p: f32 },
    /// Body torque (N·m) from a collision or prop strike.
    Kick { torque: Vec3 },
    /// A hard hit (pole, branch): a big torque for under a millisecond, so
    /// the gyro jumps by tens of rad/s between two samples.
    Impact { torque: Vec3 },
    MotorDesync { motor: u8 },
    MotorDead { motor: u8 },
    MotorDegraded { motor: u8, factor: f32 },
    /// No RC frames reach the flight controller.
    RcLoss,
    /// RC frames are corrupted (out-of-range values) with probability `p`.
    RcGlitch { p: f32 },
    /// Bidirectional DShot telemetry frames are lost.
    TelemetryLoss,
    /// Mean wind changes to this vector (m/s, NED) for the window.
    WindShift { wind: Vec3 },
    /// The firmware's clock jumps back by `back_us` once.
    ClockGlitch { back_us: u64 },
    /// IMU delivery jitter grows to `jitter_us` (scheduler overload).
    TimingJitter { jitter_us: u64 },
    /// The board's event queue drops each event with probability `p`.
    EventDrop { p: f32 },
    /// The firmware task stalls: events pile up in the board's 32-deep
    /// queue (the rest are dropped) and arrive in one burst afterwards.
    FcStall,
    /// A reboot-to-bootloader command arrives (configurator over MSP/USB),
    /// possibly mid-flight.
    RebootRequest,
}

/// What a fault excuses while it is active (and for a recovery window
/// after).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Excuse {
    /// Nothing: the firmware must fly through it.
    Nothing,
    /// Normal flight, but not the failsafe, which exists to cope with it.
    LinkOrWeather,
    /// Flight expectations, the failsafe's included: the airframe or its
    /// sensors were hit.
    Flight,
}

impl FaultKind {
    /// One of every kind.
    pub const ALL: [FaultKind; 17] = [
        FaultKind::ImuDropout,
        FaultKind::ImuStuck,
        FaultKind::ImuGarbage { p: 0.0 },
        FaultKind::Kick { torque: Vec3::ZERO },
        FaultKind::Impact { torque: Vec3::ZERO },
        FaultKind::MotorDesync { motor: 0 },
        FaultKind::MotorDead { motor: 0 },
        FaultKind::MotorDegraded { motor: 0, factor: 1.0 },
        FaultKind::RcLoss,
        FaultKind::RcGlitch { p: 0.0 },
        FaultKind::TelemetryLoss,
        FaultKind::WindShift { wind: Vec3::ZERO },
        FaultKind::ClockGlitch { back_us: 0 },
        FaultKind::TimingJitter { jitter_us: 0 },
        FaultKind::EventDrop { p: 0.0 },
        FaultKind::FcStall,
        FaultKind::RebootRequest,
    ];

    pub fn name(&self) -> &'static str {
        match self {
            FaultKind::ImuDropout => "ImuDropout",
            FaultKind::ImuStuck => "ImuStuck",
            FaultKind::ImuGarbage { .. } => "ImuGarbage",
            FaultKind::Kick { .. } => "Kick",
            FaultKind::Impact { .. } => "Impact",
            FaultKind::MotorDesync { .. } => "MotorDesync",
            FaultKind::MotorDead { .. } => "MotorDead",
            FaultKind::MotorDegraded { .. } => "MotorDegraded",
            FaultKind::RcLoss => "RcLoss",
            FaultKind::RcGlitch { .. } => "RcGlitch",
            FaultKind::TelemetryLoss => "TelemetryLoss",
            FaultKind::WindShift { .. } => "WindShift",
            FaultKind::ClockGlitch { .. } => "ClockGlitch",
            FaultKind::TimingJitter { .. } => "TimingJitter",
            FaultKind::EventDrop { .. } => "EventDrop",
            FaultKind::FcStall => "FcStall",
            FaultKind::RebootRequest => "RebootRequest",
        }
    }

    pub fn excuse(&self) -> Excuse {
        match self {
            FaultKind::Kick { .. }
            | FaultKind::Impact { .. }
            | FaultKind::MotorDesync { .. }
            | FaultKind::MotorDegraded { .. }
            | FaultKind::ImuDropout
            | FaultKind::ImuStuck
            | FaultKind::ImuGarbage { .. } => Excuse::Flight,
            FaultKind::RcLoss | FaultKind::WindShift { .. } => Excuse::LinkOrWeather,
            // A dead motor makes the whole build unflyable instead
            // (`Scenario::unflyable`).
            FaultKind::MotorDead { .. }
            | FaultKind::RcGlitch { .. }
            | FaultKind::TelemetryLoss
            | FaultKind::ClockGlitch { .. }
            | FaultKind::TimingJitter { .. }
            | FaultKind::EventDrop { .. }
            | FaultKind::FcStall
            | FaultKind::RebootRequest => Excuse::Nothing,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Fault {
    pub at_us: u64,
    pub dur_us: u64,
    pub kind: FaultKind,
}

impl Fault {
    pub fn active(&self, t: u64) -> bool {
        t >= self.at_us && t < self.at_us + self.dur_us
    }
}

/// What the virtual pilot is trying to do.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Action {
    /// Arm switch off, throttle at `throttle`.
    Disarmed { throttle: f32 },
    /// Flip the arm switch on with the throttle where it is.
    Arm { throttle: f32 },
    /// Hold position at `alt` meters.
    Hover { alt: f32 },
    /// Fly with a constant tilt (rad) toward roll/pitch, holding altitude.
    Cruise { alt: f32, roll: f32, pitch: f32 },
    /// Full-stick 360° on roll (axis 0) or pitch (axis 1) in acro.
    Flip { axis: u8, dir: f32 },
    /// Acro: roll upside down, hang there with sticks centered (12%
    /// throttle, yaw stick at `yaw`) for `secs`, then roll back upright
    /// and recover.
    InvertedHang { secs: f32, yaw: f32 },
    /// Open loop: hands off, then a doublet (`amp` of full stick one way,
    /// then the other, 0.3 s each) on roll, pitch, and yaw in turn, then
    /// hands off again. Throttle holds altitude. In acro it checks rate
    /// tracking and that the quad settles without oscillating; in angle
    /// mode, that each stick tilts the quad as far as the angle limit says.
    StickScript { amp: f32 },
    /// Full throttle in angle mode.
    Punch,
    /// Random full-deflection sticks, changing every 150 ms.
    StickSlam,
    /// Descend and touch down, keep armed (pilot disarms in the next action).
    Land,
    /// Cut throttle and drop from wherever it is.
    Drop,
}

impl Action {
    /// The pilot means to be flying: one who finds the quad disarmed arms
    /// it first.
    pub fn is_flight(&self) -> bool {
        matches!(self, Action::Hover { .. } | Action::Cruise { .. } | Action::Flip { .. } | Action::Punch | Action::StickSlam | Action::StickScript { .. })
    }

    /// Aggressive pilot input; the generator follows each with a recovery
    /// hover.
    pub fn is_maneuver(&self) -> bool {
        self.snaps_sticks() || *self == Action::Punch
    }

    /// The maneuvers whose sticks move instantly instead of at hand speed.
    /// A punch moves only the throttle, which is instant anyway; its
    /// centered roll and pitch come back at hand speed.
    pub fn snaps_sticks(&self) -> bool {
        matches!(self, Action::StickSlam | Action::Flip { .. } | Action::InvertedHang { .. } | Action::StickScript { .. })
    }

    /// May end in a tumble or a hard touchdown, so the flight expectations
    /// are off during it and for 3 s after. A stick script is judged by its
    /// own checks, and a punch in angle mode must stay level.
    pub fn is_wild(&self) -> bool {
        matches!(self, Action::Flip { .. } | Action::StickSlam | Action::Drop | Action::InvertedHang { .. })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Segment {
    pub start_us: u64,
    pub mode: FlightMode,
    pub action: Action,
}

#[derive(Clone, Debug)]
pub struct Scenario {
    pub seed: u64,
    pub name: String,
    pub duration_us: u64,
    pub frame: Airframe,
    pub battery: Battery,
    pub wind: Wind,
    pub imu: ImuModel,
    pub rc: RcModel,
    pub fc: Config,
    pub segments: Vec<Segment>,
    pub faults: Vec<Fault>,
    /// The pilot holds the arm switch on from power-up.
    pub arm_switch_at_boot: bool,
    /// The quad is being carried or moved while booting.
    pub moved_at_boot: bool,
    /// Initial attitude (roll, pitch) on the ground in rad, e.g. on its side.
    pub ground_tilt: (f32, f32),
    /// Heading the quad starts at (rad); nothing may depend on it.
    pub initial_yaw: f32,
    pub telemetry_loss: f32,
    /// Motor wires swapped (M1↔M2), a classic build mistake.
    pub motors_swapped: bool,
    /// ESCs send bidirectional-DShot eRPM. Some builds run without it
    /// (ESC firmware without bidirectional DShot, or a board build that has
    /// it off), so the firmware must work either way.
    pub esc_telemetry: bool,
    /// Added to every timestamp the firmware sees, e.g. to wrap a u32 µs
    /// clock mid-flight.
    pub clock_offset_us: u64,
}

/// Length of a [`Action::StickScript`] segment.
pub const STICK_SCRIPT_US: u64 = 3_800_000;

/// The stick a [`Action::StickScript`] holds at `dt_us` into the segment:
/// (axis 0..3 or none, deflection as a fraction of `amp`).
pub fn stick_script(dt_us: u64) -> Option<(usize, f32)> {
    let ms = dt_us / 1000;
    let doublet = |start: u64| if ms < start + 300 { 1.0 } else { -1.0 };
    match ms {
        400..1000 => Some((0, doublet(400))),
        1400..2000 => Some((1, doublet(1400))),
        2400..3000 => Some((2, doublet(2400))),
        _ => None,
    }
}

impl Scenario {
    pub fn segment_at(&self, t: u64) -> Option<&Segment> {
        self.segments.iter().rev().find(|s| s.start_us <= t)
    }

    /// The quad cannot be expected to fly normally (wrong build or permanent
    /// hardware fault).
    pub fn unflyable(&self) -> bool {
        self.motors_swapped
            || self.imu.mount != self.fc.imu_alignment
            || self.battery.absent
            || self.faults.iter().any(|f| matches!(f.kind, FaultKind::MotorDead { .. }))
    }

    /// A calm, nominal flight: arm, take off, hover, cruise, land.
    pub fn nominal(seed: u64, mode: FlightMode) -> Self {
        let s = 1_000_000;
        let mut sc = Scenario::base(seed, format!("nominal-{mode:?}"));
        sc.segments = vec![
            Segment { start_us: 0, mode, action: Action::Disarmed { throttle: 0.0 } },
            Segment { start_us: 2 * s, mode, action: Action::Arm { throttle: 0.0 } },
            Segment { start_us: 3 * s, mode, action: Action::Hover { alt: 1.5 } },
            Segment { start_us: 8 * s, mode, action: Action::Cruise { alt: 1.5, roll: 0.0, pitch: -0.3 } },
            Segment { start_us: 11 * s, mode, action: Action::Hover { alt: 1.5 } },
            Segment { start_us: 14 * s, mode, action: Action::Land },
            Segment { start_us: 18 * s, mode, action: Action::Disarmed { throttle: 0.0 } },
        ];
        sc.duration_us = 20 * s;
        sc
    }

    /// Open-ended scenario flown by a human: no pilot plan, no faults
    /// except those injected live, mild sensor imperfections.
    pub fn manual(seed: u64) -> Self {
        let mut sc = Scenario::base(seed, "manual".into());
        sc.duration_us = u64::MAX / 4;
        sc.battery.soc = 1.0;
        sc.battery.volts = Battery::ocv(1.0);
        sc.imu.gyro_bias = Vec3::new(0.01, -0.02, 0.005);
        sc.segments = vec![Segment { start_us: 0, mode: FlightMode::Angle, action: Action::Disarmed { throttle: 0.0 } }];
        sc
    }

    fn base(seed: u64, name: String) -> Self {
        Scenario {
            seed,
            name,
            duration_us: 20_000_000,
            frame: Airframe::default(),
            battery: Battery {
                capacity_ah: 0.3,
                soc: 0.95,
                r_int: 0.08,
                volts: Battery::ocv(0.95),
                amps: 0.0,
                base_amps: 0.45,
                absent: false,
            },
            wind: Wind::calm(),
            imu: ImuModel::default(),
            rc: RcModel { rate_hz: 250.0, jitter_us: 300 },
            fc: Config::default(),
            segments: Vec::new(),
            faults: Vec::new(),
            arm_switch_at_boot: false,
            moved_at_boot: false,
            ground_tilt: (0.0, 0.0),
            initial_yaw: 0.0,
            telemetry_loss: 0.01,
            motors_swapped: false,
            esc_telemetry: true,
            clock_offset_us: 0,
        }
    }

    /// Generates a random but reproducible scenario from `seed`.
    pub fn generate(seed: u64) -> Self {
        let mut r = Rng::fork(seed, "scenario");
        let mut sc = Scenario::base(seed, String::new());
        let mut tags: Vec<String> = Vec::new();

        // Airframe variation: build tolerances, battery choice, worn parts.
        let f = &mut sc.frame;
        f.mass *= r.range(0.92, 1.12);
        f.inertia = f.inertia * r.range(0.85, 1.2);
        f.k_t *= r.range(0.9, 1.08);
        f.tau_up *= r.range(0.8, 1.3);
        f.tau_down *= r.range(0.8, 1.3);
        for k in [&mut f.drag_lin, &mut f.drag_quad] {
            *k = *k * r.range(0.7, 1.4);
        }

        // Battery: charge, internal resistance (aged packs sag hard).
        sc.battery.soc = r.range(0.12, 1.0);
        // A fresh 300 mAh 1S pack is about 0.03-0.08 ohm, a worn one 0.15.
        sc.battery.r_int = r.range(0.03, 0.15);
        sc.battery.volts = Battery::ocv(sc.battery.soc);
        if sc.battery.soc < 0.3 {
            tags.push("low-battery".into());
        }
        if sc.battery.r_int > 0.11 {
            tags.push("old-pack".into());
        }

        // IMU: noise, bias, vibration, rate.
        let imu = &mut sc.imu;
        imu.odr_hz = *r.pick(&[1600.0, 3200.0, 3200.0, 4000.0]);
        imu.jitter_us = r.range_u64(2, 40);
        imu.read_delay_us = r.range_u64(80, 160);
        // The chip's own filters as the board sets them up: the BMI270's
        // normal mode at 3.2 kHz is about 751 Hz, the ICM-42688's UI filter
        // at 2 kHz about 500 Hz plus its anti-alias filter. That delays the
        // gyro by about 0.4-1 ms and leaves most of the prop vibration in.
        imu.chip_lpf_hz = r.range(300.0, 750.0);
        imu.gyro_noise *= r.range(0.5, 2.5);
        imu.accel_noise *= r.range(0.5, 2.5);
        let bias = r.range(0.0, 8.0f32.to_radians());
        imu.gyro_bias = Vec3::new(r.normal(), r.normal(), r.normal()).normalized() * bias;
        // Warm-up drift: up to about 1-2 deg/s over the first minute.
        imu.gyro_bias_drift = Vec3::new(r.normal(), r.normal(), r.normal()) * 0.0004;
        imu.accel_bias = Vec3::new(r.normal(), r.normal(), r.normal()) * 0.15;
        imu.gyro_scale = Vec3::new(1.0 + 0.02 * r.normal(), 1.0 + 0.02 * r.normal(), 1.0 + 0.02 * r.normal());
        let vib = r.range(0.5, 2.0);
        if r.chance(0.15) {
            tags.push("bent-prop".into());
            imu.vib_gyro *= 6.0 * vib;
            imu.vib_accel *= 4.0 * vib;
        } else {
            imu.vib_gyro *= vib;
            imu.vib_accel *= vib;
        }

        sc.rc.rate_hz = *r.pick(&[50.0, 150.0, 250.0, 500.0]);
        sc.rc.jitter_us = r.range_u64(50, 1500);

        // Weather: indoor calm to gusty outdoors.
        if r.chance(0.5) {
            let speed = r.range(0.0, 4.0);
            let dir = r.range(0.0, core::f32::consts::TAU);
            sc.wind.mean = Vec3::new(libm::cosf(dir) * speed, libm::sinf(dir) * speed, 0.0);
            sc.wind.gust_sigma = r.range(0.0, 1.5);
            sc.wind.gust_tau = r.range(0.5, 3.0);
            tags.push(format!("wind{speed:.0}"));
        }

        // Boards mount the IMU in any orientation; the configuration must
        // match it.
        const ALIGN: [ImuAlignment; 4] = [ImuAlignment::Cw0, ImuAlignment::Cw90, ImuAlignment::Cw180, ImuAlignment::Cw270];
        if r.chance(0.3) {
            let a = *r.pick(&ALIGN[1..]);
            sc.imu.mount = a;
            sc.fc.imu_alignment = a;
            tags.push("imu-rotated".into());
        }
        // Build mistakes and handling errors (rare).
        if r.chance(0.04) {
            // Configured a quarter turn off from how the chip is mounted.
            let i = ALIGN.iter().position(|&a| a == sc.fc.imu_alignment).unwrap();
            sc.imu.mount = ALIGN[(i + 1) % 4];
            tags.push("imu-misaligned".into());
        }
        if r.chance(0.4) {
            // No bidirectional DShot: older ESC firmware, or turned off.
            sc.esc_telemetry = false;
            tags.push("no-telemetry".into());
        }
        if r.chance(0.03) {
            sc.motors_swapped = true;
            tags.push("motors-swapped".into());
        }
        if r.chance(0.03) {
            sc.battery.absent = true;
            tags.push("usb-power".into());
        }
        if r.chance(0.08) {
            sc.arm_switch_at_boot = true;
            tags.push("arm-on-boot".into());
        }
        // The receiver often binds a second or two after power-up, after gyro
        // calibration; the first frames may already carry the arm switch on.
        let link_delay = if r.chance(0.12) {
            tags.push("late-link".into());
            if r.chance(0.6) {
                sc.arm_switch_at_boot = true;
                tags.push("arm-on-boot".into());
            }
            r.range_u64(1_200_000, 2_500_000)
        } else {
            0
        };
        if r.chance(0.08) {
            sc.moved_at_boot = true;
            tags.push("moved-at-boot".into());
        }
        if r.chance(0.06) {
            sc.ground_tilt = (r.range(0.6, 1.5) * r.sign(), 0.0);
            tags.push("on-its-side".into());
        }
        sc.initial_yaw = r.range(-core::f32::consts::PI, core::f32::consts::PI);

        // Flight plan.
        let s = 1_000_000u64;
        let mut t = (r.range_u64(15, 30) * s / 10).max(link_delay + 1_500_000);
        let mut mode = if r.chance(0.5) { FlightMode::Acro } else { FlightMode::Angle };
        sc.segments.push(Segment { start_us: 0, mode, action: Action::Disarmed { throttle: 0.0 } });
        if r.chance(0.1) {
            // Try to arm with the throttle up first; it must be refused.
            sc.segments.push(Segment { start_us: t, mode, action: Action::Arm { throttle: 0.4 } });
            t += s;
            sc.segments.push(Segment { start_us: t, mode, action: Action::Disarmed { throttle: 0.0 } });
            t += s / 2;
            tags.push("arm-throttle-up".into());
        }
        sc.segments.push(Segment { start_us: t, mode, action: Action::Arm { throttle: 0.0 } });
        t += s / 2;
        let alt = r.range(1.0, 3.0);
        sc.segments.push(Segment { start_us: t, mode, action: Action::Hover { alt } });
        t += r.range_u64(20, 40) * s / 10;
        let n = r.range_u64(3, 8);
        for _ in 0..n {
            if r.chance(0.2) {
                mode = if mode == FlightMode::Acro { FlightMode::Angle } else { FlightMode::Acro };
                tags.push("mode-switch".into());
            }
            let (action, dur) = match r.range_u64(0, 8) {
                0 | 1 => (Action::Hover { alt: r.range(1.0, 3.5) }, r.range_u64(15, 40) * s / 10),
                2 => {
                    let tilt = r.range(0.1, 0.7);
                    let dir = r.range(0.0, core::f32::consts::TAU);
                    (
                        Action::Cruise { alt: r.range(1.0, 3.0), roll: tilt * libm::sinf(dir), pitch: tilt * libm::cosf(dir) },
                        r.range_u64(10, 30) * s / 10,
                    )
                }
                3 if mode == FlightMode::Acro => {
                    // Climb first so there is room for the flip.
                    sc.segments.push(Segment { start_us: t, mode, action: Action::Hover { alt: 4.0 } });
                    t += 3 * s;
                    tags.push("flip".into());
                    (Action::Flip { axis: r.range_u64(0, 2) as u8, dir: r.sign() }, 2 * s)
                }
                6 if mode == FlightMode::Acro => {
                    // Climb high: a 5 s hang falls about 35 m. Hangs past
                    // about 4 s read like lying on the ground (see crash
                    // detection in the firmware).
                    sc.segments.push(Segment { start_us: t, mode, action: Action::Hover { alt: 45.0 } });
                    t += 22 * s;
                    tags.push("inverted-hang".into());
                    let yaw = if r.chance(0.5) { r.range(-1.0, 1.0) } else { 0.0 };
                    (Action::InvertedHang { secs: r.range(0.5, 5.0), yaw }, 10 * s)
                }
                7 => {
                    // Room below for an open-loop sequence (acro: rates;
                    // angle: how far each stick tilts the quad).
                    sc.segments.push(Segment { start_us: t, mode, action: Action::Hover { alt: 6.0 } });
                    t += 4 * s;
                    tags.push("stick-script".into());
                    (Action::StickScript { amp: r.range(0.2, 0.6) }, STICK_SCRIPT_US)
                }
                4 if mode == FlightMode::Angle => {
                    tags.push("punch".into());
                    (Action::Punch, r.range_u64(3, 8) * s / 10)
                }
                5 => {
                    tags.push("stick-slam".into());
                    (Action::StickSlam, r.range_u64(5, 20) * s / 10)
                }
                _ => (Action::Hover { alt: r.range(1.0, 3.0) }, 2 * s),
            };
            sc.segments.push(Segment { start_us: t, mode, action });
            t += dur;
            if action.is_maneuver() {
                sc.segments.push(Segment { start_us: t, mode, action: Action::Hover { alt: 2.0 } });
                t += 3 * s;
            }
        }
        let mut airborne_loss = None;
        if r.chance(0.2) {
            // Climb, then lose the link for good: the failsafe has to bring
            // the quad down by itself from up to 15 m.
            tags.push("rc-loss-airborne".into());
            let alt = r.range(1.5, 15.0);
            let climb = 3 * s + (alt / 3.0 * 1e6) as u64;
            sc.segments.push(Segment { start_us: t, mode, action: Action::Hover { alt } });
            t += climb;
            airborne_loss = Some(t - s / 2);
            sc.segments.push(Segment { start_us: t, mode, action: Action::Land });
            t += 12 * s;
        }
        if r.chance(0.1) {
            tags.push("drop".into());
            sc.segments.push(Segment { start_us: t, mode, action: Action::Drop });
            t += 3 * s;
            // A dazed pilot leaves it armed: crash detection must cut motors.
            t += 2 * s;
        } else {
            sc.segments.push(Segment { start_us: t, mode, action: Action::Land });
            t += 5 * s;
        }
        sc.segments.push(Segment { start_us: t, mode, action: Action::Disarmed { throttle: 0.0 } });
        t += 2 * s;
        sc.duration_us = t;

        // Faults: zero to three, placed while flying (from 2 s after the
        // first takeoff until the final landing begins).
        let takeoff = sc.segments.iter().find(|g| matches!(g.action, Action::Hover { .. })).map_or(0, |g| g.start_us);
        let flight = (takeoff + 2 * s, t.saturating_sub(7 * s));
        let n_faults = *r.pick(&[0u64, 0, 1, 1, 2, 3]);
        for _ in 0..n_faults {
            let at = r.range_u64(flight.0, flight.1.max(flight.0 + 1));
            let motor = r.range_u64(0, 4) as u8;
            let (kind, dur, tag) = match r.range_u64(0, 18) {
                // Log-uniform, so short 2-5 ms gaps are as common as long ones.
                0 => (FaultKind::ImuDropout, libm::expf(r.range(libm::logf(1_000.0), libm::logf(60_000.0))) as u64, "imu-dropout"),
                1 => (FaultKind::ImuStuck, r.range_u64(5_000, 100_000), "imu-stuck"),
                2 => (FaultKind::ImuGarbage { p: r.range(0.001, 0.05) }, r.range_u64(100_000, 3_000_000), "imu-garbage"),
                3 => {
                    let k = r.range(0.004, 0.02);
                    let axis = Vec3::new(r.normal(), r.normal(), 0.3 * r.normal()).normalized();
                    (FaultKind::Kick { torque: axis * k }, r.range_u64(5_000, 40_000), "kick")
                }
                4 => (FaultKind::MotorDesync { motor }, r.range_u64(20_000, 300_000), "desync"),
                5 if r.chance(0.3) => (FaultKind::MotorDead { motor }, u64::MAX / 4, "motor-dead"),
                6 => (FaultKind::MotorDegraded { motor, factor: r.range(0.6, 0.9) }, u64::MAX / 4, "motor-degraded"),
                7 | 8 => (FaultKind::RcLoss, *r.pick(&[30_000, 80_000, 150_000, 600_000, 1_500_000, 8_000_000]), "rc-loss"),
                9 => (FaultKind::RcGlitch { p: r.range(0.01, 0.3) }, r.range_u64(200_000, 3_000_000), "rc-glitch"),
                10 => (FaultKind::TelemetryLoss, r.range_u64(100_000, 2_000_000), "telemetry-loss"),
                11 => {
                    let d = r.range(0.0, core::f32::consts::TAU);
                    let v = r.range(2.0, 6.0);
                    (FaultKind::WindShift { wind: Vec3::new(libm::cosf(d) * v, libm::sinf(d) * v, 0.0) }, r.range_u64(1_000_000, 4_000_000), "wind-gust")
                }
                12 => (FaultKind::ClockGlitch { back_us: r.range_u64(1, 5_000) }, 1, "clock-glitch"),
                13 => (FaultKind::TimingJitter { jitter_us: r.range_u64(50, 400) }, r.range_u64(500_000, 3_000_000), "jitter"),
                14 => (FaultKind::RebootRequest, 1, "reboot-request"),
                15 => {
                    // 15-40 rad/s of rate change in well under a millisecond.
                    let dw = r.range(15.0, 40.0);
                    let dur = r.range_u64(300, 900);
                    let axis = Vec3::new(r.normal(), r.normal(), 0.2 * r.normal()).normalized();
                    let torque = axis * (sc.frame.inertia.x * dw / (dur as f32 * 1e-6));
                    (FaultKind::Impact { torque }, dur, "impact")
                }
                16 => (FaultKind::EventDrop { p: r.range(0.01, 0.2) }, r.range_u64(200_000, 3_000_000), "event-drop"),
                17 => (FaultKind::FcStall, r.range_u64(1_000, 15_000), "fc-stall"),
                _ => continue,
            };
            sc.faults.push(Fault { at_us: at, dur_us: dur, kind });
            tags.push(tag.into());
        }
        if let Some(at) = airborne_loss {
            sc.faults.push(Fault { at_us: at, dur_us: 12_000_000, kind: FaultKind::RcLoss });
        }
        if link_delay > 0 {
            sc.faults.push(Fault { at_us: 0, dur_us: link_delay, kind: FaultKind::RcLoss });
        }
        if r.chance(0.05) {
            // The IMU hangs (or its bus does) just as the pilot arms: the
            // first arm attempt must be refused.
            let arm = sc.segments.iter().find(|g| matches!(g.action, Action::Arm { throttle } if throttle == 0.0)).map_or(0, |g| g.start_us);
            let kind = if r.chance(0.5) { FaultKind::ImuStuck } else { FaultKind::ImuDropout };
            sc.faults.push(Fault { at_us: arm.saturating_sub(200_000), dur_us: 600_000, kind });
            tags.push("imu-dead-at-arm".into());
        }
        sc.faults.sort_by_key(|f| f.at_us);
        if r.chance(0.15) {
            // Start just below 2^32 µs so a 32-bit microsecond clock wraps
            // in the middle of the flight.
            let wrap_at = r.range_u64(3 * s, t.saturating_sub(3 * s).max(3 * s + 1));
            sc.clock_offset_us = (1u64 << 32) - wrap_at;
            tags.push("clock-wrap".into());
        }
        tags.sort();
        tags.dedup();
        sc.name = if tags.is_empty() { "clean".into() } else { tags.join(",") };
        sc
    }
}
