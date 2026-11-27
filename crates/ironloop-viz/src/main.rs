//! Interactive visualizer: fly the real IronLoop firmware in the simulated
//! world, inject faults live, or watch a DST seed replay.
//!
//! The firmware is the same `ironloop_fc::step` that runs on the quad; this
//! app only feeds it simulated events (via `ironloop_sim::Sim`) and draws the
//! result. Builds natively and for the browser (wasm32 + WebGL2).

use std::collections::VecDeque;

use bevy::prelude::*;
use bevy::window::WindowPlugin;
use ironloop_fc::math::{Quat as FQuat, Vec3 as FVec3};
use ironloop_fc::{ArmBlockers, BatteryLevel, FailsafeStage, FlightMode, Notice};
use ironloop_sim::rng::Rng;
use ironloop_sim::{FOREVER, FaultKind, Scenario, Sim, Sticks};

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(0.55, 0.7, 0.85)))
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "IronLoop — Mobula6 firmware sim".into(),
                canvas: Some("#bevy".into()),
                fit_canvas_to_parent: true,
                prevent_default_event_handling: true,
                ..default()
            }),
            ..default()
        }))
        .insert_resource(App_::new())
        .add_message::<FaultPop>()
        .add_systems(Startup, setup)
        .add_systems(Update, (input, step_sim, fault_pops, combat_log_ui, draw, publish_to_page).chain())
        .add_systems(Update, fit_hud_to_window)
        .run();
}

/// The FPV camera is tilted up like on a real whoop, so the view is level
/// when the quad leans forward to fly.
const FPV_UPTILT_DEG: f32 = 25.0;
/// Camera position on the canopy, at the lens (body frame in Bevy axes:
/// x right, y up, -z forward).
const FPV_LENS: Vec3 = Vec3::new(0.0, 0.010, -0.0235);
/// Vertical field of view. Real whoop cameras are wide fisheyes (about
/// 150° diagonal); this is the widest rectilinear view that still looks right.
const FPV_FOV_DEG: f32 = 90.0;
const THIRD_PERSON_FOV_DEG: f32 = 70.0;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum CamMode {
    Chase,
    Fpv,
    Ground,
}

#[derive(Resource)]
struct App_ {
    sim: Sim,
    paused: bool,
    time_scale: f32,
    /// DST seed being replayed (virtual pilot flying), or None for manual.
    replay: Option<u64>,
    next_seed: u64,
    sticks: Sticks,
    trail: VecDeque<Vec3>,
    cam: CamMode,
    prop_angle: [f32; 4],
    rc_loss_on: bool,
    help: bool,
    /// Arm and flight-mode button presses already applied from the page's
    /// on-screen transmitter.
    page_arm_presses: u32,
    page_mode_presses: u32,
    page_cam_presses: u32,
    page_fz_presses: u32,
    page_clr_presses: u32,
    /// Third-person camera to return to when FPV is switched off.
    prev_cam: CamMode,
    /// Automatic failure injection: while flying, add a DST-style fault
    /// every few seconds, one at a time and only the kinds the firmware
    /// flies through, so its fault handling can be watched.
    auto_faults: bool,
    next_auto_fault_us: u64,
    /// Indices into `sim.sc.faults` of the faults auto mode injected, so
    /// switching it off ends them.
    auto_fault_marks: Vec<usize>,
    /// How many faults auto mode has injected this session.
    auto_faults_injected: u32,
    /// The combat log: every fault landing and ending, newest last.
    combat_log: Vec<CombatEntry>,
    /// Which faults have been logged as started (`fault_seen[i]` tracks
    /// `sc.faults[i]`).
    fault_seen: Vec<bool>,
    rng_auto: Rng,
}

impl App_ {
    fn new() -> Self {
        let mut sim = Sim::new(Scenario::manual(1));
        sim.manual = Some(Sticks::default());
        let rng_auto = Rng::fork(sim.sc.seed, "auto-faults");
        App_ {
            sim,
            paused: false,
            time_scale: 1.0,
            replay: None,
            next_seed: 0,
            sticks: Sticks::default(),
            trail: VecDeque::new(),
            cam: CamMode::Chase,
            prop_angle: [0.0; 4],
            rc_loss_on: false,
            // The on-screen sticks sit where the help panel would; H shows it.
            help: false,
            page_arm_presses: 0,
            page_mode_presses: 0,
            page_cam_presses: 0,
            page_fz_presses: 0,
            page_clr_presses: 0,
            prev_cam: CamMode::Chase,
            auto_faults: false,
            next_auto_fault_us: 0,
            auto_fault_marks: Vec::new(),
            auto_faults_injected: 0,
            combat_log: Vec::new(),
            fault_seen: Vec::new(),
            rng_auto,
        }
    }

    fn reset_manual(&mut self) {
        let mut sim = Sim::new(Scenario::manual(self.sim.sc.seed.wrapping_add(1)));
        sim.manual = Some(Sticks::default());
        self.sim = sim;
        self.replay = None;
        self.sticks = Sticks::default();
        self.trail.clear();
        self.rc_loss_on = false;
        self.after_flight_reset();
    }

    fn load_seed(&mut self, seed: u64) {
        self.sim = Sim::new(Scenario::generate(seed));
        self.replay = Some(seed);
        self.trail.clear();
        self.rc_loss_on = false;
        self.after_flight_reset();
    }

    /// Auto fault injection follows the flight: its marks point into the old
    /// sim's fault list, and the first fault of the next flight comes soon.
    fn after_flight_reset(&mut self) {
        self.auto_fault_marks.clear();
        self.fault_seen.clear();
        self.next_auto_fault_us = self.sim.t_us + self.rng_auto.range_u64(2_000_000, 4_000_000);
    }

    /// Flips automatic failure injection, ending the faults it injected.
    fn toggle_auto_faults(&mut self) {
        self.auto_faults = !self.auto_faults;
        let t = self.sim.t_us;
        if self.auto_faults {
            self.next_auto_fault_us = t + self.rng_auto.range_u64(1_000_000, 3_000_000);
        } else {
            for i in self.auto_fault_marks.drain(..) {
                let f = &mut self.sim.sc.faults[i];
                if f.active(t) {
                    f.dur_us = t - f.at_us;
                }
            }
        }
    }

    /// The next fault to inject: one of the DST workload generator's kinds
    /// with its parameter ranges, minus the ones that end the flight (a dead
    /// motor, an impact that tumbles the quad, a mid-air reboot, a link loss
    /// long enough to land it). Motor degradation is gentler than the DST's
    /// (0.7–0.9 rather than 0.6–0.9) so the quad stays easy to fly.
    fn next_auto_fault(&mut self) -> (FaultKind, u64) {
        let r = &mut self.rng_auto;
        let motor = r.range_u64(0, 4) as u8;
        match r.range_u64(0, 15) {
            // Log-uniform, so short 2-5 ms gaps are as common as long ones.
            0 => (FaultKind::ImuDropout, r.range(1_000.0f32.ln(), 60_000.0f32.ln()).exp() as u64),
            1 => (FaultKind::ImuStuck, r.range_u64(5_000, 100_000)),
            2 => (FaultKind::ImuGarbage { p: r.range(0.001, 0.05) }, r.range_u64(100_000, 2_000_000)),
            3 => {
                let k = r.range(0.004, 0.02);
                let axis = FVec3::new(r.normal(), r.normal(), 0.3 * r.normal()).normalized();
                (FaultKind::Kick { torque: axis * k }, r.range_u64(5_000, 40_000))
            }
            4 => (FaultKind::MotorDesync { motor }, r.range_u64(20_000, 300_000)),
            5 => (FaultKind::MotorDegraded { motor, factor: r.range(0.7, 0.9) }, FOREVER),
            6 | 7 => (FaultKind::RcLoss, *r.pick(&[30_000, 80_000, 150_000, 600_000])),
            8 => (FaultKind::RcGlitch { p: r.range(0.01, 0.3) }, r.range_u64(200_000, 3_000_000)),
            9 => (FaultKind::TelemetryLoss, r.range_u64(100_000, 2_000_000)),
            10 => {
                let d = r.range(0.0, core::f32::consts::TAU);
                let v = r.range(2.0, 6.0);
                (FaultKind::WindShift { wind: FVec3::new(d.cos() * v, d.sin() * v, 0.0) }, r.range_u64(1_000_000, 4_000_000))
            }
            11 => (FaultKind::ClockGlitch { back_us: r.range_u64(1, 5_000) }, 1),
            12 => (FaultKind::TimingJitter { jitter_us: r.range_u64(50, 400) }, r.range_u64(500_000, 3_000_000)),
            13 => (FaultKind::EventDrop { p: r.range(0.01, 0.2) }, r.range_u64(200_000, 2_000_000)),
            _ => (FaultKind::FcStall, r.range_u64(1_000, 15_000)),
        }
    }

    /// While auto mode is on and the user is flying, inject one fault when
    /// the last one has long ended, then draw the next gap.
    fn auto_fault_tick(&mut self, pops: &mut MessageWriter<FaultPop>) {
        if !self.auto_faults || self.replay.is_some() {
            return;
        }
        let t = self.sim.t_us;
        let flying = self.sim.status.armed
            && self.sim.world.altitude() > 0.3
            && self.sim.status.failsafe == FailsafeStage::None
            && !self.sim.done();
        let quiet = self.sim.sc.faults.iter().all(|f| !f.active(t));
        if !flying || !quiet || t < self.next_auto_fault_us {
            return;
        }
        let (kind, dur) = self.next_auto_fault();
        self.sim.inject(kind, dur);
        self.auto_fault_marks.push(self.sim.sc.faults.len() - 1);
        self.auto_faults_injected += 1;
        self.next_auto_fault_us = t + self.rng_auto.range_u64(4_000_000, 10_000_000);
        let (label, color) = fault_pop(&kind);
        pops.write(FaultPop { label: label.into(), color });
    }

    /// Appends to the combat log, keeping the last 100 entries.
    fn log(&mut self, t_us: u64, text: String, color: Color) {
        self.combat_log.push(CombatEntry { t_us, text, color });
        if self.combat_log.len() > 100 {
            self.combat_log.remove(0);
        }
    }

    /// Watches the sim's fault list for faults starting and ending, and
    /// logs each: the combat log's hits and their outcomes.
    fn update_combat_log(&mut self) {
        let t = self.sim.t_us;
        self.fault_seen.resize(self.sim.sc.faults.len(), false);
        for i in 0..self.sim.sc.faults.len() {
            let f = self.sim.sc.faults[i];
            let was = self.fault_seen[i];
            if f.active(t) && !was {
                self.fault_seen[i] = true;
                let (label, color) = fault_pop(&f.kind);
                let dur = if f.dur_us == FOREVER {
                    "until cleared".to_string()
                } else if f.dur_us >= 1_000_000 {
                    format!("{:.1} s", f.dur_us as f64 * 1e-6)
                } else {
                    format!("{} ms", f.dur_us / 1000)
                };
                let tag = if self.auto_fault_marks.contains(&i) { "  -  auto" } else { "" };
                self.log(t, format!("{label} - {dur}{tag}"), color);
            } else if !f.active(t) && was {
                self.fault_seen[i] = false;
                let (label, _) = fault_pop(&f.kind);
                self.log(t, format!("{label} over - {:.1} s", (t - f.at_us) as f64 * 1e-6), Color::srgba(0.75, 0.8, 0.85, 0.85));
            }
        }
    }
}

/// NED world / FRD body (firmware and sim) to Bevy's Y-up, -Z-forward.
fn to_bevy(v: FVec3) -> Vec3 {
    Vec3::new(v.y, -v.z, -v.x)
}
fn quat_to_bevy(q: FQuat) -> Quat {
    Quat::from_xyzw(q.y, -q.z, -q.x, q.w)
}

#[derive(Component)]
struct Drone;
#[derive(Component)]
struct Prop(usize);
#[derive(Component)]
struct MainCam;
#[derive(Component)]
struct HudLeft;
#[derive(Component)]
struct HudRight;
#[derive(Component)]
struct HudHelp;

/// A fault that just landed, shown as a floating damage number over the
/// quad, damage-numbers-in-an-MMO style.
#[derive(Message)]
struct FaultPop {
    label: String,
    color: Color,
}

/// What a fault announces itself as, and in what color.
fn fault_pop(kind: &FaultKind) -> (String, Color) {
    let red = Color::srgb(1.0, 0.30, 0.25);
    let orange = Color::srgb(1.0, 0.62, 0.20);
    let yellow = Color::srgb(1.0, 0.92, 0.35);
    let violet = Color::srgb(0.80, 0.55, 1.0);
    let cyan = Color::srgb(0.45, 0.95, 1.0);
    match kind {
        FaultKind::ImuDropout => ("IMU DROPOUT".into(), orange),
        FaultKind::ImuStuck => ("IMU STUCK".into(), orange),
        FaultKind::ImuGarbage { .. } => ("IMU CORRUPT SAMPLES".into(), yellow),
        FaultKind::Kick { .. } => ("PROP STRIKE".into(), red),
        FaultKind::Impact { .. } => ("HARD IMPACT".into(), red),
        FaultKind::MotorDesync { motor } => (format!("MOTOR {motor} DESYNC"), red),
        FaultKind::MotorDead { motor } => (format!("MOTOR {motor} DEAD"), red),
        FaultKind::MotorDegraded { motor, factor } => {
            (format!("MOTOR {motor} WEAK {}%", ((1.0 - factor) * 100.0) as i32), orange)
        }
        FaultKind::RcLoss => ("RC LINK LOST".into(), red),
        FaultKind::RcGlitch { .. } => ("RC FRAME CORRUPT".into(), yellow),
        FaultKind::TelemetryLoss => ("DSHOT TELEM LOST".into(), yellow),
        FaultKind::WindShift { wind } => (format!("GUST {:.1} m/s", wind.norm()), cyan),
        FaultKind::ClockGlitch { .. } => ("CLOCK JUMPED BACK".into(), violet),
        FaultKind::TimingJitter { .. } => ("SCHEDULER JITTER".into(), violet),
        FaultKind::EventDrop { .. } => ("EVENT QUEUE DROP".into(), violet),
        FaultKind::FcStall => ("FC TASK STALL".into(), violet),
        FaultKind::RebootRequest => ("REBOOT REQUEST".into(), red),
    }
}

/// One floating damage number: anchored to a world point, it pops large,
/// rises, and fades.
#[derive(Component)]
struct Pop {
    anchor: Vec3,
    age: f32,
    dur: f32,
    base_size: f32,
    dx: f32,
}

/// One combat-log line.
struct CombatEntry {
    t_us: u64,
    text: String,
    color: Color,
}

#[derive(Component)]
struct HudLog;

/// Rebuilds the combat log panel's lines whenever the log changed.
fn combat_log_ui(
    app: Res<App_>,
    mut rendered: Local<usize>,
    log_ui: Query<Entity, With<HudLog>>,
    children: Query<&Children, With<HudLog>>,
    mut commands: Commands,
) {
    if app.combat_log.len() == *rendered {
        return;
    }
    *rendered = app.combat_log.len();
    let Ok(e) = log_ui.single() else { return; };
    for child in children.iter().flatten() {
        commands.entity(*child).despawn();
    }
    commands.entity(e).with_children(|p| {
        for entry in app.combat_log.iter().rev().take(9).collect::<Vec<_>>().into_iter().rev() {
            p.spawn((
                TextSpan(format!("{:>7.2}s  {}\n", entry.t_us as f64 * 1e-6, entry.text)),
                TextFont { font_size: FontSize::Px(9.5), ..default() },
                TextColor(entry.color),
            ));
        }
    });
}

/// Spawns a pop for each fault that lands and animates the live ones.
#[allow(clippy::type_complexity)]
fn fault_pops(
    time: Res<Time>,
    mut commands: Commands,
    mut events: MessageReader<FaultPop>,
    mut pops: Query<(Entity, &mut Node, &mut TextColor, &mut TextFont, &mut Pop)>,
    cam: Query<(&Camera, &GlobalTransform), (With<MainCam>, Without<Drone>)>,
    drone: Query<&GlobalTransform, (With<Drone>, Without<MainCam>)>,
) {
    let dt = time.delta_secs().min(0.1);
    if let Ok(t) = drone.single() {
        // Spawn above the canopy, jittered left and right like the MMO
        // crits stacking on one target.
        let anchor = t.translation() + Vec3::Y * 0.06;
        let seed = (time.elapsed_secs() * 997.0) as usize;
        for pop in events.read() {
            let dx = ((seed % 61) as f32 - 30.0) * 1.6;
            commands.spawn((
                Node { position_type: PositionType::Absolute, ..default() },
                Text::new(pop.label.clone()),
                TextFont { font_size: FontSize::Px(30.0), ..default() },
                TextColor(pop.color),
                TextLayout::justify(Justify::Center),
                Pop { anchor, age: 0.0, dur: 1.4, base_size: 30.0, dx },
            ));
        }
    }
    let mut dead = Vec::new();
    for (e, mut node, mut color, mut font, mut pop) in &mut pops {
        pop.age += dt;
        if pop.age >= pop.dur {
            dead.push(e);
            continue;
        }
        // Pop: starts half again as big and settles within 150 ms.
        let grow = 1.0 + 0.5 * (1.0 - (pop.age / 0.15).min(1.0));
        font.font_size = FontSize::Px(pop.base_size * grow);
        if let Ok((cam, tr)) = cam.single() {
            if let Ok(vp) = cam.world_to_viewport(tr, pop.anchor) {
                node.left = Val::Px(vp.x + pop.dx);
                node.top = Val::Px(vp.y - 40.0 - pop.age * 70.0);
            }
        }
        let fade = ((pop.dur - pop.age) / 0.5).min(1.0);
        color.0.set_alpha(fade.max(0.0));
    }
    for e in dead {
        commands.entity(e).despawn();
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut mats: ResMut<Assets<StandardMaterial>>,
    app: Res<App_>,
) {
    let mat = |mats: &mut Assets<StandardMaterial>, c: Color, rough: f32| {
        mats.add(StandardMaterial { base_color: c, perceptual_roughness: rough, ..default() })
    };

    // Ground, home pad, and reference objects.
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(400.0, 400.0))),
        MeshMaterial3d(mat(&mut mats, Color::srgb(0.32, 0.42, 0.3), 0.95)),
    ));
    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.25, 0.004))),
        MeshMaterial3d(mat(&mut mats, Color::srgb(0.85, 0.85, 0.8), 0.8)),
        Transform::from_xyz(0.0, 0.002, 0.0),
    ));
    let pillar = meshes.add(Cuboid::new(0.3, 3.0, 0.3));
    let pillar_mat = mat(&mut mats, Color::srgb(0.75, 0.55, 0.35), 0.8);
    for (x, z) in [(-4.0, -6.0), (4.0, -6.0), (-4.0, -14.0), (4.0, -14.0), (0.0, -22.0), (10.0, 4.0), (-10.0, 4.0)] {
        commands.spawn((Mesh3d(pillar.clone()), MeshMaterial3d(pillar_mat.clone()), Transform::from_xyz(x, 1.5, z)));
    }
    let gate = meshes.add(Torus::new(0.55, 0.62));
    let gate_mat = mat(&mut mats, Color::srgb(0.95, 0.35, 0.1), 0.6);
    for z in [-10.0f32, -18.0, -26.0] {
        commands.spawn((
            Mesh3d(gate.clone()),
            MeshMaterial3d(gate_mat.clone()),
            Transform::from_xyz(0.0, 1.5, z).with_rotation(Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
        ));
    }

    // The quad: 65 mm whoop with four ducts, props, canopy, and camera.
    let f = &app.sim.sc.frame;
    let duct = meshes.add(Torus::new(0.0145, 0.0175));
    let duct_mat = mat(&mut mats, Color::srgb(0.12, 0.12, 0.14), 0.5);
    let blade = meshes.add(Cuboid::new(0.029, 0.0012, 0.005));
    let front_prop = mat(&mut mats, Color::srgb(0.95, 0.4, 0.1), 0.5);
    let rear_prop = mat(&mut mats, Color::srgb(0.2, 0.5, 0.95), 0.5);
    commands
        .spawn((Drone, Transform::default(), Visibility::default()))
        .with_children(|p| {
            p.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.026, 0.012, 0.036))),
                MeshMaterial3d(mat(&mut mats, Color::srgb(0.85, 0.85, 0.9), 0.4)),
                Transform::from_xyz(0.0, 0.004, 0.0),
            ));
            p.spawn((
                // FPV camera, tilted up by the same angle the view uses.
                Mesh3d(meshes.add(Cuboid::new(0.01, 0.01, 0.006))),
                MeshMaterial3d(mat(&mut mats, Color::srgb(0.05, 0.05, 0.05), 0.3)),
                Transform::from_xyz(0.0, 0.01, -0.02).with_rotation(Quat::from_rotation_x(FPV_UPTILT_DEG.to_radians())),
            ));
            for i in 0..4 {
                let pos = to_bevy(f.motor_pos[i]);
                p.spawn((Mesh3d(duct.clone()), MeshMaterial3d(duct_mat.clone()), Transform::from_translation(pos)));
                let front = f.motor_pos[i].x > 0.0;
                p.spawn((
                    Prop(i),
                    Mesh3d(blade.clone()),
                    MeshMaterial3d(if front { front_prop.clone() } else { rear_prop.clone() }),
                    Transform::from_translation(pos + Vec3::Y * 0.004),
                ));
            }
        });

    commands.spawn((
        DirectionalLight { illuminance: 12_000.0, shadow_maps_enabled: true, ..default() },
        Transform::from_xyz(8.0, 20.0, 6.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.insert_resource(GlobalAmbientLight { brightness: 400.0, ..default() });
    commands.spawn((
        MainCam,
        Camera3d::default(),
        Projection::Perspective(PerspectiveProjection { fov: 70f32.to_radians(), near: 0.005, ..default() }),
        Transform::from_xyz(0.0, 0.4, 1.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    let font = |size: f32| TextFont { font_size: FontSize::Px(size), ..default() };
    let panel = BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.45));
    commands.spawn((
        HudLeft,
        Text::new(""),
        font(10.0),
        TextColor(Color::WHITE),
        panel,
        Node { position_type: PositionType::Absolute, top: px(6), left: px(6), padding: UiRect::all(px(6)), ..default() },
    ));
    commands.spawn((
        HudRight,
        Text::new(""),
        font(9.5),
        TextColor(Color::srgb(1.0, 0.95, 0.7)),
        panel,
        Node { position_type: PositionType::Absolute, top: px(6), right: px(6), padding: UiRect::all(px(6)), ..default() },
    ));
    commands.spawn((
        HudHelp,
        Visibility::Hidden,
        Text::new(HELP),
        font(9.5),
        TextColor(Color::srgb(0.9, 0.95, 1.0)),
        panel,
        Node { position_type: PositionType::Absolute, bottom: px(6), left: px(6), padding: UiRect::all(px(6)), ..default() },
    ));
    // The combat log, lifted above the on-screen transmitter's right stick.
    commands.spawn((
        HudLog,
        Text::new(""),
        font(9.5),
        TextColor(Color::srgb(0.9, 0.95, 1.0)),
        panel,
        Node { position_type: PositionType::Absolute, bottom: px(300), right: px(6), padding: UiRect::all(px(6)), ..default() },
    ));
}

const HELP: &str = "\
Space arm switch   W/S throttle   A/D yaw   arrows roll/pitch (Shift = full)   X cut throttle
M angle/acro   C camera   F auto faults   R reset   P pause   ,/. slower/faster   H toggle help
Gamepad (mode 2): left stick throttle/yaw, right stick roll/pitch, A arm, B mode
Faults: F auto inject while flying  1 RC loss (toggle)  2 desync 0.3s  3 kill motor
        4 IMU dropout  5 IMU garbage  6 wind gust  7 collision  8 drain battery  0 clear
DST: N replay next seed with the virtual pilot   B back to manual";

fn input(
    keys: Res<ButtonInput<KeyCode>>,
    gamepads: Query<&Gamepad>,
    time: Res<Time>,
    mut app: ResMut<App_>,
    mut help: Query<&mut Visibility, With<HudHelp>>,
    mut pops: MessageWriter<FaultPop>,
) {
    let dt = time.delta_secs().min(0.1);
    let app = &mut *app;
    if keys.just_pressed(KeyCode::KeyR) {
        app.reset_manual();
    }
    if keys.just_pressed(KeyCode::KeyN) {
        let seed = app.next_seed;
        app.next_seed += 1;
        app.load_seed(seed);
    }
    if keys.just_pressed(KeyCode::KeyB) && app.replay.is_some() {
        app.reset_manual();
    }
    if keys.just_pressed(KeyCode::KeyP) {
        app.paused = !app.paused;
    }
    if keys.just_pressed(KeyCode::Comma) {
        app.time_scale = (app.time_scale * 0.5).max(0.125);
    }
    if keys.just_pressed(KeyCode::Period) {
        app.time_scale = (app.time_scale * 2.0).min(8.0);
    }
    if keys.just_pressed(KeyCode::KeyC) {
        app.cam = match app.cam {
            CamMode::Chase => CamMode::Fpv,
            CamMode::Fpv => CamMode::Ground,
            CamMode::Ground => CamMode::Chase,
        };
        if app.cam != CamMode::Fpv {
            app.prev_cam = app.cam;
        }
    }
    if keys.just_pressed(KeyCode::KeyF) {
        app.toggle_auto_faults();
    }
    if keys.just_pressed(KeyCode::KeyH) {
        app.help = !app.help;
        for mut v in &mut help {
            *v = if app.help { Visibility::Inherited } else { Visibility::Hidden };
        }
    }

    // Fault injection.
    let sim = &mut app.sim;
    let motor = (sim.t_us / 1000 % 4) as u8;
    let mut deferred_log: Option<(u64, String, Color)> = None;
    if keys.just_pressed(KeyCode::Digit1) {
        app.rc_loss_on = !app.rc_loss_on;
        if app.rc_loss_on {
            sim.inject(FaultKind::RcLoss, FOREVER);
            let (label, color) = fault_pop(&FaultKind::RcLoss);
            pops.write(FaultPop { label: label.into(), color });
        } else {
            let t = sim.t_us;
            for f in &mut sim.sc.faults {
                if f.kind == FaultKind::RcLoss && f.active(t) {
                    f.dur_us = t - f.at_us;
                }
            }
            pops.write(FaultPop { label: "LINK OK".into(), color: Color::srgb(0.4, 1.0, 0.5) });
        }
    }
    if keys.just_pressed(KeyCode::Digit2) {
        sim.inject(FaultKind::MotorDesync { motor }, 300_000);
        let (label, color) = fault_pop(&FaultKind::MotorDesync { motor });
        pops.write(FaultPop { label: label.into(), color });
    }
    if keys.just_pressed(KeyCode::Digit3) {
        sim.inject(FaultKind::MotorDead { motor }, FOREVER);
        let (label, color) = fault_pop(&FaultKind::MotorDead { motor });
        pops.write(FaultPop { label: label.into(), color });
    }
    if keys.just_pressed(KeyCode::Digit4) {
        sim.inject(FaultKind::ImuDropout, 100_000);
        let (label, color) = fault_pop(&FaultKind::ImuDropout);
        pops.write(FaultPop { label: label.into(), color });
    }
    if keys.just_pressed(KeyCode::Digit5) {
        sim.inject(FaultKind::ImuGarbage { p: 0.05 }, 2_000_000);
        let (label, color) = fault_pop(&FaultKind::ImuGarbage { p: 0.05 });
        pops.write(FaultPop { label: label.into(), color });
    }
    if keys.just_pressed(KeyCode::Digit6) {
        let a = (sim.t_us % 6283) as f32 / 1000.0;
        let kind = FaultKind::WindShift { wind: FVec3::new(a.cos() * 5.0, a.sin() * 5.0, 0.0) };
        sim.inject(kind, 4_000_000);
        let (label, color) = fault_pop(&kind);
        pops.write(FaultPop { label: label.into(), color });
    }
    if keys.just_pressed(KeyCode::Digit7) {
        let a = (sim.t_us % 6283) as f32 / 1000.0;
        let kind = FaultKind::Kick { torque: FVec3::new(a.cos() * 0.012, a.sin() * 0.012, 0.002) };
        sim.inject(kind, 25_000);
        let (label, color) = fault_pop(&kind);
        pops.write(FaultPop { label: label.into(), color });
    }
    if keys.just_pressed(KeyCode::Digit8) {
        let b = &mut sim.world.battery;
        b.soc = (b.soc - 0.25).max(0.02);
        pops.write(FaultPop { label: "BATTERY -25%".into(), color: Color::srgb(1.0, 0.92, 0.35) });
        let t = sim.t_us;
        deferred_log = Some((t, "battery drained 25%".into(), Color::srgb(1.0, 0.92, 0.35)));
    }
    if keys.just_pressed(KeyCode::Digit0) {
        sim.clear_faults();
        app.rc_loss_on = false;
        pops.write(FaultPop { label: "CLEARED".into(), color: Color::WHITE });
        let t = sim.t_us;
        deferred_log = Some((t, "faults cleared".into(), Color::WHITE));
    }
    if let Some((t, text, color)) = deferred_log {
        app.log(t, text, color);
    }

    // Sticks.
    let s = &mut app.sticks;
    if keys.just_pressed(KeyCode::Space) {
        s.arm = !s.arm;
    }
    if keys.just_pressed(KeyCode::KeyM) {
        s.acro = !s.acro;
    }
    let full = if keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight) { 1.0 } else { 0.45 };
    let axis = |neg: KeyCode, pos: KeyCode| {
        (keys.pressed(pos) as i32 - keys.pressed(neg) as i32) as f32 * full
    };
    let spring = |cur: f32, target: f32| cur + (target - cur) * (dt * 12.0).min(1.0);
    s.roll = spring(s.roll, axis(KeyCode::ArrowLeft, KeyCode::ArrowRight));
    s.pitch = spring(s.pitch, axis(KeyCode::ArrowDown, KeyCode::ArrowUp));
    s.yaw = spring(s.yaw, axis(KeyCode::KeyA, KeyCode::KeyD));
    if keys.pressed(KeyCode::KeyW) {
        s.throttle = (s.throttle + 0.5 * dt).min(1.0);
    }
    if keys.pressed(KeyCode::KeyS) {
        s.throttle = (s.throttle - 0.5 * dt).max(0.0);
    }
    if keys.just_pressed(KeyCode::KeyX) {
        s.throttle = 0.0;
    }
    for g in &gamepads {
        let get = |a: GamepadAxis| g.get(a).unwrap_or(0.0);
        let ly = get(GamepadAxis::LeftStickY);
        // Spring-centered gamepad stick: center is hover-ish, bottom is 0.
        s.throttle = ((ly + 1.0) * 0.5).clamp(0.0, 1.0);
        s.yaw = get(GamepadAxis::LeftStickX);
        s.roll = get(GamepadAxis::RightStickX);
        s.pitch = get(GamepadAxis::RightStickY);
        if g.just_pressed(GamepadButton::South) {
            s.arm = !s.arm;
        }
        if g.just_pressed(GamepadButton::East) {
            s.acro = !s.acro;
        }
        break;
    }
    // On-screen transmitter in the web page: an axis the user is touching
    // overrides keyboard and gamepad. Released self-centering axes spring
    // back through the keyboard path above; throttle stays where it was left.
    if let Some(page) = page::read_sticks() {
        for (axis, value) in [&mut s.roll, &mut s.pitch, &mut s.yaw, &mut s.throttle].into_iter().zip(page.axes) {
            if let Some(v) = value {
                *axis = v;
            }
        }
        if page.arm_presses != app.page_arm_presses {
            app.page_arm_presses = page.arm_presses;
            s.arm = !s.arm;
        }
        if page.mode_presses != app.page_mode_presses {
            app.page_mode_presses = page.mode_presses;
            s.acro = !s.acro;
        }
        // FPV button: into the drone's camera, and back to the last
        // third-person view.
        if page.cam_presses != app.page_cam_presses {
            app.page_cam_presses = page.cam_presses;
            app.cam = if app.cam == CamMode::Fpv { app.prev_cam } else { CamMode::Fpv };
        }
        // Auto-fault button: automatic failure injection while flying.
        if page.fz_presses != app.page_fz_presses {
            app.page_fz_presses = page.fz_presses;
            app.toggle_auto_faults();
        }
        // Clear button: only visible while an "until cleared" fault is live.
        if page.clr_presses != app.page_clr_presses {
            app.page_clr_presses = page.clr_presses;
            app.sim.clear_faults();
            app.rc_loss_on = false;
            pops.write(FaultPop { label: "CLEARED".into(), color: Color::WHITE });
            app.log(app.sim.t_us, "faults cleared".into(), Color::WHITE);
        }
    }
    if app.replay.is_none() {
        app.sim.manual = Some(app.sticks);
    }
}

/// Shrinks the text HUD on small windows (phones) so the scene and the
/// on-screen sticks stay visible.
fn fit_hud_to_window(windows: Query<&Window>, mut scale: ResMut<UiScale>) {
    if let Ok(w) = windows.single() {
        let k = (w.width() / 1280.0).clamp(0.55, 1.0);
        if (scale.0 - k).abs() > 0.01 {
            scale.0 = k;
        }
    }
}

/// Tells the page what the firmware receives, so the on-screen sticks and
/// arm button also show keyboard, gamepad, and virtual-pilot input.
fn publish_to_page(app: Res<App_>) {
    let sim = &app.sim;
    let (k, st) = (&sim.sticks, &sim.status);
    let status = if st.armed {
        "ARMED".to_string()
    } else if k.arm {
        format!("blocked: {} (flip off, then on)", blockers_text(st.blockers))
    } else {
        blockers_text(st.blockers)
    };
    let fpv = app.cam == CamMode::Fpv;
    let sticky = sim.sc.faults.iter().any(|f| f.dur_us == FOREVER && f.active(sim.t_us));
    let values = [k.roll, k.pitch, k.yaw, k.throttle, k.arm as u8 as f32, st.armed as u8 as f32, k.acro as u8 as f32, fpv as u8 as f32, app.auto_faults as u8 as f32, sticky as u8 as f32];
    page::publish(&values, &status);
}

/// Bridge to the web page's on-screen transmitter (`web/index.html`).
/// The page writes `window.__ilIn` and reads `window.__ilOut`/`__ilStatus`.
#[cfg(target_arch = "wasm32")]
mod page {
    use wasm_bindgen::{JsCast, JsValue};

    pub struct PageSticks {
        /// Roll, pitch, yaw, throttle; `None` when not being touched.
        pub axes: [Option<f32>; 4],
        pub arm_presses: u32,
        pub mode_presses: u32,
        pub cam_presses: u32,
        pub fz_presses: u32,
        pub clr_presses: u32,
    }

    pub fn read_sticks() -> Option<PageSticks> {
        let v = js_sys::Reflect::get(&js_sys::global(), &JsValue::from_str("__ilIn")).ok()?;
        let x = v.dyn_into::<js_sys::Float32Array>().ok()?.to_vec();
        if x.len() < 10 {
            return None;
        }
        Some(PageSticks {
            axes: core::array::from_fn(|i| (x[4 + i] > 0.5).then_some(x[i])),
            arm_presses: x[8] as u32,
            mode_presses: x[9] as u32,
            cam_presses: x.get(10).copied().unwrap_or(0.0) as u32,
            fz_presses: x.get(11).copied().unwrap_or(0.0) as u32,
            clr_presses: x.get(12).copied().unwrap_or(0.0) as u32,
        })
    }

    pub fn publish(values: &[f32], status: &str) {
        let g = js_sys::global();
        let _ = js_sys::Reflect::set(&g, &JsValue::from_str("__ilOut"), &js_sys::Float32Array::from(values));
        let _ = js_sys::Reflect::set(&g, &JsValue::from_str("__ilStatus"), &JsValue::from_str(status));
    }
}

#[cfg(not(target_arch = "wasm32"))]
mod page {
    pub struct PageSticks {
        pub axes: [Option<f32>; 4],
        pub arm_presses: u32,
        pub mode_presses: u32,
        pub cam_presses: u32,
        pub fz_presses: u32,
        pub clr_presses: u32,
    }
    pub fn read_sticks() -> Option<PageSticks> {
        None
    }
    pub fn publish(_: &[f32], _: &str) {}
}

fn step_sim(time: Res<Time>, mut app: ResMut<App_>, mut pops: MessageWriter<FaultPop>) {
    if app.paused || app.sim.done() {
        return;
    }
    let dt = time.delta_secs().min(0.1) * app.time_scale;
    let target = app.sim.t_us + (dt * 1e6) as u64;
    app.sim.run_until(target);
    app.auto_fault_tick(&mut pops);
    app.update_combat_log();
    let pos = to_bevy(app.sim.world.body.pos);
    if app.trail.back().is_none_or(|p| p.distance(pos) > 0.05) {
        app.trail.push_back(pos);
        if app.trail.len() > 600 {
            app.trail.pop_front();
        }
    }
    for i in 0..4 {
        // Shown at a fraction of the true speed so the eye can follow it.
        let w = app.sim.world.motors[i].omega * app.sim.world.frame.spin[i];
        app.prop_angle[i] = (app.prop_angle[i] + w * dt * 0.013) % std::f32::consts::TAU;
    }
}

fn blockers_text(b: ArmBlockers) -> String {
    let names = [
        (ArmBlockers::CALIBRATING, "CALIBRATING"),
        (ArmBlockers::THROTTLE, "THROTTLE"),
        (ArmBlockers::RC_LINK, "RX_LOSS"),
        (ArmBlockers::IMU, "IMU"),
        (ArmBlockers::TILT, "ANGLE"),
        (ArmBlockers::BATTERY, "NO_BATTERY"),
    ];
    let v: Vec<&str> = names.iter().filter(|(bit, _)| b.contains(*bit)).map(|(_, n)| *n).collect();
    if v.is_empty() { "ready".into() } else { v.join(" ") }
}

#[allow(clippy::type_complexity)]
fn draw(
    app: Res<App_>,
    mut gizmos: Gizmos,
    mut drone: Query<&mut Transform, (With<Drone>, Without<Prop>, Without<MainCam>)>,
    mut props: Query<(&Prop, &mut Transform), (Without<Drone>, Without<MainCam>)>,
    mut cam: Query<(&mut Transform, &mut Projection), (With<MainCam>, Without<Drone>, Without<Prop>)>,
    mut left: Query<&mut Text, (With<HudLeft>, Without<HudRight>)>,
    mut right: Query<&mut Text, (With<HudRight>, Without<HudLeft>)>,
    time: Res<Time>,
) {
    let sim = &app.sim;
    let w = &sim.world;
    let st = &sim.status;
    let pos = to_bevy(w.body.pos);
    let rot = quat_to_bevy(w.body.att);
    if let Ok(mut t) = drone.single_mut() {
        *t = Transform::from_translation(pos).with_rotation(rot);
    }
    for (p, mut t) in &mut props {
        t.rotation = Quat::from_rotation_y(app.prop_angle[p.0]);
    }

    // Camera.
    if let Ok((mut c, mut projection)) = cam.single_mut() {
        let fov = if app.cam == CamMode::Fpv { FPV_FOV_DEG } else { THIRD_PERSON_FOV_DEG }.to_radians();
        if let Projection::Perspective(p) = &mut *projection {
            if p.fov != fov {
                p.fov = fov;
            }
        }
        let (_, _, yaw) = w.body.att.to_euler();
        let fwd = to_bevy(FVec3::new(yaw.cos(), yaw.sin(), 0.0));
        match app.cam {
            CamMode::Chase => {
                let want = pos - fwd * 0.38 + Vec3::Y * 0.14;
                let k = (time.delta_secs() * 4.0).min(1.0);
                let prev = c.translation;
                let next = prev + (want - prev) * k;
                *c = Transform::from_translation(next).looking_at(pos + Vec3::Y * 0.03, Vec3::Y);
            }
            CamMode::Fpv => {
                let uptilt = Quat::from_rotation_x(FPV_UPTILT_DEG.to_radians());
                *c = Transform::from_translation(pos + rot * FPV_LENS).with_rotation(rot * uptilt);
            }
            CamMode::Ground => {
                *c = Transform::from_xyz(0.0, 1.5, 3.0).looking_at(pos, Vec3::Y);
            }
        }
    }

    // Grid, trail, and the firmware's attitude estimate next to the truth.
    gizmos.grid(
        Isometry3d::new(Vec3::Y * 0.001, Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
        UVec2::new(60, 60),
        Vec2::splat(1.0),
        Color::srgba(1.0, 1.0, 1.0, 0.12),
    );
    for (a, b) in app.trail.iter().zip(app.trail.iter().skip(1)) {
        gizmos.line(*a, *b, Color::srgba(1.0, 0.9, 0.2, 0.7));
    }
    let up_true = rot * Vec3::Y;
    let up_est = quat_to_bevy(st.attitude) * Vec3::Y;
    gizmos.line(pos, pos + up_true * 0.12, Color::srgb(0.2, 1.0, 0.3));
    // Green: true up vector. Magenta: the firmware's estimate (tilt only;
    // yaw is unobservable without a magnetometer).
    gizmos.line(pos, pos + up_est * 0.12, Color::srgb(1.0, 0.2, 0.9));
    let shadow = Vec3::new(pos.x, 0.003, pos.z);
    gizmos.circle(Isometry3d::new(shadow, Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)), 0.04, Color::srgba(0.0, 0.0, 0.0, 0.5));

    // HUD.
    let mode = match st.mode {
        FlightMode::Acro => "ACRO",
        FlightMode::Angle => "ANGLE",
    };
    let arm = if st.armed {
        "ARMED".to_string()
    } else if sim.sticks.arm {
        format!("DISARMED ({}) - flip arm switch off, then on", blockers_text(st.blockers))
    } else {
        format!("DISARMED ({})", blockers_text(st.blockers))
    };
    let fs = match st.failsafe {
        FailsafeStage::None => "",
        FailsafeStage::Hold => "  !! FAILSAFE: HOLD",
        FailsafeStage::Landing => "  !! FAILSAFE: LANDING",
    };
    let batt = match (st.battery_volts, st.battery_level) {
        (None, _) => "--".to_string(),
        (Some(v), BatteryLevel::Ok) => format!("{v:.2} V"),
        (Some(v), BatteryLevel::Warning) => format!("{v:.2} V LOW"),
        (Some(v), BatteryLevel::Critical) => format!("{v:.2} V CRITICAL"),
    };
    let (r, p, y) = w.body.att.to_euler();
    let (er, ep, _) = st.attitude.to_euler();
    let faults: Vec<String> = sim
        .sc
        .faults
        .iter()
        .filter(|f| f.active(sim.t_us))
        .map(|f| match f.kind {
            FaultKind::Kick { .. } => "kick".to_string(),
            k => k.name().to_string(),
        })
        .collect();
    let src = match app.replay {
        Some(s) => format!("DST replay seed {s}\n{}", sim.sc.name.replace(',', " ")),
        None if app.auto_faults => format!("manual flight, auto faults ({} injected)", app.auto_faults_injected),
        None => "manual flight".to_string(),
    };
    let speed = if app.paused { "PAUSED".to_string() } else { format!("{}x", app.time_scale) };
    if let Ok(mut t) = left.single_mut() {
        t.0 = format!(
            "IronLoop firmware - {src}\n\
             t {:.1}s  {speed}   camera {:?}\n\
             {mode}  {arm}{fs}{}\n\
             battery {batt}  {:.2} A   throttle in {:.0}%  out {:.0}%\n\
             alt {:.2} m  speed {:.1} m/s  dist {:.1} m\n\
             attitude true  R{:>5.0} P{:>5.0} Y{:>5.0}\n\
             attitude est   R{:>5.0} P{:>5.0}\n\
             gyro {:>6.0} {:>6.0} {:>6.0} deg/s  setpoint {:>6.0} {:>6.0} {:>6.0}\n\
             motors {:?}{}\n\
             sticks R{:+.2} P{:+.2} Y{:+.2} T{:.2}  switch {}  LED {} beeper {}\n\
             faults: {}",
            sim.t_us as f64 * 1e-6,
            app.cam,
            if st.airmode_active && st.armed { "  airmode" } else { "" },
            w.battery.amps,
            sim.sticks.throttle * 100.0,
            st.throttle * 100.0,
            w.altitude(),
            w.body.vel.norm(),
            FVec3::new(w.body.pos.x, w.body.pos.y, 0.0).norm(),
            r.to_degrees(),
            p.to_degrees(),
            y.to_degrees(),
            er.to_degrees(),
            ep.to_degrees(),
            st.gyro.x.to_degrees(),
            st.gyro.y.to_degrees(),
            st.gyro.z.to_degrees(),
            st.rate_setpoint.x.to_degrees(),
            st.rate_setpoint.y.to_degrees(),
            st.rate_setpoint.z.to_degrees(),
            st.motors,
            if st.motor_faults.iter().any(|&f| f) { format!("  FAULT {:?}", st.motor_faults) } else { String::new() },
            sim.sticks.roll,
            sim.sticks.pitch,
            sim.sticks.yaw,
            sim.sticks.throttle,
            if sim.sticks.arm { "ON" } else { "off" },
            if sim.led { "on" } else { "off" },
            if sim.beeper { "on" } else { "off" },
            if faults.is_empty() { "none".to_string() } else { faults.join(", ") },
        );
    }
    if let Ok(mut t) = right.single_mut() {
        let mut lines: Vec<String> = sim
            .notices
            .iter()
            .rev()
            .filter(|(_, n)| !matches!(n, Notice::ImuSampleRejected | Notice::RcFrameRejected))
            .take(12)
            .map(|(t, n)| match n {
                Notice::ArmingBlocked(b) => format!("{:>7.2}s ArmingBlocked: {}", *t as f64 * 1e-6, blockers_text(*b)),
                n => format!("{:>7.2}s {n:?}", *t as f64 * 1e-6),
            })
            .collect();
        lines.reverse();
        let rejected = sim
            .notices
            .iter()
            .filter(|(_, n)| matches!(n, Notice::ImuSampleRejected | Notice::RcFrameRejected))
            .count();
        let mut out = format!("firmware notices ({rejected} bad samples rejected)\n{}", lines.join("\n"));
        if app.replay.is_some() {
            out.push_str("\n\nproperty monitor: ");
            if sim.monitor.violations.is_empty() {
                out.push_str("all properties hold");
            } else {
                for v in &sim.monitor.violations {
                    out.push_str(&format!("\nVIOLATION {} @{:.2}s {}", v.property, v.t_us as f64 * 1e-6, v.detail));
                }
            }
            if sim.done() {
                out.push_str("\n(replay finished: N next seed, B manual)");
            }
        }
        t.0 = out;
    }
}
