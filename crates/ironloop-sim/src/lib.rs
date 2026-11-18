//! Deterministic simulation testing (DST) for the IronLoop firmware.
//!
//! [`Sim`] drives the real firmware `step` function with timestamped events
//! from a simulated Mobula6: rigid-body physics, motors and ESCs, a sagging
//! battery, wind, a noisy IMU, an RC link, and a virtual pilot. A
//! [`Scenario`] generated from a seed varies all of it and injects faults; the
//! [`Monitor`] checks the safety properties and flight expectations listed in
//! [`monitor::PROPERTIES`] on every step. The same seed always produces the
//! same run, bit for bit.

pub mod monitor;
pub mod physics;
pub mod pilot;
pub mod rng;
pub mod scenario;
pub mod sim;

pub use monitor::{Monitor, Property, Violation};
pub use scenario::{Action, FOREVER, Fault, FaultKind, Scenario};
pub use pilot::Sticks;
pub use sim::Sim;

/// Result of one seeded run.
#[derive(Clone, Debug)]
pub struct RunReport {
    pub seed: u64,
    pub name: String,
    pub violations: Vec<Violation>,
    pub trace_hash: u64,
    pub stats: monitor::Stats,
    pub events: u64,
    pub sim_us: u64,
}

pub fn run_scenario(sc: Scenario) -> RunReport {
    let seed = sc.seed;
    let name = sc.name.clone();
    let mut sim = Sim::new(sc);
    // A panic in the firmware (overflow checks are on in release for the
    // firmware core) is a failure of this seed, not of the whole sweep.
    let ran = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| sim.run()));
    let panic = ran.err().map(|e| e.downcast_ref::<&str>().map(|s| s.to_string()).or_else(|| e.downcast_ref::<String>().cloned()).unwrap_or_default());
    sim.monitor.on_end(sim.t_us, panic);
    RunReport {
        seed,
        name,
        violations: sim.monitor.violations.clone(),
        trace_hash: sim.trace_hash,
        stats: sim.monitor.stats.clone(),
        events: sim.events_delivered,
        sim_us: sim.t_us,
    }
}

pub fn run_seed(seed: u64) -> RunReport {
    run_scenario(Scenario::generate(seed))
}
