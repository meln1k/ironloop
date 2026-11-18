//! Deterministic simulation test runner.
//!
//! ```text
//! dst [--seeds N] [--start S] [--seed X] [--verbose] [--properties] [--list-failures]
//! ```
//!
//! Runs `N` generated scenarios starting at seed `S` on all cores, checks
//! every property, re-runs a sample to confirm the traces are bit-identical,
//! and exits non-zero on any violation. Sweeps of 200 seeds or more also
//! fail when coverage is too thin: a property whose precondition was never
//! live in at least two seeds, a fault kind never injected in flight, or
//! less than a quarter of the simulated time spent flying. `--seed X`
//! replays one seed with its full scenario, notices, and stats.
//! `--list-failures` also prints every failing seed, one
//! `failure <seed> <property> <time_us> [<workload>] <detail>` line each,
//! for scripts.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use ironloop_sim::monitor::PROPERTIES;
use ironloop_sim::{FaultKind, RunReport, Scenario, Sim, run_seed};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let arg = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).map(|v| v.parse::<u64>().expect("number"));
    let verbose = args.iter().any(|a| a == "--verbose");

    if args.iter().any(|a| a == "--properties") {
        for p in PROPERTIES {
            println!("{:28} {}", p.name, p.doc);
        }
        return;
    }

    if let Some(seed) = arg("--seed") {
        let sc = Scenario::generate(seed);
        println!("seed {seed}: {}", sc.name);
        if verbose {
            println!("{sc:#?}");
        }
        let mut sim = Sim::new(sc);
        if args.iter().any(|a| a == "--trace") {
            // 20 Hz flight log: time, segment, armed, mode, altitude, speed,
            // true and estimated tilt, throttle, motors, battery, impact.
            println!("    t  action                   arm  alt    vz    spd  tilt  est  thr   motors                 volts  g");
            while !sim.done() {
                let t = sim.t_us + 50_000;
                sim.run_until(t);
                let w = &sim.world;
                let st = &sim.status;
                let seg = sim.sc.segment_at(sim.t_us).map(|s| format!("{:?}", s.action)).unwrap_or_default();
                println!(
                    "{:6.2} {:24.24} {:3} {:5.2} {:5.2} {:5.2} {:5.0} {:4.0} {:4.2} {:?} {:4.2} {:4.1}",
                    sim.t_us as f64 * 1e-6,
                    seg,
                    if st.armed { if st.mode == ironloop_fc::FlightMode::Acro { "ACR" } else { "ANG" } } else { "-" },
                    w.altitude(),
                    -w.body.vel.z,
                    w.body.vel.norm(),
                    w.body.att.tilt().to_degrees(),
                    st.attitude.tilt().to_degrees(),
                    st.throttle,
                    st.motors,
                    w.battery.volts,
                    w.body.impact / 9.81,
                );
            }
        }
        sim.run();
        for (t, n) in &sim.notices {
            if verbose || !matches!(n, ironloop_fc::Notice::ImuSampleRejected | ironloop_fc::Notice::RcFrameRejected) {
                println!("  {:>8.3}s  {n:?}", *t as f64 * 1e-6);
            }
        }
        println!("stats: {:?}", sim.monitor.stats);
        println!("trace hash {:016x}, {} events", sim.trace_hash, sim.events_delivered);
        for v in &sim.monitor.violations {
            println!("VIOLATION {} at {:.3}s: {}", v.property, v.t_us as f64 * 1e-6, v.detail);
        }
        std::process::exit(if sim.monitor.violations.is_empty() { 0 } else { 1 });
    }

    let n = arg("--seeds").unwrap_or(200);
    let start = arg("--start").unwrap_or(0);
    let next = Arc::new(AtomicU64::new(start));
    let reports = Arc::new(Mutex::new(Vec::<RunReport>::new()));
    let threads = std::thread::available_parallelism().map_or(2, |n| n.get());
    let began = std::time::Instant::now();
    let handles: Vec<_> = (0..threads)
        .map(|_| {
            let next = next.clone();
            let reports = reports.clone();
            std::thread::spawn(move || {
                loop {
                    let seed = next.fetch_add(1, Ordering::Relaxed);
                    if seed >= start + n {
                        break;
                    }
                    let r = run_seed(seed);
                    reports.lock().unwrap().push(r);
                }
            })
        })
        .collect();
    for h in handles {
        h.join().unwrap();
    }
    let mut reports = Arc::try_unwrap(reports).unwrap().into_inner().unwrap();
    reports.sort_by_key(|r| r.seed);

    // Determinism: replay a sample and compare full-trace hashes.
    let mut nondeterministic = Vec::new();
    for r in reports.iter().step_by((reports.len() / 8).max(1)) {
        let again = run_seed(r.seed);
        if again.trace_hash != r.trace_hash {
            nondeterministic.push(r.seed);
        }
    }

    let mut by_prop: std::collections::BTreeMap<&str, Vec<u64>> = Default::default();
    let mut tags: std::collections::BTreeMap<String, u32> = Default::default();
    let mut sim_s = 0.0;
    let mut events = 0;
    for r in &reports {
        sim_s += r.sim_us as f64 * 1e-6;
        events += r.events;
        for t in r.name.split(',') {
            *tags.entry(t.to_string()).or_default() += 1;
        }
        for v in &r.violations {
            by_prop.entry(v.property).or_default().push(r.seed);
            if verbose || by_prop[v.property].len() <= 3 {
                println!(
                    "FAIL seed {:>5} {:<26} at {:>7.3}s: {}  [{}]",
                    r.seed,
                    v.property,
                    v.t_us as f64 * 1e-6,
                    v.detail,
                    r.name
                );
            }
        }
    }
    if args.iter().any(|a| a == "--list-failures") {
        for r in &reports {
            for v in &r.violations {
                println!("failure {} {} {} [{}] {}", r.seed, v.property, v.t_us, r.name, v.detail);
            }
        }
    }
    println!();
    println!("workload coverage ({} scenarios):", reports.len());
    let line: Vec<String> = tags.iter().map(|(t, c)| format!("{t}:{c}")).collect();
    println!("  {}", line.join("  "));
    // Coverage: how many seeds exercised each property and each fault kind
    // in flight, and how much of the simulated time was flying.
    let mut live_seeds: std::collections::BTreeMap<&str, u32> = Default::default();
    let mut fault_seeds: std::collections::BTreeMap<&str, u32> = Default::default();
    let mut airborne_s = 0.0;
    for r in &reports {
        airborne_s += r.stats.airborne_us as f64 * 1e-6;
        for (p, n) in &r.stats.live {
            if *n > 0 {
                *live_seeds.entry(p).or_default() += 1;
            }
        }
        let mut kinds = r.stats.faults_in_flight.clone();
        kinds.sort();
        kinds.dedup();
        for k in kinds {
            *fault_seeds.entry(k).or_default() += 1;
        }
    }
    let gate = reports.len() >= 200;
    let mut thin = Vec::new();
    println!("  airborne {:.0}% of simulated time", 100.0 * airborne_s / sim_s.max(1e-9));
    if gate && airborne_s < 0.25 * sim_s {
        thin.push(format!("airborne only {:.0}% of the time", 100.0 * airborne_s / sim_s));
    }
    // Every fault kind must happen in flight in a sweep.
    let line: Vec<String> = FaultKind::ALL.iter().map(|k| format!("{}:{}", k.name(), fault_seeds.get(k.name()).copied().unwrap_or(0))).collect();
    println!("  faults started in flight (seeds): {}", line.join("  "));
    for k in FaultKind::ALL {
        if gate && fault_seeds.get(k.name()).copied().unwrap_or(0) == 0 {
            thin.push(format!("fault {} never injected in flight", k.name()));
        }
    }
    for k in fault_seeds.keys().filter(|k| !FaultKind::ALL.iter().any(|a| a.name() == **k)) {
        thin.push(format!("fault {k} missing from FaultKind::ALL"));
    }
    let worst_est = reports.iter().map(|r| r.stats.max_est_err_deg).fold(0.0, f32::max);
    let worst_tilt = reports.iter().map(|r| r.stats.max_tilt_angle_mode_deg).fold(0.0, f32::max);
    println!("  worst estimate error {worst_est:.1} deg, worst angle-mode tilt {worst_tilt:.1} deg");
    println!(
        "simulated {:.0} s of flight ({events} firmware steps) in {:.1} s wall",
        sim_s,
        began.elapsed().as_secs_f64()
    );
    println!("properties ({}):", PROPERTIES.len());
    for p in PROPERTIES {
        let name = &p.name;
        let live = live_seeds.get(name).copied().unwrap_or(0);
        match by_prop.get(name) {
            None => println!("  ok    {name:30} live in {live} seeds"),
            Some(seeds) => println!("  FAIL  {name}  ({} seeds, e.g. {:?})", seeds.len(), &seeds[..seeds.len().min(6)]),
        }
        if gate && live < 2 {
            thin.push(format!("property {name} live in only {live} seeds"));
        }
    }
    if nondeterministic.is_empty() {
        println!("  ok    deterministic_replay");
    } else {
        println!("  FAIL  deterministic_replay {nondeterministic:?}");
    }
    for t in &thin {
        println!("  THIN  {t}");
    }
    let failed = !by_prop.is_empty() || !nondeterministic.is_empty() || !thin.is_empty();
    std::process::exit(if failed { 1 } else { 0 });
}
