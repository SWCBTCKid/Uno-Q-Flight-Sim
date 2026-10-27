//! m1-fdm-bench: the M1-001 harness (Flight Dynamics, ORG-019).
//!
//! The parent process only orchestrates: it samples clocks/temperatures from
//! sysfs, runs every case repetition in a fresh child process (its own ASLR,
//! allocator state and file cache use), and writes one JSON row per case
//! repetition in the M0-003 envelope (kind, run_id, test, params, ..., clocks,
//! temps_max, ok, error) plus the contract's interface additions
//! (docs/verification/M1-001-fdm-plan.md, TEST-009). The child pins itself to
//! one core, drives JSBSim through the Rust -> gx_fdm shim path, writes per-step
//! raw rows and prints one `GXRESULT {...members...}` line.
//!
//! Usage (the board runner appends --out and --raw-dir; the deployer prepends
//! --run-id):
//!   m1-fdm-bench --case test=step,ac=global5000,rate=120,mode=paced,reps=3 [--case ...]
//! Case keys: test = step | alloc | loadreset | ffi_get | exceptions | idle |
//!   standalone | cxx_ffi; ac = global5000 | c172p; rate (Hz); mode = paced |
//!   free; script = flight | stall; reps; after_reset=1; log=1 (standalone);
//!   n (ffi_get calls); dur_s=N (measure the first N sim seconds; step, alloc,
//!   standalone, cxx_ffi); mem=1 (1 Hz RSS/heap series); traj=1 (30 Hz and
//!   ground trajectory TSVs, off by default since TEST-010); test-only injections inject_alloc=1, inject_slow=N,
//!   inject_perturb=STEP (recorded in params; the analyzer refuses such rows as
//!   final evidence).
mod json;
mod plat;

use gx_fdm::{Fdm, Prop, Step};
use json::{s, J};
use std::collections::BTreeMap;
use std::collections::VecDeque;
use std::fmt::Write as _;
use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

/// Contract output set (interface item 4), in this order everywhere: the
/// per-step hash, the 30 Hz rows and the standalone output directive.
const OUTPUT_SET: [&str; 20] = [
    "position/lat-geod-deg",
    "position/long-gc-deg",
    "position/h-agl-ft",
    "velocities/u-fps",
    "velocities/v-fps",
    "velocities/w-fps",
    "velocities/p-rad_sec",
    "velocities/q-rad_sec",
    "velocities/r-rad_sec",
    "attitude/phi-rad",
    "attitude/theta-rad",
    "attitude/psi-rad",
    "velocities/vc-kts",
    "velocities/h-dot-fps",
    "gear/unit[0]/compression-ft",
    "gear/unit[1]/compression-ft",
    "gear/unit[2]/compression-ft",
    "gear/unit[0]/WOW",
    "gear/unit[1]/WOW",
    "gear/unit[2]/WOW",
];
const FLIGHT_SEGMENTS: [&str; 10] = [
    "",
    "parked",
    "taxi",
    "takeoff_roll",
    "rotate",
    "liftoff",
    "circuit",
    "final",
    "touchdown",
    "rollout_end",
];
const STALL_SEGMENTS: [&str; 4] = ["", "stall_config", "decel", "stall_end"];
/// Control input perturbed by --inject-perturb (1 ulp toward zero).
// Script-owned test-only control sentinel. A one-ULP change is amplified by a
// continuous script event into a rudder command in the same step. Injection
// off (the default 1.0) does not touch any flight control.
const PERTURB_PROP: &str = "gx/inject-control";
const DEFAULT_MALLOC_DEBUG: &str = "/usr/lib/aarch64-linux-gnu/libc_malloc_debug.so.0";

// ------------------------------------------------------------------ args ----

#[derive(Clone, Debug, Default)]
struct Case(Vec<(String, String)>);

impl Case {
    fn parse(spec: &str) -> Result<Case, String> {
        let mut v = Vec::new();
        for kv in spec.split(',').filter(|x| !x.is_empty()) {
            let (k, val) = kv
                .split_once('=')
                .ok_or_else(|| format!("bad case item '{kv}' (want key=value)"))?;
            v.push((k.trim().to_string(), val.trim().to_string()));
        }
        Ok(Case(v))
    }
    fn get(&self, k: &str) -> Option<&str> {
        self.0
            .iter()
            .rev()
            .find(|(a, _)| a == k)
            .map(|(_, b)| b.as_str())
    }
    fn str_or<'a>(&'a self, k: &str, d: &'a str) -> &'a str {
        self.get(k).unwrap_or(d)
    }
    fn num<T: std::str::FromStr>(&self, k: &str, d: T) -> Result<T, String> {
        match self.get(k) {
            None => Ok(d),
            Some(x) => x
                .parse()
                .map_err(|_| format!("case key {k}: bad number '{x}'")),
        }
    }
    fn spec(&self) -> String {
        self.0
            .iter()
            .map(|(a, b)| format!("{a}={b}"))
            .collect::<Vec<_>>()
            .join(",")
    }
    fn json(&self) -> J {
        J::O(
            self.0
                .iter()
                .map(|(a, b)| (a.clone(), s(b.clone())))
                .collect(),
        )
    }
    fn injected(&self) -> bool {
        ["inject_alloc", "inject_slow", "inject_perturb"]
            .iter()
            .any(|k| self.get(k).is_some())
    }
}

#[derive(Clone, Debug)]
struct Ctx {
    run_id: String,
    out: Option<PathBuf>,
    raw_dir: PathBuf,
    stage: PathBuf,
    jsbsim_root: PathBuf,
    scripts_dir: PathBuf,
    bin_dir: PathBuf,
    core: usize,
    sampler_core: Option<usize>,
    malloc_debug: String,
}

fn arg_values(args: &[String], name: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < args.len() {
        if args[i] == name && i + 1 < args.len() {
            out.push(args[i + 1].clone());
            i += 2;
        } else {
            i += 1;
        }
    }
    out
}

fn arg_one(args: &[String], name: &str) -> Option<String> {
    arg_values(args, name).pop()
}

fn ctx_from(args: &[String]) -> Result<Ctx, String> {
    let exe = std::env::current_exe().map_err(|e| format!("current_exe: {e}"))?;
    let bin_dir = exe
        .parent()
        .map(Path::to_path_buf)
        .unwrap_or_else(|| PathBuf::from("."));
    let stage = arg_one(args, "--stage")
        .map(PathBuf::from)
        .unwrap_or_else(|| bin_dir.join(".."));
    // Absolute paths: JSBSim joins a relative initfile to the aircraft directory.
    let abs = |p: PathBuf| std::path::absolute(&p).unwrap_or(p);
    let stage = abs(stage);
    Ok(Ctx {
        run_id: arg_one(args, "--run-id").unwrap_or_else(|| "local".into()),
        out: arg_one(args, "--out").map(PathBuf::from),
        raw_dir: abs(arg_one(args, "--raw-dir")
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from("raw"))),
        jsbsim_root: abs(arg_one(args, "--jsbsim-root")
            .map(PathBuf::from)
            .unwrap_or_else(|| stage.join("jsbsim"))),
        scripts_dir: abs(arg_one(args, "--scripts-dir")
            .map(PathBuf::from)
            .unwrap_or_else(|| stage.join("tests/m1/scripts"))),
        bin_dir: abs(arg_one(args, "--bin-dir")
            .map(PathBuf::from)
            .unwrap_or(bin_dir)),
        stage,
        core: arg_one(args, "--core")
            .map(|c| c.parse().unwrap_or(1))
            .unwrap_or(1),
        sampler_core: Some(
            arg_one(args, "--sampler-core")
                .map(|c| c.parse().unwrap_or(0))
                .unwrap_or(0),
        ),
        malloc_debug: arg_one(args, "--malloc-debug-lib")
            .unwrap_or_else(|| DEFAULT_MALLOC_DEBUG.into()),
    })
}

fn pass_through_args(ctx: &Ctx) -> Vec<String> {
    vec![
        "--run-id".into(),
        ctx.run_id.clone(),
        "--raw-dir".into(),
        ctx.raw_dir.to_string_lossy().into(),
        "--stage".into(),
        ctx.stage.to_string_lossy().into(),
        "--jsbsim-root".into(),
        ctx.jsbsim_root.to_string_lossy().into(),
        "--scripts-dir".into(),
        ctx.scripts_dir.to_string_lossy().into(),
        "--bin-dir".into(),
        ctx.bin_dir.to_string_lossy().into(),
        "--core".into(),
        ctx.core.to_string(),
    ]
}

// --------------------------------------------------------------- helpers ----

/// sha256 of the deployed files as verified by the runner (`files.sha256`,
/// checked with `sha256sum -c` before the benchmark starts).
fn stage_hashes(stage: &Path) -> BTreeMap<String, String> {
    let mut m = BTreeMap::new();
    if let Ok(t) = std::fs::read_to_string(stage.join("files.sha256")) {
        for l in t.lines() {
            if let Some((h, p)) = l.split_once("  ") {
                m.insert(p.trim().to_string(), h.trim().to_string());
            }
        }
    }
    m
}

fn binary_hashes(h: &BTreeMap<String, String>) -> J {
    let g = |k: &str| h.get(k).map(|v| s(v.clone())).unwrap_or(J::Null);
    J::O(vec![
        ("m1-fdm-bench".into(), g("bin/m1-fdm-bench")),
        ("libgx_fdm.so".into(), g("bin/libgx_fdm.so")),
        ("libJSBSim.so".into(), g("bin/libJSBSim.so.1")),
        ("JSBSim".into(), g("bin/JSBSim")),
    ])
}

#[inline]
fn fnv1a(vals: &[f64]) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    for v in vals {
        for b in v.to_bits().to_le_bytes() {
            h ^= b as u64;
            h = h.wrapping_mul(0x100000001b3);
        }
    }
    h
}

fn stream_hash(hs: &[u64]) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    for x in hs {
        for b in x.to_le_bytes() {
            h ^= b as u64;
            h = h.wrapping_mul(0x100000001b3);
        }
    }
    h
}

fn pct(sorted: &[u64], p: f64) -> u64 {
    if sorted.is_empty() {
        return 0;
    }
    // nearest-rank
    let r = ((p / 100.0) * sorted.len() as f64).ceil() as usize;
    sorted[r.clamp(1, sorted.len()) - 1]
}

fn dist(v: &[u64]) -> J {
    let mut a = v.to_vec();
    a.sort_unstable();
    let mean = if a.is_empty() {
        0.0
    } else {
        a.iter().map(|x| *x as f64).sum::<f64>() / a.len() as f64
    };
    J::O(vec![
        ("n".into(), J::U(a.len() as u64)),
        ("p50".into(), J::U(pct(&a, 50.0))),
        ("p99".into(), J::U(pct(&a, 99.0))),
        ("p999".into(), J::U(pct(&a, 99.9))),
        ("max".into(), J::U(a.last().copied().unwrap_or(0))),
        ("min".into(), J::U(a.first().copied().unwrap_or(0))),
        ("mean".into(), J::F(mean)),
    ])
}

fn toward_zero_ulp(v: f64) -> f64 {
    if v == 0.0 {
        f64::from_bits(1)
    } else {
        f64::from_bits(v.to_bits() - 1)
    }
}

fn script_paths(ctx: &Ctx, ac: &str, kind: &str) -> Result<(PathBuf, PathBuf), String> {
    match kind {
        "flight" => Ok((
            ctx.scripts_dir.join(format!("{ac}_flight.xml")),
            ctx.scripts_dir.join(format!("ic_runway_{ac}.xml")),
        )),
        "stall" if ac == "global5000" => Ok((
            ctx.scripts_dir.join("global5000_stall.xml"),
            ctx.scripts_dir.join("ic_air_global5000.xml"),
        )),
        _ => Err(format!("no script '{kind}' for aircraft '{ac}'")),
    }
}

fn rel_script(p: &Path) -> String {
    format!(
        "tests/m1/scripts/{}",
        p.file_name()
            .map(|f| f.to_string_lossy().into_owned())
            .unwrap_or_default()
    )
}

/// Disables output directives the aircraft file itself declares (global5000
/// ships an active 60 Hz CSV output), so no run writes files from inside a
/// step. Uses only the public property API.
fn disable_outputs(fdm: &mut Fdm) -> u32 {
    let mut n = 0;
    for i in 0..8 {
        let name = if i == 0 {
            "simulation/output/enabled".to_string()
        } else {
            format!("simulation/output[{i}]/enabled")
        };
        match fdm.bind(&name) {
            Ok(p) => {
                fdm.set(p, 0.0);
                n += 1;
            }
            Err(_) => break,
        }
    }
    n
}

fn ms(t0: u64, t1: u64) -> f64 {
    (t1.saturating_sub(t0)) as f64 / 1e6
}

fn emit(fields: J, window: Option<(u64, u64)>) {
    if let Some((a, b)) = window {
        println!("GXWINDOW {a} {b}");
    }
    println!("GXRESULT {}", fields.members());
    let _ = std::io::stdout().flush();
}

// ----------------------------------------------------------- child: flight ----

struct Loaded {
    fdm: Fdm,
    outs: Vec<Prop>,
    seg: Option<Prop>,
    simdt: Prop,
    simtime: Prop,
    create_ms: f64,
    load_ms: f64,
    run_ic_ms: f64,
    outputs_disabled: u32,
}

fn load_flight(ctx: &Ctx, script: &Path, ic: &Path, dt: f64) -> Result<Loaded, String> {
    let t0 = plat::now_ns();
    let mut fdm = Fdm::create(&ctx.jsbsim_root).map_err(|e| e.to_string())?;
    let t1 = plat::now_ns();
    fdm.load_script(script, dt, Some(ic))
        .map_err(|e| e.to_string())?;
    let outputs_disabled = disable_outputs(&mut fdm);
    let t2 = plat::now_ns();
    fdm.run_ic().map_err(|e| e.to_string())?;
    let t3 = plat::now_ns();
    let mut outs = Vec::with_capacity(OUTPUT_SET.len());
    for n in OUTPUT_SET {
        outs.push(fdm.bind(n).map_err(|e| e.to_string())?);
    }
    let seg = fdm.bind("gx/seg").ok();
    let simdt = fdm.bind("simulation/dt").map_err(|e| e.to_string())?;
    let simtime = fdm
        .bind("simulation/sim-time-sec")
        .map_err(|e| e.to_string())?;
    Ok(Loaded {
        fdm,
        outs,
        seg,
        simdt,
        simtime,
        create_ms: ms(t0, t1),
        load_ms: ms(t1, t2),
        run_ic_ms: ms(t2, t3),
        outputs_disabled,
    })
}

fn child_flight(ctx: &Ctx, c: &Case, rep: u32) -> Result<(), String> {
    let test = c.str_or("test", "step").to_string();
    let ac = c.str_or("ac", "global5000").to_string();
    let rate: u32 = c.num("rate", 120)?;
    let mode = c.str_or("mode", "paced").to_string();
    let kind = c.str_or("script", "flight").to_string();
    let after_reset = c.str_or("after_reset", "0") == "1";
    let inject_alloc = c.str_or("inject_alloc", "0") == "1";
    let inject_slow: u64 = c.num("inject_slow", 0)?;
    let inject_perturb: i64 = c.num("inject_perturb", -1)?;
    let max_s: f64 = c.num("max_sim_s", 1600.0)?;
    // TEST-010 (performance only): dur_s=N measures the first N simulated
    // seconds of the script (a complete, valid row; script_done stays false).
    let dur_s: f64 = c.num("dur_s", 0.0)?;
    // Trajectory outputs (30 Hz and native-rate ground TSVs) served the dropped
    // stability/determinism gates; off unless traj=1.
    let traj = c.str_or("traj", "0") == "1";
    // mem=1: 1 Hz RSS / glibc heap samples between steps (allocation-free).
    let mem = c.str_or("mem", "0") == "1";
    if mode != "paced" && mode != "free" {
        return Err(format!("mode must be paced or free, got {mode}"));
    }
    if rate == 0 || rate % 30 != 0 {
        return Err(format!("rate {rate} is not a multiple of 30 Hz"));
    }
    let (script, ic) = script_paths(ctx, &ac, &kind)?;
    let pin = plat::pin_to(ctx.core);
    let affinity = plat::affinity();
    let dt = 1.0 / rate as f64;
    let period_ns = 1_000_000_000u64 / rate as u64; // for budgets; slots use exact rational below
    let mut l = load_flight(ctx, &script, &ic, dt)?;
    let jsbsim_dt = l.fdm.get(l.simdt);
    let perturb = if inject_perturb >= 0 {
        Some(l.fdm.bind(PERTURB_PROP).map_err(|e| e.to_string())?)
    } else {
        None
    };
    let seg_names: &[&str] = if kind == "stall" {
        &STALL_SEGMENTS
    } else {
        &FLIGHT_SEGMENTS
    };

    // Preallocate everything the loop touches.
    let dur_steps = (dur_s * rate as f64).round() as usize;
    let n_max = if dur_steps > 0 {
        dur_steps + 16
    } else {
        (max_s * rate as f64) as usize + 16
    };
    let mut mem_rows: Vec<[u64; 5]> =
        Vec::with_capacity(if mem { n_max / rate as usize + 4 } else { 0 });
    let dec = (rate / 30) as usize;
    let mut slot = vec![0u64; n_max];
    let mut st = vec![0u64; n_max];
    let mut en = vec![0u64; n_max];
    let mut hs = vec![0u64; n_max];
    let mut sg = vec![0u8; n_max];
    let mut simt = vec![0f64; n_max];
    // vec![0; n] is calloc'd: its pages are only faulted in as the loop
    // writes them, which shows up as ~41 B/step of RSS growth that is the
    // harness, not JSBSim (found in the first 10-minute run). Touch every
    // page now so the long-run RSS series measures JSBSim alone.
    plat::prefault(&mut slot);
    plat::prefault(&mut st);
    plat::prefault(&mut en);
    plat::prefault(&mut hs);
    plat::prefault(&mut sg);
    plat::prefault(&mut simt);
    let tcap = |x: usize| if traj { x } else { 0 };
    let mut out30: Vec<f64> = Vec::with_capacity(tcap((n_max / dec + 2) * (OUTPUT_SET.len() + 2)));
    // Full native-rate values for ground segments. The ring supplies the five
    // seconds immediately before touchdown without retaining the full flight.
    let mut ground: Vec<[f64; 22]> = Vec::with_capacity(tcap((180 * rate) as usize));
    let mut touchdown_ring: VecDeque<[f64; 22]> =
        VecDeque::with_capacity(tcap((5 * rate + 1) as usize));
    let mut touchdown_flushed = false;
    let mut vals = [0f64; 20];
    let mut events: Vec<(u8, usize, f64)> = Vec::with_capacity(32);
    let mut prev_seg = 0u8;
    let mut n = 0usize;
    let mut done = false;
    let mut err: Option<String> = None;
    let mut perturbed: Option<(f64, f64)> = None;

    let mtrace = if test == "alloc" {
        plat::mtrace_on()
    } else {
        false
    };
    let t_start = plat::now_ns() + 2_000_000;
    let paced = mode == "paced";
    loop {
        if n >= n_max {
            err = Some(format!("script did not finish within {n_max} steps"));
            break;
        }
        let s_k = t_start + (n as u128 * 1_000_000_000u128 / rate as u128) as u64;
        if paced {
            plat::sleep_until_ns(s_k);
        }
        if let Some(p) = perturb {
            if n as i64 == inject_perturb {
                let v = l.fdm.get(p);
                let nv = toward_zero_ulp(v);
                l.fdm.set(p, nv);
                perturbed = Some((v, nv));
            }
        }
        if mtrace {
            plat::mtrace_marker(); // step n+1 starts here in the trace
        }
        let a = plat::now_ns();
        let r = l.fdm.step();
        if inject_alloc {
            std::hint::black_box(Vec::<u8>::with_capacity(64));
        }
        // Step one makes every-100 strictly more than one percent, so a
        // nearest-rank p99 positive control must move above the threshold.
        if inject_slow > 0 && (n == 0 || (n as u64 + 1) % inject_slow == 0) {
            while plat::now_ns() < a + period_ns / 2 {
                std::hint::spin_loop();
            }
        }
        let b = plat::now_ns();
        slot[n] = s_k;
        st[n] = a;
        en[n] = b;
        for (i, p) in l.outs.iter().enumerate() {
            vals[i] = l.fdm.get(*p);
        }
        hs[n] = fnv1a(&vals);
        let segv = l.seg.map(|p| l.fdm.get(p) as u8).unwrap_or(0);
        sg[n] = segv;
        let idx = n + 1; // steps executed; sim time = idx * dt
        let sim_now = l.fdm.get(l.simtime);
        simt[n] = sim_now;
        if traj {
            let mut full = [0.0; 22];
            full[0] = idx as f64;
            full[1] = sim_now;
            full[2..].copy_from_slice(&vals);
            if (1..=4).contains(&segv) {
                ground.push(full);
            } else if segv >= 8 {
                if !touchdown_flushed {
                    ground.extend(touchdown_ring.drain(..));
                    touchdown_flushed = true;
                }
                ground.push(full);
            } else {
                touchdown_ring.push_back(full);
                while touchdown_ring.len() > (5 * rate) as usize {
                    touchdown_ring.pop_front();
                }
            }
            if idx % dec == 0 {
                out30.push(idx as f64);
                out30.push(l.fdm.get(l.simtime));
                out30.extend_from_slice(&vals);
            }
        }
        if segv != prev_seg && events.len() < events.capacity() {
            events.push((segv, idx, sim_now));
            prev_seg = segv;
        }
        if mem && (idx == 1 || idx % rate as usize == 0) && mem_rows.len() < mem_rows.capacity() {
            let (rss, used, sys) = plat::mem_sample();
            mem_rows.push([idx as u64, plat::now_ns(), rss, used, sys]);
        }
        n += 1;
        if dur_steps > 0 && n >= dur_steps && r.is_ok() {
            done = true;
            break;
        }
        match r {
            Ok(Step::Running) => {}
            Ok(Step::Done) => {
                done = true;
                break;
            }
            Err(e) => {
                err = Some(format!("step {idx}: {e}"));
                break;
            }
        }
    }
    let t_end = plat::now_ns();
    if mtrace {
        plat::mtrace_off();
    }
    let sim_end = l.fdm.get(l.simtime);
    let (rss_peak, rss_now) = plat::rss_kib();

    // Optional: reset and replay in the same process (reported, not gated).
    let mut reset_j = J::Null;
    if after_reset && err.is_none() {
        let r0 = plat::now_ns();
        let rr = l.fdm.reset();
        let r1 = plat::now_ns();
        // FGFDMExec::ResetToInitialConditions restarts the script's events and
        // time but leaves simulation/terminate as the script set it (finding,
        // recorded in docs/fdm/m1-001-flight-dynamics.md); clear it to replay.
        let cleared = match l.fdm.bind("simulation/terminate") {
            Ok(p) => {
                let was = l.fdm.get(p);
                l.fdm.set(p, 0.0);
                Some(was)
            }
            Err(_) => None,
        };
        let mut first_div: Option<usize> = None;
        let mut m = 0usize;
        let mut rerr = rr.err().map(|e| e.to_string());
        if rerr.is_none() {
            loop {
                if m >= n_max {
                    rerr = Some("replay did not finish".into());
                    break;
                }
                let r = l.fdm.step();
                for (i, p) in l.outs.iter().enumerate() {
                    vals[i] = l.fdm.get(*p);
                }
                let h = fnv1a(&vals);
                if first_div.is_none() && (m >= n || h != hs[m]) {
                    first_div = Some(m + 1);
                }
                m += 1;
                match r {
                    Ok(Step::Running) => {}
                    Ok(Step::Done) => break,
                    Err(e) => {
                        rerr = Some(e.to_string());
                        break;
                    }
                }
            }
        }
        if first_div.is_none() && m != n {
            first_div = Some(m.min(n) + 1);
        }
        reset_j = J::O(vec![
            ("reset_ms".into(), J::F(ms(r0, r1))),
            (
                "terminate_after_reset".into(),
                cleared.map(J::F).unwrap_or(J::Null),
            ),
            ("n_steps".into(), J::U(m as u64)),
            ("identical".into(), J::Bool(first_div.is_none())),
            (
                "first_divergent_step".into(),
                first_div.map(|x| J::U(x as u64)).unwrap_or(J::Null),
            ),
            ("error".into(), rerr.map(s).unwrap_or(J::Null)),
        ]);
    }

    // Raw rows.
    std::fs::create_dir_all(&ctx.raw_dir).map_err(|e| format!("raw dir: {e}"))?;
    let tag = format!(
        "{}.{}.rep{}",
        ctx.run_id,
        c.get("name")
            .map(|x| x.to_string())
            .unwrap_or_else(|| format!("{test}-{ac}-{rate}-{mode}-{kind}")),
        rep
    );
    let steps_file = format!("{tag}.steps.tsv");
    let mem_file = format!("{tag}.mem.tsv");
    if mem {
        let mut t = String::from("step\tt_ns\trss_kib\theap_used_bytes\theap_system_bytes\n");
        for r in &mem_rows {
            let _ = writeln!(t, "{}\t{}\t{}\t{}\t{}", r[0], r[1], r[2], r[3], r[4]);
        }
        std::fs::create_dir_all(&ctx.raw_dir).map_err(|e| format!("raw dir: {e}"))?;
        std::fs::write(ctx.raw_dir.join(&mem_file), t).map_err(|e| format!("write mem: {e}"))?;
    }
    let out_file = format!("{tag}.out30.tsv");
    let ground_file = format!("{tag}.ground.tsv");
    {
        let mut t = String::with_capacity(n * 64 + 64);
        t.push_str("step\tslot_ns\tstart_ns\tend_ns\thash\tseg\tsim_time_s\n");
        for i in 0..n {
            let _ = writeln!(
                t,
                "{}\t{}\t{}\t{}\t{:016x}\t{}\t{}",
                i + 1,
                slot[i],
                st[i],
                en[i],
                hs[i],
                sg[i],
                simt[i]
            );
        }
        std::fs::write(ctx.raw_dir.join(&steps_file), t)
            .map_err(|e| format!("write steps: {e}"))?;
        if traj {
            let mut o = String::with_capacity(out30.len() * 20 + 256);
            o.push_str("step\tsim_time_s");
            for nme in OUTPUT_SET {
                o.push('\t');
                o.push_str(nme);
            }
            o.push('\n');
            for row in out30.chunks(OUTPUT_SET.len() + 2) {
                let _ = write!(o, "{}", row[0] as u64);
                for v in &row[1..] {
                    let _ = write!(o, "\t{v}");
                }
                o.push('\n');
            }
            std::fs::write(ctx.raw_dir.join(&out_file), o).map_err(|e| format!("write out30: {e}"))?;
            let mut g = String::with_capacity(ground.len() * 320 + 256);
            g.push_str("step\tsim_time_s");
            for nme in OUTPUT_SET {
                g.push('\t');
                g.push_str(nme);
            }
            g.push('\n');
            for row in &ground {
                let _ = write!(g, "{}", row[0] as u64);
                for v in &row[1..] {
                    let _ = write!(g, "\t{v}");
                }
                g.push('\n');
            }
            std::fs::write(ctx.raw_dir.join(&ground_file), g)
                .map_err(|e| format!("write ground: {e}"))?;
        }
    }

    // Statistics (all from the raw rows).
    let step_ns: Vec<u64> = (0..n).map(|i| en[i] - st[i]).collect();
    let late_ns: Vec<u64> = (0..n).map(|i| en[i].saturating_sub(slot[i])).collect();
    let sum: u64 = step_ns.iter().sum();
    let period_exact = 1e9 / rate as f64;
    let over: Vec<J> = (0..n)
        .filter(|i| step_ns[*i] as f64 > period_exact)
        .take(50)
        .map(|i| J::A(vec![J::U(i as u64 + 1), J::U(step_ns[i]), J::U(st[i])]))
        .collect();
    let n_over = (0..n).filter(|i| step_ns[*i] as f64 > period_exact).count();
    let misses = if paced {
        (0..n).filter(|i| late_ns[*i] as f64 > period_exact).count()
    } else {
        0
    };
    let ev: Vec<J> = events
        .iter()
        .map(|(sgv, i, t)| {
            J::O(vec![
                (
                    "event".into(),
                    s(seg_names.get(*sgv as usize).copied().unwrap_or("?")),
                ),
                ("seg".into(), J::U(*sgv as u64)),
                ("step".into(), J::U(*i as u64)),
                ("sim_time_s".into(), J::F(*t)),
            ])
        })
        .collect();
    let ok = err.is_none() && done;
    let mut params = vec![
        ("aircraft".to_string(), s(ac.clone())),
        ("rate_hz".into(), J::U(rate as u64)),
        ("mode".into(), s(mode.clone())),
        ("script_kind".into(), s(kind.clone())),
        ("core".into(), J::U(ctx.core as u64)),
    ];
    if inject_alloc {
        params.push(("inject_alloc_per_step".into(), J::Bool(true)));
    }
    if inject_slow > 0 {
        params.push(("inject_slow_every".into(), J::U(inject_slow)));
    }
    if inject_perturb >= 0 {
        params.push(("inject_perturb_step".into(), J::I(inject_perturb)));
        params.push(("inject_perturb_property".into(), s(PERTURB_PROP)));
        if let Some((a, b)) = perturbed {
            params.push(("inject_perturb_values".into(), J::A(vec![J::F(a), J::F(b)])));
        }
    }
    let f = J::O(vec![
        ("test".into(), s(test.clone())),
        ("rep".into(), J::U(rep as u64)),
        ("aircraft".into(), s(ac)),
        ("rate_hz".into(), J::U(rate as u64)),
        ("mode".into(), s(mode)),
        ("params".into(), J::O(params)),
        (
            "script".into(),
            J::O(vec![
                ("path".into(), s(rel_script(&script))),
                ("ic".into(), s(rel_script(&ic))),
            ]),
        ),
        ("jsbsim_dt".into(), J::F(jsbsim_dt)),
        ("dt_requested".into(), J::F(dt)),
        ("dt_ok".into(), J::Bool(jsbsim_dt.to_bits() == dt.to_bits())),
        (
            "affinity".into(),
            J::A(affinity.iter().map(|c| J::U(*c as u64)).collect()),
        ),
        ("pin_error".into(), pin.err().map(s).unwrap_or(J::Null)),
        ("outputs_disabled".into(), J::U(l.outputs_disabled as u64)),
        ("create_ms".into(), J::F(l.create_ms)),
        ("load_ms".into(), J::F(l.load_ms)),
        ("run_ic_ms".into(), J::F(l.run_ic_ms)),
        ("n_steps".into(), J::U(n as u64)),
        ("script_done".into(), J::Bool(done)),
        ("sim_time_end_s".into(), J::F(sim_end)),
        ("events".into(), J::A(ev)),
        ("step_ns".into(), dist(&step_ns)),
        (
            "pct_one_core".into(),
            J::F(if n > 0 {
                100.0 * sum as f64 / (n as f64 * period_exact)
            } else {
                0.0
            }),
        ),
        (
            "lateness_ns".into(),
            if paced { dist(&late_ns) } else { J::Null },
        ),
        ("deadline_misses".into(), J::U(misses as u64)),
        ("steps_over_period".into(), J::U(n_over as u64)),
        ("over_period_first50".into(), J::A(over)),
        (
            "stream_hash".into(),
            s(format!("{:016x}", stream_hash(&hs[..n]))),
        ),
        ("raw_steps".into(), s(steps_file)),
        ("raw_out30".into(), if traj { s(out_file) } else { J::Null }),
        ("raw_ground".into(), if traj { s(ground_file) } else { J::Null }),
        ("raw_mem".into(), if mem { s(mem_file) } else { J::Null }),
        ("dur_s".into(), if dur_steps > 0 { J::F(dur_s) } else { J::Null }),
        ("script_complete".into(), J::Bool(done && dur_steps == 0 || n < dur_steps)),
        ("mtrace_active".into(), J::Bool(mtrace)),
        (
            "mtrace_marker_bytes".into(),
            if mtrace {
                J::U(plat::MARK_BYTES as u64)
            } else {
                J::Null
            },
        ),
        ("rss_peak_kib".into(), J::U(rss_peak)),
        ("rss_end_kib".into(), J::U(rss_now)),
        ("after_reset".into(), reset_j),
        ("t_start_ns".into(), J::U(t_start)),
        ("t_end_ns".into(), J::U(t_end)),
        ("ok".into(), J::Bool(ok)),
        (
            "error".into(),
            err.clone()
                .map(s)
                .unwrap_or_else(|| if done { J::Null } else { s("script not done") }),
        ),
    ]);
    emit(f, Some((t_start, t_end)));
    Ok(())
}

// --------------------------------------------------------- child: others ----

fn child_loadreset(ctx: &Ctx, c: &Case, rep: u32) -> Result<(), String> {
    let ac = c.str_or("ac", "global5000").to_string();
    let rate: u32 = c.num("rate", 120)?;
    let (script, ic) = script_paths(ctx, &ac, "flight")?;
    let _ = plat::pin_to(ctx.core);
    let t0 = plat::now_ns();
    let mut l = load_flight(ctx, &script, &ic, 1.0 / rate as f64)?;
    // Fly 20 s (parked hold + start of taxi), then reset to the runway.
    for _ in 0..(20 * rate) {
        l.fdm.step().map_err(|e| e.to_string())?;
    }
    let before = l.fdm.get(l.simtime);
    let r0 = plat::now_ns();
    l.fdm.reset().map_err(|e| e.to_string())?;
    let r1 = plat::now_ns();
    let after = l.fdm.get(l.simtime);
    l.fdm.step().map_err(|e| e.to_string())?;
    let t1 = plat::now_ns();
    let (rss_peak, _) = plat::rss_kib();
    emit(
        J::O(vec![
            ("test".into(), s("loadreset")),
            ("rep".into(), J::U(rep as u64)),
            ("aircraft".into(), s(ac.clone())),
            ("rate_hz".into(), J::U(rate as u64)),
            ("mode".into(), s("free")),
            (
                "params".into(),
                J::O(vec![
                    ("aircraft".into(), s(ac)),
                    ("rate_hz".into(), J::U(rate as u64)),
                    ("core".into(), J::U(ctx.core as u64)),
                ]),
            ),
            (
                "script".into(),
                J::O(vec![
                    ("path".into(), s(rel_script(&script))),
                    ("ic".into(), s(rel_script(&ic))),
                ]),
            ),
            ("jsbsim_dt".into(), J::F(l.fdm.get(l.simdt))),
            (
                "affinity".into(),
                J::A(plat::affinity().iter().map(|c| J::U(*c as u64)).collect()),
            ),
            ("create_ms".into(), J::F(l.create_ms)),
            ("load_ms".into(), J::F(l.load_ms)),
            ("run_ic_ms".into(), J::F(l.run_ic_ms)),
            (
                "load_total_ms".into(),
                J::F(l.create_ms + l.load_ms + l.run_ic_ms),
            ),
            ("reset_ms".into(), J::F(ms(r0, r1))),
            ("sim_time_before_reset_s".into(), J::F(before)),
            ("sim_time_after_reset_s".into(), J::F(after)),
            ("rss_peak_kib".into(), J::U(rss_peak)),
            ("ok".into(), J::Bool(after == 0.0)),
            (
                "error".into(),
                if after == 0.0 {
                    J::Null
                } else {
                    s("reset did not return sim time to 0")
                },
            ),
        ]),
        Some((t0, t1)),
    );
    Ok(())
}

fn child_ffi_get(ctx: &Ctx, c: &Case, rep: u32) -> Result<(), String> {
    let ac = c.str_or("ac", "global5000").to_string();
    let requested_n: u64 = c.num("n", 2_000_000)?;
    let (script, ic) = script_paths(ctx, &ac, "flight")?;
    let _ = plat::pin_to(ctx.core);
    let l = load_flight(ctx, &script, &ic, 1.0 / 120.0)?;
    let p = l.outs[12];
    let mut acc = 0.0;
    // 300 ms busy warm-up so the governor leaves its idle clock (same method
    // as ffi_bench's C++ reads).
    let w0 = plat::now_ns();
    while plat::now_ns().saturating_sub(w0) < 300_000_000 {
        for _ in 0..10_000 {
            acc += l.fdm.get(std::hint::black_box(p));
        }
    }
    let t0 = plat::now_ns();
    let mut n = 0u64;
    // Measure for long enough to collect at least five 50 ms environment
    // samples inside the actual timing window (F-07).
    while n < requested_n || plat::now_ns().saturating_sub(t0) < 350_000_000 {
        for _ in 0..10_000 {
            acc += l.fdm.get(std::hint::black_box(p));
        }
        n += 10_000;
    }
    let t1 = plat::now_ns();
    std::hint::black_box(acc);
    emit(
        J::O(vec![
            ("test".into(), s("ffi_get")),
            ("rep".into(), J::U(rep as u64)),
            ("aircraft".into(), s(ac.clone())),
            ("rate_hz".into(), J::U(120)),
            ("mode".into(), s("free")),
            (
                "params".into(),
                J::O(vec![
                    ("aircraft".into(), s(ac)),
                    ("requested_n".into(), J::U(requested_n)),
                    ("n".into(), J::U(n)),
                    ("caller".into(), s("rust")),
                    ("get_warmup_ms".into(), J::U(300)),
                    ("core".into(), J::U(ctx.core as u64)),
                ]),
            ),
            (
                "script".into(),
                J::O(vec![
                    ("path".into(), s(rel_script(&script))),
                    ("ic".into(), s(rel_script(&ic))),
                ]),
            ),
            ("jsbsim_dt".into(), J::F(l.fdm.get(l.simdt))),
            (
                "affinity".into(),
                J::A(plat::affinity().iter().map(|c| J::U(*c as u64)).collect()),
            ),
            ("ns_per_call".into(), J::F((t1 - t0) as f64 / n as f64)),
            ("ok".into(), J::Bool(true)),
            ("error".into(), J::Null),
        ]),
        Some((t0, t1)),
    );
    Ok(())
}

/// F-01 shim exception test: four failing calls, each must return an error
/// code with a non-empty message, then a valid c172p load and 100 steps.
fn child_exceptions(ctx: &Ctx, rep: u32) -> Result<(), String> {
    let t0 = plat::now_ns();
    let mut probes = Vec::new();
    let mut all_ok = true;
    let mut record = |name: &str, code: Option<i32>, msg: String, expect_err: bool| {
        let ok = if expect_err {
            code.map_or(false, |c| c < 0) && !msg.is_empty()
        } else {
            code == Some(0)
        };
        all_ok &= ok;
        probes.push(J::O(vec![
            ("probe".into(), s(name)),
            (
                "code".into(),
                code.map(|c| J::I(c as i64)).unwrap_or(J::Null),
            ),
            ("last_error".into(), s(msg)),
            ("pass".into(), J::Bool(ok)),
        ]));
    };
    // (a) nonexistent aircraft
    {
        let mut f = Fdm::create(&ctx.jsbsim_root).map_err(|e| e.to_string())?;
        match f.load_model("gx_no_such_aircraft", 1.0 / 120.0) {
            Ok(()) => record("missing_aircraft", Some(0), String::new(), true),
            Err(e) => record("missing_aircraft", Some(e.code), e.message, true),
        }
    }
    // (b) malformed script XML (written into the run's raw directory)
    {
        std::fs::create_dir_all(&ctx.raw_dir).map_err(|e| e.to_string())?;
        let bad = ctx.raw_dir.join("gx_malformed_script.xml");
        std::fs::write(
            &bad,
            "<?xml version=\"1.0\"?>\n<runscript name=\"broken\">\n  <use aircraft=\"c172p\"\n",
        )
        .map_err(|e| e.to_string())?;
        let mut f = Fdm::create(&ctx.jsbsim_root).map_err(|e| e.to_string())?;
        match f.load_script(&bad, 1.0 / 120.0, None) {
            Ok(()) => record("malformed_script", Some(0), String::new(), true),
            Err(e) => record("malformed_script", Some(e.code), e.message, true),
        }
    }
    // (c) unknown property
    {
        let (script, ic) = script_paths(ctx, "c172p", "flight")?;
        let mut l = load_flight(ctx, &script, &ic, 1.0 / 120.0)?;
        match l.fdm.bind("gx/no/such/property") {
            Ok(p) => record("unknown_property", Some(p.0), String::new(), true),
            Err(e) => record("unknown_property", Some(e.code), e.message, true),
        }
    }
    // (d) documented throwing path: a script event that sets an undefined
    //     property makes FGScript::RunScript throw a LogException inside
    //     libJSBSim on the first step; gx_fdm_step must return -1.
    {
        let script = ctx.scripts_dir.join("exception_in_step.xml");
        let ic = ctx.scripts_dir.join("ic_runway_c172p.xml");
        let mut f = Fdm::create(&ctx.jsbsim_root).map_err(|e| e.to_string())?;
        f.load_script(&script, 1.0 / 120.0, Some(&ic))
            .map_err(|e| format!("exception_in_step load: {e}"))?;
        f.run_ic().map_err(|e| e.to_string())?;
        match f.step() {
            Ok(_) => record("throw_in_step", Some(0), String::new(), true),
            Err(e) => record("throw_in_step", Some(e.code), e.message, true),
        }
    }
    // then a clean c172p load and 100 steps in the same process
    {
        let (script, ic) = script_paths(ctx, "c172p", "flight")?;
        let mut code = 0;
        let mut msg = String::new();
        match load_flight(ctx, &script, &ic, 1.0 / 120.0) {
            Ok(mut l) => {
                for _ in 0..100 {
                    if let Err(e) = l.fdm.step() {
                        code = e.code;
                        msg = e.message;
                        break;
                    }
                }
            }
            Err(e) => {
                code = -1;
                msg = e;
            }
        }
        record("clean_c172p_100_steps", Some(code), msg, false);
    }
    while plat::now_ns().saturating_sub(t0) < 350_000_000 {
        std::thread::yield_now();
    }
    emit(
        J::O(vec![
            ("test".into(), s("exceptions")),
            ("rep".into(), J::U(rep as u64)),
            ("aircraft".into(), s("c172p")),
            ("rate_hz".into(), J::U(120)),
            ("mode".into(), s("free")),
            (
                "params".into(),
                J::O(vec![
                    ("aircraft".into(), s("c172p")),
                    ("rate_hz".into(), J::U(120)),
                    ("mode".into(), s("free")),
                ]),
            ),
            (
                "script".into(),
                J::O(vec![
                    ("path".into(), s("tests/m1/scripts/exception_in_step.xml")),
                    ("ic".into(), s("tests/m1/scripts/ic_runway_c172p.xml")),
                ]),
            ),
            ("jsbsim_dt".into(), J::F(1.0 / 120.0)),
            (
                "affinity".into(),
                J::A(plat::affinity().iter().map(|c| J::U(*c as u64)).collect()),
            ),
            ("probes".into(), J::A(probes)),
            ("ok".into(), J::Bool(all_ok)),
            (
                "error".into(),
                if all_ok {
                    J::Null
                } else {
                    s("a probe did not behave as documented")
                },
            ),
        ]),
        Some((t0, plat::now_ns())),
    );
    Ok(())
}

/// JSBSim's own standalone executable (same libJSBSim.so, same script, same
/// rate) as the cross-check and as the F-05/F-06 trajectory source (with
/// log=1 it writes the full-rate output directive tests/m1/scripts/output_full_rate.xml).
fn child_standalone(ctx: &Ctx, c: &Case, rep: u32) -> Result<(), String> {
    let ac = c.str_or("ac", "global5000").to_string();
    let rate: u32 = c.num("rate", 120)?;
    let kind = c.str_or("script", "flight").to_string();
    let log = c.str_or("log", "0") == "1";
    let dur_s: f64 = c.num("dur_s", 0.0)?;
    let (script, ic) = script_paths(ctx, &ac, &kind)?;
    let _ = plat::pin_to(ctx.core);
    let exe = ctx.bin_dir.join(if cfg!(windows) {
        "JSBSim.exe"
    } else {
        "JSBSim"
    });
    let tag = format!(
        "{}.{}.rep{}",
        ctx.run_id,
        c.get("name")
            .map(|x| x.to_string())
            .unwrap_or_else(|| format!(
                "standalone-{ac}-{rate}-{kind}{}",
                if log { "-log" } else { "" }
            )),
        rep
    );
    let outdir = ctx.raw_dir.join(format!("{tag}.standalone"));
    let timing_name = format!("{tag}.steps.tsv");
    let timing_path = ctx.raw_dir.join(&timing_name);
    std::fs::create_dir_all(&outdir).map_err(|e| e.to_string())?;
    let mut args: Vec<String> = vec![
        format!("--root={}", ctx.jsbsim_root.display()),
        format!("--outputpath={}", outdir.display()),
        format!("--script={}", script.display()),
        format!("--initfile={}", ic.display()),
        format!("--simulation-rate={rate}"),
        format!("--m1-timing={}", timing_path.display()),
    ];
    if dur_s > 0.0 {
        // JSBSim's own end time (it stops once sim time exceeds it).
        args.push(format!("--end={dur_s}"));
    }
    // global5000 declares one output directive of its own (index 0).
    let own_outputs = if ac == "global5000" { 1 } else { 0 };
    if own_outputs == 1 {
        args.push("--property=simulation/output/enabled=0".into());
    }
    if log {
        args.push(format!(
            "--logdirectivefile={}",
            ctx.scripts_dir.join("output_full_rate.xml").display()
        ));
        let idx = if own_outputs == 1 {
            "simulation/output[1]".to_string()
        } else {
            "simulation/output".to_string()
        };
        args.push(format!("--property={idx}/log_rate_hz={rate}"));
    }
    let t0 = plat::now_ns();
    let o = Command::new(&exe)
        .args(&args)
        .env("JSBSIM_DEBUG", "0")
        .stdin(Stdio::null())
        .output()
        .map_err(|e| format!("spawn {}: {e}", exe.display()))?;
    let t1 = plat::now_ns();
    std::fs::write(outdir.join("stdout.txt"), &o.stdout).map_err(|e| e.to_string())?;
    std::fs::write(outdir.join("stderr.txt"), &o.stderr).map_err(|e| e.to_string())?;
    let code = o.status.code().unwrap_or(-1);
    let timing =
        std::fs::read_to_string(&timing_path).map_err(|e| format!("standalone timing TSV: {e}"))?;
    let mut step_ns = Vec::new();
    let mut late_ns = Vec::new();
    let mut read_dt = None;
    let mut first_start = t0;
    let mut last_end = t1;
    for (i, line) in timing.lines().skip(1).enumerate() {
        let x: Vec<&str> = line.split('\t').collect();
        if x.len() != 5 {
            return Err(format!("malformed standalone timing row {}", i + 2));
        }
        let slot: u64 = x[1].parse().map_err(|_| "standalone slot parse")?;
        let start: u64 = x[2].parse().map_err(|_| "standalone start parse")?;
        let end: u64 = x[3].parse().map_err(|_| "standalone end parse")?;
        let dt: f64 = x[4].parse().map_err(|_| "standalone dt parse")?;
        if i == 0 {
            first_start = start;
            read_dt = Some(dt);
        }
        last_end = end;
        step_ns.push(end.saturating_sub(start));
        late_ns.push(end.saturating_sub(slot));
    }
    let jsbsim_dt = read_dt.unwrap_or(0.0);
    emit(
        J::O(vec![
            ("test".into(), s("standalone")),
            ("rep".into(), J::U(rep as u64)),
            ("aircraft".into(), s(ac.clone())),
            ("rate_hz".into(), J::U(rate as u64)),
            ("mode".into(), s("paced")),
            (
                "params".into(),
                J::O(vec![
                    ("aircraft".into(), s(ac)),
                    ("rate_hz".into(), J::U(rate as u64)),
                    ("mode".into(), s("paced")),
                    ("script_kind".into(), s(kind)),
                    ("log".into(), J::Bool(log)),
                    ("core".into(), J::U(ctx.core as u64)),
                ]),
            ),
            (
                "script".into(),
                J::O(vec![
                    ("path".into(), s(rel_script(&script))),
                    ("ic".into(), s(rel_script(&ic))),
                ]),
            ),
            ("jsbsim_dt".into(), J::F(jsbsim_dt)),
            (
                "dt_ok".into(),
                J::Bool(jsbsim_dt.to_bits() == (1.0 / rate as f64).to_bits()),
            ),
            ("argv".into(), J::A(args.into_iter().map(s).collect())),
            (
                "affinity".into(),
                J::A(plat::affinity().iter().map(|c| J::U(*c as u64)).collect()),
            ),
            ("wall_ms".into(), J::F(ms(t0, t1))),
            ("n_steps".into(), J::U(step_ns.len() as u64)),
            ("step_ns".into(), dist(&step_ns)),
            ("lateness_ns".into(), dist(&late_ns)),
            ("raw_steps".into(), s(timing_name)),
            ("exit_code".into(), J::I(code as i64)),
            ("outdir".into(), s(format!("{tag}.standalone"))),
            ("ok".into(), J::Bool(code == 0)),
            (
                "error".into(),
                if code == 0 {
                    J::Null
                } else {
                    s(format!("JSBSim exited {code}"))
                },
            ),
        ]),
        Some((first_start, last_end)),
    );
    Ok(())
}

/// The tiny C++ caller of the same libgx_fdm.so (src/fdm/ffi_bench). It
/// prints its own GXWINDOW/GXRESULT lines; this child only pins and relays.
fn child_cxx_ffi(ctx: &Ctx, c: &Case, rep: u32) -> Result<(), String> {
    let ac = c.str_or("ac", "global5000").to_string();
    let rate: u32 = c.num("rate", 120)?;
    let mode = c.str_or("mode", "paced").to_string();
    let caller = c.str_or("caller", "shim").to_string();
    let dur_s: f64 = c.num("dur_s", 0.0)?;
    let (script, ic) = script_paths(ctx, &ac, "flight")?;
    let _ = plat::pin_to(ctx.core);
    let exe = ctx.bin_dir.join(if cfg!(windows) {
        "ffi_bench.exe"
    } else {
        "ffi_bench"
    });
    let tag = format!(
        "{}.cxx_ffi-{ac}-{rate}-{mode}-{caller}.rep{rep}",
        ctx.run_id
    );
    std::fs::create_dir_all(&ctx.raw_dir).map_err(|e| e.to_string())?;
    let o = Command::new(&exe)
        .args([
            ctx.jsbsim_root.to_string_lossy().as_ref(),
            script.to_string_lossy().as_ref(),
            ic.to_string_lossy().as_ref(),
            &rate.to_string(),
            &mode,
            &caller,
            ctx.raw_dir
                .join(format!("{tag}.steps.tsv"))
                .to_string_lossy()
                .as_ref(),
            &format!("{dur_s}"),
        ])
        .stdin(Stdio::null())
        .output()
        .map_err(|e| format!("spawn {}: {e}", exe.display()))?;
    let out = String::from_utf8_lossy(&o.stdout);
    let mut found = false;
    for line in out.lines() {
        if line.starts_with("GXWINDOW ") || line.starts_with("GXRESULT ") {
            println!("{line}");
            found |= line.starts_with("GXRESULT ");
        }
    }
    if !found {
        return Err(format!(
            "ffi_bench exit {:?}, no result; stderr: {}",
            o.status.code(),
            String::from_utf8_lossy(&o.stderr)
        ));
    }
    let _ = std::io::stdout().flush();
    let _ = (ac, tag);
    Ok(())
}

/// GPU-companion baseline: no simulation, the child just sleeps dur_s on the
/// bench core, so the companion's frame rate without the sim is measured in
/// the same harness and sampler (TEST-010 measure 4).
fn child_idle(ctx: &Ctx, c: &Case, rep: u32) -> Result<(), String> {
    let dur_s: f64 = c.num("dur_s", 30.0)?;
    let _ = plat::pin_to(ctx.core);
    let t0 = plat::now_ns();
    plat::sleep_until_ns(t0 + (dur_s * 1e9) as u64);
    let t1 = plat::now_ns();
    emit(
        J::O(vec![
            ("test".into(), s("idle")),
            ("rep".into(), J::U(rep as u64)),
            ("params".into(), J::O(vec![("dur_s".into(), J::F(dur_s))])),
            ("t_start_ns".into(), J::U(t0)),
            ("t_end_ns".into(), J::U(t1)),
            ("ok".into(), J::Bool(true)),
            ("error".into(), J::Null),
        ]),
        Some((t0, t1)),
    );
    Ok(())
}

fn run_child(ctx: &Ctx, spec: &str) -> i32 {
    let c = match Case::parse(spec) {
        Ok(c) => c,
        Err(e) => {
            emit(
                J::O(vec![("ok".into(), J::Bool(false)), ("error".into(), s(e))]),
                None,
            );
            return 3;
        }
    };
    let rep: u32 = c.num("rep", 0).unwrap_or(0);
    let r = match c.str_or("test", "step") {
        "step" | "alloc" => child_flight(ctx, &c, rep),
        "loadreset" => child_loadreset(ctx, &c, rep),
        "ffi_get" => child_ffi_get(ctx, &c, rep),
        "exceptions" => child_exceptions(ctx, rep),
        "standalone" => child_standalone(ctx, &c, rep),
        "cxx_ffi" => child_cxx_ffi(ctx, &c, rep),
        "idle" => child_idle(ctx, &c, rep),
        t => Err(format!("unknown test '{t}'")),
    };
    match r {
        Ok(()) => 0,
        Err(e) => {
            emit(
                J::O(vec![
                    ("test".into(), s(c.str_or("test", "step"))),
                    ("rep".into(), J::U(rep as u64)),
                    ("ok".into(), J::Bool(false)),
                    ("error".into(), s(e)),
                ]),
                None,
            );
            4
        }
    }
}

// ----------------------------------------------------------------- parent ----

fn clocks_json(sampler: &plat::Sampler, w: &[plat::Sample]) -> (J, J) {
    fn mode_of(v: &[u64]) -> u64 {
        let mut m: BTreeMap<u64, usize> = BTreeMap::new();
        for x in v {
            *m.entry(*x).or_default() += 1;
        }
        m.into_iter()
            .max_by(|a, b| a.1.cmp(&b.1).then(b.0.cmp(&a.0)))
            .map(|x| x.0)
            .unwrap_or(0)
    }
    let mut cpu = Vec::new();
    for (i, (pol, cpus)) in sampler.policies.iter().enumerate() {
        let v: Vec<u64> = w.iter().filter_map(|s| s.cpu_khz.get(i).copied()).collect();
        cpu.push(J::O(vec![
            ("policy".into(), s(pol.clone())),
            ("cpus".into(), s(cpus.clone())),
            ("min_khz".into(), J::U(v.iter().copied().min().unwrap_or(0))),
            ("mode_khz".into(), J::U(mode_of(&v))),
            ("max_khz".into(), J::U(v.iter().copied().max().unwrap_or(0))),
        ]));
    }
    let g: Vec<u64> = w.iter().filter_map(|s| s.gpu_hz).collect();
    let gpu = if g.is_empty() {
        J::Null
    } else {
        J::O(vec![
            ("min_hz".into(), J::U(*g.iter().min().unwrap())),
            ("mode_hz".into(), J::U(mode_of(&g))),
            ("max_hz".into(), J::U(*g.iter().max().unwrap())),
        ])
    };
    let temps = w
        .iter()
        .filter_map(|s| s.temp_max_c)
        .fold(None, |a: Option<f64>, t| Some(a.map_or(t, |a| a.max(t))));
    (
        J::O(vec![
            ("cpu".into(), J::A(cpu)),
            ("gpu".into(), gpu),
            ("n_samples".into(), J::U(w.len() as u64)),
            ("period_ms".into(), J::U(sampler.period_ms)),
        ]),
        temps.map(J::F).unwrap_or(J::Null),
    )
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let ctx = match ctx_from(&args) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("m1-fdm-bench: {e}");
            std::process::exit(2);
        }
    };
    // JSBSim console chatter off in every child (and in the standalone exe).
    if std::env::var_os("JSBSIM_DEBUG").is_none() {
        std::env::set_var("JSBSIM_DEBUG", "0");
    }
    if let Some(spec) = arg_one(&args, "--child") {
        std::process::exit(run_child(&ctx, &spec));
    }

    let cases: Vec<Case> = match arg_values(&args, "--case")
        .iter()
        .map(|c| Case::parse(c))
        .collect()
    {
        Ok(v) => v,
        Err(e) => {
            eprintln!("m1-fdm-bench: {e}");
            std::process::exit(2);
        }
    };
    if cases.is_empty() {
        eprintln!(
            "m1-fdm-bench: no --case given (see the header of rust/m1-fdm-bench/src/main.rs)"
        );
        std::process::exit(2);
    }
    let hashes = stage_hashes(&ctx.stage);
    let bin_hashes = binary_hashes(&hashes);
    let mut out_file = ctx.out.as_ref().map(|p| {
        std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(p)
            .expect("open --out")
    });
    let mut write_row = |row: &J| {
        let line = row.to_string();
        println!("{line}");
        if let Some(f) = out_file.as_mut() {
            let _ = writeln!(f, "{line}");
            let _ = f.flush();
        }
    };
    let _ = std::fs::create_dir_all(&ctx.raw_dir);
    let mut sampler = plat::Sampler::start(50, ctx.sampler_core);
    let exe = std::env::current_exe().expect("current_exe");

    write_row(&J::O(vec![
        ("kind".into(), s("info")),
        ("run_id".into(), s(ctx.run_id.clone())),
        ("test".into(), s("meta")),
        ("argv".into(), J::A(args.iter().cloned().map(s).collect())),
        ("binary_sha256".into(), bin_hashes.clone()),
        ("stage_files_sha256_present".into(), J::Bool(!hashes.is_empty())),
        ("cases".into(), J::A(cases.iter().map(|c| s(c.spec())).collect())),
        ("output_set".into(), J::A(OUTPUT_SET.iter().map(|x| s(*x)).collect())),
        ("hash".into(), s("FNV-1a 64 over the little-endian IEEE-754 bit patterns of the output set, per step")),
        ("bench_core".into(), J::U(ctx.core as u64)),
        ("sampler_core".into(), ctx.sampler_core.map(|c| J::U(c as u64)).unwrap_or(J::Null)),
        ("cpufreq_policies".into(), J::A(sampler.policies.iter().map(|(p, c)| J::O(vec![("policy".into(), s(p.clone())), ("cpus".into(), s(c.clone()))])).collect())),
        ("ok".into(), J::Bool(true)),
        ("error".into(), J::Null),
    ]));

    let mut any_fail = false;
    for c in &cases {
        let reps: u32 = c.num("reps", 1).unwrap_or(1);
        let test = c.str_or("test", "step").to_string();
        for rep in 0..reps {
            let spec = format!("{},rep={rep}", c.spec());
            let mut cmd = Command::new(&exe);
            cmd.arg("--child")
                .arg(&spec)
                .args(pass_through_args(&ctx))
                .stdin(Stdio::null());
            let mut alloc_env = J::Null;
            if test == "alloc" {
                let trace = ctx.raw_dir.join(format!(
                    "{}.alloc-{}-{}-{}{}.rep{rep}.mtrace",
                    ctx.run_id,
                    c.str_or("ac", "global5000"),
                    c.str_or("rate", "120"),
                    c.str_or("mode", "paced"),
                    if c.get("inject_alloc").is_some() {
                        "-inj"
                    } else {
                        ""
                    }
                ));
                if c.str_or("preload", "1") == "1" {
                    cmd.env("LD_PRELOAD", &ctx.malloc_debug);
                }
                cmd.env("MALLOC_TRACE", &trace);
                alloc_env = J::O(vec![
                    (
                        "interposer".into(),
                        s("glibc libc_malloc_debug.so mtrace (LD_PRELOAD + MALLOC_TRACE)"),
                    ),
                    (
                        "ld_preload".into(),
                        if c.str_or("preload", "1") == "1" {
                            s(ctx.malloc_debug.clone())
                        } else {
                            J::Null
                        },
                    ),
                    (
                        "mtrace_file".into(),
                        s(trace.file_name().unwrap().to_string_lossy().into_owned()),
                    ),
                ]);
            }
            let t0 = plat::now_ns();
            let o = cmd.output();
            let t1 = plat::now_ns();
            let (code, stdout, stderr) = match o {
                Ok(o) => (
                    o.status.code().unwrap_or(-1),
                    String::from_utf8_lossy(&o.stdout).into_owned(),
                    String::from_utf8_lossy(&o.stderr).into_owned(),
                ),
                Err(e) => (-1, String::new(), format!("spawn failed: {e}")),
            };
            let mut window = (t0, t1);
            let mut frag = String::new();
            for line in stdout.lines() {
                if let Some(r) = line.strip_prefix("GXWINDOW ") {
                    let v: Vec<u64> = r
                        .split_whitespace()
                        .filter_map(|x| x.parse().ok())
                        .collect();
                    if v.len() == 2 {
                        window = (v[0], v[1]);
                    }
                } else if let Some(r) = line.strip_prefix("GXRESULT ") {
                    frag = r.to_string();
                }
            }
            let w = sampler.window(window.0, window.1);
            let (clocks, temps) = clocks_json(&sampler, &w);
            let script_sha = c
                .get("ac")
                .and_then(|ac| script_paths(&ctx, ac, c.str_or("script", "flight")).ok())
                .map(|(sp, ip)| {
                    J::O(vec![
                        (
                            "script".into(),
                            hashes
                                .get(&rel_script(&sp))
                                .map(|h| s(h.clone()))
                                .unwrap_or(J::Null),
                        ),
                        (
                            "ic".into(),
                            hashes
                                .get(&rel_script(&ip))
                                .map(|h| s(h.clone()))
                                .unwrap_or(J::Null),
                        ),
                    ])
                })
                .unwrap_or(J::Null);
            let mut row = vec![
                ("kind".to_string(), s("measurement")),
                ("run_id".into(), s(ctx.run_id.clone())),
                ("case".into(), s(c.spec())),
                ("case_params".into(), c.json()),
                ("injected".into(), J::Bool(c.injected())),
                ("binary_sha256".into(), bin_hashes.clone()),
                ("script_sha256".into(), script_sha),
                ("alloc".into(), alloc_env),
                ("child_exit".into(), J::I(code as i64)),
                ("child_wall_ms".into(), J::F(ms(t0, t1))),
                (
                    "window_ns".into(),
                    J::A(vec![J::U(window.0), J::U(window.1)]),
                ),
                ("clocks".into(), clocks),
                ("temps_max".into(), temps),
            ];
            if frag.is_empty() {
                let tail: String = stderr
                    .chars()
                    .rev()
                    .take(2000)
                    .collect::<Vec<_>>()
                    .into_iter()
                    .rev()
                    .collect();
                row.push(("test".into(), s(test.clone())));
                row.push(("rep".into(), J::U(rep as u64)));
                row.push(("ok".into(), J::Bool(false)));
                row.push((
                    "error".into(),
                    s(format!(
                        "child exited {code} without a result; stderr tail: {tail}"
                    )),
                ));
                any_fail = true;
            } else {
                if frag.contains("\"ok\":false") || code != 0 {
                    any_fail = true;
                }
                row.push(("result".into(), J::Raw(frag)));
            }
            write_row(&J::O(row));
            if !stderr.is_empty() {
                let _ = std::fs::write(
                    ctx.raw_dir.join(format!(
                        "{}.{}.rep{rep}.stderr.txt",
                        ctx.run_id,
                        c.spec().replace([',', '='], "_")
                    )),
                    &stderr,
                );
            }
        }
    }
    sampler.stop();
    std::process::exit(if any_fail { 1 } else { 0 });
}
