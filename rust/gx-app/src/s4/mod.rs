//! M1 step S4 (SFO world): the S3 binary (S2 thread layout, JSBSim
//! global5000, real stick, INPUT-002, render thread on DRM/KMS + GLES) with
//! the GXWORLD 2.0 synthetic SFO pack, a placeholder Global 5000 and chase /
//! cockpit cameras. The render-thread architecture is S3's unchanged; see
//! `s3::runtime::Look::World` and `gx_render::{world, camera, aircraft}`.

pub const SCHEMA: &str = "gx-s4/1";

/// Appended to the first prompt.
pub const CAMERA_HINT: &str = "Button 5 (top of the stick, right of the hat) switches CHASE / COCKPIT camera. Hold base button 7 / 8 to trim nose DOWN / UP. Base 9 / 10 = flaps up / down, 11 = gear, 6 = speedbrake.";

pub const HELP: &str = "\
gx-app s4 - M1 step S4: SFO world

The S3 layout (I/O SCHED_FIFO 40 -> control mailbox -> JSBSim sim
SCHED_FIFO 50 at 120 Hz -> SnapshotExchange -> render thread SCHED_OTHER on
CPU 3 owning DRM/KMS + GBM + EGL/GLES, 30 fps) drawing the GXWORLD 2.0
synthetic SFO pack (land, Bay water, taxiway, runway 10L/28R with markings
and lights), a placeholder Global 5000, a chase or cockpit camera, and the
S3 HUD (horizon lines, IAS/HDG/AGL, INPUT-002 banner, PAUSED overlay,
prompts) on top. One render target, depth 24, frame-start clear,
end-of-frame depth/stencil invalidate (SPIKE-001c).

Bindings: trigger = Start, 2 = Pause, 3 = Reset (sets the parking brake),
4 = parking brake toggle, hat = wheel brakes (back both, left/right one
side), twist = rudder, 5 = camera (chase <-> cockpit), 7 / 8 held =
pitch trim nose down / up (HUD TRIM, Reset zeroes it, input loss holds it),
9 / 10 = flap lever up / down one detent, 11 = gear lever, 6 = speedbrake
stowed / half / full (HUD GEAR, FLAPS, SPDBRK; Reset restores gear down and
the take-off flap; input loss holds the levers; the gear stays down with
weight on wheels). `--aircraft 787-8` flies the BBJ 787.

  --display kms        DP-1 1280x720@60 (DRM master: only through the
                       watchdog launcher tools/m1_s4_launch.sh)
  --display offscreen  the same frames on the render node (non-display smoke)
  --display fake       no GPU (host development)

USAGE:
    gx-app s4 [OPTIONS]      (options as gx-app s3 --help; log default s4-run.jsonl)

SPIKE-002 (photoreal cost spike):
    --world PATH             GXWORLD 2.0 pack file (surveyed SFO) instead of the synthetic one
    --visual PATH            GXVISUAL sidecar matching --world: textured terrain, water,
                             sky/haze, markings and the textured Global 5000
    --trace                  offscreen only: scripted 4 x 30 s camera trace (threshold chase,
                             cockpit, climb-out over the water, high wide)
    --capture-dir DIR        with --trace: one PPM capture per segment

Check: python3 tools/s4_check.py s4-run.jsonl
";

/// Entry point for `gx-app s4 ...`; returns the exit code.
pub fn main(args: &[String]) -> i32 {
    crate::s3::main_step(args, crate::s3::Step::S4, HELP)
}

/// `gx-app rtprobe`: the realtime envelope a run would get, measured the way
/// the run gets it. Two threads take exactly the sim and I/O setups the
/// S2/S3/S4 layout uses (`setup_sim_thread`, `setup_io_thread`) and report
/// what took, as one JSON line. Exit 0 only if the sim thread got FIFO 50
/// and the I/O thread FIFO 40 (ISS-038: the launcher runs this inside a
/// transient unit with the candidate's exact limits before it touches the
/// desktop; the runner runs it again inside the candidate unit itself).
pub fn rt_probe_main() -> i32 {
    let sim = std::thread::spawn(|| crate::s1::rt::setup_sim_thread(2));
    let io = std::thread::spawn(|| crate::s1::rt::setup_io_thread(1));
    let (sim, io) = (sim.join().unwrap_or_default(), io.join().unwrap_or_default());
    let ok = sim.contains("\"policy\":\"fifo50\"") && io.contains("\"policy\":\"fifo40\"");
    println!("{{\"k\":\"rtprobe\",\"ok\":{ok},\"rlimit_rtprio\":{},\"sim\":{sim},\"io\":{io}}}", rlimit_rtprio());
    if ok {
        0
    } else {
        1
    }
}

fn rlimit_rtprio() -> String {
    #[cfg(target_os = "linux")]
    {
        let mut r = libc::rlimit { rlim_cur: 0, rlim_max: 0 };
        // SAFETY: valid out pointer.
        if unsafe { libc::getrlimit(libc::RLIMIT_RTPRIO, &mut r) } == 0 {
            return format!("[{},{}]", r.rlim_cur, r.rlim_max);
        }
    }
    "null".into()
}
