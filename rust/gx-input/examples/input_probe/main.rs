//! Headless on-board input smoke probe (TEST-027).
//!
//! Runs `EvdevInput` against the Logitech Extreme 3D Pro and writes every
//! `RuntimeEvent` (raw records, frames, loss/resync records, state changes)
//! as JSON lines with monotonic timestamps. It opens the evdev node
//! read-only and nonblocking (via `EvdevInput`), never grabs it, needs no
//! privileges beyond the `input` group, and opens no display.
//!
//! `--stall-at S --stall-ms M` stops reading for M ms at S seconds so the
//! kernel's per-client evdev queue overflows while the stick is moved hard,
//! forcing `SYN_DROPPED` and the live resync path.
//!
//! Analyse the output with `tools/analyze_m1_input_probe.py`.

#[cfg(target_os = "linux")]
mod probe;

#[cfg(target_os = "linux")]
fn main() {
    std::process::exit(probe::run());
}

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("input_probe: Linux evdev only");
    std::process::exit(1);
}
