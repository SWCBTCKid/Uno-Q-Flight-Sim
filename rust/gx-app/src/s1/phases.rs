//! Operator prompts for the Chief's S1 session, driven by what the threads
//! actually observe (not by a fixed clock), so a slow unplug only delays the
//! script. Pure state machine; the main thread prints its lines.

use gx_sim::{InputStatus, PauseReason, RunState, UnavailableReason};

pub const MOVE_S: u64 = 20;
pub const FLY_S: u64 = 20;
pub const UNPLUG_HOLD_S: u64 = 12;
pub const CONSOLE_WAIT_S: u64 = 30;
pub const NO_AUTO_RESUME_S: u64 = 5;

pub const BUTTONS_HELP: &str = "\
Buttons (Logitech Extreme 3D Pro, Linux evdev codes as used by gx-app):
  START = logical 0 = BTN_TRIGGER (0x120): the TRIGGER under your index finger (labelled 1)
  PAUSE = logical 1 = BTN_THUMB   (0x121): the THUMB button on the left side of the grip (labelled 2)
  RESET = logical 2 = BTN_THUMB2  (0x122): button 3 on the top of the stick head (labelled 3)
  Console: ENTER in this terminal = a Start command straight to the sim core
           (used once, while the stick is unplugged, to show it is refused)";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    Move,
    Start,
    Fly,
    Unplug,
    Unplugged,
    ConsoleStart,
    Replug,
    NoAutoResume,
    Resume,
    Free,
    Done,
}

impl Phase {
    pub fn name(self) -> &'static str {
        match self {
            Phase::Move => "move",
            Phase::Start => "start",
            Phase::Fly => "fly",
            Phase::Unplug => "unplug",
            Phase::Unplugged => "unplugged",
            Phase::ConsoleStart => "console_start",
            Phase::Replug => "replug",
            Phase::NoAutoResume => "no_auto_resume",
            Phase::Resume => "resume",
            Phase::Free => "free",
            Phase::Done => "done",
        }
    }
}

/// What the prompter sees each poll (from the shared status atomics).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Obs {
    /// Seconds since the run started, in ns.
    pub t_ns: u64,
    pub end_ns: u64,
    pub connected: bool,
    pub frames: u64,
    pub run: RunState,
    pub pause: Option<PauseReason>,
    pub status: InputStatus,
    pub loss_ms: u64,
    pub console_starts: u64,
    pub refused_console: u64,
    pub accepted_console: u64,
    pub button_presses: u64,
    pub last_buttons: u64,
}

pub struct Prompter {
    pub phase: Phase,
    entered_ns: u64,
    frames_at_entry: u64,
    unplugged_at: Option<u64>,
    paused_seen: bool,
    console_at_entry: u64,
    console_pressed_at: Option<u64>,
    console_reported: bool,
    last_tick_s: Option<u64>,
    seen_presses: u64,
    started: bool,
    fly_running_ns: u64,
    fly_running_since: Option<u64>,
}

const S: u64 = 1_000_000_000;

fn secs(ns: u64) -> u64 {
    ns / S
}

fn unplugged(obs: &Obs) -> bool {
    obs.status == InputStatus::Unavailable(UnavailableReason::Disconnected) || !obs.connected
}

impl Default for Prompter {
    fn default() -> Self {
        Self::new()
    }
}

impl Prompter {
    pub fn new() -> Self {
        Self {
            phase: Phase::Move,
            entered_ns: 0,
            frames_at_entry: 0,
            unplugged_at: None,
            paused_seen: false,
            console_at_entry: 0,
            console_pressed_at: None,
            console_reported: false,
            last_tick_s: None,
            seen_presses: 0,
            started: false,
            fly_running_ns: 0,
            fly_running_since: None,
        }
    }

    fn enter(&mut self, phase: Phase, obs: &Obs, out: &mut Vec<(Phase, String)>, text: String) {
        self.phase = phase;
        self.entered_ns = obs.t_ns;
        self.frames_at_entry = obs.frames;
        self.last_tick_s = None;
        if phase == Phase::Fly {
            self.fly_running_ns = 0;
            self.fly_running_since = (obs.run == RunState::Running).then_some(obs.t_ns);
        }
        out.push((phase, text));
    }

    fn in_phase_s(&self, obs: &Obs) -> u64 {
        secs(obs.t_ns.saturating_sub(self.entered_ns))
    }

    /// True once per whole second of the current phase.
    fn each_second(&mut self, obs: &Obs) -> Option<u64> {
        let s = self.in_phase_s(obs);
        if self.last_tick_s == Some(s) {
            None
        } else {
            self.last_tick_s = Some(s);
            Some(s)
        }
    }

    /// Advance on one observation; returns `(phase, line)` prompts to print
    /// and log.
    pub fn step(&mut self, obs: &Obs) -> Vec<(Phase, String)> {
        let mut out = Vec::new();
        if !self.started {
            self.started = true;
            self.enter(
                Phase::Move,
                obs,
                &mut out,
                format!(
                    "PHASE 1/6  MOVE THE STICK: roll, pitch, twist and throttle, for {MOVE_S} s (no buttons yet)"
                ),
            );
            return out;
        }
        if obs.button_presses != self.seen_presses {
            self.seen_presses = obs.button_presses;
            out.push((self.phase, format!("  (button press seen: logical mask {:#x})", obs.last_buttons)));
        }
        if self.phase != Phase::Done && obs.t_ns >= obs.end_ns {
            let note = if self.phase == Phase::Free {
                "RUN COMPLETE. Writing the log; copy it to the laptop and run the checker.".to_owned()
            } else {
                format!(
                    "TIME UP during phase '{}': the session did not finish all phases (the checker will say which).",
                    self.phase.name()
                )
            };
            self.enter(Phase::Done, obs, &mut out, note);
            return out;
        }
        match self.phase {
            Phase::Move => {
                if self.in_phase_s(obs) >= MOVE_S {
                    if obs.frames > self.frames_at_entry {
                        self.enter(
                            Phase::Start,
                            obs,
                            &mut out,
                            "PHASE 2/6  PRESS START = the TRIGGER (button 1)".into(),
                        );
                    } else if let Some(s) = self.each_second(obs) {
                        if s % 10 == 0 {
                            out.push((self.phase, "  no stick input seen yet: is it plugged in? keep moving it".into()));
                        }
                    }
                }
            }
            Phase::Start => {
                if obs.run == RunState::Running {
                    self.enter(
                        Phase::Fly,
                        obs,
                        &mut out,
                        format!("  sim RUNNING. Keep moving the stick for {FLY_S} s"),
                    );
                } else if let Some(s) = self.each_second(obs) {
                    if s > 0 && s % 10 == 0 {
                        out.push((self.phase, "  still waiting for START (the TRIGGER)".into()));
                    }
                }
            }
            Phase::Fly => {
                if obs.run == RunState::Running {
                    let since = self.fly_running_since.get_or_insert(obs.t_ns);
                    let running_ns = self.fly_running_ns + obs.t_ns.saturating_sub(*since);
                    if running_ns >= FLY_S * S {
                        self.fly_running_ns = running_ns;
                        self.fly_running_since = None;
                        self.enter(
                            Phase::Unplug,
                            obs,
                            &mut out,
                            format!("PHASE 3/6  UNPLUG the stick's USB cable NOW and keep it unplugged for at least {UNPLUG_HOLD_S} s"),
                        );
                    }
                } else {
                    if let Some(since) = self.fly_running_since.take() {
                        self.fly_running_ns += obs.t_ns.saturating_sub(since);
                    }
                    if let Some(s) = self.each_second(obs) {
                        if s == 0 || s % 5 == 0 {
                            out.push((self.phase, format!(
                                "  sim PAUSED ({:?}): press START = the TRIGGER to resume; running flight time {}/{} s",
                                obs.pause, secs(self.fly_running_ns), FLY_S
                            )));
                        }
                    }
                }
            }
            Phase::Unplug => {
                if unplugged(obs) {
                    self.unplugged_at = Some(obs.t_ns);
                    self.enter(
                        Phase::Unplugged,
                        obs,
                        &mut out,
                        format!("  unplug seen: INPUT-002 warning ON. Keep it UNPLUGGED ({UNPLUG_HOLD_S} s countdown)"),
                    );
                }
            }
            Phase::Unplugged | Phase::ConsoleStart => self.unplugged_step(obs, &mut out),
            Phase::Replug => {
                if obs.status == InputStatus::Valid {
                    let still_paused = obs.run != RunState::Running;
                    self.enter(
                        Phase::NoAutoResume,
                        obs,
                        &mut out,
                        format!(
                            "  input VALID again, warning cleared. Sim is {} (no auto-resume). Keep moving, {NO_AUTO_RESUME_S} s",
                            if still_paused { "still PAUSED" } else { "RUNNING (UNEXPECTED: it resumed by itself)" }
                        ),
                    );
                }
            }
            Phase::NoAutoResume => {
                if obs.run == RunState::Running {
                    out.push((self.phase, "  !! the sim resumed WITHOUT a Start: INPUT-002 violation".into()));
                    self.enter(Phase::Free, obs, &mut out, "  continuing; fly freely until the end".into());
                } else if let Some(s) = self.each_second(obs) {
                    if s >= NO_AUTO_RESUME_S {
                        self.enter(
                            Phase::Resume,
                            obs,
                            &mut out,
                            "PHASE 6/6  PRESS START = the TRIGGER (button 1): the sim must resume".into(),
                        );
                    } else if s > 0 {
                        out.push((self.phase, format!("  still paused ... {}", NO_AUTO_RESUME_S - s)));
                    }
                }
            }
            Phase::Resume => {
                if obs.run == RunState::Running {
                    let left = secs(obs.end_ns.saturating_sub(obs.t_ns));
                    self.enter(
                        Phase::Free,
                        obs,
                        &mut out,
                        format!("  RESUMED. All phases done. Fly freely until the end ({left} s left)"),
                    );
                }
            }
            Phase::Free => {
                if let Some(s) = self.each_second(obs) {
                    if s > 0 && s % 30 == 0 {
                        let left = secs(obs.end_ns.saturating_sub(obs.t_ns));
                        out.push((self.phase, format!("  {left} s left - keep moving the stick now and then")));
                    }
                }
            }
            Phase::Done => {}
        }
        out
    }

    fn unplugged_step(&mut self, obs: &Obs, out: &mut Vec<(Phase, String)>) {
        let since = obs.t_ns.saturating_sub(self.unplugged_at.unwrap_or(obs.t_ns));
        let held_s = secs(since);
        if !unplugged(obs) {
            if held_s < UNPLUG_HOLD_S {
                out.push((self.phase, format!(
                    "  replugged after only {held_s} s (needs {UNPLUG_HOLD_S} s): UNPLUG it again"
                )));
                self.paused_seen = false;
                self.enter(Phase::Unplug, obs, out, "PHASE 3/6  UNPLUG the stick again".into());
            } else {
                // Replugged before the console step was offered or done.
                self.enter_replug(obs, out);
            }
            return;
        }
        if !self.paused_seen && obs.pause == Some(PauseReason::InputLoss) {
            self.paused_seen = true;
            out.push((self.phase, format!(
                "  SIM PAUSED by INPUT-002 after {} ms of loss",
                obs.loss_ms
            )));
            self.console_at_entry = obs.console_starts;
            self.console_pressed_at = None;
            self.console_reported = false;
            let unplugged_at = self.unplugged_at;
            self.enter(
                Phase::ConsoleStart,
                obs,
                out,
                "PHASE 4/6  stick still UNPLUGGED: press ENTER in this terminal now (a Start that must be REFUSED)".into(),
            );
            self.unplugged_at = unplugged_at;
            return;
        }
        if self.phase == Phase::ConsoleStart {
            if obs.console_starts != self.console_at_entry && self.console_pressed_at.is_none() {
                self.console_pressed_at = Some(obs.t_ns);
            }
            if let (Some(at), false) = (self.console_pressed_at, self.console_reported) {
                if obs.refused_console > 0 || obs.accepted_console > 0 || obs.t_ns - at > S {
                    self.console_reported = true;
                    out.push((self.phase, if obs.accepted_console > 0 {
                        "  !! console Start was ACCEPTED during the loss: INPUT-002 violation".into()
                    } else if obs.refused_console > 0 {
                        "  console Start REFUSED (correct)".into()
                    } else {
                        "  console Start: no refusal recorded yet".into()
                    }));
                }
            }
            let waited = self.in_phase_s(obs);
            let console_done = self.console_reported || waited >= CONSOLE_WAIT_S;
            if console_done && held_s >= UNPLUG_HOLD_S {
                if !self.console_reported {
                    out.push((self.phase, "  no ENTER seen; skipping the refused-Start step".into()));
                }
                self.enter_replug(obs, out);
                return;
            }
        }
        let tick = held_s;
        if self.last_tick_s != Some(tick) && held_s <= UNPLUG_HOLD_S {
            self.last_tick_s = Some(tick);
            if held_s < UNPLUG_HOLD_S {
                out.push((self.phase, format!(
                    "  unplugged {held_s:>2} s: keep it UNPLUGGED, {} s left (loss {} ms, sim {})",
                    UNPLUG_HOLD_S - held_s,
                    obs.loss_ms,
                    if obs.run == RunState::Running { "running" } else { "PAUSED" }
                )));
            } else if self.phase == Phase::ConsoleStart && !self.console_reported {
                out.push((self.phase, "  12 s reached: press ENTER now (stick still unplugged), then you will be asked to replug".into()));
            }
        }
    }

    fn enter_replug(&mut self, obs: &Obs, out: &mut Vec<(Phase, String)>) {
        self.enter(
            Phase::Replug,
            obs,
            out,
            "PHASE 5/6  REPLUG the stick now, then MOVE it (do NOT press any button)".into(),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn obs(t_s: f64) -> Obs {
        Obs {
            t_ns: (t_s * 1e9) as u64,
            end_ns: 300 * S,
            connected: true,
            frames: 0,
            run: RunState::Paused,
            pause: Some(PauseReason::Reset),
            status: InputStatus::Valid,
            loss_ms: 0,
            console_starts: 0,
            refused_console: 0,
            accepted_console: 0,
            button_presses: 0,
            last_buttons: 0,
        }
    }

    fn drive(p: &mut Prompter, o: &Obs) -> Vec<Phase> {
        p.step(o).into_iter().map(|(phase, _)| phase).collect()
    }

    #[test]
    fn full_session_walks_every_phase_in_order() {
        let mut p = Prompter::new();
        let mut o = obs(0.0);
        drive(&mut p, &o);
        assert_eq!(p.phase, Phase::Move);
        o.frames = 50;
        o.t_ns = 20 * S;
        drive(&mut p, &o);
        assert_eq!(p.phase, Phase::Start);
        o.t_ns = 23 * S;
        o.run = RunState::Running;
        o.pause = None;
        drive(&mut p, &o);
        assert_eq!(p.phase, Phase::Fly);
        o.t_ns = 43 * S;
        drive(&mut p, &o);
        assert_eq!(p.phase, Phase::Unplug);
        o.t_ns = 45 * S;
        o.connected = false;
        o.status = InputStatus::Unavailable(UnavailableReason::Disconnected);
        drive(&mut p, &o);
        assert_eq!(p.phase, Phase::Unplugged);
        o.t_ns = 55 * S;
        o.run = RunState::Paused;
        o.pause = Some(PauseReason::InputLoss);
        o.loss_ms = 10_000;
        drive(&mut p, &o);
        assert_eq!(p.phase, Phase::ConsoleStart);
        o.t_ns = 56 * S;
        o.console_starts = 1;
        o.refused_console = 1;
        drive(&mut p, &o);
        assert_eq!(p.phase, Phase::ConsoleStart, "unplugged only 11 s");
        o.t_ns = 57 * S + 1;
        drive(&mut p, &o);
        assert_eq!(p.phase, Phase::Replug);
        o.t_ns = 60 * S;
        o.connected = true;
        o.status = InputStatus::Valid;
        drive(&mut p, &o);
        assert_eq!(p.phase, Phase::NoAutoResume);
        for s in 61..=65 {
            o.t_ns = s * S;
            drive(&mut p, &o);
        }
        assert_eq!(p.phase, Phase::Resume);
        o.t_ns = 67 * S;
        o.run = RunState::Running;
        o.pause = None;
        drive(&mut p, &o);
        assert_eq!(p.phase, Phase::Free);
        o.t_ns = 300 * S;
        let lines = p.step(&o);
        assert_eq!(p.phase, Phase::Done);
        assert!(lines.last().unwrap().1.starts_with("RUN COMPLETE"));
    }

    #[test]
    fn early_replug_asks_again_and_time_up_names_the_phase() {
        let mut p = Prompter::new();
        let mut o = obs(0.0);
        p.step(&o);
        o.frames = 1;
        o.t_ns = 20 * S;
        p.step(&o);
        o.run = RunState::Running;
        p.step(&o);
        o.t_ns = 40 * S;
        p.step(&o);
        assert_eq!(p.phase, Phase::Unplug);
        o.connected = false;
        o.status = InputStatus::Unavailable(UnavailableReason::Disconnected);
        p.step(&o);
        o.t_ns = 43 * S;
        o.connected = true;
        o.status = InputStatus::Valid;
        p.step(&o);
        assert_eq!(p.phase, Phase::Unplug);
        o.t_ns = 300 * S;
        let lines = p.step(&o);
        assert_eq!(p.phase, Phase::Done);
        assert!(lines.last().unwrap().1.contains("'unplug'"));
    }

    #[test]
    fn fly_counts_running_time_only_and_guides_an_accidental_pause() {
        let mut p = Prompter::new();
        let mut o = obs(0.0);
        p.step(&o);
        o.frames = 1;
        o.t_ns = 20 * S;
        p.step(&o);
        o.run = RunState::Running;
        o.pause = None;
        o.t_ns = 21 * S;
        p.step(&o);
        assert_eq!(p.phase, Phase::Fly);

        o.t_ns = 26 * S;
        p.step(&o);
        o.run = RunState::Paused;
        o.pause = Some(PauseReason::User);
        let lines = p.step(&o);
        assert_eq!(p.phase, Phase::Fly);
        assert!(lines.iter().any(|(_, line)| line.contains("press START")));

        o.run = RunState::Running;
        o.pause = None;
        o.t_ns = 40 * S;
        p.step(&o);
        o.t_ns = 54 * S;
        p.step(&o);
        assert_eq!(p.phase, Phase::Fly, "only 19 running seconds");
        o.t_ns = 55 * S;
        p.step(&o);
        assert_eq!(p.phase, Phase::Unplug, "20 running seconds reached");
    }
}
