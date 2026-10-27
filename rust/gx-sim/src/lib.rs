//! The deterministic, fixed-rate core of Global Express.
//!
//! [`SimCore`] and its [`SimulationBackend`] belong to one simulation thread.
//! Other threads exchange only plain controls, commands, and immutable
//! [`AircraftSnapshot`] values. Controls arrive through the single-writer /
//! single-reader [`control_mailbox`]; snapshots leave through
//! [`SnapshotExchange`].  The optional `jsbsim` feature provides the
//! only adapter to `gx-fdm`; no raw FFI is exposed here.

mod control_mailbox;
mod fdm_signs;
mod handoff;
mod georef;
mod scheduler;
pub mod autopilot;
pub mod levers;
pub mod trim;

#[cfg(feature = "jsbsim")]
mod jsbsim;

pub use georef::{GeoreferenceError, Wgs84EnuFrame};
pub use control_mailbox::{
    command_index, control_mailbox, control_mailbox_with_stale_after, CommandBatch, CommandEdges,
    ControlReader, ControlRecord, ControlStatus, ControlWriter, EdgeCounter, InputValidity,
    ReaderStats, TickControls, UnavailableReason, DEFAULT_STALE_AFTER_NS, HEARTBEAT_PERIOD_NS,
};
pub use fdm_signs::JsbsimCommands;
pub use handoff::{PublishBusy, SnapshotExchange, SnapshotPair};
pub use scheduler::{Deadline, FixedRate, TimingCounters};
pub use autopilot::{ApAction, ApInput, ApStatus, Approach, Autopilot};
pub use levers::Levers;
pub use trim::PitchTrim;

#[cfg(feature = "jsbsim")]
pub use jsbsim::{EngineOutput, InitialConditions, JsbsimBackend, JsbsimError, JsbsimSetup};

/// The production flight-dynamics cadence.
pub const SIM_HZ: u32 = 120;
pub const STEP_SECONDS: f64 = 1.0 / SIM_HZ as f64;

/// Fully normalized pilot controls. Invalid and out-of-range values are made
/// safe at the input boundary, not in the FDM adapter.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ControlInput {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub throttle: f32,
    pub left_brake: f32,
    pub right_brake: f32,
    /// Pitch trim switch: +1 = nose-up held, -1 = nose-down held, 0 =
    /// released (see [`trim`]). Not a position: neutral on input loss stops
    /// the trim where it is.
    pub trim: f32,
    /// Gear, flap and speedbrake levers (see [`levers`]). Positions, not
    /// axes: [`SimCore`] holds the last valid levers through an input loss.
    pub levers: Levers,
    /// The autopilot panel (see [`autopilot`]): targets and the last click.
    /// Held through an input loss like the levers.
    pub ap: ApInput,
}

impl ControlInput {
    pub fn normalized(self) -> Self {
        fn axis(v: f32) -> f32 {
            if v.is_finite() { v.clamp(-1.0, 1.0) } else { 0.0 }
        }
        fn unit(v: f32) -> f32 {
            if v.is_finite() { v.clamp(0.0, 1.0) } else { 0.0 }
        }
        Self {
            roll: axis(self.roll),
            pitch: axis(self.pitch),
            yaw: axis(self.yaw),
            throttle: unit(self.throttle),
            left_brake: unit(self.left_brake),
            right_brake: unit(self.right_brake),
            trim: axis(self.trim),
            levers: self.levers.normalized(),
            ap: self.ap.normalized(),
        }
    }
}

/// Named geodetic datum carried in telemetry. More variants require an
/// explicit conversion at the simulation/world seam.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum GeodeticDatum {
    #[default]
    Wgs84Ellipsoid,
}

/// Logging position. `ellipsoid_height_m` is deliberately not AGL or MSL.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GeodeticPosition {
    pub datum: GeodeticDatum,
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub ellipsoid_height_m: f64,
}

/// Authoritative render position in a specific local ENU world frame.
/// `None` in [`AircraftState::local_enu`] means invalid/unresolved and is
/// rejected by [`SimCore`] before publication.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LocalEnuPose {
    /// Stable asset/world-frame identifier. Zero is invalid.
    pub frame_id: u64,
    /// Changes whenever that identifier's origin or convention changes.
    pub frame_generation: u64,
    pub position_m: [f64; 3],
    pub velocity_mps: [f64; 3],
    /// Body-to-ENU unit quaternion, component order `[x, y, z, w]`.
    pub orientation_xyzw: [f64; 4],
}

/// State sampled from the FDM immediately after a step or reset.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AircraftState {
    pub geodetic: GeodeticPosition,
    /// Terrain-relative height is logged separately and never used as a
    /// substitute for geodetic ellipsoid height.
    pub height_agl_m: f64,
    pub local_enu: Option<LocalEnuPose>,
    pub roll_rad: f64,
    pub pitch_rad: f64,
    pub heading_rad: f64,
    pub angular_rate_rps: [f64; 3],
    pub calibrated_airspeed_mps: f64,
    pub vertical_speed_mps: f64,
    /// Normalized animation state consumed by the renderer.
    pub gear_position: f64,
    pub flap_position: f64,
    pub spoiler_position: f64,
    pub elevator_position: f64,
    pub aileron_position: f64,
    pub rudder_position: f64,
    pub engine_n1: f64,
    pub weight_on_wheels: bool,
    /// Pitch trim position, normalized elevator travel, + = nose up. Stamped
    /// by [`SimCore`] (which owns the trim), not by the backend.
    pub pitch_trim: f64,
    /// `fcs/speedbrake-pos-norm` (0 when the aircraft has none).
    pub speedbrake_position: f64,
    /// The levers the core held for this state (pilot's selection, before
    /// the squat switch). Stamped by [`SimCore`], not by the backend.
    pub levers: Levers,
    /// Autopilot modes and targets. Stamped by [`SimCore`].
    pub autopilot: ApStatus,
}

/// Immutable renderer/telemetry handoff. `sequence` is never reused;
/// `generation` changes on reset so consumers never interpolate across one.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AircraftSnapshot {
    pub sequence: u64,
    pub generation: u64,
    pub simulation_tick: u64,
    pub simulation_time_ns: u64,
    pub published_at_ns: u64,
    /// Input status and loss duration at publication, for the on-screen
    /// warning (INPUT-002).
    pub input: InputHealth,
    /// `Some` when the core was not running at publication (reset and
    /// status snapshots). `InputLoss` distinguishes the automatic pause.
    pub pause_reason: Option<PauseReason>,
    pub state: AircraftState,
}

/// Continuous input loss after which a running core pauses itself
/// (INPUT-002), measured on the simulation clock.
pub const INPUT_LOSS_PAUSE_AFTER_NS: u64 = 10_000_000_000;

/// Input status as shown to the user. Anything but `Valid` is a loss.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum InputStatus {
    Valid,
    /// Nothing published yet.
    #[default]
    NoInput,
    /// Disconnected, queue loss, or awaiting resync.
    Unavailable(UnavailableReason),
    /// No new publication for more than the stale bound (100 ms).
    Stale,
}

impl From<ControlStatus> for InputStatus {
    fn from(status: ControlStatus) -> Self {
        match status {
            ControlStatus::Valid => Self::Valid,
            ControlStatus::NoInput => Self::NoInput,
            ControlStatus::Stale { .. } => Self::Stale,
            ControlStatus::Unavailable(reason) => Self::Unavailable(reason),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct InputHealth {
    pub status: InputStatus,
    /// Whole milliseconds of continuous loss on the simulation clock; 0 when
    /// `Valid`.
    pub loss_ms: u64,
}

impl InputHealth {
    /// True when the renderer should show the input-loss warning.
    pub fn warning(&self) -> bool {
        self.status != InputStatus::Valid
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PauseReason {
    /// The pilot's Pause command.
    User,
    /// Reset always leaves the core paused.
    Reset,
    /// INPUT-002: 10 s of continuous input loss. Lifted only by Start with
    /// valid input.
    InputLoss,
    InvalidWorldPose,
}

/// Backend boundary. A value implementing this trait is constructed and owned
/// by the simulation thread. JSBSim enters only through `JsbsimBackend`.
pub trait SimulationBackend {
    type Error;

    fn reset(&mut self) -> Result<AircraftState, Self::Error>;
    fn step(
        &mut self,
        controls: ControlInput,
        dt_seconds: f64,
    ) -> Result<AircraftState, Self::Error>;

    /// Apply the pitch trim position ([`trim::PitchTrim`], + = nose up)
    /// before the next [`SimulationBackend::step`]. [`SimCore`] calls it
    /// before every step. Backends without a trim input ignore it.
    fn set_pitch_trim(&mut self, _position: f64) {}
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RunState {
    Stopped,
    Running,
    Paused,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    Start,
    Pause,
    Reset,
}

#[derive(Debug, PartialEq)]
pub enum CoreError<E> {
    Backend(E),
    InvalidWorldPose,
}

/// Single-thread-owned state machine. Time and sleeping are deliberately
/// outside this type so tests and platform loops supply a monotonic clock.
///
/// # Loss-of-input policy (INPUT-002)
///
/// The platform loop calls [`SimCore::update_input`] at every tick boundary,
/// before commands and [`SimCore::step`], with the mailbox status.
/// - Any status other than `Valid` is a **loss**. Its duration is measured on
///   the simulation clock from the first tick boundary that reported it
///   (a `Stale` status is itself first reported 100 ms after the last
///   publication). Controls are neutral (the mailbox guarantees it) and the
///   core keeps stepping.
/// - Every snapshot carries [`InputHealth`] (status and loss duration in ms)
///   and the pause reason, so the renderer can warn. While the core is not
///   running it publishes nothing per tick, so a **status snapshot** (the
///   last state again, same tick and generation, new sequence) is produced
///   whenever the status kind or pause reason changes; that is how the
///   warning clears on screen while paused.
/// - After [`INPUT_LOSS_PAUSE_AFTER_NS`] (10 s) of continuous loss a
///   running core pauses itself with [`PauseReason::InputLoss`]. At exactly
///   10 s it is paused; one tick earlier it still steps.
/// - Return of valid input clears the warning and the loss timer but never
///   resumes the core. Only an explicit `Start` does, and `Start` is refused
///   (counted in [`SimCore::refused_starts`]) unless the current input
///   status is `Valid`. Upstream, the control mailbox also withholds any
///   Start pressed before the current valid stretch.
/// - `Pause` and `Reset` are honoured during a loss (both are safe); a
///   reset does not restart the loss timer.
pub struct SimCore<B> {
    backend: B,
    state: RunState,
    pause_reason: Option<PauseReason>,
    tick: u64,
    generation: u64,
    next_sequence: u64,
    input: InputHealth,
    loss_since_ns: Option<u64>,
    last_state: Option<AircraftState>,
    refused_starts: u64,
    trim: PitchTrim,
    levers: Levers,
    initial_levers: Levers,
    autopilot: Autopilot,
    approaches: Vec<Approach>,
    /// The controls the last step gave the backend (after the autopilot).
    applied: ControlInput,
}

impl<B: SimulationBackend> SimCore<B> {
    pub fn new(backend: B) -> Self {
        Self::with_levers(backend, Levers::default())
    }

    /// A core whose levers start at, and Reset returns to, `initial` (the
    /// aircraft's parked configuration, matching the I/O side's defaults).
    pub fn with_levers(backend: B, initial: Levers) -> Self {
        let initial = initial.normalized();
        Self {
            backend,
            state: RunState::Stopped,
            pause_reason: None,
            tick: 0,
            generation: 0,
            next_sequence: 0,
            input: InputHealth { status: InputStatus::NoInput, loss_ms: 0 },
            loss_since_ns: None,
            last_state: None,
            refused_starts: 0,
            trim: PitchTrim::default(),
            levers: initial,
            initial_levers: initial,
            autopilot: Autopilot::default(),
            approaches: Vec::new(),
            applied: ControlInput::default(),
        }
    }

    /// The runway ends the autopilot's ILS can fly (world frame).
    pub fn with_approaches(mut self, approaches: Vec<Approach>) -> Self {
        self.approaches = approaches;
        self
    }

    pub fn approaches(&self) -> &[Approach] { &self.approaches }

    /// Fly the autopilot with `gains` instead of [`autopilot::Gains::DEFAULT`]
    /// (the board tuning cases). Turns the autopilot off.
    pub fn with_autopilot_gains(mut self, gains: autopilot::Gains) -> Self {
        self.autopilot = Autopilot::with_gains(gains);
        self
    }

    pub fn state(&self) -> RunState { self.state }
    pub fn simulation_tick(&self) -> u64 { self.tick }
    pub fn generation(&self) -> u64 { self.generation }
    /// `None` unless paused.
    pub fn pause_reason(&self) -> Option<PauseReason> { self.pause_reason }
    pub fn input_health(&self) -> InputHealth { self.input }
    pub fn refused_starts(&self) -> u64 { self.refused_starts }
    /// Current pitch trim position (+ = nose up).
    pub fn pitch_trim(&self) -> f64 { self.trim.position() }
    /// The held gear/flap/speedbrake levers.
    pub fn levers(&self) -> Levers { self.levers }
    pub fn autopilot(&self) -> ApStatus { self.autopilot.status() }
    /// The controls the last step gave the backend: the pilot's, with roll,
    /// pitch and throttle replaced while the autopilot or autothrottle flies,
    /// and the levers as held and squat-switched.
    pub fn applied_controls(&self) -> ControlInput { self.applied }

    /// Record the input status for this tick boundary and apply the 10 s
    /// loss pause. Returns a status snapshot to publish when the core is not
    /// running and the visible status or pause reason changed.
    pub fn update_input(
        &mut self,
        status: InputStatus,
        monotonic_now_ns: u64,
    ) -> Option<AircraftSnapshot> {
        let before = (self.input.status, self.pause_reason, self.state);
        if status == InputStatus::Valid {
            self.loss_since_ns = None;
            self.input = InputHealth { status, loss_ms: 0 };
        } else {
            let since = *self.loss_since_ns.get_or_insert(monotonic_now_ns);
            let lost_ns = monotonic_now_ns.saturating_sub(since);
            self.input = InputHealth { status, loss_ms: lost_ns / 1_000_000 };
            if self.state == RunState::Running && lost_ns >= INPUT_LOSS_PAUSE_AFTER_NS {
                self.pause(PauseReason::InputLoss);
            }
        }
        let after = (self.input.status, self.pause_reason, self.state);
        if self.state != RunState::Running && before != after {
            let state = self.last_state?;
            return Some(self.snapshot(state, monotonic_now_ns));
        }
        None
    }

    /// Apply a command at a tick boundary. Reset leaves the core paused and
    /// returns the reset state for immediate publication. Start is refused
    /// unless the input status is `Valid` (INPUT-002).
    pub fn command(
        &mut self,
        command: Command,
        monotonic_now_ns: u64,
    ) -> Result<Option<AircraftSnapshot>, CoreError<B::Error>> {
        match command {
            Command::Start => {
                if self.input.status != InputStatus::Valid {
                    self.refused_starts += 1;
                    return Ok(None);
                }
                self.state = RunState::Running;
                self.pause_reason = None;
                Ok(None)
            }
            Command::Pause => {
                if self.state != RunState::Running { return Ok(None); }
                self.pause(PauseReason::User);
                // A pause is visible even though the FDM does not advance.
                // Publish the last state so readers do not retain a snapshot
                // which incorrectly says the simulation is still running.
                let state = self.last_state;
                Ok(state.map(|state| self.snapshot(state, monotonic_now_ns)))
            }
            Command::Reset => {
                let state = self.backend.reset().map_err(CoreError::Backend)?;
                // Trim is aircraft state: Reset returns it to neutral.
                self.trim.reset();
                self.backend.set_pitch_trim(self.trim.position());
                // So are the levers (the I/O side resets its own on the
                // same press).
                self.levers = self.initial_levers;
                self.autopilot.reset();
                self.applied = ControlInput::default();
                if validate_world_pose(&state).is_err() {
                    self.pause(PauseReason::InvalidWorldPose);
                    return Err(CoreError::InvalidWorldPose);
                }
                self.tick = 0;
                self.generation = self.generation.wrapping_add(1);
                self.pause(PauseReason::Reset);
                Ok(Some(self.snapshot(state, monotonic_now_ns)))
            }
        }
    }

    /// Advance exactly one 120 Hz step only while running.
    pub fn step(
        &mut self,
        controls: ControlInput,
        monotonic_now_ns: u64,
    ) -> Result<Option<AircraftSnapshot>, CoreError<B::Error>> {
        let mut controls = controls.normalized();
        // Levers are positions: take them only from valid input, hold them
        // through a loss (whose controls are neutral), also while paused.
        let valid = self.input.status == InputStatus::Valid;
        if valid {
            self.levers = controls.levers;
            // The panel, like the levers, only from valid input.
            self.autopilot.input(controls.ap, self.last_state.as_ref(), &controls, &self.approaches);
        }
        if self.state != RunState::Running { return Ok(None); }
        let wow = self.last_state.is_some_and(|s| s.weight_on_wheels);
        controls.levers = self.levers.effective(wow);
        // The trim runs only while stepping, from the airspeed of the state
        // this step starts from; a released (or lost, neutral) switch holds it.
        let ias = self.last_state.map_or(0.0, |s| s.calibrated_airspeed_mps);
        self.trim.update(controls.trim, ias, STEP_SECONDS);
        self.backend.set_pitch_trim(self.trim.position());
        // The autopilot flies from the state this step starts from.
        if let Some(s) = self.last_state {
            self.autopilot.step(&mut controls, &s, &self.approaches, valid, STEP_SECONDS);
        }
        self.applied = controls;
        let state = self.backend
            .step(controls, STEP_SECONDS)
            .map_err(CoreError::Backend)?;
        if validate_world_pose(&state).is_err() {
            self.pause(PauseReason::InvalidWorldPose);
            return Err(CoreError::InvalidWorldPose);
        }
        self.tick = self.tick.saturating_add(1);
        Ok(Some(self.snapshot(state, monotonic_now_ns)))
    }

    fn pause(&mut self, reason: PauseReason) {
        self.state = RunState::Paused;
        self.pause_reason = Some(reason);
    }

    fn snapshot(&mut self, mut state: AircraftState, now_ns: u64) -> AircraftSnapshot {
        state.pitch_trim = self.trim.position();
        state.levers = self.levers;
        state.autopilot = self.autopilot.status();
        self.last_state = Some(state);
        self.next_sequence = self.next_sequence.wrapping_add(1);
        AircraftSnapshot {
            sequence: self.next_sequence,
            generation: self.generation,
            simulation_tick: self.tick,
            simulation_time_ns: FixedRate::elapsed_ns(SIM_HZ, self.tick),
            published_at_ns: now_ns,
            input: self.input,
            pause_reason: if self.state == RunState::Running { None } else { self.pause_reason },
            state,
        }
    }
}

fn validate_world_pose(state: &AircraftState) -> Result<(), ()> {
    let g = state.geodetic;
    let local = state.local_enu.ok_or(())?;
    if local.frame_id == 0
        || !g.latitude_deg.is_finite()
        || !(-90.0..=90.0).contains(&g.latitude_deg)
        || !g.longitude_deg.is_finite()
        || !(-180.0..=180.0).contains(&g.longitude_deg)
        || !g.ellipsoid_height_m.is_finite()
        || !state.height_agl_m.is_finite()
        || !local.position_m.into_iter().all(f64::is_finite)
        || !local.velocity_mps.into_iter().all(f64::is_finite)
        || !local.orientation_xyzw.into_iter().all(f64::is_finite)
        || !(0.999..=1.001).contains(&local.orientation_xyzw.into_iter().map(|v| v * v).sum::<f64>())
    {
        return Err(());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Default)]
    struct Fake { steps: u64, resets: u64, last: ControlInput, trim: f64, airborne: bool }
    impl SimulationBackend for Fake {
        type Error = ();
        fn reset(&mut self) -> Result<AircraftState, Self::Error> {
            self.resets += 1;
            self.steps = 0;
            Ok(self.at(valid_state(10.0)))
        }
        fn step(&mut self, c: ControlInput, dt: f64) -> Result<AircraftState, Self::Error> {
            assert_eq!(dt, STEP_SECONDS);
            self.last = c;
            self.steps += 1;
            Ok(self.at(valid_state((self.steps % 80) as f64)))
        }
        fn set_pitch_trim(&mut self, position: f64) {
            self.trim = position;
        }
    }

    impl Fake {
        /// 1000 m up when airborne (the autopilot engages from 200 ft).
        fn at(&self, mut s: AircraftState) -> AircraftState {
            if self.airborne {
                s.height_agl_m = 1000.0;
            }
            s
        }
    }

    fn valid_state(latitude_deg: f64) -> AircraftState {
        AircraftState {
            geodetic: GeodeticPosition {
                datum: GeodeticDatum::Wgs84Ellipsoid,
                latitude_deg,
                longitude_deg: -122.0,
                ellipsoid_height_m: 10.0,
            },
            height_agl_m: 5.0,
            local_enu: Some(LocalEnuPose {
                frame_id: 1,
                frame_generation: 1,
                position_m: [1.0, 2.0, 3.0],
                velocity_mps: [4.0, 5.0, 6.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            }),
            ..Default::default()
        }
    }

    #[test]
    fn pause_and_reset_are_tick_boundary_transitions() {
        let mut core = SimCore::new(Fake::default());
        assert_eq!(core.step(ControlInput::default(), 1).unwrap(), None);
        core.update_input(InputStatus::Valid, 2);
        core.command(Command::Start, 2).unwrap();
        let first = core.step(ControlInput::default(), 3).unwrap().unwrap();
        assert_eq!((first.simulation_tick, first.generation), (1, 0));
        let paused = core.command(Command::Pause, 4).unwrap().unwrap();
        assert_eq!(paused.simulation_tick, first.simulation_tick);
        assert_eq!(paused.state, first.state);
        assert_eq!(paused.pause_reason, Some(PauseReason::User));
        assert!(paused.sequence > first.sequence);
        assert_eq!(core.step(ControlInput::default(), 5).unwrap(), None);
        let reset = core.command(Command::Reset, 6).unwrap().unwrap();
        assert_eq!(core.state(), RunState::Paused);
        assert_eq!((reset.simulation_tick, reset.generation), (0, 1));
        assert_eq!(reset.state.geodetic.latitude_deg, 10.0);
        core.command(Command::Start, 7).unwrap();
        let after = core.step(ControlInput::default(), 8).unwrap().unwrap();
        assert_eq!((after.simulation_tick, after.generation), (1, 1));
        assert!(after.sequence > reset.sequence);
    }

    #[test]
    fn controls_are_sanitized_before_backend() {
        let c = ControlInput {
            roll: 4.0, pitch: f32::NAN, yaw: -3.0, throttle: 2.0,
            left_brake: -1.0, right_brake: f32::INFINITY, trim: f32::NAN,
            levers: Levers { gear_down: false, flaps: 7.0, speedbrake: f32::NAN },
            ap: ApInput::default(),
        }.normalized();
        assert_eq!(c, ControlInput {
            roll: 1.0, pitch: 0.0, yaw: -1.0, throttle: 1.0,
            left_brake: 0.0, right_brake: 0.0, trim: 0.0,
            levers: Levers { gear_down: false, flaps: 1.0, speedbrake: 0.0 },
            ap: ApInput::default(),
        });
    }

    #[test]
    fn invalid_world_pose_fails_closed() {
        let mut state = valid_state(37.0);
        state.local_enu = None;
        struct Invalid(AircraftState);
        impl SimulationBackend for Invalid {
            type Error = ();
            fn reset(&mut self) -> Result<AircraftState, ()> { Ok(self.0) }
            fn step(&mut self, _: ControlInput, _: f64) -> Result<AircraftState, ()> { Ok(self.0) }
        }
        let mut core = SimCore::new(Invalid(state));
        assert_eq!(core.command(Command::Reset, 1), Err(CoreError::InvalidWorldPose));
        assert_eq!(core.state(), RunState::Paused);
    }

    const LOST: InputStatus = InputStatus::Unavailable(UnavailableReason::ResyncRequired);
    const T0: u64 = 1_000_000_000;

    fn running_core() -> SimCore<Fake> {
        let mut core = SimCore::new(Fake::default());
        assert_eq!(core.update_input(InputStatus::Valid, 0), None);
        core.command(Command::Start, 0).unwrap();
        assert_eq!(core.step(ControlInput::default(), 1).unwrap().unwrap().input.status, InputStatus::Valid);
        core
    }

    #[test]
    fn input_loss_pauses_at_exactly_ten_seconds_on_the_sim_clock() {
        let mut core = running_core();
        assert_eq!(core.update_input(LOST, T0), None);
        let just_before = T0 + INPUT_LOSS_PAUSE_AFTER_NS - 1;
        assert_eq!(core.update_input(LOST, just_before), None);
        assert_eq!(core.state(), RunState::Running);
        let s = core.step(ControlInput::default(), just_before).unwrap().unwrap();
        assert_eq!(s.input, InputHealth { status: LOST, loss_ms: 9_999 });
        assert_eq!(s.pause_reason, None);

        let at = T0 + INPUT_LOSS_PAUSE_AFTER_NS;
        let status = core.update_input(LOST, at).expect("pause publishes a status snapshot");
        assert_eq!((core.state(), core.pause_reason()), (RunState::Paused, Some(PauseReason::InputLoss)));
        assert_eq!(status.pause_reason, Some(PauseReason::InputLoss));
        assert_eq!(status.input, InputHealth { status: LOST, loss_ms: 10_000 });
        assert_eq!(status.simulation_tick, s.simulation_tick, "status snapshot repeats the state");
        assert!(status.sequence > s.sequence);
        assert_eq!(core.step(ControlInput::default(), at).unwrap(), None, "no step once paused");
    }

    #[test]
    fn warning_is_carried_while_lost_and_cleared_when_input_returns() {
        let mut core = running_core();
        core.update_input(InputStatus::Stale, T0);
        let s = core.step(ControlInput::default(), T0 + 250_000_000).unwrap().unwrap();
        assert!(s.input.warning());
        assert_eq!(s.input.status, InputStatus::Stale);
        core.update_input(InputStatus::Stale, T0 + 250_000_000);
        let s = core.step(ControlInput::default(), T0 + 250_000_000).unwrap().unwrap();
        assert_eq!(s.input.loss_ms, 250);
        assert_eq!(core.update_input(InputStatus::Valid, T0 + 300_000_000), None);
        let s = core.step(ControlInput::default(), T0 + 300_000_000).unwrap().unwrap();
        assert_eq!(s.input, InputHealth { status: InputStatus::Valid, loss_ms: 0 });
        assert!(!s.input.warning());
        // A new loss restarts the timer from zero.
        core.update_input(LOST, T0 + 400_000_000);
        core.update_input(LOST, T0 + 400_000_000 + INPUT_LOSS_PAUSE_AFTER_NS - 1);
        assert_eq!(core.state(), RunState::Running);
    }

    #[test]
    fn returning_input_clears_the_warning_but_never_unpauses() {
        let mut core = running_core();
        core.update_input(LOST, T0);
        core.update_input(LOST, T0 + INPUT_LOSS_PAUSE_AFTER_NS).unwrap();
        let cleared = core
            .update_input(InputStatus::Valid, T0 + INPUT_LOSS_PAUSE_AFTER_NS + 1)
            .expect("warning change while paused publishes a status snapshot");
        assert_eq!(cleared.input, InputHealth { status: InputStatus::Valid, loss_ms: 0 });
        assert_eq!(cleared.pause_reason, Some(PauseReason::InputLoss));
        for t in 0..1_000 {
            assert_eq!(core.update_input(InputStatus::Valid, T0 + INPUT_LOSS_PAUSE_AFTER_NS + 2 + t), None);
            assert_eq!(core.step(ControlInput::default(), 0).unwrap(), None);
        }
        assert_eq!((core.state(), core.pause_reason()), (RunState::Paused, Some(PauseReason::InputLoss)));
    }

    #[test]
    fn start_unpauses_an_input_loss_pause_only_with_valid_input() {
        let mut core = running_core();
        core.update_input(LOST, T0);
        core.update_input(LOST, T0 + INPUT_LOSS_PAUSE_AFTER_NS);
        core.command(Command::Start, T0 + INPUT_LOSS_PAUSE_AFTER_NS).unwrap();
        assert_eq!(core.state(), RunState::Paused, "Start while lost is refused");
        core.update_input(InputStatus::Stale, T0 + INPUT_LOSS_PAUSE_AFTER_NS + 1);
        core.command(Command::Start, T0 + INPUT_LOSS_PAUSE_AFTER_NS + 1).unwrap();
        assert_eq!(core.state(), RunState::Paused, "Start while stale is refused");
        assert_eq!(core.refused_starts(), 2);
        core.update_input(InputStatus::Valid, T0 + INPUT_LOSS_PAUSE_AFTER_NS + 2);
        core.command(Command::Start, T0 + INPUT_LOSS_PAUSE_AFTER_NS + 2).unwrap();
        assert_eq!((core.state(), core.pause_reason()), (RunState::Running, None));
        let s = core.step(ControlInput::default(), T0 + INPUT_LOSS_PAUSE_AFTER_NS + 3).unwrap().unwrap();
        assert_eq!((s.input.status, s.pause_reason), (InputStatus::Valid, None));
    }

    #[test]
    fn user_pause_is_distinct_and_stays_user_through_a_long_loss() {
        let mut core = running_core();
        let paused = core.command(Command::Pause, 2).unwrap().unwrap();
        assert_eq!(paused.pause_reason, Some(PauseReason::User));
        assert_eq!(core.pause_reason(), Some(PauseReason::User));
        core.update_input(LOST, T0);
        core.update_input(LOST, T0 + 2 * INPUT_LOSS_PAUSE_AFTER_NS);
        assert_eq!(core.pause_reason(), Some(PauseReason::User));
    }

    #[test]
    fn reset_during_a_loss_resets_keeps_warning_and_timer_and_start_stays_refused() {
        let mut core = running_core();
        core.update_input(LOST, T0);
        core.update_input(LOST, T0 + 4_000_000_000);
        let reset = core.command(Command::Reset, T0 + 4_000_000_000).unwrap().unwrap();
        assert_eq!((reset.generation, reset.simulation_tick), (1, 0));
        assert_eq!(reset.input, InputHealth { status: LOST, loss_ms: 4_000 });
        assert_eq!(reset.pause_reason, Some(PauseReason::Reset));
        core.command(Command::Start, T0 + 4_000_000_000).unwrap();
        assert_eq!(core.state(), RunState::Paused);
        // Timer not restarted by the reset; reason stays Reset (not running).
        core.update_input(LOST, T0 + INPUT_LOSS_PAUSE_AFTER_NS);
        assert_eq!(core.input_health().loss_ms, 10_000);
        assert_eq!(core.pause_reason(), Some(PauseReason::Reset));
        let back = core.update_input(InputStatus::Valid, T0 + INPUT_LOSS_PAUSE_AFTER_NS + 1).unwrap();
        assert_eq!((back.generation, back.simulation_tick, back.input.status), (1, 0, InputStatus::Valid));
        core.command(Command::Start, T0 + INPUT_LOSS_PAUSE_AFTER_NS + 1).unwrap();
        assert_eq!(core.state(), RunState::Running);
    }

    fn trim_cmd(trim: f32) -> ControlInput {
        ControlInput { trim, ..ControlInput::default() }
    }

    #[test]
    fn pitch_trim_runs_while_held_stops_on_release_and_is_published() {
        let mut core = running_core();
        let mut last = None;
        for t in 0..120 {
            last = core.step(trim_cmd(1.0), 10 + t).unwrap();
        }
        let s = last.unwrap();
        let want = trim::RATE_LOW_SPEED; // 1 s at the low-speed rate (IAS 0 in the fake)
        assert!((s.state.pitch_trim - want).abs() < 1e-9, "{}", s.state.pitch_trim);
        assert!((core.backend.trim - want).abs() < 1e-9, "backend receives the position");
        for t in 0..240 {
            core.step(trim_cmd(0.0), 200 + t).unwrap();
        }
        assert!((core.pitch_trim() - want).abs() < 1e-9, "released switch holds the trim");
    }

    #[test]
    fn input002_loss_leaves_trim_unchanged_and_pause_freezes_it() {
        let mut core = running_core();
        for t in 0..60 {
            core.step(trim_cmd(-1.0), 10 + t).unwrap();
        }
        let before = core.pitch_trim();
        assert!(before < 0.0);
        // Loss: the mailbox delivers neutral controls (trim switch 0).
        core.update_input(LOST, T0);
        for t in 0..600 {
            let s = core.step(ControlInput::default(), T0 + t).unwrap().unwrap();
            assert_eq!(s.state.pitch_trim, before);
        }
        assert_eq!(core.backend.trim, before, "trim keeps acting during the loss");
        core.update_input(LOST, T0 + INPUT_LOSS_PAUSE_AFTER_NS);
        assert_eq!(core.state(), RunState::Paused);
        // Paused: no step, so a held switch cannot move it.
        assert_eq!(core.step(trim_cmd(1.0), T0 + INPUT_LOSS_PAUSE_AFTER_NS + 1).unwrap(), None);
        assert_eq!(core.pitch_trim(), before);
    }

    #[test]
    fn reset_returns_trim_to_neutral() {
        let mut core = running_core();
        for t in 0..120 {
            core.step(trim_cmd(1.0), 10 + t).unwrap();
        }
        assert!(core.pitch_trim() > 0.0);
        let paused = core.command(Command::Pause, 300).unwrap().unwrap();
        assert_eq!(paused.state.pitch_trim, core.pitch_trim(), "pause keeps it");
        let reset = core.command(Command::Reset, 301).unwrap().unwrap();
        assert_eq!(reset.state.pitch_trim, 0.0);
        assert_eq!(core.backend.trim, 0.0);
    }

    fn lever_cmd(gear_down: bool, flaps: f32, speedbrake: f32) -> ControlInput {
        ControlInput { levers: Levers { gear_down, flaps, speedbrake }, ..ControlInput::default() }
    }

    #[test]
    fn levers_reach_the_backend_and_the_snapshot() {
        let mut core = running_core();
        let s = core.step(lever_cmd(true, 0.4, 0.5), 10).unwrap().unwrap();
        assert_eq!(core.backend.last.levers, Levers { gear_down: true, flaps: 0.4, speedbrake: 0.5 });
        assert_eq!(s.state.levers, core.levers());
    }

    #[test]
    fn input002_loss_holds_the_levers_and_never_raises_the_gear() {
        let initial = Levers { gear_down: true, flaps: 0.2, speedbrake: 0.0 };
        let mut core = SimCore::with_levers(Fake::default(), initial);
        core.update_input(InputStatus::Valid, 0);
        core.command(Command::Start, 0).unwrap();
        core.step(lever_cmd(false, 0.6, 1.0), 10).unwrap();
        let held = core.levers();
        assert_eq!(held, Levers { gear_down: false, flaps: 0.6, speedbrake: 1.0 });
        // Loss: the mailbox delivers neutral controls (ControlInput::default).
        core.update_input(LOST, T0);
        let s = core.step(ControlInput::default(), T0 + 1).unwrap().unwrap();
        assert_eq!(s.state.levers, held, "held through the loss");
        assert_eq!(core.backend.last.levers.flaps, 0.6);
        // Valid input again: the levers follow the stick side once more.
        core.update_input(InputStatus::Valid, T0 + 2);
        core.step(lever_cmd(true, 0.0, 0.0), T0 + 3).unwrap();
        assert_eq!(core.levers(), Levers { gear_down: true, flaps: 0.0, speedbrake: 0.0 });
    }

    #[test]
    fn squat_switch_keeps_the_gear_down_until_lift_off() {
        let mut core = running_core();
        let mut on_ground = valid_state(10.0);
        on_ground.weight_on_wheels = true;
        core.last_state = Some(on_ground);
        core.step(lever_cmd(false, 0.0, 0.0), 10).unwrap();
        assert!(core.backend.last.levers.gear_down, "weight on wheels: commanded down");
        assert!(!core.levers().gear_down, "the lever itself stays up");
        // The Fake reports no weight on wheels after its step: airborne.
        core.step(lever_cmd(false, 0.0, 0.0), 11).unwrap();
        assert!(!core.backend.last.levers.gear_down, "airborne: the gear retracts");
    }

    #[test]
    fn levers_move_while_paused_and_reset_restores_the_initial_set() {
        let initial = Levers { gear_down: true, flaps: 0.5, speedbrake: 0.0 };
        let mut core = SimCore::with_levers(Fake::default(), initial);
        core.update_input(InputStatus::Valid, 0);
        assert_eq!(core.step(lever_cmd(true, 1.0, 0.5), 1).unwrap(), None, "stopped: no step");
        assert_eq!(core.levers().flaps, 1.0, "but the lever moved");
        let reset = core.command(Command::Reset, 2).unwrap().unwrap();
        assert_eq!(core.levers(), initial);
        assert_eq!(reset.state.levers, initial);
    }

    use autopilot::{ApAction, Lateral};

    /// A running core over an airborne Fake (1000 m, so the autopilot can
    /// engage).
    fn airborne_core() -> SimCore<Fake> {
        let mut core = SimCore::new(Fake { airborne: true, ..Fake::default() });
        core.update_input(InputStatus::Valid, 0);
        core.command(Command::Start, 0).unwrap();
        core.step(ControlInput::default(), 1).unwrap().unwrap();
        core
    }

    fn panel(serial: u32, action: ApAction) -> ControlInput {
        ControlInput {
            throttle: 0.5,
            ap: ApInput { serial, action, heading_deg: 270.0, altitude_ft: 5000.0, speed_kt: 200.0 },
            ..ControlInput::default()
        }
    }

    #[test]
    fn autopilot_actions_apply_once_per_serial() {
        let mut core = airborne_core();
        let s = core.step(panel(1, ApAction::Master), 10).unwrap().unwrap();
        assert!(s.state.autopilot.engaged, "the snapshot carries the engaged autopilot");
        // Every later publication repeats serial 1: the toggle does not repeat.
        for t in 0..240 {
            core.step(panel(1, ApAction::Master), 11 + t).unwrap();
        }
        assert!(core.autopilot().engaged);
        core.step(panel(2, ApAction::Master), 300).unwrap();
        assert!(!core.autopilot().engaged, "a new serial applies its action once");
        core.step(panel(3, ApAction::Speed), 301).unwrap();
        for t in 0..240 {
            core.step(panel(3, ApAction::Speed), 302 + t).unwrap();
        }
        assert!(core.autopilot().autothrottle, "A/T toggled on once, not back off");
    }

    #[test]
    fn input002_loss_holds_the_panel_and_the_autopilot_keeps_flying() {
        let mut core = airborne_core();
        core.step(panel(1, ApAction::Heading), 10).unwrap();
        core.step(panel(2, ApAction::Speed), 11).unwrap();
        let before = core.autopilot();
        assert!(before.engaged && before.autothrottle && before.lateral == Lateral::Heading);
        // Loss: the mailbox delivers neutral controls (ControlInput::default,
        // panel serial 0 and heading 000), which is neither a click nor an
        // override.
        core.update_input(LOST, T0);
        for t in 0..600 {
            let s = core.step(ControlInput::default(), T0 + t).unwrap().unwrap();
            let st = s.state.autopilot;
            assert!(st.engaged && st.autothrottle && st.lateral == Lateral::Heading, "{st:?}");
            assert_eq!(st.input, before.input, "the panel targets are held");
        }
        // Still flying: heading 000 -> 270 rolls left; the A/T (Fake IAS 0)
        // drives the throttle up from the pilot's 0.5, not the loss's 0.
        let applied = core.applied_controls();
        assert!(applied.roll < 0.0, "{applied:?}");
        assert!(applied.throttle > 0.5, "{applied:?}");
        assert_eq!(core.backend.last, applied, "the backend gets the autopilot's commands");
        // Valid input again with the same serial: nothing re-applies.
        core.update_input(InputStatus::Valid, T0 + 700);
        core.step(panel(2, ApAction::Speed), T0 + 700).unwrap();
        assert!(core.autopilot().autothrottle && core.autopilot().engaged);
    }

    #[test]
    fn r4_input_loss_with_the_autopilot_off_engages_lvl_until_the_stick_overrides() {
        use autopilot::{ApWarning, Vertical};
        let mut core = airborne_core();
        core.step(panel(0, ApAction::None), 10).unwrap();
        assert!(!core.autopilot().engaged);
        core.update_input(LOST, T0);
        let s = core.step(ControlInput::default(), T0).unwrap().unwrap();
        let st = s.state.autopilot;
        assert!(st.engaged && st.lateral == Lateral::WingsLevel && !st.autothrottle, "{st:?}");
        assert!(matches!(st.vertical, Vertical::LevelOff | Vertical::AltHold), "{st:?}");
        assert_eq!(st.warning, Some(ApWarning::InputLossLevel));
        // Valid input back (stick neutral): still engaged, then a firm stick
        // input disconnects it.
        core.update_input(InputStatus::Valid, T0 + 1);
        core.step(panel(0, ApAction::None), T0 + 1).unwrap();
        assert!(core.autopilot().engaged);
        core.step(ControlInput { pitch: 0.5, ..panel(0, ApAction::None) }, T0 + 2).unwrap();
        assert!(!core.autopilot().engaged);
        assert_eq!(core.autopilot().warning, Some(ApWarning::ApDisconnect));
    }

    #[test]
    fn r4_input_loss_on_the_ground_does_not_engage() {
        let mut core = running_core();
        core.update_input(LOST, T0);
        for t in 0..120 {
            let s = core.step(ControlInput::default(), T0 + t).unwrap().unwrap();
            assert!(!s.state.autopilot.engaged && s.state.autopilot.warning.is_none(), "5 m up: below MIN_ENGAGE_FT");
        }
    }

    #[test]
    fn reset_clears_the_autopilot_and_never_replays_a_click() {
        let mut core = airborne_core();
        core.step(panel(1, ApAction::Master), 10).unwrap();
        core.step(panel(2, ApAction::Speed), 11).unwrap();
        assert!(core.autopilot().engaged && core.autopilot().autothrottle);
        let reset = core.command(Command::Reset, 12).unwrap().unwrap();
        let st = reset.state.autopilot;
        assert!(!st.engaged && !st.autothrottle && st.warning.is_none(), "{st:?}");
        // A click read in the Reset's tick (commands run before the step), and
        // the clicks before it, never apply after the reset, paused or running.
        core.step(panel(3, ApAction::Level), 12).unwrap();
        core.command(Command::Start, 13).unwrap();
        for t in 0..10 {
            core.step(panel(3, ApAction::Level), 14 + t).unwrap();
        }
        let st = core.autopilot();
        assert!(!st.engaged && st.warning.is_none(), "{st:?}");
        // The next click does.
        core.step(panel(4, ApAction::Level), 30).unwrap();
        assert!(core.autopilot().engaged);
    }
}
