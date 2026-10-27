//! `gx-fdm` adapter. Construction binds property handles once; stepping uses
//! only handle-based calls. The contained `Fdm` therefore remains owned by the
//! same thread as `JsbsimBackend`.
//!
//! Control signs: see [`crate::fdm_signs`].

use crate::fdm_signs::JsbsimCommands;
use crate::{AircraftState, ControlInput, GeoreferenceError, SimulationBackend, Wgs84EnuFrame, STEP_SECONDS};
use gx_fdm::{Fdm, FdmError, Prop, Step};
use std::fmt;
use std::path::Path;

const FT_TO_M: f64 = 0.3048;
const KNOT_TO_MPS: f64 = 0.514_444;

#[derive(Debug)]
pub enum JsbsimError {
    Fdm(FdmError),
    Georeference(GeoreferenceError),
    Terminated,
    InvalidSetup(&'static str),
}

impl fmt::Display for JsbsimError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Fdm(e) => e.fmt(f),
            Self::Georeference(e) => write!(f, "invalid JSBSim/world georeference: {e:?}"),
            Self::Terminated => f.write_str("JSBSim terminated the simulation"),
            Self::InvalidSetup(what) => write!(f, "invalid JSBSim setup: {what}"),
        }
    }
}

impl std::error::Error for JsbsimError {}
impl From<FdmError> for JsbsimError {
    fn from(value: FdmError) -> Self { Self::Fdm(value) }
}

/// What `AircraftState::engine_n1` reports for this aircraft.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EngineOutput {
    /// Turbine: `propulsion/engine[0]/n1`, percent.
    TurbineN1,
    /// Piston: `propulsion/engine[0]/engine-rpm` as a percentage of
    /// `rated_rpm` (the field then is "% rated RPM", not N1).
    PistonRpm { rated_rpm: f64 },
}

/// Initial conditions, written to JSBSim's `ic/` properties before `RunIC`.
/// JSBSim keeps them, so every reset returns here.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InitialConditions {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    /// Terrain (runway) elevation above JSBSim sea level, which this seam
    /// treats as MSL (see `Wgs84EnuFrame`).
    pub terrain_elevation_msl_m: f64,
    /// Height of the JSBSim reference point above the terrain.
    pub height_agl_m: f64,
    /// True heading, degrees.
    pub heading_deg: f64,
    /// Calibrated airspeed, knots (0 for parked).
    pub airspeed_kts: f64,
}

/// Aircraft-specific load parameters. `settings` are written after `RunIC`
/// and after every reset, in order; `propulsion/set-running` = -1 (all
/// engines running) is applied after them.
#[derive(Clone, Debug, PartialEq)]
pub struct JsbsimSetup {
    pub aircraft: String,
    pub engines: usize,
    pub engine_output: EngineOutput,
    pub settings: Vec<(String, f64)>,
    pub ic: InitialConditions,
}

struct Controls {
    aileron: Prop,
    elevator: Prop,
    rudder: Prop,
    steer: Prop,
    throttles: Vec<Prop>,
    brake_left: Prop,
    brake_right: Prop,
    pitch_trim: Prop,
    /// Lever commands; `None` where the model has no such property (the
    /// c172p has fixed gear and no speedbrake).
    gear: Option<Prop>,
    flaps: Option<Prop>,
    speedbrake: Option<Prop>,
}

struct Outputs {
    lat: Prop, lon: Prop, agl: Prop, msl: Prop,
    phi: Prop, theta: Prop, psi: Prop,
    u: Prop, v: Prop, w: Prop, p: Prop, q: Prop, r: Prop,
    vc: Prop, h_dot: Prop,
    gear: Prop, flap: Prop, spoiler: Prop, speedbrake: Option<Prop>,
    elevator: Prop, aileron: Prop, rudder: Prop,
    engine: Prop, wow_0: Prop, wow_1: Prop, wow_2: Prop,
}

/// A configured and initialized JSBSim instance.
pub struct JsbsimBackend {
    fdm: Fdm,
    controls: Controls,
    outputs: Outputs,
    frame: Wgs84EnuFrame,
    engine_output: EngineOutput,
    settings: Vec<(Prop, f64)>,
    set_running: Prop,
}

impl JsbsimBackend {
    /// Load `setup.aircraft` from `jsbsim_root` at the 120 Hz step, set the
    /// initial conditions, run them, start the engines and bind the
    /// production handles. Binding requires reviewed world-frame metadata,
    /// including the signed JSBSim MSL-to-WGS84-ellipsoid correction; there
    /// is no implicit frame.
    pub fn load(jsbsim_root: &Path, setup: &JsbsimSetup, frame: Wgs84EnuFrame) -> Result<Self, JsbsimError> {
        if setup.engines == 0 {
            return Err(JsbsimError::InvalidSetup("no engines"));
        }
        let ic = setup.ic;
        if ![ic.latitude_deg, ic.longitude_deg, ic.terrain_elevation_msl_m, ic.height_agl_m, ic.heading_deg, ic.airspeed_kts]
            .iter()
            .all(|v| v.is_finite())
        {
            return Err(JsbsimError::InvalidSetup("non-finite initial condition"));
        }
        let mut fdm = Fdm::create(jsbsim_root)?;
        fdm.load_model(&setup.aircraft, STEP_SECONDS)?;
        // Order matters: position, then terrain, then height above it.
        for (name, value) in [
            ("ic/lat-geod-deg", ic.latitude_deg),
            ("ic/long-gc-deg", ic.longitude_deg),
            ("ic/terrain-elevation-ft", ic.terrain_elevation_msl_m / FT_TO_M),
            ("ic/h-agl-ft", ic.height_agl_m / FT_TO_M),
            ("ic/psi-true-deg", ic.heading_deg),
            ("ic/phi-deg", 0.0),
            ("ic/theta-deg", 0.0),
            ("ic/vc-kts", ic.airspeed_kts),
        ] {
            let prop = fdm.bind(name)?;
            fdm.set(prop, value);
        }
        fdm.run_ic()?;
        Self::bind(fdm, setup, frame)
    }

    fn bind(mut fdm: Fdm, setup: &JsbsimSetup, frame: Wgs84EnuFrame) -> Result<Self, JsbsimError> {
        macro_rules! p { ($name:expr) => { fdm.bind($name)? }; }
        let mut throttles = Vec::with_capacity(setup.engines);
        for i in 0..setup.engines {
            throttles.push(p!(&format!("fcs/throttle-cmd-norm[{i}]")));
        }
        let controls = Controls {
            aileron: p!("fcs/aileron-cmd-norm"), elevator: p!("fcs/elevator-cmd-norm"),
            rudder: p!("fcs/rudder-cmd-norm"), steer: p!("fcs/steer-cmd-norm"), throttles,
            brake_left: p!("fcs/left-brake-cmd-norm"), brake_right: p!("fcs/right-brake-cmd-norm"),
            pitch_trim: p!("fcs/pitch-trim-cmd-norm"),
            gear: fdm.bind("gear/gear-cmd-norm").ok(),
            flaps: fdm.bind("fcs/flap-cmd-norm").ok(),
            speedbrake: fdm.bind("fcs/speedbrake-cmd-norm").ok(),
        };
        let engine = match setup.engine_output {
            EngineOutput::TurbineN1 => p!("propulsion/engine[0]/n1"),
            EngineOutput::PistonRpm { .. } => p!("propulsion/engine[0]/engine-rpm"),
        };
        let outputs = Outputs {
            lat: p!("position/lat-geod-deg"), lon: p!("position/long-gc-deg"),
            agl: p!("position/h-agl-ft"), msl: p!("position/h-sl-ft"),
            phi: p!("attitude/phi-rad"),
            theta: p!("attitude/theta-rad"), psi: p!("attitude/psi-rad"),
            u: p!("velocities/u-fps"), v: p!("velocities/v-fps"), w: p!("velocities/w-fps"),
            p: p!("velocities/p-rad_sec"), q: p!("velocities/q-rad_sec"), r: p!("velocities/r-rad_sec"),
            vc: p!("velocities/vc-kts"), h_dot: p!("velocities/h-dot-fps"),
            gear: p!("gear/gear-pos-norm"), flap: p!("fcs/flap-pos-norm"),
            spoiler: p!("fcs/spoiler-pos-norm"),
            speedbrake: fdm.bind("fcs/speedbrake-pos-norm").ok(),
            elevator: p!("fcs/elevator-pos-norm"),
            aileron: p!("fcs/left-aileron-pos-norm"), rudder: p!("fcs/rudder-pos-norm"),
            engine, wow_0: p!("gear/unit[0]/WOW"),
            wow_1: p!("gear/unit[1]/WOW"), wow_2: p!("gear/unit[2]/WOW"),
        };
        let mut settings = Vec::with_capacity(setup.settings.len());
        for (name, value) in &setup.settings {
            if !value.is_finite() {
                return Err(JsbsimError::InvalidSetup("non-finite setting"));
            }
            settings.push((p!(name), *value));
        }
        // Data logging must never run on the realtime sim thread. The
        // staged models carry no <output> directive (tools/build_m1_s2.ps1);
        // if one is present anyway, keep it disabled across resets.
        for i in 0.. {
            match fdm.bind(&format!("simulation/output[{i}]/enabled")) {
                Ok(prop) => settings.push((prop, 0.0)),
                Err(_) => break,
            }
        }
        let set_running = p!("propulsion/set-running");
        let mut backend = Self {
            fdm, controls, outputs, frame, engine_output: setup.engine_output, settings, set_running,
        };
        backend.apply_settings();
        Ok(backend)
    }

    /// Re-applied after every reset: JSBSim's reset re-initializes the FCS
    /// commands, tank contents and engines.
    fn apply_settings(&mut self) {
        for &(prop, value) in &self.settings {
            self.fdm.set(prop, value);
        }
        self.fdm.set(self.set_running, -1.0);
    }

    /// Read any bound-by-name property (setup checks and tests; allocates).
    pub fn property(&mut self, name: &str) -> Result<f64, JsbsimError> {
        let prop = self.fdm.bind(name)?;
        Ok(self.fdm.get(prop))
    }

    /// Current state without stepping.
    pub fn state(&self) -> Result<AircraftState, JsbsimError> {
        self.sample()
    }

    fn sample(&self) -> Result<AircraftState, JsbsimError> {
        let o = &self.outputs;
        let phi = self.fdm.get(o.phi);
        let theta = self.fdm.get(o.theta);
        let psi = self.fdm.get(o.psi);
        let (sphi, cphi) = phi.sin_cos();
        let (stheta, ctheta) = theta.sin_cos();
        let (spsi, cpsi) = psi.sin_cos();
        let u = self.fdm.get(o.u) * FT_TO_M;
        let v = self.fdm.get(o.v) * FT_TO_M;
        let w = self.fdm.get(o.w) * FT_TO_M;
        let velocity_ned_mps = [
            ctheta * cpsi * u + (sphi * stheta * cpsi - cphi * spsi) * v
                + (cphi * stheta * cpsi + sphi * spsi) * w,
            ctheta * spsi * u + (sphi * stheta * spsi + cphi * cpsi) * v
                + (cphi * stheta * spsi - sphi * cpsi) * w,
            -stheta * u + sphi * ctheta * v + cphi * ctheta * w,
        ];
        let (geodetic, local_enu) = self.frame.project_msl(
            self.fdm.get(o.lat), self.fdm.get(o.lon), self.fdm.get(o.msl) * FT_TO_M,
            velocity_ned_mps, [phi, theta, psi],
        ).map_err(JsbsimError::Georeference)?;
        let engine = self.fdm.get(o.engine);
        Ok(AircraftState {
            geodetic,
            height_agl_m: self.fdm.get(o.agl) * FT_TO_M,
            local_enu: Some(local_enu),
            roll_rad: phi, pitch_rad: theta, heading_rad: psi,
            angular_rate_rps: [self.fdm.get(o.p), self.fdm.get(o.q), self.fdm.get(o.r)],
            calibrated_airspeed_mps: self.fdm.get(o.vc) * KNOT_TO_MPS,
            vertical_speed_mps: self.fdm.get(o.h_dot) * FT_TO_M,
            gear_position: self.fdm.get(o.gear), flap_position: self.fdm.get(o.flap),
            spoiler_position: self.fdm.get(o.spoiler), elevator_position: self.fdm.get(o.elevator),
            aileron_position: self.fdm.get(o.aileron), rudder_position: self.fdm.get(o.rudder),
            engine_n1: match self.engine_output {
                EngineOutput::TurbineN1 => engine,
                EngineOutput::PistonRpm { rated_rpm } => 100.0 * engine / rated_rpm,
            },
            weight_on_wheels: self.fdm.get(o.wow_0) != 0.0 || self.fdm.get(o.wow_1) != 0.0 || self.fdm.get(o.wow_2) != 0.0,
            pitch_trim: 0.0, // stamped by SimCore
            speedbrake_position: o.speedbrake.map_or(0.0, |p| self.fdm.get(p)),
            levers: crate::Levers::default(), // stamped by SimCore
            autopilot: crate::ApStatus::default(), // stamped by SimCore
        })
    }
}

impl SimulationBackend for JsbsimBackend {
    type Error = JsbsimError;

    fn reset(&mut self) -> Result<AircraftState, Self::Error> {
        self.fdm.reset()?;
        self.apply_settings();
        self.fdm.set(self.controls.pitch_trim, 0.0);
        self.sample()
    }

    /// `position` is [`crate::PitchTrim::position`] (+ = nose up); JSBSim's
    /// trim command shares the elevator's nose-down-positive sign.
    fn set_pitch_trim(&mut self, position: f64) {
        let value = if position.is_finite() { -position } else { 0.0 };
        self.fdm.set(self.controls.pitch_trim, value);
    }

    fn step(&mut self, input: ControlInput, _dt_seconds: f64) -> Result<AircraftState, Self::Error> {
        let cmd = JsbsimCommands::from_input(input);
        let c = &self.controls;
        self.fdm.set(c.aileron, cmd.aileron);
        self.fdm.set(c.elevator, cmd.elevator);
        self.fdm.set(c.rudder, cmd.rudder);
        self.fdm.set(c.steer, cmd.steer);
        for &throttle in &c.throttles {
            self.fdm.set(throttle, cmd.throttle);
        }
        self.fdm.set(c.brake_left, cmd.left_brake);
        self.fdm.set(c.brake_right, cmd.right_brake);
        // Levers arrive already held and squat-switched by SimCore.
        let levers = input.levers;
        for (prop, value) in [
            (c.gear, if levers.gear_down { 1.0 } else { 0.0 }),
            (c.flaps, f64::from(levers.flaps)),
            (c.speedbrake, f64::from(levers.speedbrake)),
        ] {
            if let Some(prop) = prop {
                self.fdm.set(prop, value);
            }
        }
        match self.fdm.step()? {
            Step::Running => self.sample(),
            Step::Done => Err(JsbsimError::Terminated),
        }
    }
}
