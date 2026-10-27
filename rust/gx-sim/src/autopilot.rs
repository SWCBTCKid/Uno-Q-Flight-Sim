//! Autopilot and autothrottle (plan
//! the AP-002 contract): wings level / level off,
//! heading select, altitude select and hold, speed (autothrottle) and an ILS
//! approach (localizer and 3 degree glideslope) to any runway end of the
//! world pack.
//!
//! **Where it runs.** [`crate::SimCore`] owns one [`Autopilot`] on the
//! simulation thread and calls it every 120 Hz step, before the backend:
//! while engaged it **replaces** the pilot's roll and pitch (and, with the
//! autothrottle, the throttle) with its own commands, so the surfaces move
//! exactly as if the stick moved them. Yaw, brakes, trim and levers stay the
//! pilot's. It is pure and deterministic: a function of the panel inputs,
//! the states and the step count.
//!
//! **Panel.** The pilot's panel ([`ApInput`], on the I/O side) publishes its
//! target heading, altitude and speed as positions, and each button click as
//! a new `serial` with its [`ApAction`]. The core applies an action once,
//! when the serial changes, and only from valid input; like the levers, the
//! input is held through an input loss (whose neutral stick never trips the
//! override), so the autopilot keeps flying through one. The modes are the
//! core's: the panel shows [`ApStatus`] from the snapshot, so a disconnect
//! (override, ground, Reset) shows at once.
//!
//! **Input loss** (INPUT-002, R4: Chief 2026-10-25 "keep flying, level
//! out"). An engaged autopilot keeps its modes: the loss's neutral stick is
//! not an override. With the autopilot off, a loss engages LVL (wings level,
//! level off) on the first lossy step at which the engage conditions hold,
//! with [`ApWarning::InputLossLevel`]; a refusal (on the ground, too low,
//! unusual attitude) is silent and retried every step. The autothrottle is
//! not engaged. Valid input returning leaves it engaged; the stick override
//! disconnects it as usual. The 10 s INPUT-002 pause is unchanged.
//!
//! **Disconnects.** The autopilot disconnects on a firm stick input
//! ([`OVERRIDE`] roll or pitch), on the ground, below
//! [`GLIDESLOPE_DISCONNECT_FT`] on the glideslope (the pilot lands) and on
//! the AP button; the autothrottle on a throttle lever moved
//! [`THROTTLE_OVERRIDE`] from where it was when the autothrottle engaged, on
//! the ground and on the A/T button. Each shows a warning for
//! [`WARNING_TICKS`]. Reset turns both off.
//!
//! **Laws** (degrees, feet, knots; the M1-001 flight-script structure,
//! `tests/m1/scripts/global5000_flight.xml`):
//! * roll: bank command (rate limited to [`BANK_RATE_DEG_S`], within
//!   +-[`MAX_BANK_DEG`]) -> `roll = 0.12 (phi_cmd - phi) - 0.05 p`;
//! * heading: `phi_cmd = 3 * heading error` (the roll-out lead: rolling out
//!   starts 8 deg before the heading);
//! * localizer: heading command `course - 0.03 deg/m * cross-track`, within
//!   +-30 degrees of the course;
//! * vertical: a vertical-speed command -> pitch attitude command (PI on the
//!   VS error, plus the turn compensation `10 deg (250 / KCAS)^2 (1 / cos
//!   phi - 1)`) -> `pitch = 0.1 (theta_cmd - theta) - 0.04 q`;
//! * altitude: `vs_cmd = 5 fpm/ft * error` within +-[`MAX_VS_FPM`], which
//!   is also the capture (exponential, time constant 12 s);
//! * glideslope: the path's sink rate at the ground speed plus 4 fpm per ft
//!   below the path;
//! * G/S descent (from above, LOC captured, more than 30 m above): the
//!   path's sink rate plus [`GS_DESCENT_CLOSURE_FPM`] (at most
//!   [`MAX_GS_SINK_FPM`]); G/S captures where the glideslope law asks for
//!   the same descent (`closure / 4 fpm/ft` = 150 ft above), bumplessly;
//! * autothrottle: `throttle = I + 0.03 / kt * error`, `I' = 0.004 / kt s *
//!   error`, rate limited to [`THROTTLE_RATE`]; the error is from a speed
//!   reference that ramps to the panel speed at [`SPEED_REF_RATE_KT_S`]
//!   (from the speed flown at engagement), and `I` does not integrate while
//!   the output sits at a travel stop or the rate limit in the error's
//!   direction (anti-windup, W1b).
//!
//! Altitude is the height above the (flat) sim ground, the HUD's AGL.
//! Heading is true, as the HUD's.
//!
//! **Board tuning (AP-001 W1, 2026-10-26).** The `gx-app/tests/s2_jsbsim.rs`
//! `b7878_ap_*` cases fly these laws through `SimCore` on the stock JSBSim
//! 1.3.2 787-8 (BBJ preset) on the board. With the draft gains above
//! ([`Gains::DEFAULT`]) every W1 number passes, so **no gain and no limit
//! changed**. Measured (3,000 ft, 250 kt unless stated):
//! * LVL from 30.2 deg bank / +1,994 fpm: level off captured at 14 s, bank
//!   <= 0.001 deg and altitude within 0.5 ft of the hold from 60 s to 120 s;
//! * HDG 090 -> 180: max bank 24.8 deg, within 2 deg at 68 s, no overshoot
//!   past 0.06 deg, altitude within 32 ft;
//! * ALT 3,000 -> 8,000 with SPD: max VS 1,525 fpm, overshoot 0.3 ft,
//!   captured at 211 s, then within 0.02 ft; speed within 3.0 kt;
//! * SPD 220 -> 260: within 0.01 kt from 120 s to 180 s (first in the
//!   +-3 kt band at 9 s, then a 12 kt overshoot and 71 ft of altitude
//!   excursion while the autothrottle integrator unwinds); fixed in W1b,
//!   below;
//! * ILS 28R (SFO pack) from 12 nm, 30 deg intercept, 3,000 ft, flaps 20,
//!   gear down, 160 kt: LOC captured at 11.6 nm, G/S at 9.4 nm, below
//!   1,500 ft within 1.2 m and 15.5 ft, hand-over at 99.9 ft;
//! * disconnect bump (stick neutral, no trim): 1.0 deg/s at cruise, 1.3
//!   deg/s at the ILS hand-over; under the 3 deg/s autotrim threshold (R5),
//!   so autotrim stays backlog.
//!
//! **W1b (Chief 2026-10-26: fix the SPD overshoot).** Two limit-side
//! changes, no gain change and no pitch-side change. Anti-windup alone cut
//! the 220 -> 260 overshoot from 12.3 to 4.4 kt (altitude 71 -> 59 ft) and
//! left 4.7 kt on 260 -> 220: the step demands full thrust or idle, the
//! idle deceleration is only ~1 kt/s and the engines spool for ~10 s, so
//! the speed runs past before the thrust turns. A 1 kt/s reference ramp
//! still left 3.1 / 4.9 kt (it is at the idle deceleration limit); at
//! 0.5 kt/s the throttle stays off its stops: 220 -> 260 overshoot 1.6 kt,
//! altitude within 17 ft, in +-3 kt at 74 s; 260 -> 220 overshoot 2.2 kt,
//! altitude within 16 ft, in +-3 kt at 74 s; both settled within 0.4 kt
//! from 120 s.
//!
//! **AP-002 W5 (Chief 2026-10-27: banking "seemed slow").** Sources for
//! Boeing heading select:
//! * bank limit: the MCP bank-limit selector offers AUTO and 5..25 deg
//!   (747-400: <https://aerofly.com/dokuwiki/doku.php/aircraft:boeing_747-400>,
//!   777 AFDS: <https://www.smartcockpit.com/wp_quiz/boeing-b777-auto-flight/>;
//!   both secondary sources, no 787 FCOM available), so 25 deg is kept;
//! * roll rate: Honeywell's heading-hold patent (US 5,023,796) limits roll
//!   rates in heading manoeuvres to 2 or 3 deg/s, 2 deg/s after capture
//!   (<https://patents.google.com/patent/US5023796A/en>); a published example
//!   roll-autopilot requirement (MathWorks Simulink Test) allows 6 deg/s for
//!   large commands for passenger comfort. 3 deg/s would roll in to 25 deg
//!   in 8+ s, slower than the draft the Chief found slow and outside the
//!   TL's 6 s default, so the bank command rate is 6 deg/s: the upper
//!   cited value (**engineering choice, flagged to the TL**);
//! * roll-out lead: the patent captures with a predicted lead from an
//!   exponential roll-out model; here the heading gain (3 deg bank per deg)
//!   and the 6 deg/s rate give the lead (no source for a number).
//!
//! Before -> after on the board (90 deg HDG change, 3,000 ft): roll-in to
//! 24.5 deg 12.4 -> 5.8 s at 250 kt and 12.2 -> 5.7 s at 290 kt; peak roll
//! rate 3.8 -> 6.0 deg/s (R1: no dutch-roll excursion seen, bank never past
//! 25.0 deg); heading overshoot 0.06 -> 0.02 / 0.03 deg; altitude 32 -> 11
//! ft at 250 kt and 27 -> 12 ft at 290 kt (turn compensation); within 2 deg
//! of the heading 68 -> 55 s and 81 -> 65 s.
//!
//! **AP-002 W5b (glideslope help).** `gs_dev_ft` while an approach is armed
//! or captured. Board: 28R entered on the localizer at 10 nm, 1,036 ft above
//! the path: G/S descent at 9.9 nm, max sink 1,547 fpm, speed up to 164 kt
//! (A/T at idle), G/S captured at 5.0 nm, then within 40 ft below 1,500 ft
//! and hand-over at 99.9 ft. A 1,000 fpm closure captured at 6.8 nm but
//! left 56 ft below 1,500 ft (and 171 kt): 600 fpm is the engineering
//! choice.

use crate::{AircraftState, ControlInput};

const KT: f64 = 0.514_444;
const FT: f64 = 0.3048;
const DEG: f64 = std::f64::consts::PI / 180.0;

pub const MAX_BANK_DEG: f64 = 25.0;
/// Bank command rate limit, degrees per second (the default
/// [`Gains::bank_rate`]; AP-002 W5, see the module doc for the sources).
pub const BANK_RATE_DEG_S: f64 = 6.0;
pub const MAX_VS_FPM: f64 = 1500.0;
/// Glideslope sink limit, feet per minute.
pub const MAX_GS_SINK_FPM: f64 = 2500.0;
/// [`Vertical::GsDescent`]: descend this much faster than the glide path
/// sinks, so the aircraft closes on it from above (AP-002 W5b, engineering
/// choice from the board case: 1,000 ft of excess height closes in under
/// 1.5 min with the A/T near idle).
pub const GS_DESCENT_CLOSURE_FPM: f64 = 600.0;
pub const PITCH_LIMITS_DEG: (f64, f64) = (-10.0, 20.0);
/// Stick deflection (either axis) that disconnects the autopilot.
pub const OVERRIDE: f32 = 0.3;
/// Throttle lever travel from its position at engagement that disconnects
/// the autothrottle.
pub const THROTTLE_OVERRIDE: f32 = 0.15;
/// Autothrottle output rate limit, normalized travel per second.
pub const THROTTLE_RATE: f64 = 0.2;
/// The autothrottle's speed reference moves to the panel speed at this
/// rate, knots per second (W1b: a 40 kt step at full thrust overshot and
/// lifted the aircraft 59 ft through the altitude hold).
pub const SPEED_REF_RATE_KT_S: f64 = 0.5;
/// The glideslope hands over to the pilot here (no autoland).
pub const GLIDESLOPE_DISCONNECT_FT: f64 = 100.0;
/// Minimum height to engage the autopilot.
pub const MIN_ENGAGE_FT: f64 = 200.0;
/// Level off captures the altitude once the climb or descent is below this.
pub const LEVEL_CAPTURE_FPM: f64 = 200.0;
/// How long a warning shows: 5 s.
pub const WARNING_TICKS: u32 = 600;
/// Glide path angle and the glide path intercept point past the runway end.
pub const GLIDE_DEG: f64 = 3.0;
pub const GPIP_M: f64 = 300.0;
/// Localizer capture: within this of the centreline.
pub const LOC_CAPTURE_M: f64 = 1500.0;
/// An ILS is offered within this distance of the runway end, and within
/// [`ILS_SECTOR_DEG`] of its centreline seen from the runway, to an aircraft
/// heading within [`ILS_HEADING_DEG`] of the course.
pub const ILS_RANGE_M: f64 = 40_000.0;
pub const ILS_SECTOR_DEG: f64 = 35.0;
pub const ILS_HEADING_DEG: f64 = 90.0;

pub const HEADING_STEP_DEG: f32 = 5.0;
pub const ALTITUDE_STEP_FT: f32 = 500.0;
pub const SPEED_STEP_KT: f32 = 5.0;
pub const ALTITUDE_RANGE_FT: (f32, f32) = (0.0, 41_000.0);
pub const SPEED_RANGE_KT: (f32, f32) = (100.0, 350.0);

/// The law gains (units in each field). [`Gains::DEFAULT`] is what
/// [`crate::SimCore`] flies; the board tuning cases may override them
/// ([`crate::SimCore::with_autopilot_gains`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Gains {
    /// Aileron stick per degree of bank error.
    pub roll_phi: f64,
    /// Aileron stick per degree/second of roll rate.
    pub roll_p: f64,
    /// Bank degrees per degree of heading error.
    pub heading: f64,
    /// Heading degrees per metre of localizer cross-track.
    pub loc: f64,
    /// Pitch attitude degrees per ft/s of vertical-speed error.
    pub vs_p: f64,
    /// Pitch attitude degrees per ft of vertical-speed error (integral).
    pub vs_i: f64,
    /// Elevator stick per degree of pitch error.
    pub pitch_theta: f64,
    /// Elevator stick per degree/second of pitch rate.
    pub pitch_q: f64,
    /// Vertical speed fpm per ft of altitude error.
    pub alt: f64,
    /// Vertical speed fpm per ft below the glide path.
    pub gs: f64,
    /// Throttle per knot of speed error.
    pub at_p: f64,
    /// Throttle per knot-second of speed error.
    pub at_i: f64,
    /// Bank command rate limit, degrees per second (AP-002 W5).
    pub bank_rate: f64,
    /// Turn compensation (AP-002 W5): pitch attitude degrees added per unit
    /// of `1 / cos(bank) - 1` at 250 KCAS, scaled by `(250 / KCAS)^2` (the
    /// extra angle of attack a level turn needs goes with 1 / dynamic
    /// pressure).
    pub turn: f64,
}

impl Gains {
    pub const DEFAULT: Self = Self {
        roll_phi: 0.12,
        roll_p: 0.05,
        heading: 3.0,
        loc: 0.03,
        vs_p: 0.06,
        vs_i: 0.03,
        pitch_theta: 0.1,
        pitch_q: 0.04,
        alt: 5.0,
        gs: 4.0,
        at_p: 0.03,
        at_i: 0.004,
        bank_rate: BANK_RATE_DEG_S,
        turn: 10.0,
    };
}

impl Default for Gains {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// A panel button.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ApAction {
    #[default]
    None,
    /// AP: disconnect when engaged, else engage in wings level / level off.
    Master,
    /// LVL: wings level and level off (engages the autopilot).
    Level,
    /// HDG: turn to and hold the panel heading.
    Heading,
    /// ALT: climb or descend to and hold the panel altitude.
    Altitude,
    /// A/T: toggle the autothrottle (panel speed).
    Speed,
    /// ILS: arm the localizer and glideslope of the best runway end; again
    /// to cancel.
    Approach,
}

impl ApAction {
    pub fn code(self) -> u8 {
        match self {
            Self::None => 0,
            Self::Master => 1,
            Self::Level => 2,
            Self::Heading => 3,
            Self::Altitude => 4,
            Self::Speed => 5,
            Self::Approach => 6,
        }
    }

    pub fn from_code(c: u8) -> Option<Self> {
        Some(match c {
            0 => Self::None,
            1 => Self::Master,
            2 => Self::Level,
            3 => Self::Heading,
            4 => Self::Altitude,
            5 => Self::Speed,
            6 => Self::Approach,
            _ => return None,
        })
    }
}

/// The pilot's panel, published with every control record.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ApInput {
    /// +1 per click; the core applies `action` when it changes.
    pub serial: u32,
    pub action: ApAction,
    pub heading_deg: f32,
    pub altitude_ft: f32,
    pub speed_kt: f32,
}

impl Default for ApInput {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl ApInput {
    /// No click yet; heading 000, 3000 ft, 250 kt.
    pub const DEFAULT: Self = Self { serial: 0, action: ApAction::None, heading_deg: 0.0, altitude_ft: 3000.0, speed_kt: 250.0 };

    pub fn normalized(self) -> Self {
        let finite = |v: f32, d: f32| if v.is_finite() { v } else { d };
        let d = Self::default();
        Self {
            heading_deg: finite(self.heading_deg, d.heading_deg).rem_euclid(360.0),
            altitude_ft: finite(self.altitude_ft, d.altitude_ft).clamp(ALTITUDE_RANGE_FT.0, ALTITUDE_RANGE_FT.1),
            speed_kt: finite(self.speed_kt, d.speed_kt).clamp(SPEED_RANGE_KT.0, SPEED_RANGE_KT.1),
            ..self
        }
    }
}

/// One runway end with an ILS, in the world frame (ENU metres).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Approach {
    /// Runway end name, e.g. `28R` (ASCII, space padded).
    pub name: [u8; 4],
    /// Runway end on the centreline.
    pub threshold_m: [f64; 2],
    /// True landing course, radians.
    pub course_rad: f64,
}

impl Approach {
    pub fn new(name: &str, threshold_m: [f64; 2], course_deg: f64) -> Self {
        let mut n = [b' '; 4];
        for (d, s) in n.iter_mut().zip(name.bytes().filter(u8::is_ascii_graphic)) {
            *d = s;
        }
        Self { name: n, threshold_m, course_rad: course_deg.rem_euclid(360.0) * DEG }
    }

    pub fn name(&self) -> &str {
        std::str::from_utf8(&self.name).unwrap_or("").trim_end()
    }

    /// (along, cross): metres along the landing course from the runway end
    /// (negative on the approach) and right of the centreline.
    pub fn track(&self, position_m: [f64; 2]) -> (f64, f64) {
        let (s, c) = self.course_rad.sin_cos();
        let d = [position_m[0] - self.threshold_m[0], position_m[1] - self.threshold_m[1]];
        (d[0] * s + d[1] * c, d[0] * c - d[1] * s)
    }

    /// Glide path height above the runway at `along`, metres.
    pub fn path_height_m(&self, along: f64) -> f64 {
        (GPIP_M - along).max(0.0) * (GLIDE_DEG * DEG).tan()
    }
}

/// The approach the ILS button arms: the runway end ahead whose centreline
/// the aircraft is nearest, within the service volume.
pub fn select_approach(approaches: &[Approach], position_m: [f64; 2], heading_rad: f64) -> Option<usize> {
    approaches
        .iter()
        .enumerate()
        .filter_map(|(i, a)| {
            let (along, cross) = a.track(position_m);
            let ahead = (-ILS_RANGE_M..-500.0).contains(&along);
            let sector = cross.atan2(-along).abs() < ILS_SECTOR_DEG * DEG;
            let heading = wrap_deg((a.course_rad - heading_rad) / DEG).abs() < ILS_HEADING_DEG;
            (ahead && sector && heading).then_some((i, cross.abs()))
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(i, _)| i)
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Lateral {
    #[default]
    WingsLevel,
    Heading,
    Localizer,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Vertical {
    /// Stop the climb or descent, then hold the altitude reached.
    #[default]
    LevelOff,
    /// Hold [`ApStatus::hold_ft`].
    AltHold,
    /// Climb or descend to the panel altitude, then hold it.
    AltSelect,
    Glideslope,
    /// AP-002 (interface, FD implements): localizer captured with the
    /// aircraft above the glide path; descend to meet the glideslope from
    /// above, then capture it.
    GsDescent,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ApWarning {
    /// Autopilot disconnected (stick, ground, button, glideslope minimum).
    ApDisconnect,
    /// Autothrottle disconnected.
    AtDisconnect,
    /// Engage refused: on the ground, too low or in an unusual attitude.
    NotAvailable,
    /// ILS pressed with no runway in range ahead.
    NoIls,
    /// An input loss engaged the autopilot in LVL (R4, Chief 2026-10-25).
    InputLossLevel,
}

/// What the panel and HUD show; stamped into every snapshot.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ApStatus {
    pub engaged: bool,
    pub lateral: Lateral,
    pub vertical: Vertical,
    pub loc_armed: bool,
    pub gs_armed: bool,
    /// Index into the core's approaches while the ILS is armed or captured.
    pub approach: Option<u8>,
    pub autothrottle: bool,
    pub hold_ft: f32,
    /// The panel targets the core last read.
    pub input: ApInput,
    pub warning: Option<ApWarning>,
    /// AP-002 (interface, FD implements): height above the glide path, ft
    /// (negative below), while an approach is armed or captured; else `None`.
    pub gs_dev_ft: Option<f32>,
}

/// The autopilot. Owned by [`crate::SimCore`].
#[derive(Clone, Debug, Default)]
pub struct Autopilot {
    status: ApStatus,
    applied_serial: u32,
    warning_ticks: u32,
    bank_cmd_deg: f64,
    theta_int_deg: f64,
    throttle: f64,
    throttle_int: f64,
    /// The speed the autothrottle holds now, knots (ramps to the panel's).
    speed_ref_kt: f64,
    throttle_at_engage: f32,
    /// Set by [`Autopilot::reset`]: the next panel read only adopts its
    /// serial, so a click read with the Reset (or before it) never applies
    /// after it.
    resync: bool,
    gains: Gains,
}

impl Autopilot {
    pub fn with_gains(gains: Gains) -> Self {
        Self { gains, ..Self::default() }
    }

    pub fn gains(&self) -> Gains {
        self.gains
    }

    pub fn status(&self) -> ApStatus {
        self.status
    }

    /// Reset: everything off. The next panel read's serial counts as
    /// applied, so no click from before the reset (or read in the same tick,
    /// whose commands run first) replays after it.
    pub fn reset(&mut self) {
        let input = self.status.input;
        *self = Self { applied_serial: self.applied_serial, resync: true, gains: self.gains, ..Self::default() };
        self.status.input = input;
    }

    /// Take the panel from valid input: targets always, the action once.
    pub fn input(&mut self, input: ApInput, state: Option<&AircraftState>, pilot: &ControlInput, approaches: &[Approach]) {
        let input = input.normalized();
        self.status.input = input;
        if std::mem::take(&mut self.resync) || input.serial == self.applied_serial {
            self.applied_serial = input.serial;
            return;
        }
        self.applied_serial = input.serial;
        let Some(s) = state else { return };
        match input.action {
            ApAction::None => {}
            ApAction::Master if self.status.engaged => self.disconnect_ap(true),
            ApAction::Master | ApAction::Level => {
                if self.engage(s) {
                    self.status.lateral = Lateral::WingsLevel;
                    self.status.vertical = Vertical::LevelOff;
                    self.disarm();
                }
            }
            ApAction::Heading => {
                // Leaving the localizer cancels the approach (no glideslope
                // without it).
                if self.engage(s) {
                    self.status.lateral = Lateral::Heading;
                    if matches!(self.status.vertical, Vertical::Glideslope | Vertical::GsDescent) {
                        self.status.vertical = Vertical::LevelOff;
                    }
                    self.status.loc_armed = false;
                    self.status.gs_armed = false;
                    self.status.approach = None;
                }
            }
            ApAction::Altitude => {
                if self.engage(s) {
                    self.status.vertical = Vertical::AltSelect;
                    self.status.gs_armed = false;
                    if self.status.lateral != Lateral::Localizer && !self.status.loc_armed {
                        self.status.approach = None;
                    }
                }
            }
            ApAction::Speed => {
                if self.status.autothrottle {
                    self.disconnect_at(true);
                } else if s.weight_on_wheels {
                    self.warn(ApWarning::NotAvailable);
                } else {
                    self.status.autothrottle = true;
                    self.throttle = f64::from(pilot.throttle);
                    self.throttle_int = self.throttle;
                    self.throttle_at_engage = pilot.throttle;
                    self.speed_ref_kt = s.calibrated_airspeed_mps / KT;
                }
            }
            ApAction::Approach => {
                if self.status.approach.is_some() {
                    self.disarm();
                    if self.status.lateral == Lateral::Localizer {
                        self.status.lateral = Lateral::WingsLevel;
                    }
                    if matches!(self.status.vertical, Vertical::Glideslope | Vertical::GsDescent) {
                        self.status.vertical = Vertical::LevelOff;
                    }
                    // A captured approach is cancelled too (disarm keeps it
                    // only while a captured mode still uses it).
                    self.status.approach = None;
                    self.status.gs_dev_ft = None;
                    return;
                }
                let pos = s.local_enu.map_or([f64::NAN; 2], |p| [p.position_m[0], p.position_m[1]]);
                match select_approach(approaches, pos, s.heading_rad) {
                    Some(i) if self.engage(s) => {
                        self.status.approach = Some(i as u8);
                        self.status.loc_armed = true;
                        self.status.gs_armed = true;
                    }
                    Some(_) => {}
                    None => self.warn(ApWarning::NoIls),
                }
            }
        }
    }

    /// One step: disconnect checks, mode transitions, then (while engaged)
    /// overwrite the pilot's roll / pitch / throttle in `controls`.
    /// `input_valid` false (an input loss, neutral controls) skips the
    /// stick and throttle overrides, so the autopilot flies through it.
    pub fn step(&mut self, controls: &mut ControlInput, s: &AircraftState, approaches: &[Approach], input_valid: bool, dt: f64) {
        self.warning_ticks = self.warning_ticks.saturating_sub(1);
        if self.warning_ticks == 0 {
            self.status.warning = None;
        }
        let agl_ft = s.height_agl_m / FT;
        if s.weight_on_wheels {
            self.disconnect_ap(false);
            self.disconnect_at(false);
        }
        if input_valid && self.status.engaged && (controls.roll.abs() > OVERRIDE || controls.pitch.abs() > OVERRIDE) {
            self.disconnect_ap(true);
        }
        if input_valid && self.status.autothrottle && (controls.throttle - self.throttle_at_engage).abs() > THROTTLE_OVERRIDE {
            self.disconnect_at(true);
        }
        if self.status.engaged
            && matches!(self.status.vertical, Vertical::Glideslope | Vertical::GsDescent)
            && agl_ft < GLIDESLOPE_DISCONNECT_FT
        {
            self.disconnect_ap(true);
        }
        // R4: an input loss with the autopilot off engages LVL as soon as the
        // engage conditions hold (tried every lossy step, refused silently).
        if !input_valid && !self.status.engaged && self.engage_quietly(s) {
            self.warn(ApWarning::InputLossLevel);
        }

        if self.status.engaged {
            let approach = self.status.approach.and_then(|i| approaches.get(usize::from(i)));
            let pos = s.local_enu.map(|p| [p.position_m[0], p.position_m[1]]);
            let track = approach.zip(pos).map(|(a, p)| (a, a.track(p)));
            // Captures.
            if let Some((_, (along, cross))) = track {
                if self.status.loc_armed && cross.abs() < LOC_CAPTURE_M && along < 0.0 {
                    self.status.loc_armed = false;
                    self.status.lateral = Lateral::Localizer;
                }
            }
            self.status.gs_dev_ft = None;
            if let Some((a, (along, _))) = track {
                let dev_m = s.height_agl_m - a.path_height_m(along);
                self.status.gs_dev_ft = Some((dev_m / FT) as f32);
                if self.status.gs_armed && self.status.lateral == Lateral::Localizer {
                    // From G/S descent, capture where the glideslope law
                    // asks for the descent already flown (closure / gs ft
                    // above the path), so the capture is bumpless.
                    let from_above = self.status.vertical == Vertical::GsDescent
                        && dev_m / FT <= GS_DESCENT_CLOSURE_FPM / self.gains.gs;
                    if from_above || (-15.0..30.0).contains(&dev_m) {
                        self.status.gs_armed = false;
                        self.status.vertical = Vertical::Glideslope;
                    } else if dev_m >= 30.0 && self.status.vertical != Vertical::GsDescent {
                        // W5b: above the path on the localizer: descend to
                        // meet it (G/S stays armed and captures in the band).
                        self.status.vertical = Vertical::GsDescent;
                    }
                }
            }
            if self.status.vertical == Vertical::LevelOff && (s.vertical_speed_mps / FT * 60.0).abs() < LEVEL_CAPTURE_FPM {
                self.status.vertical = Vertical::AltHold;
                self.status.hold_ft = agl_ft as f32;
            }
            if self.status.vertical == Vertical::AltSelect && (f64::from(self.status.input.altitude_ft) - agl_ft).abs() < 20.0 {
                self.status.vertical = Vertical::AltHold;
                self.status.hold_ft = self.status.input.altitude_ft;
            }

            // Lateral: bank command.
            let g = self.gains;
            let heading_deg = s.heading_rad / DEG;
            let target_bank = match (self.status.lateral, track) {
                (Lateral::Localizer, Some((a, (_, cross)))) => {
                    let cmd = a.course_rad / DEG - (g.loc * cross).clamp(-30.0, 30.0);
                    g.heading * wrap_deg(cmd - heading_deg)
                }
                (Lateral::Heading, _) => g.heading * wrap_deg(f64::from(self.status.input.heading_deg) - heading_deg),
                _ => 0.0,
            }
            .clamp(-MAX_BANK_DEG, MAX_BANK_DEG);
            let step = g.bank_rate * dt;
            self.bank_cmd_deg += (target_bank - self.bank_cmd_deg).clamp(-step, step);
            let p_deg = s.angular_rate_rps[0] / DEG;
            let roll = g.roll_phi * (self.bank_cmd_deg - s.roll_rad / DEG) - g.roll_p * p_deg;

            // Vertical: VS command -> pitch command.
            let vs_fpm = s.vertical_speed_mps / FT * 60.0;
            let vs_cmd = match (self.status.vertical, track) {
                (Vertical::Glideslope, Some((a, (along, _)))) => {
                    let gs = s.local_enu.map_or(s.calibrated_airspeed_mps, |p| p.velocity_mps[0].hypot(p.velocity_mps[1]));
                    let sink = gs * (GLIDE_DEG * DEG).tan() / FT * 60.0;
                    let below_ft = (a.path_height_m(along) - s.height_agl_m) / FT;
                    (-sink + g.gs * below_ft).clamp(-MAX_GS_SINK_FPM, MAX_VS_FPM)
                }
                (Vertical::GsDescent, Some(_)) => {
                    let gs = s.local_enu.map_or(s.calibrated_airspeed_mps, |p| p.velocity_mps[0].hypot(p.velocity_mps[1]));
                    let sink = gs * (GLIDE_DEG * DEG).tan() / FT * 60.0;
                    (-(sink + GS_DESCENT_CLOSURE_FPM)).max(-MAX_GS_SINK_FPM)
                }
                (Vertical::AltSelect, _) => (g.alt * (f64::from(self.status.input.altitude_ft) - agl_ft)).clamp(-MAX_VS_FPM, MAX_VS_FPM),
                (Vertical::AltHold, _) => (g.alt * (f64::from(self.status.hold_ft) - agl_ft)).clamp(-MAX_VS_FPM, MAX_VS_FPM),
                _ => 0.0,
            };
            let err_fps = (vs_cmd - vs_fpm) / 60.0;
            let (lo, hi) = PITCH_LIMITS_DEG;
            self.theta_int_deg = (self.theta_int_deg + g.vs_i * err_fps * dt).clamp(lo, hi);
            // W5: a level turn at bank phi needs 1 / cos(phi) g of lift.
            let kcas = (s.calibrated_airspeed_mps / KT).max(100.0);
            let turn = g.turn * (250.0 / kcas).powi(2) * (1.0 / s.roll_rad.cos().max(0.5) - 1.0);
            let theta_cmd = (self.theta_int_deg + g.vs_p * err_fps + turn).clamp(lo, hi);
            let q_deg = s.angular_rate_rps[1] / DEG;
            let pitch = g.pitch_theta * (theta_cmd - s.pitch_rad / DEG) - g.pitch_q * q_deg;

            controls.roll = (roll as f32).clamp(-1.0, 1.0);
            controls.pitch = (pitch as f32).clamp(-1.0, 1.0);
        }

        if self.status.autothrottle {
            let ramp = SPEED_REF_RATE_KT_S * dt;
            self.speed_ref_kt += (f64::from(self.status.input.speed_kt) - self.speed_ref_kt).clamp(-ramp, ramp);
            let err_kt = self.speed_ref_kt - s.calibrated_airspeed_mps / KT;
            let g = self.gains;
            let step = THROTTLE_RATE * dt;
            // Anti-windup (W1b): no integration while the output is held at
            // a travel stop or by the rate limit in the error's direction.
            let wanted = self.throttle_int + g.at_p * err_kt;
            let held = (err_kt > 0.0 && (wanted >= 1.0 || wanted - self.throttle > step))
                || (err_kt < 0.0 && (wanted <= 0.0 || wanted - self.throttle < -step));
            if !held {
                self.throttle_int = (self.throttle_int + g.at_i * err_kt * dt).clamp(0.0, 1.0);
            }
            let target = (self.throttle_int + g.at_p * err_kt).clamp(0.0, 1.0);
            self.throttle += (target - self.throttle).clamp(-step, step);
            controls.throttle = self.throttle as f32;
        }
    }

    fn engage(&mut self, s: &AircraftState) -> bool {
        let ok = self.engage_quietly(s);
        if !ok {
            self.warn(ApWarning::NotAvailable);
        }
        ok
    }

    /// [`Autopilot::engage`] without the refusal warning.
    fn engage_quietly(&mut self, s: &AircraftState) -> bool {
        if self.status.engaged {
            return true;
        }
        let ok = !s.weight_on_wheels
            && s.height_agl_m / FT >= MIN_ENGAGE_FT
            && s.roll_rad.abs() < 45.0 * DEG
            && s.pitch_rad.abs() < 25.0 * DEG;
        if !ok {
            return false;
        }
        self.status.engaged = true;
        self.status.lateral = Lateral::WingsLevel;
        self.status.vertical = Vertical::LevelOff;
        // Bumpless: start from the attitude flown.
        self.bank_cmd_deg = s.roll_rad / DEG;
        self.theta_int_deg = s.pitch_rad / DEG;
        true
    }

    fn disarm(&mut self) {
        self.status.loc_armed = false;
        self.status.gs_armed = false;
        if self.status.vertical == Vertical::GsDescent {
            self.status.vertical = Vertical::LevelOff;
        }
        if self.status.lateral != Lateral::Localizer && self.status.vertical != Vertical::Glideslope {
            self.status.approach = None;
        }
    }

    fn disconnect_ap(&mut self, warn: bool) {
        if !self.status.engaged {
            return;
        }
        let at = self.status.autothrottle;
        self.status = ApStatus { input: self.status.input, autothrottle: at, warning: self.status.warning, ..ApStatus::default() };
        if warn {
            self.warn(ApWarning::ApDisconnect);
        }
    }

    fn disconnect_at(&mut self, warn: bool) {
        if !self.status.autothrottle {
            return;
        }
        self.status.autothrottle = false;
        if warn {
            self.warn(ApWarning::AtDisconnect);
        }
    }

    fn warn(&mut self, w: ApWarning) {
        self.status.warning = Some(w);
        self.warning_ticks = WARNING_TICKS;
    }
}

/// Wrap to (-180, 180].
pub fn wrap_deg(d: f64) -> f64 {
    let w = d.rem_euclid(360.0);
    if w > 180.0 { w - 360.0 } else { w }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::LocalEnuPose;

    /// A point-mass aircraft: roll and pitch rates follow the stick, the
    /// flight path is the pitch less 2 degrees, turns are coordinated, and
    /// thrust minus drag and gravity along the path change the speed.
    struct Plant {
        s: AircraftState,
        speed_kt: f64,
        pos: [f64; 2],
    }

    impl Plant {
        fn new(alt_ft: f64, heading_deg: f64, speed_kt: f64) -> Self {
            let mut p = Self { s: AircraftState::default(), speed_kt, pos: [0.0, 0.0] };
            p.s.height_agl_m = alt_ft * FT;
            p.s.heading_rad = heading_deg * DEG;
            p.s.pitch_rad = 2.0 * DEG;
            p.sync();
            p
        }

        fn at(mut self, pos: [f64; 2]) -> Self {
            self.pos = pos;
            self.sync();
            self
        }

        fn sync(&mut self) {
            let v = self.speed_kt * KT;
            let gamma = self.s.pitch_rad - 2.0 * DEG;
            let (sh, ch) = self.s.heading_rad.sin_cos();
            self.s.calibrated_airspeed_mps = v;
            self.s.vertical_speed_mps = v * gamma.sin();
            self.s.local_enu = Some(LocalEnuPose {
                frame_id: 1,
                position_m: [self.pos[0], self.pos[1], self.s.height_agl_m],
                velocity_mps: [v * gamma.cos() * sh, v * gamma.cos() * ch, self.s.vertical_speed_mps],
                ..LocalEnuPose::default()
            });
        }

        fn step(&mut self, c: &ControlInput, dt: f64) {
            let p = 15.0 * DEG * f64::from(c.roll);
            let q = 6.0 * DEG * f64::from(c.pitch);
            self.s.angular_rate_rps = [p, q, 0.0];
            self.s.roll_rad += p * dt;
            self.s.pitch_rad += q * dt;
            let v = self.speed_kt * KT;
            let gamma = self.s.pitch_rad - 2.0 * DEG;
            self.s.heading_rad = (self.s.heading_rad + 9.81 * self.s.roll_rad.tan() / v * dt).rem_euclid(2.0 * std::f64::consts::PI);
            self.s.height_agl_m += v * gamma.sin() * dt;
            let (sh, ch) = self.s.heading_rad.sin_cos();
            self.pos[0] += v * gamma.cos() * sh * dt;
            self.pos[1] += v * gamma.cos() * ch * dt;
            self.speed_kt += (6.0 * (f64::from(c.throttle) - 0.5) - 9.81 * gamma.sin() / KT) * dt;
            self.sync();
        }
    }

    const DT: f64 = 1.0 / 120.0;

    struct Rig {
        ap: Autopilot,
        plant: Plant,
        panel: ApInput,
        pilot: ControlInput,
        approaches: Vec<Approach>,
    }

    impl Rig {
        fn new(plant: Plant) -> Self {
            Self { ap: Autopilot::default(), plant, panel: ApInput::default(), pilot: ControlInput { throttle: 0.5, ..ControlInput::default() }, approaches: Vec::new() }
        }

        fn click(&mut self, action: ApAction) {
            self.panel.serial += 1;
            self.panel.action = action;
            self.ap.input(self.panel, Some(&self.plant.s), &self.pilot, &self.approaches);
        }

        fn tick(&mut self) {
            self.ap.input(self.panel, Some(&self.plant.s), &self.pilot, &self.approaches);
            let mut c = self.pilot;
            self.ap.step(&mut c, &self.plant.s, &self.approaches, true, DT);
            self.plant.step(&c, DT);
        }

        /// Fly `seconds`; returns the most extreme (bank, VS fpm) seen.
        fn fly(&mut self, seconds: f64) -> (f64, f64) {
            let mut worst = (0.0f64, 0.0f64);
            for _ in 0..(seconds / DT) as usize {
                self.tick();
                worst.0 = worst.0.max(self.plant.s.roll_rad.abs() / DEG);
                worst.1 = worst.1.max((self.plant.s.vertical_speed_mps / FT * 60.0).abs());
            }
            worst
        }

        fn alt_ft(&self) -> f64 {
            self.plant.s.height_agl_m / FT
        }

        fn heading_deg(&self) -> f64 {
            self.plant.s.heading_rad / DEG
        }
    }

    #[test]
    fn engage_is_refused_on_the_ground_and_too_low() {
        let mut r = Rig::new(Plant::new(5000.0, 90.0, 250.0));
        r.plant.s.weight_on_wheels = true;
        r.click(ApAction::Level);
        assert!(!r.ap.status().engaged);
        assert_eq!(r.ap.status().warning, Some(ApWarning::NotAvailable));
        let mut r = Rig::new(Plant::new(150.0, 90.0, 250.0));
        r.click(ApAction::Heading);
        assert!(!r.ap.status().engaged, "below {MIN_ENGAGE_FT} ft");
        r.click(ApAction::Speed);
        assert!(r.ap.status().autothrottle, "the autothrottle engages at any height in the air");
    }

    #[test]
    fn a_click_applies_once_per_serial() {
        let mut r = Rig::new(Plant::new(5000.0, 90.0, 250.0));
        r.click(ApAction::Master);
        assert!(r.ap.status().engaged);
        // The same serial again (every later publication) does nothing.
        for _ in 0..10 {
            r.ap.input(r.panel, Some(&r.plant.s), &r.pilot, &r.approaches);
        }
        assert!(r.ap.status().engaged);
        r.click(ApAction::Master);
        assert!(!r.ap.status().engaged);
        assert_eq!(r.ap.status().warning, Some(ApWarning::ApDisconnect));
    }

    #[test]
    fn level_rolls_wings_level_and_holds_the_altitude_it_reaches() {
        let mut r = Rig::new(Plant::new(5000.0, 90.0, 250.0));
        r.plant.s.roll_rad = 30.0 * DEG;
        r.plant.s.pitch_rad = 8.0 * DEG; // climbing ~2800 fpm
        r.plant.sync();
        r.click(ApAction::Level);
        assert_eq!(r.ap.status().vertical, Vertical::LevelOff);
        r.fly(60.0);
        let st = r.ap.status();
        assert_eq!(st.vertical, Vertical::AltHold);
        assert!(r.plant.s.roll_rad.abs() < 1.0 * DEG, "wings level: {}", r.plant.s.roll_rad / DEG);
        assert!((r.alt_ft() - f64::from(st.hold_ft)).abs() < 30.0, "holds {} at {}", st.hold_ft, r.alt_ft());
        assert!(f64::from(st.hold_ft) > 5000.0 && f64::from(st.hold_ft) < 6500.0, "captured {}", st.hold_ft);
    }

    #[test]
    fn heading_turns_the_short_way_within_the_bank_limit() {
        let mut r = Rig::new(Plant::new(5000.0, 350.0, 250.0));
        r.panel.heading_deg = 80.0;
        r.click(ApAction::Heading);
        let (bank, _) = r.fly(90.0);
        assert!(bank <= MAX_BANK_DEG + 1.0, "bank {bank}");
        assert!(wrap_deg(r.heading_deg() - 80.0).abs() < 1.0, "heading {}", r.heading_deg());
        assert!((r.alt_ft() - 5000.0).abs() < 100.0, "level in the turn: {}", r.alt_ft());
    }

    #[test]
    fn altitude_select_climbs_at_the_limit_and_captures() {
        let mut r = Rig::new(Plant::new(3000.0, 90.0, 250.0));
        r.panel.altitude_ft = 8000.0;
        r.click(ApAction::Altitude);
        r.click(ApAction::Speed);
        let (_, vs) = r.fly(400.0);
        assert!(vs < MAX_VS_FPM + 150.0, "vs {vs}");
        assert_eq!(r.ap.status().vertical, Vertical::AltHold);
        assert!((r.alt_ft() - 8000.0).abs() < 30.0, "alt {}", r.alt_ft());
        assert!((r.plant.speed_kt - 250.0).abs() < 5.0, "speed {}", r.plant.speed_kt);
    }

    #[test]
    fn autothrottle_holds_the_panel_speed() {
        let mut r = Rig::new(Plant::new(5000.0, 90.0, 220.0));
        r.panel.speed_kt = 260.0;
        r.click(ApAction::Level);
        r.click(ApAction::Speed);
        r.fly(200.0);
        assert!((r.plant.speed_kt - 260.0).abs() < 2.0, "speed {}", r.plant.speed_kt);
    }

    #[test]
    fn stick_and_throttle_overrides_disconnect_but_an_input_loss_does_not() {
        let mut r = Rig::new(Plant::new(5000.0, 90.0, 250.0));
        r.click(ApAction::Level);
        r.click(ApAction::Speed);
        // Loss: neutral controls (throttle 0), not an override.
        let mut neutral = ControlInput::default();
        r.ap.step(&mut neutral, &r.plant.s, &[], false, DT);
        assert!(r.ap.status().engaged && r.ap.status().autothrottle);
        let mut c = ControlInput { pitch: 0.5, throttle: 0.5, ..ControlInput::default() };
        r.ap.step(&mut c, &r.plant.s, &[], true, DT);
        assert!(!r.ap.status().engaged);
        assert!(r.ap.status().autothrottle, "the stick leaves the autothrottle on");
        assert_eq!(r.ap.status().warning, Some(ApWarning::ApDisconnect));
        let mut c = ControlInput { throttle: 0.9, ..ControlInput::default() };
        r.ap.step(&mut c, &r.plant.s, &[], true, DT);
        assert!(!r.ap.status().autothrottle);
        assert_eq!(r.ap.status().warning, Some(ApWarning::AtDisconnect));
        // The warning clears after WARNING_TICKS.
        for _ in 0..WARNING_TICKS {
            r.ap.step(&mut ControlInput::default(), &r.plant.s, &[], true, DT);
        }
        assert_eq!(r.ap.status().warning, None);
    }

    #[test]
    fn r4_an_input_loss_engages_lvl_when_off_and_keeps_engaged_modes() {
        // Off: the first lossy step engages wings level / level off.
        let mut r = Rig::new(Plant::new(5000.0, 90.0, 250.0));
        r.plant.s.roll_rad = 20.0 * DEG;
        r.plant.sync();
        let mut neutral = ControlInput::default();
        r.ap.step(&mut neutral, &r.plant.s, &[], false, DT);
        let st = r.ap.status();
        // Level already: level off captures the altitude at once.
        assert!(st.engaged && st.lateral == Lateral::WingsLevel && st.vertical == Vertical::AltHold, "{st:?}");
        assert!(!st.autothrottle, "the autothrottle is not engaged");
        assert_eq!(st.warning, Some(ApWarning::InputLossLevel));
        assert!(neutral.roll < 0.0, "it flies on the same step: {neutral:?}");
        // Valid input returning (stick neutral) leaves it engaged; the stick
        // override then disconnects it.
        r.ap.step(&mut ControlInput::default(), &r.plant.s, &[], true, DT);
        assert!(r.ap.status().engaged);
        r.ap.step(&mut ControlInput { roll: 0.5, ..ControlInput::default() }, &r.plant.s, &[], true, DT);
        assert!(!r.ap.status().engaged);
        assert_eq!(r.ap.status().warning, Some(ApWarning::ApDisconnect));

        // Engaged: a loss keeps the modes and the warning stays as it was.
        let mut r = Rig::new(Plant::new(5000.0, 90.0, 250.0));
        r.panel.heading_deg = 180.0;
        r.click(ApAction::Heading);
        r.click(ApAction::Speed);
        r.fly(1.0);
        let before = r.ap.status();
        assert_eq!(before.vertical, Vertical::AltHold);
        r.ap.step(&mut ControlInput::default(), &r.plant.s, &[], false, DT);
        let st = r.ap.status();
        assert!(st.engaged && st.autothrottle && st.lateral == Lateral::Heading && st.vertical == before.vertical, "{st:?}");
        assert_eq!(st.warning, before.warning);
    }

    #[test]
    fn r4_no_engage_on_the_ground_or_too_low_and_no_warning_spam() {
        let mut r = Rig::new(Plant::new(5000.0, 90.0, 250.0));
        r.plant.s.weight_on_wheels = true;
        for _ in 0..10 {
            r.ap.step(&mut ControlInput::default(), &r.plant.s, &[], false, DT);
        }
        assert!(!r.ap.status().engaged);
        assert_eq!(r.ap.status().warning, None, "no NotAvailable spam on the ground");
        // Too low, then climbing through MIN_ENGAGE_FT during the loss.
        let mut r = Rig::new(Plant::new(150.0, 90.0, 250.0));
        r.plant.s.pitch_rad = 6.0 * DEG; // climbing ~1,700 fpm
        r.plant.sync();
        let mut engaged_at = None;
        for i in 0..(30.0 / DT) as usize {
            let mut c = ControlInput { throttle: 0.5, ..ControlInput::default() };
            r.ap.step(&mut c, &r.plant.s, &[], false, DT);
            let st = r.ap.status();
            if !st.engaged {
                assert_eq!(st.warning, None, "silent while too low");
            } else if engaged_at.is_none() {
                engaged_at = Some(i);
                assert_eq!(st.warning, Some(ApWarning::InputLossLevel));
                assert!(r.alt_ft() >= MIN_ENGAGE_FT && r.alt_ft() < MIN_ENGAGE_FT + 5.0, "at {}", r.alt_ft());
            }
            r.plant.step(&c, DT);
        }
        assert!(engaged_at.is_some(), "engages as soon as the conditions hold");
        // Unusual attitude: refused silently.
        let mut r = Rig::new(Plant::new(5000.0, 90.0, 250.0));
        r.plant.s.roll_rad = 60.0 * DEG;
        r.ap.step(&mut ControlInput::default(), &r.plant.s, &[], false, DT);
        assert!(!r.ap.status().engaged && r.ap.status().warning.is_none());
    }

    #[test]
    fn w1b_the_autothrottle_ramps_its_reference_and_does_not_wind_up() {
        let mut r = Rig::new(Plant::new(5000.0, 90.0, 220.0));
        r.click(ApAction::Level);
        r.click(ApAction::Speed);
        r.panel.speed_kt = 260.0;
        // The reference ramps at SPEED_REF_RATE_KT_S.
        r.fly(1.0);
        assert!((r.ap.speed_ref_kt - 220.5).abs() < 0.05, "ramp {}", r.ap.speed_ref_kt);
        let mut c = ControlInput { throttle: 0.5, ..ControlInput::default() };
        r.plant.speed_kt = 200.0; // 20 kt below the reference: wanted > 1
        r.plant.sync();
        r.ap.throttle = 1.0;
        let i0 = r.ap.throttle_int;
        r.ap.step(&mut c, &r.plant.s, &[], true, DT);
        assert_eq!(c.throttle, 1.0);
        assert_eq!(r.ap.throttle_int, i0, "no integration at full travel");
    }

    #[test]
    fn w5b_from_above_the_path_descends_to_the_glideslope_and_captures_it() {
        // On the centreline 18 km out, 4,000 ft: ~850 ft above the path.
        let plant = Plant::new(4000.0, 270.0, 160.0).at([18_000.0, 0.0]);
        let mut r = Rig::new(plant);
        r.approaches = vec![Approach::new("27", [0.0, 0.0], 270.0)];
        r.panel.heading_deg = 270.0;
        r.panel.speed_kt = 160.0;
        r.click(ApAction::Heading);
        r.click(ApAction::Speed);
        r.click(ApAction::Approach);
        r.fly(DT);
        let st = r.ap.status();
        let (along, _) = r.approaches[0].track(r.plant.pos);
        let dev = (r.plant.s.height_agl_m - r.approaches[0].path_height_m(along)) / FT;
        assert!((f64::from(st.gs_dev_ft.unwrap()) - dev).abs() < 5.0, "{st:?} {dev}");
        assert_eq!((st.lateral, st.vertical), (Lateral::Localizer, Vertical::GsDescent), "{st:?}");
        let mut worst_sink = 0.0f64;
        let mut captured_at = None;
        for _ in 0..(300.0 / DT) as usize {
            r.tick();
            let st = r.ap.status();
            if st.vertical == Vertical::GsDescent {
                worst_sink = worst_sink.max(-r.plant.s.vertical_speed_mps / FT * 60.0);
            }
            if st.vertical == Vertical::Glideslope {
                captured_at = Some(r.approaches[0].track(r.plant.pos).0);
                break;
            }
        }
        let at = captured_at.expect("captured from above");
        assert!(at < -4.0 * 1852.0, "captured {at} m out");
        assert!(worst_sink <= MAX_GS_SINK_FPM + 100.0, "sink {worst_sink}");
        // Cancelling the approach clears the deviation and the descent.
        r.click(ApAction::Approach);
        r.fly(DT);
        let st = r.ap.status();
        assert!(st.gs_dev_ft.is_none() && st.vertical != Vertical::Glideslope && st.vertical != Vertical::GsDescent, "{st:?}");
    }

    #[test]
    fn w5b_below_the_path_waits_for_the_glideslope_as_before() {
        let plant = Plant::new(2000.0, 270.0, 160.0).at([18_000.0, 0.0]);
        let mut r = Rig::new(plant);
        r.approaches = vec![Approach::new("27", [0.0, 0.0], 270.0)];
        r.click(ApAction::Heading);
        r.click(ApAction::Approach);
        r.fly(5.0);
        let st = r.ap.status();
        assert!(st.gs_armed && st.vertical == Vertical::AltHold && st.gs_dev_ft.unwrap() < -500.0, "{st:?}");
    }

    #[test]
    fn approach_geometry_and_selection() {
        // Runway end at the origin, landing west (270).
        let a = Approach::new("27", [0.0, 0.0], 270.0);
        assert_eq!(a.name(), "27");
        let (along, cross) = a.track([10_000.0, 500.0]);
        assert!((along + 10_000.0).abs() < 1e-6, "{along}");
        assert!((cross - 500.0).abs() < 1e-6, "north of a westbound course is right: {cross}");
        assert!((a.path_height_m(-10_000.0) - 10_300.0 * (3.0 * DEG).tan()).abs() < 1e-6);
        let b = Approach::new("09", [-3000.0, 0.0], 90.0);
        let all = [a, b];
        assert_eq!(select_approach(&all, [10_000.0, 500.0], 270.0 * DEG), Some(0));
        assert_eq!(select_approach(&all, [10_000.0, 500.0], 90.0 * DEG), None, "heading away");
        assert_eq!(select_approach(&all, [-20_000.0, 0.0], 90.0 * DEG), Some(1));
        assert_eq!(select_approach(&all, [10_000.0, 9_000.0], 270.0 * DEG), None, "outside the sector");
    }

    #[test]
    fn ils_intercepts_the_localizer_rides_the_glideslope_and_hands_over() {
        // 15 km out, 1200 m north of a westbound course, 2500 ft, heading
        // 240 (a 30 degree intercept from the right).
        let plant = Plant::new(2500.0, 240.0, 160.0).at([15_000.0, 1200.0]);
        let mut r = Rig::new(plant);
        r.approaches = vec![Approach::new("27", [0.0, 0.0], 270.0)];
        r.panel.heading_deg = 240.0;
        r.panel.speed_kt = 160.0;
        r.click(ApAction::Heading);
        r.click(ApAction::Speed);
        r.click(ApAction::Approach);
        let st = r.ap.status();
        assert!(st.loc_armed && st.gs_armed && st.approach == Some(0), "{st:?}");
        let mut on_gs = false;
        let mut worst_gs_dev_ft = 0.0f64;
        for _ in 0..(400.0 / DT) as usize {
            r.tick();
            let st = r.ap.status();
            let (along, cross) = r.approaches[0].track(r.plant.pos);
            if st.vertical == Vertical::Glideslope {
                on_gs = true;
                if r.alt_ft() < 1500.0 {
                    let dev = (r.plant.s.height_agl_m - r.approaches[0].path_height_m(along)) / FT;
                    worst_gs_dev_ft = worst_gs_dev_ft.max(dev.abs());
                    assert!(cross.abs() < 30.0, "on the localizer below 1500 ft: {cross} m");
                }
            }
            if !st.engaged {
                break;
            }
        }
        assert!(on_gs, "glideslope captured");
        let st = r.ap.status();
        assert!(!st.engaged, "handed over");
        assert_eq!(st.warning, Some(ApWarning::ApDisconnect));
        assert!(r.alt_ft() < GLIDESLOPE_DISCONNECT_FT + 5.0 && r.alt_ft() > 50.0, "at {}", r.alt_ft());
        assert!(worst_gs_dev_ft < 40.0, "glideslope deviation {worst_gs_dev_ft} ft");
    }

    #[test]
    fn heading_cancels_an_armed_approach_and_reset_clears_everything() {
        let mut r = Rig::new(Plant::new(2500.0, 270.0, 160.0).at([15_000.0, 0.0]));
        r.approaches = vec![Approach::new("27", [0.0, 0.0], 270.0)];
        r.click(ApAction::Approach);
        assert!(r.ap.status().engaged && r.ap.status().approach.is_some());
        r.click(ApAction::Heading);
        assert_eq!(r.ap.status().approach, None);
        assert!(!r.ap.status().loc_armed && !r.ap.status().gs_armed);
        r.click(ApAction::Speed);
        r.ap.reset();
        let st = r.ap.status();
        assert!(!st.engaged && !st.autothrottle && st.warning.is_none());
        // The last click is not replayed after a reset.
        r.ap.input(r.panel, Some(&r.plant.s), &r.pilot, &r.approaches);
        assert!(!r.ap.status().autothrottle);
    }

    #[test]
    fn panel_input_is_clamped_and_wrapped() {
        let i = ApInput { heading_deg: -10.0, altitude_ft: 90_000.0, speed_kt: f32::NAN, ..ApInput::default() }.normalized();
        assert_eq!(i.heading_deg, 350.0);
        assert_eq!(i.altitude_ft, ALTITUDE_RANGE_FT.1);
        assert_eq!(i.speed_kt, ApInput::DEFAULT.speed_kt);
        for c in 0..=6 {
            assert_eq!(ApAction::from_code(c).unwrap().code(), c);
        }
        assert_eq!(ApAction::from_code(7), None);
    }
}
