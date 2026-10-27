//! Pitch trim (controls diagnosis, `docs/fdm/controls-diagnosis`): the pilot
//! holds a trim switch, the trim position runs at a fixed rate, and the
//! position is written to JSBSim's `fcs/pitch-trim-cmd-norm`, which the
//! global5000 FCS adds to `fcs/elevator-cmd-norm` (`Pitch Trim Sum`, clipped
//! to +-1) before the elevator.
//!
//! * **Command**: [`crate::ControlInput::trim`], the switch, `+1` = nose-up
//!   held, `-1` = nose-down held, `0` = released. It travels through the
//!   control mailbox like any axis, so INPUT-002 loss delivers `0`: the trim
//!   **stops where it is and keeps acting** (it is aircraft state, like a
//!   stabilizer, not a pilot axis that returns to neutral).
//! * **Position**: owned by [`crate::SimCore`] on the simulation thread,
//!   integrated only while the core steps (paused = frozen), zeroed by Reset,
//!   and published in every snapshot as `AircraftState::pitch_trim`.
//! * **Rate**: [`RATE_LOW_SPEED`] per second up to [`RATE_FULL_BELOW_KT`],
//!   falling linearly to [`RATE_HIGH_SPEED`] at [`RATE_HALF_ABOVE_KT`] and
//!   above, as real stabilizer trim slows with airspeed. The diagnosis
//!   measured the elevator's pitch authority growing about with IAS^2.
//! * **Range**: +-[`LIMIT`] of the elevator's normalized travel, which covers
//!   the measured hold (+35 % pull at 130 kt to -8 % push at 220 kt) with
//!   margin, and always leaves the stick at least half the elevator.
//!
//! Sign: pilot convention, `+` = nose up. JSBSim's elevator command is
//! positive trailing-edge down (nose down), so the property is `-position`
//! (see [`crate::fdm_signs`]).

/// Trim travel per second while a switch is held, up to [`RATE_FULL_BELOW_KT`].
/// 0 to +30 % in 3.75 s, full range (-0.5..+0.5) in 12.5 s.
pub const RATE_LOW_SPEED: f64 = 0.08;
/// Trim travel per second at and above [`RATE_HALF_ABOVE_KT`].
pub const RATE_HIGH_SPEED: f64 = 0.04;
pub const RATE_FULL_BELOW_KT: f64 = 200.0;
pub const RATE_HALF_ABOVE_KT: f64 = 250.0;
/// Trim authority, fraction of the elevator's normalized travel.
pub const LIMIT: f64 = 0.5;

const MPS_TO_KT: f64 = 1.0 / 0.514_444;

/// The trim rate (normalized travel per second) at a calibrated airspeed.
pub fn rate_at(ias_kt: f64) -> f64 {
    if !ias_kt.is_finite() || ias_kt <= RATE_FULL_BELOW_KT {
        return RATE_LOW_SPEED;
    }
    if ias_kt >= RATE_HALF_ABOVE_KT {
        return RATE_HIGH_SPEED;
    }
    let u = (ias_kt - RATE_FULL_BELOW_KT) / (RATE_HALF_ABOVE_KT - RATE_FULL_BELOW_KT);
    RATE_LOW_SPEED + u * (RATE_HIGH_SPEED - RATE_LOW_SPEED)
}

/// The trim position. Deterministic: a pure function of the commands, the
/// airspeeds and the step count.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PitchTrim {
    position: f64,
}

impl PitchTrim {
    /// Normalized position, `+` = nose up, within `+-LIMIT`.
    pub fn position(&self) -> f64 {
        self.position
    }

    /// Advance one step: `command` is the (normalized) switch, `ias_mps` the
    /// calibrated airspeed of the state the step starts from.
    pub fn update(&mut self, command: f32, ias_mps: f64, dt_seconds: f64) {
        let c = if command > 0.5 {
            1.0
        } else if command < -0.5 {
            -1.0
        } else {
            return;
        };
        let rate = rate_at(ias_mps * MPS_TO_KT);
        self.position = (self.position + c * rate * dt_seconds).clamp(-LIMIT, LIMIT);
    }

    pub fn reset(&mut self) {
        self.position = 0.0;
    }

    /// Value for JSBSim `fcs/pitch-trim-cmd-norm` (positive = nose down).
    pub fn jsbsim_cmd_norm(&self) -> f64 {
        -self.position
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::STEP_SECONDS;

    fn hold(t: &mut PitchTrim, cmd: f32, ias_kt: f64, seconds: f64) {
        for _ in 0..(seconds * 120.0).round() as usize {
            t.update(cmd, ias_kt / MPS_TO_KT, STEP_SECONDS);
        }
    }

    #[test]
    fn nose_up_hold_runs_at_the_low_speed_rate_and_releases_in_place() {
        let mut t = PitchTrim::default();
        hold(&mut t, 1.0, 150.0, 1.0);
        assert!((t.position() - 0.08).abs() < 1e-9, "{}", t.position());
        hold(&mut t, 0.0, 150.0, 5.0);
        assert!((t.position() - 0.08).abs() < 1e-9, "released switch holds the trim");
        assert!((t.jsbsim_cmd_norm() + 0.08).abs() < 1e-9, "nose up = negative elevator command");
        hold(&mut t, -1.0, 150.0, 2.0);
        assert!((t.position() + 0.08).abs() < 1e-9);
    }

    #[test]
    fn rate_halves_with_airspeed_and_travel_is_limited() {
        assert_eq!(rate_at(120.0), RATE_LOW_SPEED);
        assert_eq!(rate_at(200.0), RATE_LOW_SPEED);
        assert!((rate_at(225.0) - 0.06).abs() < 1e-12);
        assert_eq!(rate_at(281.0), RATE_HIGH_SPEED);
        assert_eq!(rate_at(f64::NAN), RATE_LOW_SPEED);
        let mut t = PitchTrim::default();
        hold(&mut t, -1.0, 270.0, 1.0);
        assert!((t.position() + 0.04).abs() < 1e-9);
        hold(&mut t, 1.0, 100.0, 60.0);
        assert_eq!(t.position(), LIMIT);
        hold(&mut t, -1.0, 100.0, 60.0);
        assert_eq!(t.position(), -LIMIT);
        t.reset();
        assert_eq!(t.position(), 0.0);
    }

    #[test]
    fn partial_and_non_finite_commands_do_not_move_the_trim() {
        let mut t = PitchTrim::default();
        for c in [0.0, 0.4, -0.4, f32::NAN] {
            t.update(c, 70.0, STEP_SECONDS);
        }
        assert_eq!(t.position(), 0.0);
    }
}
