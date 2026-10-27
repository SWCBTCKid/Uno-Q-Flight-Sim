//! Pilot controls -> JSBSim command properties: the one place where the sign
//! conventions meet. Pure and FFI-free, so it is tested on every host.
//!
//! | gx `ControlInput` (pilot convention)   | JSBSim property            | value      |
//! |----------------------------------------|----------------------------|------------|
//! | `roll` +1 = stick right                | `fcs/aileron-cmd-norm`     | `+roll`    |
//! | `pitch` +1 = stick pulled back         | `fcs/elevator-cmd-norm`    | `-pitch`   |
//! | `yaw` +1 = twist right (right pedal)   | `fcs/rudder-cmd-norm`      | `-yaw`     |
//! | `yaw` +1 = twist right                 | `fcs/steer-cmd-norm`       | `+yaw`     |
//! | `throttle` 1 = lever forward           | `fcs/throttle-cmd-norm[i]` | `throttle` |
//! | `left_brake` / `right_brake`           | `fcs/{left,right}-brake-cmd-norm` | as is |
//! | trim position +1 = nose up (`SimCore`)  | `fcs/pitch-trim-cmd-norm`  | `-position` |
//!
//! JSBSim's own conventions (the same ones FlightGear's JSBSim interface
//! uses: it passes aileron and elevator through, negates the rudder and
//! passes the rudder to the steering): positive `aileron-cmd-norm` rolls
//! right; positive `elevator-cmd-norm` is trailing edge down, **nose down**;
//! positive `rudder-cmd-norm` yaws **left**; `pitch-trim-cmd-norm` is summed
//! with `elevator-cmd-norm` (global5000 `Pitch Trim Sum`), so it shares the
//! elevator's sign; positive `steer-cmd-norm`
//! steers right. The M1-001 flight scripts (`tests/m1/scripts`) close their
//! loops with exactly these signs (e.g. `elevator = -0.4 (theta_cmd -
//! theta) + 0.06 q`, `rudder = 0.02 r - 0.08 heading_error`, `aileron =
//! 0.04 (phi_cmd - phi) - 0.02 p`). The board test
//! `gx-app/tests/s2_jsbsim.rs` checks the resulting body-rate signs in the
//! air.

use crate::ControlInput;

/// Values written to JSBSim's command properties for one step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JsbsimCommands {
    pub aileron: f64,
    pub elevator: f64,
    pub rudder: f64,
    pub steer: f64,
    pub throttle: f64,
    pub left_brake: f64,
    pub right_brake: f64,
}

impl JsbsimCommands {
    /// `input` must already be normalized (`SimCore` does that).
    pub fn from_input(input: ControlInput) -> Self {
        Self {
            aileron: f64::from(input.roll),
            elevator: -f64::from(input.pitch),
            rudder: -f64::from(input.yaw),
            steer: f64::from(input.yaw),
            throttle: f64::from(input.throttle),
            left_brake: f64::from(input.left_brake),
            right_brake: f64::from(input.right_brake),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cmd(f: impl FnOnce(&mut ControlInput)) -> JsbsimCommands {
        let mut c = ControlInput::default();
        f(&mut c);
        JsbsimCommands::from_input(c.normalized())
    }

    #[test]
    fn pull_back_is_negative_elevator_which_is_nose_up_in_jsbsim() {
        assert_eq!(cmd(|c| c.pitch = 1.0).elevator, -1.0);
        assert_eq!(cmd(|c| c.pitch = -0.5).elevator, 0.5);
    }

    #[test]
    fn stick_right_is_positive_aileron_right_roll() {
        assert_eq!(cmd(|c| c.roll = 0.75).aileron, 0.75);
    }

    #[test]
    fn twist_right_is_negative_rudder_and_positive_steer() {
        let c = cmd(|c| c.yaw = 1.0);
        assert_eq!((c.rudder, c.steer), (-1.0, 1.0));
    }

    #[test]
    fn throttle_and_brakes_pass_through_and_neutral_is_all_zero() {
        let c = cmd(|c| {
            c.throttle = 0.8;
            c.left_brake = 1.0;
            c.right_brake = 0.25;
        });
        assert_eq!((c.throttle, c.left_brake, c.right_brake), (f64::from(0.8f32), 1.0, 0.25));
        let n = JsbsimCommands::from_input(ControlInput::default());
        assert_eq!(n, JsbsimCommands {
            aileron: 0.0, elevator: -0.0, rudder: -0.0, steer: 0.0, throttle: 0.0,
            left_brake: 0.0, right_brake: 0.0,
        });
    }
}
