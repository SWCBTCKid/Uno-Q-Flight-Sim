//! Gear, flap and speedbrake levers (INPUT-005, plan
//! the public 787 player-aircraft profile).
//!
//! The levers are **positions**, not axes. The I/O thread owns the virtual
//! levers that the stick buttons move (gx-app `StickLevers`) and publishes
//! them in every [`crate::ControlInput`]. [`crate::SimCore`] keeps the last
//! levers it read with **valid** input. During an input loss the mailbox
//! delivers neutral controls, but the core keeps the held levers, so a loss
//! can never raise the gear or move the flaps. Reset returns the core's
//! levers to the aircraft's initial set, and the I/O side resets its own
//! levers on the same press, so both agree again.
//!
//! **Squat switch.** While any wheel has weight on it, the core commands the
//! gear down whatever the lever says. A lever raised on the ground takes
//! effect after lift-off. The HUD shows the lever (`GEAR UP`) against the
//! gear position, so the disagreement is visible.

/// Lever positions as commanded by the pilot. Values are JSBSim command
/// norms: `flaps` is `fcs/flap-cmd-norm` and `speedbrake` is
/// `fcs/speedbrake-cmd-norm`, both 0..1.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Levers {
    pub gear_down: bool,
    pub flaps: f32,
    pub speedbrake: f32,
}

impl Default for Levers {
    /// Gear down, flaps up, speedbrake stowed. Gear down is the safe default
    /// for any value that was never set.
    fn default() -> Self {
        Self { gear_down: true, flaps: 0.0, speedbrake: 0.0 }
    }
}

impl Levers {
    pub fn normalized(self) -> Self {
        fn unit(v: f32) -> f32 {
            if v.is_finite() { v.clamp(0.0, 1.0) } else { 0.0 }
        }
        Self { gear_down: self.gear_down, flaps: unit(self.flaps), speedbrake: unit(self.speedbrake) }
    }

    /// What the FDM receives: the lever, except that weight on wheels holds
    /// the gear down (squat switch).
    pub fn effective(self, weight_on_wheels: bool) -> Self {
        Self { gear_down: self.gear_down || weight_on_wheels, ..self }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_gear_down_clean() {
        assert_eq!(Levers::default(), Levers { gear_down: true, flaps: 0.0, speedbrake: 0.0 });
    }

    #[test]
    fn normalized_clamps_and_rejects_non_finite() {
        let l = Levers { gear_down: false, flaps: 1.5, speedbrake: f32::NAN }.normalized();
        assert_eq!(l, Levers { gear_down: false, flaps: 1.0, speedbrake: 0.0 });
        assert_eq!(Levers { gear_down: true, flaps: -0.2, speedbrake: 0.5 }.normalized().flaps, 0.0);
    }

    #[test]
    fn squat_switch_holds_the_gear_down_on_the_ground_only() {
        let up = Levers { gear_down: false, ..Levers::default() };
        assert!(up.effective(true).gear_down, "weight on wheels: gear stays down");
        assert!(!up.effective(false).gear_down, "airborne: the lever wins");
        assert_eq!(up.effective(true).flaps, up.flaps);
    }
}
