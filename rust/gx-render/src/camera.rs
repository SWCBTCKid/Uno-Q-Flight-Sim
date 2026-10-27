//! S4 cameras: chase (behind and above, heading-stabilized, no roll) and
//! cockpit (the JSBSim eye point, rolling and pitching with the aircraft).
//!
//! All matrices are built in f64. The ground is uploaded in absolute pack
//! coordinates, so its MVP carries the camera translation; the aircraft's
//! MVP is composed camera-relative (aircraft position minus eye in f64) so
//! nothing large reaches f32.

use crate::math::{add, look_dir, m3_mul_v, normalize, perspective, quat_to_m3, scale, sub, M3, M4, V3};
use crate::scene::{HEIGHT, WIDTH};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CameraMode {
    Chase,
    Cockpit,
}

impl CameraMode {
    pub fn name(self) -> &'static str {
        match self {
            CameraMode::Chase => "chase",
            CameraMode::Cockpit => "cockpit",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            CameraMode::Chase => "CHASE CAM",
            CameraMode::Cockpit => "COCKPIT CAM",
        }
    }

    pub fn next(self) -> Self {
        match self {
            CameraMode::Chase => CameraMode::Cockpit,
            CameraMode::Cockpit => CameraMode::Chase,
        }
    }

    /// The mode after `presses` camera-button presses from `Chase`.
    pub fn after_presses(presses: u64) -> Self {
        if presses % 2 == 0 {
            CameraMode::Chase
        } else {
            CameraMode::Cockpit
        }
    }

    pub fn vfov_deg(self) -> f64 {
        match self {
            CameraMode::Chase => 50.0,
            CameraMode::Cockpit => 55.0,
        }
    }

    fn near_m(self) -> f64 {
        match self {
            CameraMode::Chase => 1.0,
            CameraMode::Cockpit => 0.4,
        }
    }
}

pub const FAR_M: f64 = 220_000.0;
/// Chase camera: distance behind the CG, height above it, aim point offsets.
pub const CHASE_BACK_M: f64 = 42.0;
pub const CHASE_UP_M: f64 = 9.0;

/// A camera for one frame.
#[derive(Clone, Copy, Debug)]
pub struct View {
    pub mode: CameraMode,
    pub eye: V3,
    /// Rotation-only view matrix (eye at the origin).
    pub rot: M4,
    pub proj: M4,
    /// World-space unit vectors of the camera (for the CPU ray reference).
    pub forward: V3,
    pub right: V3,
    pub up: V3,
}

/// Aircraft pose for the renderer: ENU position of the JSBSim CG and the
/// body(FRD)-to-ENU quaternion `[x, y, z, w]` (gx-sim `LocalEnuPose`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub position_m: V3,
    pub orientation_xyzw: [f64; 4],
}

impl Pose {
    pub fn rotation(&self) -> M3 {
        quat_to_m3(self.orientation_xyzw)
    }

    /// Body FRD point to ENU.
    pub fn to_world(&self, body: V3) -> V3 {
        add(self.position_m, m3_mul_v(&self.rotation(), body))
    }

    /// Model matrix (body FRD -> ENU) relative to `origin`.
    pub fn model_relative(&self, origin: V3) -> M4 {
        M4::from_rt(&self.rotation(), sub(self.position_m, origin))
    }
}

impl View {
    pub fn new(mode: CameraMode, pose: &Pose) -> Self {
        let r = pose.rotation();
        let nose = m3_mul_v(&r, [1.0, 0.0, 0.0]);
        let (eye, forward, up) = match mode {
            CameraMode::Cockpit => (pose.to_world(crate::aircraft::active().eye), nose, m3_mul_v(&r, [0.0, 0.0, -1.0])),
            CameraMode::Chase => {
                let mut h = [nose[0], nose[1], 0.0];
                if (h[0] * h[0] + h[1] * h[1]).sqrt() < 0.2 {
                    // Nose near vertical: follow the body's down vector instead.
                    let d = m3_mul_v(&r, [0.0, 0.0, 1.0]);
                    h = [-d[0], -d[1], 0.0];
                }
                let h = normalize(h);
                let k = crate::aircraft::active().chase_scale;
                let eye = add(add(pose.position_m, scale(h, -CHASE_BACK_M * k)), [0.0, 0.0, CHASE_UP_M * k]);
                let target = add(add(pose.position_m, scale(h, 6.0)), [0.0, 0.0, 1.5]);
                (eye, sub(target, eye), [0.0, 0.0, 1.0])
            }
        };
        let rot = look_dir([0.0; 3], forward, up);
        let proj = perspective(mode.vfov_deg().to_radians(), f64::from(WIDTH / HEIGHT), mode.near_m(), FAR_M);
        let f = normalize(forward);
        let right = [rot.0[0][0], rot.0[1][0], rot.0[2][0]];
        let cam_up = [rot.0[0][1], rot.0[1][1], rot.0[2][1]];
        View { mode, eye, rot, proj, forward: f, right, up: cam_up }
    }

    /// MVP of geometry stored in absolute pack coordinates.
    pub fn world_mvp(&self) -> M4 {
        let mut t = M4::identity();
        t.0[3] = [-self.eye[0], -self.eye[1], -self.eye[2], 1.0];
        self.proj.mul(&self.rot).mul(&t)
    }

    /// MVP of the aircraft model (body FRD) at `pose`.
    pub fn model_mvp(&self, pose: &Pose) -> M4 {
        self.proj.mul(&self.rot).mul(&pose.model_relative(self.eye))
    }

    /// Pixels per degree at the screen centre (for a conformal HUD ladder).
    pub fn px_per_deg(&self) -> f32 {
        let half = (self.mode.vfov_deg().to_radians() / 2.0).tan();
        ((f64::from(HEIGHT) / 2.0) / half * std::f64::consts::PI / 180.0) as f32
    }

    /// World-space ray through the centre of pixel (x, y) (top-left origin).
    pub fn ray(&self, x: f64, y: f64) -> V3 {
        let half = (self.mode.vfov_deg().to_radians() / 2.0).tan();
        let aspect = f64::from(WIDTH / HEIGHT);
        let nx = (x + 0.5) / f64::from(WIDTH) * 2.0 - 1.0;
        let ny = 1.0 - (y + 0.5) / f64::from(HEIGHT) * 2.0;
        normalize(add(add(self.forward, scale(self.right, nx * half * aspect)), scale(self.up, ny * half)))
    }

    /// Screen position (pixels, top-left origin) of a world point, if in front.
    pub fn project(&self, p: V3) -> Option<[f64; 2]> {
        let c = self.world_mvp().apply(p);
        (c[3] > 0.0).then(|| [(c[0] / c[3] + 1.0) / 2.0 * f64::from(WIDTH), (1.0 - c[1] / c[3]) / 2.0 * f64::from(HEIGHT)])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Heading 90 (east), level: body FRD -> ENU maps x to east, y to south,
    /// z to down. Quaternion of that rotation.
    pub fn east_level() -> [f64; 4] {
        // body->NED is yaw 90; NED->ENU swaps x/y and flips z: R = [[0,1,0],[1,0,0],[0,0,-1]]*Rz(90).
        // Rz(90) = [[0,-1,0],[1,0,0],[0,0,1]] -> R = [[1,0,0],[0,-1,0],[0,0,-1]]: 180 deg about x.
        [1.0, 0.0, 0.0, 0.0]
    }

    #[test]
    fn east_level_pose_maps_body_axes() {
        let p = Pose { position_m: [-1470.0, 0.0, 2.7], orientation_xyzw: east_level() };
        let r = p.rotation();
        assert_eq!(m3_mul_v(&r, [1.0, 0.0, 0.0]), [1.0, 0.0, 0.0], "nose east");
        assert_eq!(m3_mul_v(&r, [0.0, 1.0, 0.0]), [0.0, -1.0, 0.0], "right wing south");
        assert_eq!(m3_mul_v(&r, [0.0, 0.0, 1.0]), [0.0, 0.0, -1.0], "down is down");
    }

    #[test]
    fn chase_sees_the_aircraft_ahead_and_the_runway_below() {
        let p = Pose { position_m: [-1470.0, 0.0, 2.7], orientation_xyzw: east_level() };
        let v = View::new(CameraMode::Chase, &p);
        assert!((v.eye[0] - (-1470.0 - CHASE_BACK_M)).abs() < 1e-9 && (v.eye[2] - (2.7 + CHASE_UP_M)).abs() < 1e-9);
        let s = v.project(p.position_m).unwrap();
        assert!((s[0] - 640.0).abs() < 1.0 && s[1] > 360.0 && s[1] < 520.0, "aircraft just below centre: {s:?}");
        // Runway far ahead is near the horizon, above the aircraft on screen.
        let far = v.project([1_400.0, 0.0, 0.0]).unwrap();
        assert!(far[1] < s[1] && far[1] > 200.0, "horizon ~9 deg above the aim: {far:?}");
        // The centre ray agrees with projection.
        let r = v.ray(639.5, 359.5);
        assert!((r[0] - v.forward[0]).abs() < 1e-9 && (r[2] - v.forward[2]).abs() < 1e-9);
        let mvp = v.model_mvp(&p).apply([0.0; 3]);
        let w = v.world_mvp().apply(p.position_m);
        assert!((mvp[0] / mvp[3] - w[0] / w[3]).abs() < 1e-9, "camera-relative model matches world");
    }

    #[test]
    fn cockpit_rolls_with_the_aircraft_and_switches() {
        let p = Pose { position_m: [0.0, 0.0, 300.0], orientation_xyzw: east_level() };
        let v = View::new(CameraMode::Cockpit, &p);
        assert!((v.eye[0] - 18.019).abs() < 1e-3 && (v.eye[2] - (300.0 + 2.516)).abs() < 1e-3, "{:?}", v.eye);
        assert!((v.up[2] - 1.0).abs() < 1e-9);
        // A point straight ahead at eye height is at the centre.
        let c = v.project([5_000.0, v.eye[1], v.eye[2]]).unwrap();
        assert!((c[0] - 640.0).abs() < 0.5 && (c[1] - 360.0).abs() < 0.5, "{c:?}");
        // 10 deg below the nose is 10 * px_per_deg below the centre (to first order).
        let d = 5_000.0 * 10f64.to_radians().tan();
        let b = v.project([5_000.0, v.eye[1], v.eye[2] - d]).unwrap();
        let ppd = f64::from(v.px_per_deg());
        assert!((b[1] - 360.0 - 10.0 * ppd).abs() < 0.03 * 10.0 * ppd, "{b:?} vs {ppd}");
        assert_eq!(CameraMode::after_presses(0), CameraMode::Chase);
        assert_eq!(CameraMode::after_presses(3), CameraMode::Cockpit);
        assert_eq!(CameraMode::Chase.next(), CameraMode::Cockpit);
    }
}
