//! Placeholder Global 5000 (S4): an untextured low-poly model with flat,
//! per-face shaded colours, in the JSBSim body frame relative to the
//! simulation's reference point.
//!
//! Frame: FRD (x forward, y right, z down), metres, origin at the JSBSim
//! vehicle location, which is the CG (`global5000.xml` `<location name="CG">`
//! x 790.82 in, z -29.07 in, structural frame x aft / y right / z up). The
//! gear contact points and the eye point below are converted from the same
//! file, so the drawn wheels touch exactly where JSBSim's gear touch (the
//! S4 on-runway check uses them). Overall size follows the published Global
//! 5000 dimensions (length 29.5 m, span 28.65 m, height 7.7 m); the shape is
//! deliberately simple (no licensing question: generated here, no external
//! asset).

use crate::math::{cross, normalize, sub, V3};
use crate::scene::Rgb;

/// One 3D vertex: position and colour (24 bytes, tightly packed).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vertex3 {
    pub pos: [f32; 3],
    pub color: Rgb,
}

const IN: f64 = 0.0254;
const CG_X_IN: f64 = 790.82;
const CG_Z_IN: f64 = -29.07;

/// Structural inches (x aft, y right, z up) to body FRD metres about the CG.
pub fn structural_to_body(x_in: f64, y_in: f64, z_in: f64) -> V3 {
    [(CG_X_IN - x_in) * IN, y_in * IN, (CG_Z_IN - z_in) * IN]
}

/// JSBSim gear contact points (`<contact type="BOGEY">` NOSE, LEFT_MAIN,
/// RIGHT_MAIN), body FRD metres: the wheel bottoms of the model.
pub fn gear_contacts() -> [(&'static str, V3); 3] {
    [
        ("nose", structural_to_body(151.16, 0.0, -139.54)),
        ("left_main", structural_to_body(822.43, -100.44, -139.54)),
        ("right_main", structural_to_body(822.43, 100.44, -139.54)),
    ]
}

/// JSBSim `EYEPOINT` (pilot's eye), body FRD metres.
pub fn eye_point() -> V3 {
    structural_to_body(81.40, -30.00, 70.00)
}

/// Where the flown aircraft touches the ground and where its pilot sits,
/// body FRD metres about its JSBSim CG: the S4 on-runway check
/// (`world::ground_check`) and the cockpit camera use the active one.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Geometry {
    pub contacts: [(&'static str, V3); 3],
    pub eye: V3,
    /// Chase-camera distance multiplier (1 for the 28.7 m-span Global 5000).
    pub chase_scale: f64,
}

impl Geometry {
    pub fn global5000() -> Self {
        Self { contacts: gear_contacts(), eye: eye_point(), chase_scale: 1.0 }
    }

    /// Stock JSBSim 787-8 (`787-8.xml`): CG x -16, z 22.5972 in; contacts
    /// NOSE_GEAR (-809.50, 0, -143.37), mains (77.11, +-193.60, -153.63) in.
    /// The eye is FlightGear's captain view (`787-8-set.xml`: x -0.603,
    /// y 2.242, z -22.612 m in view axes = model (-22.612, -0.603, 2.242)).
    pub fn b7878() -> Self {
        const CG: (f64, f64) = (-16.0, 22.5972);
        let s = |x: f64, y: f64, z: f64| [(CG.0 - x) * IN, y * IN, (CG.1 - z) * IN];
        Self {
            contacts: [
                ("nose", s(-809.50058, 0.0, -143.36872)),
                ("left_main", s(77.1058, -193.59978, -153.63242)),
                ("right_main", s(77.1058, 193.59978, -153.63242)),
            ],
            eye: s(-22.612 / IN, -0.603 / IN, 2.242 / IN),
            // 60.1 m span: about twice the Global 5000's, framed alike.
            chase_scale: 1.9,
        }
    }
}

static ACTIVE: std::sync::OnceLock<Geometry> = std::sync::OnceLock::new();

/// Select the flown aircraft's geometry once, before rendering starts.
/// Later calls are ignored (the first selection wins).
pub fn set_active(g: Geometry) {
    let _ = ACTIVE.set(g);
}

/// The active geometry; global5000 until [`set_active`] is called.
pub fn active() -> Geometry {
    ACTIVE.get().copied().unwrap_or_else(Geometry::global5000)
}

pub const WHITE: Rgb = [0.93, 0.94, 0.96];
pub const WING: Rgb = [0.70, 0.72, 0.76];
pub const ENGINE: Rgb = [0.52, 0.54, 0.58];
pub const FIN: Rgb = [0.12, 0.22, 0.48];
pub const WINDOW: Rgb = [0.06, 0.08, 0.13];
pub const TYRE: Rgb = [0.06, 0.06, 0.06];
pub const STRUT: Rgb = [0.45, 0.45, 0.47];

/// Flat shading baked per face: light from above, slightly ahead and left.
fn shade(color: Rgb, n: V3) -> Rgb {
    let light = normalize([0.3, -0.35, -1.0]); // FRD: up is -z
    let lambert = (n[0] * light[0] + n[1] * light[1] + n[2] * light[2]).abs();
    let k = (0.62 + 0.38 * lambert) as f32;
    color.map(|c| (c * k).min(1.0))
}

#[derive(Default)]
struct Builder {
    v: Vec<Vertex3>,
}

impl Builder {
    fn tri(&mut self, a: V3, b: V3, c: V3, color: Rgb) {
        let n = normalize(cross(sub(b, a), sub(c, a)));
        let col = shade(color, n);
        for p in [a, b, c] {
            self.v.push(Vertex3 { pos: p.map(|x| x as f32), color: col });
        }
    }

    fn quad(&mut self, a: V3, b: V3, c: V3, d: V3, color: Rgb) {
        self.tri(a, b, c, color);
        self.tri(a, c, d, color);
    }

    /// Prism of `sides` between rings; ring i: (x, centre z, radius y,
    /// radius z). The upper faces of segment `window` (if any) get WINDOW.
    fn body(&mut self, rings: &[(f64, f64, f64, f64)], sides: usize, y0: f64, color: Rgb, window: Option<usize>) {
        let angle = |k: f64| std::f64::consts::TAU * (k + 0.5) / sides as f64;
        let ring = |&(x, zc, ry, rz): &(f64, f64, f64, f64), k: usize| {
            let a = angle(k as f64);
            [x, y0 + ry * a.cos(), zc + rz * a.sin()]
        };
        for (i, w) in rings.windows(2).enumerate() {
            for k in 0..sides {
                let (a, b) = (ring(&w[0], k), ring(&w[0], (k + 1) % sides));
                let (c, d) = (ring(&w[1], (k + 1) % sides), ring(&w[1], k));
                // FRD: z down, so an upper face has a negative mid-angle sine.
                let upper = angle(k as f64 + 0.5).sin() < -0.2;
                let col = if window == Some(i) && upper { WINDOW } else { color };
                self.quad(a, b, c, d, col);
            }
        }
        for &end in &[rings[0], rings[rings.len() - 1]] {
            if end.2 > 0.0 {
                let centre = [end.0, y0, end.1];
                for k in 0..sides {
                    self.tri(centre, ring(&end, k), ring(&end, (k + 1) % sides), color);
                }
            }
        }
    }

    fn slab(&mut self, root_le: V3, root_te: V3, tip_te: V3, tip_le: V3, t: f64, color: Rgb) {
        // A thin plate: top and bottom faces plus the leading edge.
        let up = |p: V3| [p[0], p[1], p[2] - t / 2.0];
        let dn = |p: V3| [p[0], p[1], p[2] + t / 2.0];
        self.quad(up(root_le), up(root_te), up(tip_te), up(tip_le), color);
        self.quad(dn(root_le), dn(tip_le), dn(tip_te), dn(root_te), color);
        self.quad(up(root_le), up(tip_le), dn(tip_le), dn(root_le), color);
        self.quad(up(root_te), dn(root_te), dn(tip_te), up(tip_te), color);
        self.quad(up(tip_le), up(tip_te), dn(tip_te), dn(tip_le), color);
    }

    fn boxy(&mut self, c: V3, half: V3, color: Rgb) {
        let p = |sx: f64, sy: f64, sz: f64| [c[0] + sx * half[0], c[1] + sy * half[1], c[2] + sz * half[2]];
        let f = [
            [p(1., -1., -1.), p(1., 1., -1.), p(1., 1., 1.), p(1., -1., 1.)],
            [p(-1., -1., -1.), p(-1., -1., 1.), p(-1., 1., 1.), p(-1., 1., -1.)],
            [p(-1., -1., -1.), p(1., -1., -1.), p(1., -1., 1.), p(-1., -1., 1.)],
            [p(-1., 1., -1.), p(-1., 1., 1.), p(1., 1., 1.), p(1., 1., -1.)],
            [p(-1., -1., -1.), p(-1., 1., -1.), p(1., 1., -1.), p(1., -1., -1.)],
            [p(-1., -1., 1.), p(1., -1., 1.), p(1., 1., 1.), p(-1., 1., 1.)],
        ];
        for q in f {
            self.quad(q[0], q[1], q[2], q[3], color);
        }
    }
}

/// The placeholder model as a triangle list (body FRD, about the CG).
pub fn global5000() -> Vec<Vertex3> {
    let mut b = Builder::default();
    // Fuselage: nose tip 20.1 m ahead of the CG, tail 9.4 m behind, axis
    // 0.3 m below the CG, 1.35 m radius (2.7 m cabin).
    let zc = 0.30;
    b.body(
        &[
            (20.09, zc + 0.35, 0.0, 0.0),
            (19.2, zc + 0.15, 0.65, 0.6),
            (17.6, zc, 1.15, 1.15),
            (15.6, zc, 1.35, 1.35),
            (-3.5, zc, 1.35, 1.35),
            (-7.5, zc - 0.35, 0.85, 0.75),
            (-9.4, zc - 0.55, 0.25, 0.25),
        ],
        8,
        0.0,
        WHITE,
        Some(1),
    );
    // Wings: low, swept, 28.65 m span, 3 deg dihedral, winglets.
    for s in [-1.0, 1.0] {
        let root_le = [3.2, s * 1.2, zc + 1.0];
        let root_te = [-2.3, s * 1.2, zc + 1.0];
        let tip_te = [-7.6, s * 14.3, zc + 1.0 - 13.1 * 0.052];
        let tip_le = [-5.9, s * 14.3, zc + 1.0 - 13.1 * 0.052];
        b.slab(root_le, root_te, tip_te, tip_le, 0.30, WING);
        let top = |p: V3| [p[0] - 0.9, p[1], p[2] - 1.6];
        b.slab(tip_le, tip_te, top(tip_te), top(tip_le), 0.12, WING);
        // Engine nacelles on the rear fuselage, with a short pylon.
        b.body(&[(-2.6, zc - 0.7, 0.0, 0.0), (-2.8, zc - 0.7, 0.62, 0.62), (-6.2, zc - 0.7, 0.55, 0.55), (-6.6, zc - 0.7, 0.35, 0.35)], 8, s * 2.35, ENGINE, None);
        b.boxy([-4.6, s * 1.55, zc - 0.7], [1.2, 0.35, 0.1], ENGINE);
        // Horizontal tail on top of the fin (T-tail).
        b.slab([-7.9, s * 0.2, zc - 5.3], [-9.6, s * 0.2, zc - 5.3], [-10.2, s * 4.75, zc - 5.45], [-9.3, s * 4.75, zc - 5.45], 0.18, WHITE);
    }
    // Vertical fin: a plate in the x-z plane, both sides.
    for y in [-0.12, 0.12] {
        let p = |x: f64, z: f64| [x, y, zc + z];
        b.quad(p(-4.6, -1.2), p(-7.6, -5.3), p(-9.8, -5.3), p(-9.2, -0.6), FIN);
    }
    // Landing gear: struts down to the JSBSim contact points, wheel blocks
    // whose bottoms are exactly those points.
    for (_, c) in gear_contacts() {
        let wheel_r = 0.38;
        b.boxy([c[0], c[1], c[2] - wheel_r], [wheel_r, 0.28, wheel_r], TYRE);
        let top = zc + 1.0;
        b.boxy([c[0], c[1], (top + c[2] - 2.0 * wheel_r) / 2.0], [0.09, 0.09, (c[2] - 2.0 * wheel_r - top).abs() / 2.0 + 0.05], STRUT);
    }
    b.v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn b7878_geometry_matches_its_xml_and_sits_lower_and_longer() {
        let g = Geometry::b7878();
        // Mains 1.96 m behind the CG at 4.48 m below it; nose 20.15 m ahead.
        assert!((g.contacts[1].1[0] + 2.365).abs() < 0.01 && (g.contacts[1].1[2] - 4.476).abs() < 0.01, "{:?}", g.contacts[1]);
        assert!((g.contacts[0].1[0] - 20.155).abs() < 0.01, "{:?}", g.contacts[0]);
        assert_eq!(g.contacts[1].1[1], -g.contacts[2].1[1]);
        // Captain: 22.2 m ahead of the CG, left of the centreline, above it.
        assert!((g.eye[0] - 22.206).abs() < 0.01 && g.eye[1] < 0.0 && g.eye[2] < -1.0, "{:?}", g.eye);
        assert_eq!(Geometry::global5000().contacts, gear_contacts());
    }

    #[test]
    fn contacts_and_eye_match_global5000_xml() {
        let g = gear_contacts();
        assert!((g[0].1[0] - 16.2474).abs() < 1e-3 && (g[0].1[2] - 2.80594).abs() < 1e-4, "{g:?}");
        assert!((g[1].1[0] + 0.80289).abs() < 1e-4 && (g[1].1[1] + 2.55118).abs() < 1e-4);
        assert_eq!(g[1].1[2], g[2].1[2]);
        let e = eye_point();
        assert!((e[0] - 18.019).abs() < 1e-3 && (e[2] + 2.516).abs() < 1e-3, "{e:?}");
    }

    #[test]
    fn model_is_bounded_and_wheels_touch_the_contacts() {
        let m = global5000();
        assert_eq!(m.len() % 3, 0);
        assert!(m.len() < 4_000, "low poly: {} vertices", m.len());
        let lo = (0..3).map(|i| m.iter().map(|v| v.pos[i]).fold(f32::INFINITY, f32::min)).collect::<Vec<_>>();
        let hi = (0..3).map(|i| m.iter().map(|v| v.pos[i]).fold(f32::NEG_INFINITY, f32::max)).collect::<Vec<_>>();
        assert!((hi[0] - lo[0] - 29.9).abs() < 0.8, "length {}", hi[0] - lo[0]);
        assert!((hi[1] - lo[1] - 28.6).abs() < 0.3, "span {}", hi[1] - lo[1]);
        // Lowest point = the gear contact plane (z down).
        assert!((f64::from(hi[2]) - gear_contacts()[0].1[2]).abs() < 1e-5, "{}", hi[2]);
        let height = hi[2] - lo[2];
        assert!((height - 7.7).abs() < 0.6, "height {height}");
        assert!(m.iter().all(|v| v.color.iter().all(|c| (0.0..=1.0).contains(c))));
    }
}
