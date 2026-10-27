//! S2 aircraft presets and the runway-threshold initial position, derived
//! from the GXWORLD 2.0 pack's frame and geoid separation. FFI-free, so the
//! geometry is tested on every host; `jsbsim_setup` (feature `jsbsim`)
//! turns it into the gx-sim `JsbsimSetup`.

use crate::WorldFrame;
use gx_sim::{Approach, GeodeticDatum, GeodeticPosition, Levers};
use gx_world::Layer;

/// Aircraft the S2 binary can fly. `global5000` is the M1 target; `787-8`
/// is the SPIKE-002b player aircraft (BBJ 787, plan
/// `2026-10-24-b787-player-aircraft.md`); `c172p` is the M1-001 reference and
/// fallback.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Aircraft {
    Global5000,
    B7878,
    C172p,
}

/// One flap lever detent: the label the HUD shows (degrees) and the
/// `fcs/flap-cmd-norm` it commands.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlapDetent {
    pub label: u8,
    pub norm: f32,
}

const fn detent(label: u8, norm: f32) -> FlapDetent {
    FlapDetent { label, norm }
}

/// global5000.xml: flaps 0 / 15 / 30 deg over the 0..1 command.
const GLOBAL5000_FLAPS: [FlapDetent; 3] = [detent(0, 0.0), detent(15, 0.5), detent(30, 1.0)];
/// The 787 flap lever (UP, 1, 5, 15, 20, 25, 30) on the stock JSBSim 787-8,
/// whose flap command spans 0..35 deg.
const B7878_FLAPS: [FlapDetent; 7] = [
    detent(0, 0.0), detent(1, 1.0 / 35.0), detent(5, 5.0 / 35.0), detent(15, 15.0 / 35.0),
    detent(20, 20.0 / 35.0), detent(25, 25.0 / 35.0), detent(30, 30.0 / 35.0),
];
/// c172p.xml: 0 / 10 / 20 / 30 deg over 0..1.
const C172P_FLAPS: [FlapDetent; 4] = [detent(0, 0.0), detent(10, 1.0 / 3.0), detent(20, 2.0 / 3.0), detent(30, 1.0)];

impl Aircraft {
    pub fn parse(name: &str) -> Option<Self> {
        match name {
            "global5000" => Some(Self::Global5000),
            "787-8" => Some(Self::B7878),
            "c172p" => Some(Self::C172p),
            _ => None,
        }
    }

    /// JSBSim model directory name.
    pub fn name(self) -> &'static str {
        match self {
            Self::Global5000 => "global5000",
            Self::B7878 => "787-8",
            Self::C172p => "c172p",
        }
    }

    pub fn engines(self) -> usize {
        match self {
            Self::Global5000 | Self::B7878 => 2,
            Self::C172p => 1,
        }
    }

    /// Rotation speed (KCAS) for the S2 prompts and checker. These are the
    /// M1-001 scripted-profile values at the same weight and flap setting
    /// (`tests/m1/scripts/*_flight.xml`, `VR`).
    pub fn vr_kt(self) -> f64 {
        match self {
            Self::Global5000 => 108.0,
            // Light BBJ (no payload, 20 t fuel, 129 t) at flaps 5.
            Self::B7878 => 135.0,
            Self::C172p => 55.0,
        }
    }

    /// Height of the JSBSim reference point above the runway when parked
    /// (the M1-001 runway initial conditions).
    pub fn parked_agl_m(self) -> f64 {
        match self {
            Self::Global5000 => 9.5 * 0.3048,
            // CG (z 22.6 in) to the main contacts (z -153.6 in) is 14.7 ft;
            // the same 0.3 ft margin as global5000.
            Self::B7878 => 15.0 * 0.3048,
            Self::C172p => 4.7 * 0.3048,
        }
    }

    /// The flap lever's detents, from up to full.
    pub fn flap_detents(self) -> &'static [FlapDetent] {
        match self {
            Self::Global5000 => &GLOBAL5000_FLAPS,
            Self::B7878 => &B7878_FLAPS,
            Self::C172p => &C172P_FLAPS,
        }
    }

    /// Index in [`Aircraft::flap_detents`] of the take-off flap: global5000
    /// 15 (the M1-001 profile), 787 flaps 5, c172p up.
    pub fn takeoff_flap_index(self) -> usize {
        match self {
            Self::Global5000 => 1,
            Self::B7878 => 2,
            Self::C172p => 0,
        }
    }

    /// Levers at start and after every Reset: gear down, take-off flap,
    /// speedbrake stowed.
    pub fn initial_levers(self) -> Levers {
        Levers {
            gear_down: true,
            flaps: self.flap_detents()[self.takeoff_flap_index()].norm,
            speedbrake: 0.0,
        }
    }

    /// Properties written after every (re)initialization. Gear, flap and
    /// speedbrake start at [`Aircraft::initial_levers`]; from the first step
    /// on, the levers (INPUT-005) drive them.
    pub fn settings(self) -> Vec<(String, f64)> {
        let flaps = f64::from(self.initial_levers().flaps);
        let mut s: Vec<(String, f64)> = vec![
            ("gear/gear-cmd-norm".into(), 1.0),
            ("fcs/center-brake-cmd-norm".into(), 0.0),
        ];
        match self {
            Self::Global5000 => {
                s.push(("fcs/flap-cmd-norm".into(), flaps));
                s.push(("fcs/speedbrake-cmd-norm".into(), 0.0));
                for tank in 0..3 {
                    s.push((format!("propulsion/tank[{tank}]/contents-lbs"), 5031.0));
                }
            }
            Self::B7878 => {
                s.push(("fcs/flap-cmd-norm".into(), flaps));
                s.push(("fcs/speedbrake-cmd-norm".into(), 0.0));
                // BBJ, light: 10 t in each wing tank, centre tank empty, and
                // the airline cabin/cargo point masses (1..5) removed; the
                // crew (0) stays. 283,642 lb; lift-off ~149 KIAS rotating at
                // VR 135 (host JSBSim 1.3.2 profile, 2026-10-24).
                s.push(("propulsion/tank[0]/contents-lbs".into(), 22_046.0));
                s.push(("propulsion/tank[1]/contents-lbs".into(), 22_046.0));
                s.push(("propulsion/tank[2]/contents-lbs".into(), 0.0));
                for i in 1..6 {
                    s.push((format!("inertia/pointmass-weight-lbs[{i}]"), 0.0));
                }
            }
            Self::C172p => {
                s.push(("fcs/flap-cmd-norm".into(), 0.0));
                s.push(("fcs/mixture-cmd-norm[0]".into(), 1.0));
            }
        }
        s
    }
}

/// Where the aircraft starts: a runway threshold of the world pack.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Threshold {
    pub runway: &'static str,
    /// Position on the centreline, `THRESHOLD_INSET_M` inside the runway end.
    pub enu_m: [f64; 2],
    /// True heading along the runway, degrees.
    pub heading_deg: f64,
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    /// Runway surface elevation above MSL (pack geoid model).
    pub surface_msl_m: f64,
}

pub const THRESHOLD_INSET_M: f64 = 30.0;

/// The western threshold of the pack's first runway, facing along it. The
/// synthetic pack's runway is axis-aligned; anything else is refused rather
/// than guessed.
pub fn threshold(frame: &WorldFrame) -> Result<Threshold, String> {
    let (name, positions) = frame
        .world()
        .chunks
        .iter()
        .flat_map(|c| c.lods.iter().filter(|l| l.level == 0))
        .flat_map(|l| l.layers.iter())
        .find_map(|layer| match layer {
            Layer::Runway(m) => Some((m.name.clone(), m.mesh.positions.clone())),
            _ => None,
        })
        .ok_or("world pack has no runway")?;
    if positions.is_empty() {
        return Err("runway mesh is empty".into());
    }
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in &positions {
        for i in 0..3 {
            lo[i] = lo[i].min(f64::from(p[i]));
            hi[i] = hi[i].max(f64::from(p[i]));
        }
    }
    let (dx, dy) = (hi[0] - lo[0], hi[1] - lo[1]);
    // Axis-aligned rectangle: four corners on the bounding box.
    let on_box = positions.iter().all(|p| {
        let near = |v: f32, a: f64, b: f64| (f64::from(v) - a).abs() < 0.01 || (f64::from(v) - b).abs() < 0.01;
        near(p[0], lo[0], hi[0]) && near(p[1], lo[1], hi[1])
    });
    let (enu, heading_deg) = if on_box && dx.max(dy) >= 1000.0 && dx.min(dy) > 0.0 {
        if dx >= dy {
            ([lo[0] + THRESHOLD_INSET_M, 0.5 * (lo[1] + hi[1])], 90.0)
        } else {
            ([0.5 * (lo[0] + hi[0]), lo[1] + THRESHOLD_INSET_M], 0.0)
        }
    } else {
        // SPIKE-002 surveyed pack: one oriented rectangle (4 corners).
        oriented_threshold(&positions).ok_or_else(|| format!("runway {name} is not a rectangular strip of >= 1000 m"))?
    };
    let surface_up_m = hi[2];
    let origin = frame.origin();
    let origin_msl_m = origin.ellipsoid_height_m - frame.msl_to_ellipsoid_m();
    let surface_msl_m = origin_msl_m + surface_up_m;
    let (latitude_deg, longitude_deg) =
        enu_to_latlon(frame, enu, surface_msl_m + frame.msl_to_ellipsoid_m())?;
    Ok(Threshold {
        runway: if name == "10L/28R" { "10L/28R" } else { "runway" },
        enu_m: enu,
        heading_deg,
        latitude_deg,
        longitude_deg,
        surface_msl_m,
    })
}

/// Western-end threshold of an oriented rectangular runway (4 corners in
/// any order): `THRESHOLD_INSET_M` inside the end whose midpoint is further
/// west, on the centreline, facing along the runway (true heading).
fn oriented_threshold(positions: &[[f32; 3]]) -> Option<([f64; 2], f64)> {
    let [from, to] = oriented_ends(positions)?;
    let len = ((to[0] - from[0]).powi(2) + (to[1] - from[1]).powi(2)).sqrt();
    let u = [(to[0] - from[0]) / len, (to[1] - from[1]) / len];
    let enu = [from[0] + THRESHOLD_INSET_M * u[0], from[1] + THRESHOLD_INSET_M * u[1]];
    Some((enu, u[0].atan2(u[1]).to_degrees().rem_euclid(360.0)))
}

/// The two ends (midpoints of the short edges, west end first) of an
/// oriented rectangular runway of >= 1000 m (4 corners in any order).
fn oriented_ends(positions: &[[f32; 3]]) -> Option<[[f64; 2]; 2]> {
    if positions.len() != 4 {
        return None;
    }
    let p: Vec<[f64; 2]> = positions.iter().map(|q| [f64::from(q[0]), f64::from(q[1])]).collect();
    let d = |a: [f64; 2], b: [f64; 2]| ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt();
    // Corner 0's two neighbours are the two nearest other corners.
    let mut others = [1usize, 2, 3];
    others.sort_by(|&a, &b| d(p[0], p[a]).total_cmp(&d(p[0], p[b])));
    let (short, long, opposite) = (others[0], others[1], others[2]);
    let (width, length) = (d(p[0], p[short]), d(p[0], p[long]));
    let rect = (d(p[short], p[opposite]) - length).abs() < 0.05
        && (d(p[long], p[opposite]) - width).abs() < 0.05
        && (d(p[0], p[opposite]) - d(p[short], p[long])).abs() < 0.05;
    if !rect || length < 1000.0 || width <= 0.0 {
        return None;
    }
    let mid = |a: [f64; 2], b: [f64; 2]| [0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])];
    let (mut from, mut to) = (mid(p[0], p[short]), mid(p[long], p[opposite]));
    if to[0] < from[0] {
        std::mem::swap(&mut from, &mut to);
    }
    Some([from, to])
}

/// The two ends of a runway mesh: an axis-aligned rectangle (every vertex
/// on the bounding box, as the synthetic pack's) or an oriented 4-corner
/// rectangle (the SPIKE-002 surveyed pack's), >= 1000 m long.
fn runway_ends(positions: &[[f32; 3]]) -> Option<[[f64; 2]; 2]> {
    if positions.is_empty() {
        return None;
    }
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for p in positions {
        for i in 0..2 {
            lo[i] = lo[i].min(f64::from(p[i]));
            hi[i] = hi[i].max(f64::from(p[i]));
        }
    }
    let (dx, dy) = (hi[0] - lo[0], hi[1] - lo[1]);
    let on_box = positions.iter().all(|p| {
        let near = |v: f32, a: f64, b: f64| (f64::from(v) - a).abs() < 0.01 || (f64::from(v) - b).abs() < 0.01;
        near(p[0], lo[0], hi[0]) && near(p[1], lo[1], hi[1])
    });
    if on_box && dx.max(dy) >= 1000.0 && dx.min(dy) > 0.0 {
        let (mx, my) = (0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]));
        return Some(if dx >= dy { [[lo[0], my], [hi[0], my]] } else { [[mx, lo[1]], [mx, hi[1]]] });
    }
    oriented_ends(positions)
}

/// I4 (AP-001): an ILS approach to each end of every level-0 runway layer,
/// in the pack's frame. The threshold is the runway end on the centreline
/// and the course is true, along the runway from that end. Each end is
/// named from the layer name (`10L/28R` -> `10L`, `28R`) by the runway
/// number nearest its course (the numbers are magnetic: within 30 degrees);
/// a name that does not match gives the number of the course itself. A
/// runway that is not a rectangle of >= 1000 m is skipped, and a runway
/// repeated in several chunks is listed once.
pub fn approaches(frame: &WorldFrame) -> Vec<Approach> {
    let mut out: Vec<Approach> = Vec::new();
    let runways = frame
        .world()
        .chunks
        .iter()
        .flat_map(|c| c.lods.iter().filter(|l| l.level == 0))
        .flat_map(|l| l.layers.iter())
        .filter_map(|layer| match layer {
            Layer::Runway(m) => Some(m),
            _ => None,
        });
    for m in runways {
        let Some([a, b]) = runway_ends(&m.mesh.positions) else { continue };
        for (end, other) in [(a, b), (b, a)] {
            let course = (other[0] - end[0]).atan2(other[1] - end[1]).to_degrees().rem_euclid(360.0);
            let name = end_name(&m.name, course);
            let approach = Approach::new(&name, end, course);
            let dup = out.iter().any(|o| {
                o.name == approach.name
                    && (o.threshold_m[0] - end[0]).abs() < 1.0
                    && (o.threshold_m[1] - end[1]).abs() < 1.0
            });
            if !dup {
                out.push(approach);
            }
        }
    }
    out
}

/// The part of a runway name (`10L/28R`) whose number (tens of degrees,
/// magnetic) is within 30 degrees of `course_deg` (true); otherwise the
/// course's own number (`01`..`36`).
fn end_name(layer: &str, course_deg: f64) -> String {
    let diff = |n: u32| (f64::from(n) * 10.0 - course_deg + 540.0).rem_euclid(360.0) - 180.0;
    let best = layer
        .split('/')
        .filter_map(|part| {
            let part = part.trim();
            let digits = part.bytes().take_while(u8::is_ascii_digit).count();
            let n: u32 = part[..digits].parse().ok()?;
            ((1..=36).contains(&n) && part.len() <= 3).then_some((part, diff(n).abs()))
        })
        .min_by(|a, b| a.1.total_cmp(&b.1));
    match best {
        Some((part, d)) if d <= 30.0 => part.to_owned(),
        _ => {
            let n = (course_deg / 10.0).round() as u32;
            format!("{:02}", if n == 0 { 36 } else { n })
        }
    }
}

/// Latitude/longitude whose ENU east/north in `frame` (at ellipsoid height
/// `h`) are `target`: Newton iteration on gx-world's own forward conversion,
/// so the result agrees with the pose the core will compute to < 1 mm.
pub fn enu_to_latlon(frame: &WorldFrame, target: [f64; 2], h: f64) -> Result<(f64, f64), String> {
    let origin = frame.origin();
    let mut p = GeodeticPosition {
        datum: GeodeticDatum::Wgs84Ellipsoid,
        latitude_deg: origin.latitude_deg,
        longitude_deg: origin.longitude_deg,
        ellipsoid_height_m: h,
    };
    const M_PER_DEG: f64 = 111_320.0;
    for _ in 0..10 {
        let enu = frame.enu_m(p);
        let (ex, ny) = (target[0] - enu[0], target[1] - enu[1]);
        if ex.abs() < 1e-4 && ny.abs() < 1e-4 {
            return Ok((p.latitude_deg, p.longitude_deg));
        }
        p.latitude_deg += ny / M_PER_DEG;
        p.longitude_deg += ex / (M_PER_DEG * p.latitude_deg.to_radians().cos());
    }
    Err("threshold position did not converge".into())
}

#[cfg(feature = "jsbsim")]
pub fn jsbsim_setup(aircraft: Aircraft, t: &Threshold) -> gx_sim::JsbsimSetup {
    use gx_sim::{EngineOutput, InitialConditions, JsbsimSetup};
    JsbsimSetup {
        aircraft: aircraft.name().into(),
        engines: aircraft.engines(),
        engine_output: match aircraft {
            Aircraft::Global5000 | Aircraft::B7878 => EngineOutput::TurbineN1,
            Aircraft::C172p => EngineOutput::PistonRpm { rated_rpm: 2700.0 },
        },
        settings: aircraft.settings(),
        ic: InitialConditions {
            latitude_deg: t.latitude_deg,
            longitude_deg: t.longitude_deg,
            terrain_elevation_msl_m: t.surface_msl_m,
            height_agl_m: aircraft.parked_agl_m(),
            heading_deg: t.heading_deg,
            airspeed_kts: 0.0,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::s1::synthetic_world_frame;

    #[test]
    fn threshold_is_on_the_synthetic_runway_centreline_facing_east() {
        let frame = synthetic_world_frame().unwrap();
        let t = threshold(&frame).unwrap();
        assert_eq!(t.runway, "10L/28R");
        assert_eq!(t.enu_m, [-1_500.0 + THRESHOLD_INSET_M, 0.0]);
        assert_eq!(t.heading_deg, 90.0);
        // Origin at field elevation 4.0 m MSL; the runway surface is 0.2 m up.
        assert!((t.surface_msl_m - 4.2).abs() < 1e-6, "{}", t.surface_msl_m);
        // Converting back lands on the threshold to < 1 mm.
        let enu = frame.enu_m(GeodeticPosition {
            datum: GeodeticDatum::Wgs84Ellipsoid,
            latitude_deg: t.latitude_deg,
            longitude_deg: t.longitude_deg,
            ellipsoid_height_m: t.surface_msl_m + frame.msl_to_ellipsoid_m(),
        });
        assert!((enu[0] - t.enu_m[0]).abs() < 1e-3 && (enu[1] - t.enu_m[1]).abs() < 1e-3, "{enu:?}");
        assert!((enu[2] - 0.2).abs() < 0.5, "{enu:?}");
        // SFO, roughly 1.5 km west of the reference point.
        assert!((t.latitude_deg - 37.6188).abs() < 0.001 && (t.longitude_deg + 122.392).abs() < 0.002, "{t:?}");
    }

    #[test]
    fn approaches_name_both_ends_of_the_synthetic_runway() {
        let frame = synthetic_world_frame().unwrap();
        let a = approaches(&frame);
        assert_eq!(a.len(), 2, "{a:?}");
        // x -1500..1500 on y = 0: 10L lands east from the west end, 28R west.
        let (e, w) = (&a[0], &a[1]);
        assert_eq!((e.name(), w.name()), ("10L", "28R"));
        assert_eq!((e.threshold_m, w.threshold_m), ([-1_500.0, 0.0], [1_500.0, 0.0]));
        assert!((e.course_rad.to_degrees() - 90.0).abs() < 1e-9 && (w.course_rad.to_degrees() - 270.0).abs() < 1e-9);
        // The threshold of the S2 start is 30 m inside the 10L end, on its course.
        let t = threshold(&frame).unwrap();
        let (along, cross) = e.track(t.enu_m);
        assert!((along - THRESHOLD_INSET_M).abs() < 1e-9 && cross.abs() < 1e-9, "{along} {cross}");
    }

    #[test]
    fn approaches_on_an_oriented_runway_and_end_names() {
        // SFO 10L/28R: 3,600 m on true heading 117.8 (magnetic ~104).
        let (h, len, w) = (117.8f64.to_radians(), 3_600.0, 61.0);
        let (u, n) = ([h.sin(), h.cos()], [-h.cos(), h.sin()]);
        let corner = |a: f64, b: f64| {
            [(a * u[0] * len / 2.0 + b * n[0] * w / 2.0) as f32, (a * u[1] * len / 2.0 + b * n[1] * w / 2.0) as f32, 0.2]
        };
        let corners = [corner(-1.0, -1.0), corner(1.0, -1.0), corner(1.0, 1.0), corner(-1.0, 1.0)];
        let [west, east] = runway_ends(&corners).unwrap();
        assert!((west[0] + 1_800.0 * u[0]).abs() < 0.01 && (east[1] - 1_800.0 * u[1]).abs() < 0.01, "{west:?} {east:?}");
        assert_eq!(end_name("10L/28R", 117.8), "10L");
        assert_eq!(end_name("10L/28R", 297.8), "28R");
        assert_eq!(end_name("01R/19L", 207.8), "19L");
        assert_eq!(end_name("runway", 297.8), "30");
        assert_eq!(end_name("10L/28R", 3.0), "36", "no part within 30 degrees");
        assert!(runway_ends(&corners[..3]).is_none());
    }

    /// `GX_WORLD_PACK=...sfo-real.gxworld cargo test -p gx-app approaches_of_a_pack -- --ignored --nocapture`
    #[test]
    #[ignore = "needs GX_WORLD_PACK"]
    fn approaches_of_a_pack() {
        let path = std::env::var("GX_WORLD_PACK").expect("GX_WORLD_PACK");
        let frame = crate::spike002::world_frame_from_pack(&path).unwrap();
        for a in approaches(&frame) {
            eprintln!("{} at {:?} course {:.3}", a.name(), a.threshold_m, a.course_rad.to_degrees());
        }
    }

    #[test]
    fn presets() {
        assert_eq!(Aircraft::parse("global5000"), Some(Aircraft::Global5000));
        assert_eq!(Aircraft::parse("c172p"), Some(Aircraft::C172p));
        assert_eq!(Aircraft::parse("787-8"), Some(Aircraft::B7878));
        assert_eq!(Aircraft::parse("b747"), None);
        assert!(Aircraft::Global5000.settings().iter().any(|(k, v)| k == "fcs/flap-cmd-norm" && *v == 0.5));
        assert_eq!(Aircraft::C172p.engines(), 1);
        assert_eq!(Aircraft::B7878.engines(), 2);
    }

    #[test]
    fn flap_detents_rise_to_full_and_the_initial_levers_match_the_settings() {
        for a in [Aircraft::Global5000, Aircraft::B7878, Aircraft::C172p] {
            let d = a.flap_detents();
            assert_eq!(d[0].norm, 0.0, "{a:?} starts at flaps up");
            assert!(d.windows(2).all(|w| w[1].norm > w[0].norm && w[1].label > w[0].label), "{a:?}");
            assert!(d.iter().all(|x| x.norm <= 1.0));
            let levers = a.initial_levers();
            assert!(levers.gear_down && levers.speedbrake == 0.0);
            let flap = a.settings().into_iter().find(|(k, _)| k == "fcs/flap-cmd-norm").unwrap().1;
            assert_eq!(flap, f64::from(levers.flaps), "{a:?}: settings and levers agree");
        }
        assert_eq!(Aircraft::Global5000.initial_levers().flaps, 0.5, "unchanged M1-001 take-off flap");
        assert_eq!(Aircraft::B7878.flap_detents()[Aircraft::B7878.takeoff_flap_index()].label, 5);
    }
}
