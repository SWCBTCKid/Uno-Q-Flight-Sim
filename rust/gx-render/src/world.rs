//! S4 world geometry: the GXWORLD 2.0 pack turned into GPU-ready vertex
//! lists once, at start, on the render thread. Pure and host-testable.
//!
//! * **Ground** (land, Bay water, taxiways, runways, surface markings) is one
//!   static triangle list in painter's order. All of it lies within 0.3 m of
//!   z = 0, so it is drawn with the depth test off (no z-fighting between
//!   the coplanar layers at any distance); only the aircraft is depth-tested.
//! * Beyond the pack's +/-3 km a renderer-made **surround** continues the
//!   pack's land (west of the pack's land/water boundary) and water (east)
//!   out to 150 km, so the horizon is ground, not sky. It is drawn first and
//!   the pack over it. It carries no data claim.
//! * **Earth curvature.** The pack is flat local ENU, while JSBSim's terrain
//!   is the ellipsoid-parallel surface at the runway elevation. Every vertex
//!   is lowered by d^2 / 2R (d = horizontal distance from the pack origin, R
//!   the local Gaussian radius): 0.17 m at the S2 threshold 1.47 km from the
//!   origin, 0.71 m at 3 km, 1.8 km at 150 km. With it the drawn runway is
//!   the surface the simulated wheels roll on, to millimetres. Ground
//!   triangles are subdivided to edges <= 250 m so the planar facets follow
//!   the curve to ~1 mm over the pack.
//! * **Lights** are a static point list (pack colour), drawn as round point
//!   sprites sized by distance.

use crate::aircraft::Vertex3;
use crate::math::{cross, dot, sub, V3};
use crate::scene::Rgb;
use gx_world::{Layer, WorldAsset};

pub const LAND: Rgb = [0.34, 0.44, 0.24];
pub const WATER: Rgb = [0.13, 0.29, 0.44];
pub const TAXIWAY: Rgb = [0.33, 0.33, 0.34];
pub const RUNWAY: Rgb = [0.19, 0.19, 0.21];
pub const MARKING: Rgb = [0.92, 0.92, 0.90];
/// Frame-start clear colour (the sky).
pub const SKY_CLEAR: Rgb = [0.52, 0.69, 0.88];

/// What the S4 world pass needs per frame (matrices column-major f32).
#[derive(Clone, Copy, Debug)]
pub struct WorldDraw {
    pub clear: Rgb,
    pub ground_mvp: [f32; 16],
    /// `None` in the cockpit view (the camera is inside the aircraft).
    pub aircraft_mvp: Option<[f32; 16]>,
    /// Point size numerator: size = k / clip w, clamped 2..7 px.
    pub point_k: f32,
}

/// The static S4 geometry handed to `linux::KmsDisplay::load_world` /
/// `linux::Offscreen::load_world` once.
pub struct WorldBuffers<'a> {
    pub ground: &'a [Vertex3],
    pub lights: &'a [Vertex3],
    pub aircraft: &'a [Vertex3],
}

/// Longest ground triangle edge after subdivision.
pub const MAX_EDGE_M: f64 = 250.0;
/// Surround extent (half-width) beyond which nothing is drawn.
pub const SURROUND_M: f64 = 150_000.0;

/// Local Gaussian radius of curvature sqrt(M N) of WGS84 at `lat_deg`.
pub fn gaussian_radius_m(lat_deg: f64) -> f64 {
    let a = 6_378_137.0f64;
    let f = 1.0 / 298.257_223_563;
    let e2 = f * (2.0 - f);
    let s = lat_deg.to_radians().sin();
    let w = 1.0 - e2 * s * s;
    let m = a * (1.0 - e2) / w.powf(1.5);
    let n = a / w.sqrt();
    (m * n).sqrt()
}

/// Height of the ellipsoid-parallel surface below the ENU tangent plane at
/// horizontal offset (x, y) from the origin.
pub fn curvature_drop_m(x: f64, y: f64, radius_m: f64) -> f64 {
    (x * x + y * y) / (2.0 * radius_m)
}

/// Draw order rank of a ground layer (lower first).
fn rank(layer: &Layer) -> Option<(u8, Rgb, &'static str)> {
    match layer {
        Layer::Land(_) => Some((0, LAND, "land")),
        Layer::Water(_) => Some((1, WATER, "water")),
        Layer::Taxiway(_) => Some((2, TAXIWAY, "taxiway")),
        Layer::Runway(_) => Some((3, RUNWAY, "runway")),
        Layer::Markings(_) => Some((4, MARKING, "markings")),
        Layer::Lights(_) => None,
    }
}

/// One source (pack) ground triangle, kept for surface queries.
#[derive(Clone, Copy, Debug)]
struct SourceTri {
    rank: u8,
    kind: &'static str,
    p: [V3; 3],
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct WorldStats {
    pub pack_triangles: u32,
    pub ground_triangles: u32,
    pub surround_triangles: u32,
    pub lights: u32,
    pub markings_triangles: u32,
    pub radius_m: f64,
}

pub struct WorldMesh {
    /// Painter's order: surround, land, water, taxiways, runways, markings.
    pub ground: Vec<Vertex3>,
    pub lights: Vec<Vertex3>,
    pub stats: WorldStats,
    pub radius_m: f64,
    source: Vec<SourceTri>,
}

/// Rendered surface under (x, y): what the pack draws there (top-most layer)
/// and its height including the curvature drop.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Surface {
    pub kind: &'static str,
    pub z_m: f64,
}

impl WorldMesh {
    /// Level-0 LOD of every chunk. `world` is a validated pack.
    pub fn build(world: &WorldAsset) -> Result<Self, String> {
        let radius_m = gaussian_radius_m(world.origin.latitude_deg);
        let mut source = Vec::new();
        let mut lights = Vec::new();
        let mut land_east = f64::NEG_INFINITY;
        let mut water_west = f64::INFINITY;
        for layer in world.chunks.iter().flat_map(|c| c.lods.iter().filter(|l| l.level == 0)).flat_map(|l| l.layers.iter()) {
            if let Layer::Lights(ll) = layer {
                for l in &ll.lights {
                    let p = l.position.map(f64::from);
                    let z = p[2] - curvature_drop_m(p[0], p[1], radius_m);
                    let color = l.color_rgb8.map(|c| f32::from(c) / 255.0);
                    lights.push(Vertex3 { pos: [p[0] as f32, p[1] as f32, z as f32], color });
                }
                continue;
            }
            let (r, _, kind) = rank(layer).expect("non-light layer");
            let mesh = match layer {
                Layer::Land(m) | Layer::Water(m) | Layer::Markings(m) => m,
                Layer::Runway(n) | Layer::Taxiway(n) => &n.mesh,
                Layer::Lights(_) => unreachable!(),
            };
            for t in mesh.indices.chunks_exact(3) {
                let p = [0, 1, 2].map(|k| mesh.positions[t[k] as usize].map(f64::from));
                for q in &p {
                    if kind == "land" {
                        land_east = land_east.max(q[0]);
                    }
                    if kind == "water" {
                        water_west = water_west.min(q[0]);
                    }
                }
                source.push(SourceTri { rank: r, kind, p });
            }
        }
        if source.is_empty() {
            return Err("world pack has no ground geometry".into());
        }
        source.sort_by_key(|t| t.rank);
        // Land/water boundary for the surround: the pack's own split if it
        // has both, else all land.
        let split_x = if land_east.is_finite() && water_west.is_finite() { 0.5 * (land_east + water_west) } else { f64::INFINITY };

        let mut ground = Vec::with_capacity(200_000);
        let mut stats = WorldStats { radius_m, lights: lights.len() as u32, pack_triangles: source.len() as u32, ..Default::default() };
        surround(&mut ground, split_x, radius_m);
        stats.surround_triangles = (ground.len() / 3) as u32;
        let colors = [LAND, WATER, TAXIWAY, RUNWAY, MARKING];
        for t in &source {
            let before = ground.len();
            subdivide(&mut ground, t.p, colors[t.rank as usize], radius_m, 0);
            if t.kind == "markings" {
                stats.markings_triangles += ((ground.len() - before) / 3) as u32;
            }
        }
        stats.ground_triangles = (ground.len() / 3) as u32;
        Ok(Self { ground, lights, stats, radius_m, source })
    }

    /// Top-most pack surface at (x, y), z including the curvature drop.
    pub fn surface_at(&self, x: f64, y: f64) -> Option<Surface> {
        let mut out = None;
        for t in &self.source {
            if let Some(z) = planar_z(t.p, x, y) {
                out = Some(Surface { kind: t.kind, z_m: z - curvature_drop_m(x, y, self.radius_m) });
            }
        }
        out
    }

    /// CPU reference for the offline render test: colour of the last-drawn
    /// ground triangle hit by the ray (painter's order, as the GPU draws
    /// the ground without a depth test), or `None` (sky).
    pub fn ground_ray(&self, origin: V3, dir: V3) -> Option<Rgb> {
        let mut out = None;
        for t in self.ground.chunks_exact(3) {
            let p = [t[0].pos, t[1].pos, t[2].pos].map(|v| v.map(f64::from));
            if ray_triangle(origin, dir, p).is_some() {
                out = Some(t[0].color);
            }
        }
        out
    }
}

/// The S4 on-runway gate: the plan's "aircraft sits on the runway within
/// 0.5 m". Each drawn wheel bottom (= the JSBSim gear contact point, see
/// `aircraft::gear_contacts`) placed by the simulation pose, against the
/// rendered pack surface under it (pack height minus the curvature drop).
/// A wrong geoid separation shows here as a ~32 m (ignored) or ~64 m (wrong
/// sign) gap.
pub const ON_RUNWAY_LIMIT_M: f64 = 0.5;

#[derive(Clone, Debug, PartialEq)]
pub struct WheelGap {
    pub gear: &'static str,
    pub wheel_enu: V3,
    pub surface: Option<Surface>,
    /// Wheel bottom minus surface (negative: below the drawn surface).
    pub gap_m: Option<f64>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct GroundCheck {
    pub wheels: Vec<WheelGap>,
    pub max_abs_gap_m: Option<f64>,
    /// Every wheel over a runway (or its markings) and within the limit.
    pub on_runway: bool,
}

pub fn ground_check(mesh: &WorldMesh, pose: &crate::camera::Pose) -> GroundCheck {
    let wheels: Vec<WheelGap> = crate::aircraft::active().contacts
        .iter()
        .map(|&(gear, body)| {
            let w = pose.to_world(body);
            let surface = mesh.surface_at(w[0], w[1]);
            WheelGap { gear, wheel_enu: w, surface, gap_m: surface.map(|s| w[2] - s.z_m) }
        })
        .collect();
    let gaps: Vec<f64> = wheels.iter().filter_map(|w| w.gap_m).collect();
    let max_abs = (gaps.len() == wheels.len()).then(|| gaps.iter().fold(0.0f64, |m, g| m.max(g.abs())));
    let on_runway = max_abs.is_some_and(|m| m <= ON_RUNWAY_LIMIT_M)
        && wheels.iter().all(|w| w.surface.is_some_and(|s| s.kind == "runway" || s.kind == "markings"));
    GroundCheck { wheels, max_abs_gap_m: max_abs, on_runway }
}

/// Möller-Trumbore: distance along `dir` (unit or not) to the hit, if any.
pub fn ray_triangle(o: V3, d: V3, p: [V3; 3]) -> Option<f64> {
    let e1 = sub(p[1], p[0]);
    let e2 = sub(p[2], p[0]);
    let h = cross(d, e2);
    let a = dot(e1, h);
    if a.abs() < 1e-12 {
        return None;
    }
    let f = 1.0 / a;
    let s = sub(o, p[0]);
    let u = f * dot(s, h);
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = cross(s, e1);
    let v = f * dot(d, q);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = f * dot(e2, q);
    (t > 1e-6).then_some(t)
}

/// z of the triangle's plane at (x, y) if (x, y) is inside its xy projection.
fn planar_z(p: [V3; 3], x: f64, y: f64) -> Option<f64> {
    let (a, b, c) = (p[0], p[1], p[2]);
    let det = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1]);
    if det.abs() < 1e-12 {
        return None;
    }
    let l1 = ((b[1] - c[1]) * (x - c[0]) + (c[0] - b[0]) * (y - c[1])) / det;
    let l2 = ((c[1] - a[1]) * (x - c[0]) + (a[0] - c[0]) * (y - c[1])) / det;
    let l3 = 1.0 - l1 - l2;
    let eps = -1e-9;
    (l1 >= eps && l2 >= eps && l3 >= eps).then(|| l1 * a[2] + l2 * b[2] + l3 * c[2])
}

fn push_tri(out: &mut Vec<Vertex3>, p: [V3; 3], color: Rgb, radius_m: f64) {
    for q in p {
        let z = q[2] - curvature_drop_m(q[0], q[1], radius_m);
        out.push(Vertex3 { pos: [q[0] as f32, q[1] as f32, z as f32], color });
    }
}

/// Midpoint subdivision until every edge is <= MAX_EDGE_M (planar in the
/// pack; the drop is applied per vertex).
fn subdivide(out: &mut Vec<Vertex3>, p: [V3; 3], color: Rgb, radius_m: f64, depth: u32) {
    let len = |a: V3, b: V3| ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt();
    let longest = len(p[0], p[1]).max(len(p[1], p[2])).max(len(p[2], p[0]));
    if longest <= MAX_EDGE_M || depth >= 12 {
        push_tri(out, p, color, radius_m);
        return;
    }
    let mid = |a: V3, b: V3| [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, (a[2] + b[2]) / 2.0];
    let (ab, bc, ca) = (mid(p[0], p[1]), mid(p[1], p[2]), mid(p[2], p[0]));
    for t in [[p[0], ab, ca], [ab, p[1], bc], [ca, bc, p[2]], [ab, bc, ca]] {
        subdivide(out, t, color, radius_m, depth + 1);
    }
}

/// Grid lines of the surround: dense near the pack, geometric outwards.
fn surround_lines(split_x: f64) -> Vec<f64> {
    let mut v = vec![0.0];
    let mut d = 1_000.0;
    while d < SURROUND_M {
        v.push(d);
        v.push(-d);
        d *= 1.45;
    }
    v.push(SURROUND_M);
    v.push(-SURROUND_M);
    if split_x.is_finite() && split_x.abs() < SURROUND_M {
        v.push(split_x);
    }
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    v.dedup_by(|a, b| (*a - *b).abs() < 1.0);
    v
}

fn surround(out: &mut Vec<Vertex3>, split_x: f64, radius_m: f64) {
    let xs = surround_lines(split_x);
    let ys = surround_lines(f64::INFINITY);
    for wx in xs.windows(2) {
        for wy in ys.windows(2) {
            let water = wx[0] >= split_x - 0.5;
            let (color, z) = if water { (WATER, -1.0) } else { (LAND, 0.0) };
            let (x0, x1, y0, y1) = (wx[0], wx[1], wy[0], wy[1]);
            let a = [x0, y0, z];
            let b = [x1, y0, z];
            let c = [x1, y1, z];
            let d = [x0, y1, z];
            push_tri(out, [a, b, c], color, radius_m);
            push_tri(out, [a, c, d], color, radius_m);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gx_world::synthetic_sfo_like;

    #[test]
    fn curvature_matches_the_s3_run_measurement() {
        // S3 run: ENU z - runway z - JSBSim AGL = -0.168 m at the threshold.
        let r = gaussian_radius_m(37.6188);
        assert!((r - 6_372_000.0).abs() < 8_000.0, "{r}");
        let d = curvature_drop_m(-1_469.98, 0.0, r);
        assert!((d - 0.1696).abs() < 0.002, "{d}");
    }

    #[test]
    fn pack_builds_in_painters_order_with_surround_and_lights() {
        let w = WorldMesh::build(&synthetic_sfo_like()).unwrap();
        let s = &w.stats;
        assert!(s.surround_triangles > 100 && s.ground_triangles > s.surround_triangles, "{s:?}");
        assert!(s.lights >= 100, "edge + centreline lights: {s:?}");
        assert!(s.markings_triangles >= 50, "{s:?}");
        assert!(w.ground.len() < 600_000, "{} ground vertices", w.ground.len());
        // Painter's order: once the runway colour appears, no land/water follows.
        let first_runway = w.ground.iter().position(|v| v.color == RUNWAY).unwrap();
        assert!(w.ground[first_runway..].iter().all(|v| v.color != LAND && v.color != WATER));
        let last_mark = w.ground.iter().rposition(|v| v.color == MARKING).unwrap();
        assert_eq!(last_mark, w.ground.len() - 1, "markings are drawn last");
    }

    #[test]
    fn surface_query_finds_runway_and_water_with_the_drop() {
        let w = WorldMesh::build(&synthetic_sfo_like()).unwrap();
        let s = w.surface_at(-1_470.0, 0.0).unwrap();
        assert!(s.kind == "runway" || s.kind == "markings", "{s:?}");
        assert!((s.z_m - (0.2 - 0.1696)).abs() < 0.06, "{s:?}");
        let east = w.surface_at(2_500.0, 1_000.0).unwrap();
        assert_eq!(east.kind, "water");
        let t = w.surface_at(0.0, 120.0).unwrap();
        assert_eq!(t.kind, "taxiway");
        assert!(w.surface_at(10_000.0, 0.0).is_none(), "outside the pack");
    }

    #[test]
    fn s3_threshold_pose_sits_on_the_runway_and_a_geoid_error_fails() {
        let w = WorldMesh::build(&synthetic_sfo_like()).unwrap();
        // Settled S3 pose at the threshold: CG ENU z 2.7234 m, level, east.
        let pose = |z: f64| crate::camera::Pose { position_m: [-1_469.9755, 0.0, z], orientation_xyzw: [1.0, 0.0, 0.0, 0.0] };
        let g = ground_check(&w, &pose(2.7234));
        assert!(g.on_runway, "{g:?}");
        let gap = g.max_abs_gap_m.unwrap();
        assert!(gap < 0.2, "settled: gear compression 0.115 m + the 5 cm marking layer: {gap}");
        // Reset pose (h-agl 9.5 ft) is above, still within the limit.
        assert!(ground_check(&w, &pose(2.8775)).on_runway);
        // Ignoring the geoid separation (N = -32.3 m) puts it 32 m off.
        let bad = ground_check(&w, &pose(2.7234 + 32.2753));
        assert!(!bad.on_runway && bad.max_abs_gap_m.unwrap() > 30.0);
        // Off the runway (on the taxiway) is not "on the runway".
        let taxi = crate::camera::Pose { position_m: [0.0, 120.0, 2.7234 - 0.05], orientation_xyzw: [1.0, 0.0, 0.0, 0.0] };
        assert!(!ground_check(&w, &taxi).on_runway);
    }

    #[test]
    fn ray_hits_the_last_drawn_ground_layer() {
        let w = WorldMesh::build(&synthetic_sfo_like()).unwrap();
        // Straight down onto the runway edge region away from markings.
        assert_eq!(w.ground_ray([-500.0, 25.0, 100.0], [0.0, 0.0, -1.0]), Some(RUNWAY));
        assert_eq!(w.ground_ray([-500.0, 300.0, 100.0], [0.0, 0.0, -1.0]), Some(LAND));
        assert_eq!(w.ground_ray([2_000.0, 600.0, 100.0], [0.0, 0.0, -1.0]), Some(WATER));
        assert_eq!(w.ground_ray([-500.0, 300.0, 100.0], [0.0, 0.0, 1.0]), None, "sky");
        // Far surround beyond the pack.
        assert_eq!(w.ground_ray([-20_000.0, 0.0, 100.0], [0.0, 0.0, -1.0]), Some(LAND));
        assert_eq!(w.ground_ray([20_000.0, 0.0, 100.0], [0.0, 0.0, -1.0]), Some(WATER));
    }
}
