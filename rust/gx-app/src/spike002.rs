//! SPIKE-002: the surveyed-SFO GXWORLD 2.0 pack.
//!
//! The offline converter (`tools/spike002_convert.py`) reduces OSM/FAA
//! runway data to a small, line-based world source (`gx-spike002-world/1`):
//! oriented runway rectangles, runway-marking quads and lights, all in the
//! pack's local ENU metres. `gx-app spike002-world SRC OUT` (host, offline)
//! turns it into a validated GXWORLD 2.0 pack with `gx_world::write_world`;
//! the board only ever reads the pack (`--world PATH`), never GIS data.
//!
//! Source lines (whitespace separated, `#` comments):
//!
//! ```text
//! gx-spike002-world/1
//! frame <frame_id hex> <generation>
//! origin <lat deg> <lon deg> <field elevation MSL m>
//! bounds <minx> <miny> <minz> <maxx> <maxy> <maxz>
//! runway <name> <x y z> x4            (corners in order around the strip)
//! marking <x y z> x4                  (one quad; all quads form one layer)
//! light <x y z> <r g b> <kind 1..5> <cd>
//! ```

use crate::{validate_world_pack, WorldFrame};
use gx_world::{
    write_world, Bounds3, Chunk, GeodeticPoint, GeoidModel, Georeference, Layer, Light, LightLayer, LightType, Limits,
    Lod, Mesh, NamedMesh, WorldAsset, SFO_GEOID_SEPARATION_EGM2008_M,
};
use std::sync::Arc;

pub const SOURCE_SCHEMA: &str = "gx-spike002-world/1";

fn nums<const N: usize>(it: &mut std::str::SplitWhitespace<'_>, what: &str) -> Result<[f64; N], String> {
    let mut out = [0.0; N];
    for v in &mut out {
        let t = it.next().ok_or_else(|| format!("{what}: too few values"))?;
        *v = t.parse::<f64>().map_err(|_| format!("{what}: bad number {t}"))?;
        if !v.is_finite() {
            return Err(format!("{what}: non-finite"));
        }
    }
    Ok(out)
}

fn quad(m: &mut Mesh, v: [f64; 12]) {
    let base = m.positions.len() as u32;
    for k in 0..4 {
        m.positions.push([v[3 * k] as f32, v[3 * k + 1] as f32, v[3 * k + 2] as f32]);
    }
    m.indices.extend([base, base + 1, base + 2, base, base + 2, base + 3]);
}

/// Parse a world source into a (not yet validated) pack.
pub fn world_from_source(text: &str) -> Result<WorldAsset, String> {
    let mut lines = text.lines().map(str::trim).filter(|l| !l.is_empty() && !l.starts_with('#'));
    if lines.next() != Some(SOURCE_SCHEMA) {
        return Err(format!("not a {SOURCE_SCHEMA} source"));
    }
    let (mut frame, mut origin, mut bounds) = (None, None, None);
    let mut layers = Vec::new();
    let mut markings = Mesh { positions: Vec::new(), indices: Vec::new() };
    let mut lights = Vec::new();
    for (n, line) in lines.enumerate() {
        let mut it = line.split_whitespace();
        let key = it.next().unwrap_or_default();
        let what = format!("line {}: {key}", n + 2);
        match key {
            "frame" => {
                let id = it.next().ok_or(format!("{what}: id"))?;
                let id = u64::from_str_radix(id.trim_start_matches("0x"), 16).map_err(|_| format!("{what}: id"))?;
                let generation: u64 = it.next().and_then(|g| g.parse().ok()).ok_or(format!("{what}: generation"))?;
                frame = Some((id, generation));
            }
            "origin" => origin = Some(nums::<3>(&mut it, &what)?),
            "bounds" => {
                let b = nums::<6>(&mut it, &what)?;
                bounds = Some(Bounds3 { min: [b[0] as f32, b[1] as f32, b[2] as f32], max: [b[3] as f32, b[4] as f32, b[5] as f32] });
            }
            "runway" => {
                let name = it.next().ok_or(format!("{what}: name"))?.to_string();
                let mut mesh = Mesh { positions: Vec::new(), indices: Vec::new() };
                quad(&mut mesh, nums::<12>(&mut it, &what)?);
                layers.push(Layer::Runway(NamedMesh { name, mesh }));
            }
            "marking" => quad(&mut markings, nums::<12>(&mut it, &what)?),
            "light" => {
                let v = nums::<8>(&mut it, &what)?;
                let kind = match v[6] as u8 {
                    1 => LightType::RunwayEdge,
                    2 => LightType::RunwayCenterline,
                    3 => LightType::TaxiwayEdge,
                    4 => LightType::TaxiwayCenterline,
                    5 => LightType::Approach,
                    _ => return Err(format!("{what}: light kind")),
                };
                lights.push(Light {
                    position: [v[0] as f32, v[1] as f32, v[2] as f32],
                    color_rgb8: [v[3] as u8, v[4] as u8, v[5] as u8],
                    kind,
                    intensity_cd: v[7] as f32,
                });
            }
            _ => return Err(format!("{what}: unknown record")),
        }
        if it.next().is_some() {
            return Err(format!("{what}: trailing values"));
        }
    }
    let (frame_id, frame_generation) = frame.ok_or("no frame record")?;
    let o = origin.ok_or("no origin record")?;
    let bounds = bounds.ok_or("no bounds record")?;
    if !layers.iter().any(|l| matches!(l, Layer::Runway(_))) {
        return Err("no runway".into());
    }
    if !markings.indices.is_empty() {
        layers.push(Layer::Markings(markings));
    }
    if !lights.is_empty() {
        layers.push(Layer::Lights(LightLayer { lights }));
    }
    Ok(WorldAsset {
        origin: GeodeticPoint {
            latitude_deg: o[0],
            longitude_deg: o[1],
            height_m: o[2] + SFO_GEOID_SEPARATION_EGM2008_M,
        },
        georeference: Georeference {
            frame_id,
            frame_generation,
            geoid_model: GeoidModel::Egm2008,
            geoid_separation_m: SFO_GEOID_SEPARATION_EGM2008_M,
        },
        chunks: vec![Chunk {
            id: 1,
            bounds,
            lods: vec![Lod { level: 0, min_distance_m: 0.0, max_distance_m: 30_000.0, layers }],
        }],
    })
}

/// Read and validate a GXWORLD pack file (the board side).
pub fn world_frame_from_pack(path: &str) -> Result<WorldFrame, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("world pack {path}: {e}"))?;
    let world = validate_world_pack(&bytes).map_err(|e| format!("world pack {path}: {e}"))?;
    Ok(WorldFrame::new(Arc::new(world)))
}

/// `gx-app spike002-world SRC OUT` (host tool).
pub fn tool_main(args: &[String]) -> i32 {
    let [src, out] = args else {
        eprintln!("usage: gx-app spike002-world SOURCE.txt OUT.gxworld");
        return 2;
    };
    let run = || -> Result<String, String> {
        let text = std::fs::read_to_string(src).map_err(|e| format!("{src}: {e}"))?;
        let world = world_from_source(&text)?;
        let mut pack = Vec::new();
        write_world(&world, Limits::default(), &mut pack).map_err(|e| e.to_string())?;
        let back = validate_world_pack(&pack).map_err(|e| e.to_string())?;
        let frame = WorldFrame::new(Arc::new(back));
        let t = crate::s2::setup::threshold(&frame)?;
        std::fs::write(out, &pack).map_err(|e| format!("{out}: {e}"))?;
        Ok(format!(
            "{{\"k\":\"spike002_world\",\"out\":{},\"bytes\":{},\"frame_id\":{},\"threshold\":{{\"runway\":\"{}\",\"enu_m\":[{:.2},{:.2}],\"heading_deg\":{:.3},\"lat\":{:.7},\"lon\":{:.7}}}}}",
            crate::s1::rec::esc(out),
            pack.len(),
            frame.frame_id(),
            t.runway,
            t.enu_m[0],
            t.enu_m[1],
            t.heading_deg,
            t.latitude_deg,
            t.longitude_deg
        ))
    };
    match run() {
        Ok(line) => {
            println!("{line}");
            0
        }
        Err(e) => {
            eprintln!("gx-app spike002-world: {e}");
            1
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 3,600 m x 61 m strip on true heading 118 deg, like SFO 10L/28R.
    fn source() -> String {
        let (h, len, w) = (118f64.to_radians(), 3_600.0, 61.0);
        let u = [h.sin(), h.cos()];
        let n = [-u[1], u[0]];
        let c = [-100.0, 50.0];
        let corner = |a: f64, b: f64| {
            format!("{:.3} {:.3} 0.2", c[0] + a * u[0] * len / 2.0 + b * n[0] * w / 2.0, c[1] + a * u[1] * len / 2.0 + b * n[1] * w / 2.0)
        };
        format!(
            "{SOURCE_SCHEMA}\nframe 53464f5030320001 1\norigin 37.6188056 -122.3754167 4.0\nbounds -12000 -12000 -50 12000 12000 600\n\
             runway 10L/28R {} {} {} {}\nmarking 0 0 0.25 1 0 0.25 1 1 0.25 0 1 0.25\nlight 0 0 0.5 255 250 235 1 10000\n",
            corner(-1.0, -1.0),
            corner(1.0, -1.0),
            corner(1.0, 1.0),
            corner(-1.0, 1.0)
        )
    }

    #[test]
    fn oriented_runway_threshold_is_the_west_end_facing_along_the_strip() {
        let world = world_from_source(&source()).unwrap();
        let mut pack = Vec::new();
        write_world(&world, Limits::default(), &mut pack).unwrap();
        let frame = WorldFrame::new(Arc::new(validate_world_pack(&pack).unwrap()));
        assert_eq!(frame.frame_id(), 0x5346_4f50_3032_0001);
        let t = crate::s2::setup::threshold(&frame).unwrap();
        assert_eq!(t.runway, "10L/28R");
        assert!((t.heading_deg - 118.0).abs() < 1e-6, "{t:?}");
        let h = 118f64.to_radians();
        let want = [-100.0 - (1_800.0 - 30.0) * h.sin(), 50.0 - (1_800.0 - 30.0) * h.cos()];
        assert!((t.enu_m[0] - want[0]).abs() < 0.01 && (t.enu_m[1] - want[1]).abs() < 0.01, "{t:?} {want:?}");
        assert!((t.surface_msl_m - 4.2).abs() < 1e-5);
    }

    #[test]
    fn malformed_sources_are_refused() {
        assert!(world_from_source("nope\n").is_err());
        let good = source();
        assert!(world_from_source(&good.replace("runway 10L/28R", "runway")).is_err());
        assert!(world_from_source(&good.replace("light 0 0 0.5", "light 0 0 0.5 9")).is_err());
        assert!(world_from_source(&good.replace("bounds", "# bounds")).is_err());
    }
}

/// SPIKE-002 render options for one run (set once before the threads start).
#[derive(Clone, Debug, Default)]
pub struct RenderOpts {
    /// Deterministic camera/pose trace (offscreen measurement runs).
    pub trace: Option<TraceOrigin>,
    /// Directory for one lossless PPM capture per trace segment.
    pub capture_dir: Option<String>,
}

/// Where the trace is anchored: the pack's threshold (ENU, true heading).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TraceOrigin {
    pub threshold_enu: [f64; 2],
    pub heading_deg: f64,
    pub runway_z: f64,
}

pub static RENDER_OPTS: std::sync::OnceLock<RenderOpts> = std::sync::OnceLock::new();

/// Trace segment length.
pub const SEGMENT_S: f64 = 30.0;
/// Unreported warm-up (chase on the threshold) before the first segment.
pub const WARMUP_S: f64 = 10.0;
/// Segment names; index 4 is the warm-up.
pub const SEGMENTS: [&str; 5] = ["threshold_chase", "cockpit", "climbout_water", "high_wide", "warmup"];

/// Body(FRD)-to-ENU quaternion `[x, y, z, w]` for true heading and pitch
/// (degrees), wings level.
pub fn attitude(heading_deg: f64, pitch_deg: f64) -> [f64; 4] {
    let (h, p) = (heading_deg.to_radians(), pitch_deg.to_radians());
    let x = [h.sin() * p.cos(), h.cos() * p.cos(), p.sin()];
    let y = [h.cos(), -h.sin(), 0.0];
    let z = [x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2], x[0] * y[1] - x[1] * y[0]];
    // Rows r, columns c: m[r][c] = column c's component r.
    let m = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]];
    let tr = m[0][0] + m[1][1] + m[2][2];
    let q = if tr > 0.0 {
        let s = (tr + 1.0).sqrt() * 2.0;
        [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s]
    } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
        let s = (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt() * 2.0;
        [0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s]
    } else if m[1][1] > m[2][2] {
        let s = (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt() * 2.0;
        [(m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s]
    } else {
        let s = (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt() * 2.0;
        [(m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s]
    };
    let n = q.iter().map(|v| v * v).sum::<f64>().sqrt();
    q.map(|v| v / n)
}

/// One trace step: segment index, whether the camera is the cockpit, and
/// the synthetic pose (`None`: use the simulated aircraft, i.e. on the
/// threshold) at `t` seconds after the first world frame (warm-up first).
pub fn trace_pose(o: &TraceOrigin, t: f64) -> (usize, bool, Option<([f64; 3], [f64; 4])>) {
    if t < WARMUP_S {
        return (4, false, None);
    }
    let t_s = t - WARMUP_S;
    let seg = ((t_s / SEGMENT_S) as usize) % 4;
    let u = t_s % SEGMENT_S;
    let h = o.heading_deg.to_radians();
    let dir = [h.sin(), h.cos()];
    match seg {
        0 => (0, false, None),
        1 => (1, true, None),
        2 => {
            // Rotation 1,800 m down the runway, 85 m/s, 7 deg climb path.
            let a = 1_800.0 + 85.0 * u;
            let z = o.runway_z + 2.8 + 85.0 * u * 7f64.to_radians().tan();
            let p = [o.threshold_enu[0] + a * dir[0], o.threshold_enu[1] + a * dir[1], z];
            (2, false, Some((p, attitude(o.heading_deg, 9.0))))
        }
        _ => {
            // 1,100 m up, 7 km from the airport reference point over the Bay,
            // orbiting slowly while looking past the airport.
            let bearing = (112.0 + 0.6 * u).to_radians();
            let p = [7_000.0 * bearing.sin(), 7_000.0 * bearing.cos(), 1_100.0];
            let heading = (bearing.to_degrees() + 180.0 + 18.0).rem_euclid(360.0);
            (3, false, Some((p, attitude(heading, 0.0))))
        }
    }
}

#[cfg(test)]
mod trace_tests {
    use super::*;

    #[test]
    fn attitude_maps_body_axes_like_the_s4_east_level_pose() {
        let q = attitude(90.0, 0.0);
        let r = gx_render::math::quat_to_m3(q);
        let nose = gx_render::math::m3_mul_v(&r, [1.0, 0.0, 0.0]);
        let right = gx_render::math::m3_mul_v(&r, [0.0, 1.0, 0.0]);
        let down = gx_render::math::m3_mul_v(&r, [0.0, 0.0, 1.0]);
        for (v, w) in [(nose, [1.0, 0.0, 0.0]), (right, [0.0, -1.0, 0.0]), (down, [0.0, 0.0, -1.0])] {
            assert!((0..3).all(|i| (v[i] - w[i]).abs() < 1e-12), "{v:?} {w:?}");
        }
        let r = gx_render::math::quat_to_m3(attitude(118.0, 9.0));
        let nose = gx_render::math::m3_mul_v(&r, [1.0, 0.0, 0.0]);
        assert!((nose[2] - 9f64.to_radians().sin()).abs() < 1e-12);
        assert!((nose[0].atan2(nose[1]).to_degrees() - 118.0).abs() < 1e-9);
    }

    #[test]
    fn trace_cycles_through_the_four_segments() {
        let o = TraceOrigin { threshold_enu: [-1_557.0, 1_087.0], heading_deg: 117.8, runway_z: 0.2 };
        assert_eq!(trace_pose(&o, 1.0).0, 4);
        assert_eq!(trace_pose(&o, 11.0).0, 0);
        assert!(trace_pose(&o, 41.0).1);
        let (s, cockpit, p) = trace_pose(&o, 85.0);
        assert!(s == 2 && !cockpit && p.unwrap().0[2] > 100.0);
        assert_eq!(trace_pose(&o, 110.0).0, 3);
        assert_eq!(trace_pose(&o, 131.0).0, 0);
    }
}
