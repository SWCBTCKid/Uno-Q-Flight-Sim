use crate::{
    Bounds3, Chunk, GeodeticPoint, GeoidModel, Georeference, Layer, Light, LightLayer, LightType, Lod, Mesh, NamedMesh,
    WorldAsset,
};

/// KSFO field elevation, 13 ft MSL, in metres (FAA airport data, rounded).
pub const SFO_FIELD_ELEVATION_MSL_M: f64 = 4.0;

/// EGM2008 geoid undulation at the KSFO reference point (37.6189 N,
/// 122.3754 W), metres, `ellipsoid = MSL + N`.
///
/// Source: GeographicLib GeoidEval (online, 2026-10-17), which evaluates the
/// NGA EGM2008 grid: EGM2008 -32.2753 m (EGM96 -32.2140 m, EGM84
/// -32.8659 m). Over a +/-3 km box around the origin (this fixture's chunk
/// extent) EGM2008 ranges -32.2721 to -32.2910 m, so a single origin value
/// is within 0.016 m everywhere in the pack (EGM96: within 0.085 m). For the
/// production SFO pack the offline tool must evaluate EGM2008 from NGA's
/// published grid at the surveyed origin and record it in the pack.
pub const SFO_GEOID_SEPARATION_EGM2008_M: f64 = -32.2753;

/// Small, entirely synthetic airport-like scene for host tests and bring-up.
/// It is not surveyed airport data and must never be shipped as navigation data.
pub fn synthetic_sfo_like() -> WorldAsset {
    let bounds = Bounds3 {
        min: [-3_000.0, -3_000.0, -20.0],
        max: [3_000.0, 3_000.0, 100.0],
    };
    WorldAsset {
        // KSFO reference point, retained only as coordinate metadata. The
        // origin sits at field elevation; its height is ellipsoidal.
        origin: GeodeticPoint {
            latitude_deg: 37.618_805_6,
            longitude_deg: -122.375_416_7,
            height_m: SFO_FIELD_ELEVATION_MSL_M + SFO_GEOID_SEPARATION_EGM2008_M,
        },
        georeference: Georeference {
            // ASCII "SFOSYN" + 0x0001: a synthetic frame, never a surveyed id.
            frame_id: 0x5346_4f53_594e_0001,
            frame_generation: 1,
            geoid_model: GeoidModel::Egm2008,
            geoid_separation_m: SFO_GEOID_SEPARATION_EGM2008_M,
        },
        chunks: vec![Chunk {
            id: 1,
            bounds,
            lods: vec![Lod {
                level: 0,
                min_distance_m: 0.0,
                max_distance_m: 20_000.0,
                layers: vec![
                    Layer::Land(rect(-3_000.0, -3_000.0, 1_000.0, 3_000.0, 0.0)),
                    Layer::Water(rect(1_000.0, -3_000.0, 3_000.0, 3_000.0, -1.0)),
                    Layer::Runway(NamedMesh {
                        name: "10L/28R".into(),
                        mesh: rect(-1_500.0, -40.0, 1_500.0, 40.0, 0.2),
                    }),
                    Layer::Taxiway(NamedMesh {
                        name: "A".into(),
                        mesh: rect(-1_500.0, 100.0, 1_500.0, 140.0, 0.15),
                    }),
                    Layer::Markings(runway_markings()),
                    Layer::Lights(LightLayer { lights: airport_lights() }),
                ],
            }],
        }],
    }
}

/// Synthetic surface markings on runway 10L/28R (x -1500..1500, y -40..40,
/// surface z 0.2): threshold bars and aiming points at both ends, a dashed
/// centreline and side stripes, all 5 cm above the runway. FAA-like
/// proportions, not surveyed.
fn runway_markings() -> Mesh {
    let mut m = Mesh { positions: Vec::new(), indices: Vec::new() };
    let z = 0.25;
    for end in [-1.0f32, 1.0] {
        // Threshold bars: 6 each side of the centreline, 30 m x 1.8 m.
        let (x0, x1) = sorted(end * 1_494.0, end * 1_464.0);
        for i in 0..6 {
            let c = 3.0 + 5.4 * i as f32;
            for side in [-1.0f32, 1.0] {
                let (y0, y1) = sorted(side * (c - 0.9), side * (c + 0.9));
                add_rect(&mut m, x0, y0, x1, y1, z);
            }
        }
        // Aiming point: 45 m x 9 m blocks, 300 m in.
        let (a0, a1) = sorted(end * 1_200.0, end * 1_155.0);
        for side in [-1.0f32, 1.0] {
            let (y0, y1) = sorted(side * 10.0, side * 19.0);
            add_rect(&mut m, a0, y0, a1, y1, z);
        }
    }
    // Centreline: 36 m dashes, 24 m gaps, 0.9 m wide.
    let mut x = -1_400.0f32;
    while x + 36.0 <= 1_400.0 {
        add_rect(&mut m, x, -0.45, x + 36.0, 0.45, z);
        x += 60.0;
    }
    // Side stripes.
    for side in [-1.0f32, 1.0] {
        let (y0, y1) = sorted(side * 37.6, side * 38.5);
        add_rect(&mut m, -1_494.0, y0, 1_494.0, y1, z);
    }
    m
}

/// Synthetic airport lights: runway edge (white, every 60 m), centreline
/// (white, the last 300 m before the east end red), threshold (green, west)
/// and end (red, east) bars, approach lights west of the threshold, taxiway
/// A edge (blue) and centreline (green).
fn airport_lights() -> Vec<Light> {
    let mut v = Vec::new();
    let mut put = |x: f32, y: f32, z: f32, color_rgb8: [u8; 3], kind: LightType, intensity_cd: f32| {
        v.push(Light { position: [x, y, z], color_rgb8, kind, intensity_cd });
    };
    for i in 0..=50 {
        let x = -1_500.0 + 60.0 * i as f32;
        for y in [-41.0, 41.0] {
            put(x, y, 0.5, [255, 250, 235], LightType::RunwayEdge, 10_000.0);
        }
    }
    for i in 0..100 {
        let x = -1_485.0 + 30.0 * i as f32;
        let red = x > 1_200.0;
        put(x, 0.6, 0.3, if red { [255, 40, 30] } else { [255, 250, 235] }, LightType::RunwayCenterline, 5_000.0);
    }
    for i in 0..=20 {
        let y = -40.0 + 4.0 * i as f32;
        put(-1_502.0, y, 0.5, [40, 255, 90], LightType::RunwayEdge, 10_000.0);
        put(1_502.0, y, 0.5, [255, 40, 30], LightType::RunwayEdge, 10_000.0);
    }
    for i in 1..=10 {
        let x = -1_500.0 - 30.0 * i as f32;
        for y in [-4.0, 0.0, 4.0] {
            put(x, y, 1.0, [255, 255, 255], LightType::Approach, 20_000.0);
        }
    }
    for i in 0..=50 {
        let x = -1_500.0 + 60.0 * i as f32;
        for y in [99.0, 141.0] {
            put(x, y, 0.4, [40, 90, 255], LightType::TaxiwayEdge, 200.0);
        }
        put(x + 30.0, 120.0, 0.25, [30, 230, 90], LightType::TaxiwayCenterline, 200.0);
    }
    v
}

fn sorted(a: f32, b: f32) -> (f32, f32) {
    if a <= b {
        (a, b)
    } else {
        (b, a)
    }
}

fn add_rect(m: &mut Mesh, x0: f32, y0: f32, x1: f32, y1: f32, z: f32) {
    let base = m.positions.len() as u32;
    m.positions.extend([[x0, y0, z], [x1, y0, z], [x1, y1, z], [x0, y1, z]]);
    m.indices.extend([base, base + 1, base + 2, base, base + 2, base + 3]);
}

fn rect(x0: f32, y0: f32, x1: f32, y1: f32, z: f32) -> Mesh {
    Mesh {
        positions: vec![[x0, y0, z], [x1, y0, z], [x1, y1, z], [x0, y1, z]],
        indices: vec![0, 1, 2, 0, 2, 3],
    }
}
