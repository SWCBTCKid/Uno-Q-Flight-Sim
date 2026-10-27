use gx_world::{
    read_world, synthetic_sfo_like, write_world, EnuPoint, FormatError, GeodeticPoint, Layer,
    Limits,
};

#[test]
fn synthetic_fixture_round_trips_exactly() {
    let expected = synthetic_sfo_like();
    let mut bytes = Vec::new();
    write_world(&expected, Limits::default(), &mut bytes).unwrap();
    let actual = read_world(bytes.as_slice(), Limits::default()).unwrap();
    assert_eq!(actual, expected);
    assert_eq!(actual.chunks[0].lods[0].layers.len(), 6);
}

#[test]
fn all_sfo_layer_types_are_present() {
    let world = synthetic_sfo_like();
    let layers = &world.chunks[0].lods[0].layers;
    assert!(layers.iter().any(|layer| matches!(layer, Layer::Land(_))));
    assert!(layers.iter().any(|layer| matches!(layer, Layer::Water(_))));
    assert!(layers.iter().any(|layer| matches!(layer, Layer::Runway(_))));
    assert!(layers
        .iter()
        .any(|layer| matches!(layer, Layer::Taxiway(_))));
    assert!(layers
        .iter()
        .any(|layer| matches!(layer, Layer::Markings(_))));
    assert!(layers.iter().any(|layer| matches!(layer, Layer::Lights(_))));
}

#[test]
fn rejects_payload_corruption() {
    let mut bytes = Vec::new();
    write_world(&synthetic_sfo_like(), Limits::default(), &mut bytes).unwrap();
    let last = bytes.len() - 1;
    bytes[last] ^= 0x80;
    assert!(matches!(
        read_world(bytes.as_slice(), Limits::default()),
        Err(FormatError::HashMismatch { .. })
    ));
}

#[test]
fn rejects_excessive_count_before_allocation() {
    let mut bytes = Vec::new();
    write_world(&synthetic_sfo_like(), Limits::default(), &mut bytes).unwrap();
    // Header chunk count is at byte 32. Hash covers payload, not header.
    bytes[32..36].copy_from_slice(&u32::MAX.to_le_bytes());
    assert!(matches!(
        read_world(bytes.as_slice(), Limits::default()),
        Err(FormatError::LimitExceeded("max_chunks"))
    ));
}

#[test]
fn rejects_unknown_coordinate_metadata() {
    let mut bytes = Vec::new();
    write_world(&synthetic_sfo_like(), Limits::default(), &mut bytes).unwrap();
    // Coordinate-system tag is the final u32 in the fixed header.
    bytes[36..40].copy_from_slice(&0_u32.to_le_bytes());
    assert!(matches!(
        read_world(bytes.as_slice(), Limits::default()),
        Err(FormatError::Invalid("unsupported coordinate system"))
    ));
}

#[test]
fn writer_rejects_geometry_outside_chunk() {
    let mut world = synthetic_sfo_like();
    if let Layer::Land(mesh) = &mut world.chunks[0].lods[0].layers[0] {
        mesh.positions[0][0] = -3_001.0;
    }
    assert!(matches!(
        write_world(&world, Limits::default(), Vec::new()),
        Err(FormatError::Invalid("position outside chunk bounds"))
    ));
}

#[test]
fn enu_origin_is_zero_and_east_is_positive_x() {
    let world = synthetic_sfo_like();
    assert_enu_near(
        world.geodetic_to_enu(world.origin),
        EnuPoint {
            east_m: 0.0,
            north_m: 0.0,
            up_m: 0.0,
        },
        1.0e-7,
    );
    let east = world.geodetic_to_enu(GeodeticPoint {
        longitude_deg: world.origin.longitude_deg + 0.001,
        ..world.origin
    });
    assert!(east.east_m > 80.0 && east.east_m < 100.0);
    assert!(east.north_m.abs() < 1.0);
}

fn assert_enu_near(actual: EnuPoint, expected: EnuPoint, epsilon: f64) {
    assert!((actual.east_m - expected.east_m).abs() < epsilon);
    assert!((actual.north_m - expected.north_m).abs() < epsilon);
    assert!((actual.up_m - expected.up_m).abs() < epsilon);
}

// ---- Format 2.0 georeference block (ISS-035) ----

use gx_world::{
    GeoidModel, Georeference, FORMAT_MAJOR, SFO_FIELD_ELEVATION_MSL_M,
    SFO_GEOID_SEPARATION_EGM2008_M,
};

const HEADER: usize = 40;
/// Payload starts with origin (3 x f64), then the georeference block.
const GEO: usize = HEADER + 24;

fn packed() -> Vec<u8> {
    let mut bytes = Vec::new();
    write_world(&synthetic_sfo_like(), Limits::default(), &mut bytes).unwrap();
    bytes
}

/// Re-seal header length and FNV-1a hash after a payload edit, so the test
/// exercises field validation rather than the hash check.
fn reseal(bytes: &mut [u8]) {
    let payload_len = (bytes.len() - HEADER) as u64;
    bytes[16..24].copy_from_slice(&payload_len.to_le_bytes());
    let mut hash = 0xcbf2_9ce4_8422_2325_u64;
    for &byte in &bytes[HEADER..] {
        hash ^= u64::from(byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    bytes[24..32].copy_from_slice(&hash.to_le_bytes());
}

fn read(bytes: &[u8]) -> Result<gx_world::WorldAsset, FormatError> {
    read_world(bytes, Limits::default())
}

fn assert_invalid(bytes: &[u8], reason: &str) {
    match read(bytes) {
        Err(FormatError::Invalid(actual)) => assert_eq!(actual, reason),
        other => panic!("expected Invalid({reason}), got {other:?}"),
    }
}

#[test]
fn georeference_round_trips_and_is_at_expected_offset() {
    let bytes = packed();
    assert_eq!(u16::from_le_bytes([bytes[8], bytes[9]]), FORMAT_MAJOR);
    assert_eq!(FORMAT_MAJOR, 2);
    assert_eq!(&bytes[GEO..GEO + 4], b"GEOR");
    let world = read(&bytes).unwrap();
    assert_eq!(
        world.georeference,
        Georeference {
            frame_id: 0x5346_4f53_594e_0001,
            frame_generation: 1,
            geoid_model: GeoidModel::Egm2008,
            geoid_separation_m: -32.2753,
        }
    );
    assert_eq!(world, synthetic_sfo_like());
}

#[test]
fn rejects_zero_frame_id_and_generation() {
    let mut bytes = packed();
    bytes[GEO + 8..GEO + 16].fill(0);
    reseal(&mut bytes);
    assert_invalid(&bytes, "frame id is zero");

    let mut bytes = packed();
    bytes[GEO + 16..GEO + 24].fill(0);
    reseal(&mut bytes);
    assert_invalid(&bytes, "frame generation is zero");
}

#[test]
fn rejects_unknown_geoid_model_and_reserved_bits() {
    for model in [0_u32, 3, u32::MAX] {
        let mut bytes = packed();
        bytes[GEO + 24..GEO + 28].copy_from_slice(&model.to_le_bytes());
        reseal(&mut bytes);
        assert_invalid(&bytes, "unknown geoid model");
    }
    let mut bytes = packed();
    bytes[GEO + 28] = 1;
    reseal(&mut bytes);
    assert_invalid(&bytes, "georeference reserved field");
}

#[test]
fn rejects_non_finite_or_out_of_range_separation() {
    for value in [f64::NAN, f64::INFINITY, -110.5, 90.5, 1.0e6] {
        let mut bytes = packed();
        bytes[GEO + 32..GEO + 40].copy_from_slice(&value.to_le_bytes());
        reseal(&mut bytes);
        assert_invalid(&bytes, "geoid separation out of range");
    }
    // Global extremes stay accepted.
    for value in [-106.9_f64, 86.0] {
        let mut bytes = packed();
        bytes[GEO + 32..GEO + 40].copy_from_slice(&value.to_le_bytes());
        reseal(&mut bytes);
        assert_eq!(read(&bytes).unwrap().georeference.geoid_separation_m, value);
    }
}

#[test]
fn rejects_missing_or_resized_georeference_block() {
    // A 2.0 header over a 1.0-shaped payload (block absent).
    let mut bytes = packed();
    bytes.drain(GEO..GEO + 40);
    reseal(&mut bytes);
    assert_invalid(&bytes, "missing georeference block");

    let mut bytes = packed();
    bytes[GEO + 4..GEO + 8].copy_from_slice(&24_u32.to_le_bytes());
    reseal(&mut bytes);
    assert_invalid(&bytes, "georeference block size");

    // Payload ends inside the block.
    let mut bytes = packed();
    bytes.truncate(GEO + 20);
    reseal(&mut bytes);
    assert_invalid(&bytes, "truncated payload");
}

#[test]
fn rejects_obsolete_v1_pack_with_clear_error() {
    // Genuine 1.0 layout: no georeference block, major 1.
    let mut bytes = packed();
    bytes.drain(GEO..GEO + 40);
    bytes[8..10].copy_from_slice(&1_u16.to_le_bytes());
    reseal(&mut bytes);
    let error = read(&bytes).unwrap_err();
    assert!(matches!(error, FormatError::ObsoleteVersion { major: 1, minor: 0 }), "{error:?}");
    let message = error.to_string();
    assert!(message.contains("obsolete") && message.contains("geoid separation"), "{message}");
}

#[test]
fn rejects_future_versions() {
    for (major, minor) in [(3_u16, 0_u16), (2, 1), (0, 0)] {
        let mut bytes = packed();
        bytes[8..10].copy_from_slice(&major.to_le_bytes());
        bytes[10..12].copy_from_slice(&minor.to_le_bytes());
        assert!(
            matches!(read(&bytes), Err(FormatError::UnsupportedVersion { major: a, minor: b }) if a == major && b == minor),
            "{major}.{minor}"
        );
    }
}

#[test]
fn writer_rejects_invalid_georeference() {
    let mut world = synthetic_sfo_like();
    world.georeference.frame_id = 0;
    assert!(matches!(
        write_world(&world, Limits::default(), Vec::new()),
        Err(FormatError::Invalid("frame id is zero"))
    ));
    let mut world = synthetic_sfo_like();
    world.georeference.geoid_separation_m = f64::NAN;
    assert!(matches!(
        write_world(&world, Limits::default(), Vec::new()),
        Err(FormatError::Invalid("geoid separation out of range"))
    ));
}

/// An aircraft at the runway's MSL elevation must sit on the runway in ENU.
/// Applying the separation with the wrong sign would put it ~64 m off, and
/// ignoring it ~32 m off.
#[test]
fn msl_runway_elevation_lands_on_runway_surface() {
    let world = read(&packed()).unwrap();
    let runway_z = world.chunks[0].lods[0]
        .layers
        .iter()
        .find_map(|layer| match layer {
            Layer::Runway(named) => Some(f64::from(named.mesh.positions[0][2])),
            _ => None,
        })
        .unwrap();
    let o = world.origin;
    // At the origin and 1 km east along the runway centreline.
    for d_lon in [0.0, 1_000.0 / (111_320.0 * o.latitude_deg.to_radians().cos())] {
        let enu = world.msl_to_enu(o.latitude_deg, o.longitude_deg + d_lon, SFO_FIELD_ELEVATION_MSL_M);
        assert!((enu.up_m - runway_z).abs() < 0.5, "up {} vs runway {runway_z}", enu.up_m);
    }
    let wrong_sign = world.geodetic_to_enu(GeodeticPoint {
        height_m: SFO_FIELD_ELEVATION_MSL_M - SFO_GEOID_SEPARATION_EGM2008_M,
        ..o
    });
    assert!((wrong_sign.up_m - 64.55).abs() < 0.1, "{}", wrong_sign.up_m);
}

/// World-level ENU against a hand-computed WGS84 reference: from
/// (0N, 0E, 0 m) the point (0N, 90E, 0 m) is at ENU (a, 0, -a).
#[test]
fn world_enu_matches_hand_computed_reference() {
    let mut world = synthetic_sfo_like();
    world.origin = GeodeticPoint { latitude_deg: 0.0, longitude_deg: 0.0, height_m: 0.0 };
    let a = 6_378_137.0;
    assert_enu_near(
        world.geodetic_to_enu(GeodeticPoint { latitude_deg: 0.0, longitude_deg: 90.0, height_m: 0.0 }),
        EnuPoint { east_m: a, north_m: 0.0, up_m: -a },
        1.0e-6,
    );
}

/// S4: the synthetic pack carries runway surface markings inside the runway
/// and edge/centreline/threshold/taxiway lights inside the chunk bounds.
#[test]
fn synthetic_pack_has_markings_and_lights_for_s4() {
    let world = synthetic_sfo_like();
    let chunk = &world.chunks[0];
    let layers = &chunk.lods[0].layers;
    let marks = layers.iter().find_map(|l| match l { Layer::Markings(m) => Some(m), _ => None }).unwrap();
    assert!(marks.indices.len() / 6 >= 60, "{} marking quads", marks.indices.len() / 6);
    assert!(marks.positions.iter().all(|p| p[0].abs() <= 1_500.0 && p[1].abs() <= 40.0 && p[2] > 0.2));
    let lights = layers.iter().find_map(|l| match l { Layer::Lights(l) => Some(&l.lights), _ => None }).unwrap();
    assert!(lights.len() >= 300, "{} lights", lights.len());
    let b = chunk.bounds;
    assert!(lights.iter().all(|l| (0..3).all(|i| l.position[i] >= b.min[i] && l.position[i] <= b.max[i])));
    for kind in [gx_world::LightType::RunwayEdge, gx_world::LightType::RunwayCenterline, gx_world::LightType::TaxiwayEdge,
                 gx_world::LightType::TaxiwayCenterline, gx_world::LightType::Approach] {
        assert!(lights.iter().any(|l| l.kind == kind), "{kind:?}");
    }
}
