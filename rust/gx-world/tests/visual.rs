use gx_world::*;

fn asset(encoding: TextureEncoding) -> VisualAsset {
    let world = synthetic_sfo_like();
    let bytes = match encoding {
        TextureEncoding::Rgba8Srgb => vec![255; 4 * 4 * 4],
        TextureEncoding::Astc6x6Srgb => vec![0; 16],
    };
    VisualAsset {
        format_major: VISUAL_FORMAT_MAJOR,
        // The 1.1 channel contract (fixed hinges 1..=5); 1.2 below.
        format_minor: 1,
        frame_id: world.georeference.frame_id,
        frame_generation: world.georeference.frame_generation,
        textures: vec![Texture {
            id: 1,
            width: 4,
            height: 4,
            mip_levels: vec![bytes],
            encoding,
        }],
        materials: vec![Material {
            id: 2,
            base_color_texture: 1,
            base_color_factor: [1.0; 4],
            roughness: 0.8,
            metallic: 0.0,
        }],
        meshes: vec![VisualMesh {
            kind: VisualKind::Terrain,
            material: 2,
            vertices: vec![
                VisualVertex {
                    position: [0.0, 0.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                    uv: [0.0, 0.0],
                    animation: 0,
                },
                VisualVertex {
                    position: [1.0, 0.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                    uv: [1.0, 0.0],
                    animation: 0,
                },
                VisualVertex {
                    position: [0.0, 1.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                    uv: [0.0, 1.0],
                    animation: 0,
                },
            ],
            indices: vec![0, 1, 2],
        }],
        joints: vec![],
    }
}

#[test]
fn validates_rgba_and_astc_payload_sizes() {
    asset(TextureEncoding::Rgba8Srgb)
        .validate(VisualLimits::default())
        .unwrap();
    asset(TextureEncoding::Astc6x6Srgb)
        .validate(VisualLimits::default())
        .unwrap();
    let mut bad = asset(TextureEncoding::Astc6x6Srgb);
    bad.textures[0].mip_levels[0].pop();
    assert_eq!(
        bad.validate(VisualLimits::default()),
        Err(VisualError::Invalid("texture mip byte length"))
    );
}

#[test]
fn rejects_stale_frame_and_dangling_references() {
    let world = synthetic_sfo_like();
    let mut visual = asset(TextureEncoding::Rgba8Srgb);
    assert!(visual.matches_world(&world));
    visual.frame_generation += 1;
    assert!(!visual.matches_world(&world));
    visual.materials[0].base_color_texture = 99;
    assert_eq!(
        visual.validate(VisualLimits::default()),
        Err(VisualError::Invalid("material texture reference"))
    );
}

#[test]
fn texture_budget_is_enforced_before_gpu_upload() {
    let visual = asset(TextureEncoding::Rgba8Srgb);
    let limits = VisualLimits {
        max_texture_bytes: 63,
        ..VisualLimits::default()
    };
    assert_eq!(
        visual.validate(limits),
        Err(VisualError::LimitExceeded("max_texture_bytes"))
    );
}

#[test]
fn gxvisual_binary_round_trips_and_rejects_bad_magic() {
    let mut want = asset(TextureEncoding::Astc6x6Srgb);
    want.meshes[0].kind = VisualKind::Aircraft;
    want.meshes[0].vertices[0].animation = 5;
    let mut bytes = Vec::new();
    write_visual(&mut bytes, &want, VisualLimits::default()).unwrap();
    assert_eq!(&bytes[..8], b"GXVISUAL");
    assert_eq!(read_visual(bytes.as_slice(), VisualLimits::default()).unwrap(), want);
    bytes[0] = b'X';
    assert_eq!(read_visual(bytes.as_slice(), VisualLimits::default()).unwrap_err(), "bad GXVISUAL magic");
}

#[test]
fn gxvisual_10_defaults_animation_to_static() {
    let mut want = asset(TextureEncoding::Rgba8Srgb);
    want.format_minor = 0;
    let mut bytes = Vec::new();
    write_visual(&mut bytes, &want, VisualLimits::default()).unwrap();
    let got = read_visual(bytes.as_slice(), VisualLimits::default()).unwrap();
    assert!(got.meshes.iter().flat_map(|m| &m.vertices).all(|v| v.animation == 0));
}

#[test]
fn animation_is_restricted_to_aircraft_meshes() {
    let mut visual = asset(TextureEncoding::Rgba8Srgb);
    visual.meshes[0].vertices[0].animation = 1;
    assert_eq!(
        visual.validate(VisualLimits::default()),
        Err(VisualError::Invalid("animation on non-aircraft mesh"))
    );
    visual.meshes[0].kind = VisualKind::Aircraft;
    visual.validate(VisualLimits::default()).unwrap();
}

fn hinge(parent: u32, driver: JointDriver) -> Joint {
    Joint {
        parent,
        driver,
        kind: JointKind::Rotate,
        center: [1.0, 2.0, 3.0],
        axis: [0.0, 1.0, 0.0],
        table: vec![[0.0, 0.0], [0.5, 30.0], [1.0, 40.0]],
    }
}

fn jointed() -> VisualAsset {
    let mut v = asset(TextureEncoding::Rgba8Srgb);
    v.format_minor = 2;
    v.meshes[0].kind = VisualKind::Aircraft;
    v.joints = vec![hinge(0, JointDriver::Gear), hinge(1, JointDriver::Flap)];
    v.meshes[0].vertices[1].animation = 2;
    v
}

#[test]
fn gxvisual_12_joint_table_round_trips() {
    let want = jointed();
    want.validate(VisualLimits::default()).unwrap();
    let mut bytes = Vec::new();
    write_visual(&mut bytes, &want, VisualLimits::default()).unwrap();
    assert_eq!(read_visual(bytes.as_slice(), VisualLimits::default()).unwrap(), want);
    // A 1.1 pack still reads (no joint table on the wire).
    let mut old = asset(TextureEncoding::Rgba8Srgb);
    old.meshes[0].kind = VisualKind::Aircraft;
    old.meshes[0].vertices[0].animation = 5;
    let mut bytes = Vec::new();
    write_visual(&mut bytes, &old, VisualLimits::default()).unwrap();
    assert_eq!(read_visual(bytes.as_slice(), VisualLimits::default()).unwrap(), old);
}

#[test]
fn joint_tables_are_validated() {
    let bad = |f: &dyn Fn(&mut VisualAsset)| {
        let mut v = jointed();
        f(&mut v);
        v.validate(VisualLimits::default())
    };
    let invalid = Err(VisualError::Invalid("joint values"));
    assert_eq!(bad(&|v| v.joints[0].parent = 1), invalid, "parent must precede");
    assert_eq!(bad(&|v| v.joints[1].axis = [0.0, 2.0, 0.0]), invalid, "unit axis");
    assert_eq!(bad(&|v| v.joints[1].table = vec![[0.0, 1.0]]), invalid, "two points minimum");
    assert_eq!(bad(&|v| v.joints[1].table[1][0] = 0.0), invalid, "ascending inputs");
    assert_eq!(bad(&|v| v.joints[1].table[1][1] = f32::NAN), invalid);
    assert_eq!(bad(&|v| v.joints[0].center[2] = f32::INFINITY), invalid);
    assert_eq!(bad(&|v| v.meshes[0].vertices[0].animation = 3), Err(VisualError::Invalid("vertex values")), "joint id in range");
    assert_eq!(bad(&|v| v.format_minor = 1), Err(VisualError::Invalid("joints require GXVISUAL 1.2")));
    assert_eq!(bad(&|v| v.joints = vec![hinge(0, JointDriver::Gear); MAX_JOINTS + 1]), Err(VisualError::LimitExceeded("max_joints")));
}

#[test]
fn joint_tables_interpolate_and_hold_at_the_ends() {
    let j = hinge(0, JointDriver::Gear);
    assert_eq!(j.evaluate(-1.0), 0.0);
    assert_eq!(j.evaluate(0.25), 15.0);
    assert_eq!(j.evaluate(0.75), 35.0);
    assert_eq!(j.evaluate(2.0), 40.0);
    assert_eq!(j.evaluate(f32::NAN), 0.0);
}

#[test]
fn building_meshes_round_trip_in_12_and_are_refused_before() {
    let mut v = asset(TextureEncoding::Rgba8Srgb);
    v.format_minor = 2;
    v.meshes[0].kind = VisualKind::Building;
    let mut bytes = Vec::new();
    write_visual(&mut bytes, &v, VisualLimits::default()).unwrap();
    assert_eq!(read_visual(bytes.as_slice(), VisualLimits::default()).unwrap(), v);
    v.format_minor = 1;
    assert_eq!(v.validate(VisualLimits::default()), Err(VisualError::Invalid("building meshes require GXVISUAL 1.2")));
    v.format_minor = 2;
    v.meshes[0].vertices[0].animation = 1;
    assert!(v.validate(VisualLimits::default()).is_err(), "buildings never animate");
    v.meshes[0].vertices[0].animation = 0;
    v.meshes[0].vertices[0].uv = [37.5, -2.25];
    assert_eq!(v.validate(VisualLimits::default()), Ok(()), "building UVs repeat");
    v.meshes[0].vertices[0].uv = [MAX_BUILDING_UV + 1.0, 0.0];
    assert_eq!(v.validate(VisualLimits::default()), Err(VisualError::Invalid("vertex values")));
    v.meshes[0].kind = VisualKind::Terrain;
    v.meshes[0].vertices[0].uv = [1.5, 0.0];
    assert_eq!(v.validate(VisualLimits::default()), Err(VisualError::Invalid("vertex values")), "other kinds stay in 0..1");
}
