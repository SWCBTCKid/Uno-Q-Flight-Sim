use gx_render::photo::*;
use gx_world::*;

fn visual(encoding: TextureEncoding) -> VisualAsset {
    let world = synthetic_sfo_like();
    let texture_bytes = match encoding {
        TextureEncoding::Astc6x6Srgb => 16,
        TextureEncoding::Rgba8Srgb => 64,
    };
    VisualAsset {
        format_major: 1,
        format_minor: 1, // the 1.1 hinge channels
        frame_id: world.georeference.frame_id,
        frame_generation: world.georeference.frame_generation,
        textures: vec![Texture {
            id: 10,
            width: 4,
            height: 4,
            mip_levels: vec![vec![7; texture_bytes]],
            encoding,
        }],
        materials: vec![Material {
            id: 20,
            base_color_texture: 10,
            base_color_factor: [1.0; 4],
            roughness: 0.7,
            metallic: 0.1,
        }],
        meshes: vec![VisualMesh {
            kind: VisualKind::Aircraft,
            material: 20,
            vertices: vec![
                VisualVertex {
                    position: [0.0, 0.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                    uv: [0.0, 0.0],
                    animation: 1,
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
fn prepares_batches_and_exact_memory_accounting() {
    let visual = visual(TextureEncoding::Rgba8Srgb);
    let plan = prepare_photo(&synthetic_sfo_like(), &visual, "", VisualLimits::default()).unwrap();
    assert_eq!(plan.textures[0].gl_internal_format, GL_SRGB8_ALPHA8);
    assert_eq!((plan.batches[0].first_index, plan.batches[0].index_count), (0, 3));
    assert_eq!(plan.indices, vec![0, 1, 2]);
    assert_eq!(plan.vertices.iter().map(|v| v.animation).collect::<Vec<_>>(), [1.0, 0.0, 0.0]);
    assert_eq!(plan.batches[0].kind, VisualKind::Aircraft);
    assert_eq!(plan.memory.texture_bytes, 64);
    assert_eq!(
        plan.memory.vertex_bytes,
        3 * std::mem::size_of::<TexturedVertex>() as u64
    );
    assert_eq!(plan.memory.index_bytes, 12);
    assert_eq!(
        plan.memory.gpu_static_bytes,
        plan.memory.texture_bytes + plan.memory.vertex_bytes + 12
    );
    assert_eq!(plan.memory.triangles, [0, 0, 0, 1]);
}

#[test]
fn astc_is_gated_by_an_exact_extension_token() {
    let world = synthetic_sfo_like();
    let visual = visual(TextureEncoding::Astc6x6Srgb);
    assert!(prepare_photo(
        &world,
        &visual,
        "GL_KHR_texture_compression_astc_ldr_extra",
        VisualLimits::default()
    )
    .is_err());
    let plan = prepare_photo(&world, &visual, ASTC_EXTENSION, VisualLimits::default()).unwrap();
    assert!(plan.textures[0].compressed);
    assert_eq!(
        plan.textures[0].gl_internal_format,
        GL_COMPRESSED_SRGB8_ALPHA8_ASTC_6X6_KHR
    );
}

#[test]
fn stale_visual_pack_never_reaches_gles() {
    let world = synthetic_sfo_like();
    let mut visual = visual(TextureEncoding::Rgba8Srgb);
    visual.frame_generation += 1;
    assert_eq!(
        prepare_photo(&world, &visual, "", VisualLimits::default()).unwrap_err(),
        "visual/world frame identity mismatch"
    );
}

#[test]
fn shader_contract_is_gles3_textured_and_lit() {
    for token in ["#version 300 es", "a_normal", "a_uv", "a_animation", "u_joints[195]", "u_mvp", "u_normal"] {
        assert!(PHOTO_VERTEX_SHADER.contains(token));
    }
    for token in [
        "sampler2D",
        "texture(",
        "u_base_color_factor",
        "u_roughness",
        "precision highp float",
        "encode(",
    ] {
        assert!(photo_fragment_shader().contains(token));
    }
    assert!(sky_fragment_shader().contains("u_forward") && SKY_VERTEX_SHADER.contains("gl_VertexID"));
}

fn rotate_about(p: [f32; 3], origin: [f32; 3], axis: [f32; 3], angle: f32) -> [f32; 3] {
    let n = (axis.iter().map(|v| v * v).sum::<f32>()).sqrt();
    let a = axis.map(|v| v / n);
    let v = std::array::from_fn::<_, 3, _>(|i| p[i] - origin[i]);
    let cross = [a[1] * v[2] - a[2] * v[1], a[2] * v[0] - a[0] * v[2], a[0] * v[1] - a[1] * v[0]];
    let dot: f32 = a.iter().zip(v).map(|(a, v)| a * v).sum();
    let (s, c) = angle.sin_cos();
    std::array::from_fn(|i| origin[i] + v[i] * c + cross[i] * s + a[i] * dot * (1.0 - c))
}

#[test]
fn positive_surface_positions_move_trailing_edges_with_jsbsim_signs() {
    // FRD: +y right, +z down. JSBSim/project contract: positive aileron
    // produces right roll (right TE up, left TE down); positive elevator is
    // TE down; positive rudder is TE left (negative y).
    let right_aileron = [-4.5, 10.0, 1.0];
    let left_aileron = [-4.5, -10.0, 1.0];
    let ra = rotate_about(right_aileron, [-2.2, 7.0, 1.1], [-3.7, 6.7, -0.4], -0.35);
    let la = rotate_about(left_aileron, [-2.2, -7.0, 1.1], [-3.7, -6.7, -0.4], -0.35);
    assert!(ra[2] < right_aileron[2], "positive aileron: right TE must rise in FRD: {ra:?}");
    assert!(la[2] > left_aileron[2], "positive aileron: left TE must fall in FRD: {la:?}");

    let right_elevator = [-14.0, 3.0, -4.0];
    let left_elevator = [-14.0, -3.0, -4.0];
    let re = rotate_about(right_elevator, [-11.35, 0.3, -4.05], [-3.3, 4.3, 0.25], 0.35);
    let le = rotate_about(left_elevator, [-11.35, -0.3, -4.05], [-3.3, -4.3, 0.25], -0.35);
    assert!(re[2] > right_elevator[2] && le[2] > left_elevator[2], "positive elevator: both TEs must fall in FRD: {re:?} {le:?}");

    let rudder = [-12.0, 0.0, -2.0];
    let r = rotate_about(rudder, [-10.25, 0.0, -1.0], [-2.25, 0.0, -3.0], -0.35);
    assert!(r[1] < rudder[1], "positive rudder: TE must move left in FRD: {r:?}");

    // The 1.1 hinges now run as joints: they must reproduce the former
    // shader formula (angle = -aileron*0.35, +elevator*0.35 / -elevator*0.35,
    // -rudder*0.35 rad) exactly, so the accepted bomb-5000 animation holds.
    let drivers = [0.7, -0.4, 0.9, 0.0, 0.0, 0.0];
    let m = joint_transforms(&legacy_joints(), &drivers);
    let close = |a: [f32; 3], b: [f32; 3]| a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-4);
    let cases: [(usize, [f32; 3], [f32; 3], [f32; 3], f32); 5] = [
        (1, right_aileron, [-2.2, 7.0, 1.1], [-3.7, 6.7, -0.4], -0.7 * 0.35),
        (2, left_aileron, [-2.2, -7.0, 1.1], [-3.7, -6.7, -0.4], -0.7 * 0.35),
        (3, right_elevator, [-11.35, 0.3, -4.05], [-3.3, 4.3, 0.25], -0.4 * 0.35),
        (4, left_elevator, [-11.35, -0.3, -4.05], [-3.3, -4.3, 0.25], 0.4 * 0.35),
        (5, rudder, [-10.25, 0.0, -1.0], [-2.25, 0.0, -3.0], -0.9 * 0.35),
    ];
    for (j, p, o, a, angle) in cases {
        let got = transform_point(&m[j], p);
        let want = rotate_about(p, o, a, angle);
        assert!(close(got, want), "joint {j}: {got:?} vs {want:?}");
    }
}

#[test]
fn driver_uniforms_are_finite_and_bounded() {
    assert_eq!(sanitize_drivers([f64::NAN, f64::INFINITY, f64::NEG_INFINITY, f64::NAN, 2.0, -1.0]), [0.0, 0.0, 0.0, 0.0, 1.0, 0.0]);
    assert_eq!(sanitize_drivers([2.0, -2.0, 0.25, 0.5, 0.5, 1.0]), [1.0, -1.0, 0.25, 0.5, 0.5, 1.0]);
}

#[test]
fn joints_compose_parent_first_and_translate_along_the_axis() {
    // A gear leg (gear driver) about +y through (0,0,0), and a door child
    // translating 1 m along +x at gear 1: the door moves, then is carried
    // by the leg.
    let leg = Joint {
        parent: 0,
        driver: JointDriver::Gear,
        kind: JointKind::Rotate,
        center: [0.0, 0.0, 0.0],
        axis: [0.0, 1.0, 0.0],
        table: vec![[0.0, 0.0], [1.0, 90.0]],
    };
    let slide = Joint {
        parent: 1,
        driver: JointDriver::Gear,
        kind: JointKind::Translate,
        center: [0.0; 3],
        axis: [1.0, 0.0, 0.0],
        table: vec![[0.0, 0.0], [1.0, 1.0]],
    };
    let m = joint_transforms(&[leg, slide], &[0.0, 0.0, 0.0, 0.0, 0.0, 1.0]);
    let close = |a: [f32; 3], b: [f32; 3]| a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-5);
    // +90 deg right-handed about +y takes +x to -z.
    assert!(close(transform_point(&m[1], [1.0, 0.0, 0.0]), [0.0, 0.0, -1.0]));
    // Child: (0,0,0) slides to (1,0,0), then the leg rotates it to (0,0,-1).
    assert!(close(transform_point(&m[2], [0.0, 0.0, 0.0]), [0.0, 0.0, -1.0]));
    assert_eq!(transform_point(&m[0], [3.0, 4.0, 5.0]), [3.0, 4.0, 5.0], "joint 0 is static");
    assert_eq!(joint_rows(&[], &[0.0; 6]).len(), 3);
    assert!(3 * (gx_world::MAX_JOINTS + 1) == JOINT_ROWS && PHOTO_VERTEX_SHADER.contains(&format!("u_joints[{JOINT_ROWS}]")));
}

#[test]
fn batches_draw_terrain_then_decals_then_aircraft_with_rebased_indices() {
    let world = synthetic_sfo_like();
    let mut v = visual(TextureEncoding::Rgba8Srgb);
    let mut decal = v.meshes[0].clone();
    decal.kind = VisualKind::Decal;
    decal.vertices.iter_mut().for_each(|vertex| vertex.animation = 0);
    let mut terrain = v.meshes[0].clone();
    terrain.kind = VisualKind::Terrain;
    terrain.vertices.iter_mut().for_each(|vertex| vertex.animation = 0);
    v.meshes.push(decal);
    v.meshes.push(terrain);
    let plan = prepare_photo(&world, &v, "", VisualLimits::default()).unwrap();
    let kinds: Vec<_> = plan.batches.iter().map(|b| b.kind).collect();
    assert_eq!(kinds, [VisualKind::Terrain, VisualKind::Decal, VisualKind::Aircraft]);
    assert_eq!(plan.indices, vec![0, 1, 2, 3, 4, 5, 6, 7, 8]);
    assert_eq!(plan.batches[2].first_index, 6);
    assert_eq!(plan.memory.triangles, [1, 1, 0, 1]);
}

#[test]
fn only_building_only_textures_repeat() {
    let world = synthetic_sfo_like();
    let mut v = visual(TextureEncoding::Rgba8Srgb);
    v.format_minor = 2;
    v.meshes[0].vertices.iter_mut().for_each(|vertex| vertex.animation = 0);
    let shared = v.textures[0].clone();
    // 11: building only (facade). 12: building and terrain (roofs on the photo).
    for (id, material) in [(11, 21), (12, 22)] {
        v.textures.push(Texture { id, ..shared.clone() });
        v.materials.push(Material { id: material, base_color_texture: id, ..v.materials[0] });
        let mut b = v.meshes[0].clone();
        b.kind = VisualKind::Building;
        b.material = material;
        v.meshes.push(b);
    }
    let mut terrain = v.meshes[0].clone();
    terrain.kind = VisualKind::Terrain;
    terrain.material = 22;
    v.meshes.push(terrain);
    let plan = prepare_photo(&world, &v, "", VisualLimits::default()).unwrap();
    let repeat: Vec<_> = plan.textures.iter().map(|t| (t.id, t.repeat)).collect();
    assert_eq!(repeat, [(10, false), (11, true), (12, false)]);
}
