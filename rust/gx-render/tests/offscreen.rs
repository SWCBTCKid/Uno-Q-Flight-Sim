//! Offline render test on the board GPU: the S3 frame rendered by the real
//! GBM/EGL/GLES path into an offscreen GBM surface on the render node (no
//! DRM master, nothing on screen), read back and compared with the CPU
//! rasterization of the same triangle list. Also proves the GPU timer.
//!
//! Needs the Adreno render node, so it runs only when GX_RENDER_OFFSCREEN_TEST=1
//! (set by tools/m1_s3_board_tests.sh / m1_s4_board_tests.sh on the board);
//! elsewhere it is skipped. S4 adds the world frame (depth target, static
//! buffers, ground/aircraft/lights/HUD) against a CPU ray-cast reference.

#![cfg(target_os = "linux")]

use gx_render::linux::Offscreen;
use gx_render::scene::{HudInput, Rgb, Scene, BANNER, GROUND, HUD_TEXT, SKY};
use gx_sim::{InputHealth, InputStatus, PauseReason, RunState, UnavailableReason};

fn enabled() -> bool {
    if std::env::var("GX_RENDER_OFFSCREEN_TEST").as_deref() == Ok("1") {
        return true;
    }
    eprintln!("SKIP: set GX_RENDER_OFFSCREEN_TEST=1 on the board to render on the GPU");
    false
}

fn u8c(c: Rgb) -> [u8; 3] {
    c.map(|v| (v * 255.0).round() as u8)
}

fn close(a: [u8; 3], b: [u8; 3]) -> bool {
    a.iter().zip(b).all(|(x, y)| x.abs_diff(y) <= 2)
}

#[test]
fn gpu_frame_matches_the_cpu_reference() {
    if !enabled() {
        return;
    }
    let node = std::env::var("GX_RENDER_NODE").unwrap_or_else(|_| "/dev/dri/renderD128".into());
    let mut gpu = Offscreen::open(&node).expect("offscreen GBM/EGL/GLES on the render node");
    let info = gpu.info();
    eprintln!("renderer {} / {}; depth {} stencil {}; clear {}; invalidate {}; timer {} {}",
        info.gl.renderer, info.gl.version, info.gl.depth_bits, info.gl.stencil_bits, info.gl.clear_mask,
        info.gl.invalidate, info.gl.timer, info.gl.timer_reason);
    assert!(info.gl.timer, "GL_EXT_disjoint_timer_query required: {}", info.gl.timer_reason);

    let mut scene = Scene::new();
    let prompt = vec!["PHASE TEST: offscreen render".to_owned()];
    let lost = InputHealth { status: InputStatus::Unavailable(UnavailableReason::Disconnected), loss_ms: 4_000 };
    let cases = [
        (0.0, 0.0, RunState::Running, None, InputHealth { status: InputStatus::Valid, loss_ms: 0 }),
        (25f64.to_radians(), 8f64.to_radians(), RunState::Running, None, lost),
        (-10f64.to_radians(), -5f64.to_radians(), RunState::Paused, Some(PauseReason::User), InputHealth { status: InputStatus::Valid, loss_ms: 0 }),
    ];
    let probes: Vec<(u32, u32)> = vec![(640, 200), (640, 520), (300, 300), (980, 420), (4, 600), (21, 13), (150, 310), (1270, 700)];
    let mut gpu_ns = Vec::new();
    for round in 0..3 {
        for (roll, pitch, run, pause, input) in cases {
            let hud = HudInput {
                roll_rad: roll,
                pitch_rad: pitch,
                heading_rad: 0.0,
                airspeed_kt: 142,
                altitude_agl_ft: 1234,
                heading_deg: 281,
                vertical_speed_fpm: 850,
                engine_n1_pct: 87,
                pitch_trim_pct: None,
                config: None,
                run,
                pause,
                input,
                prompt: &prompt,
                panel: None,
            };
            scene.build(&hud);
            let (report, px) = gpu.present_probe(scene.vertices(), &probes).expect("present");
            for (&(x, y), got) in probes.iter().zip(&px) {
                let want = u8c(scene.color_at(x as f32, y as f32, [0.0; 3]));
                assert!(close(*got, want), "round {round} roll {roll:.2} pitch {pitch:.2}: pixel ({x},{y}) GPU {got:?} != CPU {want:?}");
            }
            if let Some(ns) = report.gpu.ns {
                gpu_ns.push(ns);
            }
            assert!(report.timing.fence_ns >= report.timing.swap_end_ns);
        }
    }
    // The probes cover the scene's main colours at least once.
    let mut seen = Vec::new();
    for (roll, pitch, run, pause, input) in cases {
        let hud = HudInput { roll_rad: roll, pitch_rad: pitch, heading_rad: 0.0, airspeed_kt: 142, altitude_agl_ft: 1234, heading_deg: 281, vertical_speed_fpm: 850, engine_n1_pct: 87, pitch_trim_pct: None, config: None, run, pause, input, prompt: &prompt, panel: None };
        scene.build(&hud);
        seen.extend(probes.iter().map(|&(x, y)| scene.color_at(x as f32, y as f32, [0.0; 3])));
    }
    for c in [SKY, GROUND, BANNER, HUD_TEXT] {
        assert!(seen.contains(&c), "probe set misses {c:?}");
    }
    gpu_ns.sort_unstable();
    eprintln!("GPU time over {} timed frames: min {} ns, median {} ns, max {} ns", gpu_ns.len(), gpu_ns[0], gpu_ns[gpu_ns.len() / 2], gpu_ns[gpu_ns.len() - 1]);
    assert!(gpu_ns.len() >= 6, "GPU timer produced {} samples of 9", gpu_ns.len());
    assert!(gpu_ns.iter().all(|&ns| ns > 0 && ns < 33_000_000));
}

// ------------------------------------------------------------------- S4 --

use gx_render::aircraft::global5000;
use gx_render::camera::{CameraMode, View};
use gx_render::linux::{WorldBuffers, WorldDraw};
use gx_render::reference::{probe_set, test_hud, S4_CASES};
use gx_render::world::{WorldMesh, LAND, RUNWAY, SKY_CLEAR, WATER};

#[test]
fn gpu_world_frame_matches_the_cpu_ray_reference() {
    if !enabled() {
        return;
    }
    let node = std::env::var("GX_RENDER_NODE").unwrap_or_else(|_| "/dev/dri/renderD128".into());
    let mut gpu = Offscreen::open_world(&node).expect("offscreen target with a depth buffer");
    let info = gpu.info();
    assert!(info.gl.depth_bits >= 24, "depth {}", info.gl.depth_bits);
    assert!(info.gl.clear_mask.contains("depth") && info.gl.invalidate.contains("depth"), "{} / {}", info.gl.clear_mask, info.gl.invalidate);
    let world = WorldMesh::build(&gx_world::synthetic_sfo_like()).unwrap();
    let model = global5000();
    let bytes = gpu.load_world(&WorldBuffers { ground: &world.ground, lights: &world.lights, aircraft: &model }).expect("upload");
    eprintln!("S4 world: {} ground + {} aircraft triangles, {} lights, {bytes} static bytes", world.ground.len() / 3, model.len() / 3, world.lights.len());

    let prompt = vec!["PHASE TEST: offscreen S4 world".to_owned()];
    let mut scene = Scene::new();
    let mut gpu_ns = Vec::new();
    let mut seen: Vec<Rgb> = Vec::new();
    for &(name, mode, pose) in S4_CASES().iter() {
        let view = View::new(mode, &pose);
        let hud = test_hud(&prompt);
        scene.build_overlay(&hud, view.px_per_deg(), mode.label());
        let draw = WorldDraw {
            clear: SKY_CLEAR,
            ground_mvp: view.world_mvp().to_f32(),
            aircraft_mvp: (mode == CameraMode::Chase).then(|| view.model_mvp(&pose).to_f32()),
            point_k: 2_000.0,
        };
        let (probes, want) = probe_set(&view, &pose, &world, &model, &scene);
        assert!(probes.len() >= 120, "{name}: only {} usable probes", probes.len());
        for round in 0..2 {
            let (report, px) = gpu.present_world_probe(&draw, scene.vertices(), &probes).expect("present");
            assert_eq!(report.draws, if mode == CameraMode::Chase { 4 } else { 3 }, "{name}");
            let bad: Vec<String> = probes
                .iter()
                .zip(&px)
                .zip(&want)
                .filter(|((_, got), w)| !close(**got, u8c(**w)))
                .map(|(((x, y), got), w)| format!("({x},{y}) GPU {got:?} CPU {:?}", u8c(*w)))
                .collect();
            // A probe exactly on a sub-pixel edge the cross test missed may
            // flip between rasterizer and ray cast: allow 1 %.
            assert!(bad.len() * 100 <= probes.len(), "{name} round {round}: {} of {} probes differ: {:?}", bad.len(), probes.len(), &bad[..bad.len().min(8)]);
            if let Some(ns) = report.gpu.ns {
                gpu_ns.push(ns);
            }
        }
        seen.extend(want);
        eprintln!("{name}: {} probes compared", probes.len());
    }
    for (c, what) in [(SKY_CLEAR, "sky"), (LAND, "land"), (WATER, "water"), (RUNWAY, "runway")] {
        assert!(seen.contains(&c), "the probes never compared {what}");
    }
    assert!(seen.iter().any(|c| model.iter().any(|v| v.color == *c)), "the probes never compared the aircraft");
    gpu_ns.sort_unstable();
    eprintln!("S4 GPU time over {} timed frames: min {} ns, median {} ns, max {} ns", gpu_ns.len(), gpu_ns[0], gpu_ns[gpu_ns.len() / 2], gpu_ns[gpu_ns.len() - 1]);
    assert!(gpu_ns.len() >= 6 && gpu_ns.iter().all(|&ns| ns > 0 && ns < 33_000_000));
}

// -------------------------------------------------------------- AP-001 --

use gx_render::panel::{ACTIVE, BG, INFO_X, TOP, WARN};
use gx_render::reference::{busiest_hud, TEST_CURSOR};

/// AP-001 W2: the autopilot panel strip, the cursor, the lit AP button and
/// the warning line drawn by the GPU match the CPU reference (the host test
/// `tests/panel_frame.rs` checks the same frame on the CPU only).
#[test]
fn gpu_panel_frame_matches_the_cpu_reference() {
    if !enabled() {
        return;
    }
    let node = std::env::var("GX_RENDER_NODE").unwrap_or_else(|_| "/dev/dri/renderD128".into());
    let mut gpu = Offscreen::open(&node).expect("offscreen GBM/EGL/GLES on the render node");
    let prompt = vec!["PHASE TEST: offscreen autopilot panel".to_owned()];
    let mut scene = Scene::new();
    let drawn = scene.build(&busiest_hud(&prompt, true));
    let cpu = |x: u32, y: u32| scene.color_at(x as f32, y as f32, [0.0; 3]);
    // A grid over the strip (glyphs and buttons are pixel-aligned, so every
    // pixel centre is clear of an edge), away from the slanted cursor edges,
    // plus named probes: cursor fill and edge, AP button, warning glyphs.
    let (cx, cy) = (TEST_CURSOR.0 as u32, TEST_CURSOR.1 as u32);
    let mut probes: Vec<(u32, u32)> = (TOP as u32..720)
        .step_by(5)
        .flat_map(|y| (0..1280).step_by(7).map(move |x| (x, y)))
        .filter(|&(x, y)| !(cx.saturating_sub(5)..cx + 22).contains(&x) || !(cy.saturating_sub(6)..cy + 28).contains(&y))
        .collect();
    let named = [(cx + 1, cy + 6), (cx + 13, cy + 13), (14, TOP as u32 + 12)];
    probes.extend(named);
    let warn: Vec<(u32, u32)> = (TOP as u32..720).flat_map(|y| (INFO_X as u32..1280).map(move |x| (x, y))).filter(|&(x, y)| cpu(x, y) == WARN).take(8).collect();
    assert_eq!(warn.len(), 8, "the CPU frame draws the warning");
    probes.extend(&warn);
    let want: Vec<Rgb> = probes.iter().map(|&(x, y)| cpu(x, y)).collect();
    assert_eq!(&want[want.len() - 11..want.len() - 8], &[[1.0; 3], [0.0; 3], ACTIVE]);
    for c in [BG, ACTIVE, WARN, [1.0; 3]] {
        assert!(want.contains(&c), "probe set misses {c:?}");
    }
    let mut gpu_ns = Vec::new();
    for round in 0..3 {
        let (report, px) = gpu.present_probe(scene.vertices(), &probes).expect("present");
        let bad: Vec<String> = probes
            .iter()
            .zip(&px)
            .zip(&want)
            .filter(|((_, got), w)| !close(**got, u8c(**w)))
            .map(|(((x, y), got), w)| format!("({x},{y}) GPU {got:?} CPU {:?}", u8c(*w)))
            .collect();
        assert!(bad.is_empty(), "round {round}: {} of {} probes differ: {:?}", bad.len(), probes.len(), &bad[..bad.len().min(8)]);
        if let Some(ns) = report.gpu.ns {
            gpu_ns.push(ns);
        }
    }
    gpu_ns.sort_unstable();
    eprintln!("panel frame: {} vertices, {} probes; GPU ns {:?}", drawn.vertices, probes.len(), gpu_ns);
}
