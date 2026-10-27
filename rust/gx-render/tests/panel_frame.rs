//! AP-001 W2 host render test: an S4 HUD frame with the autopilot panel,
//! CPU-rasterized from the same triangle list the GPU draws (the board GPU
//! comparison is `tests/offscreen.rs`). Shows the panel strip, the cursor,
//! the lit AP button and the warning line; the banner and prompts sit above
//! the strip; the busiest frame stays inside the vertex buffer.
//!
//! `GX_PANEL_PPM_DIR=<dir>` also writes the full frames as PPM images for a
//! visual check.

use gx_render::panel::{self, ACTIVE, ARMED, BG, INFO_ARMED, INFO_END, INFO_X, TOP, WARN};
use gx_render::reference::{busiest_hud, test_hud, TEST_CURSOR};
use gx_render::scene::{Rgb, Scene, BANNER, HEIGHT, MAX_VERTICES, PANEL, PROMPT_TEXT, WIDTH};
use gx_render::world::{LAND, SKY_CLEAR};

const CAMERA: &str = "CHASE CAM";

/// Two long operator prompts: four lines once wrapped at 100 columns.
fn prompts() -> Vec<String> {
    let line = "PHASE ILS 28R: INTERCEPT THE LOCALIZER AT 3000 FT, FLAPS 20, GEAR DOWN, 160 KT; PRESS ILS ON THE PANEL, THEN AP; \
                WATCH LOC AND G/S TURN GREEN AND TAKE OVER AT 100 FT WHEN THE PANEL SAYS AP DISCONNECT";
    vec![line.to_owned(), line.to_owned()]
}

/// Every pixel of the frame over a stand-in world (sky above the centre
/// row, land below), triangles in painter's order with `color_at`'s
/// pixel-centre rule.
fn raster(scene: &Scene) -> Vec<Rgb> {
    let (w, h) = (WIDTH as usize, HEIGHT as usize);
    let mut px: Vec<Rgb> = (0..w * h).map(|i| if i / w < h / 2 { SKY_CLEAR } else { LAND }).collect();
    let to_px = |p: [f32; 2]| [(p[0] + 1.0) * WIDTH / 2.0, (1.0 - p[1]) * HEIGHT / 2.0];
    for t in scene.vertices().chunks_exact(3) {
        let v = [to_px(t[0].pos), to_px(t[1].pos), to_px(t[2].pos)];
        let lo = |i: usize| v.iter().map(|p| p[i]).fold(f32::MAX, f32::min).floor().max(0.0) as usize;
        let hi = |i: usize, n: usize| (v.iter().map(|p| p[i]).fold(f32::MIN, f32::max).ceil() as usize).min(n);
        for y in lo(1)..hi(1, h) {
            for x in lo(0)..hi(0, w) {
                // The exact test `color_at` uses, on one triangle.
                let p = [(x as f32 + 0.5) / (WIDTH / 2.0) - 1.0, 1.0 - (y as f32 + 0.5) / (HEIGHT / 2.0)];
                let e = |a: [f32; 2], b: [f32; 2]| (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
                let d = [e(t[0].pos, t[1].pos), e(t[1].pos, t[2].pos), e(t[2].pos, t[0].pos)];
                if d.iter().all(|&d| d >= 0.0) || d.iter().all(|&d| d <= 0.0) {
                    px[y * w + x] = t[0].color;
                }
            }
        }
    }
    px
}

fn dump(name: &str, px: &[Rgb]) {
    let Ok(dir) = std::env::var("GX_PANEL_PPM_DIR") else { return };
    let mut ppm = format!("P6\n{} {}\n255\n", WIDTH as u32, HEIGHT as u32).into_bytes();
    ppm.extend(px.iter().flat_map(|c| c.map(|v| (v.clamp(0.0, 1.0) * 255.0).round() as u8)));
    let path = format!("{dir}/{name}.ppm");
    std::fs::write(&path, ppm).expect("write the PPM");
    eprintln!("wrote {path}");
}

fn rows_with(px: &[Rgb], colour: Rgb) -> Vec<usize> {
    let w = WIDTH as usize;
    let mut rows: Vec<usize> = px.chunks_exact(w).enumerate().filter(|(_, r)| r.contains(&colour)).map(|(y, _)| y).collect();
    rows.dedup();
    rows
}

#[test]
fn the_panel_frame_shows_the_strip_cursor_lit_ap_and_warning() {
    let p = prompts();
    let mut scene = Scene::new();
    scene.build_overlay(&busiest_hud(&p, true), 12.0, CAMERA);
    // Every probe is checked with the CPU reference the GPU test uses.
    assert_eq!(scene.color_at(1279.0, 719.0, [0.0; 3]), BG, "the strip reaches the corner");
    assert_eq!(scene.color_at(2.0, TOP + 1.0, [0.0; 3]), BG, "the strip starts at TOP");
    assert_eq!(scene.color_at(14.0, TOP + 12.0, [0.0; 3]), ACTIVE, "AP engaged: AP button lit");
    assert_eq!(scene.color_at(1198.0, TOP + 12.0, [0.0; 3]), ARMED, "EXIT armed: amber");
    let (cx, cy) = TEST_CURSOR;
    assert_eq!(scene.color_at(cx + 1.0, cy + 6.0, [0.0; 3]), [1.0; 3], "cursor (white fill)");
    assert_eq!(scene.color_at(cx + 13.0, cy + 13.0, [0.0; 3]), [0.0; 3], "cursor (black edge, right)");
    assert_eq!(scene.color_at(cx - 1.0, cy + 8.0, [0.0; 3]), [0.0; 3], "cursor (black edge, left)");

    let px = raster(&scene);
    dump("panel_warning", &px);
    let w = WIDTH as usize;
    let info = |c: Rgb| px[(TOP as usize) * w..].chunks_exact(w).any(|r| r[INFO_X as usize..].contains(&c));
    assert!(info(WARN), "the A/T DISCONNECT line is drawn in its lit phase");
    // The banner and prompts moved above the strip; nothing of theirs is in it.
    for (c, what) in [(BANNER, "banner"), (PROMPT_TEXT, "prompt")] {
        let rows = rows_with(&px, c);
        assert!(!rows.is_empty(), "{what} drawn");
        assert!(*rows.last().unwrap() < TOP as usize, "{what} reaches row {} (strip at {TOP})", rows.last().unwrap());
    }
    // Banner, prompt panel and the top-bar trim/config strips do not overlap.
    let banner = rows_with(&px, BANNER);
    let prompt_top = (0..TOP as usize).rev().take_while(|&y| px[y * w + 2] == PANEL).last().unwrap();
    assert!(*banner.last().unwrap() < prompt_top, "banner rows {banner:?} vs prompt panel from {prompt_top}");
    assert!(banner[0] > 128, "banner below the trim/config strips");

    // The blink's dark phase hides only the warning line.
    let mut v = busiest_hud(&p, true);
    v.panel.as_mut().unwrap().blink = false;
    scene.build_overlay(&v, 12.0, CAMERA);
    let px = raster(&scene);
    assert!(!px[(TOP as usize) * w..].chunks_exact(w).any(|r| r[INFO_X as usize..].contains(&WARN)), "warning hidden in the dark phase");
    assert_eq!(scene.color_at(14.0, TOP + 12.0, [0.0; 3]), ACTIVE);

    // The ILS line instead of a warning; the S4 HUD without a panel has no strip.
    scene.build_overlay(&busiest_hud(&p, false), 12.0, CAMERA);
    let px = raster(&scene);
    dump("panel_ils", &px);
    // G/S DES and the +1230 deviation: amber text in the status column, and
    // nothing of the column spills into the gap before EXIT.
    let column = |c: Rgb, x0: usize, x1: usize| px[(TOP as usize) * w..].chunks_exact(w).any(|r| r[x0..x1].contains(&c));
    assert!(column(INFO_ARMED, INFO_X as usize, INFO_END as usize), "G/S DES / deviation drawn");
    assert!(!column(INFO_ARMED, INFO_END as usize, 1196), "status column stays left of EXIT");
    scene.build_overlay(&test_hud(&p), 12.0, CAMERA);
    assert_ne!(scene.color_at(1279.0, 719.0, [0.0; 3]), BG);
}

#[test]
fn the_busiest_panel_frame_fits_the_vertex_buffer() {
    let p = prompts();
    let mut scene = Scene::new();
    let mut hud = busiest_hud(&p, true);
    let with = scene.build_overlay(&hud, 12.0, CAMERA).vertices as usize;
    let ils = scene.build_overlay(&busiest_hud(&p, false), 12.0, CAMERA).vertices as usize;
    hud.panel = None;
    let without = scene.build_overlay(&hud, 12.0, CAMERA).vertices as usize;
    let mut alone = Scene::new();
    panel::draw(&mut alone, &busiest_hud(&p, true).panel.unwrap());
    eprintln!(
        "busiest S4 HUD: {with} vertices with the panel (warning), {ils} (ILS line), {without} without; panel alone {} ({} triangles); MAX_VERTICES {MAX_VERTICES}",
        alone.vertices().len(),
        alone.vertices().len() / 3
    );
    assert_eq!(with - without, alone.vertices().len(), "nothing was dropped at the buffer limit");
    // Scene::push drops whole triangles at the limit, so a frame that fitted
    // has room for at least one more rectangle and still shows its cursor
    // (the last thing drawn).
    assert!(with.max(ils) + 6 <= MAX_VERTICES, "{with} / {ils}");
    scene.build_overlay(&busiest_hud(&p, true), 12.0, CAMERA);
    assert_eq!(scene.color_at(TEST_CURSOR.0 + 1.0, TEST_CURSOR.1 + 6.0, [0.0; 3]), [1.0; 3]);

    // Adversarial bound: four wrapped prompt lines of the font's densest glyph.
    let dense = vec![format!("{0} {0}", "#".repeat(99)), format!("{0} {0}", "#".repeat(99))];
    let n = scene.build_overlay(&busiest_hud(&dense, true), 12.0, CAMERA).vertices as usize;
    eprintln!("adversarial (400 '#' prompt glyphs): {n} vertices");
    assert!(n + 6 <= MAX_VERTICES, "{n}");
}
