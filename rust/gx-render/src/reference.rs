//! CPU reference for the S4 offline render test (tests/offscreen.rs on the
//! board GPU, and the host test below that proves the probe set itself is
//! meaningful). Not used at run time.
//!
//! A pixel's reference colour: the HUD over everything, then the nearest
//! aircraft triangle (the GPU depth-tests the aircraft), then the last-drawn
//! ground triangle hit by the pixel's ray (the GPU draws the ground in
//! painter's order without a depth test), else the sky clear colour. Light
//! sprites are not modelled: probes near a projected light are skipped.

use crate::aircraft::Vertex3;
use crate::camera::{CameraMode, Pose, View};
use crate::panel::PanelView;
use crate::scene::{ConfigReadout, GearText, HudInput, Rgb, Scene};
use crate::world::{ray_triangle, WorldMesh, SKY_CLEAR};
use gx_sim::autopilot::{ApWarning, Lateral, Vertical};
use gx_sim::{ApInput, ApStatus, InputHealth, InputStatus, PauseReason, RunState};

pub fn cpu_pixel(view: &View, pose: &Pose, world: &WorldMesh, model: &[Vertex3], hud: &Scene, x: f64, y: f64) -> Rgb {
    const MARK: Rgb = [-1.0, -1.0, -1.0];
    let h = hud.color_at(x as f32, y as f32, MARK);
    if h != MARK {
        return h;
    }
    let dir = view.ray(x, y);
    if view.mode == CameraMode::Chase {
        let mut best: Option<(f64, Rgb)> = None;
        for t in model.chunks_exact(3) {
            let p = [t[0].pos, t[1].pos, t[2].pos].map(|v| pose.to_world(v.map(f64::from)));
            if let Some(d) = ray_triangle(view.eye, dir, p) {
                if best.map_or(true, |b| d < b.0) {
                    best = Some((d, t[0].color));
                }
            }
        }
        if let Some((_, c)) = best {
            return c;
        }
    }
    world.ground_ray(view.eye, dir).unwrap_or(SKY_CLEAR)
}

/// Body FRD -> ENU quaternion for heading/pitch/roll, built as gx-sim's
/// `body_to_enu_quaternion` does.
pub fn quat_heading_pitch_roll(h: f64, p: f64, r: f64) -> [f64; 4] {
    let (sr, cr) = r.sin_cos();
    let (st, ct) = p.sin_cos();
    let (sy, cy) = h.sin_cos();
    let b2n = [
        [ct * cy, sr * st * cy - cr * sy, cr * st * cy + sr * sy],
        [ct * sy, sr * st * sy + cr * cy, cr * st * sy - sr * cy],
        [-st, sr * ct, cr * ct],
    ];
    let m = [b2n[1], b2n[0], [-b2n[2][0], -b2n[2][1], -b2n[2][2]]];
    let w = (1.0 + m[0][0] + m[1][1] + m[2][2]).max(0.0).sqrt() / 2.0;
    let q = if w > 1e-3 {
        [(m[2][1] - m[1][2]) / (4.0 * w), (m[0][2] - m[2][0]) / (4.0 * w), (m[1][0] - m[0][1]) / (4.0 * w), w]
    } else {
        let x = (1.0 + m[0][0] - m[1][1] - m[2][2]).max(0.0).sqrt() / 2.0;
        [x, (m[0][1] + m[1][0]) / (4.0 * x), (m[0][2] + m[2][0]) / (4.0 * x), (m[2][1] - m[1][2]) / (4.0 * x)]
    };
    let n = q.iter().map(|v| v * v).sum::<f64>().sqrt();
    q.map(|v| v / n)
}

/// The four S4 render-test views: both cameras at the threshold (settled
/// S3 pose) and both in flight (banked/pitched, over the bay and the field).
#[allow(non_snake_case)]
pub fn S4_CASES() -> Vec<(&'static str, CameraMode, Pose)> {
    let d = f64::to_radians;
    let level_east = quat_heading_pitch_roll(d(90.0), 0.0, 0.0);
    vec![
        ("chase, threshold", CameraMode::Chase, Pose { position_m: [-1_469.9755, 0.0, 2.7234], orientation_xyzw: level_east }),
        ("cockpit, threshold", CameraMode::Cockpit, Pose { position_m: [-1_469.9755, 0.0, 2.7234], orientation_xyzw: level_east }),
        ("chase, climbing right turn over the bay", CameraMode::Chase, Pose { position_m: [600.0, -300.0, 350.0], orientation_xyzw: quat_heading_pitch_roll(d(70.0), d(8.0), d(20.0)) }),
        ("cockpit, nose down over the field", CameraMode::Cockpit, Pose { position_m: [-2_500.0, 400.0, 400.0], orientation_xyzw: quat_heading_pitch_roll(d(100.0), d(-12.0), d(-10.0)) }),
    ]
}

/// The HUD the render test draws over every case.
pub fn test_hud(prompt: &[String]) -> HudInput<'_> {
    HudInput {
        roll_rad: 0.1,
        pitch_rad: 0.05,
        heading_rad: 1.5,
        airspeed_kt: 142,
        altitude_agl_ft: 1234,
        heading_deg: 90,
        vertical_speed_fpm: 850,
        engine_n1_pct: 87,
        pitch_trim_pct: None,
        config: None,
        run: RunState::Running,
        pause: None,
        input: InputHealth { status: InputStatus::Valid, loss_ms: 0 },
        prompt,
        panel: None,
    }
}

/// The autopilot panel the render tests draw: AP, LOC and A/T engaged with
/// the aircraft descending onto the glideslope from 1,230 ft above it
/// (`G/S DES`, `28R LOC G/S +1230`), the widest targets, EXIT armed
/// (`EXIT?`), the cursor over the HDG `+` button and, with `warning`, the
/// longest warning (INPUT LOSS - LVL) in its lit blink phase instead of the
/// ILS line.
pub fn test_panel(warning: bool) -> PanelView {
    let status = ApStatus {
        engaged: true,
        lateral: Lateral::Localizer,
        vertical: Vertical::GsDescent,
        gs_armed: true,
        gs_dev_ft: Some(1_234.0),
        approach: Some(0),
        autothrottle: true,
        hold_ft: 45_000.0,
        input: ApInput { heading_deg: 358.0, altitude_ft: 45_000.0, speed_kt: 350.0, ..ApInput::DEFAULT },
        warning: warning.then_some(ApWarning::InputLossLevel),
        ..ApStatus::default()
    };
    PanelView { status, approach: Some("28R".into()), cursor: Some(TEST_CURSOR), mouse: true, blink: true, exit_armed: true }
}

/// Where [`test_panel`] puts the cursor tip (over the HDG `+` button).
pub const TEST_CURSOR: (f32, f32) = (356.0, 676.0);

/// The busiest HUD a frame can draw: the longest centre overlay and the
/// no-input banner together, the trim and config strips (speedbrake out),
/// the widest readouts, the caller's prompts and [`test_panel`].
pub fn busiest_hud(prompt: &[String], warning: bool) -> HudInput<'_> {
    HudInput {
        pitch_trim_pct: Some(-100),
        config: Some(ConfigReadout { gear: GearText::Transit, gear_caution: true, flaps_label: 30, flaps_moving: true, speedbrake_pct: 100 }),
        run: RunState::Paused,
        pause: Some(PauseReason::InvalidWorldPose),
        input: InputHealth { status: InputStatus::NoInput, loss_ms: 0 },
        altitude_agl_ft: -9_999,
        vertical_speed_fpm: -99_999,
        panel: Some(test_panel(warning)),
        ..test_hud(prompt)
    }
}

/// A 20 x 12 probe grid; a probe is kept only where the reference is uniform
/// over a +/-2 px cross (not on an edge) and no light sprite is within 7 px.
pub fn probe_set(view: &View, pose: &Pose, world: &WorldMesh, model: &[Vertex3], hud: &Scene) -> (Vec<(u32, u32)>, Vec<Rgb>) {
    let lights: Vec<[f64; 2]> = world.lights.iter().filter_map(|l| view.project(l.pos.map(f64::from))).collect();
    let mut probes = Vec::new();
    let mut want = Vec::new();
    for gy in 0..12 {
        for gx in 0..20 {
            let (x, y) = (32.0 + 61.0 * gx as f64, 24.0 + 58.0 * gy as f64);
            let c = cpu_pixel(view, pose, world, model, hud, x, y);
            let uniform = [(2.0, 0.0), (-2.0, 0.0), (0.0, 2.0), (0.0, -2.0)]
                .iter()
                .all(|(dx, dy)| cpu_pixel(view, pose, world, model, hud, x + dx, y + dy) == c);
            let near_light = lights.iter().any(|l| (l[0] - x).abs() < 7.0 && (l[1] - y).abs() < 7.0);
            if uniform && !near_light {
                probes.push((x as u32, y as u32));
                want.push(c);
            }
        }
    }
    (probes, want)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aircraft::global5000;
    use crate::world::{LAND, RUNWAY, WATER};

    /// The probe set the board test compares is large and covers every
    /// scene element (so a GPU pass that drew nothing, the wrong camera or
    /// the wrong layer order would fail there).
    #[test]
    fn probe_sets_cover_sky_land_water_runway_and_aircraft() {
        let world = WorldMesh::build(&gx_world::synthetic_sfo_like()).unwrap();
        let model = global5000();
        let prompt = vec!["PHASE TEST: offscreen S4 world".to_owned()];
        let mut scene = Scene::new();
        let mut seen = Vec::new();
        for (name, mode, pose) in S4_CASES() {
            let view = View::new(mode, &pose);
            scene.build_overlay(&test_hud(&prompt), view.px_per_deg(), mode.label());
            let (probes, want) = probe_set(&view, &pose, &world, &model, &scene);
            assert!(probes.len() >= 120, "{name}: {} probes", probes.len());
            seen.extend(want);
        }
        for (c, what) in [(SKY_CLEAR, "sky"), (LAND, "land"), (WATER, "water"), (RUNWAY, "runway")] {
            assert!(seen.contains(&c), "never probes {what}");
        }
        assert!(seen.iter().any(|c| model.iter().any(|v| v.color == *c)), "never probes the aircraft");
    }
}
