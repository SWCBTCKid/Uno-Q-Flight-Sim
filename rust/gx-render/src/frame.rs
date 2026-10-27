//! Pure snapshot sampling for the render thread.
//!
//! Snapshot pins have already been released before this code runs. Continuous
//! fields are sampled one simulation tick behind the latest publication;
//! warnings and other safety/discrete fields always come from `latest`.

use crate::scene::{ConfigReadout, GearText};
use gx_sim::{ApStatus, InputHealth, Levers, PauseReason, SnapshotPair};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SampleKind {
    Interpolated,
    Clamped,
    LatestOnly,
    Discontinuity,
    InvalidContinuousState,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RenderSample {
    pub kind: SampleKind,
    pub snapshot_sequence: u64,
    pub generation: u64,
    pub simulation_time_ns: u64,
    pub frame_id: u64,
    pub frame_generation: u64,
    pub position_m: [f64; 3],
    pub orientation_xyzw: [f64; 4],
    pub roll_rad: f64,
    pub pitch_rad: f64,
    pub heading_rad: f64,
    pub calibrated_airspeed_mps: f64,
    pub height_agl_m: f64,
    pub vertical_speed_mps: f64,
    pub gear_position: f64,
    pub flap_position: f64,
    pub spoiler_position: f64,
    pub elevator_position: f64,
    pub aileron_position: f64,
    pub rudder_position: f64,
    pub engine_n1: f64,
    pub weight_on_wheels: bool,
    /// Pitch trim position of the latest snapshot (+ = nose up); a step
    /// value, never interpolated.
    pub pitch_trim: f64,
    /// `fcs/speedbrake-pos-norm`, interpolated like the other surfaces.
    pub speedbrake_position: f64,
    /// The pilot's gear/flap/speedbrake levers of the latest snapshot.
    pub levers: Levers,
    /// The autopilot of the latest snapshot (modes, targets, warning): a
    /// discrete state, never interpolated (AP-001).
    pub autopilot: ApStatus,
    pub input: InputHealth,
    pub pause_reason: Option<PauseReason>,
}

impl RenderSample {
    fn latest(pair: SnapshotPair, kind: SampleKind) -> Self {
        let state = pair.latest.state;
        let pose = state.local_enu.unwrap_or_default();
        Self {
            kind,
            snapshot_sequence: pair.latest.sequence,
            generation: pair.latest.generation,
            simulation_time_ns: pair.latest.simulation_time_ns,
            frame_id: pose.frame_id,
            frame_generation: pose.frame_generation,
            position_m: pose.position_m,
            orientation_xyzw: pose.orientation_xyzw,
            roll_rad: state.roll_rad,
            pitch_rad: state.pitch_rad,
            heading_rad: state.heading_rad,
            calibrated_airspeed_mps: state.calibrated_airspeed_mps,
            height_agl_m: state.height_agl_m,
            vertical_speed_mps: state.vertical_speed_mps,
            gear_position: state.gear_position,
            flap_position: state.flap_position,
            spoiler_position: state.spoiler_position,
            elevator_position: state.elevator_position,
            aileron_position: state.aileron_position,
            rudder_position: state.rudder_position,
            engine_n1: state.engine_n1,
            weight_on_wheels: state.weight_on_wheels,
            pitch_trim: state.pitch_trim,
            speedbrake_position: state.speedbrake_position,
            levers: state.levers,
            autopilot: state.autopilot,
            input: pair.latest.input,
            pause_reason: pair.latest.pause_reason,
        }
    }
}

/// Samples a pair at `render_now_ns`, delayed by `delay_ns`. There is never
/// extrapolation. The caller normally supplies one 120 Hz tick as the delay.
pub fn sample(pair: SnapshotPair, render_now_ns: u64, delay_ns: u64) -> RenderSample {
    let Some(previous) = pair.previous else {
        return RenderSample::latest(pair, SampleKind::LatestOnly);
    };
    let latest = pair.latest;
    let Some(a_pose) = previous.state.local_enu else {
        return RenderSample::latest(pair, SampleKind::Discontinuity);
    };
    let Some(b_pose) = latest.state.local_enu else {
        return RenderSample::latest(pair, SampleKind::Discontinuity);
    };
    if previous.generation != latest.generation
        || a_pose.frame_id != b_pose.frame_id
        || a_pose.frame_generation != b_pose.frame_generation
        || previous.simulation_time_ns >= latest.simulation_time_ns
    {
        return RenderSample::latest(pair, SampleKind::Discontinuity);
    }

    let elapsed = render_now_ns.saturating_sub(latest.published_at_ns);
    let target = latest.simulation_time_ns as i128 + elapsed as i128 - delay_ns as i128;
    let start = previous.simulation_time_ns as i128;
    let end = latest.simulation_time_ns as i128;
    let unclamped = (target - start) as f64 / (end - start) as f64;
    let alpha = unclamped.clamp(0.0, 1.0);
    let kind = if alpha == unclamped {
        SampleKind::Interpolated
    } else {
        SampleKind::Clamped
    };

    let lerp = |a: f64, b: f64| a + (b - a) * alpha;
    let heading_delta = (latest.state.heading_rad - previous.state.heading_rad
        + std::f64::consts::PI)
        .rem_euclid(std::f64::consts::TAU)
        - std::f64::consts::PI;
    let mut q1 = b_pose.orientation_xyzw;
    if dot4(a_pose.orientation_xyzw, q1) < 0.0 {
        for v in &mut q1 {
            *v = -*v;
        }
    }
    let q = normalize4(std::array::from_fn(|i| lerp(a_pose.orientation_xyzw[i], q1[i])));

    let continuous = [
        previous.state.roll_rad,
        previous.state.pitch_rad,
        previous.state.heading_rad,
        latest.state.roll_rad,
        latest.state.pitch_rad,
        latest.state.heading_rad,
        previous.state.calibrated_airspeed_mps,
        latest.state.calibrated_airspeed_mps,
        previous.state.height_agl_m,
        latest.state.height_agl_m,
        previous.state.vertical_speed_mps,
        latest.state.vertical_speed_mps,
        previous.state.elevator_position,
        latest.state.elevator_position,
        previous.state.aileron_position,
        latest.state.aileron_position,
        previous.state.rudder_position,
        latest.state.rudder_position,
    ];
    if q.is_none()
        || continuous.iter().any(|v| !v.is_finite())
        || a_pose.position_m.iter().chain(b_pose.position_m.iter()).any(|v| !v.is_finite())
    {
        return RenderSample::latest(pair, SampleKind::InvalidContinuousState);
    }

    RenderSample {
        kind,
        snapshot_sequence: latest.sequence,
        generation: latest.generation,
        simulation_time_ns: latest.simulation_time_ns,
        frame_id: b_pose.frame_id,
        frame_generation: b_pose.frame_generation,
        position_m: std::array::from_fn(|i| lerp(a_pose.position_m[i], b_pose.position_m[i])),
        orientation_xyzw: q.unwrap(),
        roll_rad: lerp(previous.state.roll_rad, latest.state.roll_rad),
        pitch_rad: lerp(previous.state.pitch_rad, latest.state.pitch_rad),
        heading_rad: (previous.state.heading_rad + heading_delta * alpha)
            .rem_euclid(std::f64::consts::TAU),
        calibrated_airspeed_mps: lerp(
            previous.state.calibrated_airspeed_mps,
            latest.state.calibrated_airspeed_mps,
        ),
        height_agl_m: lerp(previous.state.height_agl_m, latest.state.height_agl_m),
        vertical_speed_mps: lerp(previous.state.vertical_speed_mps, latest.state.vertical_speed_mps),
        gear_position: lerp(previous.state.gear_position, latest.state.gear_position),
        flap_position: lerp(previous.state.flap_position, latest.state.flap_position),
        spoiler_position: lerp(previous.state.spoiler_position, latest.state.spoiler_position),
        elevator_position: lerp(previous.state.elevator_position, latest.state.elevator_position),
        aileron_position: lerp(previous.state.aileron_position, latest.state.aileron_position),
        rudder_position: lerp(previous.state.rudder_position, latest.state.rudder_position),
        engine_n1: lerp(previous.state.engine_n1, latest.state.engine_n1),
        weight_on_wheels: latest.state.weight_on_wheels,
        pitch_trim: latest.state.pitch_trim,
        speedbrake_position: lerp(previous.state.speedbrake_position, latest.state.speedbrake_position),
        levers: latest.state.levers,
        autopilot: latest.state.autopilot,
        input: latest.input,
        pause_reason: latest.pause_reason,
    }
}

fn dot4(a: [f64; 4], b: [f64; 4]) -> f64 {
    a.into_iter().zip(b).map(|(a, b)| a * b).sum()
}

fn normalize4(mut q: [f64; 4]) -> Option<[f64; 4]> {
    let norm = dot4(q, q).sqrt();
    if !norm.is_finite() || norm <= f64::EPSILON {
        return None;
    }
    for v in &mut q {
        *v /= norm;
    }
    Some(q)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HudReadout {
    pub airspeed_kt: u16,
    pub altitude_agl_ft: i32,
    pub heading_deg: u16,
    pub vertical_speed_fpm: i32,
    pub engine_n1_pct: u16,
    /// Pitch trim in percent of elevator travel, + = nose up.
    pub pitch_trim_pct: i16,
    pub input_warning: bool,
    pub input_loss_tenths: u16,
}

impl From<&RenderSample> for HudReadout {
    fn from(sample: &RenderSample) -> Self {
        fn rounded(value: f64, low: i64, high: i64) -> i64 {
            if value.is_finite() {
                (value.round() as i64).clamp(low, high)
            } else {
                0
            }
        }
        Self {
            airspeed_kt: rounded(sample.calibrated_airspeed_mps / 0.514_444, 0, 999) as u16,
            altitude_agl_ft: rounded(sample.height_agl_m / 0.3048, -9_999, 99_999) as i32,
            heading_deg: rounded(sample.heading_rad.to_degrees().rem_euclid(360.0), 0, 359)
                as u16,
            vertical_speed_fpm: rounded(sample.vertical_speed_mps / 0.3048 * 60.0, -99_999, 99_999) as i32,
            engine_n1_pct: rounded(sample.engine_n1, 0, 999) as u16,
            pitch_trim_pct: rounded(sample.pitch_trim * 100.0, -100, 100) as i16,
            input_warning: sample.input.warning(),
            input_loss_tenths: (sample.input.loss_ms / 100).min(u64::from(u16::MAX)) as u16,
        }
    }
}

/// The HUD gear/flap/speedbrake readout (INPUT-005) of a sample.
/// `flap_detents` are the aircraft's (`fcs/flap-cmd-norm`, label) pairs; the
/// label shown is that of the detent nearest the flap lever.
pub fn config_readout(sample: &RenderSample, flap_detents: &[(f32, u8)]) -> ConfigReadout {
    let finite = |v: f64| if v.is_finite() { v } else { 0.0 };
    let gear_pos = finite(sample.gear_position);
    let gear = if gear_pos >= 0.99 {
        GearText::Down
    } else if gear_pos <= 0.01 {
        GearText::Up
    } else {
        GearText::Transit
    };
    // Lever and gear disagree: in transit, or the squat switch holds it down.
    let gear_caution = gear == GearText::Transit || (gear == GearText::Down) != sample.levers.gear_down;
    let lever = sample.levers.flaps;
    let flaps_label = flap_detents
        .iter()
        .min_by(|a, b| (a.0 - lever).abs().total_cmp(&(b.0 - lever).abs()))
        .map_or(0, |d| d.1);
    let flaps_moving = (finite(sample.flap_position) - f64::from(lever)).abs() > 0.005;
    let sb = finite(sample.speedbrake_position).max(f64::from(sample.levers.speedbrake));
    ConfigReadout {
        gear,
        gear_caution,
        flaps_label,
        flaps_moving,
        speedbrake_pct: (sb.clamp(0.0, 1.0) * 100.0).round() as u8,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gx_sim::{AircraftSnapshot, AircraftState, InputStatus, LocalEnuPose};

    fn snapshot(sequence: u64, sim_ns: u64, published_ns: u64, heading: f64) -> AircraftSnapshot {
        AircraftSnapshot {
            sequence,
            generation: 4,
            simulation_time_ns: sim_ns,
            published_at_ns: published_ns,
            state: AircraftState {
                local_enu: Some(LocalEnuPose {
                    frame_id: 7,
                    frame_generation: 2,
                    position_m: [sequence as f64 * 10.0, 2.0, 3.0],
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    ..Default::default()
                }),
                roll_rad: sequence as f64,
                pitch_rad: sequence as f64 * 2.0,
                heading_rad: heading,
                calibrated_airspeed_mps: 51.4444,
                height_agl_m: 304.8,
                ..Default::default()
            },
            ..Default::default()
        }
    }

    #[test]
    fn samples_midpoint_one_tick_behind_and_keeps_latest_safety_state() {
        let a = snapshot(1, 100, 1_000, 350f64.to_radians());
        let mut b = snapshot(2, 200, 1_100, 10f64.to_radians());
        b.state.vertical_speed_mps = 5.08;
        b.state.engine_n1 = 87.4;
        b.state.elevator_position = -1.0;
        b.state.aileron_position = 0.8;
        b.state.rudder_position = -0.6;
        b.input = InputHealth { status: InputStatus::Stale, loss_ms: 1_250 };
        b.state.pitch_trim = 0.124;
        b.state.autopilot = ApStatus { engaged: true, hold_ft: 3_000.0, ..ApStatus::default() };
        let out = sample(SnapshotPair { previous: Some(a), latest: b }, 1_150, 100);
        assert_eq!(out.kind, SampleKind::Interpolated);
        assert_eq!(out.autopilot, b.state.autopilot, "the latest autopilot, never a blend");
        assert_eq!(out.position_m[0], 15.0);
        assert!((out.heading_rad.to_degrees() - 0.0).abs() < 1e-9);
        assert_eq!(out.input, b.input);
        assert_eq!([out.elevator_position, out.aileron_position, out.rudder_position], [-0.5, 0.4, -0.3]);
        assert_eq!(HudReadout::from(&out), HudReadout {
            airspeed_kt: 100,
            altitude_agl_ft: 1_000,
            heading_deg: 0,
            vertical_speed_fpm: 500,
            engine_n1_pct: 44,
            pitch_trim_pct: 12,
            input_warning: true,
            input_loss_tenths: 12,
        });
    }

    const DETENTS: [(f32, u8); 3] = [(0.0, 0), (5.0 / 35.0, 5), (30.0 / 35.0, 30)];

    fn config(gear: f64, gear_lever_down: bool, flap_pos: f64, flap_lever: f32, sb_pos: f64, sb_lever: f32) -> ConfigReadout {
        let mut b = snapshot(2, 200, 1_100, 0.0);
        b.state.gear_position = gear;
        b.state.flap_position = flap_pos;
        b.state.speedbrake_position = sb_pos;
        b.state.levers = Levers { gear_down: gear_lever_down, flaps: flap_lever, speedbrake: sb_lever };
        config_readout(&sample(SnapshotPair { previous: None, latest: b }, 1_150, 100), &DETENTS)
    }

    #[test]
    fn config_readout_names_gear_flaps_and_speedbrake() {
        let c = config(1.0, true, 5.0 / 35.0, 5.0 / 35.0, 0.0, 0.0);
        assert_eq!(c, ConfigReadout { gear: GearText::Down, gear_caution: false, flaps_label: 5, flaps_moving: false, speedbrake_pct: 0 });
        assert_eq!(config(0.0, false, 0.0, 0.0, 0.0, 0.0).gear, GearText::Up);
        let transit = config(0.4, false, 0.0, 0.0, 0.0, 0.0);
        assert_eq!((transit.gear, transit.gear_caution), (GearText::Transit, true));
        // Squat switch: lever up, gear still down on the ground.
        assert!(config(1.0, false, 0.0, 0.0, 0.0, 0.0).gear_caution);
        // Flaps travelling to 30: label is the lever's, amber while moving.
        let f = config(1.0, true, 0.3, 30.0 / 35.0, 0.0, 0.0);
        assert_eq!((f.flaps_label, f.flaps_moving), (30, true));
        // Speedbrake: lever leads, position follows; the larger is shown.
        assert_eq!(config(1.0, true, 0.0, 0.0, 0.2, 0.5).speedbrake_pct, 50);
        assert_eq!(config(1.0, true, 0.0, 0.0, 1.0, 0.0).speedbrake_pct, 100);
        // Non-finite FDM values never panic or show nonsense.
        let n = config(f64::NAN, true, f64::NAN, 0.0, f64::NAN, 0.0);
        assert_eq!((n.gear, n.speedbrake_pct), (GearText::Up, 0));
    }

    #[test]
    fn reset_or_frame_discontinuity_never_blends() {
        let a = snapshot(1, 100, 1_000, 0.0);
        let mut b = snapshot(2, 200, 1_100, 1.0);
        b.generation += 1;
        b.state.autopilot.engaged = true;
        let out = sample(SnapshotPair { previous: Some(a), latest: b }, 1_150, 100);
        assert_eq!(out.kind, SampleKind::Discontinuity);
        assert!(out.autopilot.engaged, "latest-only samples carry the autopilot too");
        assert_eq!(out.position_m, [20.0, 2.0, 3.0]);
    }

    #[test]
    fn no_previous_and_nonfinite_state_fall_back_to_latest() {
        let b = snapshot(2, 200, 1_100, 1.0);
        assert_eq!(
            sample(SnapshotPair { previous: None, latest: b }, 1_150, 100).kind,
            SampleKind::LatestOnly
        );
        let mut a = snapshot(1, 100, 1_000, 0.0);
        a.state.pitch_rad = f64::NAN;
        assert_eq!(
            sample(SnapshotPair { previous: Some(a), latest: b }, 1_150, 100).kind,
            SampleKind::InvalidContinuousState
        );
        a.state.pitch_rad = 2.0;
        a.state.rudder_position = f64::NAN;
        assert_eq!(
            sample(SnapshotPair { previous: Some(a), latest: b }, 1_150, 100).kind,
            SampleKind::InvalidContinuousState
        );
    }
}
