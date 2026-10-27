use std::cell::{Cell, RefCell};
use std::sync::Arc;

use gx_app::{
    button, command_from_frame, controls_from_frame, validate_world_pack, AppError, FakeBackend,
    OfflineApp, SimLoop, SnapshotSink, WorldFrame, FAKE_GEAR_HEIGHT_M, SIM_TICKS_PER_RENDER,
};
use gx_input::ControlFrame;
use gx_sim::{
    AircraftSnapshot, AircraftState, Command, ControlInput, PauseReason, PublishBusy, RunState,
    SimulationBackend, STEP_SECONDS,
};
use gx_world::{synthetic_sfo_like, write_world, FormatError, Limits};

fn world_frame() -> WorldFrame {
    let mut bytes = Vec::new();
    write_world(&synthetic_sfo_like(), Limits::default(), &mut bytes).unwrap();
    WorldFrame::new(Arc::new(validate_world_pack(&bytes).unwrap()))
}

fn fake() -> FakeBackend {
    FakeBackend::new(world_frame())
}

fn frame(pressed: u64) -> ControlFrame {
    ControlFrame {
        sequence: 7,
        roll: -0.75,
        pitch: 0.5,
        yaw: -0.25,
        throttle: 0.8,
        brake_left: 0.2,
        brake_right: 0.3,
        buttons_pressed: pressed,
        ..ControlFrame::default()
    }
}

fn frame_with(pressed: u64, yaw: f32, throttle: f32) -> ControlFrame {
    ControlFrame { yaw, throttle, buttons_pressed: pressed, ..ControlFrame::default() }
}

#[test]
fn maps_input_conventions_without_sign_or_range_changes() {
    let controls = controls_from_frame(&frame(0));
    assert_eq!(controls.roll, -0.75);
    assert_eq!(controls.pitch, 0.5);
    assert_eq!(controls.yaw, -0.25);
    assert_eq!(controls.throttle, 0.8);
    assert_eq!(controls.left_brake, 0.2);
    assert_eq!(controls.right_brake, 0.3);
}

#[test]
fn base_profile_leaves_hat_and_non_command_buttons_unmapped() {
    let frame = ControlFrame {
        hat_x: 1.0,
        hat_y: -1.0,
        buttons_pressed: 1 << 4,
        ..ControlFrame::default()
    };
    let controls = controls_from_frame(&frame);
    assert_eq!((controls.left_brake, controls.right_brake), (0.0, 0.0));
    assert_eq!(command_from_frame(&frame), None);
}

#[test]
fn generates_start_pause_and_reset_with_safety_precedence() {
    assert_eq!(
        command_from_frame(&frame(1 << button::START)),
        Some(Command::Start)
    );
    assert_eq!(
        command_from_frame(&frame(1 << button::PAUSE)),
        Some(Command::Pause)
    );
    assert_eq!(
        command_from_frame(&frame(1 << button::RESET)),
        Some(Command::Reset)
    );
    assert_eq!(
        command_from_frame(&frame((1 << button::START) | (1 << button::RESET))),
        Some(Command::Reset)
    );
    assert_eq!(command_from_frame(&frame(0)), None);
}

#[test]
fn advances_four_sim_ticks_per_render_and_publishes_interpolatable_pair() {
    assert_eq!(SIM_TICKS_PER_RENDER, 4);
    let mut app = OfflineApp::new(fake());
    app.ingest_controls(&frame(1 << button::START), 0).unwrap();
    let pair = app
        .advance_render_sample([8_333_333, 16_666_666, 25_000_000, 33_333_333])
        .unwrap()
        .unwrap();
    assert_eq!(app.simulation_tick(), 4);
    assert_eq!(pair.latest.simulation_tick, 4);
    assert_eq!(pair.previous.unwrap().simulation_tick, 3);
    assert_eq!(pair.latest.generation, pair.previous.unwrap().generation);
}

#[test]
fn pause_stops_ticks_and_reset_breaks_interpolation_generation() {
    let mut app = OfflineApp::new(fake());
    app.ingest_controls(&frame(1 << button::START), 0).unwrap();
    app.advance_render_sample([1, 2, 3, 4]).unwrap();
    let paused = app
        .ingest_controls(&frame(1 << button::PAUSE), 5)
        .unwrap()
        .expect("Pause publishes its visible state immediately");
    assert_eq!(paused.pause_reason, Some(PauseReason::User));
    let paused_pair = app.advance_render_sample([6, 7, 8, 9]).unwrap().unwrap();
    assert_eq!(paused_pair.latest, paused);
    assert_eq!(app.simulation_tick(), 4);
    assert_eq!(app.run_state(), RunState::Paused);

    let reset = app
        .ingest_controls(&frame(1 << button::RESET), 10)
        .unwrap()
        .unwrap();
    assert_eq!(reset.generation, 1);
    assert_eq!(reset.simulation_tick, 0);
    let pair = app
        .advance_render_sample([11, 12, 13, 14])
        .unwrap()
        .unwrap();
    assert_eq!(pair.latest, reset);
    assert!(pair.previous.is_none(), "must not interpolate across reset");
}

#[test]
fn accepts_valid_world_and_rejects_corrupt_world() {
    let mut bytes = Vec::new();
    write_world(&synthetic_sfo_like(), Limits::default(), &mut bytes).unwrap();
    assert_eq!(validate_world_pack(&bytes).unwrap().chunks.len(), 1);
    let last = bytes.len() - 1;
    bytes[last] ^= 0x80;
    assert!(matches!(
        validate_world_pack(&bytes),
        Err(FormatError::HashMismatch { .. })
    ));
}

fn same_orientation(a: [f64; 4], b: [f64; 4]) -> bool {
    let dot: f64 = (0..4).map(|i| a[i] * b[i]).sum();
    (dot.abs() - 1.0).abs() < 1e-9
}

#[test]
fn snapshot_pose_is_consistent_with_world_origin() {
    let frame = world_frame();
    let origin = frame.world().origin;
    let mut app = OfflineApp::new(FakeBackend::new(frame.clone()));

    let reset = app
        .ingest_controls(&frame_with(1 << button::RESET, 0.0, 0.0), 0)
        .unwrap()
        .unwrap();
    let g = reset.state.geodetic;
    assert_eq!(
        (g.latitude_deg, g.longitude_deg),
        (origin.latitude_deg, origin.longitude_deg)
    );
    assert_eq!(g.ellipsoid_height_m, origin.height_m + FAKE_GEAR_HEIGHT_M);
    assert_eq!(reset.state.height_agl_m, FAKE_GEAR_HEIGHT_M);
    let pose = reset.state.local_enu.unwrap();
    assert_eq!(
        (pose.frame_id, pose.frame_generation),
        (frame.frame_id(), frame.frame_generation())
    );
    assert_ne!(pose.frame_id, 0);
    assert!(pose.position_m[0].abs() < 1e-6 && pose.position_m[1].abs() < 1e-6);
    assert!((pose.position_m[2] - FAKE_GEAR_HEIGHT_M).abs() < 1e-6);

    // Full throttle, heading north, for one second: about 100 m north.
    app.ingest_controls(&frame_with(1 << button::START, 0.0, 1.0), 1)
        .unwrap();
    let mut last = None;
    for render in 0..30 {
        let t = render * 4 + 2;
        last = app.advance_render_sample([t, t + 1, t + 2, t + 3]).unwrap();
    }
    let latest = last.unwrap().latest;
    assert_eq!(latest.simulation_tick, 120);
    let pose = latest.state.local_enu.unwrap();
    assert_eq!(pose.position_m, frame.enu_m(latest.state.geodetic));
    assert!(pose.position_m[0].abs() < 1e-3, "east {}", pose.position_m[0]);
    assert!(
        (99.0..101.0).contains(&pose.position_m[1]),
        "north {}",
        pose.position_m[1]
    );
    assert!(
        (pose.position_m[2] - FAKE_GEAR_HEIGHT_M).abs() < 0.01,
        "up {}",
        pose.position_m[2]
    );
    assert_eq!(pose.velocity_mps, [0.0, 100.0, 0.0]);

    // Cross-check against gx-sim's MSL georeference built from the pack's
    // frame and separation: feed it the MSL height an FDM would report.
    let sim_frame = frame.sim_frame();
    let s = latest.state;
    let (sim_geo, sim_pose) = sim_frame
        .project_msl(
            s.geodetic.latitude_deg,
            s.geodetic.longitude_deg,
            s.geodetic.ellipsoid_height_m - frame.msl_to_ellipsoid_m(),
            [pose.velocity_mps[1], pose.velocity_mps[0], -pose.velocity_mps[2]],
            [s.roll_rad, s.pitch_rad, s.heading_rad],
        )
        .unwrap();
    for axis in 0..3 {
        assert!((sim_pose.position_m[axis] - pose.position_m[axis]).abs() < 1e-6);
    }
    assert!((sim_geo.ellipsoid_height_m - s.geodetic.ellipsoid_height_m).abs() < 1e-9);
    assert_eq!((sim_pose.frame_id, sim_pose.frame_generation), (pose.frame_id, pose.frame_generation));
    assert_eq!(sim_pose.velocity_mps, pose.velocity_mps);
    assert!(same_orientation(sim_pose.orientation_xyzw, pose.orientation_xyzw));
}

#[test]
fn pose_matches_gx_sim_georeference_for_nonzero_attitude_and_offset() {
    let frame = world_frame();
    let mut backend = FakeBackend::new(frame.clone());
    let controls = ControlInput {
        roll: 0.4,
        pitch: -0.3,
        yaw: 0.9,
        throttle: 0.5,
        ..Default::default()
    };
    let mut state = backend.reset().unwrap();
    for _ in 0..240 {
        state = backend.step(controls, STEP_SECONDS).unwrap();
    }
    let pose = state.local_enu.unwrap();
    assert!(pose.position_m[0] > 10.0 && pose.position_m[1] > 10.0);
    let sim_frame = frame.sim_frame();
    let (_, sim_pose) = sim_frame
        .project_msl(
            state.geodetic.latitude_deg,
            state.geodetic.longitude_deg,
            state.geodetic.ellipsoid_height_m - frame.msl_to_ellipsoid_m(),
            [0.0; 3],
            [state.roll_rad, state.pitch_rad, state.heading_rad],
        )
        .unwrap();
    assert!(same_orientation(sim_pose.orientation_xyzw, pose.orientation_xyzw));
    for axis in 0..3 {
        assert!((sim_pose.position_m[axis] - pose.position_m[axis]).abs() < 1e-6);
    }
}

/// Wraps the fake and corrupts the pose on a chosen step or on reset.
struct CorruptPose {
    inner: FakeBackend,
    steps: u64,
    corrupt_on_step: u64,
    corrupt_reset: bool,
}

impl SimulationBackend for CorruptPose {
    type Error = std::convert::Infallible;

    fn reset(&mut self) -> Result<AircraftState, Self::Error> {
        let mut state = self.inner.reset()?;
        if self.corrupt_reset {
            state.local_enu = None;
        }
        Ok(state)
    }

    fn step(&mut self, c: ControlInput, dt: f64) -> Result<AircraftState, Self::Error> {
        let mut state = self.inner.step(c, dt)?;
        self.steps += 1;
        if self.steps == self.corrupt_on_step {
            state.local_enu.as_mut().unwrap().position_m[1] = f64::NAN;
        }
        Ok(state)
    }
}

#[test]
fn invalid_world_pose_is_rejected_and_never_published() {
    let mut app = OfflineApp::new(CorruptPose {
        inner: fake(),
        steps: 0,
        corrupt_on_step: 3,
        corrupt_reset: false,
    });
    app.ingest_controls(&frame(1 << button::START), 0).unwrap();
    let error = app.advance_render_sample([1, 2, 3, 4]).unwrap_err();
    assert_eq!(error, AppError::InvalidWorldPose);
    assert_eq!(app.run_state(), RunState::Paused);
    assert_eq!(app.simulation_tick(), 2);
    let pair = app.exchange().try_latest_pair().unwrap();
    assert_eq!(pair.latest.simulation_tick, 2, "invalid tick-3 state must not publish");

    let mut app = OfflineApp::new(CorruptPose {
        inner: fake(),
        steps: 0,
        corrupt_on_step: 0,
        corrupt_reset: true,
    });
    assert_eq!(
        app.ingest_controls(&frame(1 << button::RESET), 0).unwrap_err(),
        AppError::InvalidWorldPose
    );
    assert!(app.exchange().try_latest_pair().is_none());
    assert_eq!(app.publish_stats().offered, 0);
}

/// Reports busy for the next `busy_left` attempts, then accepts.
#[derive(Default)]
struct BusySink {
    busy_left: Cell<u32>,
    accepted: RefCell<Vec<u64>>,
}

impl SnapshotSink for BusySink {
    fn try_publish(&self, snapshot: AircraftSnapshot) -> Result<(), PublishBusy> {
        if self.busy_left.get() > 0 {
            self.busy_left.set(self.busy_left.get() - 1);
            return Err(PublishBusy(snapshot));
        }
        self.accepted.borrow_mut().push(snapshot.sequence);
        Ok(())
    }
}

#[test]
fn publish_busy_is_retried_counted_and_superseded_by_newer() {
    let sink = Arc::new(BusySink::default());
    let mut sim = SimLoop::new(fake(), Arc::clone(&sink));
    sim.ingest_controls(&frame(1 << button::START), 0).unwrap();
    sim.tick(1).unwrap(); // seq 1 published
    sink.busy_left.set(2);
    sim.tick(2).unwrap(); // seq 2 busy -> pending
    let stats = sim.publish_stats();
    assert!(stats.pending);
    assert_eq!(stats.busy_events, 1);
    sim.tick(3).unwrap(); // seq 3 supersedes 2, busy again
    sim.tick(4).unwrap(); // seq 4 supersedes 3, accepted
    let stats = sim.publish_stats();
    assert_eq!(*sink.accepted.borrow(), vec![1, 4]);
    assert_eq!(
        (stats.offered, stats.published, stats.superseded, stats.busy_events, stats.pending),
        (4, 2, 2, 2, false)
    );
    assert_eq!(stats.max_consecutive_busy, 2);

    // While paused nothing new is produced; the pending reset snapshot is
    // retried at every tick boundary until it gets through.
    sim.ingest_controls(&frame(1 << button::PAUSE), 5).unwrap();
    sink.busy_left.set(3);
    let reset = sim
        .ingest_controls(&frame(1 << button::RESET), 6)
        .unwrap()
        .unwrap();
    assert!(sim.publish_stats().pending);
    for now in 7..9 {
        assert_eq!(sim.tick(now).unwrap(), None);
        assert!(sim.publish_stats().pending);
    }
    sim.tick(9).unwrap();
    let stats = sim.publish_stats();
    assert!(!stats.pending);
    assert_eq!(sink.accepted.borrow().last(), Some(&reset.sequence));
    assert_eq!(stats.offered, stats.published + stats.superseded);
    assert_eq!(stats.busy_events, 5);
    assert_eq!(stats.max_consecutive_busy, 3);
}

#[test]
fn unpinned_exchange_accepts_every_snapshot_first_time() {
    let mut app = OfflineApp::new(fake());
    app.ingest_controls(&frame(1 << button::START), 0).unwrap();
    for i in 0..10 {
        let t = i * 4 + 1;
        app.advance_render_sample([t, t + 1, t + 2, t + 3]).unwrap();
    }
    let stats = app.publish_stats();
    assert_eq!(
        (stats.offered, stats.published, stats.busy_events, stats.pending),
        (40, 40, 0, false)
    );
}

#[test]
fn world_frame_uses_pack_frame_identity_and_separation() {
    let frame = world_frame();
    let geo = frame.world().georeference;
    assert_eq!((frame.frame_id(), frame.frame_generation()), (geo.frame_id, geo.frame_generation));
    assert_eq!(frame.msl_to_ellipsoid_m(), gx_world::SFO_GEOID_SEPARATION_EGM2008_M);
    let sim = frame.sim_frame();
    assert_eq!((sim.frame_id(), sim.frame_generation()), (geo.frame_id, geo.frame_generation));
    assert_eq!(sim.origin(), frame.origin());

    // Same geometry, new generation in the pack => new pose tag.
    let mut world = frame.world().clone();
    world.georeference.frame_generation += 1;
    let bumped = WorldFrame::new(Arc::new(world));
    let pose = bumped.pose(frame.origin(), [0.0; 3], [0.0; 3]);
    assert_eq!(pose.frame_generation, geo.frame_generation + 1);
}

/// An FDM reporting the runway's MSL elevation at the origin must put the
/// aircraft on the runway surface in the pack's ENU frame.
#[test]
fn msl_runway_elevation_projects_onto_runway_through_sim_frame() {
    let frame = world_frame();
    let runway_z = frame.world().chunks[0].lods[0]
        .layers
        .iter()
        .find_map(|layer| match layer {
            gx_world::Layer::Runway(named) => Some(f64::from(named.mesh.positions[0][2])),
            _ => None,
        })
        .unwrap();
    let o = frame.origin();
    let (_, pose) = frame
        .sim_frame()
        .project_msl(o.latitude_deg, o.longitude_deg, gx_world::SFO_FIELD_ELEVATION_MSL_M, [0.0; 3], [0.0; 3])
        .unwrap();
    assert!((pose.position_m[2] - runway_z).abs() < 0.5, "up {}", pose.position_m[2]);
    assert!(pose.position_m[0].abs() < 1e-6 && pose.position_m[1].abs() < 1e-6);
}

/// Both pose paths agree with a hand-computed WGS84 reference: from
/// (0N, 0E, 0 m) the point (0N, 90E, 0 m) is at ENU (a, 0, -a).
#[test]
fn world_and_sim_frames_match_hand_computed_reference() {
    let mut world = synthetic_sfo_like();
    world.origin = gx_world::GeodeticPoint { latitude_deg: 0.0, longitude_deg: 0.0, height_m: 0.0 };
    let frame = WorldFrame::new(Arc::new(world));
    let a = 6_378_137.0;
    let expected = [a, 0.0, -a];
    let point = gx_sim::GeodeticPosition { longitude_deg: 90.0, ..Default::default() };
    let world_enu = frame.enu_m(point);
    let (_, sim_pose) = frame
        .sim_frame()
        .project_msl(0.0, 90.0, -frame.msl_to_ellipsoid_m(), [0.0; 3], [0.0; 3])
        .unwrap();
    for axis in 0..3 {
        assert!((world_enu[axis] - expected[axis]).abs() < 1e-6, "{world_enu:?}");
        assert!((sim_pose.position_m[axis] - expected[axis]).abs() < 1e-6, "{:?}", sim_pose.position_m);
    }
}
