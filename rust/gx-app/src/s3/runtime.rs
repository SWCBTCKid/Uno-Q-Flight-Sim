//! The S3 render thread: the S2 layout's 30 Hz snapshot consumer, which
//! exclusively owns the display (DRM/KMS + GBM + EGL/GLES, ARCH-004).
//!
//! Per frame: wake (vblank-locked on KMS, 30 Hz timer otherwise), copy the
//! latest snapshot pair (`try_latest_pair` releases every reader pin before
//! it returns, ISS-034), read the sim's current run/pause/input state from
//! the board atomics, interpolate one tick behind (`gx_render::frame`),
//! build the scene, present it, log one `frame` record. Nothing here can
//! block the simulation: the exchange is lock-free, the board is relaxed
//! atomics, the prompts are `try_lock`ed and every log send is a bounded
//! `try_send`. The thread is `SCHED_OTHER` nice 0 on CPU 3 (the S1/S2
//! reader's policy): it never competes with the FIFO sim and I/O threads.
//!
//! S4 ([`Look::World`]) keeps all of that and adds, on this same thread:
//! the world pack turned into static GPU buffers once at start, a chase or
//! cockpit camera per frame (numbered button 5 toggles; the I/O thread only
//! counts presses), the 3D pass before the HUD, the on-runway check
//! (`ground_check` records) and per-frame camera/pose fields in the log.

use crate::s1::rec::{ReadRec, Rec};
use crate::s1::rt::{self, Clock};
use crate::s1::runtime::{Board, ReaderTotals, READER_HZ};
use crate::s2::runtime::SnapshotCtx;
use gx_render::frame::{sample, HudReadout, RenderSample, SampleKind};
use gx_render::pacing::Samples;
use gx_render::camera::{CameraMode, Pose, View};
use gx_render::panel::PanelView;
use gx_render::scene::{Banner, Drawn, HudInput, Overlay, Scene, Vertex};
use gx_render::world::{ground_check, GroundCheck, WorldBuffers, WorldDraw, WorldMesh, ON_RUNWAY_LIMIT_M, SKY_CLEAR};
use gx_render::photo::{PhotoDraw, PhotoMemory};
use gx_sim::{FixedRate, InputHealth, RunState};
use gx_world::WorldAsset;
use std::fmt::Write as _;
use std::io;
use std::sync::atomic::Ordering;
use std::sync::Arc;

/// What the render thread draws.
#[derive(Clone)]
pub enum Look {
    /// S3: full-screen artificial horizon and the HUD.
    Horizon,
    /// S4: the world pack, the placeholder Global 5000, chase/cockpit
    /// cameras, the HUD as an overlay.
    World(Arc<WorldAsset>),
    /// SPIKE-002 treatment: the same world pack (surfaces, G0 check, lights)
    /// plus its frame-matched photoreal visual sidecar.
    Photo(Arc<WorldAsset>, Arc<PhotoAsset>),
}

/// A GXVISUAL sidecar read and validated before the threads start.
pub struct PhotoAsset {
    pub visual: gx_world::VisualAsset,
    pub path: String,
    pub file_bytes: u64,
    /// Monotonic ns spent reading + validating the file (before the epoch).
    pub read_ns: u64,
}

/// Light point size numerator (pixels x metres): 7 px within ~300 m, 2 px
/// beyond 1 km.
pub const POINT_K: f32 = 2_000.0;
/// The "settled" on-runway check runs this long after Start, if still on
/// the wheels.
pub const SETTLED_CHECK_NS: u64 = 3_000_000_000;

pub const RENDER_CPU: usize = 3;
pub const INTERPOLATION_DELAY_NS: u64 = 1_000_000_000 / gx_sim::SIM_HZ as u64;
/// Frames kept for percentiles (20 min at 30 fps); counts and maxima are exact.
const STATS_CAP: usize = 36_000;

/// One presented frame, as the presenter reports it (monotonic ns).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Presented {
    pub token: u64,
    pub commit_ns: u64,
    /// Trustworthy (fresh) kernel flip stamp.
    pub flip_ns: Option<u64>,
    /// Vblank the frame went out on (estimated for a stale stamp).
    pub vblank_ns: u64,
    pub status: &'static str,
    pub interval_ns: Option<u64>,
    pub gpu_ns: Option<u64>,
    pub gpu_status: &'static str,
    pub draw_start_ns: u64,
    pub swap_end_ns: u64,
    pub fence_ns: u64,
    pub foreign_events: u32,
    pub draws: u32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Counts {
    pub presented: u64,
    pub stale_stamps: u64,
    pub errors: u64,
}

/// The display owned by the render thread.
pub trait Presenter {
    /// Draw and present one frame; blocks (bounded) until it is on screen.
    fn present(&mut self, verts: &[Vertex]) -> Result<Presented, String>;
    /// Monotonic time to sample the next frame (`None`: the 30 Hz timer).
    fn next_wake_ns(&self) -> Option<u64>;
    /// JSON object members describing the target (no braces).
    fn info_json(&self) -> String;
    fn counts(&self) -> Counts;
    /// Release the display (KMS: restore the saved CRTC, drop master).
    fn shutdown(self) -> Result<String, String>;
    /// S4: upload the static world once; returns the bytes uploaded.
    fn load_world(&mut self, _w: &WorldBuffers<'_>) -> Result<u64, String> {
        Err("this presenter has no world pass".into())
    }
    /// S4: the world pass, then the HUD `verts`; as [`Presenter::present`].
    fn present_world(&mut self, _draw: &WorldDraw, verts: &[Vertex]) -> Result<Presented, String> {
        self.present(verts)
    }
    /// SPIKE-002: upload the visual sidecar once (after `load_world`).
    fn load_photo(&mut self, _w: &WorldAsset, _v: &gx_world::VisualAsset) -> Result<PhotoMemory, String> {
        Err("this presenter has no photo pass".into())
    }
    /// SPIKE-002: the photo pass, then the HUD `verts`.
    fn present_photo(&mut self, draw: &PhotoDraw, verts: &[Vertex]) -> Result<Presented, String> {
        self.present_world(&draw.world, verts)
    }
    /// SPIKE-002 capture (offscreen only): as `present_world`/`present_photo`
    /// plus the whole frame as top-down RGB; `None` where unsupported.
    fn present_capture(&mut self, world: &WorldDraw, photo: Option<&PhotoDraw>, verts: &[Vertex]) -> Result<(Presented, Option<Vec<u8>>), String> {
        match photo {
            Some(d) => self.present_photo(d, verts),
            None => self.present_world(world, verts),
        }
        .map(|p| (p, None))
    }
}

/// Host/test presenter: nothing is drawn; the frame "flips" when presented.
pub struct FakePresenter<C: Clock> {
    clock: C,
    token: u64,
    last: Option<u64>,
}

impl<C: Clock> FakePresenter<C> {
    pub fn new(clock: C) -> Self {
        Self { clock, token: 0, last: None }
    }
}

impl<C: Clock> Presenter for FakePresenter<C> {
    fn present(&mut self, verts: &[Vertex]) -> Result<Presented, String> {
        if verts.is_empty() {
            return Err("empty scene".into());
        }
        self.token += 1;
        let now = self.clock.now_ns();
        let interval_ns = self.last.map(|l| now.saturating_sub(l));
        self.last = Some(now);
        Ok(Presented {
            token: self.token,
            commit_ns: now,
            flip_ns: Some(now),
            vblank_ns: now,
            status: "ok",
            interval_ns,
            gpu_ns: None,
            gpu_status: "unavailable",
            draw_start_ns: now,
            swap_end_ns: now,
            fence_ns: now,
            foreign_events: 0,
            draws: 1,
        })
    }

    fn next_wake_ns(&self) -> Option<u64> {
        None
    }

    fn info_json(&self) -> String {
        "\"target\":\"fake\"".into()
    }

    fn load_world(&mut self, w: &WorldBuffers<'_>) -> Result<u64, String> {
        if w.ground.is_empty() || w.aircraft.is_empty() {
            return Err("empty world".into());
        }
        Ok((std::mem::size_of_val(w.ground) + std::mem::size_of_val(w.lights) + std::mem::size_of_val(w.aircraft)) as u64)
    }

    fn present_world(&mut self, draw: &WorldDraw, verts: &[Vertex]) -> Result<Presented, String> {
        if draw.ground_mvp.iter().any(|v| !v.is_finite()) {
            return Err("non-finite ground MVP".into());
        }
        let mut p = self.present(verts)?;
        p.draws = 3 + u32::from(draw.aircraft_mvp.is_some());
        Ok(p)
    }

    fn load_photo(&mut self, w: &WorldAsset, v: &gx_world::VisualAsset) -> Result<PhotoMemory, String> {
        gx_render::photo::prepare_photo(w, v, gx_render::photo::ASTC_EXTENSION, gx_world::VisualLimits::default()).map(|p| p.memory)
    }

    fn present_photo(&mut self, draw: &PhotoDraw, verts: &[Vertex]) -> Result<Presented, String> {
        let mut p = self.present_world(&draw.world, verts)?;
        p.draws = 7 - u32::from(draw.world.aircraft_mvp.is_none());
        Ok(p)
    }

    fn counts(&self) -> Counts {
        Counts { presented: self.token, ..Counts::default() }
    }

    fn shutdown(self) -> Result<String, String> {
        Ok("fake".into())
    }
}

#[cfg(target_os = "linux")]
mod native {
    use super::{Counts, Presented, Presenter};
    use crate::s1::rec::esc;
    use gx_render::linux::{DisplayInfo, FrameReport, KmsDisplay, Offscreen};
    use gx_render::photo::{PhotoDraw, PhotoMemory};
    use gx_render::scene::Vertex;
    use gx_render::world::{WorldBuffers, WorldDraw};
    use gx_world::{VisualAsset, WorldAsset};

    fn presented(f: FrameReport) -> Presented {
        Presented {
            token: f.token,
            commit_ns: f.commit_ns,
            flip_ns: f.flip_ns,
            vblank_ns: f.vblank_ns,
            status: f.status,
            interval_ns: f.interval_ns,
            gpu_ns: f.gpu.ns,
            gpu_status: f.gpu.status,
            draw_start_ns: f.timing.draw_start_ns,
            swap_end_ns: f.timing.swap_end_ns,
            fence_ns: f.timing.fence_ns,
            foreign_events: f.foreign_events,
            draws: f.draws,
        }
    }

    pub fn info_json(i: &DisplayInfo, modeset_path: &str) -> String {
        let g = &i.gl;
        format!(
            "\"target\":\"{}\",\"drm_master\":{},\"connector_id\":{},\"crtc_id\":{},\"plane_id\":{},\"mode\":{{\"name\":{},\"hdisplay\":{},\"vdisplay\":{},\"clock_khz\":{},\"htotal\":{},\"vtotal\":{},\"vrefresh\":{}}},\
\"period_ns\":{},\"vblanks_per_frame\":{},\"saved_crtc\":{{\"buffer_id\":{},\"mode_valid\":{}}},\"modeset\":\"{}\",\
\"renderer\":{},\"gl_version\":{},\"depth_bits\":{},\"stencil_bits\":{},\"frame_start_clear\":\"{}\",\"frame_end_invalidate\":{},\"draws_per_frame\":{},\"render_targets\":1,\
\"gpu_timer\":{},\"gpu_timer_reason\":{}",
            i.target,
            i.drm_master,
            i.connector_id,
            i.crtc_id,
            i.plane_id,
            esc(&i.mode_name),
            i.width,
            i.height,
            i.clock_khz,
            i.htotal,
            i.vtotal,
            i.vrefresh,
            i.period_ns,
            i.vblanks_per_frame,
            i.saved_crtc_buffer,
            i.saved_crtc_mode_valid,
            modeset_path,
            esc(&g.renderer),
            esc(&g.version),
            g.depth_bits,
            g.stencil_bits,
            g.clear_mask,
            esc(g.invalidate),
            if g.depth_bits > 0 { "\"<=4 (ground, aircraft, lights, HUD)\"" } else { "1" },
            g.timer,
            esc(&g.timer_reason),
        )
    }

    impl Presenter for KmsDisplay {
        fn present(&mut self, verts: &[Vertex]) -> Result<Presented, String> {
            KmsDisplay::present(self, verts).map(presented).map_err(|e| e.to_string())
        }
        fn next_wake_ns(&self) -> Option<u64> {
            KmsDisplay::next_wake_ns(self)
        }
        fn info_json(&self) -> String {
            info_json(&self.info(), self.modeset_path())
        }
        fn counts(&self) -> Counts {
            let c = KmsDisplay::counts(self);
            Counts { presented: c.presented, stale_stamps: c.stale_stamps, errors: c.errors }
        }
        fn shutdown(self) -> Result<String, String> {
            KmsDisplay::shutdown(self).map_err(|e| e.to_string())
        }
        fn load_world(&mut self, w: &WorldBuffers<'_>) -> Result<u64, String> {
            KmsDisplay::load_world(self, w).map_err(|e| e.to_string())
        }
        fn present_world(&mut self, draw: &WorldDraw, verts: &[Vertex]) -> Result<Presented, String> {
            KmsDisplay::present_world(self, draw, verts).map(presented).map_err(|e| e.to_string())
        }
        fn load_photo(&mut self, w: &WorldAsset, v: &VisualAsset) -> Result<PhotoMemory, String> {
            KmsDisplay::load_photo(self, w, v).map_err(|e| e.to_string())
        }
        fn present_photo(&mut self, draw: &PhotoDraw, verts: &[Vertex]) -> Result<Presented, String> {
            KmsDisplay::present_photo(self, draw, verts).map(presented).map_err(|e| e.to_string())
        }
    }

    /// Offscreen keeps its own presented count (no flips, no stamps).
    pub struct OffscreenPresenter {
        pub inner: Offscreen,
        pub presented: u64,
    }

    impl Presenter for OffscreenPresenter {
        fn present(&mut self, verts: &[Vertex]) -> Result<Presented, String> {
            let r = self.inner.present(verts).map(presented).map_err(|e| e.to_string())?;
            self.presented += 1;
            Ok(r)
        }
        fn next_wake_ns(&self) -> Option<u64> {
            None
        }
        fn info_json(&self) -> String {
            info_json(&self.inner.info(), "none")
        }
        fn counts(&self) -> Counts {
            Counts { presented: self.presented, ..Counts::default() }
        }
        fn shutdown(self) -> Result<String, String> {
            drop(self.inner);
            Ok("offscreen released".into())
        }
        fn load_world(&mut self, w: &WorldBuffers<'_>) -> Result<u64, String> {
            self.inner.load_world(w).map_err(|e| e.to_string())
        }
        fn present_world(&mut self, draw: &WorldDraw, verts: &[Vertex]) -> Result<Presented, String> {
            let r = self.inner.present_world(draw, verts).map(presented).map_err(|e| e.to_string())?;
            self.presented += 1;
            Ok(r)
        }
        fn load_photo(&mut self, w: &WorldAsset, v: &VisualAsset) -> Result<PhotoMemory, String> {
            self.inner.load_photo(w, v).map_err(|e| e.to_string())
        }
        fn present_photo(&mut self, draw: &PhotoDraw, verts: &[Vertex]) -> Result<Presented, String> {
            let (r, _) = self.inner.present_photo(draw, verts, false).map_err(|e| e.to_string())?;
            self.presented += 1;
            Ok(presented(r))
        }
        fn present_capture(&mut self, world: &WorldDraw, photo: Option<&PhotoDraw>, verts: &[Vertex]) -> Result<(Presented, Option<Vec<u8>>), String> {
            let (r, image) = match photo {
                Some(d) => self.inner.present_photo(d, verts, true).map_err(|e| e.to_string())?,
                None => self.inner.present_world_capture(world, verts).map_err(|e| e.to_string())?,
            };
            self.presented += 1;
            Ok((presented(r), image))
        }
    }
}

#[cfg(target_os = "linux")]
pub use native::OffscreenPresenter;

fn opt(v: Option<u64>) -> String {
    v.map_or("null".into(), |v| v.to_string())
}

fn kind_str(k: SampleKind) -> &'static str {
    match k {
        SampleKind::Interpolated => "interpolated",
        SampleKind::Clamped => "clamped",
        SampleKind::LatestOnly => "latest_only",
        SampleKind::Discontinuity => "discontinuity",
        SampleKind::InvalidContinuousState => "invalid_state",
    }
}

fn pct_json(s: &Samples) -> String {
    format!(
        "{{\"n\":{},\"p50\":{},\"p99\":{},\"max\":{}}}",
        s.count,
        opt(s.pct(50)),
        opt(s.pct(99)),
        if s.count > 0 { s.max.to_string() } else { "null".into() }
    )
}

struct LoadedWorld {
    mesh: WorldMesh,
    frame_id: u64,
    frame_generation: u64,
    photo: bool,
}

/// SPIKE-002: the world (as S4, for surfaces, the G0 check and the lights),
/// then the visual sidecar; logs `world` and `visual_asset`.
fn load_photo<P: Presenter>(p: &mut P, asset: &WorldAsset, photo: &PhotoAsset, text: &mut crate::s2::runtime::TextTx, t: u64) -> Result<LoadedWorld, String> {
    let mut w = load_world(p, asset, text, t)?;
    let t0 = crate::s1::rt::MonotonicClock.now_ns();
    let m = p.load_photo(asset, &photo.visual)?;
    let uploaded = crate::s1::rt::MonotonicClock.now_ns();
    let v = &photo.visual;
    let tex: Vec<String> = v
        .textures
        .iter()
        .map(|t| format!("{{\"id\":{},\"w\":{},\"h\":{},\"levels\":{},\"encoding\":\"{:?}\",\"bytes\":{}}}", t.id, t.width, t.height, t.mip_levels.len(), t.encoding, t.byte_len()))
        .collect();
    text.send(format!(
        "{{\"k\":\"visual_asset\",\"t\":{},\"scene\":\"treatment\",\"format\":\"GXVISUAL {}.{}\",\"path\":{},\"file_bytes\":{},\"read_ns\":{},\"frame_id\":{},\"frame_generation\":{},\
\"textures\":[{}],\"materials\":{},\"meshes\":{},\"triangles\":{{\"terrain\":{},\"decal\":{},\"building\":{},\"aircraft\":{}}},\"texture_bytes\":{},\"vertex_bytes\":{},\"index_bytes\":{},\"gpu_static_bytes\":{},\"upload_ns\":{}}}",
        t + (uploaded - t0),
        v.format_major,
        v.format_minor,
        crate::s1::rec::esc(&photo.path),
        photo.file_bytes,
        photo.read_ns,
        v.frame_id,
        v.frame_generation,
        tex.join(","),
        v.materials.len(),
        v.meshes.len(),
        m.triangles[0],
        m.triangles[1],
        m.triangles[2],
        m.triangles[3],
        m.texture_bytes,
        m.vertex_bytes,
        m.index_bytes,
        m.gpu_static_bytes,
        uploaded - t0,
    ));
    w.photo = true;
    Ok(w)
}

/// Builds the static world and the aircraft and uploads them; logs `world`.
fn load_world<P: Presenter>(p: &mut P, asset: &WorldAsset, text: &mut crate::s2::runtime::TextTx, t: u64) -> Result<LoadedWorld, String> {
    let t0 = crate::s1::rt::MonotonicClock.now_ns();
    let mesh = WorldMesh::build(asset)?;
    let aircraft = gx_render::aircraft::global5000();
    let built = crate::s1::rt::MonotonicClock.now_ns();
    let bytes = p.load_world(&WorldBuffers { ground: &mesh.ground, lights: &mesh.lights, aircraft: &aircraft })?;
    let uploaded = crate::s1::rt::MonotonicClock.now_ns();
    let st = &mesh.stats;
    let g = asset.georeference;
    text.send(format!(
        "{{\"k\":\"world\",\"t\":{t},\"frame_id\":{},\"frame_generation\":{},\"geoid_separation_m\":{},\"pack_triangles\":{},\"ground_triangles\":{},\"surround_triangles\":{},\
\"markings_triangles\":{},\"lights\":{},\"aircraft_triangles\":{},\"static_vertex_bytes\":{bytes},\"curvature_radius_m\":{:.0},\"build_ns\":{},\"upload_ns\":{},\
\"camera_button\":\"button 5 / BTN_TOP2\",\"on_runway_limit_m\":{ON_RUNWAY_LIMIT_M}}}",
        g.frame_id,
        g.frame_generation,
        g.geoid_separation_m,
        st.pack_triangles,
        st.ground_triangles,
        st.surround_triangles,
        st.markings_triangles,
        st.lights,
        aircraft.len() / 3,
        st.radius_m,
        built - t0,
        uploaded - built,
    ));
    Ok(LoadedWorld { mesh, frame_id: g.frame_id, frame_generation: g.frame_generation, photo: false })
}

fn ground_json(t: u64, generation: u64, when: &str, agl_m: f64, g: &GroundCheck) -> String {
    let wheels: Vec<String> = g
        .wheels
        .iter()
        .map(|w| {
            format!(
                "{{\"gear\":\"{}\",\"enu\":[{:.4},{:.4},{:.4}],\"surface\":{},\"surface_z\":{},\"gap_m\":{}}}",
                w.gear,
                w.wheel_enu[0],
                w.wheel_enu[1],
                w.wheel_enu[2],
                w.surface.map_or("null".into(), |s| format!("\"{}\"", s.kind)),
                w.surface.map_or("null".into(), |s| format!("{:.4}", s.z_m)),
                w.gap_m.map_or("null".into(), |v| format!("{v:.4}")),
            )
        })
        .collect();
    format!(
        "{{\"k\":\"ground_check\",\"t\":{t},\"gen\":{generation},\"when\":\"{when}\",\"on_runway\":{},\"max_abs_gap_m\":{},\"limit_m\":{ON_RUNWAY_LIMIT_M},\"agl_m\":{agl_m:.4},\"wheels\":[{}]}}",
        g.on_runway,
        g.max_abs_gap_m.map_or("null".into(), |v| format!("{v:.4}")),
        wheels.join(",")
    )
}

/// The aircraft's flap detents as (`fcs/flap-cmd-norm`, label) for the HUD
/// FLAPS readout; set once before the threads start (`s3::main`).
pub static HUD_FLAP_DETENTS: std::sync::OnceLock<Vec<(f32, u8)>> = std::sync::OnceLock::new();

/// The approach names (`28R`), in the order of the core's approaches
/// (`s2::setup::approaches`), for the autopilot panel: `ApStatus.approach`
/// indexes this list. Set once before the threads start (`s3::main`).
pub static HUD_APPROACHES: std::sync::OnceLock<Vec<String>> = std::sync::OnceLock::new();

/// Blink period of the panel's warning line (on half, off half).
const PANEL_BLINK_NS: u64 = 1_000_000_000;


/// The S4 autopilot panel from the latest snapshot's autopilot (default
/// before the first snapshot) and the board's mouse state.
fn panel_view(s: Option<&RenderSample>, board: &Board, since_start_ns: u64) -> PanelView {
    let status = s.map(|s| s.autopilot).unwrap_or_default();
    let approach = status.approach.and_then(|i| HUD_APPROACHES.get()?.get(usize::from(i)).cloned());
    PanelView {
        status,
        approach,
        cursor: board.cursor(),
        mouse: board.mouse_ok(),
        blink: since_start_ns % PANEL_BLINK_NS < PANEL_BLINK_NS / 2,
        exit_armed: board.exit_armed(),
    }
}

/// The render thread body. `factory` runs here, so every native object is
/// created, used and destroyed by this one thread.
pub fn render_loop<C, P, F>(ctx: SnapshotCtx<C>, factory: F) -> io::Result<(ReaderTotals, String)>
where
    C: Clock,
    P: Presenter,
    F: FnOnce() -> Result<P, String>,
{
    render_loop_look(ctx, factory, Look::Horizon)
}

/// [`render_loop`] drawing `look`.
pub fn render_loop_look<C, P, F>(ctx: SnapshotCtx<C>, factory: F, look: Look) -> io::Result<(ReaderTotals, String)>
where
    C: Clock,
    P: Presenter,
    F: FnOnce() -> Result<P, String>,
{
    let rt = rt::setup_reader_thread(RENDER_CPU);
    let SnapshotCtx { clock, epoch, exchange, board, mut log, mut text, prompt } = ctx;
    let rel = |t: u64| t.saturating_sub(epoch);
    let mut p = match factory() {
        Ok(p) => p,
        Err(e) => {
            text.send(format!("{{\"k\":\"render_error\",\"t\":{},\"stage\":\"open\",\"what\":{}}}", rel(clock.now_ns()), crate::s1::rec::esc(&e)));
            return Err(io::Error::other(format!("display: {e}")));
        }
    };
    text.send(format!("{{\"k\":\"display\",\"t\":{},{},\"thread\":{rt}}}", rel(clock.now_ns()), p.info_json()));
    let world = match &look {
        Look::Horizon => None,
        Look::World(asset) => match load_world(&mut p, asset, &mut text, rel(clock.now_ns())) {
            Ok(w) => Some(w),
            Err(e) => {
                text.send(format!("{{\"k\":\"render_error\",\"t\":{},\"stage\":\"world\",\"what\":{}}}", rel(clock.now_ns()), crate::s1::rec::esc(&e)));
                let _ = p.shutdown();
                return Err(io::Error::other(format!("world: {e}")));
            }
        },
        Look::Photo(asset, photo) => match load_photo(&mut p, asset, photo, &mut text, rel(clock.now_ns())) {
            Ok(w) => Some(w),
            Err(e) => {
                text.send(format!("{{\"k\":\"render_error\",\"t\":{},\"stage\":\"visual_asset\",\"what\":{}}}", rel(clock.now_ns()), crate::s1::rec::esc(&e)));
                let _ = p.shutdown();
                return Err(io::Error::other(format!("visual asset: {e}")));
            }
        },
    };
    // SPIKE-002 offscreen trace/captures (never set for the S4 runs).
    let opts = crate::spike002::RENDER_OPTS.get().cloned().unwrap_or_default();
    if let Some(o) = &opts.trace {
        text.send(format!(
            "{{\"k\":\"spike002_trace\",\"t\":{},\"segment_s\":{},\"segments\":[\"{}\"],\"threshold_enu\":[{:.2},{:.2}],\"heading_deg\":{:.3},\"capture_dir\":{}}}",
            rel(clock.now_ns()),
            crate::spike002::SEGMENT_S,
            crate::spike002::SEGMENTS.join("\",\""),
            o.threshold_enu[0],
            o.threshold_enu[1],
            o.heading_deg,
            opts.capture_dir.as_deref().map_or("null".into(), crate::s1::rec::esc),
        ));
    }
    let mut trace_t0: Option<u64> = None;
    let mut captured = [false; 5];
    let photo_t0 = clock.now_ns();
    let mut camera = CameraMode::Chase;
    let mut camera_frames = [0u64; 2];
    let (mut world_frames, mut no_pose_frames, mut frame_mismatch) = (0u64, 0u64, 0u64);
    let mut checked: Option<(u64, bool, bool)> = None; // (generation, reset check done, settled check done)
    let mut running_since: Option<u64> = None;
    let mut ground_fail = 0u64;
    let mut ground_checks = 0u64;
    let mut panel_frames = 0u64;

    let mut scene = Scene::new();
    let mut prompt_seq = 0u64;
    let mut prompt_lines: Vec<String> = Vec::new();
    let mut totals = ReaderTotals::default();
    let (mut intervals, mut gpu, mut latency, mut cpu) = (
        Samples::with_capacity(STATS_CAP),
        Samples::with_capacity(STATS_CAP),
        Samples::with_capacity(STATS_CAP),
        Samples::with_capacity(STATS_CAP),
    );
    let mut frames = 0u64;
    let mut foreign = 0u64;
    let mut gpu_status: [u64; 4] = [0; 4]; // ok, not_ready, disjoint, unavailable
    let mut banner_frames = [0u64; 4]; // none, no_input, stick_lost, paused_stick_lost
    let mut overlay_frames = 0u64;
    let mut last_drawn: Option<(Banner, Overlay)> = None;
    let mut first_flip: Option<u64> = None;
    let mut last_flip = 0u64;
    let start = clock.now_ns();
    let mut k = 1u64;
    let mut failure: Option<String> = None;

    while !board.stop_sim.load(Ordering::Relaxed) {
        let due = p.next_wake_ns().unwrap_or(start + FixedRate::elapsed_ns(READER_HZ, k));
        clock.sleep_until(due);
        if board.stop_sim.load(Ordering::Relaxed) {
            break;
        }
        let t0 = clock.now_ns();
        let pair = exchange.try_latest_pair();
        let t1 = clock.now_ns();
        let copy_ns = t1 - t0;
        totals.reads += 1;
        totals.copy_max_ns = totals.copy_max_ns.max(copy_ns);
        // The sim's current state (not the frozen paused snapshot): INPUT-002
        // must clear the warning the moment the stick returns, paused or not.
        let (run, pause, status) = board.vis();
        let input = InputHealth { status, loss_ms: board.loss_ms.load(Ordering::Relaxed) };
        if let Some((seq, lines)) = prompt.try_get_newer(prompt_seq) {
            prompt_seq = seq;
            prompt_lines = lines;
        }
        let paused = run != RunState::Running;
        let (s, read): (Option<RenderSample>, ReadRec) = match pair {
            Some(pair) => {
                let latest = pair.latest;
                let age_ns = t1.saturating_sub(latest.published_at_ns);
                if !paused {
                    totals.age_max_running_ns = totals.age_max_running_ns.max(age_ns);
                }
                let rec = ReadRec {
                    t: t1,
                    ok: true,
                    seq: latest.sequence,
                    generation: latest.generation,
                    tick: latest.simulation_tick,
                    age_ns,
                    copy_ns,
                    paused,
                    pair: pair.previous.is_some(),
                    warning: input.warning(),
                };
                (Some(sample(pair, t1, INTERPOLATION_DELAY_NS)), rec)
            }
            None => {
                totals.none += 1;
                let rec = ReadRec { t: t1, ok: false, seq: 0, generation: 0, tick: 0, age_ns: 0, copy_ns, paused, pair: false, warning: input.warning() };
                (None, rec)
            }
        };
        log.send(Rec::Read(read));
        let hud_numbers = s.as_ref().map(HudReadout::from);
        let hud = HudInput {
            roll_rad: s.map_or(0.0, |s| s.roll_rad),
            pitch_rad: s.map_or(0.0, |s| s.pitch_rad),
            heading_rad: s.map_or(0.0, |s| s.heading_rad),
            airspeed_kt: hud_numbers.map_or(0, |h| h.airspeed_kt),
            altitude_agl_ft: hud_numbers.map_or(0, |h| h.altitude_agl_ft),
            heading_deg: hud_numbers.map_or(0, |h| h.heading_deg),
            vertical_speed_fpm: hud_numbers.map_or(0, |h| h.vertical_speed_fpm),
            engine_n1_pct: hud_numbers.map_or(0, |h| h.engine_n1_pct),
            pitch_trim_pct: hud_numbers.map(|h| h.pitch_trim_pct),
            config: s.as_ref().map(|s| gx_render::frame::config_readout(s, HUD_FLAP_DETENTS.get().map_or(&[][..], Vec::as_slice))),
            run,
            pause,
            input,
            prompt: &prompt_lines,
            // AP-001: the mouse panel in the S4 looks only (S3 has no mouse).
            panel: world.is_some().then(|| panel_view(s.as_ref(), &board, t1.saturating_sub(start))),
        };
        let panel_drawn = hud.panel.is_some();
        let mut cam_json = String::new();
        let result = match &world {
            None => {
                let drawn = scene.build(&hud);
                (drawn, p.present(scene.vertices()))
            }
            Some(w) => {
                let presses = board.camera_presses.load(Ordering::Relaxed);
                let mut mode = CameraMode::after_presses(presses);
                let mut trace_seg: Option<(usize, Option<(gx_render::math::V3, [f64; 4])>, f64)> = None;
                if let Some(o) = &opts.trace {
                    let ts = (t1 - *trace_t0.get_or_insert(t1)) as f64 * 1e-9;
                    let (seg, cockpit, synthetic) = crate::spike002::trace_pose(o, ts);
                    mode = if cockpit { CameraMode::Cockpit } else { CameraMode::Chase };
                    let u = if seg == 4 { 0.0 } else { (ts - crate::spike002::WARMUP_S) % crate::spike002::SEGMENT_S };
                    trace_seg = Some((seg, synthetic, u));
                }
                if mode != camera {
                    camera = mode;
                    text.send(format!("{{\"k\":\"camera\",\"t\":{},\"camera\":\"{}\",\"presses\":{presses}}}", rel(t1), mode.name()));
                }
                let pose = s.filter(|s| {
                    let same = s.frame_id == w.frame_id && s.frame_generation == w.frame_generation;
                    if !same {
                        frame_mismatch += 1;
                    }
                    same && s.position_m.iter().chain(s.orientation_xyzw.iter()).all(|v| v.is_finite())
                });
                match pose {
                    Some(s) => {
                        let pose = Pose { position_m: s.position_m, orientation_xyzw: s.orientation_xyzw };
                        let view = View::new(mode, &pose);
                        // On-runway gate: the first pose of every snapshot
                        // generation (start, each Reset) and, once per
                        // generation, after 3 s of running still on the wheels.
                        if checked.map(|c| c.0) != Some(s.generation) {
                            checked = Some((s.generation, false, false));
                            running_since = None;
                        }
                        if run == RunState::Running {
                            running_since.get_or_insert(t1);
                        } else {
                            running_since = None;
                        }
                        let c = checked.as_mut().unwrap();
                        let settled_due = running_since.is_some_and(|r| t1 - r >= SETTLED_CHECK_NS) && s.weight_on_wheels;
                        if !c.1 || (settled_due && !c.2) {
                            let when = if !c.1 { "reset" } else { "settled" };
                            if !c.1 {
                                c.1 = true;
                            } else {
                                c.2 = true;
                            }
                            let g = ground_check(&w.mesh, &pose);
                            ground_checks += 1;
                            ground_fail += u64::from(!g.on_runway);
                            text.send(ground_json(rel(t1), s.generation, when, s.height_agl_m, &g));
                        }
                        // SPIKE-002 trace: the view follows the scripted pose
                        // (the G0 check above used the simulated one).
                        let (pose, view) = match trace_seg {
                            Some((_, Some((position_m, orientation_xyzw)), _)) => {
                                let tp = Pose { position_m, orientation_xyzw };
                                (tp, View::new(mode, &tp))
                            }
                            _ => (pose, view),
                        };
                        let drawn = scene.build_overlay(&hud, view.px_per_deg(), mode.label());
                        let draw = WorldDraw {
                            clear: SKY_CLEAR,
                            ground_mvp: view.world_mvp().to_f32(),
                            aircraft_mvp: (mode == CameraMode::Chase).then(|| view.model_mvp(&pose).to_f32()),
                            point_k: POINT_K,
                        };
                        world_frames += 1;
                        camera_frames[usize::from(mode == CameraMode::Cockpit)] += 1;
                        let _ = write!(cam_json, ",\"cam\":\"{}\",\"pos\":[{:.2},{:.2},{:.2}]", mode.name(), pose.position_m[0], pose.position_m[1], pose.position_m[2]);
                        let photo_draw = w.photo.then(|| PhotoDraw::new(
                            draw,
                            &view,
                            &pose,
                            (t1 - photo_t0) as f64 * 1e-9,
                            [
                                s.aileron_position, s.elevator_position, s.rudder_position,
                                s.flap_position, s.speedbrake_position, s.gear_position,
                            ],
                        ));
                        let capture = match (trace_seg, &opts.capture_dir) {
                            (Some((seg, _, u)), Some(_)) if seg < 4 && u >= 15.0 && !captured[seg] => {
                                captured[seg] = true;
                                Some(seg)
                            }
                            _ => None,
                        };
                        if let Some((seg, _, _)) = trace_seg {
                            let _ = write!(cam_json, ",\"seg\":\"{}\"", crate::spike002::SEGMENTS[seg]);
                        }
                        let presented = match capture {
                            Some(seg) => {
                                let _ = write!(cam_json, ",\"capture\":true");
                                p.present_capture(&draw, photo_draw.as_ref(), scene.vertices()).map(|(f, image)| {
                                    if let (Some(image), Some(dir)) = (image, &opts.capture_dir) {
                                        let name = format!("{dir}/{}-{}.ppm", if w.photo { "T" } else { "C" }, crate::spike002::SEGMENTS[seg]);
                                        let mut ppm = format!("P6\n{} {}\n255\n", gx_render::scene::WIDTH as u32, gx_render::scene::HEIGHT as u32).into_bytes();
                                        ppm.extend_from_slice(&image);
                                        if let Err(e) = std::fs::write(&name, &ppm) {
                                            text.send(format!("{{\"k\":\"capture_error\",\"what\":{}}}", crate::s1::rec::esc(&e.to_string())));
                                        } else {
                                            text.send(format!("{{\"k\":\"capture\",\"t\":{},\"seg\":\"{}\",\"file\":{}}}", rel(t1), crate::spike002::SEGMENTS[seg], crate::s1::rec::esc(&name)));
                                        }
                                    }
                                    f
                                })
                            }
                            None => match &photo_draw {
                                Some(d) => p.present_photo(d, scene.vertices()),
                                None => p.present_world(&draw, scene.vertices()),
                            },
                        };
                        (drawn, presented)
                    }
                    None => {
                        no_pose_frames += 1;
                        let _ = write!(cam_json, ",\"cam\":\"none\"");
                        let drawn = scene.build(&hud);
                        (drawn, p.present(scene.vertices()))
                    }
                }
            }
        };
        let drawn: Drawn = result.0;
        let f = match result.1 {
            Ok(f) => f,
            Err(e) => {
                text.send(format!("{{\"k\":\"render_error\",\"t\":{},\"stage\":\"present\",\"frame\":{},\"what\":{}}}", rel(clock.now_ns()), frames + 1, crate::s1::rec::esc(&e)));
                failure = Some(e);
                break;
            }
        };
        frames += 1;
        panel_frames += u64::from(panel_drawn);
        foreign += u64::from(f.foreign_events);
        if let Some(i) = f.interval_ns {
            intervals.push(i);
        }
        if let Some(g) = f.gpu_ns {
            gpu.push(g);
        }
        gpu_status[match f.gpu_status {
            "ok" => 0,
            "not_ready" => 1,
            "disjoint" => 2,
            _ => 3,
        }] += 1;
        latency.push(f.vblank_ns.saturating_sub(t1));
        cpu.push(f.swap_end_ns.saturating_sub(f.draw_start_ns));
        first_flip.get_or_insert(f.vblank_ns);
        last_flip = f.vblank_ns;
        banner_frames[match drawn.banner {
            Banner::None => 0,
            Banner::NoInput => 1,
            Banner::StickLost { .. } => 2,
            Banner::PausedStickLost => 3,
        }] += 1;
        overlay_frames += u64::from(drawn.overlay != Overlay::None);

        let mut line = String::with_capacity(512);
        let _ = write!(
            line,
            "{{\"k\":\"frame\",\"t\":{},\"n\":{frames},\"token\":{},\"status\":\"{}\",\"commit_t\":{},\"flip_t\":{},\"vblank_t\":{},\"interval_ns\":{},\
\"gpu_ns\":{},\"gpu\":\"{}\",\"cpu_ns\":{},\"fence_ns\":{},\"sample_to_vblank_ns\":{},\"seq\":{},\"kind\":\"{}\",\"run\":\"{}\",\"input\":\"{}\",\
\"banner\":\"{}\",\"banner_text\":{},\"overlay\":\"{}\",\"overlay_text\":{},\"roll_deg\":{:.2},\"pitch_deg\":{:.2},\"ias_kt\":{},\"agl_ft\":{},\"hdg_deg\":{},\"vs_fpm\":{},\"n1_pct\":{},\"verts\":{},\"foreign\":{}{}}}",
            rel(t1),
            f.token,
            f.status,
            rel(f.commit_ns),
            f.flip_ns.map_or("null".into(), |t| rel(t).to_string()),
            rel(f.vblank_ns),
            opt(f.interval_ns),
            opt(f.gpu_ns),
            f.gpu_status,
            f.swap_end_ns.saturating_sub(f.draw_start_ns),
            f.fence_ns.saturating_sub(f.swap_end_ns),
            f.vblank_ns.saturating_sub(t1),
            s.map_or(0, |s| s.snapshot_sequence),
            s.map_or("none", |s| kind_str(s.kind)),
            crate::s1::rec::run_str(run),
            crate::s1::rec::status_str(status),
            drawn.banner.name(),
            drawn.banner.text().map_or("null".into(), |t| crate::s1::rec::esc(&t)),
            drawn.overlay.name(),
            drawn.overlay.text().map_or("null".into(), |t| crate::s1::rec::esc(&t)),
            drawn.roll_deg,
            drawn.pitch_deg,
            drawn.airspeed_kt,
            drawn.altitude_agl_ft,
            drawn.heading_deg,
            drawn.vertical_speed_fpm,
            drawn.engine_n1_pct,
            drawn.vertices,
            f.foreign_events,
            if world.is_some() { format!(",\"draws\":{}{cam_json}", f.draws) } else { String::new() },
        );
        text.send(line);
        if last_drawn != Some((drawn.banner, drawn.overlay)) {
            last_drawn = Some((drawn.banner, drawn.overlay));
            text.send(format!(
                "{{\"k\":\"drawn\",\"t\":{},\"n\":{frames},\"banner\":\"{}\",\"banner_text\":{},\"overlay\":\"{}\",\"overlay_text\":{}}}",
                rel(f.vblank_ns),
                drawn.banner.name(),
                drawn.banner.text().map_or("null".into(), |t| crate::s1::rec::esc(&t)),
                drawn.overlay.name(),
                drawn.overlay.text().map_or("null".into(), |t| crate::s1::rec::esc(&t)),
            ));
        }

        k += 1;
        let now = clock.now_ns();
        while start + FixedRate::elapsed_ns(READER_HZ, k) <= now {
            k += 1;
        }
    }

    let counts = p.counts();
    let restore = p.shutdown();
    let span_ns = last_flip.saturating_sub(first_flip.unwrap_or(last_flip));
    let fps = if frames > 1 && span_ns > 0 { (frames - 1) as f64 * 1e9 / span_ns as f64 } else { 0.0 };
    text.send(format!(
        "{{\"k\":\"render_summary\",\"t\":{},\"frames\":{frames},\"presented\":{},\"stale_stamps\":{},\"present_errors\":{},\"foreign_events\":{foreign},\
\"fps\":{fps:.3},\"span_ns\":{span_ns},\"interval_ns\":{},\"gpu_ns\":{},\"gpu_status\":{{\"ok\":{},\"not_ready\":{},\"disjoint\":{},\"unavailable\":{}}},\
\"sample_to_vblank_ns\":{},\"cpu_ns\":{},\"banner_frames\":{{\"none\":{},\"no_input\":{},\"stick_lost\":{},\"paused_stick_lost\":{}}},\"overlay_frames\":{overlay_frames},\
\"failure\":{},\"restore\":{},\"text_drops\":{},\"thread\":{rt}{}}}",
        rel(clock.now_ns()),
        counts.presented,
        counts.stale_stamps,
        counts.errors,
        pct_json(&intervals),
        pct_json(&gpu),
        gpu_status[0],
        gpu_status[1],
        gpu_status[2],
        gpu_status[3],
        pct_json(&latency),
        pct_json(&cpu),
        banner_frames[0],
        banner_frames[1],
        banner_frames[2],
        banner_frames[3],
        failure.as_deref().map_or("null".into(), crate::s1::rec::esc),
        match &restore {
            Ok(s) => format!("{{\"ok\":true,\"detail\":{}}}", crate::s1::rec::esc(s)),
            Err(e) => format!("{{\"ok\":false,\"detail\":{}}}", crate::s1::rec::esc(e)),
        },
        text.dropped,
        if world.is_some() {
            format!(
                ",\"world\":{{\"frames\":{world_frames},\"no_pose_frames\":{no_pose_frames},\"frame_mismatch\":{frame_mismatch},\"camera_frames\":{{\"chase\":{},\"cockpit\":{}}},\"ground_checks\":{ground_checks},\"ground_check_failures\":{ground_fail},\"panel_frames\":{panel_frames}}}",
                camera_frames[0], camera_frames[1]
            )
        } else {
            String::new()
        },
    ));
    totals.drops = log.dropped;
    if let Some(e) = failure {
        return Err(io::Error::other(format!("present: {e}")));
    }
    Ok((totals, rt))
}

#[cfg(test)]
mod tests {
    use super::*;
    use gx_sim::autopilot::Lateral;
    use gx_sim::{AircraftSnapshot, AircraftState, ApStatus, SnapshotPair};

    #[test]
    fn the_panel_shows_the_latest_autopilot_and_names_its_approach() {
        let _ = HUD_APPROACHES.set(vec!["10L".into(), "28R".into()]);
        let ap = ApStatus { engaged: true, lateral: Lateral::Localizer, approach: Some(1), ..ApStatus::default() };
        let latest = AircraftSnapshot { state: AircraftState { autopilot: ap, ..Default::default() }, ..Default::default() };
        let s = sample(SnapshotPair { previous: None, latest }, 0, 0);
        let board = Board::default();
        let v = panel_view(Some(&s), &board, 0);
        assert_eq!((v.status, v.approach.as_deref()), (ap, Some("28R")));
        assert!(v.blink && !panel_view(Some(&s), &board, PANEL_BLINK_NS / 2).blink, "1 s blink, lit first");
        // An index past the list (or no list) shows no name rather than panicking.
        let s = RenderSample { autopilot: ApStatus { approach: Some(9), ..ap }, ..s };
        assert_eq!(panel_view(Some(&s), &board, 0).approach, None);
        // Before the first snapshot: the panel's defaults, autopilot off.
        assert!(!panel_view(None, &board, 0).status.engaged);
    }
}
