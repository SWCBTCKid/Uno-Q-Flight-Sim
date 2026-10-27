//! Pure S3 scene: what one frame draws, as a flat list of opaque coloured
//! triangles, plus a record of the text it drew (for the render-side log).
//!
//! Everything here is host-testable. The GLES backend uploads
//! [`Scene::vertices`] into one vertex buffer and issues one draw call, so a
//! frame is a single opaque pass: frame-start clear, one draw, end-of-frame
//! depth/stencil invalidate (SPIKE-001c). Later triangles overdraw earlier
//! ones (painter's order); [`Scene::color_at`] rasterizes the same list on
//! the CPU for the offline render test.
//!
//! Screen: 1280x720 pixels, origin top-left, y down. Vertices are NDC.

use gx_sim::{InputHealth, InputStatus, PauseReason, RunState};

pub const WIDTH: f32 = 1280.0;
pub const HEIGHT: f32 = 720.0;
/// Pitch scale of the artificial horizon.
pub const PX_PER_DEG: f32 = 8.0;
/// INPUT-002: the automatic pause after 10 s of continuous loss.
pub const LOSS_PAUSE_S: u64 = 10;

pub type Rgb = [f32; 3];
pub const SKY: Rgb = [0.16, 0.43, 0.72];
pub const GROUND: Rgb = [0.40, 0.25, 0.12];
pub const WHITE: Rgb = [1.0, 1.0, 1.0];
pub const SYMBOL: Rgb = [1.0, 0.85, 0.0];
pub const HUD_TEXT: Rgb = [0.35, 1.0, 0.45];
pub const PANEL: Rgb = [0.02, 0.03, 0.05];
pub const BANNER: Rgb = [0.78, 0.05, 0.03];
pub const OVERLAY_TEXT: Rgb = [1.0, 0.72, 0.0];
pub const PROMPT_TEXT: Rgb = [0.92, 0.92, 0.92];

/// One vertex: NDC position and colour (20 bytes, tightly packed).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vertex {
    pub pos: [f32; 2],
    pub color: Rgb,
}

/// The INPUT-002 warning banner (bottom of the screen).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Banner {
    None,
    /// No valid input yet since start-up (stick never moved / not plugged).
    NoInput,
    /// Input lost while the sim keeps running: "STICK LOST - N S".
    StickLost { seconds: u64 },
    /// The 10 s loss pause, stick still lost.
    PausedStickLost,
}

/// The centre overlay whenever the sim is not running.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Overlay {
    None,
    /// Stopped before the first Start.
    Ready,
    Paused(PauseReason),
}

impl Banner {
    pub fn text(self) -> Option<String> {
        match self {
            Banner::None => None,
            Banner::NoInput => Some("NO STICK INPUT - MOVE THE STICK".into()),
            Banner::StickLost { seconds } => Some(format!("STICK LOST - {seconds} S")),
            Banner::PausedStickLost => Some("PAUSED - PRESS TRIGGER WHEN THE STICK IS BACK".into()),
        }
    }

    /// Stable log name.
    pub fn name(self) -> &'static str {
        match self {
            Banner::None => "none",
            Banner::NoInput => "no_input",
            Banner::StickLost { .. } => "stick_lost",
            Banner::PausedStickLost => "paused_stick_lost",
        }
    }
}

impl Overlay {
    pub fn text(self) -> Option<String> {
        match self {
            Overlay::None => None,
            Overlay::Ready => Some("READY - PULL TRIGGER TO START".into()),
            Overlay::Paused(r) => Some(format!("PAUSED ({}) - PULL TRIGGER", pause_label(r))),
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Overlay::None => "none",
            Overlay::Ready => "ready",
            Overlay::Paused(PauseReason::User) => "paused_user",
            Overlay::Paused(PauseReason::Reset) => "paused_reset",
            Overlay::Paused(PauseReason::InputLoss) => "paused_input_loss",
            Overlay::Paused(PauseReason::InvalidWorldPose) => "paused_invalid_world_pose",
        }
    }
}

pub fn pause_label(r: PauseReason) -> &'static str {
    match r {
        PauseReason::User => "USER",
        PauseReason::Reset => "RESET",
        PauseReason::InputLoss => "STICK LOST",
        PauseReason::InvalidWorldPose => "INVALID POSITION",
    }
}

/// INPUT-002 banner from the sim's current state (not a historic snapshot:
/// while paused no snapshots are published, but the warning must still clear
/// the moment the stick returns).
pub fn banner_for(run: RunState, pause: Option<PauseReason>, input: InputHealth) -> Banner {
    if input.status == InputStatus::Valid {
        return Banner::None;
    }
    if run != RunState::Running && pause == Some(PauseReason::InputLoss) {
        return Banner::PausedStickLost;
    }
    if input.status == InputStatus::NoInput {
        return Banner::NoInput;
    }
    Banner::StickLost { seconds: (input.loss_ms / 1000).min(LOSS_PAUSE_S) }
}

/// Centre overlay. The input-loss pause with the stick still lost is already
/// explained by the banner, so it shows no second message.
pub fn overlay_for(run: RunState, pause: Option<PauseReason>, banner: Banner) -> Overlay {
    match (run, pause) {
        (RunState::Running, _) => Overlay::None,
        (_, Some(PauseReason::InputLoss)) if banner == Banner::PausedStickLost => Overlay::None,
        (_, Some(r)) => Overlay::Paused(r),
        (RunState::Stopped, None) => Overlay::Ready,
        (RunState::Paused, None) => Overlay::Paused(PauseReason::User),
    }
}

/// Everything one frame shows.
#[derive(Clone, Debug, PartialEq)]
pub struct HudInput<'a> {
    pub roll_rad: f64,
    pub pitch_rad: f64,
    pub heading_rad: f64,
    pub airspeed_kt: u16,
    pub altitude_agl_ft: i32,
    pub heading_deg: u16,
    pub vertical_speed_fpm: i32,
    pub engine_n1_pct: u16,
    /// Pitch trim, percent of elevator travel (+ = nose up); `None` draws no
    /// trim readout.
    pub pitch_trim_pct: Option<i16>,
    /// Gear/flap/speedbrake readout (INPUT-005); `None` draws none.
    pub config: Option<ConfigReadout>,
    pub run: RunState,
    pub pause: Option<PauseReason>,
    pub input: InputHealth,
    /// Up to two operator prompt lines (newest last), drawn at the bottom
    /// (above the autopilot panel when there is one).
    pub prompt: &'a [String],
    /// The autopilot panel ([`crate::panel`]); `None` draws none.
    pub panel: Option<crate::panel::PanelView>,
}

/// Gear position as the HUD names it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GearText {
    Down,
    Up,
    Transit,
}

/// The gear/flap/speedbrake strip under the VS readout
/// ([`crate::frame::config_readout`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ConfigReadout {
    pub gear: GearText,
    /// Gear in transit, or lever and gear disagree (squat switch): amber.
    pub gear_caution: bool,
    /// Flap lever detent label (degrees).
    pub flaps_label: u8,
    /// Flaps still travelling to the lever: amber.
    pub flaps_moving: bool,
    /// Speedbrake (larger of lever and position), percent; 0 hides it.
    pub speedbrake_pct: u8,
}

impl ConfigReadout {
    /// `GEAR DN  FLAPS 5`-style texts with their colours, left to right.
    pub fn texts(&self) -> [(String, Rgb); 3] {
        let gear = match self.gear {
            GearText::Down => "GEAR DN",
            GearText::Up => "GEAR UP",
            GearText::Transit => "GEAR ---",
        };
        let colour = |caution: bool| if caution { OVERLAY_TEXT } else { HUD_TEXT };
        let sb = match self.speedbrake_pct {
            0 => String::new(),
            100 => "SPDBRK FULL".to_string(),
            p => format!("SPDBRK {p}%"),
        };
        [
            (gear.to_string(), colour(self.gear_caution)),
            (format!("FLAPS {}", self.flaps_label), colour(self.flaps_moving)),
            (sb, OVERLAY_TEXT),
        ]
    }
}

/// What a frame drew, for the frame log.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Drawn {
    pub banner: Banner,
    pub overlay: Overlay,
    pub roll_deg: f32,
    pub pitch_deg: f32,
    pub airspeed_kt: u16,
    pub altitude_agl_ft: i32,
    pub heading_deg: u16,
    pub vertical_speed_fpm: i32,
    pub engine_n1_pct: u16,
    pub pitch_trim_pct: Option<i16>,
    pub vertices: u32,
}

#[derive(Clone, Copy, Debug)]
enum Style<'a> {
    S3,
    Overlay { px_per_deg: f32, camera_label: &'a str },
}

/// Reusable per-frame geometry buffer (no reallocation after warm-up).
pub struct Scene {
    verts: Vec<Vertex>,
}

impl Default for Scene {
    fn default() -> Self {
        Self::new()
    }
}

/// Upper bound on vertices per frame; the GL vertex buffer is sized to it.
pub const MAX_VERTICES: usize = 65_536;

impl Scene {
    pub fn new() -> Self {
        Self { verts: Vec::with_capacity(16_384) }
    }

    pub fn vertices(&self) -> &[Vertex] {
        &self.verts
    }

    /// The S3 frame: full-screen sky/ground artificial horizon plus the HUD.
    pub fn build(&mut self, hud: &HudInput<'_>) -> Drawn {
        self.build_styled(hud, Style::S3)
    }

    /// The S4 overlay drawn over the 3D world: the same HUD, but the
    /// horizon and pitch ladder are lines only (no sky/ground fill), scaled
    /// to the camera (`px_per_deg`), plus the camera name.
    pub fn build_overlay(&mut self, hud: &HudInput<'_>, px_per_deg: f32, camera_label: &str) -> Drawn {
        self.build_styled(hud, Style::Overlay { px_per_deg, camera_label })
    }

    fn build_styled(&mut self, hud: &HudInput<'_>, style: Style<'_>) -> Drawn {
        self.verts.clear();
        let roll = if hud.roll_rad.is_finite() { hud.roll_rad as f32 } else { 0.0 };
        let pitch = if hud.pitch_rad.is_finite() { hud.pitch_rad as f32 } else { 0.0 };
        match style {
            Style::S3 => self.horizon(roll, pitch, PX_PER_DEG, true),
            Style::Overlay { px_per_deg, .. } => self.horizon(roll, pitch, px_per_deg, false),
        }
        self.aircraft_symbol();

        // Top bar: primary flight values plus the Chief-requested engine N1
        // and vertical speed in feet per minute.
        self.rect(0.0, 0.0, WIDTH, 94.0, PANEL);
        self.text(&format!("IAS {:3} KT", hud.airspeed_kt), 16.0, 12.0, 4.0, HUD_TEXT);
        self.text_centered(&format!("HDG {:03}", hud.heading_deg), 12.0, 4.0, HUD_TEXT);
        let alt = format!("AGL {:5} FT", hud.altitude_agl_ft);
        let w = text_width(&alt, 4.0);
        self.text(&alt, WIDTH - 16.0 - w, 12.0, 4.0, HUD_TEXT);
        self.text(&format!("N1 {:3}%", hud.engine_n1_pct), 16.0, 58.0, 3.0, HUD_TEXT);
        let vs = format!("VS {:+6} FPM", hud.vertical_speed_fpm);
        let vs_w = text_width(&vs, 3.0);
        self.text(&vs, WIDTH - 16.0 - vs_w, 58.0, 3.0, HUD_TEXT);
        if let Some(t) = hud.pitch_trim_pct {
            // Small readout under the N1 value, on its own panel strip.
            let s = trim_text(t);
            self.rect(0.0, 94.0, text_width(&s, 3.0) + 32.0, 34.0, PANEL);
            self.text(&s, 16.0, 100.0, 3.0, HUD_TEXT);
        }
        if let Some(c) = hud.config {
            // Right-hand strip under VS: gear, flaps, speedbrake (if out).
            let texts = c.texts();
            let width: f32 = texts.iter().filter(|(t, _)| !t.is_empty()).map(|(t, _)| text_width(t, 3.0) + 24.0).sum();
            let mut x = WIDTH - width;
            self.rect(x - 16.0, 94.0, width + 16.0, 34.0, PANEL);
            for (t, colour) in texts.iter().filter(|(t, _)| !t.is_empty()) {
                self.text(t, x, 100.0, 3.0, *colour);
                x += text_width(t, 3.0) + 24.0;
            }
        }
        if let Style::Overlay { camera_label, .. } = style {
            self.text_centered(camera_label, 62.0, 2.0, HUD_TEXT);
        }

        let banner = banner_for(hud.run, hud.pause, hud.input);
        let overlay = overlay_for(hud.run, hud.pause, banner);
        if let Some(t) = overlay.text() {
            self.rect(140.0, 300.0, WIDTH - 280.0, 72.0, PANEL);
            self.text_centered(&t, 322.0, 4.0, OVERLAY_TEXT);
        }
        // The panel takes the bottom strip: the banner and prompts move up.
        let lift = if hud.panel.is_some() { HEIGHT - crate::panel::TOP } else { 0.0 };
        if let Some(t) = banner.text() {
            self.rect(0.0, 570.0 - 2.0 * lift, WIDTH, 64.0, BANNER);
            self.text_centered(&t, 588.0 - 2.0 * lift, 4.0, WHITE);
        }
        self.prompt(hud.prompt, HEIGHT - lift);
        if let Some(panel) = &hud.panel {
            crate::panel::draw(self, panel);
        }

        Drawn {
            banner,
            overlay,
            roll_deg: roll.to_degrees(),
            pitch_deg: pitch.to_degrees(),
            airspeed_kt: hud.airspeed_kt,
            altitude_agl_ft: hud.altitude_agl_ft,
            heading_deg: hud.heading_deg,
            vertical_speed_fpm: hud.vertical_speed_fpm,
            engine_n1_pct: hud.engine_n1_pct,
            pitch_trim_pct: hud.pitch_trim_pct,
            vertices: self.verts.len() as u32,
        }
    }

    // ------------------------------------------------------------ geometry --

    /// Sky/ground split by the horizon line, rotated by roll and shifted by
    /// pitch, plus the pitch ladder. Centred coordinates, y up: a right roll
    /// raises the right end of the horizon, nose-up moves it down.
    fn horizon(&mut self, roll: f32, pitch: f32, px_per_deg: f32, filled: bool) {
        let (s, c) = roll.sin_cos();
        let d = [c, s]; // along the horizon
        let n = [-s, c]; // towards the sky
        let pitch_px = pitch.to_degrees() * px_per_deg;
        let h0 = [-n[0] * pitch_px, -n[1] * pitch_px];
        let line = if filled { WHITE } else { HUD_TEXT };
        if filled {
            let screen = [[-640.0, -360.0], [640.0, -360.0], [640.0, 360.0], [-640.0, 360.0]];
            let side = |p: [f32; 2]| (p[0] - h0[0]) * n[0] + (p[1] - h0[1]) * n[1];
            let sky = clip(&screen, |p| side(p));
            let ground = clip(&screen, |p| -side(p));
            self.fan(&sky, SKY);
            self.fan(&ground, GROUND);
        }
        self.line_c(h0, d, n, 900.0, 3.0, line);
        let mut k = -30;
        while k <= 30 {
            if k != 0 {
                let off = k as f32 * px_per_deg;
                if (off - pitch_px).abs() < 230.0 {
                    let centre = [h0[0] + n[0] * off, h0[1] + n[1] * off];
                    let half = if k % 10 == 0 { 70.0 } else { 35.0 };
                    self.line_c(centre, d, n, half, 2.0, line);
                }
            }
            k += 5;
        }
    }

    /// Fixed aircraft reference (does not roll or pitch).
    fn aircraft_symbol(&mut self) {
        let (cx, cy) = (WIDTH / 2.0, HEIGHT / 2.0);
        self.rect(cx - 160.0, cy - 3.0, 100.0, 6.0, SYMBOL);
        self.rect(cx + 60.0, cy - 3.0, 100.0, 6.0, SYMBOL);
        self.rect(cx - 66.0, cy - 3.0, 6.0, 20.0, SYMBOL);
        self.rect(cx + 60.0, cy - 3.0, 6.0, 20.0, SYMBOL);
        self.rect(cx - 5.0, cy - 5.0, 10.0, 10.0, SYMBOL);
    }

    fn prompt(&mut self, lines: &[String], bottom: f32) {
        const SCALE: f32 = 2.0;
        const COLS: usize = 100;
        let mut wrapped: Vec<String> = Vec::new();
        for l in lines.iter().rev().take(2).rev() {
            wrapped.extend(wrap(&l.to_ascii_uppercase(), COLS));
        }
        let keep = wrapped.len().saturating_sub(4);
        let wrapped = &wrapped[keep..];
        if wrapped.is_empty() {
            return;
        }
        let line_h = 9.0 * SCALE;
        let top = bottom - 8.0 - wrapped.len() as f32 * line_h;
        self.rect(0.0, top - 6.0, WIDTH, bottom - top + 6.0, PANEL);
        for (i, l) in wrapped.iter().enumerate() {
            self.text(l, 20.0, top + i as f32 * line_h, SCALE, PROMPT_TEXT);
        }
    }

    // ---------------------------------------------------------- primitives --

    fn push(&mut self, x: f32, y: f32, color: Rgb) {
        if self.verts.len() < MAX_VERTICES {
            self.verts.push(Vertex { pos: [x / (WIDTH / 2.0) - 1.0, 1.0 - y / (HEIGHT / 2.0)], color });
        }
    }

    fn tri_px(&mut self, a: [f32; 2], b: [f32; 2], c: [f32; 2], color: Rgb) {
        if self.verts.len() + 3 > MAX_VERTICES {
            return;
        }
        for p in [a, b, c] {
            self.push(p[0], p[1], color);
        }
    }

    /// Triangle in pixels (top-left origin).
    pub fn tri(&mut self, a: [f32; 2], b: [f32; 2], c: [f32; 2], color: Rgb) {
        self.tri_px(a, b, c, color);
    }

    /// Axis-aligned rectangle in pixels (top-left origin).
    pub fn rect(&mut self, x: f32, y: f32, w: f32, h: f32, color: Rgb) {
        let (a, b, c, d) = ([x, y], [x + w, y], [x + w, y + h], [x, y + h]);
        self.tri_px(a, b, c, color);
        self.tri_px(a, c, d, color);
    }

    /// Convex polygon in centred y-up coordinates.
    fn fan(&mut self, poly: &[[f32; 2]], color: Rgb) {
        let px = |p: [f32; 2]| [p[0] + WIDTH / 2.0, HEIGHT / 2.0 - p[1]];
        for i in 1..poly.len().saturating_sub(1) {
            self.tri_px(px(poly[0]), px(poly[i]), px(poly[i + 1]), color);
        }
    }

    /// A thick line centred at `c` along `d` (unit), half-length `half`,
    /// thickness `t`, in centred y-up coordinates.
    fn line_c(&mut self, c: [f32; 2], d: [f32; 2], n: [f32; 2], half: f32, t: f32, color: Rgb) {
        let h = t / 2.0;
        let corner = |a: f32, b: f32| [c[0] + d[0] * a + n[0] * b, c[1] + d[1] * a + n[1] * b];
        let q = [corner(-half, -h), corner(half, -h), corner(half, h), corner(-half, h)];
        self.fan(&q, color);
    }

    /// Text at pixel (x, y) = top-left of the first glyph; 5x7 glyphs scaled
    /// by `scale`, advance 6 * scale. Lit runs in a row become one rectangle.
    pub fn text(&mut self, s: &str, x: f32, y: f32, scale: f32, color: Rgb) {
        for (i, ch) in s.chars().enumerate() {
            let gx = x + i as f32 * 6.0 * scale;
            for (r, bits) in glyph(ch).iter().enumerate() {
                let mut col = 0;
                while col < 5 {
                    if bits & (0x10 >> col) == 0 {
                        col += 1;
                        continue;
                    }
                    let start = col;
                    while col < 5 && bits & (0x10 >> col) != 0 {
                        col += 1;
                    }
                    self.rect(
                        gx + start as f32 * scale,
                        y + r as f32 * scale,
                        (col - start) as f32 * scale,
                        scale,
                        color,
                    );
                }
            }
        }
    }

    pub fn text_centered(&mut self, s: &str, y: f32, scale: f32, color: Rgb) {
        let x = ((WIDTH - text_width(s, scale)) / 2.0).max(0.0);
        self.text(s, x, y, scale, color);
    }

    // ------------------------------------------------------- CPU raster --

    /// Colour of pixel centre (x, y) after drawing every triangle in order
    /// over `clear`. Offline reference for tests; not used at run time.
    pub fn color_at(&self, x: f32, y: f32, clear: Rgb) -> Rgb {
        let p = [(x + 0.5) / (WIDTH / 2.0) - 1.0, 1.0 - (y + 0.5) / (HEIGHT / 2.0)];
        let mut out = clear;
        for t in self.verts.chunks_exact(3) {
            if inside(p, t[0].pos, t[1].pos, t[2].pos) {
                out = t[0].color;
            }
        }
        out
    }
}

/// The HUD trim readout: `TRIM +12 UP`, `TRIM  -5 DN`, `TRIM   0`.
pub fn trim_text(pct: i16) -> String {
    match pct {
        0 => "TRIM   0".to_string(),
        p if p > 0 => format!("TRIM {:+3} UP", p),
        p => format!("TRIM {:+3} DN", p),
    }
}

pub fn text_width(s: &str, scale: f32) -> f32 {
    let n = s.chars().count() as f32;
    if n == 0.0 {
        0.0
    } else {
        n * 6.0 * scale - scale
    }
}

fn wrap(s: &str, cols: usize) -> Vec<String> {
    let mut out = Vec::new();
    let mut line = String::new();
    for word in s.split_whitespace() {
        if !line.is_empty() && line.len() + 1 + word.len() > cols {
            out.push(std::mem::take(&mut line));
        }
        if !line.is_empty() {
            line.push(' ');
        }
        line.push_str(&word[..word.len().min(cols)]);
    }
    if !line.is_empty() {
        out.push(line);
    }
    out
}

fn inside(p: [f32; 2], a: [f32; 2], b: [f32; 2], c: [f32; 2]) -> bool {
    let e = |u: [f32; 2], v: [f32; 2]| (v[0] - u[0]) * (p[1] - u[1]) - (v[1] - u[1]) * (p[0] - u[0]);
    let (d0, d1, d2) = (e(a, b), e(b, c), e(c, a));
    (d0 >= 0.0 && d1 >= 0.0 && d2 >= 0.0) || (d0 <= 0.0 && d1 <= 0.0 && d2 <= 0.0)
}

/// Sutherland-Hodgman: keep the part of a convex polygon where `f >= 0`.
fn clip(poly: &[[f32; 2]], f: impl Fn([f32; 2]) -> f32) -> Vec<[f32; 2]> {
    let mut out = Vec::with_capacity(poly.len() + 2);
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
        let (fa, fb) = (f(a), f(b));
        if fa >= 0.0 {
            out.push(a);
        }
        if (fa >= 0.0) != (fb >= 0.0) {
            let t = fa / (fa - fb);
            out.push([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]);
        }
    }
    out
}

/// 5x7 glyph rows, top first; bit 4 is the leftmost column. Lower case is
/// drawn as upper case; anything unknown as '?'.
pub fn glyph(c: char) -> [u8; 7] {
    match c.to_ascii_uppercase() {
        ' ' => [0; 7],
        '0' => [0b01110, 0b10001, 0b10011, 0b10101, 0b11001, 0b10001, 0b01110],
        '1' => [0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110],
        '2' => [0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0b01000, 0b11111],
        '3' => [0b11111, 0b00010, 0b00100, 0b00010, 0b00001, 0b10001, 0b01110],
        '4' => [0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010],
        '5' => [0b11111, 0b10000, 0b11110, 0b00001, 0b00001, 0b10001, 0b01110],
        '6' => [0b00110, 0b01000, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110],
        '7' => [0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000, 0b01000],
        '8' => [0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110],
        '9' => [0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00010, 0b01100],
        'A' => [0b01110, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001],
        'B' => [0b11110, 0b10001, 0b10001, 0b11110, 0b10001, 0b10001, 0b11110],
        'C' => [0b01110, 0b10001, 0b10000, 0b10000, 0b10000, 0b10001, 0b01110],
        'D' => [0b11100, 0b10010, 0b10001, 0b10001, 0b10001, 0b10010, 0b11100],
        'E' => [0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b11111],
        'F' => [0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b10000],
        'G' => [0b01110, 0b10001, 0b10000, 0b10111, 0b10001, 0b10001, 0b01111],
        'H' => [0b10001, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001],
        'I' => [0b01110, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110],
        'J' => [0b00111, 0b00010, 0b00010, 0b00010, 0b00010, 0b10010, 0b01100],
        'K' => [0b10001, 0b10010, 0b10100, 0b11000, 0b10100, 0b10010, 0b10001],
        'L' => [0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b11111],
        'M' => [0b10001, 0b11011, 0b10101, 0b10101, 0b10001, 0b10001, 0b10001],
        'N' => [0b10001, 0b10001, 0b11001, 0b10101, 0b10011, 0b10001, 0b10001],
        'O' => [0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110],
        'P' => [0b11110, 0b10001, 0b10001, 0b11110, 0b10000, 0b10000, 0b10000],
        'Q' => [0b01110, 0b10001, 0b10001, 0b10001, 0b10101, 0b10010, 0b01101],
        'R' => [0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010, 0b10001],
        'S' => [0b01111, 0b10000, 0b10000, 0b01110, 0b00001, 0b00001, 0b11110],
        'T' => [0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100],
        'U' => [0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110],
        'V' => [0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01010, 0b00100],
        'W' => [0b10001, 0b10001, 0b10001, 0b10101, 0b10101, 0b10101, 0b01010],
        'X' => [0b10001, 0b10001, 0b01010, 0b00100, 0b01010, 0b10001, 0b10001],
        'Y' => [0b10001, 0b10001, 0b10001, 0b01010, 0b00100, 0b00100, 0b00100],
        'Z' => [0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b10000, 0b11111],
        '-' => [0, 0, 0, 0b11111, 0, 0, 0],
        '+' => [0, 0b00100, 0b00100, 0b11111, 0b00100, 0b00100, 0],
        '.' => [0, 0, 0, 0, 0, 0b01100, 0b01100],
        ',' => [0, 0, 0, 0, 0b01100, 0b00100, 0b01000],
        ':' => [0, 0b01100, 0b01100, 0, 0b01100, 0b01100, 0],
        ';' => [0, 0b01100, 0b01100, 0, 0b01100, 0b00100, 0b01000],
        '(' => [0b00010, 0b00100, 0b01000, 0b01000, 0b01000, 0b00100, 0b00010],
        ')' => [0b01000, 0b00100, 0b00010, 0b00010, 0b00010, 0b00100, 0b01000],
        '/' => [0, 0b00001, 0b00010, 0b00100, 0b01000, 0b10000, 0],
        '%' => [0b11000, 0b11001, 0b00010, 0b00100, 0b01000, 0b10011, 0b00011],
        '!' => [0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0, 0b00100],
        '=' => [0, 0, 0b11111, 0, 0b11111, 0, 0],
        '<' => [0b00010, 0b00100, 0b01000, 0b10000, 0b01000, 0b00100, 0b00010],
        '>' => [0b01000, 0b00100, 0b00010, 0b00001, 0b00010, 0b00100, 0b01000],
        '\'' => [0b01100, 0b00100, 0b01000, 0, 0, 0, 0],
        '"' => [0b01010, 0b01010, 0b01010, 0, 0, 0, 0],
        '~' => [0, 0, 0b01000, 0b10101, 0b00010, 0, 0],
        '#' => [0b01010, 0b01010, 0b11111, 0b01010, 0b11111, 0b01010, 0b01010],
        '*' => [0, 0b00100, 0b10101, 0b01110, 0b10101, 0b00100, 0],
        '_' => [0, 0, 0, 0, 0, 0, 0b11111],
        _ => [0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0, 0b00100],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn health(status: InputStatus, loss_ms: u64) -> InputHealth {
        InputHealth { status, loss_ms }
    }

    fn hud(prompt: &[String]) -> HudInput<'_> {
        HudInput {
            roll_rad: 0.0,
            pitch_rad: 0.0,
            heading_rad: 0.0,
            airspeed_kt: 0,
            altitude_agl_ft: 0,
            heading_deg: 0,
            vertical_speed_fpm: 0,
            engine_n1_pct: 30,
            pitch_trim_pct: None,
            config: None,
            run: RunState::Running,
            pause: None,
            input: health(InputStatus::Valid, 0),
            prompt,
            panel: None,
        }
    }

    const LOST: InputStatus = InputStatus::Unavailable(gx_sim::UnavailableReason::Disconnected);

    #[test]
    fn input002_banner_sequence() {
        let r = RunState::Running;
        assert_eq!(banner_for(r, None, health(InputStatus::Valid, 0)), Banner::None);
        assert_eq!(banner_for(RunState::Stopped, None, health(InputStatus::NoInput, 0)), Banner::NoInput);
        let b = banner_for(r, None, health(LOST, 3_400));
        assert_eq!(b, Banner::StickLost { seconds: 3 });
        assert_eq!(b.text().unwrap(), "STICK LOST - 3 S");
        assert_eq!(banner_for(r, None, health(InputStatus::Stale, 120)), Banner::StickLost { seconds: 0 });
        let p = Some(PauseReason::InputLoss);
        let b = banner_for(RunState::Paused, p, health(LOST, 10_000));
        assert_eq!(b, Banner::PausedStickLost);
        assert_eq!(b.text().unwrap(), "PAUSED - PRESS TRIGGER WHEN THE STICK IS BACK");
        assert_eq!(overlay_for(RunState::Paused, p, b), Overlay::None, "banner already says it");
        // The stick is back: banner clears, the pause stays until TRIGGER.
        let b = banner_for(RunState::Paused, p, health(InputStatus::Valid, 0));
        assert_eq!(b, Banner::None);
        let o = overlay_for(RunState::Paused, p, b);
        assert_eq!(o.text().unwrap(), "PAUSED (STICK LOST) - PULL TRIGGER");
    }

    #[test]
    fn overlay_whenever_not_running() {
        let v = Banner::None;
        assert_eq!(overlay_for(RunState::Running, None, v), Overlay::None);
        assert_eq!(overlay_for(RunState::Stopped, None, v), Overlay::Ready);
        assert_eq!(
            overlay_for(RunState::Paused, Some(PauseReason::User), v).text().unwrap(),
            "PAUSED (USER) - PULL TRIGGER"
        );
        assert_eq!(overlay_for(RunState::Paused, Some(PauseReason::Reset), v).name(), "paused_reset");
    }

    #[test]
    fn horizon_follows_pitch_and_roll() {
        let mut s = Scene::new();
        let mut h = hud(&[]);
        s.build(&h);
        // Level: sky above the centre, ground below (away from the symbol).
        assert_eq!(s.color_at(640.0, 250.0, [0.0; 3]), SKY);
        assert_eq!(s.color_at(640.0, 470.0, [0.0; 3]), GROUND);
        // Nose up 10 deg: horizon 80 px lower, so 40 px below centre is sky.
        h.pitch_rad = 10f64.to_radians();
        s.build(&h);
        assert_eq!(s.color_at(400.0, 400.0, [0.0; 3]), SKY);
        assert_eq!(s.color_at(400.0, 460.0, [0.0; 3]), GROUND);
        // Right roll 30 deg: right side of the horizon rises.
        h.pitch_rad = 0.0;
        h.roll_rad = 30f64.to_radians();
        s.build(&h);
        assert_eq!(s.color_at(1000.0, 300.0, [0.0; 3]), GROUND, "right of centre, above the line");
        assert_eq!(s.color_at(280.0, 420.0, [0.0; 3]), SKY, "left of centre, below the line");
    }

    #[test]
    fn banner_and_overlay_are_drawn_and_reported() {
        let mut s = Scene::new();
        let mut h = hud(&[]);
        h.input = health(LOST, 4_000);
        let d = s.build(&h);
        assert_eq!(d.banner, Banner::StickLost { seconds: 4 });
        assert_eq!(d.overlay, Overlay::None);
        assert_eq!(s.color_at(4.0, 600.0, [0.0; 3]), BANNER);
        h.run = RunState::Paused;
        h.pause = Some(PauseReason::User);
        h.input = health(InputStatus::Valid, 0);
        let d = s.build(&h);
        assert_eq!((d.banner, d.overlay), (Banner::None, Overlay::Paused(PauseReason::User)));
        assert_eq!(s.color_at(150.0, 310.0, [0.0; 3]), PANEL);
        assert_ne!(s.color_at(4.0, 600.0, [0.0; 3]), BANNER);
    }

    #[test]
    fn text_glyphs_and_bounds() {
        let mut s = Scene::new();
        s.text("1", 0.0, 0.0, 1.0, WHITE);
        // '1' = 00100 / 01100 / 00100 x4 / 01110: 1+1+1+1+1+1+1 runs.
        assert_eq!(s.vertices().len(), 7 * 6);
        let p = ["x".repeat(500)];
        let d = s.build(&hud(&p));
        assert!(d.vertices as usize <= MAX_VERTICES);
        assert_eq!(wrap("aa bb cc", 5), vec!["aa bb", "cc"]);
        assert_eq!(text_width("AB", 2.0), 22.0);
        for c in "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-+.,:;()/%!=<>'\"~#*_".chars() {
            assert_ne!(glyph(c), glyph('\u{1}'), "{c} has its own glyph");
        }
    }

    #[test]
    fn s4_overlay_draws_lines_only_and_the_camera_name() {
        let mut s = Scene::new();
        let h = hud(&[]);
        s.build_overlay(&h, 12.0, "CHASE CAM");
        // No sky/ground fill: away from the HUD elements the clear colour shows.
        let marker = [0.5, 0.5, 0.5];
        assert_eq!(s.color_at(400.0, 250.0, marker), marker);
        assert_eq!(s.color_at(400.0, 470.0, marker), marker);
        // The horizon line is drawn (green) through the centre row.
        assert_eq!(s.color_at(300.0, 360.0, marker), HUD_TEXT);
        // Ladder at 12 px/deg: the +10 deg line is 120 px above the horizon.
        assert_eq!(s.color_at(640.0 - 50.0, 360.0 - 120.0, marker), HUD_TEXT);
        // "C" of the centred camera label on the second top-bar row.
        let camera_x = (WIDTH - text_width("CHASE CAM", 2.0)) * 0.5;
        assert_eq!(s.color_at(camera_x + 3.0, 62.5, marker), HUD_TEXT);
        // S3 build still fills.
        s.build(&h);
        assert_eq!(s.color_at(400.0, 250.0, marker), SKY);
    }

    #[test]
    fn hud_numbers_render() {
        let mut s = Scene::new();
        let mut h = hud(&[]);
        h.airspeed_kt = 123;
        h.heading_deg = 270;
        h.altitude_agl_ft = 1500;
        h.vertical_speed_fpm = -750;
        h.engine_n1_pct = 91;
        let d = s.build(&h);
        assert_eq!((d.airspeed_kt, d.heading_deg, d.altitude_agl_ft, d.vertical_speed_fpm, d.engine_n1_pct), (123, 270, 1500, -750, 91));
        assert!(
            (16..220).any(|x| (58..82).any(|y| s.color_at(x as f32, y as f32, [0.0; 3]) == HUD_TEXT)),
            "N1 label is visible"
        );
        assert!(
            (900..1264).any(|x| (58..82).any(|y| s.color_at(x as f32, y as f32, [0.0; 3]) == HUD_TEXT)),
            "vertical-speed FPM label is visible"
        );
        // "IAS" starts at x=16, y=12, scale 4: the 'I' top bar is lit.
        assert_eq!(s.color_at(17.0 + 4.0, 13.0, [0.0; 3]), HUD_TEXT);
    }

    #[test]
    fn trim_readout_renders_only_when_given() {
        assert_eq!(trim_text(12), "TRIM +12 UP");
        assert_eq!(trim_text(-5), "TRIM  -5 DN");
        assert_eq!(trim_text(0), "TRIM   0");
        let lit = |s: &Scene| (16..200).any(|x| (100..122).any(|y| s.color_at(x as f32, y as f32, [0.0; 3]) == HUD_TEXT));
        let mut s = Scene::new();
        let mut h = hud(&[]);
        let d = s.build(&h);
        assert_eq!(d.pitch_trim_pct, None);
        assert!(!lit(&s), "no trim readout without a value");
        h.pitch_trim_pct = Some(-23);
        let d = s.build_overlay(&h, 12.0, "CHASE CAM");
        assert_eq!(d.pitch_trim_pct, Some(-23));
        assert!(lit(&s), "TRIM readout is visible under N1");
    }

    #[test]
    fn config_strip_renders_right_under_vs_with_caution_colours() {
        let c = ConfigReadout { gear: GearText::Down, gear_caution: false, flaps_label: 5, flaps_moving: true, speedbrake_pct: 50 };
        let t = c.texts();
        assert_eq!([t[0].0.as_str(), t[1].0.as_str(), t[2].0.as_str()], ["GEAR DN", "FLAPS 5", "SPDBRK 50%"]);
        assert_eq!([t[0].1, t[1].1, t[2].1], [HUD_TEXT, OVERLAY_TEXT, OVERLAY_TEXT]);
        assert_eq!(ConfigReadout { speedbrake_pct: 0, ..c }.texts()[2].0, "", "stowed speedbrake is hidden");
        assert_eq!(ConfigReadout { gear: GearText::Transit, ..c }.texts()[0].0, "GEAR ---");
        let lit = |s: &Scene, colour: Rgb| (700..1280).any(|x| (100..122).any(|y| s.color_at(x as f32, y as f32, [0.0; 3]) == colour));
        let mut s = Scene::new();
        let mut h = hud(&[]);
        s.build(&h);
        assert!(!lit(&s, OVERLAY_TEXT), "nothing without a readout");
        h.config = Some(c);
        s.build_overlay(&h, 12.0, "CHASE CAM");
        assert!(lit(&s, HUD_TEXT) && lit(&s, OVERLAY_TEXT), "green gear, amber flaps/speedbrake");
    }
}
