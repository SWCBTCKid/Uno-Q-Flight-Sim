//! The autopilot panel: a strip of mouse buttons along the bottom of the
//! screen (the AP-002 mouse-panel contract). One layout serves
//! both sides: the I/O side hit-tests clicks with [`hit`], the scene draws
//! it with [`draw`] from the core's [`ApStatus`]. Pure and host-testable.
//!
//! `AP | LVL | HDG - 270 + | ALT - 3000 + | SPD - 250 + | ILS | status | EXIT`
//!
//! A mode button is green while its mode is active, amber while armed. `-`
//! / `+` step the target ([`gx_sim::autopilot::HEADING_STEP_DEG`] etc.); the
//! wheel over a value steps it finely; a click on a value syncs it to the
//! current heading, altitude or speed. EXIT (AP-002, far right, away from
//! the mode buttons) reads `EXIT?` on amber while armed for the confirming
//! click. The status column holds two lines: the vertical mode, then the
//! approach with its glideslope deviation (or a warning).

use crate::scene::{text_width, Rgb, Scene, HEIGHT};
use gx_sim::autopilot::{Lateral, Vertical, ALTITUDE_STEP_FT, HEADING_STEP_DEG, SPEED_STEP_KT};
use gx_sim::{ApAction, ApStatus};

pub const TOP: f32 = HEIGHT - 64.0;
const BUTTON_Y: f32 = TOP + 10.0;
const BUTTON_H: f32 = 44.0;
const SCALE: f32 = 2.0;

pub const BG: Rgb = [0.03, 0.04, 0.06];
pub const OFF: Rgb = [0.16, 0.18, 0.22];
pub const HOVER: Rgb = [0.26, 0.29, 0.34];
pub const ACTIVE: Rgb = [0.08, 0.50, 0.18];
pub const ARMED: Rgb = [0.62, 0.42, 0.0];
pub const LABEL: Rgb = [0.95, 0.95, 0.95];
pub const VALUE: Rgb = [0.35, 1.0, 0.45];
pub const VALUE_BG: Rgb = [0.0, 0.0, 0.0];
pub const INFO: Rgb = [0.35, 1.0, 0.45];
pub const INFO_ARMED: Rgb = [1.0, 0.72, 0.0];
pub const WARN: Rgb = [1.0, 0.25, 0.2];
/// An ILS leg neither armed nor captured.
pub const OFF_TEXT: Rgb = [0.55, 0.58, 0.62];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Target {
    Heading,
    Altitude,
    Speed,
}

impl Target {
    /// (`-`/`+` step, wheel step).
    pub fn steps(self) -> (f32, f32) {
        match self {
            Self::Heading => (HEADING_STEP_DEG, 1.0),
            Self::Altitude => (ALTITUDE_STEP_FT, 100.0),
            Self::Speed => (SPEED_STEP_KT, 1.0),
        }
    }
}

/// What a point of the panel does.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Hit {
    Action(ApAction),
    /// `-` (-1) or `+` (+1).
    Step(Target, i8),
    /// The value box: click syncs, the wheel fine-steps.
    Value(Target),
    /// AP-002 (interface): EXIT ends the run; the first click arms it
    /// (`PanelView::exit_armed`), a second click within the arm window
    /// confirms. Graphics places the button, Platform acts on it.
    Exit,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Button {
    pub x: f32,
    pub w: f32,
    pub hit: Hit,
}

impl Button {
    pub fn contains(&self, x: f32, y: f32) -> bool {
        x >= self.x && x < self.x + self.w && (BUTTON_Y..BUTTON_Y + BUTTON_H).contains(&y)
    }
}

const fn b(x: f32, w: f32, hit: Hit) -> Button {
    Button { x, w, hit }
}

/// AP-002 layout: AP 72 px (the old gap at x = 84..96 before LVL is kept),
/// other mode buttons 60 px, steps 32 px, value boxes sized to their widest
/// value at scale 2.5 (`359`, `45000`, `350`) plus margin; 6 px inside a
/// group, 14 px between groups. That frees a 256 px status column (21
/// glyphs at scale 2) and the EXIT button at the right edge.
pub const BUTTONS: [Button; 16] = [
    b(12.0, 72.0, Hit::Action(ApAction::Master)),
    b(96.0, 60.0, Hit::Action(ApAction::Level)),
    b(170.0, 60.0, Hit::Action(ApAction::Heading)),
    b(236.0, 32.0, Hit::Step(Target::Heading, -1)),
    b(274.0, 64.0, Hit::Value(Target::Heading)),
    b(344.0, 32.0, Hit::Step(Target::Heading, 1)),
    b(390.0, 60.0, Hit::Action(ApAction::Altitude)),
    b(456.0, 32.0, Hit::Step(Target::Altitude, -1)),
    b(494.0, 88.0, Hit::Value(Target::Altitude)),
    b(588.0, 32.0, Hit::Step(Target::Altitude, 1)),
    b(634.0, 60.0, Hit::Action(ApAction::Speed)),
    b(700.0, 32.0, Hit::Step(Target::Speed, -1)),
    b(738.0, 64.0, Hit::Value(Target::Speed)),
    b(808.0, 32.0, Hit::Step(Target::Speed, 1)),
    b(854.0, 60.0, Hit::Action(ApAction::Approach)),
    b(EXIT_X, 72.0, Hit::Exit),
];
/// Left edge of the status column (14 px after the ILS button).
pub const INFO_X: f32 = 928.0;
const EXIT_X: f32 = 1196.0;
/// The status column ends 12 px before EXIT.
pub const INFO_END: f32 = EXIT_X - 12.0;

/// The button under virtual-screen pixel (x, y), if any.
pub fn hit(x: f32, y: f32) -> Option<Hit> {
    BUTTONS.iter().find(|b| b.contains(x, y)).map(|b| b.hit)
}

/// Everything the panel shows.
#[derive(Clone, Debug, PartialEq)]
pub struct PanelView {
    pub status: ApStatus,
    /// Name of the armed or captured approach (`28R`).
    pub approach: Option<String>,
    /// Cursor in virtual-screen pixels; `None` hides it.
    pub cursor: Option<(f32, f32)>,
    /// False: no mouse could be opened (the panel says so).
    pub mouse: bool,
    /// Blink phase for warnings (frame counter / 15 is fine).
    pub blink: bool,
    /// AP-002 (interface): EXIT clicked once and waiting for the confirming
    /// click (from `Board::exit_armed`).
    pub exit_armed: bool,
}

fn label(hit: Hit) -> &'static str {
    match hit {
        Hit::Action(ApAction::Master) => "AP",
        Hit::Action(ApAction::Level) => "LVL",
        Hit::Action(ApAction::Heading) => "HDG",
        Hit::Action(ApAction::Altitude) => "ALT",
        Hit::Action(ApAction::Speed) => "SPD",
        Hit::Action(ApAction::Approach) => "ILS",
        Hit::Action(ApAction::None) => "",
        Hit::Step(_, d) if d < 0 => "-",
        Hit::Step(..) => "+",
        Hit::Value(_) => "",
        Hit::Exit => "EXIT",
    }
}

/// `None` = off, `Some(false)` = active (green), `Some(true)` = armed (amber).
fn lit(hit: Hit, s: &ApStatus) -> Option<bool> {
    let on = |c: bool| c.then_some(false);
    match hit {
        Hit::Action(ApAction::Master) => on(s.engaged),
        Hit::Action(ApAction::Level) => on(s.engaged && s.lateral == Lateral::WingsLevel),
        Hit::Action(ApAction::Heading) => on(s.engaged && s.lateral == Lateral::Heading),
        Hit::Action(ApAction::Altitude) => on(s.engaged && matches!(s.vertical, Vertical::AltSelect | Vertical::AltHold | Vertical::LevelOff)),
        Hit::Action(ApAction::Speed) => on(s.autothrottle),
        Hit::Action(ApAction::Approach) => {
            if s.engaged && (s.lateral == Lateral::Localizer || s.vertical == Vertical::Glideslope) {
                Some(false)
            } else if s.loc_armed || s.gs_armed {
                Some(true)
            } else {
                None
            }
        }
        _ => None,
    }
}

fn value_text(t: Target, s: &ApStatus) -> String {
    match t {
        Target::Heading => format!("{:03}", (s.input.heading_deg.round() as i32).rem_euclid(360)),
        Target::Altitude => format!("{}", s.input.altitude_ft.round() as i32),
        Target::Speed => format!("{}", s.input.speed_kt.round() as i32),
    }
}

/// One status line: text segments, each with its colour, left to right.
pub type InfoLine = Vec<(String, Rgb)>;

/// The two status lines: the vertical mode, then the ILS or a warning
/// (which wins). On the ILS line each of `LOC` and `G/S` is green once
/// captured, amber while armed and grey otherwise, so a captured localizer
/// with the glideslope still armed reads at a glance (the 5x7 font has no
/// lower case to tell them apart). AP-002: the line ends with the height
/// above the glide path (`+900`, `-200`; 10 ft steps), green within
/// [`GS_DEV_OK_FT`], amber beyond; `G/S DES` (descending to meet the
/// glideslope from above) is amber on the mode line.
pub fn info_lines(v: &PanelView) -> [InfoLine; 2] {
    use gx_sim::autopilot::ApWarning;
    let s = &v.status;
    let mode = if !s.engaged {
        (if s.autothrottle { "A/T ONLY" } else { "AP OFF" }).to_string()
    } else {
        match s.vertical {
            Vertical::LevelOff => "LEVEL OFF".to_string(),
            Vertical::AltHold => format!("ALT {}", s.hold_ft.round() as i32),
            Vertical::AltSelect => "ALT SEL".to_string(),
            Vertical::Glideslope => "G/S".to_string(),
            Vertical::GsDescent => "G/S DES".to_string(),
        }
    };
    let mode_colour = if s.engaged && s.vertical == Vertical::GsDescent { INFO_ARMED } else { INFO };
    let second = match (s.warning, v.mouse) {
        (Some(w), _) => vec![(
            match w {
                ApWarning::ApDisconnect => "AP DISCONNECT",
                ApWarning::AtDisconnect => "A/T DISCONNECT",
                ApWarning::NotAvailable => "NOT AVAILABLE",
                ApWarning::NoIls => "NO ILS AHEAD",
                ApWarning::InputLossLevel => "INPUT LOSS - LVL",
            }
            .to_string(),
            WARN,
        )],
        (None, false) => vec![("NO MOUSE".to_string(), INFO_ARMED)],
        // The name comes from the render side (HUD_APPROACHES); without it
        // the line still shows the legs and the deviation, as `ILS`.
        (None, true) if v.approach.is_some() || s.approach.is_some() || s.gs_dev_ft.is_some() => {
            let name = v.approach.as_deref().unwrap_or("ILS");
            let state = |captured: bool, armed: bool| if captured { INFO } else if armed { INFO_ARMED } else { OFF_TEXT };
            let armed = s.loc_armed || s.gs_armed;
            let mut line = vec![
                (format!("{name} "), if armed { INFO_ARMED } else { INFO }),
                ("LOC ".to_string(), state(s.lateral == Lateral::Localizer, s.loc_armed)),
                ("G/S".to_string(), state(s.vertical == Vertical::Glideslope, s.gs_armed)),
            ];
            if let Some(dev) = s.gs_dev_ft.filter(|d| d.is_finite()) {
                let ok = dev.abs() <= GS_DEV_OK_FT;
                line.push((format!(" {}", gs_dev_text(dev)), if ok { INFO } else { INFO_ARMED }));
            }
            line
        }
        (None, true) => Vec::new(),
    };
    [vec![(mode, mode_colour)], second]
}

/// Glide-path deviation shown green up to this size (the W5b band), amber
/// beyond.
pub const GS_DEV_OK_FT: f32 = 50.0;

/// `+900` / `-200` / `+0`: feet above the path, 10 ft steps, clamped to
/// +/-9990 so the line never outgrows the column.
pub fn gs_dev_text(dev_ft: f32) -> String {
    let tens = (dev_ft.clamp(-9_990.0, 9_990.0) / 10.0).round() as i32;
    format!("{:+}", tens * 10)
}

/// Draw the panel (and the cursor, last) into `scene`.
pub fn draw(scene: &mut Scene, v: &PanelView) {
    scene.rect(0.0, TOP, crate::scene::WIDTH, HEIGHT - TOP, BG);
    let hover = v.cursor.and_then(|(x, y)| hit(x, y));
    for button in &BUTTONS {
        let fill = match (button.hit, lit(button.hit, &v.status)) {
            (Hit::Value(_), _) => VALUE_BG,
            (Hit::Exit, _) if v.exit_armed => ARMED,
            (_, Some(false)) => ACTIVE,
            (_, Some(true)) => ARMED,
            (h, None) if hover == Some(h) => HOVER,
            _ => OFF,
        };
        scene.rect(button.x, BUTTON_Y, button.w, BUTTON_H, fill);
        let (text, colour) = match button.hit {
            Hit::Value(t) => (value_text(t, &v.status), VALUE),
            Hit::Exit if v.exit_armed => ("EXIT?".to_string(), LABEL),
            h => (label(h).to_string(), LABEL),
        };
        let scale = if matches!(button.hit, Hit::Value(_)) { 2.5 } else { SCALE };
        let x = button.x + (button.w - text_width(&text, scale)) / 2.0;
        let y = BUTTON_Y + (BUTTON_H - 7.0 * scale) / 2.0;
        scene.text(&text, x, y, scale, colour);
    }
    let [mode, second] = info_lines(v);
    let warn = v.status.warning.is_some();
    for (line, y, shown) in [(mode, BUTTON_Y + 4.0, true), (second, BUTTON_Y + 26.0, !warn || v.blink)] {
        if !shown {
            continue;
        }
        let mut x = INFO_X;
        for (text, colour) in &line {
            scene.text(text, x, y, SCALE, *colour);
            x += text.chars().count() as f32 * 6.0 * SCALE;
        }
    }
    if let Some((x, y)) = v.cursor {
        cursor(scene, x, y);
    }
}

/// An arrow pointer with its tip at (x, y): white, inside a black edge on
/// every side (it must read over the light sky as well as the dark strip).
fn cursor(scene: &mut Scene, x: f32, y: f32) {
    let white = [[x, y], [x, y + 18.0], [x + 12.0, y + 13.0]];
    let c = [x + 4.0, y + 31.0 / 3.0]; // centroid
    let edge = white.map(|p| [c[0] + (p[0] - c[0]) * 1.35, c[1] + (p[1] - c[1]) * 1.35]);
    scene.tri(edge[0], edge[1], edge[2], [0.0, 0.0, 0.0]);
    scene.tri(white[0], white[1], white[2], [1.0, 1.0, 1.0]);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn buttons_do_not_overlap_and_fit_the_strip() {
        for (i, a) in BUTTONS.iter().enumerate() {
            if a.hit == Hit::Exit {
                assert!(a.x >= INFO_END && a.x + a.w <= crate::scene::WIDTH - 12.0, "{a:?}");
            } else {
                assert!(a.x >= 12.0 && a.x + a.w + 14.0 <= INFO_X, "{a:?}");
            }
            let widest = match a.hit {
                Hit::Exit => "EXIT?",
                Hit::Value(Target::Heading) => "359",
                Hit::Value(Target::Altitude) => "45000",
                Hit::Value(Target::Speed) => "350",
                h => label(h),
            };
            let scale = if matches!(a.hit, Hit::Value(_)) { 2.5 } else { SCALE };
            assert!(text_width(widest, scale) + 8.0 <= a.w, "{a:?}: {widest} does not fit");
            for c in &BUTTONS[i + 1..] {
                assert!(a.x + a.w <= c.x || c.x + c.w <= a.x, "{a:?} overlaps {c:?}");
            }
        }
        assert!(BUTTON_Y + BUTTON_H <= HEIGHT);
    }

    #[test]
    fn every_status_line_fits_right_of_the_buttons() {
        use gx_sim::autopilot::ApWarning;
        let base = ApStatus { engaged: true, vertical: Vertical::AltHold, hold_ft: -99_999.0, gs_dev_ft: Some(-1e9), ..ApStatus::default() };
        let mut views = vec![PanelView { status: base, approach: Some("28R".into()), cursor: None, mouse: true, blink: true, exit_armed: false }];
        for w in [ApWarning::ApDisconnect, ApWarning::AtDisconnect, ApWarning::NotAvailable, ApWarning::NoIls, ApWarning::InputLossLevel] {
            views.push(PanelView { status: ApStatus { warning: Some(w), ..base }, ..views[0].clone() });
        }
        views.push(PanelView { mouse: false, ..views[0].clone() });
        views.push(PanelView { status: ApStatus { vertical: Vertical::GsDescent, ..base }, ..views[0].clone() });
        for v in &views {
            for line in info_lines(v) {
                let text: String = line.iter().map(|(t, _)| t.as_str()).collect();
                assert!(INFO_X + text_width(&text, SCALE) <= INFO_END, "{text:?} overflows");
            }
        }
    }

    #[test]
    fn hits_find_the_button_under_the_cursor() {
        let y = BUTTON_Y + 20.0;
        assert_eq!(hit(20.0, y), Some(Hit::Action(ApAction::Master)));
        assert_eq!(hit(250.0, y), Some(Hit::Step(Target::Heading, -1)));
        assert_eq!(hit(520.0, y), Some(Hit::Value(Target::Altitude)));
        assert_eq!(hit(860.0, y), Some(Hit::Action(ApAction::Approach)));
        assert_eq!(hit(1230.0, y), Some(Hit::Exit));
        assert_eq!(hit(1000.0, y), None, "the status column");
        assert_eq!(hit(90.0, y), None, "the gap between AP and LVL");
        assert_eq!(hit(20.0, TOP - 5.0), None, "above the strip");
    }

    #[test]
    fn modes_light_their_buttons_and_lines_name_them() {
        let mut s = ApStatus { engaged: true, lateral: Lateral::Heading, vertical: Vertical::AltHold, hold_ft: 4000.0, ..ApStatus::default() };
        assert_eq!(lit(Hit::Action(ApAction::Heading), &s), Some(false));
        assert_eq!(lit(Hit::Action(ApAction::Level), &s), None);
        s.loc_armed = true;
        s.gs_armed = true;
        assert_eq!(lit(Hit::Action(ApAction::Approach), &s), Some(true), "armed is amber");
        let v = PanelView { status: s, approach: Some("28R".into()), cursor: None, mouse: true, blink: true, exit_armed: false };
        let joined = |l: &InfoLine| l.iter().map(|(t, _)| t.as_str()).collect::<String>();
        let lines = info_lines(&v);
        assert_eq!(joined(&lines[0]), "ALT 4000");
        assert_eq!(joined(&lines[1]), "28R LOC G/S");
        assert_eq!(lines[1].iter().map(|l| l.1).collect::<Vec<_>>(), [INFO_ARMED; 3], "both legs armed");
        s.lateral = Lateral::Localizer;
        s.loc_armed = false;
        assert_eq!(lit(Hit::Action(ApAction::Approach), &s), Some(false));
        let v = PanelView { status: s, ..v };
        assert_eq!(info_lines(&v)[1].iter().map(|l| l.1).collect::<Vec<_>>(), [INFO_ARMED, INFO, INFO_ARMED], "LOC captured, G/S armed");
        s.warning = Some(gx_sim::autopilot::ApWarning::ApDisconnect);
        let v = PanelView { status: s, ..v };
        assert_eq!(info_lines(&v)[1], vec![("AP DISCONNECT".to_string(), WARN)]);
    }

    #[test]
    fn glideslope_deviation_and_descent_read_clearly() {
        assert_eq!([gs_dev_text(904.0), gs_dev_text(-196.0), gs_dev_text(-4.0), gs_dev_text(1e6)], ["+900", "-200", "+0", "+9990"]);
        let s = ApStatus { engaged: true, lateral: Lateral::Localizer, vertical: Vertical::GsDescent, gs_armed: true, approach: Some(0), gs_dev_ft: Some(904.0), ..ApStatus::default() };
        let v = PanelView { status: s, approach: Some("28R".into()), cursor: None, mouse: true, blink: true, exit_armed: false };
        let [mode, ils] = info_lines(&v);
        assert_eq!(mode, vec![("G/S DES".to_string(), INFO_ARMED)], "descending onto the path: amber");
        assert_eq!(ils.last().unwrap(), &(" +900".to_string(), INFO_ARMED), "far above: amber");
        let s = ApStatus { vertical: Vertical::Glideslope, gs_armed: false, gs_dev_ft: Some(-31.0), ..s };
        let [mode, ils] = info_lines(&PanelView { status: s, ..v.clone() });
        assert_eq!((mode[0].clone(), ils.last().unwrap().clone()), (("G/S".to_string(), INFO), (" -30".to_string(), INFO)));
        // No name list (HUD_APPROACHES unset): the legs and deviation still show.
        let [_, ils] = info_lines(&PanelView { approach: None, ..v.clone() });
        assert_eq!(ils.iter().map(|l| l.0.as_str()).collect::<String>(), "ILS LOC G/S +900");
        // No approach at all: no line, no deviation.
        let none = ApStatus { gs_dev_ft: None, approach: None, ..ApStatus::default() };
        assert!(info_lines(&PanelView { status: none, approach: None, ..v })[1].is_empty());
    }

    #[test]
    fn exit_reads_exit_and_turns_amber_with_a_question_mark_when_armed() {
        let exit = BUTTONS.iter().find(|b| b.hit == Hit::Exit).unwrap();
        let mut v = PanelView { status: ApStatus::default(), approach: None, cursor: None, mouse: true, blink: true, exit_armed: false };
        for (armed, fill, text) in [(false, OFF, "EXIT"), (true, ARMED, "EXIT?")] {
            v.exit_armed = armed;
            let mut scene = Scene::new();
            draw(&mut scene, &v);
            assert_eq!(scene.color_at(exit.x + 2.0, BUTTON_Y + 2.0, [0.5; 3]), fill);
            // The label is centred, so its first column moves with its length:
            // the E's left stroke sits exactly where `text` puts it.
            let x0 = exit.x + (exit.w - text_width(text, SCALE)) / 2.0;
            let y0 = BUTTON_Y + (BUTTON_H - 7.0 * SCALE) / 2.0;
            assert_eq!(scene.color_at(x0 + 0.5, y0 + 6.5, [0.5; 3]), LABEL, "{text}");
            assert_eq!(scene.color_at(x0 - 1.5, y0 + 6.5, [0.5; 3]), fill, "{text}");
        }
    }

    #[test]
    fn the_panel_draws_its_values_and_the_cursor() {
        let mut scene = Scene::new();
        let status = ApStatus { input: gx_sim::ApInput { heading_deg: 275.0, ..Default::default() }, ..ApStatus::default() };
        let v = PanelView { status, approach: None, cursor: Some((640.0, 300.0)), mouse: true, blink: false, exit_armed: false };
        draw(&mut scene, &v);
        assert_eq!(value_text(Target::Heading, &v.status), "275");
        assert_eq!(scene.color_at(641.0, 305.0, [0.0; 3]), [1.0, 1.0, 1.0], "cursor");
        assert_eq!(scene.color_at(14.0, BUTTON_Y + 2.0, [0.5; 3]), OFF, "AP button off");
    }
}
