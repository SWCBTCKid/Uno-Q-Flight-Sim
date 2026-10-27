//! AP-001 W3 read-only board probe of the autopilot panel mouse: opens the
//! mouse (`--mouse PATH`, default the first `/dev/input/by-id/*-event-mouse`)
//! with `gx_input::mouse::linux::MouseReader` exactly as the `gx-mouse`
//! thread does (read-only, nonblocking, never grabbed), and prints one JSON
//! line per report with the cursor in the virtual 1280x720 screen, then a
//! summary. Needs only the `input` group.
//!
//!     mouse_probe [--mouse PATH] [--seconds N]

#[cfg(target_os = "linux")]
fn main() {
    use gx_input::mouse::{find_mouse, linux::MouseReader, Pointer, DEFAULT_BY_ID, DEFAULT_SENSITIVITY};
    use std::time::Instant;
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut path = None;
    let mut seconds = 10u64;
    let mut it = args.iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--mouse" => path = it.next().map(std::path::PathBuf::from),
            "--seconds" => seconds = it.next().and_then(|v| v.parse().ok()).unwrap_or(10),
            other => {
                eprintln!("mouse_probe: unknown argument {other}");
                std::process::exit(2);
            }
        }
    }
    let Some(path) = path.or_else(|| find_mouse(std::path::Path::new(DEFAULT_BY_ID))) else {
        println!("{{\"k\":\"probe\",\"ok\":false,\"why\":\"no *-event-mouse\"}}");
        std::process::exit(1);
    };
    let t0 = Instant::now();
    let mut reader = match MouseReader::open(&path) {
        Ok(r) => r,
        Err(e) => {
            println!("{{\"k\":\"probe\",\"ok\":false,\"path\":\"{}\",\"why\":\"{e}\"}}", path.display());
            std::process::exit(1);
        }
    };
    println!(
        "{{\"k\":\"open\",\"path\":\"{}\",\"node\":\"{}\",\"open_us\":{}}}",
        path.display(),
        reader.node.display(),
        t0.elapsed().as_micros()
    );
    let mut pointer = Pointer::new(1280.0, 720.0, DEFAULT_SENSITIVITY);
    let (mut reports, mut clicks, mut wheel) = (0u64, 0u64, 0i64);
    let mut error = None;
    while t0.elapsed().as_secs() < seconds {
        let r = reader.wait_and_drain(100, &mut |r| {
            let now = t0.elapsed().as_nanos() as u64;
            let at = pointer.apply(&r, now);
            reports += 1;
            clicks += u64::from(r.left_pressed);
            wheel += i64::from(r.wheel);
            println!(
                "{{\"k\":\"report\",\"t_ms\":{},\"dx\":{},\"dy\":{},\"wheel\":{},\"left\":{},\"cursor\":{:?},\"click_at\":{:?}}}",
                now / 1_000_000, r.dx, r.dy, r.wheel, r.left_pressed, pointer.position(now), at
            );
        });
        if let Err(e) = r {
            error = Some(e.to_string());
            break;
        }
    }
    println!(
        "{{\"k\":\"summary\",\"ok\":{},\"seconds\":{seconds},\"events\":{},\"reports\":{reports},\"clicks\":{clicks},\"wheel\":{wheel},\"dropped_reports\":{},\"error\":{:?}}}",
        error.is_none(),
        reader.events,
        reader.dropped_reports(),
        error
    );
}

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("mouse_probe: Linux evdev only");
    std::process::exit(1);
}
