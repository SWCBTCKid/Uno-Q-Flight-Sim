//! Platform layer: monotonic clock, pacing sleep, CPU affinity, glibc mtrace
//! control (through dlsym, so the binary links without it), and the sysfs
//! clock/temperature sampler. Linux is the measured target; the other branch
//! exists only so the harness builds and runs on the Windows dev host.

#[cfg(target_os = "linux")]
mod imp {
    use std::ffi::CString;

    /// CLOCK_MONOTONIC in ns (vDSO, no allocation).
    #[inline]
    pub fn now_ns() -> u64 {
        let mut ts = libc::timespec {
            tv_sec: 0,
            tv_nsec: 0,
        };
        unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
        ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
    }

    /// Sleeps until an absolute CLOCK_MONOTONIC deadline (no-op if past).
    #[inline]
    pub fn sleep_until_ns(t: u64) {
        let ts = libc::timespec {
            tv_sec: (t / 1_000_000_000) as libc::time_t,
            tv_nsec: (t % 1_000_000_000) as libc::c_long,
        };
        loop {
            let rc = unsafe {
                libc::clock_nanosleep(
                    libc::CLOCK_MONOTONIC,
                    libc::TIMER_ABSTIME,
                    &ts,
                    std::ptr::null_mut(),
                )
            };
            if rc != libc::EINTR {
                break;
            }
        }
    }

    /// Pins the calling thread to one CPU.
    pub fn pin_to(core: usize) -> Result<(), String> {
        unsafe {
            let mut set: libc::cpu_set_t = std::mem::zeroed();
            libc::CPU_SET(core, &mut set);
            if libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &set) != 0 {
                return Err(format!(
                    "sched_setaffinity({core}) failed: {}",
                    std::io::Error::last_os_error()
                ));
            }
        }
        Ok(())
    }

    /// The calling thread's CPU set.
    pub fn affinity() -> Vec<usize> {
        let mut out = Vec::new();
        unsafe {
            let mut set: libc::cpu_set_t = std::mem::zeroed();
            if libc::sched_getaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &mut set) == 0 {
                for c in 0..libc::CPU_SETSIZE as usize {
                    if libc::CPU_ISSET(c, &set) {
                        out.push(c);
                    }
                }
            }
        }
        out
    }

    /// Resolves mtrace/muntrace inside the preloaded libc_malloc_debug.so.0.
    /// There they are versioned compat symbols (aarch64 base version
    /// GLIBC_2.17), so an unversioned dlsym binds libc.so.6's no-op stubs
    /// instead (seen on the board with LD_DEBUG=bindings).
    fn sym(name: &str) -> Option<unsafe extern "C" fn()> {
        let n = CString::new(name).ok()?;
        let lib = CString::new("libc_malloc_debug.so.0").ok()?;
        let ver = CString::new("GLIBC_2.17").ok()?;
        unsafe {
            let h = libc::dlopen(lib.as_ptr(), libc::RTLD_NOW | libc::RTLD_NOLOAD);
            if h.is_null() {
                return None;
            }
            let mut p = libc::dlvsym(h, n.as_ptr(), ver.as_ptr());
            if p.is_null() {
                p = libc::dlsym(h, n.as_ptr());
            }
            if p.is_null() {
                None
            } else {
                Some(std::mem::transmute::<
                    *mut libc::c_void,
                    unsafe extern "C" fn(),
                >(p))
            }
        }
    }

    /// glibc mtrace(): starts tracing to $MALLOC_TRACE when libc_malloc_debug.so
    /// is preloaded (glibc >= 2.34); otherwise a no-op stub.
    pub fn mtrace_on() -> bool {
        match sym("mtrace") {
            Some(f) => {
                unsafe { f() };
                true
            }
            None => false,
        }
    }

    /// Step marker in the mtrace log: one malloc/free pair of MARK_BYTES
    /// from the harness itself, so the analyzer can split the trace by step.
    #[inline]
    pub fn mtrace_marker() {
        unsafe {
            let p = libc::malloc(super::MARK_BYTES);
            std::hint::black_box(p);
            libc::free(p);
        }
    }

    pub fn mtrace_off() {
        if let Some(f) = sym("muntrace") {
            unsafe { f() };
        }
    }

    /// Peak and current RSS (KiB) from /proc/self/status.
    pub fn rss_kib() -> (u64, u64) {
        let s = std::fs::read_to_string("/proc/self/status").unwrap_or_default();
        let get = |k: &str| {
            s.lines()
                .find(|l| l.starts_with(k))
                .and_then(|l| l.split_whitespace().nth(1))
                .and_then(|v| v.parse().ok())
                .unwrap_or(0)
        };
        (get("VmHWM:"), get("VmRSS:"))
    }

    /// Allocation-free memory sample for in-loop use: (RSS KiB from
    /// /proc/self/statm, glibc heap bytes in use, glibc heap bytes obtained
    /// from the system) via mallinfo2().
    pub fn mem_sample() -> (u64, u64, u64) {
        let mut buf = [0u8; 128];
        let path = b"/proc/self/statm\0";
        let mut rss = 0u64;
        unsafe {
            let fd = libc::open(path.as_ptr() as *const libc::c_char, libc::O_RDONLY);
            if fd >= 0 {
                let n = libc::read(fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len());
                libc::close(fd);
                if n > 0 {
                    // "size resident shared ..." in pages
                    let mut field = 0;
                    let mut v = 0u64;
                    for &b in &buf[..n as usize] {
                        if b == b' ' {
                            if field == 1 {
                                break;
                            }
                            field += 1;
                            v = 0;
                        } else if b.is_ascii_digit() {
                            v = v * 10 + (b - b'0') as u64;
                        }
                    }
                    let page = libc::sysconf(libc::_SC_PAGESIZE) as u64;
                    rss = v * page / 1024;
                }
            }
            let mi = libc::mallinfo2();
            (rss, mi.uordblks as u64, (mi.arena + mi.hblkhd) as u64)
        }
    }
}

#[cfg(not(target_os = "linux"))]
mod imp {
    use std::sync::OnceLock;
    use std::time::Instant;
    static T0: OnceLock<Instant> = OnceLock::new();
    pub fn now_ns() -> u64 {
        T0.get_or_init(Instant::now).elapsed().as_nanos() as u64 + 1_000_000_000
    }
    pub fn sleep_until_ns(t: u64) {
        let n = now_ns();
        if t > n {
            std::thread::sleep(std::time::Duration::from_nanos(t - n));
        }
    }
    pub fn pin_to(_core: usize) -> Result<(), String> {
        Err("CPU pinning not implemented on this host (dev only)".into())
    }
    pub fn affinity() -> Vec<usize> {
        Vec::new()
    }
    pub fn mtrace_on() -> bool {
        false
    }
    pub fn mtrace_off() {}
    pub fn mtrace_marker() {}
    pub fn rss_kib() -> (u64, u64) {
        (0, 0)
    }
    pub fn mem_sample() -> (u64, u64, u64) {
        (0, 0, 0)
    }
}

pub use imp::*;

/// Size of the per-step marker allocation in the mtrace log (0xffb).
pub const MARK_BYTES: usize = 4091;

// ---------------------------------------------------------------- sampler ----

/// One sysfs sample: CPU kHz per cpufreq policy, GPU Hz, max thermal zone (C).
#[derive(Clone, Debug)]
pub struct Sample {
    pub t_ns: u64,
    pub cpu_khz: Vec<u64>,
    pub gpu_hz: Option<u64>,
    pub temp_max_c: Option<f64>,
}

pub struct Sampler {
    pub policies: Vec<(String, String)>, // (policy name, related cpus)
    pub period_ms: u64,
    stop: std::sync::Arc<std::sync::atomic::AtomicBool>,
    samples: std::sync::Arc<std::sync::Mutex<Vec<Sample>>>,
    handle: Option<std::thread::JoinHandle<()>>,
}

const GPU_DEVFREQ: &str = "/sys/class/devfreq/5900000.gpu/cur_freq";

fn read_u64(p: &str) -> Option<u64> {
    std::fs::read_to_string(p).ok()?.trim().parse().ok()
}

impl Sampler {
    /// Starts a read-only sysfs sampler thread (pinned to `core` if given).
    pub fn start(period_ms: u64, core: Option<usize>) -> Sampler {
        let mut policies = Vec::new();
        if let Ok(rd) = std::fs::read_dir("/sys/devices/system/cpu/cpufreq") {
            let mut names: Vec<String> = rd
                .filter_map(|e| e.ok())
                .map(|e| e.file_name().to_string_lossy().into_owned())
                .filter(|n| n.starts_with("policy"))
                .collect();
            names.sort();
            for n in names {
                let cpus = std::fs::read_to_string(format!(
                    "/sys/devices/system/cpu/cpufreq/{n}/related_cpus"
                ))
                .map(|s| s.trim().to_string())
                .unwrap_or_default();
                policies.push((n, cpus));
            }
        }
        let mut zones = Vec::new();
        if let Ok(rd) = std::fs::read_dir("/sys/class/thermal") {
            for e in rd.filter_map(|e| e.ok()) {
                let n = e.file_name().to_string_lossy().into_owned();
                if n.starts_with("thermal_zone") {
                    zones.push(format!("/sys/class/thermal/{n}/temp"));
                }
            }
        }
        zones.sort();
        let stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let samples = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
        let (s2, st2, pol) = (samples.clone(), stop.clone(), policies.clone());
        let handle = std::thread::spawn(move || {
            if let Some(c) = core {
                let _ = pin_to(c);
            }
            let mut next = now_ns();
            while !st2.load(std::sync::atomic::Ordering::Relaxed) {
                let cpu_khz = pol
                    .iter()
                    .map(|(n, _)| {
                        read_u64(&format!(
                            "/sys/devices/system/cpu/cpufreq/{n}/scaling_cur_freq"
                        ))
                        .unwrap_or(0)
                    })
                    .collect();
                let gpu_hz = read_u64(GPU_DEVFREQ);
                let temp_max_c = zones
                    .iter()
                    .filter_map(|z| read_u64(z))
                    .map(|m| m as f64 / 1000.0)
                    .fold(None, |a: Option<f64>, t| Some(a.map_or(t, |a| a.max(t))));
                s2.lock().unwrap().push(Sample {
                    t_ns: now_ns(),
                    cpu_khz,
                    gpu_hz,
                    temp_max_c,
                });
                next += period_ms * 1_000_000;
                sleep_until_ns(next);
            }
        });
        Sampler {
            policies,
            period_ms,
            stop,
            samples,
            handle: Some(handle),
        }
    }

    /// Samples taken within [t0, t1] (ns).
    pub fn window(&self, t0: u64, t1: u64) -> Vec<Sample> {
        self.samples
            .lock()
            .unwrap()
            .iter()
            .filter(|s| s.t_ns >= t0 && s.t_ns <= t1)
            .cloned()
            .collect()
    }

    pub fn stop(&mut self) {
        self.stop.store(true, std::sync::atomic::Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

impl Drop for Sampler {
    fn drop(&mut self) {
        self.stop();
    }
}

/// Writes one element per 4 KiB page (volatile, value unchanged) so a
/// calloc'd buffer is resident before a timed loop.
pub fn prefault<T: Copy>(v: &mut [T]) {
    let per_page = (4096 / std::mem::size_of::<T>().max(1)).max(1);
    let mut i = 0;
    while i < v.len() {
        unsafe {
            let p = v.as_mut_ptr().add(i);
            std::ptr::write_volatile(p, std::ptr::read_volatile(p));
        }
        i += per_page;
    }
}
