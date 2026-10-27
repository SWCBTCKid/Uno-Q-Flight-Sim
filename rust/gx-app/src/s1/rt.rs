//! Monotonic clock, absolute-deadline sleep and the unprivileged scheduling
//! knobs used by the S1 board binary.
//!
//! On Linux every thread uses `CLOCK_MONOTONIC` directly: `clock_gettime`
//! for timestamps and `clock_nanosleep(TIMER_ABSTIME)` for deadlines, so
//! frame timestamps, tick deadlines and snapshot stamps share one epoch and
//! sleeping never accumulates drift. Elsewhere (host tests) a
//! `std::time::Instant` clock with relative sleeps stands in.

use std::io;

/// One clock shared by all S1 threads.
pub trait Clock: Clone + Send + Sync + 'static {
    fn now_ns(&self) -> u64;
    /// Sleep until the absolute monotonic time `deadline_ns` (returns at once
    /// if it has passed).
    fn sleep_until(&self, deadline_ns: u64);
}

#[derive(Clone, Copy, Debug, Default)]
pub struct MonotonicClock;

#[cfg(target_os = "linux")]
mod sys {
    use std::ffi::{c_int, c_long, c_uint, c_ulong};

    #[repr(C)]
    pub struct Timespec {
        pub tv_sec: c_long,
        pub tv_nsec: c_long,
    }

    #[repr(C)]
    pub struct SchedParam {
        pub sched_priority: c_int,
    }

    pub const CLOCK_MONOTONIC: c_int = 1;
    pub const TIMER_ABSTIME: c_int = 1;
    pub const EINTR: c_int = 4;
    pub const PR_SET_TIMERSLACK: c_int = 29;
    pub const SCHED_FIFO: c_int = 1;
    pub const PRIO_PROCESS: c_int = 0;

    extern "C" {
        pub fn clock_gettime(clock: c_int, ts: *mut Timespec) -> c_int;
        pub fn clock_nanosleep(
            clock: c_int,
            flags: c_int,
            request: *const Timespec,
            remain: *mut Timespec,
        ) -> c_int;
        pub fn prctl(option: c_int, ...) -> c_int;
        pub fn sched_setscheduler(pid: c_int, policy: c_int, param: *const SchedParam) -> c_int;
        pub fn sched_setaffinity(pid: c_int, cpusetsize: usize, mask: *const c_ulong) -> c_int;
        pub fn setpriority(which: c_int, who: c_uint, prio: c_int) -> c_int;
        pub fn gettid() -> c_int;
        pub fn syscall(number: c_long, ...) -> c_long;
    }

    /// `struct sched_attr`, `SCHED_ATTR_SIZE_VER0` layout.
    #[repr(C)]
    #[derive(Default)]
    pub struct SchedAttr {
        pub size: u32,
        pub sched_policy: u32,
        pub sched_flags: u64,
        pub sched_nice: i32,
        pub sched_priority: u32,
        pub sched_runtime: u64,
        pub sched_deadline: u64,
        pub sched_period: u64,
    }

    pub const SCHED_OTHER: u32 = 0;
    #[cfg(target_arch = "aarch64")]
    pub const SYS_SCHED_SETATTR: Option<(c_long, c_long)> = Some((274, 275));
    #[cfg(target_arch = "x86_64")]
    pub const SYS_SCHED_SETATTR: Option<(c_long, c_long)> = Some((314, 315));
    #[cfg(not(any(target_arch = "aarch64", target_arch = "x86_64")))]
    pub const SYS_SCHED_SETATTR: Option<(c_long, c_long)> = None;

    pub fn ulong(v: u64) -> c_ulong {
        v as c_ulong
    }
}

/// Pin the calling thread to one logical CPU. This is unprivileged for the
/// caller's own thread and avoids migration between the UNO Q's four CPUs.
pub fn set_thread_affinity(cpu: usize) -> io::Result<()> {
    #[cfg(target_os = "linux")]
    {
        const CPU_SET_BITS: usize = 1024;
        const WORD_BITS: usize = std::mem::size_of::<std::ffi::c_ulong>() * 8;
        if cpu >= CPU_SET_BITS {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "CPU index exceeds cpu_set_t",
            ));
        }
        let mut mask = [0 as std::ffi::c_ulong; CPU_SET_BITS / WORD_BITS];
        mask[cpu / WORD_BITS] |= (1 as std::ffi::c_ulong) << (cpu % WORD_BITS);
        // SAFETY: pid 0 means the calling thread and `mask` is a complete
        // glibc-compatible 1024-bit cpu_set_t for the duration of the call.
        let rc = unsafe { sys::sched_setaffinity(0, std::mem::size_of_val(&mask), mask.as_ptr()) };
        if rc != 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = cpu;
        Err(io::ErrorKind::Unsupported.into())
    }
}

#[cfg(target_os = "linux")]
impl Clock for MonotonicClock {
    fn now_ns(&self) -> u64 {
        let mut ts = sys::Timespec { tv_sec: 0, tv_nsec: 0 };
        // SAFETY: `ts` is a valid, writable timespec for the call.
        let rc = unsafe { sys::clock_gettime(sys::CLOCK_MONOTONIC, &mut ts) };
        assert_eq!(rc, 0, "clock_gettime(CLOCK_MONOTONIC) failed");
        ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
    }

    fn sleep_until(&self, deadline_ns: u64) {
        let ts = sys::Timespec {
            tv_sec: (deadline_ns / 1_000_000_000) as _,
            tv_nsec: (deadline_ns % 1_000_000_000) as _,
        };
        loop {
            // SAFETY: `ts` is valid for the call; `remain` is unused with
            // TIMER_ABSTIME and may be null.
            let rc = unsafe {
                sys::clock_nanosleep(
                    sys::CLOCK_MONOTONIC,
                    sys::TIMER_ABSTIME,
                    &ts,
                    std::ptr::null_mut(),
                )
            };
            if rc != sys::EINTR {
                return;
            }
        }
    }
}

#[cfg(not(target_os = "linux"))]
impl Clock for MonotonicClock {
    fn now_ns(&self) -> u64 {
        use std::sync::OnceLock;
        use std::time::Instant;
        static START: OnceLock<Instant> = OnceLock::new();
        // Offset so that 0 is never a real timestamp.
        START.get_or_init(Instant::now).elapsed().as_nanos() as u64 + 1_000_000_000
    }

    fn sleep_until(&self, deadline_ns: u64) {
        let now = self.now_ns();
        if deadline_ns > now {
            std::thread::sleep(std::time::Duration::from_nanos(deadline_ns - now));
        }
    }
}

/// Set this thread's timer slack (default 50 µs) so absolute-deadline
/// wakeups are not coalesced late. Unprivileged.
pub fn set_timer_slack_ns(ns: u64) -> io::Result<()> {
    #[cfg(target_os = "linux")]
    {
        // SAFETY: PR_SET_TIMERSLACK takes one unsigned long; the rest are
        // ignored.
        let rc = unsafe {
            sys::prctl(sys::PR_SET_TIMERSLACK, sys::ulong(ns), sys::ulong(0), sys::ulong(0), sys::ulong(0))
        };
        if rc != 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = ns;
        Err(io::ErrorKind::Unsupported.into())
    }
}

/// `SCHED_FIFO` for the calling thread. Needs CAP_SYS_NICE or a non-zero
/// RLIMIT_RTPRIO; expected to fail with EPERM for the `arduino` user.
pub fn try_sched_fifo(priority: i32) -> io::Result<()> {
    #[cfg(target_os = "linux")]
    {
        let param = sys::SchedParam { sched_priority: priority };
        // SAFETY: pid 0 is the calling thread; `param` is valid.
        let rc = unsafe { sys::sched_setscheduler(0, sys::SCHED_FIFO, &param) };
        if rc != 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = priority;
        Err(io::ErrorKind::Unsupported.into())
    }
}

/// Set the calling thread's nice value. Raising it (lower priority) is always
/// allowed; lowering it needs CAP_SYS_NICE or RLIMIT_NICE.
pub fn set_thread_nice(nice: i32) -> io::Result<()> {
    #[cfg(target_os = "linux")]
    {
        // SAFETY: plain syscalls on the calling thread's tid.
        let rc = unsafe {
            let tid = sys::gettid();
            sys::setpriority(sys::PRIO_PROCESS, tid as _, nice)
        };
        if rc != 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = nice;
        Err(io::ErrorKind::Unsupported.into())
    }
}

/// EEVDF request size ("slice") for the calling `SCHED_OTHER` thread, via
/// `sched_setattr(sched_runtime)`, keeping `nice`. Linux 6.12+ accepts this
/// unprivileged (range 0.1-100 ms). A slice shorter than the default lets the
/// thread's wakeup preempt a default-slice task already running on its CPU
/// (EEVDF `PREEMPT_SHORT`) instead of waiting out that task's slice. Returns
/// the slice the kernel reports back, so the evidence shows what took.
pub fn set_fair_slice_ns(slice_ns: u64, nice: i32) -> io::Result<u64> {
    #[cfg(target_os = "linux")]
    {
        let Some((set, get)) = sys::SYS_SCHED_SETATTR else {
            return Err(io::ErrorKind::Unsupported.into());
        };
        let size = std::mem::size_of::<sys::SchedAttr>() as u32;
        let attr = sys::SchedAttr {
            size,
            sched_policy: sys::SCHED_OTHER,
            sched_nice: nice,
            sched_runtime: slice_ns,
            ..Default::default()
        };
        // SAFETY: pid 0 is the calling thread; `attr` is a valid VER0
        // sched_attr whose `size` field matches its layout; flags 0.
        let rc = unsafe { sys::syscall(set, 0 as std::ffi::c_long, &attr as *const sys::SchedAttr, 0 as std::ffi::c_long) };
        if rc != 0 {
            return Err(io::Error::last_os_error());
        }
        let mut back = sys::SchedAttr::default();
        // SAFETY: as above; the kernel writes at most `size` bytes.
        let rc = unsafe {
            sys::syscall(get, 0 as std::ffi::c_long, &mut back as *mut sys::SchedAttr, size as std::ffi::c_long, 0 as std::ffi::c_long)
        };
        if rc != 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(back.sched_runtime)
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = (slice_ns, nice);
        Err(io::ErrorKind::Unsupported.into())
    }
}

/// EEVDF slice for the sim thread: above its worst observed tick work
/// (0.47 ms on the board), below the default 2.25 ms on four CPUs.
pub const SIM_SLICE_NS: u64 = 1_000_000;
/// EEVDF slice for the I/O thread: its loop is ~0.1 ms.
pub const IO_SLICE_NS: u64 = 500_000;

fn slice_outcome(r: io::Result<u64>) -> String {
    match r {
        Ok(ns) => format!("{{\"ok\":{ns}}}"),
        Err(e) => format!("\"{}\"", outcome(Err(e))),
    }
}

fn outcome(r: io::Result<()>) -> String {
    match r {
        Ok(()) => "ok".into(),
        Err(e) => match e.raw_os_error() {
            Some(1) => "EPERM".into(),
            Some(13) => "EACCES".into(),
            _ => e.to_string(),
        },
    }
}

/// `SCHED_FIFO` priority of the simulation thread.
pub const SIM_FIFO_PRIORITY: i32 = 50;
/// `SCHED_FIFO` priority of the I/O thread: below the sim, so a heartbeat
/// never delays a tick, and above every `SCHED_OTHER` task on the board.
pub const IO_FIFO_PRIORITY: i32 = 40;

/// Best available realtime setup for a periodic thread: timer slack 1 ns,
/// affinity to `cpu`, then `SCHED_FIFO` `priority` if permitted (BOARD-001
/// grants rtprio 60 to `arduino`), else nice -10 if permitted, else
/// `SCHED_OTHER` nice 0; under `SCHED_OTHER` a short EEVDF slice. Returns a
/// JSON object describing what took.
///
/// A FIFO thread must block every iteration (RT throttling only stops a
/// spinner at 95 % of a CPU). Both callers sleep on an absolute deadline
/// that is strictly in the future when computed; see `io_loop` and
/// `Cadence::complete`.
fn setup_periodic_thread(cpu: usize, priority: i32, slice_ns: u64) -> String {
    let slack = outcome(set_timer_slack_ns(1));
    let affinity = outcome(set_thread_affinity(cpu));
    let fifo = outcome(try_sched_fifo(priority));
    let (policy, nice, slice) = if fifo == "ok" {
        (format!("fifo{priority}"), "not_tried".to_owned(), "\"not_tried\"".to_owned())
    } else {
        let nice = outcome(set_thread_nice(-10));
        let value = if nice == "ok" { -10 } else { 0 };
        let slice = slice_outcome(set_fair_slice_ns(slice_ns, value));
        (format!("other_nice{value}"), nice, slice)
    };
    format!(
        "{{\"policy\":\"{policy}\",\"cpu\":{cpu},\"affinity\":\"{affinity}\",\"timerslack_1ns\":\"{slack}\",\"fifo{priority}\":\"{fifo}\",\"nice-10\":\"{nice}\",\"slice_ns\":{slice}}}"
    )
}

/// Simulation thread: `SCHED_FIFO` 50, with the fallback chain above.
pub fn setup_sim_thread(cpu: usize) -> String {
    setup_periodic_thread(cpu, SIM_FIFO_PRIORITY, SIM_SLICE_NS)
}

/// I/O thread: `SCHED_FIFO` 40, with the same fallback chain (it must
/// heartbeat every <= 10 ms).
pub fn setup_io_thread(cpu: usize) -> String {
    setup_periodic_thread(cpu, IO_FIFO_PRIORITY, IO_SLICE_NS)
}

/// Reader ("render") thread: timer slack only, default priority and slice,
/// so it competes for CPU as the real render thread will.
pub fn setup_reader_thread(cpu: usize) -> String {
    format!(
        "{{\"policy\":\"other_nice0\",\"cpu\":{cpu},\"affinity\":\"{}\",\"timerslack_1ns\":\"{}\"}}",
        outcome(set_thread_affinity(cpu)),
        outcome(set_timer_slack_ns(1))
    )
}

/// `SCHED_OTHER` for the calling thread (always permitted).
pub fn set_sched_other() -> io::Result<()> {
    #[cfg(target_os = "linux")]
    {
        let param = sys::SchedParam { sched_priority: 0 };
        // SAFETY: pid 0 is the calling thread; `param` is valid.
        let rc = unsafe { sys::sched_setscheduler(0, 0, &param) };
        if rc != 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }
    #[cfg(not(target_os = "linux"))]
    {
        Err(io::ErrorKind::Unsupported.into())
    }
}

/// Log thread: explicitly `SCHED_OTHER` nice 0 and unpinned. Its file
/// writes may block, so it must never be realtime, whatever spawned it.
pub fn setup_log_thread() -> String {
    format!("{{\"policy\":\"other_nice0\",\"sched_other\":\"{}\"}}", outcome(set_sched_other()))
}

/// Prompter/stdin threads: lowest priority that matters, nice +10.
pub fn setup_background_thread() -> String {
    format!("{{\"policy\":\"other_nice10\",\"nice\":\"{}\"}}", outcome(set_thread_nice(10)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clock_is_monotonic_and_sleep_until_reaches_the_deadline() {
        let clock = MonotonicClock;
        let a = clock.now_ns();
        let deadline = a + 2_000_000;
        clock.sleep_until(deadline);
        let b = clock.now_ns();
        assert!(b >= deadline, "woke at {b} before {deadline}");
        // A deadline in the past returns immediately.
        clock.sleep_until(a);
        assert!(clock.now_ns() >= b);
    }
}
