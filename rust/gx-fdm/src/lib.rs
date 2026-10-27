//! Safe wrapper over the gx_fdm C shim (JSBSim behind `extern "C"`, ARCH-002).
//!
//! * [`Fdm`] owns one `gx_fdm*`; `Drop` calls `gx_fdm_destroy`.
//! * Errors carry the shim's return code and `gx_fdm_last_error()` text.
//! * [`Prop`] handles come from [`Fdm::bind`]; [`Fdm::get`] / [`Fdm::set`] are
//!   the allocation-free per-step path (no strings cross the boundary).
use gx_fdm_sys as sys;
use std::ffi::{CStr, CString};
use std::fmt;
use std::os::raw::c_char;
use std::path::Path;
use std::ptr::NonNull;

/// Error returned by a shim call: the C return code and the shim's message.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FdmError {
    pub code: i32,
    pub message: String,
}

impl fmt::Display for FdmError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "gx_fdm error {}: {}", self.code, self.message)
    }
}

impl std::error::Error for FdmError {}

pub type Result<T> = std::result::Result<T, FdmError>;

/// Result of one [`Fdm::step`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Step {
    /// The step ran; the simulation continues.
    Running,
    /// The script reached its end time or set `simulation/terminate`.
    Done,
}

/// A bound property handle (index into the shim's handle table).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Prop(pub i32);

/// One JSBSim instance behind the shim.
pub struct Fdm {
    raw: NonNull<sys::gx_fdm>,
}

// The shim has no thread affinity; one Fdm is used by one thread at a time,
// which `&mut self` on every mutating call and the absence of `Sync` enforce.
unsafe impl Send for Fdm {}

fn cstring(s: &str) -> Result<CString> {
    CString::new(s).map_err(|_| FdmError {
        code: -100,
        message: format!("interior NUL in {s:?}"),
    })
}

fn path_cstring(p: &Path) -> Result<CString> {
    cstring(&p.to_string_lossy())
}

impl Fdm {
    /// `gx_fdm_create`: `jsbsim_root` holds JSBSim's `aircraft/`, `engine/`, `systems/`.
    pub fn create(jsbsim_root: &Path) -> Result<Fdm> {
        let root = path_cstring(jsbsim_root)?;
        let mut err = [0 as c_char; 512];
        let p = unsafe { sys::gx_fdm_create(root.as_ptr(), err.as_mut_ptr(), err.len()) };
        match NonNull::new(p) {
            Some(raw) => Ok(Fdm { raw }),
            None => {
                let msg = unsafe { CStr::from_ptr(err.as_ptr()) }
                    .to_string_lossy()
                    .into_owned();
                Err(FdmError {
                    code: -1,
                    message: msg,
                })
            }
        }
    }

    fn check(&mut self, rc: i32) -> Result<i32> {
        if rc < 0 {
            Err(FdmError {
                code: rc,
                message: self.last_error(),
            })
        } else {
            Ok(rc)
        }
    }

    /// The shim's last error text ("" if none).
    pub fn last_error(&mut self) -> String {
        let p = unsafe { sys::gx_fdm_last_error(self.raw.as_ptr()) };
        if p.is_null() {
            return String::new();
        }
        unsafe { CStr::from_ptr(p) }.to_string_lossy().into_owned()
    }

    /// `gx_fdm_load_model`.
    pub fn load_model(&mut self, aircraft: &str, dt_s: f64) -> Result<()> {
        let a = cstring(aircraft)?;
        let rc = unsafe { sys::gx_fdm_load_model(self.raw.as_ptr(), a.as_ptr(), dt_s) };
        self.check(rc).map(|_| ())
    }

    /// `gx_fdm_load_script` (`initfile == None`) or `gx_fdm_load_script_ic`.
    pub fn load_script(&mut self, script: &Path, dt_s: f64, initfile: Option<&Path>) -> Result<()> {
        let s = path_cstring(script)?;
        let rc = match initfile {
            None => unsafe { sys::gx_fdm_load_script(self.raw.as_ptr(), s.as_ptr(), dt_s) },
            Some(ic) => {
                let i = path_cstring(ic)?;
                unsafe {
                    sys::gx_fdm_load_script_ic(self.raw.as_ptr(), s.as_ptr(), dt_s, i.as_ptr())
                }
            }
        };
        self.check(rc).map(|_| ())
    }

    /// `gx_fdm_run_ic`.
    pub fn run_ic(&mut self) -> Result<()> {
        let rc = unsafe { sys::gx_fdm_run_ic(self.raw.as_ptr()) };
        self.check(rc).map(|_| ())
    }

    /// `gx_fdm_step`: one dt.
    #[inline]
    pub fn step(&mut self) -> Result<Step> {
        let rc = unsafe { sys::gx_fdm_step(self.raw.as_ptr()) };
        match rc {
            0 => Ok(Step::Running),
            1 => Ok(Step::Done),
            _ => Err(FdmError {
                code: rc,
                message: self.last_error(),
            }),
        }
    }

    /// `gx_fdm_bind`: resolve an existing property once.
    pub fn bind(&mut self, property: &str) -> Result<Prop> {
        let p = cstring(property)?;
        let rc = unsafe { sys::gx_fdm_bind(self.raw.as_ptr(), p.as_ptr()) };
        self.check(rc).map(Prop)
    }

    /// `gx_fdm_get` (NaN for an invalid handle).
    #[inline]
    pub fn get(&self, prop: Prop) -> f64 {
        unsafe { sys::gx_fdm_get(self.raw.as_ptr(), prop.0) }
    }

    /// `gx_fdm_set`.
    #[inline]
    pub fn set(&mut self, prop: Prop, value: f64) {
        unsafe { sys::gx_fdm_set(self.raw.as_ptr(), prop.0, value) }
    }

    /// `gx_fdm_reset`: back to the initial conditions (restarts a loaded script).
    pub fn reset(&mut self) -> Result<()> {
        let rc = unsafe { sys::gx_fdm_reset(self.raw.as_ptr()) };
        self.check(rc).map(|_| ())
    }
}

impl Drop for Fdm {
    fn drop(&mut self) {
        unsafe { sys::gx_fdm_destroy(self.raw.as_ptr()) }
    }
}
