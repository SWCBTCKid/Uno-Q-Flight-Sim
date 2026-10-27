//! Raw declarations of the gx_fdm C shim (`src/fdm/shim/gx_fdm.h`), hand-written
//! (ARCH-002 allows hand-written or bindgen `-sys` declarations; the header is
//! eleven functions, so bindgen would add a libclang build dependency for no gain).
//! Keep in lockstep with gx_fdm.h.
#![allow(non_camel_case_types)]

use std::os::raw::{c_char, c_double, c_int};

#[repr(C)]
pub struct gx_fdm {
    _private: [u8; 0],
    _marker: core::marker::PhantomData<(*mut u8, core::marker::PhantomPinned)>,
}

extern "C" {
    pub fn gx_fdm_create(
        jsbsim_root: *const c_char,
        err: *mut c_char,
        err_len: usize,
    ) -> *mut gx_fdm;
    pub fn gx_fdm_load_model(fdm: *mut gx_fdm, aircraft: *const c_char, dt_s: c_double) -> c_int;
    pub fn gx_fdm_load_script(
        fdm: *mut gx_fdm,
        script_path: *const c_char,
        dt_s: c_double,
    ) -> c_int;
    pub fn gx_fdm_load_script_ic(
        fdm: *mut gx_fdm,
        script_path: *const c_char,
        dt_s: c_double,
        initfile: *const c_char,
    ) -> c_int;
    pub fn gx_fdm_run_ic(fdm: *mut gx_fdm) -> c_int;
    pub fn gx_fdm_step(fdm: *mut gx_fdm) -> c_int;
    pub fn gx_fdm_bind(fdm: *mut gx_fdm, property: *const c_char) -> c_int;
    pub fn gx_fdm_get(fdm: *mut gx_fdm, handle: c_int) -> c_double;
    pub fn gx_fdm_set(fdm: *mut gx_fdm, handle: c_int, value: c_double);
    pub fn gx_fdm_reset(fdm: *mut gx_fdm) -> c_int;
    pub fn gx_fdm_last_error(fdm: *mut gx_fdm) -> *const c_char;
    pub fn gx_fdm_destroy(fdm: *mut gx_fdm);
}
