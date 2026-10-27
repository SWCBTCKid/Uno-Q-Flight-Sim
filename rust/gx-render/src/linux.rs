//! Native S3 presentation on the UNO Q stack: DRM/KMS + GBM + EGL/GLES,
//! loaded with dlopen (ARCH-002; the cross sysroot has no graphics -dev
//! packages), owned and used by exactly one thread (the render thread).
//!
//! Two targets share one GLES path ([`Gl`]):
//! * [`KmsDisplay`]: `/dev/dri/card0`, DRM master, DP-1 at 1280x720@60, one
//!   atomic commit in flight, **per-commit flip tokens** (ISS-031): the
//!   commit's user data is its own never-reused token, never a pointer, and
//!   [`PresentTracker`] attributes each event by token. A single stale
//!   vblank stamp is presented-and-counted as in the SPIKE-001b fix; any
//!   other stamp anomaly on the pending token still retires that buffer
//!   (the flip happened) and is counted as an error; an event for any other
//!   token never retires anything.
//! * [`Offscreen`]: `/dev/dri/renderD128` (no master, no display), the same
//!   GBM/EGL/GLES frame for the non-display board smoke and the offline
//!   render test (pixels read back).
//!
//! GMEM rules (SPIKE-001c): one opaque pass per frame into the default
//! framebuffer: clear every attachment at frame start, invalidate
//! depth/stencil (if the config has them) before the swap, no render-target
//! switches. S3 issues one draw call. S4 (`*_world` constructors) adds a
//! 24-bit depth buffer and draws, in this one pass: the static ground
//! (depth test off, painter's order), the aircraft (depth tested), the light
//! points (depth tested, no depth writes) and the HUD (no depth): at most
//! four draws, one render target. GPU time: GL_EXT_disjoint_timer_query
//! `GL_TIME_ELAPSED_EXT` from before the clear to after `eglSwapBuffers`
//! (brackets the tile pass and its resolve on freedreno), read after a
//! fence, classified like the spike (ok / not_ready / disjoint /
//! unavailable).

#![allow(clippy::missing_safety_doc)]

use crate::native::*;
use crate::pacing::{IntervalTracker, VblankClock};
use crate::present::{EventStatus, PendingPresent, PresentTracker};
use crate::aircraft::Vertex3;
use crate::scene::{Vertex, HEIGHT, MAX_VERTICES, WIDTH};
pub use crate::world::{WorldBuffers, WorldDraw};
use crate::photo::{photo_fragment_shader, prepare_photo, sky_fragment_shader, PhotoBatch, PhotoDraw, PhotoMemory, TexturedVertex};
use crate::photo::{PHOTO_VERTEX_SHADER, SKY_VERTEX_SHADER, SUN_ENU};
use std::cell::RefCell;
use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_int, c_uint, c_void};
use std::ptr;

const W: u32 = WIDTH as u32;
const H: u32 = HEIGHT as u32;
/// Present target rate on the 60 Hz mode.
pub const TARGET_FPS: u32 = 30;
const FLIP_TIMEOUT_NS: u64 = 500_000_000;
/// The first commit is a full modeset of DP-1.
const MODESET_TIMEOUT_NS: u64 = 3_000_000_000;
const FENCE_TIMEOUT_NS: u64 = 100_000_000;

// ------------------------------------------------------------------ errors --

#[derive(Debug, Clone)]
pub struct Error {
    pub stage: &'static str,
    pub detail: String,
}

impl Error {
    fn new(stage: &'static str, detail: impl Into<String>) -> Self {
        Self { stage, detail: detail.into() }
    }
    fn errno(stage: &'static str) -> Self {
        Self::new(stage, std::io::Error::last_os_error().to_string())
    }
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}: {}", self.stage, self.detail)
    }
}

impl std::error::Error for Error {}

pub fn monotonic_ns() -> u64 {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    // SAFETY: valid out pointer; CLOCK_MONOTONIC always exists.
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
}

fn sleep_until_monotonic_ns(deadline_ns: u64) {
    let ts = libc::timespec {
        tv_sec: (deadline_ns / 1_000_000_000) as _,
        tv_nsec: (deadline_ns % 1_000_000_000) as _,
    };
    loop {
        // SAFETY: `ts` remains valid for the call; TIMER_ABSTIME does not use
        // a remaining-time output pointer.
        let rc = unsafe { libc::clock_nanosleep(libc::CLOCK_MONOTONIC, libc::TIMER_ABSTIME, &ts, ptr::null_mut()) };
        if rc != libc::EINTR {
            return;
        }
    }
}

// ------------------------------------------------------------------ loader --

struct Library(*mut c_void);

impl Library {
    fn open(name: &str) -> Result<Self, Error> {
        let n = CString::new(name).expect("library name");
        // SAFETY: plain dlopen of a system library.
        let p = unsafe { libc::dlopen(n.as_ptr(), libc::RTLD_NOW | libc::RTLD_LOCAL) };
        if p.is_null() {
            Err(Error::new("dlopen", format!("{name}: {}", dl_error())))
        } else {
            Ok(Self(p))
        }
    }

    fn raw(&self, name: &str) -> *mut c_void {
        let n = CString::new(name).expect("symbol name");
        // SAFETY: valid handle and C string.
        unsafe {
            libc::dlerror();
            libc::dlsym(self.0, n.as_ptr())
        }
    }

    /// SAFETY: `T` must be the exact `extern "C"` fn type of `name`.
    unsafe fn sym<T: Copy>(&self, name: &str) -> Result<T, Error> {
        let p = self.raw(name);
        if p.is_null() {
            return Err(Error::new("dlsym", format!("{name}: {}", dl_error())));
        }
        assert_eq!(std::mem::size_of::<T>(), std::mem::size_of::<*mut c_void>());
        // SAFETY: caller guarantees the type; sizes checked.
        Ok(unsafe { std::mem::transmute_copy(&p) })
    }
}

impl Drop for Library {
    fn drop(&mut self) {
        // Mesa/glvnd keep global state and threads; never unload them
        // mid-process (unloading the GL driver has crashed processes).
    }
}

fn dl_error() -> String {
    // SAFETY: dlerror returns a thread-local C string or null.
    unsafe {
        let p = libc::dlerror();
        if p.is_null() {
            "unknown loader error".into()
        } else {
            CStr::from_ptr(p).to_string_lossy().into_owned()
        }
    }
}

macro_rules! api {
    ($name:ident { $($field:ident : $ty:ty => $symbol:literal),+ $(,)? }) => {
        #[allow(dead_code)]
        struct $name { $($field: $ty),+ }
        impl $name {
            /// SAFETY: the declared types must match the C prototypes
            /// (native.rs, checked against the pinned headers).
            unsafe fn load(lib: &Library) -> Result<Self, Error> {
                Ok(Self { $($field: unsafe { lib.sym::<$ty>($symbol)? }),+ })
            }
        }
    };
}

type Fd = c_int;
api!(DrmApi {
    set_master: unsafe extern "C" fn(Fd) -> c_int => "drmSetMaster",
    drop_master: unsafe extern "C" fn(Fd) -> c_int => "drmDropMaster",
    set_client_cap: unsafe extern "C" fn(Fd, u64, u64) -> c_int => "drmSetClientCap",
    get_cap: unsafe extern "C" fn(Fd, u64, *mut u64) -> c_int => "drmGetCap",
    get_resources: unsafe extern "C" fn(Fd) -> *mut drmModeRes => "drmModeGetResources",
    free_resources: unsafe extern "C" fn(*mut drmModeRes) -> () => "drmModeFreeResources",
    get_connector: unsafe extern "C" fn(Fd, u32) -> *mut drmModeConnector => "drmModeGetConnector",
    free_connector: unsafe extern "C" fn(*mut drmModeConnector) -> () => "drmModeFreeConnector",
    get_encoder: unsafe extern "C" fn(Fd, u32) -> *mut drmModeEncoder => "drmModeGetEncoder",
    free_encoder: unsafe extern "C" fn(*mut drmModeEncoder) -> () => "drmModeFreeEncoder",
    get_crtc: unsafe extern "C" fn(Fd, u32) -> *mut drmModeCrtc => "drmModeGetCrtc",
    free_crtc: unsafe extern "C" fn(*mut drmModeCrtc) -> () => "drmModeFreeCrtc",
    set_crtc: unsafe extern "C" fn(Fd, u32, u32, u32, u32, *mut u32, c_int, *mut drmModeModeInfo) -> c_int => "drmModeSetCrtc",
    get_plane_resources: unsafe extern "C" fn(Fd) -> *mut drmModePlaneRes => "drmModeGetPlaneResources",
    free_plane_resources: unsafe extern "C" fn(*mut drmModePlaneRes) -> () => "drmModeFreePlaneResources",
    get_plane: unsafe extern "C" fn(Fd, u32) -> *mut drmModePlane => "drmModeGetPlane",
    free_plane: unsafe extern "C" fn(*mut drmModePlane) -> () => "drmModeFreePlane",
    object_get_properties: unsafe extern "C" fn(Fd, u32, u32) -> *mut drmModeObjectProperties => "drmModeObjectGetProperties",
    free_object_properties: unsafe extern "C" fn(*mut drmModeObjectProperties) -> () => "drmModeFreeObjectProperties",
    get_property: unsafe extern "C" fn(Fd, u32) -> *mut drmModePropertyRes => "drmModeGetProperty",
    free_property: unsafe extern "C" fn(*mut drmModePropertyRes) -> () => "drmModeFreeProperty",
    atomic_alloc: unsafe extern "C" fn() -> *mut drmModeAtomicReq => "drmModeAtomicAlloc",
    atomic_free: unsafe extern "C" fn(*mut drmModeAtomicReq) -> () => "drmModeAtomicFree",
    atomic_add: unsafe extern "C" fn(*mut drmModeAtomicReq, u32, u32, u64) -> c_int => "drmModeAtomicAddProperty",
    atomic_commit: unsafe extern "C" fn(Fd, *mut drmModeAtomicReq, u32, *mut c_void) -> c_int => "drmModeAtomicCommit",
    create_blob: unsafe extern "C" fn(Fd, *const c_void, usize, *mut u32) -> c_int => "drmModeCreatePropertyBlob",
    destroy_blob: unsafe extern "C" fn(Fd, u32) -> c_int => "drmModeDestroyPropertyBlob",
    add_fb2: unsafe extern "C" fn(Fd, u32, u32, u32, *const u32, *const u32, *const u32, *mut u32, u32) -> c_int => "drmModeAddFB2",
    rm_fb: unsafe extern "C" fn(Fd, u32) -> c_int => "drmModeRmFB",
    handle_event: unsafe extern "C" fn(Fd, *mut drmEventContext) -> c_int => "drmHandleEvent",
});

api!(GbmApi {
    create_device: unsafe extern "C" fn(Fd) -> *mut gbm_device => "gbm_create_device",
    destroy_device: unsafe extern "C" fn(*mut gbm_device) -> () => "gbm_device_destroy",
    create_surface: unsafe extern "C" fn(*mut gbm_device, u32, u32, u32, u32) -> *mut gbm_surface => "gbm_surface_create",
    destroy_surface: unsafe extern "C" fn(*mut gbm_surface) -> () => "gbm_surface_destroy",
    lock_front: unsafe extern "C" fn(*mut gbm_surface) -> *mut gbm_bo => "gbm_surface_lock_front_buffer",
    release: unsafe extern "C" fn(*mut gbm_surface, *mut gbm_bo) -> () => "gbm_surface_release_buffer",
    bo_width: unsafe extern "C" fn(*mut gbm_bo) -> u32 => "gbm_bo_get_width",
    bo_height: unsafe extern "C" fn(*mut gbm_bo) -> u32 => "gbm_bo_get_height",
    bo_format: unsafe extern "C" fn(*mut gbm_bo) -> u32 => "gbm_bo_get_format",
    bo_handle: unsafe extern "C" fn(*mut gbm_bo) -> gbm_bo_handle => "gbm_bo_get_handle",
    bo_stride: unsafe extern "C" fn(*mut gbm_bo) -> u32 => "gbm_bo_get_stride",
});

api!(EglApi {
    get_error: unsafe extern "C" fn() -> EGLint => "eglGetError",
    get_platform_display: unsafe extern "C" fn(EGLenum, *mut c_void, *const EGLAttrib) -> EGLDisplay => "eglGetPlatformDisplay",
    initialize: unsafe extern "C" fn(EGLDisplay, *mut EGLint, *mut EGLint) -> EGLBoolean => "eglInitialize",
    terminate: unsafe extern "C" fn(EGLDisplay) -> EGLBoolean => "eglTerminate",
    bind_api: unsafe extern "C" fn(EGLenum) -> EGLBoolean => "eglBindAPI",
    choose_config: unsafe extern "C" fn(EGLDisplay, *const EGLint, *mut EGLConfig, EGLint, *mut EGLint) -> EGLBoolean => "eglChooseConfig",
    get_config_attrib: unsafe extern "C" fn(EGLDisplay, EGLConfig, EGLint, *mut EGLint) -> EGLBoolean => "eglGetConfigAttrib",
    create_context: unsafe extern "C" fn(EGLDisplay, EGLConfig, EGLContext, *const EGLint) -> EGLContext => "eglCreateContext",
    destroy_context: unsafe extern "C" fn(EGLDisplay, EGLContext) -> EGLBoolean => "eglDestroyContext",
    create_window_surface: unsafe extern "C" fn(EGLDisplay, EGLConfig, *mut c_void, *const EGLAttrib) -> EGLSurface => "eglCreatePlatformWindowSurface",
    destroy_surface: unsafe extern "C" fn(EGLDisplay, EGLSurface) -> EGLBoolean => "eglDestroySurface",
    make_current: unsafe extern "C" fn(EGLDisplay, EGLSurface, EGLSurface, EGLContext) -> EGLBoolean => "eglMakeCurrent",
    swap_buffers: unsafe extern "C" fn(EGLDisplay, EGLSurface) -> EGLBoolean => "eglSwapBuffers",
    release_thread: unsafe extern "C" fn() -> EGLBoolean => "eglReleaseThread",
    get_proc_address: unsafe extern "C" fn(*const c_char) -> *mut c_void => "eglGetProcAddress",
});

api!(GlApi {
    get_string: unsafe extern "C" fn(GLenum) -> *const u8 => "glGetString",
    get_error: unsafe extern "C" fn() -> GLenum => "glGetError",
    get_integerv: unsafe extern "C" fn(GLenum, *mut GLint) -> () => "glGetIntegerv",
    viewport: unsafe extern "C" fn(GLint, GLint, GLsizei, GLsizei) -> () => "glViewport",
    disable: unsafe extern "C" fn(GLenum) -> () => "glDisable",
    clear_color: unsafe extern "C" fn(f32, f32, f32, f32) -> () => "glClearColor",
    clear: unsafe extern "C" fn(GLbitfield) -> () => "glClear",
    create_shader: unsafe extern "C" fn(GLenum) -> GLuint => "glCreateShader",
    shader_source: unsafe extern "C" fn(GLuint, GLsizei, *const *const c_char, *const GLint) -> () => "glShaderSource",
    compile_shader: unsafe extern "C" fn(GLuint) -> () => "glCompileShader",
    get_shader_iv: unsafe extern "C" fn(GLuint, GLenum, *mut GLint) -> () => "glGetShaderiv",
    get_shader_info_log: unsafe extern "C" fn(GLuint, GLsizei, *mut GLsizei, *mut c_char) -> () => "glGetShaderInfoLog",
    delete_shader: unsafe extern "C" fn(GLuint) -> () => "glDeleteShader",
    create_program: unsafe extern "C" fn() -> GLuint => "glCreateProgram",
    attach_shader: unsafe extern "C" fn(GLuint, GLuint) -> () => "glAttachShader",
    link_program: unsafe extern "C" fn(GLuint) -> () => "glLinkProgram",
    get_program_iv: unsafe extern "C" fn(GLuint, GLenum, *mut GLint) -> () => "glGetProgramiv",
    delete_program: unsafe extern "C" fn(GLuint) -> () => "glDeleteProgram",
    use_program: unsafe extern "C" fn(GLuint) -> () => "glUseProgram",
    gen_vertex_arrays: unsafe extern "C" fn(GLsizei, *mut GLuint) -> () => "glGenVertexArrays",
    bind_vertex_array: unsafe extern "C" fn(GLuint) -> () => "glBindVertexArray",
    delete_vertex_arrays: unsafe extern "C" fn(GLsizei, *const GLuint) -> () => "glDeleteVertexArrays",
    gen_buffers: unsafe extern "C" fn(GLsizei, *mut GLuint) -> () => "glGenBuffers",
    bind_buffer: unsafe extern "C" fn(GLenum, GLuint) -> () => "glBindBuffer",
    buffer_data: unsafe extern "C" fn(GLenum, GLsizeiptr, *const c_void, GLenum) -> () => "glBufferData",
    delete_buffers: unsafe extern "C" fn(GLsizei, *const GLuint) -> () => "glDeleteBuffers",
    vertex_attrib_pointer: unsafe extern "C" fn(GLuint, GLint, GLenum, u8, GLsizei, *const c_void) -> () => "glVertexAttribPointer",
    enable_vertex_attrib_array: unsafe extern "C" fn(GLuint) -> () => "glEnableVertexAttribArray",
    draw_arrays: unsafe extern "C" fn(GLenum, GLint, GLsizei) -> () => "glDrawArrays",
    invalidate_framebuffer: unsafe extern "C" fn(GLenum, GLsizei, *const GLenum) -> () => "glInvalidateFramebuffer",
    fence_sync: unsafe extern "C" fn(GLenum, GLbitfield) -> GLsync => "glFenceSync",
    client_wait_sync: unsafe extern "C" fn(GLsync, GLbitfield, GLuint64) -> GLenum => "glClientWaitSync",
    delete_sync: unsafe extern "C" fn(GLsync) -> () => "glDeleteSync",
    read_pixels: unsafe extern "C" fn(GLint, GLint, GLsizei, GLsizei, GLenum, GLenum, *mut c_void) -> () => "glReadPixels",
    gen_queries: unsafe extern "C" fn(GLsizei, *mut GLuint) -> () => "glGenQueries",
    delete_queries: unsafe extern "C" fn(GLsizei, *const GLuint) -> () => "glDeleteQueries",
    begin_query: unsafe extern "C" fn(GLenum, GLuint) -> () => "glBeginQuery",
    end_query: unsafe extern "C" fn(GLenum) -> () => "glEndQuery",
    get_query_objectuiv: unsafe extern "C" fn(GLuint, GLenum, *mut GLuint) -> () => "glGetQueryObjectuiv",
    enable: unsafe extern "C" fn(GLenum) -> () => "glEnable",
    depth_mask: unsafe extern "C" fn(u8) -> () => "glDepthMask",
    depth_func: unsafe extern "C" fn(GLenum) -> () => "glDepthFunc",
    clear_depthf: unsafe extern "C" fn(f32) -> () => "glClearDepthf",
    get_uniform_location: unsafe extern "C" fn(GLuint, *const c_char) -> GLint => "glGetUniformLocation",
    uniform_matrix4fv: unsafe extern "C" fn(GLint, GLsizei, u8, *const f32) -> () => "glUniformMatrix4fv",
    uniform1f: unsafe extern "C" fn(GLint, f32) -> () => "glUniform1f",
    // SPIKE-002 photo pass.
    uniform1i: unsafe extern "C" fn(GLint, GLint) -> () => "glUniform1i",
    uniform2fv: unsafe extern "C" fn(GLint, GLsizei, *const f32) -> () => "glUniform2fv",
    uniform3fv: unsafe extern "C" fn(GLint, GLsizei, *const f32) -> () => "glUniform3fv",
    uniform4fv: unsafe extern "C" fn(GLint, GLsizei, *const f32) -> () => "glUniform4fv",
    uniform_matrix3fv: unsafe extern "C" fn(GLint, GLsizei, u8, *const f32) -> () => "glUniformMatrix3fv",
    gen_textures: unsafe extern "C" fn(GLsizei, *mut GLuint) -> () => "glGenTextures",
    delete_textures: unsafe extern "C" fn(GLsizei, *const GLuint) -> () => "glDeleteTextures",
    active_texture: unsafe extern "C" fn(GLenum) -> () => "glActiveTexture",
    bind_texture: unsafe extern "C" fn(GLenum, GLuint) -> () => "glBindTexture",
    tex_parameteri: unsafe extern "C" fn(GLenum, GLenum, GLint) -> () => "glTexParameteri",
    tex_parameterf: unsafe extern "C" fn(GLenum, GLenum, f32) -> () => "glTexParameterf",
    tex_image_2d: unsafe extern "C" fn(GLenum, GLint, GLint, GLsizei, GLsizei, GLint, GLenum, GLenum, *const c_void) -> () => "glTexImage2D",
    compressed_tex_image_2d: unsafe extern "C" fn(GLenum, GLint, GLenum, GLsizei, GLsizei, GLint, GLsizei, *const c_void) -> () => "glCompressedTexImage2D",
    draw_elements: unsafe extern "C" fn(GLenum, GLsizei, GLenum, *const c_void) -> () => "glDrawElements",
});

type QueryU64 = unsafe extern "C" fn(GLuint, GLenum, *mut GLuint64);

struct Libs {
    _drm: Option<Library>,
    _gbm: Library,
    _egl: Library,
    _gl: Library,
    drm: Option<DrmApi>,
    gbm: GbmApi,
    egl: EglApi,
    gl: GlApi,
}

impl Libs {
    fn load(with_drm: bool) -> Result<Self, Error> {
        let drm_lib = if with_drm { Some(Library::open("libdrm.so.2")?) } else { None };
        let gbm_lib = Library::open("libgbm.so.1")?;
        let egl_lib = Library::open("libEGL.so.1")?;
        let gl_lib = Library::open("libGLESv2.so.2")?;
        // SAFETY: the api! declarations match the C prototypes.
        unsafe {
            Ok(Self {
                drm: match &drm_lib {
                    Some(l) => Some(DrmApi::load(l)?),
                    None => None,
                },
                gbm: GbmApi::load(&gbm_lib)?,
                egl: EglApi::load(&egl_lib)?,
                gl: GlApi::load(&gl_lib)?,
                _drm: drm_lib,
                _gbm: gbm_lib,
                _egl: egl_lib,
                _gl: gl_lib,
            })
        }
    }
}

// ----------------------------------------------------------------- GBM/EGL --

/// What the GL side reports once, at start.
#[derive(Clone, Debug, Default)]
pub struct GlInfo {
    pub renderer: String,
    pub version: String,
    pub depth_bits: i32,
    pub stencil_bits: i32,
    pub clear_mask: &'static str,
    pub invalidate: &'static str,
    pub timer: bool,
    pub timer_reason: String,
    /// S4: bytes of static vertex data uploaded (ground, lights, aircraft).
    pub static_vertex_bytes: u64,
}

struct EglTarget {
    device: *mut gbm_device,
    surface: *mut gbm_surface,
    display: EGLDisplay,
    context: EGLContext,
    egl_surface: EGLSurface,
    current: bool,
    initialized: bool,
}

impl EglTarget {
    fn empty() -> Self {
        Self {
            device: ptr::null_mut(),
            surface: ptr::null_mut(),
            display: ptr::null_mut(),
            context: ptr::null_mut(),
            egl_surface: ptr::null_mut(),
            current: false,
            initialized: false,
        }
    }

    /// GBM device + surface on `fd`, EGL GLES3 context on an XRGB8888 config.
    /// Returns the config's depth and stencil sizes. Partial state is kept in
    /// `self` so [`Self::destroy`] cleans up on any failure.
    unsafe fn create(&mut self, l: &Libs, fd: Fd, gbm_flags: u32, want_depth: bool) -> Result<(i32, i32), Error> {
        let (gbm, egl) = (&l.gbm, &l.egl);
        let egl_err = || format!("0x{:x}", unsafe { (egl.get_error)() });
        self.device = unsafe { (gbm.create_device)(fd) };
        if self.device.is_null() {
            return Err(Error::errno("gbm_device"));
        }
        self.surface = unsafe { (gbm.create_surface)(self.device, W, H, DRM_FORMAT_XRGB8888, gbm_flags) };
        if self.surface.is_null() {
            return Err(Error::errno("gbm_surface"));
        }
        self.display = unsafe { (egl.get_platform_display)(EGL_PLATFORM_GBM_KHR, self.device.cast(), ptr::null()) };
        if self.display.is_null() {
            return Err(Error::new("egl_display", egl_err()));
        }
        let (mut major, mut minor) = (0, 0);
        if unsafe { (egl.initialize)(self.display, &mut major, &mut minor) } == 0 {
            return Err(Error::new("egl_initialize", egl_err()));
        }
        self.initialized = true;
        if unsafe { (egl.bind_api)(EGL_OPENGL_ES_API) } == 0 {
            return Err(Error::new("egl_bind_api", egl_err()));
        }
        let attrs = [
            EGL_SURFACE_TYPE, EGL_WINDOW_BIT, EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT_KHR,
            EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 0,
            EGL_DEPTH_SIZE, if want_depth { 24 } else { 0 }, EGL_NONE,
        ];
        let mut configs = [ptr::null_mut(); 64];
        let mut count = 0;
        if unsafe { (egl.choose_config)(self.display, attrs.as_ptr(), configs.as_mut_ptr(), 64, &mut count) } == 0
            || count <= 0
        {
            return Err(Error::new("egl_config", format!("eglChooseConfig found none ({})", egl_err())));
        }
        let attrib = |c: EGLConfig, a: EGLint| {
            let mut v = 0;
            (unsafe { (egl.get_config_attrib)(self.display, c, a, &mut v) } != 0).then_some(v)
        };
        let config = configs
            .into_iter()
            .take(count as usize)
            .find(|&c| attrib(c, EGL_NATIVE_VISUAL_ID).is_some_and(|v| v as u32 == DRM_FORMAT_XRGB8888))
            .ok_or_else(|| Error::new("egl_config", "no config with an XRGB8888 visual"))?;
        let depth = attrib(config, EGL_DEPTH_SIZE).unwrap_or(0);
        let stencil = attrib(config, EGL_STENCIL_SIZE).unwrap_or(0);
        if want_depth && depth < 24 {
            return Err(Error::new("egl_config", format!("S4 needs a 24-bit depth buffer, config has {depth}")));
        }
        let ctx_attrs = [EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE];
        self.context = unsafe { (egl.create_context)(self.display, config, ptr::null_mut(), ctx_attrs.as_ptr()) };
        if self.context.is_null() {
            return Err(Error::new("egl_context", egl_err()));
        }
        self.egl_surface = unsafe { (egl.create_window_surface)(self.display, config, self.surface.cast(), ptr::null()) };
        if self.egl_surface.is_null() {
            return Err(Error::new("egl_surface", egl_err()));
        }
        if unsafe { (egl.make_current)(self.display, self.egl_surface, self.egl_surface, self.context) } == 0 {
            return Err(Error::new("egl_make_current", egl_err()));
        }
        self.current = true;
        Ok((depth, stencil))
    }

    /// Releases every EGL/GBM object (GL objects must be deleted first).
    unsafe fn destroy(&mut self, l: &Libs) {
        let egl = &l.egl;
        unsafe {
            if self.current {
                (egl.make_current)(self.display, ptr::null_mut(), ptr::null_mut(), ptr::null_mut());
                self.current = false;
            }
            if !self.egl_surface.is_null() {
                (egl.destroy_surface)(self.display, self.egl_surface);
                self.egl_surface = ptr::null_mut();
            }
            if !self.context.is_null() {
                (egl.destroy_context)(self.display, self.context);
                self.context = ptr::null_mut();
            }
            if self.initialized {
                (egl.terminate)(self.display);
                self.initialized = false;
            }
            (egl.release_thread)();
            if !self.surface.is_null() {
                (l.gbm.destroy_surface)(self.surface);
                self.surface = ptr::null_mut();
            }
            if !self.device.is_null() {
                (l.gbm.destroy_device)(self.device);
                self.device = ptr::null_mut();
            }
        }
    }
}

// --------------------------------------------------------------------- GL --

const VERTEX_SHADER: &str = "#version 300 es
layout(location = 0) in vec2 a_pos;
layout(location = 1) in vec3 a_col;
out vec3 v_col;
void main() { v_col = a_col; gl_Position = vec4(a_pos, 0.0, 1.0); }
";

const FRAGMENT_SHADER: &str = "#version 300 es
precision mediump float;
in vec3 v_col;
out vec4 o_col;
void main() { o_col = vec4(v_col, 1.0); }
";

/// S4 world program: absolute or model coordinates times `u_mvp`; light
/// points sized by distance and drawn round.
const WORLD_VERTEX_SHADER: &str = "#version 300 es
uniform mat4 u_mvp;
uniform float u_point_k;
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_col;
out vec3 v_col;
void main() {
    v_col = a_col;
    gl_Position = u_mvp * vec4(a_pos, 1.0);
    gl_PointSize = clamp(u_point_k / max(gl_Position.w, 1.0), 2.0, 7.0);
}
";

const WORLD_FRAGMENT_SHADER: &str = "#version 300 es
precision mediump float;
uniform float u_round;
in vec3 v_col;
out vec4 o_col;
void main() {
    if (u_round > 0.5 && length(gl_PointCoord - vec2(0.5)) > 0.5) { discard; }
    o_col = vec4(v_col, 1.0);
}
";

struct StaticMesh {
    vao: GLuint,
    vbo: GLuint,
    count: GLsizei,
}

struct WorldGl {
    program: GLuint,
    u_mvp: GLint,
    u_point_k: GLint,
    u_round: GLint,
    ground: StaticMesh,
    lights: StaticMesh,
    aircraft: StaticMesh,
}

/// SPIKE-002 photo pass: one textured program, one sky program, one static
/// vertex + index buffer, the asset's textures.
struct PhotoGl {
    program: GLuint,
    u: PhotoUniforms,
    sky: GLuint,
    sky_vao: GLuint,
    sky_u: [GLint; 5],
    vao: GLuint,
    vbo: GLuint,
    ibo: GLuint,
    textures: Vec<(u32, GLuint)>,
    batches: Vec<PhotoBatch>,
    /// Aircraft joints (GXVISUAL 1.2 or the 1.1 hinges), composed per frame.
    joints: Vec<gx_world::Joint>,
    /// SPIKE-002 cost isolation (env GX_S002_SKIP, comma list of terrain,
    /// decals, aircraft, sky, lights); empty in every recorded run.
    skip: String,
}

struct PhotoUniforms {
    mvp: GLint,
    rel: GLint,
    normal: GLint,
    base_color: GLint,
    factor: GLint,
    roughness: GLint,
    gloss: GLint,
    eye: GLint,
    time: GLint,
    sun: GLint,
    joints: GLint,
}

/// GPU timer sample, classified as in SPIKE-001b.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GpuSample {
    pub ns: Option<u64>,
    pub status: &'static str,
}

/// CPU-side timing of one frame (monotonic ns).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DrawTiming {
    pub draw_start_ns: u64,
    pub swap_end_ns: u64,
    pub fence_ns: u64,
}

struct Gl {
    program: GLuint,
    vao: GLuint,
    vbo: GLuint,
    query: GLuint,
    query_u64: Option<QueryU64>,
    clear_mask: GLbitfield,
    invalidate: Vec<GLenum>,
    info: GlInfo,
    world: Option<WorldGl>,
    photo: Option<PhotoGl>,
    extensions: String,
}

impl Gl {
    unsafe fn new(l: &Libs, depth: i32, stencil: i32) -> Result<Self, Error> {
        let gl = &l.gl;
        let renderer = gl_string(gl, GL_RENDERER);
        if classify_renderer(&renderer) != RendererClass::Adreno {
            return Err(Error::new("renderer", format!("'{renderer}' is not the Adreno/freedreno GPU")));
        }
        let extensions = gl_string(gl, GL_EXTENSIONS);
        let mut s = Self {
            program: 0,
            vao: 0,
            vbo: 0,
            query: 0,
            query_u64: None,
            clear_mask: GL_COLOR_BUFFER_BIT
                | if depth > 0 { GL_DEPTH_BUFFER_BIT } else { 0 }
                | if stencil > 0 { GL_STENCIL_BUFFER_BIT } else { 0 },
            invalidate: [(depth > 0, GL_DEPTH), (stencil > 0, GL_STENCIL)]
                .into_iter()
                .filter_map(|(on, a)| on.then_some(a))
                .collect(),
            world: None,
            photo: None,
            extensions: extensions.clone(),
            info: GlInfo {
                renderer,
                version: gl_string(gl, GL_VERSION),
                depth_bits: depth,
                stencil_bits: stencil,
                clear_mask: match (depth > 0, stencil > 0) {
                    (true, true) => "color|depth|stencil",
                    (true, false) => "color|depth",
                    (false, true) => "color|stencil",
                    (false, false) => "color",
                },
                invalidate: match (depth > 0, stencil > 0) {
                    (true, true) => "depth|stencil",
                    (true, false) => "depth",
                    (false, true) => "stencil",
                    (false, false) => "none (config has no depth/stencil)",
                },
                ..Default::default()
            },
        };
        s.program = unsafe { compile_program(gl, VERTEX_SHADER, FRAGMENT_SHADER)? };
        unsafe {
            (gl.gen_vertex_arrays)(1, &mut s.vao);
            (gl.bind_vertex_array)(s.vao);
            (gl.gen_buffers)(1, &mut s.vbo);
            (gl.bind_buffer)(GL_ARRAY_BUFFER, s.vbo);
            let stride = std::mem::size_of::<Vertex>() as GLsizei;
            (gl.vertex_attrib_pointer)(0, 2, GL_FLOAT, GL_FALSE, stride, ptr::null());
            (gl.enable_vertex_attrib_array)(0);
            (gl.vertex_attrib_pointer)(1, 3, GL_FLOAT, GL_FALSE, stride, 8usize as *const c_void);
            (gl.enable_vertex_attrib_array)(1);
            (gl.use_program)(s.program);
            for cap in [GL_BLEND, GL_DEPTH_TEST, GL_STENCIL_TEST, GL_SCISSOR_TEST, GL_CULL_FACE, GL_DITHER] {
                (gl.disable)(cap);
            }
            (gl.viewport)(0, 0, W as GLsizei, H as GLsizei);
        }
        // GPU timer (GL_EXT_disjoint_timer_query): never silently absent.
        if !has_token(&extensions, "GL_EXT_disjoint_timer_query") {
            s.info.timer_reason = "GL_EXT_disjoint_timer_query not advertised".into();
        } else {
            let n = CString::new("glGetQueryObjectui64vEXT").unwrap();
            let p = unsafe { (l.egl.get_proc_address)(n.as_ptr()) };
            if p.is_null() {
                s.info.timer_reason = "glGetQueryObjectui64vEXT not resolvable".into();
            } else {
                // SAFETY: eglGetProcAddress returns the entry point with the
                // gl2ext.h prototype.
                s.query_u64 = Some(unsafe { std::mem::transmute::<*mut c_void, QueryU64>(p) });
                let mut disjoint = 0;
                unsafe {
                    (gl.gen_queries)(1, &mut s.query);
                    (gl.get_integerv)(GL_GPU_DISJOINT_EXT, &mut disjoint); // clears the flag
                }
                if s.query == 0 || unsafe { (gl.get_error)() } != GL_NO_ERROR {
                    s.query_u64 = None;
                    s.info.timer_reason = "glGenQueries/GL_GPU_DISJOINT_EXT failed".into();
                } else {
                    s.info.timer = true;
                }
            }
        }
        let err = unsafe { (gl.get_error)() };
        if err != GL_NO_ERROR {
            return Err(Error::new("gl_setup", format!("GL error 0x{err:x}")));
        }
        Ok(s)
    }

    /// Uploads the static S4 geometry (once) and builds the world program.
    /// Returns the bytes uploaded.
    unsafe fn load_world(&mut self, l: &Libs, w: &WorldBuffers<'_>) -> Result<u64, Error> {
        let gl = &l.gl;
        if self.world.is_some() {
            return Err(Error::new("load_world", "world already loaded"));
        }
        if self.info.depth_bits < 24 {
            return Err(Error::new("load_world", "the S4 world needs the depth-buffer target (open_world)"));
        }
        let program = unsafe { compile_program(gl, WORLD_VERTEX_SHADER, WORLD_FRAGMENT_SHADER)? };
        let loc = |name: &str| {
            let c = CString::new(name).unwrap();
            unsafe { (gl.get_uniform_location)(program, c.as_ptr()) }
        };
        let (u_mvp, u_point_k, u_round) = (loc("u_mvp"), loc("u_point_k"), loc("u_round"));
        if u_mvp < 0 || u_point_k < 0 || u_round < 0 {
            unsafe { (gl.delete_program)(program) };
            return Err(Error::new("load_world", "world program uniforms not found"));
        }
        let mesh = |v: &[Vertex3]| -> StaticMesh {
            let mut m = StaticMesh { vao: 0, vbo: 0, count: v.len() as GLsizei };
            unsafe {
                (gl.gen_vertex_arrays)(1, &mut m.vao);
                (gl.bind_vertex_array)(m.vao);
                (gl.gen_buffers)(1, &mut m.vbo);
                (gl.bind_buffer)(GL_ARRAY_BUFFER, m.vbo);
                (gl.buffer_data)(GL_ARRAY_BUFFER, std::mem::size_of_val(v) as GLsizeiptr, v.as_ptr().cast(), GL_STATIC_DRAW);
                let stride = std::mem::size_of::<Vertex3>() as GLsizei;
                (gl.vertex_attrib_pointer)(0, 3, GL_FLOAT, GL_FALSE, stride, ptr::null());
                (gl.enable_vertex_attrib_array)(0);
                (gl.vertex_attrib_pointer)(1, 3, GL_FLOAT, GL_FALSE, stride, 12usize as *const c_void);
                (gl.enable_vertex_attrib_array)(1);
            }
            m
        };
        let world = WorldGl { program, u_mvp, u_point_k, u_round, ground: mesh(w.ground), lights: mesh(w.lights), aircraft: mesh(w.aircraft) };
        unsafe {
            // Back to the HUD's buffers so the S3 path is untouched.
            (gl.bind_vertex_array)(self.vao);
            (gl.bind_buffer)(GL_ARRAY_BUFFER, self.vbo);
        }
        self.world = Some(world);
        let err = unsafe { (gl.get_error)() };
        if err != GL_NO_ERROR {
            return Err(Error::new("load_world", format!("GL error 0x{err:x}")));
        }
        let bytes = (std::mem::size_of_val(w.ground) + std::mem::size_of_val(w.lights) + std::mem::size_of_val(w.aircraft)) as u64;
        self.info.static_vertex_bytes = bytes;
        Ok(bytes)
    }

    /// SPIKE-002: uploads a validated visual sidecar (textures, one vertex
    /// and one index buffer) and builds the photo and sky programs. The S4
    /// world (`load_world`) stays loaded for the lights. Returns the static
    /// bytes handed to GL.
    unsafe fn load_photo(&mut self, l: &Libs, world: &gx_world::WorldAsset, visual: &gx_world::VisualAsset) -> Result<PhotoMemory, Error> {
        let gl = &l.gl;
        if self.photo.is_some() {
            return Err(Error::new("load_photo", "photo scene already loaded"));
        }
        if self.info.depth_bits < 24 {
            return Err(Error::new("load_photo", "the photo scene needs the depth-buffer target (open_world)"));
        }
        let plan = prepare_photo(world, visual, &self.extensions, gx_world::VisualLimits::default())
            .map_err(|e| Error::new("load_photo", e))?;
        let program = unsafe { compile_program(gl, PHOTO_VERTEX_SHADER, &photo_fragment_shader())? };
        let sky = unsafe { compile_program(gl, SKY_VERTEX_SHADER, &sky_fragment_shader())? };
        let loc = |p: GLuint, name: &str| {
            let c = CString::new(name).unwrap();
            unsafe { (gl.get_uniform_location)(p, c.as_ptr()) }
        };
        let u = PhotoUniforms {
            mvp: loc(program, "u_mvp"),
            rel: loc(program, "u_rel"),
            normal: loc(program, "u_normal"),
            base_color: loc(program, "u_base_color"),
            factor: loc(program, "u_base_color_factor"),
            roughness: loc(program, "u_roughness"),
            gloss: loc(program, "u_gloss"),
            eye: loc(program, "u_eye"),
            time: loc(program, "u_time"),
            sun: loc(program, "u_sun"),
            joints: loc(program, "u_joints"),
        };
        let sky_u = ["u_right", "u_up", "u_forward", "u_tan", "u_sun"].map(|n| loc(sky, n));
        if [u.mvp, u.rel, u.normal, u.base_color, u.factor, u.roughness, u.gloss, u.eye, u.time, u.sun, u.joints].contains(&-1) || sky_u.contains(&-1) {
            unsafe {
                (gl.delete_program)(program);
                (gl.delete_program)(sky);
            }
            return Err(Error::new("load_photo", "photo/sky program uniforms not found"));
        }
        let aniso = has_token(&self.extensions, "GL_EXT_texture_filter_anisotropic");
        // SPIKE-002 default 2x; GX_S002_ANISO overrides for cost experiments.
        let max_aniso: f32 = std::env::var("GX_S002_ANISO").ok().and_then(|v| v.parse().ok()).unwrap_or(2.0);
        let mut textures = Vec::with_capacity(plan.textures.len());
        let (mut vao, mut vbo, mut ibo, mut sky_vao) = (0, 0, 0, 0);
        unsafe {
            (gl.active_texture)(GL_TEXTURE0);
            let no_textures = std::env::var("GX_S002_SKIP").is_ok_and(|v| v.contains("upload"));
            for t in plan.textures.iter().filter(|_| !no_textures) {
                let mut name = 0;
                (gl.gen_textures)(1, &mut name);
                (gl.bind_texture)(GL_TEXTURE_2D, name);
                let min = if t.levels > 1 { GL_LINEAR_MIPMAP_LINEAR } else { GL_LINEAR };
                (gl.tex_parameteri)(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, min);
                (gl.tex_parameteri)(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                let wrap = if t.repeat { GL_REPEAT } else { GL_CLAMP_TO_EDGE };
                (gl.tex_parameteri)(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap);
                (gl.tex_parameteri)(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap);
                if aniso && t.levels > 1 && max_aniso > 1.0 {
                    (gl.tex_parameterf)(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, max_aniso);
                }
                let (mut w, mut h) = (t.width, t.height);
                for (level, bytes) in t.mip_levels.iter().enumerate() {
                    if t.compressed {
                        (gl.compressed_tex_image_2d)(GL_TEXTURE_2D, level as GLint, t.gl_internal_format, w as GLsizei, h as GLsizei, 0, bytes.len() as GLsizei, bytes.as_ptr().cast());
                    } else {
                        (gl.tex_image_2d)(GL_TEXTURE_2D, level as GLint, t.gl_internal_format as GLint, w as GLsizei, h as GLsizei, 0, GL_RGBA, GL_UNSIGNED_BYTE, bytes.as_ptr().cast());
                    }
                    w = (w / 2).max(1);
                    h = (h / 2).max(1);
                }
                textures.push((t.id, name));
            }
            (gl.gen_vertex_arrays)(1, &mut vao);
            (gl.bind_vertex_array)(vao);
            (gl.gen_buffers)(1, &mut vbo);
            (gl.bind_buffer)(GL_ARRAY_BUFFER, vbo);
            (gl.buffer_data)(GL_ARRAY_BUFFER, std::mem::size_of_val(plan.vertices.as_slice()) as GLsizeiptr, plan.vertices.as_ptr().cast(), GL_STATIC_DRAW);
            (gl.gen_buffers)(1, &mut ibo);
            (gl.bind_buffer)(GL_ELEMENT_ARRAY_BUFFER, ibo);
            (gl.buffer_data)(GL_ELEMENT_ARRAY_BUFFER, std::mem::size_of_val(plan.indices.as_slice()) as GLsizeiptr, plan.indices.as_ptr().cast(), GL_STATIC_DRAW);
            let stride = std::mem::size_of::<TexturedVertex>() as GLsizei;
            (gl.vertex_attrib_pointer)(0, 3, GL_FLOAT, GL_FALSE, stride, ptr::null());
            (gl.enable_vertex_attrib_array)(0);
            (gl.vertex_attrib_pointer)(1, 3, GL_FLOAT, GL_FALSE, stride, 12usize as *const c_void);
            (gl.enable_vertex_attrib_array)(1);
            (gl.vertex_attrib_pointer)(2, 2, GL_FLOAT, GL_FALSE, stride, 24usize as *const c_void);
            (gl.enable_vertex_attrib_array)(2);
            (gl.vertex_attrib_pointer)(3, 1, GL_FLOAT, GL_FALSE, stride, 32usize as *const c_void);
            (gl.enable_vertex_attrib_array)(3);
            (gl.gen_vertex_arrays)(1, &mut sky_vao);
            // Back to the HUD's buffers so the S3 path is untouched.
            (gl.bind_vertex_array)(self.vao);
            (gl.bind_buffer)(GL_ARRAY_BUFFER, self.vbo);
        }
        let skip = std::env::var("GX_S002_SKIP").unwrap_or_default();
        self.photo = Some(PhotoGl { program, u, sky, sky_vao, sky_u, vao, vbo, ibo, textures, batches: plan.batches, joints: plan.joints, skip });
        let err = unsafe { (gl.get_error)() };
        if err != GL_NO_ERROR {
            return Err(Error::new("load_photo", format!("GL error 0x{err:x}")));
        }
        Ok(plan.memory)
    }

    /// The SPIKE-002 frame body (between clear and HUD): terrain, decals,
    /// aircraft, sky, lights. Returns the draw calls issued.
    unsafe fn draw_photo(&self, gl: &GlApi, p: &PhotoGl, pd: &PhotoDraw) -> u32 {
        const I3: [f32; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let mut draws = 0;
        unsafe {
            (gl.use_program)(p.program);
            (gl.uniform1i)(p.u.base_color, 0);
            (gl.uniform3fv)(p.u.eye, 1, pd.eye.as_ptr());
            (gl.uniform3fv)(p.u.sun, 1, SUN_ENU.as_ptr());
            (gl.uniform1f)(p.u.time, pd.time_s);
            let rows = crate::photo::joint_rows(&p.joints, &pd.drivers);
            debug_assert!(rows.len() <= crate::photo::JOINT_ROWS);
            (gl.uniform4fv)(p.u.joints, rows.len() as GLsizei, rows.as_ptr().cast());
            (gl.active_texture)(GL_TEXTURE0);
            (gl.bind_vertex_array)(p.vao);
            let mut batch = |b: &PhotoBatch, mvp: &[f32; 16], rel: &[f32; 16], normal: &[f32; 9], gloss: f32| {
                let texture = p.textures.iter().find(|(id, _)| *id == b.texture_id).map_or(0, |(_, n)| *n);
                (gl.uniform_matrix4fv)(p.u.mvp, 1, GL_FALSE, mvp.as_ptr());
                (gl.uniform_matrix4fv)(p.u.rel, 1, GL_FALSE, rel.as_ptr());
                (gl.uniform_matrix3fv)(p.u.normal, 1, GL_FALSE, normal.as_ptr());
                (gl.uniform4fv)(p.u.factor, 1, b.base_color_factor.as_ptr());
                (gl.uniform1f)(p.u.roughness, b.roughness);
                (gl.uniform1f)(p.u.gloss, gloss);
                (gl.bind_texture)(GL_TEXTURE_2D, texture);
                (gl.draw_elements)(GL_TRIANGLES, b.index_count as GLsizei, GL_UNSIGNED_INT, (b.first_index as usize * 4) as *const c_void);
                draws += 1;
            };
            // Terrain: depth tested and written.
            (gl.enable)(GL_DEPTH_TEST);
            (gl.depth_func)(GL_LESS);
            (gl.depth_mask)(1);
            for b in p.batches.iter().filter(|b| b.kind == gx_world::VisualKind::Terrain && !p.skip.contains("terrain")) {
                batch(b, &pd.world.ground_mvp, &pd.ground_rel, &I3, 0.0);
            }
            // Decals: painter's order over the terrain, like the S4 ground.
            (gl.disable)(GL_DEPTH_TEST);
            (gl.depth_mask)(0);
            for b in p.batches.iter().filter(|b| b.kind == gx_world::VisualKind::Decal && !p.skip.contains("decals")) {
                batch(b, &pd.world.ground_mvp, &pd.ground_rel, &I3, 0.0);
            }
            // Buildings: after the decals (markings never paint a wall),
            // depth tested and written, like terrain.
            (gl.enable)(GL_DEPTH_TEST);
            (gl.depth_func)(GL_LESS);
            (gl.depth_mask)(1);
            for b in p.batches.iter().filter(|b| b.kind == gx_world::VisualKind::Building && !p.skip.contains("buildings")) {
                batch(b, &pd.world.ground_mvp, &pd.ground_rel, &I3, 0.0);
            }
            // Aircraft: depth tested against the terrain.
            if let Some(m) = pd.world.aircraft_mvp.as_ref() {
                (gl.enable)(GL_DEPTH_TEST);
                (gl.depth_func)(GL_LESS);
                (gl.depth_mask)(1);
                for b in p.batches.iter().filter(|b| b.kind == gx_world::VisualKind::Aircraft && !p.skip.contains("aircraft")) {
                    batch(b, m, &pd.aircraft_rel, &pd.aircraft_rot, 0.35 * (1.0 - b.roughness) + 0.1);
                }
            }
            // Sky only where nothing was drawn (cleared depth 1.0).
            (gl.enable)(GL_DEPTH_TEST);
            (gl.depth_func)(GL_LEQUAL);
            (gl.depth_mask)(0);
            (gl.use_program)(p.sky);
            for (k, v) in [pd.right, pd.up, pd.forward].iter().enumerate() {
                (gl.uniform3fv)(p.sky_u[k], 1, v.as_ptr());
            }
            (gl.uniform2fv)(p.sky_u[3], 1, pd.tan_half.as_ptr());
            (gl.uniform3fv)(p.sky_u[4], 1, SUN_ENU.as_ptr());
            if !p.skip.contains("sky") {
                (gl.bind_vertex_array)(p.sky_vao);
                (gl.draw_arrays)(GL_TRIANGLES, 0, 3);
                draws += 1;
            }
            // Airport lights (the S4 world program and buffer), depth tested
            // against terrain and aircraft, no writes.
            if let Some(w) = self.world.as_ref().filter(|_| !p.skip.contains("lights")) {
                (gl.depth_func)(GL_LESS);
                (gl.use_program)(w.program);
                (gl.uniform1f)(w.u_point_k, pd.world.point_k);
                (gl.uniform1f)(w.u_round, 1.0);
                (gl.uniform_matrix4fv)(w.u_mvp, 1, GL_FALSE, pd.world.ground_mvp.as_ptr());
                (gl.bind_vertex_array)(w.lights.vao);
                (gl.draw_arrays)(GL_POINTS, 0, w.lights.count);
                draws += 1;
            }
            (gl.disable)(GL_DEPTH_TEST);
        }
        draws
    }

    /// One frame: (begin timer) clear, the S4 world draws if `world` is
    /// given, the HUD draw, invalidate, `before_swap` (pixel probes), swap,
    /// (end timer), fence wait, timer read. Returns the draw-call count too.
    unsafe fn frame(
        &mut self,
        l: &Libs,
        world: Option<&WorldDraw>,
        photo: Option<&PhotoDraw>,
        verts: &[Vertex],
        before_swap: &mut dyn FnMut(&GlApiRef),
        swap: &mut dyn FnMut() -> bool,
    ) -> Result<(GpuSample, DrawTiming, u32), Error> {
        let gl = &l.gl;
        let n = verts.len().min(MAX_VERTICES);
        let mut t = DrawTiming { draw_start_ns: monotonic_ns(), ..Default::default() };
        let mut draws = 0u32;
        unsafe {
            if self.query_u64.is_some() {
                (gl.begin_query)(GL_TIME_ELAPSED_EXT, self.query);
            }
            let clear = world.or(photo.map(|p| &p.world)).map_or([0.0; 3], |w| w.clear);
            (gl.clear_color)(clear[0], clear[1], clear[2], 1.0);
            if self.clear_mask & GL_DEPTH_BUFFER_BIT != 0 {
                (gl.depth_mask)(1);
                (gl.clear_depthf)(1.0);
            }
            (gl.clear)(self.clear_mask);
            if let (Some(p), Some(pd)) = (self.photo.as_ref(), photo) {
                if !p.skip.contains("all") {
                    draws += self.draw_photo(gl, p, pd);
                }
            } else if let (Some(w), Some(wd)) = (self.world.as_ref(), world) {
                (gl.use_program)(w.program);
                (gl.uniform1f)(w.u_point_k, wd.point_k);
                // Ground: painter's order, no depth test, no depth writes.
                (gl.disable)(GL_DEPTH_TEST);
                (gl.depth_mask)(0);
                (gl.uniform1f)(w.u_round, 0.0);
                (gl.uniform_matrix4fv)(w.u_mvp, 1, GL_FALSE, wd.ground_mvp.as_ptr());
                (gl.bind_vertex_array)(w.ground.vao);
                (gl.draw_arrays)(GL_TRIANGLES, 0, w.ground.count);
                draws += 1;
                (gl.enable)(GL_DEPTH_TEST);
                (gl.depth_func)(GL_LESS);
                if let Some(m) = wd.aircraft_mvp {
                    (gl.depth_mask)(1);
                    (gl.uniform_matrix4fv)(w.u_mvp, 1, GL_FALSE, m.as_ptr());
                    (gl.bind_vertex_array)(w.aircraft.vao);
                    (gl.draw_arrays)(GL_TRIANGLES, 0, w.aircraft.count);
                    draws += 1;
                }
                // Lights: depth tested against the aircraft, no writes.
                (gl.depth_mask)(0);
                (gl.uniform1f)(w.u_round, 1.0);
                (gl.uniform_matrix4fv)(w.u_mvp, 1, GL_FALSE, wd.ground_mvp.as_ptr());
                (gl.bind_vertex_array)(w.lights.vao);
                (gl.draw_arrays)(GL_POINTS, 0, w.lights.count);
                draws += 1;
                (gl.disable)(GL_DEPTH_TEST);
            }
            (gl.use_program)(self.program);
            (gl.bind_vertex_array)(self.vao);
            (gl.bind_buffer)(GL_ARRAY_BUFFER, self.vbo);
            (gl.buffer_data)(
                GL_ARRAY_BUFFER,
                (n * std::mem::size_of::<Vertex>()) as GLsizeiptr,
                verts.as_ptr().cast(),
                GL_STREAM_DRAW,
            );
            (gl.draw_arrays)(GL_TRIANGLES, 0, n as GLsizei);
            draws += 1;
            if !self.invalidate.is_empty() {
                (gl.invalidate_framebuffer)(GL_FRAMEBUFFER, self.invalidate.len() as GLsizei, self.invalidate.as_ptr());
            }
        }
        before_swap(&GlApiRef(gl));
        let swapped = swap();
        t.swap_end_ns = monotonic_ns();
        unsafe {
            if self.query_u64.is_some() {
                (gl.end_query)(GL_TIME_ELAPSED_EXT);
            }
            let fence = (gl.fence_sync)(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
            let err = (gl.get_error)();
            if fence.is_null() || err != GL_NO_ERROR {
                return Err(Error::new("draw", format!("GL error 0x{err:x}")));
            }
            let wr = (gl.client_wait_sync)(fence, GL_SYNC_FLUSH_COMMANDS_BIT, FENCE_TIMEOUT_NS);
            (gl.delete_sync)(fence);
            t.fence_ns = monotonic_ns();
            if !swapped {
                return Err(Error::new("egl_swap", format!("0x{:x}", (l.egl.get_error)())));
            }
            if wr != GL_ALREADY_SIGNALED && wr != GL_CONDITION_SATISFIED {
                return Err(Error::new("fence", format!("glClientWaitSync 0x{wr:x}")));
            }
            let gpu = match self.query_u64 {
                None => GpuSample { ns: None, status: "unavailable" },
                Some(read_u64) => {
                    let (mut available, mut elapsed, mut disjoint) = (0, 0u64, 0);
                    (gl.get_query_objectuiv)(self.query, GL_QUERY_RESULT_AVAILABLE_EXT, &mut available);
                    if available != 0 {
                        read_u64(self.query, GL_QUERY_RESULT_EXT, &mut elapsed);
                    }
                    (gl.get_integerv)(GL_GPU_DISJOINT_EXT, &mut disjoint);
                    if available == 0 {
                        GpuSample { ns: None, status: "not_ready" }
                    } else if disjoint != 0 {
                        GpuSample { ns: None, status: "disjoint" }
                    } else {
                        GpuSample { ns: Some(elapsed), status: "ok" }
                    }
                }
            };
            Ok((gpu, t, draws))
        }
    }

    unsafe fn destroy(&mut self, gl: &GlApi) {
        unsafe {
            if let Some(p) = self.photo.take() {
                (gl.delete_buffers)(1, &p.vbo);
                (gl.delete_buffers)(1, &p.ibo);
                (gl.delete_vertex_arrays)(1, &p.vao);
                (gl.delete_vertex_arrays)(1, &p.sky_vao);
                for (_, t) in &p.textures {
                    (gl.delete_textures)(1, t);
                }
                (gl.delete_program)(p.program);
                (gl.delete_program)(p.sky);
            }
            if let Some(w) = self.world.take() {
                for m in [&w.ground, &w.lights, &w.aircraft] {
                    (gl.delete_buffers)(1, &m.vbo);
                    (gl.delete_vertex_arrays)(1, &m.vao);
                }
                (gl.delete_program)(w.program);
            }
            if self.query != 0 {
                (gl.delete_queries)(1, &self.query);
                self.query = 0;
            }
            if self.vbo != 0 {
                (gl.delete_buffers)(1, &self.vbo);
                self.vbo = 0;
            }
            if self.vao != 0 {
                (gl.delete_vertex_arrays)(1, &self.vao);
                self.vao = 0;
            }
            if self.program != 0 {
                (gl.delete_program)(self.program);
                self.program = 0;
            }
        }
    }
}

/// Read-only GL handle for pixel probes inside a frame.
pub struct GlApiRef<'a>(&'a GlApi);

impl GlApiRef<'_> {
    /// RGB of pixel (x, y), top-left origin, read from the back buffer.
    pub fn pixel(&self, x: u32, y: u32) -> [u8; 3] {
        let mut px = [0u8; 4];
        // SAFETY: 1x1 RGBA read into a 4-byte buffer on the current context.
        unsafe {
            (self.0.read_pixels)(x as GLint, (H - 1 - y) as GLint, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, px.as_mut_ptr().cast())
        };
        [px[0], px[1], px[2]]
    }

    /// The whole back buffer as top-down RGB rows (SPIKE-002 captures).
    pub fn frame_rgb(&self) -> Vec<u8> {
        let mut rgba = vec![0u8; (W * H * 4) as usize];
        // SAFETY: W x H RGBA read into a buffer of exactly that size.
        unsafe { (self.0.read_pixels)(0, 0, W as GLint, H as GLint, GL_RGBA, GL_UNSIGNED_BYTE, rgba.as_mut_ptr().cast()) };
        let row = W as usize * 4;
        let mut rgb = Vec::with_capacity((W * H * 3) as usize);
        for y in (0..H as usize).rev() {
            for px in rgba[y * row..(y + 1) * row].chunks_exact(4) {
                rgb.extend_from_slice(&px[..3]);
            }
        }
        rgb
    }
}

fn gl_string(gl: &GlApi, name: GLenum) -> String {
    // SAFETY: glGetString returns a static C string or null.
    unsafe {
        let p = (gl.get_string)(name);
        if p.is_null() {
            String::new()
        } else {
            CStr::from_ptr(p.cast()).to_string_lossy().into_owned()
        }
    }
}

unsafe fn compile_program(gl: &GlApi, vs: &str, fs: &str) -> Result<GLuint, Error> {
    unsafe fn shader(gl: &GlApi, kind: GLenum, src: &str) -> Result<GLuint, Error> {
        unsafe {
            let s = (gl.create_shader)(kind);
            let c = CString::new(src).unwrap();
            let p = c.as_ptr();
            (gl.shader_source)(s, 1, &p, ptr::null());
            (gl.compile_shader)(s);
            let mut ok = 0;
            (gl.get_shader_iv)(s, GL_COMPILE_STATUS, &mut ok);
            if ok == 0 {
                let mut log = [0 as c_char; 1024];
                (gl.get_shader_info_log)(s, 1024, ptr::null_mut(), log.as_mut_ptr());
                (gl.delete_shader)(s);
                let msg = CStr::from_ptr(log.as_ptr()).to_string_lossy().into_owned();
                return Err(Error::new("shader_compile", msg));
            }
            Ok(s)
        }
    }
    unsafe {
        let v = shader(gl, GL_VERTEX_SHADER, vs)?;
        let f = match shader(gl, GL_FRAGMENT_SHADER, fs) {
            Ok(f) => f,
            Err(e) => {
                (gl.delete_shader)(v);
                return Err(e);
            }
        };
        let p = (gl.create_program)();
        (gl.attach_shader)(p, v);
        (gl.attach_shader)(p, f);
        (gl.link_program)(p);
        (gl.delete_shader)(v);
        (gl.delete_shader)(f);
        let mut ok = 0;
        (gl.get_program_iv)(p, GL_LINK_STATUS, &mut ok);
        if ok == 0 {
            (gl.delete_program)(p);
            return Err(Error::new("program_link", "GLES program link failed"));
        }
        Ok(p)
    }
}

fn open_rw(path: &str) -> Result<Fd, Error> {
    let p = CString::new(path).unwrap();
    // SAFETY: plain open(2).
    let fd = unsafe { libc::open(p.as_ptr(), libc::O_RDWR | libc::O_CLOEXEC) };
    if fd < 0 {
        Err(Error::new("open", format!("{path}: {}", std::io::Error::last_os_error())))
    } else {
        Ok(fd)
    }
}

// ------------------------------------------------------------- flip events --

#[derive(Clone, Copy, Debug)]
struct FlipEvent {
    token: u64,
    sequence: u32,
    ns: u64,
    crtc: u32,
}

thread_local! {
    /// Events delivered by drmHandleEvent on this (the render) thread.
    static EVENTS: RefCell<Vec<FlipEvent>> = RefCell::new(Vec::with_capacity(16));
}

/// `user_data` is the commit's token itself (an integer, never a pointer),
/// so a late or duplicated event can never dereference freed memory.
unsafe extern "C" fn on_page_flip(_fd: c_int, seq: c_uint, sec: c_uint, usec: c_uint, crtc: c_uint, data: *mut c_void) {
    let ev = FlipEvent { token: data as usize as u64, sequence: seq, ns: sec as u64 * 1_000_000_000 + usec as u64 * 1_000, crtc };
    EVENTS.with(|e| {
        if let Ok(mut v) = e.try_borrow_mut() {
            if v.len() < 64 {
                v.push(ev);
            }
        }
    });
}

fn take_events() -> Vec<FlipEvent> {
    EVENTS.with(|e| e.try_borrow_mut().map(|mut v| std::mem::take(&mut *v)).unwrap_or_default())
}

// --------------------------------------------------------------------- KMS --

#[derive(Default)]
struct Props {
    conn_crtc: u32,
    crtc_mode: u32,
    crtc_active: u32,
    fb: u32,
    plane_crtc: u32,
    sx: u32,
    sy: u32,
    sw: u32,
    sh: u32,
    cx: u32,
    cy: u32,
    cw: u32,
    ch: u32,
}

/// Display facts, reported once at start.
#[derive(Clone, Debug, Default)]
pub struct DisplayInfo {
    pub target: &'static str,
    pub drm_master: bool,
    pub connector_id: u32,
    pub crtc_id: u32,
    pub plane_id: u32,
    pub mode_name: String,
    pub width: u16,
    pub height: u16,
    pub clock_khz: u32,
    pub htotal: u16,
    pub vtotal: u16,
    pub vrefresh: u32,
    pub period_ns: u64,
    pub vblanks_per_frame: u64,
    pub saved_crtc_buffer: u32,
    pub saved_crtc_mode_valid: bool,
    pub gl: GlInfo,
}

/// One presented frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FrameReport {
    pub token: u64,
    /// Atomic commit issued (monotonic ns).
    pub commit_ns: u64,
    /// Kernel flip stamp when trustworthy (fresh); `None` if stale, anomalous
    /// or offscreen.
    pub flip_ns: Option<u64>,
    /// Vblank this frame was (or is estimated to be) scanned out from.
    pub vblank_ns: u64,
    pub sequence: u32,
    /// `ok`, `ok_stale_vblank_stamp`, an anomaly name, or `offscreen`.
    pub status: &'static str,
    /// Fresh-to-fresh interval to the directly preceding frame.
    pub interval_ns: Option<u64>,
    pub gpu: GpuSample,
    pub timing: DrawTiming,
    /// Unrelated (other-token) events seen while waiting.
    pub foreign_events: u32,
    /// Draw calls in this frame's single pass.
    pub draws: u32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PresentCounts {
    pub presented: u64,
    pub stale_stamps: u64,
    pub errors: u64,
}

pub fn status_name(s: EventStatus) -> &'static str {
    match s {
        EventStatus::Presented => "ok",
        EventStatus::PresentedWithStaleVblankStamp => "ok_stale_vblank_stamp",
        EventStatus::WrongCrtc => "wrong_crtc",
        EventStatus::NoPendingCommit => "no_pending_commit",
        EventStatus::DuplicateCommitEvent => "duplicate_commit_event",
        EventStatus::UnknownCommit => "unknown_commit",
        EventStatus::SequenceRegressed => "sequence_regressed",
        EventStatus::TimestampRegressed => "timestamp_regressed",
        EventStatus::StampInconsistent => "stamp_inconsistent",
        EventStatus::TimestampBeforeCommit => "timestamp_before_commit",
        EventStatus::RepeatedStaleStamp => "stale_stamp_repeated",
    }
}

/// Direct-scanout presenter. Construct, use and drop on one thread.
pub struct KmsDisplay {
    libs: Libs,
    fd: Fd,
    master: bool,
    connector: u32,
    crtc: u32,
    crtc_index: u32,
    plane: u32,
    mode: drmModeModeInfo,
    saved: Option<drmModeCrtc>,
    blob: u32,
    props: Props,
    target: EglTarget,
    gl: Option<Gl>,
    tracker: PresentTracker,
    next_token: u64,
    scanout: Option<*mut gbm_bo>,
    fbs: Vec<(usize, u32)>,
    modeset_done: bool,
    modeset_path: &'static str,
    pacing: VblankClock,
    intervals: IntervalTracker,
    last_vblank_ns: Option<u64>,
    frame: u64,
    closed: bool,
    depth: bool,
}

impl KmsDisplay {
    /// Opens card0, acquires DRM master (fails if Xorg still holds it) and
    /// sets up DP-1 1280x720@60 + GBM/EGL/GLES. Nothing reaches the screen
    /// until the first [`Self::present`] (its commit performs the modeset).
    pub fn open(card: &str) -> Result<Self, Error> {
        Self::open_with(card, false)
    }

    /// [`Self::open`] with a 24-bit depth buffer, for the S4 world.
    pub fn open_world(card: &str) -> Result<Self, Error> {
        Self::open_with(card, true)
    }

    /// Uploads the static S4 geometry once (needs [`Self::open_world`]).
    pub fn load_world(&mut self, w: &WorldBuffers<'_>) -> Result<u64, Error> {
        let gl = self.gl.as_mut().ok_or_else(|| Error::new("load_world", "GL not initialized"))?;
        unsafe { gl.load_world(&self.libs, w) }
    }

    fn open_with(card: &str, depth: bool) -> Result<Self, Error> {
        let libs = Libs::load(true)?;
        let fd = open_rw(card)?;
        let mut s = Self {
            libs,
            fd,
            master: false,
            connector: 0,
            crtc: 0,
            crtc_index: 0,
            plane: 0,
            mode: drmModeModeInfo::default(),
            saved: None,
            blob: 0,
            props: Props::default(),
            target: EglTarget::empty(),
            gl: None,
            tracker: PresentTracker::new(0),
            next_token: 1,
            scanout: None,
            fbs: Vec::with_capacity(8),
            modeset_done: false,
            modeset_path: "none",
            pacing: VblankClock::from_mode(1, 1, 1, 1).unwrap(),
            intervals: IntervalTracker::default(),
            last_vblank_ns: None,
            frame: 0,
            closed: false,
            depth,
        };
        // On error `s` drops here and tears down whatever was created.
        unsafe { s.init()? };
        Ok(s)
    }

    fn drm(&self) -> &DrmApi {
        self.libs.drm.as_ref().expect("KMS loads libdrm")
    }

    unsafe fn init(&mut self) -> Result<(), Error> {
        let fd = self.fd;
        unsafe {
            if (self.drm().set_master)(fd) != 0 {
                return Err(Error::new(
                    "drm_set_master",
                    format!("{} (is Xorg/LightDM still running?)", std::io::Error::last_os_error()),
                ));
            }
            self.master = true;
            if (self.drm().set_client_cap)(fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1) != 0
                || (self.drm().set_client_cap)(fd, DRM_CLIENT_CAP_ATOMIC, 1) != 0
            {
                return Err(Error::errno("drm_client_cap"));
            }
            let mut mono = 0;
            if (self.drm().get_cap)(fd, DRM_CAP_TIMESTAMP_MONOTONIC, &mut mono) != 0 || mono != 1 {
                return Err(Error::new("drm_timestamp", "flip events are not CLOCK_MONOTONIC"));
            }
            self.discover()?;
            let (depth, stencil) = self.target.create(&self.libs, fd, GBM_BO_USE_SCANOUT | GBM_BO_USE_RENDERING, self.depth)?;
            self.gl = Some(Gl::new(&self.libs, depth, stencil)?);
        }
        self.tracker = PresentTracker::new(self.crtc);
        self.pacing = VblankClock::from_mode(self.mode.clock, self.mode.htotal, self.mode.vtotal, TARGET_FPS)
            .ok_or_else(|| Error::new("mode", "mode has no timing"))?;
        Ok(())
    }

    unsafe fn props_of(&self, obj: u32, ty: u32) -> Vec<(String, u32, u64)> {
        let mut out = Vec::new();
        unsafe {
            let ps = (self.drm().object_get_properties)(self.fd, obj, ty);
            if ps.is_null() {
                return out;
            }
            for i in 0..(*ps).count_props as usize {
                let id = *(*ps).props.add(i);
                let value = *(*ps).prop_values.add(i);
                let p = (self.drm().get_property)(self.fd, id);
                if !p.is_null() {
                    let name = CStr::from_ptr((*p).name.as_ptr()).to_string_lossy().into_owned();
                    out.push((name, id, value));
                    (self.drm().free_property)(p);
                }
            }
            (self.drm().free_object_properties)(ps);
        }
        out
    }

    unsafe fn find_prop(&self, obj: u32, ty: u32, name: &str) -> u32 {
        unsafe { self.props_of(obj, ty) }.into_iter().find(|p| p.0 == name).map_or(0, |p| p.1)
    }

    /// Topology exactly as the M0-001C candidate: the one DisplayPort
    /// connector with type id 1, connected; its progressive 1280x720 mode
    /// closest to 60 Hz; the encoder's CRTC (else the first possible); the
    /// XRGB8888 primary plane of that CRTC; the pre-modeset CRTC saved.
    unsafe fn discover(&mut self) -> Result<(), Error> {
        let fd = self.fd;
        unsafe {
            let res = (self.drm().get_resources)(fd);
            if res.is_null() {
                return Err(Error::errno("drm_resources"));
            }
            let r = &*res;
            let ids = std::slice::from_raw_parts(r.connectors, r.count_connectors.max(0) as usize).to_vec();
            let crtcs = std::slice::from_raw_parts(r.crtcs, r.count_crtcs.max(0) as usize).to_vec();
            (self.drm().free_resources)(res);
            let mut found = Vec::new();
            for id in ids {
                let c = (self.drm().get_connector)(fd, id);
                if c.is_null() {
                    continue;
                }
                if (*c).connector_type == DRM_MODE_CONNECTOR_DisplayPort && (*c).connector_type_id == 1 {
                    found.push((id, (*c).connection == DRM_MODE_CONNECTED));
                }
                (self.drm().free_connector)(c);
            }
            match found.as_slice() {
                [] => return Err(Error::new("connector", "no DP-1 connector")),
                [(_, false)] => return Err(Error::new("connector", "DP-1 is disconnected")),
                [(id, true)] => self.connector = *id,
                _ => return Err(Error::new("connector", "ambiguous DP-1")),
            }
            let c = (self.drm().get_connector)(fd, self.connector);
            if c.is_null() {
                return Err(Error::errno("connector"));
            }
            let modes = std::slice::from_raw_parts((*c).modes, (*c).count_modes.max(0) as usize);
            let best = modes
                .iter()
                .filter(|m| m.hdisplay == 1280 && m.vdisplay == 720 && m.flags & ((1 << 4) | (1 << 5)) == 0)
                .filter_map(|m| {
                    let r = refresh_mhz(m)?;
                    (59_000..=61_000).contains(&r).then_some((r.abs_diff(60_000), *m))
                })
                .min_by_key(|(e, _)| *e)
                .map(|(_, m)| m);
            let encoder_ids: Vec<u32> = std::iter::once((*c).encoder_id)
                .chain(std::slice::from_raw_parts((*c).encoders, (*c).count_encoders.max(0) as usize).iter().copied())
                .collect();
            (self.drm().free_connector)(c);
            self.mode = best.ok_or_else(|| Error::new("mode", "DP-1 has no progressive 1280x720@~60 mode"))?;
            let mut possible = 0;
            for (i, &enc) in encoder_ids.iter().enumerate() {
                if enc == 0 || self.crtc != 0 {
                    continue;
                }
                let e = (self.drm().get_encoder)(fd, enc);
                if e.is_null() {
                    continue;
                }
                if i == 0 && (*e).crtc_id != 0 {
                    self.crtc = (*e).crtc_id;
                } else if possible == 0 {
                    possible = (*e).possible_crtcs;
                }
                (self.drm().free_encoder)(e);
            }
            if self.crtc == 0 {
                self.crtc = crtcs.iter().take(32).enumerate().find(|(i, _)| possible & (1 << i) != 0).map_or(0, |(_, &c)| c);
            }
            if self.crtc == 0 {
                return Err(Error::new("crtc", "no CRTC can drive DP-1"));
            }
            self.crtc_index = crtcs.iter().position(|&c| c == self.crtc).unwrap_or(0) as u32;
            let pr = (self.drm().get_plane_resources)(fd);
            if pr.is_null() {
                return Err(Error::errno("planes"));
            }
            let planes = std::slice::from_raw_parts((*pr).planes, (*pr).count_planes as usize).to_vec();
            (self.drm().free_plane_resources)(pr);
            for id in planes {
                if self.plane != 0 {
                    break;
                }
                let p = (self.drm().get_plane)(fd, id);
                if p.is_null() {
                    continue;
                }
                let on_crtc = (*p).possible_crtcs & (1 << self.crtc_index) != 0;
                let formats = std::slice::from_raw_parts((*p).formats, (*p).count_formats as usize);
                let xrgb = formats.contains(&DRM_FORMAT_XRGB8888);
                (self.drm().free_plane)(p);
                let primary = self
                    .props_of(id, DRM_MODE_OBJECT_PLANE)
                    .iter()
                    .any(|(n, _, v)| n == "type" && *v == DRM_PLANE_TYPE_PRIMARY);
                if on_crtc && xrgb && primary {
                    self.plane = id;
                }
            }
            if self.plane == 0 {
                return Err(Error::new("planes", format!("no XRGB8888 primary plane for CRTC {}", self.crtc)));
            }
            let pl = self.plane;
            self.props = Props {
                conn_crtc: self.find_prop(self.connector, DRM_MODE_OBJECT_CONNECTOR, "CRTC_ID"),
                crtc_mode: self.find_prop(self.crtc, DRM_MODE_OBJECT_CRTC, "MODE_ID"),
                crtc_active: self.find_prop(self.crtc, DRM_MODE_OBJECT_CRTC, "ACTIVE"),
                fb: self.find_prop(pl, DRM_MODE_OBJECT_PLANE, "FB_ID"),
                plane_crtc: self.find_prop(pl, DRM_MODE_OBJECT_PLANE, "CRTC_ID"),
                sx: self.find_prop(pl, DRM_MODE_OBJECT_PLANE, "SRC_X"),
                sy: self.find_prop(pl, DRM_MODE_OBJECT_PLANE, "SRC_Y"),
                sw: self.find_prop(pl, DRM_MODE_OBJECT_PLANE, "SRC_W"),
                sh: self.find_prop(pl, DRM_MODE_OBJECT_PLANE, "SRC_H"),
                cx: self.find_prop(pl, DRM_MODE_OBJECT_PLANE, "CRTC_X"),
                cy: self.find_prop(pl, DRM_MODE_OBJECT_PLANE, "CRTC_Y"),
                cw: self.find_prop(pl, DRM_MODE_OBJECT_PLANE, "CRTC_W"),
                ch: self.find_prop(pl, DRM_MODE_OBJECT_PLANE, "CRTC_H"),
            };
            let p = &self.props;
            if [p.conn_crtc, p.crtc_mode, p.crtc_active, p.fb, p.plane_crtc, p.sx, p.sy, p.sw, p.sh, p.cx, p.cy, p.cw, p.ch]
                .contains(&0)
            {
                return Err(Error::new("atomic_props", "a required atomic property is missing"));
            }
            let saved = (self.drm().get_crtc)(fd, self.crtc);
            if !saved.is_null() {
                self.saved = Some(*saved);
                (self.drm().free_crtc)(saved);
            }
            if (self.drm().create_blob)(fd, (&self.mode as *const drmModeModeInfo).cast(), std::mem::size_of::<drmModeModeInfo>(), &mut self.blob) != 0 {
                return Err(Error::errno("mode_blob"));
            }
        }
        Ok(())
    }

    pub fn info(&self) -> DisplayInfo {
        let name: Vec<u8> = self.mode.name.iter().take_while(|&&c| c != 0).map(|&c| c as u8).collect();
        DisplayInfo {
            target: "kms",
            drm_master: self.master,
            connector_id: self.connector,
            crtc_id: self.crtc,
            plane_id: self.plane,
            mode_name: String::from_utf8_lossy(&name).into_owned(),
            width: self.mode.hdisplay,
            height: self.mode.vdisplay,
            clock_khz: self.mode.clock,
            htotal: self.mode.htotal,
            vtotal: self.mode.vtotal,
            vrefresh: self.mode.vrefresh,
            period_ns: self.pacing.period_ns,
            vblanks_per_frame: self.pacing.divisor,
            saved_crtc_buffer: self.saved.map_or(0, |c| c.buffer_id),
            saved_crtc_mode_valid: self.saved.is_some_and(|c| c.mode_valid != 0),
            gl: self.gl.as_ref().map(|g| g.info.clone()).unwrap_or_default(),
        }
    }

    pub fn modeset_path(&self) -> &'static str {
        self.modeset_path
    }

    pub fn counts(&self) -> PresentCounts {
        PresentCounts {
            presented: self.tracker.presented_count(),
            stale_stamps: self.tracker.stale_stamp_count(),
            errors: self.tracker.error_count(),
        }
    }

    /// Start sampling/rendering the next frame as soon as this flip retires.
    /// The commit path separately enforces the 30 fps presentation cadence.
    pub fn next_wake_ns(&self) -> Option<u64> {
        self.last_vblank_ns.map(|v| self.pacing.next_wake(v))
    }

    unsafe fn fb_for(&mut self, bo: *mut gbm_bo) -> Result<u32, Error> {
        if let Some(&(_, fb)) = self.fbs.iter().find(|(b, _)| *b == bo as usize) {
            return Ok(fb);
        }
        let gbm = &self.libs.gbm;
        unsafe {
            let handles = [(gbm.bo_handle)(bo).u32_, 0, 0, 0];
            let pitches = [(gbm.bo_stride)(bo), 0, 0, 0];
            let offsets = [0u32; 4];
            let mut fb = 0;
            if (self.drm().add_fb2)(self.fd, (gbm.bo_width)(bo), (gbm.bo_height)(bo), (gbm.bo_format)(bo), handles.as_ptr(), pitches.as_ptr(), offsets.as_ptr(), &mut fb, 0) != 0 {
                return Err(Error::errno("add_fb2"));
            }
            self.fbs.push((bo as usize, fb));
            Ok(fb)
        }
    }

    unsafe fn commit(&self, fb: u32, token: u64, modeset: bool, blocking: bool) -> Result<(), Error> {
        let drm = self.drm();
        unsafe {
            let req = (drm.atomic_alloc)();
            if req.is_null() {
                return Err(Error::new("atomic_alloc", "out of memory"));
            }
            let p = &self.props;
            let (w, h) = (u64::from(self.mode.hdisplay), u64::from(self.mode.vdisplay));
            let mut items = vec![(self.plane, p.fb, u64::from(fb))];
            if modeset {
                items.extend([
                    (self.connector, p.conn_crtc, u64::from(self.crtc)),
                    (self.crtc, p.crtc_mode, u64::from(self.blob)),
                    (self.crtc, p.crtc_active, 1),
                    (self.plane, p.plane_crtc, u64::from(self.crtc)),
                    (self.plane, p.sx, 0),
                    (self.plane, p.sy, 0),
                    (self.plane, p.sw, w << 16),
                    (self.plane, p.sh, h << 16),
                    (self.plane, p.cx, 0),
                    (self.plane, p.cy, 0),
                    (self.plane, p.cw, w),
                    (self.plane, p.ch, h),
                ]);
            }
            let mut result = Ok(());
            for (obj, prop, value) in items {
                if (drm.atomic_add)(req, obj, prop, value) < 0 {
                    result = Err(Error::errno("atomic_add"));
                    break;
                }
            }
            let mut flags = DRM_MODE_PAGE_FLIP_EVENT;
            if !blocking {
                flags |= DRM_MODE_ATOMIC_NONBLOCK;
            }
            if modeset {
                flags |= DRM_MODE_ATOMIC_ALLOW_MODESET;
            }
            if result.is_ok() && (drm.atomic_commit)(self.fd, req, flags, token as usize as *mut c_void) != 0 {
                result = Err(Error::errno(if modeset { "atomic_modeset" } else { "atomic_flip" }));
            }
            (drm.atomic_free)(req);
            result
        }
    }

    /// Draws `verts`, commits the frame with a fresh token and waits (bounded)
    /// for that token's flip event.
    pub fn present(&mut self, verts: &[Vertex]) -> Result<FrameReport, Error> {
        self.present_inner(None, verts)
    }

    /// The S4 frame: the world pass, then the HUD `verts`, then as [`Self::present`].
    pub fn present_world(&mut self, world: &WorldDraw, verts: &[Vertex]) -> Result<FrameReport, Error> {
        self.present_inner(Some(world), verts)
    }

    /// Uploads the SPIKE-002 visual sidecar once (needs [`Self::open_world`]
    /// and, for the lights, [`Self::load_world`]).
    pub fn load_photo(&mut self, world: &gx_world::WorldAsset, visual: &gx_world::VisualAsset) -> Result<PhotoMemory, Error> {
        let gl = self.gl.as_mut().ok_or_else(|| Error::new("load_photo", "GL not initialized"))?;
        unsafe { gl.load_photo(&self.libs, world, visual) }
    }

    /// The SPIKE-002 frame: the photo pass, then the HUD `verts`.
    pub fn present_photo(&mut self, photo: &PhotoDraw, verts: &[Vertex]) -> Result<FrameReport, Error> {
        self.present_photo_inner(None, Some(photo), verts)
    }

    fn present_inner(&mut self, world: Option<&WorldDraw>, verts: &[Vertex]) -> Result<FrameReport, Error> {
        self.present_photo_inner(world, None, verts)
    }

    fn present_photo_inner(&mut self, world: Option<&WorldDraw>, photo: Option<&PhotoDraw>, verts: &[Vertex]) -> Result<FrameReport, Error> {
        if self.closed {
            return Err(Error::new("present", "display closed"));
        }
        self.frame += 1;
        let (display, surface) = (self.target.display, self.target.egl_surface);
        let egl_swap = self.libs.egl.swap_buffers;
        let mut gl = self.gl.take().expect("GL initialized");
        let drawn = unsafe {
            gl.frame(&self.libs, world, photo, verts, &mut |_| {}, &mut || (egl_swap)(display, surface) != 0)
        };
        self.gl = Some(gl);
        let (gpu, timing, draws) = drawn?;
        unsafe {
            let bo = (self.libs.gbm.lock_front)(self.target.surface);
            if bo.is_null() {
                return Err(Error::new("gbm_lock_front", "no front buffer after swap"));
            }
            let fb = match self.fb_for(bo) {
                Ok(fb) => fb,
                Err(e) => {
                    (self.libs.gbm.release)(self.target.surface, bo);
                    return Err(e);
                }
            };
            let token = self.next_token;
            self.next_token += 1;
            // Render from the completed flip onward, then hold a fast buffer
            // until just after the intermediate vblank. This preserves the
            // 30 fps cap without reducing the GPU's two-vblank frame budget
            // to one vblank (SPIKE-002's 20 fps pacing cliff).
            if let Some(last) = self.last_vblank_ns {
                sleep_until_monotonic_ns(self.pacing.earliest_commit(last));
            }
            let commit_ns = monotonic_ns();
            let pending = PendingPresent { token, framebuffer_id: fb, buffer_id: bo as usize as u64, snapshot_sequence: self.frame, submit_ns: commit_ns, commit_flags: 0 };
            if let Err(e) = self.tracker.submit(pending) {
                (self.libs.gbm.release)(self.target.surface, bo);
                return Err(Error::new("present_submit", e));
            }
            let modeset = !self.modeset_done;
            let mut committed = self.commit(fb, token, modeset, false);
            if modeset {
                self.modeset_path = "nonblocking";
                if committed.is_err() {
                    // Atomic commits are all-or-nothing: retry the identical
                    // modeset as a blocking commit (still with the flip event).
                    self.modeset_path = "blocking_fallback";
                    committed = self.commit(fb, token, true, true);
                }
            }
            if let Err(e) = committed {
                self.tracker.cancel_failed_commit(token);
                (self.libs.gbm.release)(self.target.surface, bo);
                return Err(e);
            }
            self.modeset_done = true;
            let timeout = if modeset { MODESET_TIMEOUT_NS } else { FLIP_TIMEOUT_NS };
            let (status, flip, foreign) = match self.wait_flip(token, commit_ns + timeout) {
                Ok(v) => v,
                Err(e) => {
                    // The commit may still complete: keep `bo` locked; teardown
                    // restores the CRTC before anything is released.
                    self.scanout_pending_leak(bo);
                    return Err(e);
                }
            };
            // The new buffer is on screen: the previous one may be reused.
            if let Some(old) = self.scanout.replace(bo) {
                (self.libs.gbm.release)(self.target.surface, old);
            }
            let fresh = matches!(status, EventStatus::Presented);
            let vblank_ns = if fresh {
                self.pacing.on_fresh_flip(flip.ns);
                flip.ns
            } else {
                self.pacing.vblank_after(commit_ns).unwrap_or(commit_ns)
            };
            self.last_vblank_ns = Some(vblank_ns);
            let interval_ns = self.intervals.on_flip(self.frame, fresh.then_some(flip.ns));
            Ok(FrameReport {
                token,
                commit_ns,
                flip_ns: fresh.then_some(flip.ns),
                vblank_ns,
                sequence: flip.sequence,
                status: status_name(status),
                interval_ns,
                gpu,
                timing,
                foreign_events: foreign,
                draws,
            })
        }
    }

    fn scanout_pending_leak(&mut self, _bo: *mut gbm_bo) {
        // Deliberately not released: the kernel may still scan it out.
    }

    /// Waits until the event for `token` arrives (or `deadline_ns`).
    /// Returns its tracker status, the event and the number of events for
    /// other tokens (errors, never retire anything).
    fn wait_flip(&mut self, token: u64, deadline_ns: u64) -> Result<(EventStatus, FlipEvent, u32), Error> {
        let mut ctx = drmEventContext {
            version: DRM_EVENT_CONTEXT_VERSION_PAGE_FLIP2,
            vblank_handler: None,
            page_flip_handler: None,
            page_flip_handler2: Some(on_page_flip),
            sequence_handler: None,
        };
        let mut foreign = 0;
        loop {
            // Judge every delivered event (a batch may hold more than one).
            let mut ours = None;
            for ev in take_events() {
                let c = self.tracker.on_event(ev.crtc, ev.token, ev.sequence, ev.ns);
                if c.status.presented() || (ev.token == token && self.tracker.retire_rejected(token).is_some()) {
                    ours = Some((c.status, ev));
                } else {
                    foreign += 1;
                }
            }
            if let Some((status, ev)) = ours {
                return Ok((status, ev, foreign));
            }
            let now = monotonic_ns();
            if now >= deadline_ns {
                return Err(Error::new("flip_timeout", format!("no page-flip event for token {token} by the deadline")));
            }
            let mut pfd = libc::pollfd { fd: self.fd, events: libc::POLLIN, revents: 0 };
            let ms = ((deadline_ns - now) / 1_000_000 + 1).min(i32::MAX as u64) as c_int;
            // SAFETY: one valid pollfd.
            let n = unsafe { libc::poll(&mut pfd, 1, ms) };
            if n < 0 {
                if std::io::Error::last_os_error().kind() == std::io::ErrorKind::Interrupted {
                    continue;
                }
                return Err(Error::errno("event_poll"));
            }
            // SAFETY: valid fd and context; the handler only records events.
            if n > 0 && unsafe { (self.drm().handle_event)(self.fd, &mut ctx) } != 0 {
                return Err(Error::errno("event_read"));
            }
        }
    }

    /// Restores the pre-modeset CRTC (as M0-001C) and releases everything.
    /// Returns the restore outcome for the log.
    pub fn shutdown(mut self) -> Result<String, Error> {
        self.teardown()
    }

    fn teardown(&mut self) -> Result<String, Error> {
        if self.closed {
            return Ok("already closed".into());
        }
        self.closed = true;
        // Let an in-flight flip land first (bounded) so nothing scanned out
        // is released under the display.
        if let Some(p) = self.tracker.pending() {
            let _ = self.wait_flip(p.token, monotonic_ns() + 100_000_000);
        }
        let mut outcome = Ok("no modeset; nothing to restore".to_owned());
        unsafe {
            if self.master && self.modeset_done {
                let mut conn = self.connector;
                let mut saved = self.saved;
                let r = match saved.as_mut() {
                    Some(s) if s.mode_valid != 0 && s.buffer_id != 0 => {
                        (self.drm().set_crtc)(self.fd, self.crtc, s.buffer_id, s.x, s.y, &mut conn, 1, &mut s.mode)
                    }
                    _ => (self.drm().set_crtc)(self.fd, self.crtc, 0, 0, 0, ptr::null_mut(), 0, ptr::null_mut()),
                };
                outcome = if r == 0 {
                    Ok(format!("set_crtc ok (saved buffer {})", self.saved.map_or(0, |s| s.buffer_id)))
                } else {
                    Err(Error::errno("restore_crtc"))
                };
            }
            if let Some(bo) = self.scanout.take() {
                if !self.target.surface.is_null() {
                    (self.libs.gbm.release)(self.target.surface, bo);
                }
            }
            if let Some(mut gl) = self.gl.take() {
                if self.target.current {
                    gl.destroy(&self.libs.gl);
                }
            }
            let fbs = std::mem::take(&mut self.fbs);
            for (_, fb) in fbs {
                (self.drm().rm_fb)(self.fd, fb);
            }
            self.target.destroy(&self.libs);
            if self.blob != 0 {
                (self.drm().destroy_blob)(self.fd, self.blob);
                self.blob = 0;
            }
            if self.master {
                (self.drm().drop_master)(self.fd);
                self.master = false;
            }
            if self.fd >= 0 {
                libc::close(self.fd);
                self.fd = -1;
            }
        }
        outcome
    }
}

impl Drop for KmsDisplay {
    fn drop(&mut self) {
        let _ = self.teardown();
    }
}

fn refresh_mhz(m: &drmModeModeInfo) -> Option<u64> {
    let denom = u64::from(m.htotal) * u64::from(m.vtotal);
    (denom > 0).then(|| u64::from(m.clock) * 1_000_000 / denom)
}

// --------------------------------------------------------------- offscreen --

/// The same frame on the render node: no DRM master, nothing on screen.
pub struct Offscreen {
    libs: Libs,
    fd: Fd,
    target: EglTarget,
    gl: Option<Gl>,
    front: Option<*mut gbm_bo>,
    frame: u64,
    closed: bool,
}

impl Offscreen {
    pub fn open(node: &str) -> Result<Self, Error> {
        Self::open_with(node, false)
    }

    /// [`Self::open`] with a 24-bit depth buffer, for the S4 world.
    pub fn open_world(node: &str) -> Result<Self, Error> {
        Self::open_with(node, true)
    }

    /// Uploads the static S4 geometry once (needs [`Self::open_world`]).
    pub fn load_world(&mut self, w: &WorldBuffers<'_>) -> Result<u64, Error> {
        let gl = self.gl.as_mut().ok_or_else(|| Error::new("load_world", "GL not initialized"))?;
        unsafe { gl.load_world(&self.libs, w) }
    }

    fn open_with(node: &str, depth: bool) -> Result<Self, Error> {
        let libs = Libs::load(false)?;
        let fd = open_rw(node)?;
        let mut s = Self { libs, fd, target: EglTarget::empty(), gl: None, front: None, frame: 0, closed: false };
        unsafe {
            let (depth, stencil) = s.target.create(&s.libs, fd, GBM_BO_USE_RENDERING, depth)?;
            s.gl = Some(Gl::new(&s.libs, depth, stencil)?);
        }
        Ok(s)
    }

    pub fn info(&self) -> DisplayInfo {
        DisplayInfo {
            target: "offscreen",
            width: W as u16,
            height: H as u16,
            gl: self.gl.as_ref().map(|g| g.info.clone()).unwrap_or_default(),
            ..Default::default()
        }
    }

    /// Renders one frame; `probes` are read back (before the swap) as RGB.
    pub fn present_probe(&mut self, verts: &[Vertex], probes: &[(u32, u32)]) -> Result<(FrameReport, Vec<[u8; 3]>), Error> {
        self.present_probe_inner(None, verts, probes)
    }

    /// The S4 frame offscreen, with pixel probes (read before the swap).
    pub fn present_world_probe(&mut self, world: &WorldDraw, verts: &[Vertex], probes: &[(u32, u32)]) -> Result<(FrameReport, Vec<[u8; 3]>), Error> {
        self.present_probe_inner(Some(world), verts, probes)
    }

    pub fn present_world(&mut self, world: &WorldDraw, verts: &[Vertex]) -> Result<FrameReport, Error> {
        self.present_probe_inner(Some(world), verts, &[]).map(|(r, _)| r)
    }

    /// Uploads the SPIKE-002 visual sidecar once (needs [`Self::open_world`]).
    pub fn load_photo(&mut self, world: &gx_world::WorldAsset, visual: &gx_world::VisualAsset) -> Result<PhotoMemory, Error> {
        let gl = self.gl.as_mut().ok_or_else(|| Error::new("load_photo", "GL not initialized"))?;
        unsafe { gl.load_photo(&self.libs, world, visual) }
    }

    /// The SPIKE-002 frame offscreen; with `capture`, the whole frame is read
    /// back (before the swap) as top-down RGB rows.
    pub fn present_photo(&mut self, photo: &PhotoDraw, verts: &[Vertex], capture: bool) -> Result<(FrameReport, Option<Vec<u8>>), Error> {
        let mut image = None;
        let (r, _) = self.present_frame(None, Some(photo), verts, &[], if capture { Some(&mut image) } else { None })?;
        Ok((r, image))
    }

    fn present_probe_inner(&mut self, world: Option<&WorldDraw>, verts: &[Vertex], probes: &[(u32, u32)]) -> Result<(FrameReport, Vec<[u8; 3]>), Error> {
        self.present_frame(world, None, verts, probes, None)
    }

    /// The S4 frame offscreen plus the whole frame as top-down RGB (SPIKE-002 control captures).
    pub fn present_world_capture(&mut self, world: &WorldDraw, verts: &[Vertex]) -> Result<(FrameReport, Option<Vec<u8>>), Error> {
        let mut image = None;
        let (r, _) = self.present_frame(Some(world), None, verts, &[], Some(&mut image))?;
        Ok((r, image))
    }

    fn present_frame(
        &mut self,
        world: Option<&WorldDraw>,
        photo: Option<&PhotoDraw>,
        verts: &[Vertex],
        probes: &[(u32, u32)],
        mut capture: Option<&mut Option<Vec<u8>>>,
    ) -> Result<(FrameReport, Vec<[u8; 3]>), Error> {
        self.frame += 1;
        let (display, surface) = (self.target.display, self.target.egl_surface);
        let egl_swap = self.libs.egl.swap_buffers;
        let mut out = Vec::with_capacity(probes.len());
        let mut gl = self.gl.take().expect("GL initialized");
        let drawn = unsafe {
            gl.frame(
                &self.libs,
                world,
                photo,
                verts,
                &mut |g| {
                    out.extend(probes.iter().map(|&(x, y)| g.pixel(x, y)));
                    if let Some(c) = capture.as_deref_mut() {
                        *c = Some(g.frame_rgb());
                    }
                },
                &mut || (egl_swap)(display, surface) != 0,
            )
        };
        self.gl = Some(gl);
        let (gpu, timing, draws) = drawn?;
        unsafe {
            // A GBM surface has a few buffers: lock the new front and give the
            // previous one back, or the next swap runs out.
            let bo = (self.libs.gbm.lock_front)(self.target.surface);
            if bo.is_null() {
                return Err(Error::new("gbm_lock_front", "no front buffer after swap"));
            }
            if let Some(old) = self.front.replace(bo) {
                (self.libs.gbm.release)(self.target.surface, old);
            }
        }
        let report = FrameReport {
            token: self.frame,
            commit_ns: timing.fence_ns,
            flip_ns: None,
            vblank_ns: timing.fence_ns,
            sequence: 0,
            status: "offscreen",
            interval_ns: None,
            gpu,
            timing,
            foreign_events: 0,
            draws,
        };
        Ok((report, out))
    }

    pub fn present(&mut self, verts: &[Vertex]) -> Result<FrameReport, Error> {
        self.present_probe(verts, &[]).map(|(r, _)| r)
    }

    fn teardown(&mut self) {
        if self.closed {
            return;
        }
        self.closed = true;
        unsafe {
            if let Some(bo) = self.front.take() {
                (self.libs.gbm.release)(self.target.surface, bo);
            }
            if let Some(mut gl) = self.gl.take() {
                if self.target.current {
                    gl.destroy(&self.libs.gl);
                }
            }
            self.target.destroy(&self.libs);
            if self.fd >= 0 {
                libc::close(self.fd);
                self.fd = -1;
            }
        }
    }
}

impl Drop for Offscreen {
    fn drop(&mut self) {
        self.teardown();
    }
}
