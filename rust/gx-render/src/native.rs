//! Hand-declared ABI for the dlopen'ed libdrm / libgbm / EGL / GLES subset.
//!
//! The pinned cross sysroot has no graphics development packages (as for the
//! M0-001C candidate), so only what S3 uses is declared. Layouts mirror the
//! M0-001C `src/m0/scanout/native_api.h` declarations, which are verified
//! against the hash-pinned Debian trixie arm64 headers that match the
//! board's runtime (libdrm 2.4.124-2, libgbm 25.2.6-1~bpo13+1, libglvnd
//! 1.7.0-1+b2; `tools/fetch_m0_scanout_headers.py`).
//!
//! Every size, field offset and constant below is listed in
//! `abi_layout.txt`. Two checks consume that one table:
//! * `tests::layouts_and_constants_match_the_table` (host and board):
//!   the Rust declarations equal the table;
//! * `tools/check_m1_render_abi.py`: compiles the same table as C
//!   `_Static_assert`s against the pinned headers with the pinned Zig
//!   (aarch64-linux-gnu.2.34). Together: Rust == table == headers.
//!
//! Compiled on every platform (plain `repr(C)` data, no FFI calls) so the
//! layout check also runs on the host.

#![allow(non_camel_case_types, non_snake_case, non_upper_case_globals, dead_code)]

use std::os::raw::{c_char, c_int, c_uint, c_void};

// ------------------------------------------------------------------ libdrm --

pub const DRM_CLIENT_CAP_UNIVERSAL_PLANES: u64 = 2;
pub const DRM_CLIENT_CAP_ATOMIC: u64 = 3;
pub const DRM_CAP_TIMESTAMP_MONOTONIC: u64 = 6;
pub const DRM_MODE_OBJECT_CRTC: u32 = 0xcccc_cccc;
pub const DRM_MODE_OBJECT_CONNECTOR: u32 = 0xc0c0_c0c0;
pub const DRM_MODE_OBJECT_PLANE: u32 = 0xeeee_eeee;
pub const DRM_MODE_PAGE_FLIP_EVENT: u32 = 0x01;
pub const DRM_MODE_ATOMIC_NONBLOCK: u32 = 0x0200;
pub const DRM_MODE_ATOMIC_ALLOW_MODESET: u32 = 0x0400;
pub const DRM_PLANE_TYPE_PRIMARY: u64 = 1;
pub const DRM_MODE_CONNECTED: c_int = 1;
pub const DRM_MODE_CONNECTOR_DisplayPort: u32 = 10;
pub const DRM_EVENT_CONTEXT_VERSION_PAGE_FLIP2: c_int = 3;
pub const DRM_FORMAT_XRGB8888: u32 = u32::from_le_bytes(*b"XR24");

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct drmModeModeInfo {
    pub clock: u32,
    pub hdisplay: u16,
    pub hsync_start: u16,
    pub hsync_end: u16,
    pub htotal: u16,
    pub hskew: u16,
    pub vdisplay: u16,
    pub vsync_start: u16,
    pub vsync_end: u16,
    pub vtotal: u16,
    pub vscan: u16,
    pub vrefresh: u32,
    pub flags: u32,
    pub type_: u32,
    pub name: [c_char; 32],
}

#[repr(C)]
pub struct drmModeRes {
    pub count_fbs: c_int,
    pub fbs: *mut u32,
    pub count_crtcs: c_int,
    pub crtcs: *mut u32,
    pub count_connectors: c_int,
    pub connectors: *mut u32,
    pub count_encoders: c_int,
    pub encoders: *mut u32,
    pub min_width: u32,
    pub max_width: u32,
    pub min_height: u32,
    pub max_height: u32,
}

#[repr(C)]
pub struct drmModeConnector {
    pub connector_id: u32,
    pub encoder_id: u32,
    pub connector_type: u32,
    pub connector_type_id: u32,
    pub connection: c_int,
    pub mmWidth: u32,
    pub mmHeight: u32,
    pub subpixel: c_int,
    pub count_modes: c_int,
    pub modes: *mut drmModeModeInfo,
    pub count_props: c_int,
    pub props: *mut u32,
    pub prop_values: *mut u64,
    pub count_encoders: c_int,
    pub encoders: *mut u32,
}

#[repr(C)]
pub struct drmModeEncoder {
    pub encoder_id: u32,
    pub encoder_type: u32,
    pub crtc_id: u32,
    pub possible_crtcs: u32,
    pub possible_clones: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct drmModeCrtc {
    pub crtc_id: u32,
    pub buffer_id: u32,
    pub x: u32,
    pub y: u32,
    pub width: u32,
    pub height: u32,
    pub mode_valid: c_int,
    pub mode: drmModeModeInfo,
    pub gamma_size: c_int,
}

#[repr(C)]
pub struct drmModePlaneRes {
    pub count_planes: u32,
    pub planes: *mut u32,
}

#[repr(C)]
pub struct drmModePlane {
    pub count_formats: u32,
    pub formats: *mut u32,
    pub plane_id: u32,
    pub crtc_id: u32,
    pub fb_id: u32,
    pub crtc_x: u32,
    pub crtc_y: u32,
    pub x: u32,
    pub y: u32,
    pub possible_crtcs: u32,
    pub gamma_size: u32,
}

#[repr(C)]
pub struct drmModeObjectProperties {
    pub count_props: u32,
    pub props: *mut u32,
    pub prop_values: *mut u64,
}

#[repr(C)]
pub struct drmModePropertyRes {
    pub prop_id: u32,
    pub flags: u32,
    pub name: [c_char; 32],
    pub count_values: c_int,
    pub values: *mut u64,
    pub count_enums: c_int,
    pub enums: *mut c_void,
    pub count_blobs: c_int,
    pub blob_ids: *mut u32,
}

pub enum drmModeAtomicReq {}

pub type PageFlipHandler2 =
    unsafe extern "C" fn(c_int, c_uint, c_uint, c_uint, c_uint, *mut c_void);

#[repr(C)]
pub struct drmEventContext {
    pub version: c_int,
    pub vblank_handler: Option<unsafe extern "C" fn(c_int, c_uint, c_uint, c_uint, *mut c_void)>,
    pub page_flip_handler: Option<unsafe extern "C" fn(c_int, c_uint, c_uint, c_uint, *mut c_void)>,
    pub page_flip_handler2: Option<PageFlipHandler2>,
    pub sequence_handler: Option<unsafe extern "C" fn(c_int, u64, u64, u64)>,
}

// ------------------------------------------------------------------ libgbm --

pub enum gbm_device {}
pub enum gbm_surface {}
pub enum gbm_bo {}

#[repr(C)]
#[derive(Clone, Copy)]
pub union gbm_bo_handle {
    pub ptr: *mut c_void,
    pub s32: i32,
    pub u32_: u32,
    pub s64: i64,
    pub u64_: u64,
}

pub const GBM_BO_USE_SCANOUT: u32 = 1 << 0;
pub const GBM_BO_USE_RENDERING: u32 = 1 << 2;

// --------------------------------------------------------------------- EGL --

pub type EGLBoolean = c_uint;
pub type EGLenum = c_uint;
pub type EGLint = i32;
pub type EGLAttrib = isize;
pub type EGLDisplay = *mut c_void;
pub type EGLConfig = *mut c_void;
pub type EGLContext = *mut c_void;
pub type EGLSurface = *mut c_void;

pub const EGL_NONE: EGLint = 0x3038;
pub const EGL_OPENGL_ES_API: EGLenum = 0x30A0;
pub const EGL_SURFACE_TYPE: EGLint = 0x3033;
pub const EGL_WINDOW_BIT: EGLint = 0x0004;
pub const EGL_RENDERABLE_TYPE: EGLint = 0x3040;
pub const EGL_OPENGL_ES3_BIT_KHR: EGLint = 0x0040;
pub const EGL_RED_SIZE: EGLint = 0x3024;
pub const EGL_GREEN_SIZE: EGLint = 0x3023;
pub const EGL_BLUE_SIZE: EGLint = 0x3022;
pub const EGL_ALPHA_SIZE: EGLint = 0x3021;
pub const EGL_DEPTH_SIZE: EGLint = 0x3025;
pub const EGL_STENCIL_SIZE: EGLint = 0x3026;
pub const EGL_NATIVE_VISUAL_ID: EGLint = 0x302E;
pub const EGL_CONTEXT_CLIENT_VERSION: EGLint = 0x3098;
pub const EGL_PLATFORM_GBM_KHR: EGLenum = 0x31D7;

// -------------------------------------------------------------------- GLES --

pub type GLenum = c_uint;
pub type GLbitfield = c_uint;
pub type GLint = c_int;
pub type GLuint = c_uint;
pub type GLsizei = c_int;
pub type GLsizeiptr = isize;
pub type GLintptr = isize;
pub type GLuint64 = u64;
pub type GLsync = *mut c_void;

pub const GL_NO_ERROR: GLenum = 0;
pub const GL_RENDERER: GLenum = 0x1F01;
pub const GL_VERSION: GLenum = 0x1F02;
pub const GL_EXTENSIONS: GLenum = 0x1F03;
pub const GL_VERTEX_SHADER: GLenum = 0x8B31;
pub const GL_FRAGMENT_SHADER: GLenum = 0x8B30;
pub const GL_COMPILE_STATUS: GLenum = 0x8B81;
pub const GL_LINK_STATUS: GLenum = 0x8B82;
pub const GL_TRIANGLES: GLenum = 0x0004;
pub const GL_POINTS: GLenum = 0x0000;
pub const GL_LESS: GLenum = 0x0201;
pub const GL_STATIC_DRAW: GLenum = 0x88E4;
pub const GL_FLOAT: GLenum = 0x1406;
pub const GL_FALSE: u8 = 0;
pub const GL_ARRAY_BUFFER: GLenum = 0x8892;
pub const GL_STREAM_DRAW: GLenum = 0x88E0;
pub const GL_COLOR_BUFFER_BIT: GLbitfield = 0x4000;
pub const GL_DEPTH_BUFFER_BIT: GLbitfield = 0x0100;
pub const GL_STENCIL_BUFFER_BIT: GLbitfield = 0x0400;
pub const GL_FRAMEBUFFER: GLenum = 0x8D40;
/// Default-framebuffer attachment names for glInvalidateFramebuffer.
pub const GL_DEPTH: GLenum = 0x1801;
pub const GL_STENCIL: GLenum = 0x1802;
pub const GL_BLEND: GLenum = 0x0BE2;
pub const GL_DEPTH_TEST: GLenum = 0x0B71;
pub const GL_DITHER: GLenum = 0x0BD0;
pub const GL_SCISSOR_TEST: GLenum = 0x0C11;
pub const GL_STENCIL_TEST: GLenum = 0x0B90;
pub const GL_CULL_FACE: GLenum = 0x0B44;
pub const GL_RGBA: GLenum = 0x1908;
pub const GL_UNSIGNED_BYTE: GLenum = 0x1401;
// SPIKE-002 photo pass (textures, indexed draws).
pub const GL_TEXTURE_2D: GLenum = 0x0DE1;
pub const GL_TEXTURE0: GLenum = 0x84C0;
pub const GL_TEXTURE_MIN_FILTER: GLenum = 0x2801;
pub const GL_TEXTURE_MAG_FILTER: GLenum = 0x2800;
pub const GL_TEXTURE_WRAP_S: GLenum = 0x2802;
pub const GL_TEXTURE_WRAP_T: GLenum = 0x2803;
pub const GL_TEXTURE_MAX_ANISOTROPY_EXT: GLenum = 0x84FE;
pub const GL_LINEAR: GLint = 0x2601;
pub const GL_LINEAR_MIPMAP_LINEAR: GLint = 0x2703;
pub const GL_CLAMP_TO_EDGE: GLint = 0x812F;
pub const GL_REPEAT: GLint = 0x2901;
pub const GL_ELEMENT_ARRAY_BUFFER: GLenum = 0x8893;
pub const GL_UNSIGNED_INT: GLenum = 0x1405;
pub const GL_LEQUAL: GLenum = 0x0203;
pub const GL_MAX_TEXTURE_SIZE: GLenum = 0x0D33;
pub const GL_SYNC_GPU_COMMANDS_COMPLETE: GLenum = 0x9117;
pub const GL_SYNC_FLUSH_COMMANDS_BIT: GLbitfield = 0x0000_0001;
pub const GL_ALREADY_SIGNALED: GLenum = 0x911A;
pub const GL_CONDITION_SATISFIED: GLenum = 0x911C;
// GL_EXT_disjoint_timer_query (gl2ext.h).
pub const GL_QUERY_RESULT_EXT: GLenum = 0x8866;
pub const GL_QUERY_RESULT_AVAILABLE_EXT: GLenum = 0x8867;
pub const GL_TIME_ELAPSED_EXT: GLenum = 0x88BF;
pub const GL_GPU_DISJOINT_EXT: GLenum = 0x8FBB;

/// Renderer string classification, as the M0-001C candidate
/// (`scanout::classify_renderer`): software rasterizers are rejected,
/// Adreno/freedreno/"FD<digit>" accepted, anything else unknown.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RendererClass {
    Adreno,
    Software,
    Unknown,
}

pub fn classify_renderer(renderer: &str) -> RendererClass {
    let r = renderer.to_ascii_lowercase();
    if ["llvmpipe", "softpipe", "software rasterizer", "swrast", "lavapipe"].iter().any(|t| r.contains(t)) {
        return RendererClass::Software;
    }
    if r.contains("adreno") || r.contains("freedreno") {
        return RendererClass::Adreno;
    }
    let b = r.as_bytes();
    if b.windows(3).any(|w| w[0] == b'f' && w[1] == b'd' && w[2].is_ascii_digit()) {
        return RendererClass::Adreno;
    }
    RendererClass::Unknown
}

/// `name` is a space-separated token of `list` (GL/EGL extension strings).
pub fn has_token(list: &str, name: &str) -> bool {
    list.split(' ').any(|t| t == name)
}

/// The table both ABI checks read.
pub const ABI_LAYOUT: &str = include_str!("abi_layout.txt");

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;
    use std::mem::{offset_of, size_of};

    macro_rules! entries {
        ($m:ident; $( $c:literal => $t:ty { $( $cf:literal : $rf:ident ),* } )*) => {$(
            $m.insert(format!("size {}", $c), size_of::<$t>() as u64);
            $( $m.insert(format!("offset {}.{}", $c, $cf), offset_of!($t, $rf) as u64); )*
        )*};
    }

    fn rust_side() -> BTreeMap<String, u64> {
        let mut m = BTreeMap::new();
        entries!(m;
            "drmModeModeInfo" => drmModeModeInfo { "clock": clock, "hdisplay": hdisplay, "htotal": htotal,
                "vdisplay": vdisplay, "vtotal": vtotal, "vrefresh": vrefresh, "flags": flags, "type": type_, "name": name }
            "drmModeRes" => drmModeRes { "count_fbs": count_fbs, "fbs": fbs, "count_crtcs": count_crtcs, "crtcs": crtcs,
                "count_connectors": count_connectors, "connectors": connectors, "count_encoders": count_encoders,
                "encoders": encoders, "min_width": min_width, "max_height": max_height }
            "drmModeConnector" => drmModeConnector { "connector_id": connector_id, "encoder_id": encoder_id,
                "connector_type": connector_type, "connector_type_id": connector_type_id, "connection": connection,
                "mmWidth": mmWidth, "mmHeight": mmHeight, "subpixel": subpixel, "count_modes": count_modes,
                "modes": modes, "count_props": count_props, "props": props, "prop_values": prop_values,
                "count_encoders": count_encoders, "encoders": encoders }
            "drmModeEncoder" => drmModeEncoder { "encoder_id": encoder_id, "encoder_type": encoder_type,
                "crtc_id": crtc_id, "possible_crtcs": possible_crtcs, "possible_clones": possible_clones }
            "drmModeCrtc" => drmModeCrtc { "crtc_id": crtc_id, "buffer_id": buffer_id, "x": x, "y": y,
                "width": width, "height": height, "mode_valid": mode_valid, "mode": mode, "gamma_size": gamma_size }
            "drmModePlaneRes" => drmModePlaneRes { "count_planes": count_planes, "planes": planes }
            "drmModePlane" => drmModePlane { "count_formats": count_formats, "formats": formats, "plane_id": plane_id,
                "crtc_id": crtc_id, "fb_id": fb_id, "crtc_x": crtc_x, "crtc_y": crtc_y, "x": x, "y": y,
                "possible_crtcs": possible_crtcs, "gamma_size": gamma_size }
            "drmModeObjectProperties" => drmModeObjectProperties { "count_props": count_props, "props": props,
                "prop_values": prop_values }
            "drmModePropertyRes" => drmModePropertyRes { "prop_id": prop_id, "flags": flags, "name": name,
                "count_values": count_values, "values": values, "count_enums": count_enums, "enums": enums,
                "count_blobs": count_blobs, "blob_ids": blob_ids }
            "drmEventContext" => drmEventContext { "version": version, "vblank_handler": vblank_handler,
                "page_flip_handler": page_flip_handler, "page_flip_handler2": page_flip_handler2,
                "sequence_handler": sequence_handler }
            "union gbm_bo_handle" => gbm_bo_handle { }
        );
        for (name, v) in [
            ("DRM_CLIENT_CAP_UNIVERSAL_PLANES", DRM_CLIENT_CAP_UNIVERSAL_PLANES),
            ("DRM_CLIENT_CAP_ATOMIC", DRM_CLIENT_CAP_ATOMIC),
            ("DRM_CAP_TIMESTAMP_MONOTONIC", DRM_CAP_TIMESTAMP_MONOTONIC),
            ("DRM_MODE_OBJECT_CRTC", DRM_MODE_OBJECT_CRTC.into()),
            ("DRM_MODE_OBJECT_CONNECTOR", DRM_MODE_OBJECT_CONNECTOR.into()),
            ("DRM_MODE_OBJECT_PLANE", DRM_MODE_OBJECT_PLANE.into()),
            ("DRM_MODE_PAGE_FLIP_EVENT", DRM_MODE_PAGE_FLIP_EVENT.into()),
            ("DRM_MODE_ATOMIC_NONBLOCK", DRM_MODE_ATOMIC_NONBLOCK.into()),
            ("DRM_MODE_ATOMIC_ALLOW_MODESET", DRM_MODE_ATOMIC_ALLOW_MODESET.into()),
            ("DRM_PLANE_TYPE_PRIMARY", DRM_PLANE_TYPE_PRIMARY),
            ("DRM_MODE_CONNECTED", DRM_MODE_CONNECTED as u64),
            ("DRM_MODE_CONNECTOR_DisplayPort", DRM_MODE_CONNECTOR_DisplayPort.into()),
            ("DRM_EVENT_CONTEXT_VERSION_PAGE_FLIP2", DRM_EVENT_CONTEXT_VERSION_PAGE_FLIP2 as u64),
            ("DRM_FORMAT_XRGB8888", DRM_FORMAT_XRGB8888.into()),
            ("GBM_BO_USE_SCANOUT", GBM_BO_USE_SCANOUT.into()),
            ("GBM_BO_USE_RENDERING", GBM_BO_USE_RENDERING.into()),
            ("EGL_NONE", EGL_NONE as u64),
            ("EGL_OPENGL_ES_API", EGL_OPENGL_ES_API.into()),
            ("EGL_SURFACE_TYPE", EGL_SURFACE_TYPE as u64),
            ("EGL_WINDOW_BIT", EGL_WINDOW_BIT as u64),
            ("EGL_RENDERABLE_TYPE", EGL_RENDERABLE_TYPE as u64),
            ("EGL_OPENGL_ES3_BIT_KHR", EGL_OPENGL_ES3_BIT_KHR as u64),
            ("EGL_RED_SIZE", EGL_RED_SIZE as u64),
            ("EGL_GREEN_SIZE", EGL_GREEN_SIZE as u64),
            ("EGL_BLUE_SIZE", EGL_BLUE_SIZE as u64),
            ("EGL_ALPHA_SIZE", EGL_ALPHA_SIZE as u64),
            ("EGL_DEPTH_SIZE", EGL_DEPTH_SIZE as u64),
            ("EGL_STENCIL_SIZE", EGL_STENCIL_SIZE as u64),
            ("EGL_NATIVE_VISUAL_ID", EGL_NATIVE_VISUAL_ID as u64),
            ("EGL_CONTEXT_CLIENT_VERSION", EGL_CONTEXT_CLIENT_VERSION as u64),
            ("EGL_PLATFORM_GBM_KHR", EGL_PLATFORM_GBM_KHR.into()),
            ("GL_NO_ERROR", GL_NO_ERROR.into()),
            ("GL_RENDERER", GL_RENDERER.into()),
            ("GL_VERSION", GL_VERSION.into()),
            ("GL_EXTENSIONS", GL_EXTENSIONS.into()),
            ("GL_VERTEX_SHADER", GL_VERTEX_SHADER.into()),
            ("GL_FRAGMENT_SHADER", GL_FRAGMENT_SHADER.into()),
            ("GL_COMPILE_STATUS", GL_COMPILE_STATUS.into()),
            ("GL_LINK_STATUS", GL_LINK_STATUS.into()),
            ("GL_TRIANGLES", GL_TRIANGLES.into()),
            ("GL_POINTS", GL_POINTS.into()),
            ("GL_LESS", GL_LESS.into()),
            ("GL_STATIC_DRAW", GL_STATIC_DRAW.into()),
            ("GL_FLOAT", GL_FLOAT.into()),
            ("GL_FALSE", GL_FALSE.into()),
            ("GL_ARRAY_BUFFER", GL_ARRAY_BUFFER.into()),
            ("GL_STREAM_DRAW", GL_STREAM_DRAW.into()),
            ("GL_COLOR_BUFFER_BIT", GL_COLOR_BUFFER_BIT.into()),
            ("GL_DEPTH_BUFFER_BIT", GL_DEPTH_BUFFER_BIT.into()),
            ("GL_STENCIL_BUFFER_BIT", GL_STENCIL_BUFFER_BIT.into()),
            ("GL_FRAMEBUFFER", GL_FRAMEBUFFER.into()),
            ("GL_DEPTH", GL_DEPTH.into()),
            ("GL_STENCIL", GL_STENCIL.into()),
            ("GL_BLEND", GL_BLEND.into()),
            ("GL_DEPTH_TEST", GL_DEPTH_TEST.into()),
            ("GL_DITHER", GL_DITHER.into()),
            ("GL_SCISSOR_TEST", GL_SCISSOR_TEST.into()),
            ("GL_STENCIL_TEST", GL_STENCIL_TEST.into()),
            ("GL_CULL_FACE", GL_CULL_FACE.into()),
            ("GL_RGBA", GL_RGBA.into()),
            ("GL_UNSIGNED_BYTE", GL_UNSIGNED_BYTE.into()),
            ("GL_SYNC_GPU_COMMANDS_COMPLETE", GL_SYNC_GPU_COMMANDS_COMPLETE.into()),
            ("GL_SYNC_FLUSH_COMMANDS_BIT", GL_SYNC_FLUSH_COMMANDS_BIT.into()),
            ("GL_ALREADY_SIGNALED", GL_ALREADY_SIGNALED.into()),
            ("GL_CONDITION_SATISFIED", GL_CONDITION_SATISFIED.into()),
            ("GL_QUERY_RESULT_EXT", GL_QUERY_RESULT_EXT.into()),
            ("GL_QUERY_RESULT_AVAILABLE_EXT", GL_QUERY_RESULT_AVAILABLE_EXT.into()),
            ("GL_TIME_ELAPSED_EXT", GL_TIME_ELAPSED_EXT.into()),
            ("GL_GPU_DISJOINT_EXT", GL_GPU_DISJOINT_EXT.into()),
        ] {
            m.insert(format!("const {name}"), v);
        }
        m
    }

    fn table() -> BTreeMap<String, u64> {
        let mut m = BTreeMap::new();
        for line in ABI_LAYOUT.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let (key, value) = line.rsplit_once(' ').expect("KEY VALUE");
            let v = match value.strip_prefix("0x") {
                Some(h) => u64::from_str_radix(h, 16).unwrap(),
                None => value.parse().unwrap(),
            };
            assert!(m.insert(key.to_owned(), v).is_none(), "duplicate {key}");
        }
        m
    }

    #[test]
    fn layouts_and_constants_match_the_table() {
        let (rust, table) = (rust_side(), table());
        let only_rust: Vec<_> = rust.keys().filter(|k| !table.contains_key(*k)).collect();
        let only_table: Vec<_> = table.keys().filter(|k| !rust.contains_key(*k)).collect();
        assert!(only_rust.is_empty() && only_table.is_empty(), "rust-only {only_rust:?}, table-only {only_table:?}");
        for (k, v) in &rust {
            assert_eq!(table[k], *v, "{k}");
        }
        // Spot values that M0-001C native_api.h asserts.
        assert_eq!(table["size drmModeModeInfo"], 68);
        assert_eq!(table["size drmEventContext"], 40);
    }

    /// Prints the Rust side in table format (for extending the table; the
    /// C check then confirms every value against the pinned headers).
    #[test]
    #[ignore]
    fn print_rust_side() {
        for (k, v) in rust_side() {
            println!("{k} {v}");
        }
    }

    #[test]
    fn renderer_classes_and_tokens() {
        assert_eq!(classify_renderer("FD702"), RendererClass::Adreno);
        assert_eq!(classify_renderer("Adreno (TM) 702"), RendererClass::Adreno);
        assert_eq!(classify_renderer("llvmpipe (LLVM 19.1.7, 128 bits)"), RendererClass::Software);
        assert_eq!(classify_renderer("Mali-G52"), RendererClass::Unknown);
        assert!(has_token("GL_A GL_EXT_disjoint_timer_query GL_B", "GL_EXT_disjoint_timer_query"));
        assert!(!has_token("GL_EXT_disjoint_timer_query2", "GL_EXT_disjoint_timer_query"));
    }
}
