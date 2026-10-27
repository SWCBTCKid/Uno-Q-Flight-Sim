//! M1's Rust-owned rendering and direct-scanout boundary.
//!
//! Platform FFI and GLES live behind this crate. Pure policy such as commit
//! attribution, pacing and the scene itself stays host-testable so failures
//! cannot silently retire the wrong GBM buffer or draw the wrong warning.

pub mod aircraft;
pub mod camera;
pub mod frame;
pub mod math;
pub mod native;
pub mod pacing;
pub mod panel;
pub mod photo;
pub mod present;
pub mod reference;
pub mod scene;
pub mod world;

#[cfg(target_os = "linux")]
pub mod linux;
