// Platform link glue (M1-001). Links the C shim libgx_fdm.so dynamically.
//
// Two modes:
// - default: `-l gx_fdm` with GX_FDM_LIB_DIR as search path (set by
//   rust/.cargo/config.toml to ../build/m1-fdm/cxx/out). Works for native
//   host builds and Linux hosts.
// - GX_FDM_LINK_BY_PATH=1 (set by tools/build_m1_fdm.ps1 for the board
//   cross-build): emit nothing here; the build script adds the library as a
//   positional linker input instead (`--config` rustflags `-C link-arg=<path>`).
//   Reason: Zig 0.17.0 on a Windows host turns a found `-lgx_fdm` into
//   `-l:C:\...\libgx_fdm.so`, which ld.lld rejects ("unable to find library").
//   A positional path links fine. See docs/platform/m1-001-build.md.
//
// At run time the binary finds the library through RUNPATH $ORIGIN (same
// directory), see rust/.cargo/config.toml.
fn main() {
    println!("cargo:rerun-if-env-changed=GX_FDM_LIB_DIR");
    println!("cargo:rerun-if-env-changed=GX_FDM_LINK_BY_PATH");
    let by_path = std::env::var("GX_FDM_LINK_BY_PATH")
        .map(|v| v == "1")
        .unwrap_or(false);
    if let Ok(dir) = std::env::var("GX_FDM_LIB_DIR") {
        println!("cargo:rerun-if-changed={dir}/libgx_fdm.so");
        if !by_path {
            println!("cargo:rustc-link-search=native={dir}");
        }
    }
    if !by_path {
        println!("cargo:rustc-link-lib=dylib=gx_fdm");
    }
}
