# M1-001 (ARCH-002): toolchain for third-party C++ built with its own
# settings -- JSBSim and the gx_fdm C shim. Exceptions and RTTI stay ON
# (compiler defaults) and the code links the board's C++ runtime,
# libstdc++.so.6 (Debian 13, GCC 14.2.0-19), instead of Zig's bundled libc++.
#
# Why not Zig's libc++: Zig links it statically with hidden visibility into
# each shared object, so libJSBSim.so and libgx_fdm.so would each carry a
# private runtime and a typed catch in the shim misses exceptions thrown in
# JSBSim (measured on the board, docs/platform/m1-001-build.md).
#
# The M0 toolchain file (uno-q-aarch64-linux-gnu.cmake, -fno-exceptions
# -fno-rtti) is unchanged and still builds every M0 target.
#
# Inputs: UNO_Q_ZIG (pinned Zig 0.17.0 zig.exe) and GX_CXXRT_ROOT
# (third_party/m1/gcc14-runtime from tools/fetch_m1_fdm_deps.py).
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES UNO_Q_ZIG GX_CXXRT_ROOT)
if(NOT DEFINED UNO_Q_ZIG OR NOT EXISTS "${UNO_Q_ZIG}")
  message(FATAL_ERROR "Set UNO_Q_ZIG to the verified Zig 0.17.0 zig.exe")
endif()
if(NOT DEFINED GX_CXXRT_ROOT OR NOT EXISTS "${GX_CXXRT_ROOT}/include/aarch64-linux-gnu/c++/14/bits/c++config.h")
  message(FATAL_ERROR "Set GX_CXXRT_ROOT to third_party/m1/gcc14-runtime (run tools/fetch_m1_fdm_deps.py)")
endif()
set(CMAKE_C_COMPILER "${UNO_Q_ZIG}" "cc")
set(CMAKE_CXX_COMPILER "${UNO_Q_ZIG}" "c++")
set(CMAKE_C_FLAGS_INIT "-target aarch64-linux-gnu.2.34")
# -I, not -isystem: Zig passes its glibc headers as -isystem, and libstdc++'s
# <cstdlib> needs `#include_next <stdlib.h>` to find them *after* its own dir.
set(CMAKE_CXX_FLAGS_INIT "-target aarch64-linux-gnu.2.34 -nostdinc++ -I${GX_CXXRT_ROOT}/include/c++/14 -I${GX_CXXRT_ROOT}/include/aarch64-linux-gnu/c++/14")
# --allow-shlib-undefined: the board's libgcc_s.so.1 itself needs
# _dl_find_object@GLIBC_2.35, which Zig's glibc 2.34 link stubs lack. That is
# the board's own library on the board's glibc 2.41; our objects stay <= 2.34
# (checked by tools/elf_m1_report.py --max-glibc 2.34).
# --strip-debug: Zig emits DWARF by default (27 MB libJSBSim); .symtab stays for perf.
set(_gx_link "-target aarch64-linux-gnu.2.34 -nostdlib++ -Wl,--allow-shlib-undefined -Wl,--strip-debug")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${_gx_link}")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "${_gx_link}")
set(CMAKE_MODULE_LINKER_FLAGS_INIT "${_gx_link}")
# DT_NEEDED libstdc++.so.6 and libgcc_s.so.1 (the unwinder), resolved on the
# board from /usr/lib/aarch64-linux-gnu; linked by path, so nothing on the
# host is searched.
set(CMAKE_CXX_STANDARD_LIBRARIES "\"${GX_CXXRT_ROOT}/lib/libstdc++.so.6\" \"${GX_CXXRT_ROOT}/lib/libgcc_s.so.1\"")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE NEVER)
