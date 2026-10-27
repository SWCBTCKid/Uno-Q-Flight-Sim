# M1-001 C++ side (ARCH-002): JSBSim v1.3.2 as a shared library plus the
# gx_fdm C shim, built with their own settings (exceptions/RTTI on, linked to
# the board's libstdc++). A separate CMake project from the repository root so
# the M0 targets and their -fno-exceptions/-fno-rtti flags stay untouched.
#
# Configure with cmake/toolchains/uno-q-aarch64-linux-gnu-libstdcxx.cmake and
# -DGX_JSBSIM_SOURCE_DIR=<third_party/m1/jsbsim-1.3.2>. Driven by
# tools/build_m1_fdm.ps1.
#
# Outputs (all in ${CMAKE_BINARY_DIR}/out, RUNPATH $ORIGIN so they find each
# other in one deployed directory):
#   libJSBSim.so.1  (soname; libJSBSim.so -> link name)
#   libgx_fdm.so    (the C shim; the only library Rust links)
#   JSBSim          (JSBSim's standalone executable, measure 2 cross-check)
cmake_minimum_required(VERSION 3.20)
project(global_express_m1_fdm LANGUAGES C CXX)

if(NOT DEFINED GX_JSBSIM_SOURCE_DIR OR NOT EXISTS "${GX_JSBSIM_SOURCE_DIR}/src/FGFDMExec.h")
  message(FATAL_ERROR "Set GX_JSBSIM_SOURCE_DIR to the fetched JSBSim v1.3.2 tree")
endif()
get_filename_component(GX_REPO "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/out")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/out")
# No host build-tree paths in the binaries: use the install RPATH at build time.
set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)
set(CMAKE_INSTALL_RPATH "$ORIGIN")

# JSBSim's own options: shared library, no docs/Python/tests.
set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)
set(BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
set(SYSTEM_EXPAT OFF CACHE BOOL "" FORCE)
set(CMAKE_DISABLE_FIND_PACKAGE_CxxTest ON)
set(CMAKE_DISABLE_FIND_PACKAGE_PkgConfig ON)
add_subdirectory("${GX_JSBSIM_SOURCE_DIR}" jsbsim EXCLUDE_FROM_ALL)
# JSBSim stamps __DATE__/__TIME__ into its version string; Zig makes that an
# error (-Werror=date-time). Allow it for that one file; the build script sets
# SOURCE_DATE_EPOCH so the stamp is the pinned release date, not build time.
set_source_files_properties("${GX_JSBSIM_SOURCE_DIR}/src/FGJSBBase.cpp"
  DIRECTORY "${GX_JSBSIM_SOURCE_DIR}/src" PROPERTIES COMPILE_OPTIONS "-Wno-date-time")

# The shim: Flight Dynamics' real files when present, else the temporary
# Platform stub. Same API (docs/tasks/M1-001.md), same output name.
if(EXISTS "${GX_REPO}/src/fdm/shim/gx_fdm.cpp" AND EXISTS "${GX_REPO}/src/fdm/shim/gx_fdm.h")
  set(GX_FDM_SHIM_DIR "${GX_REPO}/src/fdm/shim")
  set(GX_FDM_SHIM_SRC "${GX_FDM_SHIM_DIR}/gx_fdm.cpp")
  set(GX_FDM_SHIM_KIND real)
else()
  set(GX_FDM_SHIM_DIR "${GX_REPO}/src/fdm/shim_stub")
  set(GX_FDM_SHIM_SRC "${GX_FDM_SHIM_DIR}/gx_fdm_stub.cpp")
  set(GX_FDM_SHIM_KIND stub)
endif()
message(STATUS "gx_fdm shim: ${GX_FDM_SHIM_KIND} (${GX_FDM_SHIM_SRC})")
file(WRITE "${CMAKE_BINARY_DIR}/out/gx_fdm_shim_kind.txt" "${GX_FDM_SHIM_KIND}\n")

add_library(gx_fdm SHARED "${GX_FDM_SHIM_SRC}")
target_include_directories(gx_fdm PUBLIC "${GX_FDM_SHIM_DIR}")
target_compile_features(gx_fdm PRIVATE cxx_std_17)
target_link_libraries(gx_fdm PRIVATE libJSBSim)
# Export only the C API (gx_fdm_*), whatever visibility the header declares.
target_link_options(gx_fdm PRIVATE "-Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/gx_fdm.map")
set_target_properties(gx_fdm PROPERTIES LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/gx_fdm.map")

add_custom_target(m1_fdm ALL DEPENDS gx_fdm libJSBSim JSBSim)
