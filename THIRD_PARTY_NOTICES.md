# Third-party notices

Uno Q Flight Sim includes build and conversion support for third-party open
source software and data. Project copyright does not extend to those works.

## JSBSim

- Upstream: https://github.com/JSBSim-Team/jsbsim
- Pinned alpha input: v1.3.2, commit `14d23722f7f529e62cd488a76e5e8c4320613c3a`
- License: GNU Lesser General Public License 2.1 or later, subject to the
  notices and per-file exceptions contained in the upstream source archive.
- Integration: dynamically linked shared library; the fetch script preserves
  the complete upstream source used by the build.

## FlightGear Aircraft-2024 787-8

- Upstream archive: https://fgaddon.b-cdn.net/Aircraft-2024/787-8.zip
- Pinned alpha archive SHA-256:
  `9061f74b83edeacbbc0b205c128494edb81a29b5b35530e21f1660ddd32018c6`
- License: GNU General Public License 2.0-compatible aircraft content; consult
  the aircraft's `COPYING` and author files from the exact acquired revision.
- Modifications performed by this project: conversion to GXVISUAL 1.2,
  animation-joint extraction, and generation of a fictional dark BBJ livery.
- Any distributed generated aircraft pack must include the applicable GPL,
  authorship, exact source revision, modification notice and corresponding
  source or a GPL-compliant source offer.

## Rust crates and build tools

Rust dependencies are pinned by `rust/Cargo.lock` and retain their individual
licenses. Zig, Rust, cargo-zigbuild, CMake, Ninja, Python, astcenc and system
libraries are external build tools and are not relicensed by this project.
