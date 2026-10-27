# Uno Q Flight Sim — AP-002 Alpha

An experimental native flight simulator for the Arduino UNO Q. AP-002 flies
the stock JSBSim 787-8 with mouse-controlled LVL, HDG, ALT, SPD and ILS modes,
including nearest-flight-path runway selection, glideslope capture from above,
and a manual handoff at 100 ft for the flare.

This repository is an alpha source release. It is not navigation software,
certified training software, or suitable for controlling a real aircraft.

## Tested hardware

- Arduino UNO Q running the tested Debian/Qualcomm graphics stack
- Display connected to DP-1 through the tested hub
- Logitech Extreme 3D Pro joystick
- Linux evdev mouse
- Windows development host with PowerShell, Rust 1.90, Zig 0.17,
  cargo-zigbuild 0.23.4, CMake, Ninja, Python and OpenSSH

## Source and assets

Project-authored source is GPL-2.0-or-later. The full profile is assembled
from public upstream sources. The acquisition and conversion pipeline does not
claim ownership of those inputs. Read [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)
and [ASSET_SOURCES.md](ASSET_SOURCES.md) before distributing a generated pack.

Generated scenery, downloaded aircraft sources, compiler archives, build
products and flight logs are intentionally excluded from Git.

## Alpha build status

The pre-release acceptance candidate passed 14/14 real-board JSBSim cases and
23/23 board executables. Its 765-second flight had zero simulation deadline
misses and a smooth ILS handoff. The public export must independently
reproduce those gates before `v0.1.0-alpha.1` is published.

## Build and run AP-002

Install the host prerequisites above, then install the pinned Python packages:

```powershell
py -m pip install -r tools\spike002-requirements.txt
```

Set `GX_ZIG_ROOT` and `GX_ZIG_ARCHIVE` to Zig 0.17.0 and its original ZIP.
Set `GX_CMAKE` and `GX_NINJA` if those tools are not on `PATH`. Download Arm
ASTC Encoder 5.7.0 from Arm's official release and pass its executable path:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\prepare_alpha_assets.ps1 `
  -Astcenc C:\path\to\astcenc-sse4.1.exe
powershell -NoProfile -ExecutionPolicy Bypass -File tools\build_spike002.ps1
```

The first command downloads and verifies the pinned GPL FlightGear 787,
bounded public SFO source data, and NOAA lidar, then creates
`build\alpha-runtime`. It can download roughly 1 GB and produces large local
build files. The second command fetches pinned JSBSim/AArch64 dependencies,
cross-builds the application, runs the static platform checks, and creates
`build\m1-s4\stage`.

Deploy without taking over the display:

```powershell
$env:GX_BOARD='arduino@uno-q-hostname'
powershell -NoProfile -ExecutionPolicy Bypass -File tools\deploy_m1_s4.ps1 `
  -RunId ap002-alpha -ArtifactRoot artifacts\local\ap002-alpha
```

The deploy script prints the final `ssh` command. Run that command from a
physical session when ready. With the default zero duration, the flight ends
on a double-click of `EXIT`; a three-hour deadman remains active. The visible
launcher temporarily takes over the display and requires `sudo`. The restore
path verifies the desktop before returning.

To use an already acquired FlightGear 787 source tree, add
`-Aircraft787 C:\path\to\787-8` to `prepare_alpha_assets.ps1`. The pipeline
records hashes and provenance in its generated manifests. Do not commit
`build`, `asset-cache`, generated packs, or flight logs.

## No affiliation

This community project is not affiliated with or endorsed by Arduino, Boeing,
FlightGear, JSBSim, the FAA, or any imagery/data provider. Aircraft names may
be trademarks of their respective owners.
