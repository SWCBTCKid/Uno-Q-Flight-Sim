[CmdletBinding()]
param(
 [string]$ZigRoot=$env:GX_ZIG_ROOT,
 [string]$ZigArchive=$env:GX_ZIG_ARCHIVE,
 [string]$CMake=$env:GX_CMAKE,
 [string]$Ninja=$env:GX_NINJA,
 [string]$BuildDir='',
 # Delete the C++ and Rust build trees first (measure 9 "clean" build).
 [switch]$Clean,
 # Also use an empty Zig cache (Zig's libc stubs/compiler-rt/libunwind are
 # rebuilt): the coldest possible build on this host.
 [switch]$ColdZigCache,
 # Skip the C++ side (fast Rust-only iteration; libgx_fdm.so must exist).
 [switch]$RustOnly)
$ErrorActionPreference='Stop'
# M1-001 (ARCH-002) cross-build: JSBSim v1.3.2 (shared) + gx_fdm shim (shared)
# + the Rust workspace in rust/ via cargo-zigbuild, all for the UNO Q
# (aarch64-linux-gnu, glibc 2.34 ceiling). Output: a self-contained stage
# directory that tools/deploy_run_m1_fdm.ps1 pushes to the board:
#   <BuildDir>\stage\bin\     m1-fdm-bench, JSBSim,
#                             libgx_fdm.so, libJSBSim.so.1
#                             (RUNPATH $ORIGIN everywhere; no LD_LIBRARY_PATH)
#   <BuildDir>\stage\jsbsim\  JSBSim data root: aircraft\ engine\ systems\ scripts\
#   <BuildDir>\stage\tests\m1\scripts\   Flight Dynamics scripts, if present
#   <BuildDir>\stage\probe\   Platform exception probe script
#   <BuildDir>\stage\build_manifest.m1-fdm.json, files.sha256, elf_report.json
# The M0 targets (tools/build_m0_target.ps1, cmake/toolchains/uno-q-aarch64-
# linux-gnu.cmake) are not touched.
$repo=(Resolve-Path "$PSScriptRoot\..").Path
if(!$ZigRoot){$ZigRoot=Join-Path $repo '.toolchains\zig-0.17.0'}
if(!$ZigArchive){$ZigArchive=Join-Path $repo '.toolchains\zig-0.17.0.zip'}
if(!$BuildDir){$BuildDir=Join-Path $repo 'build\m1-fdm'}
$cxxDir=Join-Path $BuildDir 'cxx'; $rustDir=Join-Path $BuildDir 'rust'; $stage=Join-Path $BuildDir 'stage'
$cmake=$(if($CMake){$CMake}else{(Get-Command cmake -ErrorAction Stop).Source})
$ninja=$(if($Ninja){$Ninja}else{(Get-Command ninja -ErrorAction Stop).Source})
$zig="$ZigRoot\zig.exe"
$expectedZig='b5663f69581dcf391293fbf16c06cb80d81d806545ce618b4d0bab7f0eb8c428'
$expectedRustc='rustc 1.90.0 (1159e78c4 2025-09-14)'
$expectedZigbuild='cargo-zigbuild 0.23.4'
$rustTarget='aarch64-unknown-linux-gnu'
function Need($ok,$msg){if(!$ok){throw $msg}}

# ---- toolchain checks (same Zig pin as M0) -----------------------------------
Need ((Test-Path -LiteralPath $cmake) -and (Test-Path -LiteralPath $ninja)) 'VS bundled CMake/Ninja are unavailable'
Need (Test-Path -LiteralPath $ZigArchive) "Pinned Zig archive unavailable: $ZigArchive"
$actual=(Get-FileHash -Algorithm SHA256 -LiteralPath $ZigArchive).Hash.ToLowerInvariant()
Need ($actual -eq $expectedZig) "Zig archive SHA-256 mismatch: $actual"
$zigVersion=(& $zig version).Trim()
Need (!$LASTEXITCODE -and $zigVersion -eq '0.17.0') "Expected Zig 0.17.0, found '$zigVersion'"
# rust/rust-toolchain.toml selects the pinned rustc when run from rust/.
Push-Location (Join-Path $repo 'rust')
try{
  $rustc=(& rustc --version).Trim(); Need (!$LASTEXITCODE -and $rustc -eq $expectedRustc) "Expected '$expectedRustc', found '$rustc' (rustup toolchain install 1.90.0 --target $rustTarget)"
  $cargoV=(& cargo --version).Trim()
  $targets=@(& rustup target list --installed); Need ($targets -contains $rustTarget) "Rust target $rustTarget missing (rustup target add $rustTarget --toolchain 1.90.0)"
}finally{Pop-Location}
$zigbuild=(& cargo-zigbuild --version|Select-Object -First 1).Trim()
Need (!$LASTEXITCODE -and $zigbuild -eq $expectedZigbuild) "Expected '$expectedZigbuild', found '$zigbuild' (cargo install cargo-zigbuild --version 0.23.4 --locked)"

# ---- pinned sources ------------------------------------------------------------
& py "$repo\tools\fetch_m1_fdm_deps.py"
Need (!$LASTEXITCODE) 'fetch_m1_fdm_deps.py failed'
$third=Join-Path $repo 'third_party\m1'
$stamp=Get-Content -Raw -LiteralPath "$third\fetch_stamp.json"|ConvertFrom-Json
$jsbsimSrc=Join-Path $third 'jsbsim-1.3.2'; $cxxrt=Join-Path $third 'gcc14-runtime'
& py "$repo\tools\patch_jsbsim_m1.py" $jsbsimSrc
Need (!$LASTEXITCODE) 'deterministic JSBSim standalone timing patch failed'
$standaloneDriverHash=(Get-FileHash -Algorithm SHA256 -LiteralPath "$jsbsimSrc\src\JSBSim.cpp").Hash.ToLowerInvariant()
# JSBSim stamps __DATE__/__TIME__: make it the tag commit time, not build time.
$env:SOURCE_DATE_EPOCH=[string]$stamp.jsbsim_source_date_epoch

if($Clean){foreach($d in $cxxDir,$rustDir){if(Test-Path -LiteralPath $d){Remove-Item -LiteralPath $d -Recurse -Force}}}
if($ColdZigCache){
  $zc=Join-Path $BuildDir 'zig-cache-cold'
  if(Test-Path -LiteralPath $zc){Remove-Item -LiteralPath $zc -Recurse -Force}
  New-Item -ItemType Directory -Path $zc|Out-Null
  $env:ZIG_GLOBAL_CACHE_DIR=$zc; $env:ZIG_LOCAL_CACHE_DIR=$zc
}
$timing=[ordered]@{}
$fwd={param($p)$p.Replace('\','/')}

# ---- C++: JSBSim + shim ----------------------------------------------------------
if(!$RustOnly){
  $sw=[Diagnostics.Stopwatch]::StartNew()
  & $cmake -S "$repo\cmake\m1_fdm" -B $cxxDir -G Ninja "-DCMAKE_MAKE_PROGRAM=$ninja" `
    "-DCMAKE_TOOLCHAIN_FILE=$(& $fwd "$repo\cmake\toolchains\uno-q-aarch64-linux-gnu-libstdcxx.cmake")" `
    "-DUNO_Q_ZIG=$(& $fwd $zig)" "-DGX_CXXRT_ROOT=$(& $fwd $cxxrt)" "-DGX_JSBSIM_SOURCE_DIR=$(& $fwd $jsbsimSrc)" -DCMAKE_BUILD_TYPE=Release
  Need (!$LASTEXITCODE) "CMake configure failed: $LASTEXITCODE"
  $timing.cxx_configure_s=[math]::Round($sw.Elapsed.TotalSeconds,2); $sw.Restart()
  & $cmake --build $cxxDir --target m1_fdm
  Need (!$LASTEXITCODE) "C++ build failed: $LASTEXITCODE"
  $timing.cxx_build_s=[math]::Round($sw.Elapsed.TotalSeconds,2)
}
$cxxOut=Join-Path $cxxDir 'out'
foreach($f in 'libgx_fdm.so','libJSBSim.so.1','JSBSim'){Need (Test-Path -LiteralPath "$cxxOut\$f") "C++ output missing: $cxxOut\$f"}

# ---- Rust: cargo-zigbuild ----------------------------------------------------------
$env:CARGO_ZIGBUILD_ZIG_PATH=$zig
$env:CARGO_ZIGBUILD_PYTHON_PATH='gx-no-python-ziglang'   # never pick up a pip `ziglang`
$env:CARGO_TARGET_DIR=$rustDir
$env:GX_FDM_LIB_DIR=$cxxOut
$env:GX_FDM_LINK_BY_PATH='1'   # Zig 0.17 Windows-host -l bug, see gx-fdm-sys/build.rs
New-Item -ItemType Directory -Force -Path $BuildDir|Out-Null
$linkToml=Join-Path $BuildDir 'gx_fdm_link.toml'
$linkArg=& $fwd "$cxxOut\libgx_fdm.so"
# A --config file, not an inline value: PS 5.1 mangles embedded quotes.
[IO.File]::WriteAllText($linkToml,"[target.$rustTarget]`nrustflags = [`"-C`", `"link-arg=$linkArg`"]`n",[Text.UTF8Encoding]::new($false))
Push-Location (Join-Path $repo 'rust')
try{
  $sw=[Diagnostics.Stopwatch]::StartNew()
  & cargo zigbuild --release --locked --target "$rustTarget.2.34" --config $linkToml
  Need (!$LASTEXITCODE) "cargo zigbuild failed: $LASTEXITCODE"
  $timing.rust_build_s=[math]::Round($sw.Elapsed.TotalSeconds,2)
}finally{Pop-Location}
$rustOut=Join-Path $rustDir "$rustTarget\release"
$bench=Join-Path $rustOut 'm1-fdm-bench'
Need (Test-Path -LiteralPath $bench) "Rust output missing: $bench"

# ---- stage -----------------------------------------------------------------------
if(Test-Path -LiteralPath $stage){Remove-Item -LiteralPath $stage -Recurse -Force}
New-Item -ItemType Directory -Path "$stage\bin","$stage\jsbsim","$stage\probe"|Out-Null
Copy-Item -LiteralPath $bench,"$cxxOut\libgx_fdm.so","$cxxOut\libJSBSim.so.1","$cxxOut\JSBSim" -Destination "$stage\bin"
foreach($d in 'aircraft','engine','systems','scripts'){Copy-Item -LiteralPath "$jsbsimSrc\$d" -Destination "$stage\jsbsim\$d" -Recurse}
if(Test-Path -LiteralPath "$repo\tests\m1\scripts"){
  New-Item -ItemType Directory -Path "$stage\tests\m1"|Out-Null
  Copy-Item -LiteralPath "$repo\tests\m1\scripts" -Destination "$stage\tests\m1\scripts" -Recurse
}
Copy-Item -LiteralPath "$repo\tests\m1_platform\exception_probe.xml" -Destination "$stage\probe"

# ---- ABI gates: AArch64, GLIBC <= 2.34 (ours), GLIBCXX <= board libstdc++ -----------
$bins=@('m1-fdm-bench','libgx_fdm.so','libJSBSim.so.1','JSBSim'|ForEach-Object{"$stage\bin\$_"})
& py "$repo\tools\elf_m1_report.py" @bins --max-glibc 2.34 --max-glibcxx 3.4.33 --output "$stage\elf_report.json"|Out-Null
Need (!$LASTEXITCODE) "ELF ABI gate failed (see $stage\elf_report.json)"
$elf=Get-Content -Raw -LiteralPath "$stage\elf_report.json"|ConvertFrom-Json
& py "$repo\tools\elf_m1_report.py" "$stage\bin\m1-fdm-bench" "$stage\bin\libgx_fdm.so" --forbid-defined-jsbsim|Out-Null
Need (!$LASTEXITCODE) "Harness/shim defines JSBSim symbols: LGPL shared-library boundary breached (see $stage\elf_report.json)"

# files.sha256: `sha256sum -c` format, LF, relative to the stage root.
$lines=foreach($f in Get-ChildItem -LiteralPath $stage -Recurse -File|Sort-Object FullName){
  $rel=$f.FullName.Substring($stage.Length+1).Replace('\','/')
  "$((Get-FileHash -Algorithm SHA256 -LiteralPath $f.FullName).Hash.ToLowerInvariant())  $rel"
}
[IO.File]::WriteAllText("$stage\files.sha256",(($lines -join "`n")+"`n"),[Text.UTF8Encoding]::new($false))

$sourceCommit=(& git -C $repo rev-parse HEAD).Trim()
$sourceDirty=[bool]((& git -C $repo status --porcelain) -join '')
$cxxCmds=@(& $ninja -C $cxxDir -t commands gx_fdm)
$artifacts=[ordered]@{}
foreach($e in $elf){
  $n=Split-Path -Leaf $e.path
  $artifacts[$n]=[ordered]@{size_bytes=$e.size_bytes;sha256=(Get-FileHash -Algorithm SHA256 -LiteralPath $e.path).Hash.ToLowerInvariant();
    needed=$e.needed;runpath=$e.rpath_runpath;glibc_max=$e.glibc_max;glibcxx_max=$e.glibcxx_max;exports=$e.exports;
    defined_jsbsim=$e.defined_jsbsim}
}
[ordered]@{
 source_commit=$sourceCommit; source_dirty=$sourceDirty
 shim_kind=(Get-Content -LiteralPath "$cxxOut\gx_fdm_shim_kind.txt" -TotalCount 1).Trim()
 toolchain=[ordered]@{zig=$zigVersion;zig_archive_sha256=$expectedZig;rustc=$rustc;cargo=$cargoV;cargo_zigbuild=$zigbuild
   rust_target="$rustTarget.2.34";cxx_target='aarch64-linux-gnu.2.34';cxx_runtime='Debian 13 libstdc++6/libgcc-s1 14.2.0-19 (board copy)'}
 jsbsim=$stamp.pins.jsbsim; cxx_runtime_debs=$stamp.pins.debs
 jsbsim_standalone_driver_sha256=$standaloneDriverHash
 source_date_epoch=$env:SOURCE_DATE_EPOCH
 cold_zig_cache=[bool]$ColdZigCache; clean=[bool]$Clean; rust_only=[bool]$RustOnly
 timing_s=$timing
 artifacts=$artifacts
 shim_compile_command=($cxxCmds|Where-Object{$_ -match '\s-c\s'}|Select-Object -Last 1)
 shim_link_command=($cxxCmds|Select-Object -Last 1)
}|ConvertTo-Json -Depth 6|Set-Content -LiteralPath "$stage\build_manifest.m1-fdm.json" -Encoding utf8
Write-Host "M1 FDM stage ready: $stage (timing: $(($timing.GetEnumerator()|ForEach-Object{"$($_.Key)=$($_.Value)"}) -join ', '))"
exit 0
