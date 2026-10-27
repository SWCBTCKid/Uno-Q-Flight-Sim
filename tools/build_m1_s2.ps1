[CmdletBinding()]
param(
 [string]$ZigRoot=$env:GX_ZIG_ROOT,
 [string]$ZigArchive=$env:GX_ZIG_ARCHIVE,
 [string]$BuildDir='',
 # Reuse an existing build\m1-fdm C++ output (libgx_fdm.so, libJSBSim.so.1).
 [switch]$SkipFdm
)
$ErrorActionPreference='Stop'
# M1 S2 board build (ARCH-002, BUILD-003): the pinned JSBSim v1.3.2 + gx_fdm
# shim C++ build of tools\build_m1_fdm.ps1 (Debian 13 libstdc++ headers, the
# board's own libstdc++.so.6/libgcc_s.so.1), then gx-app with the `jsbsim`
# feature and every workspace test binary through cargo-zigbuild for
# aarch64-unknown-linux-gnu.2.34. The FFI crates cannot link on the Windows
# host, so their tests run on the board (tools\m1_s2_board_tests.sh).
# Output <BuildDir>\stage:
#   bin\      gx-app, libgx_fdm.so, libJSBSim.so.1 (RUNPATH $ORIGIN)
#   jsbsim\   aircraft\global5000, aircraft87-8, aircraft\c172p, engine\, systems\
#   tests\    AArch64 test executables + the two libraries
#   *.sh      board runners; build_manifest.m1-s2.json, elf_report.json,
#             files.sha256 (sha256sum -c format)
$repo=(Resolve-Path "$PSScriptRoot\..").Path
if(!$ZigRoot){$ZigRoot=Join-Path $repo '.toolchains\zig-0.17.0'}
if(!$ZigArchive){$ZigArchive=Join-Path $repo '.toolchains\zig-0.17.0.zip'}
if(!$BuildDir){$BuildDir=Join-Path $repo 'build\m1-s2'}
$stage=Join-Path $BuildDir 'stage'
$targetDir=Join-Path $BuildDir 'rust'
$fdmOut=Join-Path $repo 'build\m1-fdm\cxx\out'
$jsbsimSrc=Join-Path $repo 'third_party\m1\jsbsim-1.3.2'
$zig=Join-Path $ZigRoot 'zig.exe'
$rustTarget='aarch64-unknown-linux-gnu'
function Need($ok,$msg){if(!$ok){throw $msg}}

if(!$SkipFdm){
  # Toolchain pins (Zig sha256, rustc 1.90.0, cargo-zigbuild 0.23.4) are
  # checked there; it also fetches/verifies the pinned sources.
  & powershell -NoProfile -ExecutionPolicy Bypass -File "$PSScriptRoot\build_m1_fdm.ps1" -ZigRoot $ZigRoot -ZigArchive $ZigArchive
  Need (!$LASTEXITCODE) "build_m1_fdm.ps1 failed: $LASTEXITCODE"
}
foreach($f in 'libgx_fdm.so','libJSBSim.so.1'){Need (Test-Path -LiteralPath "$fdmOut\$f") "C++ output missing: $fdmOut\$f (run without -SkipFdm)"}
Need (Test-Path -LiteralPath "$jsbsimSrc\aircraft\global5000") "JSBSim data missing: $jsbsimSrc"
$zigVersion=(& $zig version).Trim()
Need ($zigVersion -eq '0.17.0') "Expected Zig 0.17.0, found '$zigVersion'"

$sourceCommit=(& git -C $repo rev-parse HEAD).Trim()
$sourceDirty=[bool]((& git -C $repo status --porcelain --untracked-files=no) -join '')
$env:CARGO_ZIGBUILD_ZIG_PATH=$zig
$env:CARGO_ZIGBUILD_PYTHON_PATH='gx-no-python-ziglang'
$env:CARGO_TARGET_DIR=$targetDir
$env:GX_COMMIT=$sourceCommit
$env:GX_FDM_LIB_DIR=$fdmOut
$env:GX_FDM_LINK_BY_PATH='1'   # Zig 0.17 Windows-host -l bug, see gx-fdm-sys/build.rs
New-Item -ItemType Directory -Force -Path $BuildDir|Out-Null
$linkToml=Join-Path $BuildDir 'gx_fdm_link.toml'
$linkArg=("$fdmOut\libgx_fdm.so").Replace('\','/')
[IO.File]::WriteAllText($linkToml,"[target.$rustTarget]`nrustflags = [`"-C`", `"link-arg=$linkArg`", `"-C`", `"link-arg=-Wl,-rpath,`$ORIGIN`"]`n",[Text.UTF8Encoding]::new($false))
$testList=Join-Path $BuildDir 'tests.json'
Push-Location (Join-Path $repo 'rust')
try {
  & cargo zigbuild --release --locked --target "$rustTarget.2.34" --config $linkToml -p gx-app --features gx-app/jsbsim
  Need (!$LASTEXITCODE) "cargo zigbuild (gx-app) failed: $LASTEXITCODE"
  # Every workspace test target, with the JSBSim feature on, for the board.
  $json=& cargo zigbuild --release --locked --target "$rustTarget.2.34" --config $linkToml --workspace --tests --features gx-app/jsbsim,gx-sim/jsbsim --message-format=json
  Need (!$LASTEXITCODE) "cargo zigbuild --tests failed: $LASTEXITCODE"
} finally { Pop-Location }
$tests=@()
foreach($line in $json){
  if($line -notmatch '"reason":"compiler-artifact"'){continue}
  $m=$line|ConvertFrom-Json
  if($m.profile.test -and $m.executable){$tests+=[pscustomobject]@{name="$($m.target.name)-$($m.target.kind[0])";exe=$m.executable}}
}
Need ($tests.Count -ge 5) "too few test executables found ($($tests.Count))"

$binary=Join-Path $targetDir "$rustTarget\release\gx-app"
Need (Test-Path -LiteralPath $binary -PathType Leaf) "Cross-built gx-app missing: $binary"
if(Test-Path -LiteralPath $stage){Remove-Item -LiteralPath $stage -Recurse -Force}
New-Item -ItemType Directory -Path "$stage\bin","$stage\tests","$stage\jsbsim\aircraft" -Force|Out-Null
Copy-Item -LiteralPath $binary -Destination "$stage\bin\gx-app"
foreach($f in 'libgx_fdm.so','libJSBSim.so.1'){Copy-Item -LiteralPath "$fdmOut\$f" -Destination "$stage\bin"; Copy-Item -LiteralPath "$fdmOut\$f" -Destination "$stage\tests"}
foreach($t in $tests){Copy-Item -LiteralPath $t.exe -Destination "$stage\tests\$($t.name)"}
foreach($a in 'global5000','787-8','c172p'){Copy-Item -LiteralPath "$jsbsimSrc\aircraft\$a" -Destination "$stage\jsbsim\aircraft\$a" -Recurse}
# No file I/O from the realtime sim thread: global5000 and c172p carry a CSV <output>
# logging directive, which JSBSim opens (truncate + header) in RunIC, i.e. on
# every reset, on the thread that owns JSBSim. Remove exactly that element
# from the staged copies (the flight model itself is untouched); the
# adapter also forces simulation/output[i]/enabled = 0 if any remains.
# The stock 787-8 has none (its <output> elements are FCS outputs).
$removed=[ordered]@{}
$directives=[ordered]@{'global5000'=1;'787-8'=0;'c172p'=1}
foreach($a in $directives.Keys){
  $xml="$stage\jsbsim\aircraft\$a\$a.xml"
  $text=[IO.File]::ReadAllText($xml)
  $rx=[regex]'(?s)[ \t]*<output\s+name="[^"]*"[^>]*>.*?</output>\s*'
  $n=$rx.Matches($text).Count
  Need ($n -eq $directives[$a]) "expected $($directives[$a]) <output name=...> logging directive(s) in $a.xml, found $n"
  [IO.File]::WriteAllText($xml,$rx.Replace($text,''),[Text.UTF8Encoding]::new($false))
  $removed[$a]=$n
}
foreach($d in 'engine','systems'){Copy-Item -LiteralPath "$jsbsimSrc\$d" -Destination "$stage\jsbsim\$d" -Recurse}
foreach($s in 'm1_s2_board_tests.sh'){
  $text=[IO.File]::ReadAllText((Join-Path $PSScriptRoot $s)).Replace("`r`n","`n")
  [IO.File]::WriteAllText((Join-Path $stage $s),$text,[Text.UTF8Encoding]::new($false))
}

# ABI gates (BUILD-003): our objects GLIBC <= 2.34, GLIBCXX <= 3.4.33; the
# LGPL boundary: gx-app and the shim define no JSBSim symbols.
$elfs=@("$stage\bin\gx-app","$stage\bin\libgx_fdm.so","$stage\bin\libJSBSim.so.1")+@(Get-ChildItem -LiteralPath "$stage\tests" -File|Where-Object{$_.Name -notlike '*.so*'}|ForEach-Object{$_.FullName})
& py "$PSScriptRoot\elf_m1_report.py" @elfs --max-glibc 2.34 --max-glibcxx 3.4.33 --output "$stage\elf_report.json"|Out-Null
Need (!$LASTEXITCODE) "ELF ABI gate failed (see $stage\elf_report.json)"
& py "$PSScriptRoot\elf_m1_report.py" "$stage\bin\gx-app" "$stage\bin\libgx_fdm.so" --forbid-defined-jsbsim|Out-Null
Need (!$LASTEXITCODE) 'gx-app or the shim defines JSBSim symbols: LGPL shared-library boundary breached'

$hash={param($p)(Get-FileHash -Algorithm SHA256 -LiteralPath $p).Hash.ToLowerInvariant()}
[ordered]@{
  schema='gx-s2-stage/1'; source_commit=$sourceCommit; source_dirty=$sourceDirty
  toolchain=[ordered]@{zig=$zigVersion;rustc=(& rustc --version).Trim();target="$rustTarget.2.34";cxx_runtime='Debian 13 libstdc++6/libgcc-s1 14.2.0-19 (board copy, BUILD-003)'}
  gx_app=[ordered]@{sha256=(& $hash "$stage\bin\gx-app");size_bytes=(Get-Item -LiteralPath "$stage\bin\gx-app").Length;features=@('jsbsim')}
  libgx_fdm_sha256=(& $hash "$stage\bin\libgx_fdm.so"); libJSBSim_sha256=(& $hash "$stage\bin\libJSBSim.so.1")
  jsbsim='v1.3.2 (tools/fetch_m1_fdm_deps.py pin)'; aircraft=@('global5000','787-8','c172p')
  jsbsim_csv_output_directives_removed=$removed
  tests=@($tests|ForEach-Object{$_.name})
}|ConvertTo-Json -Depth 5|Set-Content -LiteralPath "$stage\build_manifest.m1-s2.json" -Encoding utf8
$lines=foreach($f in Get-ChildItem -LiteralPath $stage -Recurse -File|Where-Object{$_.Name -ne 'files.sha256'}|Sort-Object FullName){
  "$(& $hash $f.FullName)  $($f.FullName.Substring($stage.Length+1).Replace('\','/'))"
}
[IO.File]::WriteAllText("$stage\files.sha256",(($lines -join "`n")+"`n"),[Text.UTF8Encoding]::new($false))
Write-Host "M1 S2 stage ready: $stage (commit $sourceCommit, dirty=$sourceDirty, $($tests.Count) test executables)"
