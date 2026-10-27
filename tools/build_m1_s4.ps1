[CmdletBinding()]
param(
 [string]$ZigRoot=$env:GX_ZIG_ROOT,
 [string]$ZigArchive=$env:GX_ZIG_ARCHIVE,
 [string]$BuildDir='',
 [switch]$SkipFdm
)
$ErrorActionPreference='Stop'
# S4 keeps the complete, reproducible S2 AArch64/JSBSim stage and replaces its
# board entry points with the S4 direct-KMS launch envelope. The S4
# controller is the accepted M0-001C controller plus exactly one change
# (LimitRTPRIO=60 / LimitNICE=30 on the candidate unit, ISS-038/BOARD-002);
# both hashes are pinned below and tools/test_m1_s4_platform.py pins the diff.
$repo=(Resolve-Path "$PSScriptRoot\..").Path
if(!$ZigRoot){$ZigRoot=Join-Path $repo '.toolchains\zig-0.17.0'}
if(!$ZigArchive){$ZigArchive=Join-Path $repo '.toolchains\zig-0.17.0.zip'}
if(!$BuildDir){$BuildDir=Join-Path $repo 'build\m1-s4'}
$stage=Join-Path $BuildDir 'stage'
function Need($ok,$msg){if(!$ok){throw $msg}}
$m0Controller='c4c270059a22592f63195f3f58cef372bfb3598e1137042ebfa56fe18ad5febc'
$s4Controller='a6e1bbc252e14c884f5c6a5008700c0fbdbf5f804022c9dee640d1aab270a4f0'

$buildArgs=@('-NoProfile','-ExecutionPolicy','Bypass','-File',"$PSScriptRoot\build_m1_s2.ps1",'-ZigRoot',$ZigRoot,'-ZigArchive',$ZigArchive,'-BuildDir',$BuildDir)
if($SkipFdm){$buildArgs+='-SkipFdm'}
& powershell @buildArgs
Need (!$LASTEXITCODE) "build_m1_s2.ps1 failed: $LASTEXITCODE"

foreach($old in 'm1_s2_run.sh','m1_s2_smoke.sh','m1_s2_replay.sh','m1_s2_board_tests.sh'){
  $path=Join-Path $stage $old
  if(Test-Path -LiteralPath $path){Remove-Item -LiteralPath $path -Force}
}
foreach($s in 'm1_s4_run.sh','m1_s4_launch.sh','m1_s4_kms_window.sh','m1_s4_board_tests.sh','m1_s4_smoke.sh'){
  $text=[IO.File]::ReadAllText((Join-Path $PSScriptRoot $s)).Replace("`r`n","`n")
  [IO.File]::WriteAllText((Join-Path $stage $s),$text,[Text.UTF8Encoding]::new($false))
}
$m0=(Get-FileHash -Algorithm SHA256 -LiteralPath "$PSScriptRoot\m0_001c_kms_window.sh").Hash.ToLowerInvariant()
Need ($m0 -eq $m0Controller) "accepted M0-001C controller hash changed: $m0"
$controller=(Get-FileHash -Algorithm SHA256 -LiteralPath "$stage\m1_s4_kms_window.sh").Hash.ToLowerInvariant()
Need ($controller -eq $s4Controller) "S4 controller hash changed: $controller"
$ErrorActionPreference='Continue'
& py -I "$PSScriptRoot\test_m1_s4_platform.py"
$platformRc=$LASTEXITCODE
$ErrorActionPreference='Stop'
Need ($platformRc -eq 0) 'S4 platform static tests failed (controller diff, launcher bounds, RT properties)'

$s2=Get-Content -Raw -LiteralPath "$stage\build_manifest.m1-s2.json"|ConvertFrom-Json
$manifest=[ordered]@{
  schema='gx-s4-stage/1'; source_commit=$s2.source_commit; source_dirty=$s2.source_dirty
  toolchain=$s2.toolchain; gx_app=$s2.gx_app
  libgx_fdm_sha256=$s2.libgx_fdm_sha256; libJSBSim_sha256=$s2.libJSBSim_sha256
  jsbsim=$s2.jsbsim; aircraft=$s2.aircraft
  jsbsim_csv_output_directives_removed=$s2.jsbsim_csv_output_directives_removed
  tests=$s2.tests
  visible_controller=[ordered]@{name='m1_s4_kms_window.sh';sha256=$controller;derived_from=[ordered]@{name='m0_001c_kms_window.sh';sha256=$m0;provenance='accepted M0-001C controller, integration commit 3d419e1'};change='candidate unit: -p LimitRTPRIO=60 -p LimitNICE=30 (ISS-038, BOARD-002)'}
  display=[ordered]@{connector='DP-1';render_mode='1280x720@60';desktop_restore='1920x1080@60'}
  world=[ordered]@{pack='gx-world synthetic_sfo_like (GXWORLD 2.0, synthetic, not surveyed)';aircraft_model='generated placeholder Global 5000 (gx-render aircraft.rs, no external asset)'}
}
$manifest|ConvertTo-Json -Depth 6|Set-Content -LiteralPath "$stage\build_manifest.m1-s4.json" -Encoding utf8
Remove-Item -LiteralPath "$stage\build_manifest.m1-s2.json","$stage\files.sha256" -Force
$hash={param($p)(Get-FileHash -Algorithm SHA256 -LiteralPath $p).Hash.ToLowerInvariant()}
$lines=foreach($f in Get-ChildItem -LiteralPath $stage -Recurse -File|Where-Object{$_.Name -ne 'files.sha256'}|Sort-Object FullName){
  "$(& $hash $f.FullName)  $($f.FullName.Substring($stage.Length+1).Replace('\','/'))"
}
[IO.File]::WriteAllText("$stage\files.sha256",(($lines -join "`n")+"`n"),[Text.UTF8Encoding]::new($false))
Write-Host "M1 S4 stage ready: $stage (commit $($s2.source_commit), dirty=$($s2.source_dirty))"
