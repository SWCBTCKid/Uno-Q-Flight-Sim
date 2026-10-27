[CmdletBinding()]
param(
 [string]$Runtime='',
 [string]$ZigRoot=$env:GX_ZIG_ROOT,
 [string]$ZigArchive=$env:GX_ZIG_ARCHIVE,
 # JSBSim aircraft the bundle flies (m1_s4_launch.sh reads stage/aircraft).
 [ValidateSet('global5000','787-8','c172p')][string]$Aircraft='787-8',
 [switch]$SkipFdm
)
$ErrorActionPreference='Stop'
# SPIKE-002 bundle: the S4 stage (build_m1_s4.ps1: same launcher, controller,
# watchdog, RT envelope and tests) plus spike002/ runtime assets and the
# offscreen smoke + sampler scripts. files.sha256 is regenerated over the
# whole stage, so the runner's sha256sum check covers the assets too.
$repo=(Resolve-Path "$PSScriptRoot\..").Path
if(!$Runtime){$Runtime=Join-Path $repo 'build\alpha-runtime'}
function Need($ok,$msg){if(!$ok){throw $msg}}
$s4Args=@('-NoProfile','-ExecutionPolicy','Bypass','-File',"$PSScriptRoot\build_m1_s4.ps1")
if($ZigRoot){$s4Args+=@('-ZigRoot',$ZigRoot)}
if($ZigArchive){$s4Args+=@('-ZigArchive',$ZigArchive)}
if($SkipFdm){$s4Args+='-SkipFdm'}
& powershell @s4Args
Need (!$LASTEXITCODE) "build_m1_s4.ps1 failed: $LASTEXITCODE"
$stage=Join-Path $repo 'build\m1-s4\stage'
$assets=Join-Path $stage 'spike002'
New-Item -ItemType Directory -Force -Path $assets|Out-Null
foreach($f in 'sfo-photoreal.gxvisual','sfo-real.gxworld','manifest.json'){
  $src=Join-Path $Runtime $f; Need (Test-Path -LiteralPath $src) "runtime asset missing: $src"
  Copy-Item -LiteralPath $src -Destination (Join-Path $assets $f) -Force
}
foreach($f in 'b787-manifest.json','buildings-manifest.json','FlightGear-787-8-COPYING','FlightGear-787-8-Authors.txt'){
  $src=Join-Path $Runtime $f
  if(Test-Path -LiteralPath $src){Copy-Item -LiteralPath $src -Destination (Join-Path $assets $f) -Force}
}
[IO.File]::WriteAllText((Join-Path $stage 'aircraft'),"$Aircraft`n",[Text.UTF8Encoding]::new($false))
foreach($s in 'spike002_smoke.sh','spike002_sampler.sh','spike002_bwtrace.sh'){
  $text=[IO.File]::ReadAllText((Join-Path $PSScriptRoot $s)).Replace("`r`n","`n")
  [IO.File]::WriteAllText((Join-Path $stage $s),$text,[Text.UTF8Encoding]::new($false))
}
$hash={param($p)(Get-FileHash -Algorithm SHA256 -LiteralPath $p).Hash.ToLowerInvariant()}
$m=Get-Content -Raw -LiteralPath "$stage\build_manifest.m1-s4.json"|ConvertFrom-Json
$m|Add-Member -NotePropertyName spike002 -NotePropertyValue ([ordered]@{
  gxvisual_sha256=(& $hash "$assets\sfo-photoreal.gxvisual"); gxvisual_bytes=(Get-Item "$assets\sfo-photoreal.gxvisual").Length
  gxworld_sha256=(& $hash "$assets\sfo-real.gxworld"); gxworld_bytes=(Get-Item "$assets\sfo-real.gxworld").Length
  runtime_manifest_sha256=(& $hash "$assets\manifest.json")
  runner='m1_s4_run.sh adds --world spike002/sfo-real.gxworld --visual spike002/sfo-photoreal.gxvisual when present'
  aircraft=$(if($Aircraft -eq '787-8'){'787-8: FlightGear FGAddon 787-8 (BBJ livery, GXVISUAL 1.2 joints, tools/b787_pack.py), GPL-2.0'}else{"${Aircraft}: caller-supplied visual asset; retain its license and notices"})
}) -Force
$m|ConvertTo-Json -Depth 6|Set-Content -LiteralPath "$stage\build_manifest.m1-s4.json" -Encoding utf8
Remove-Item -LiteralPath "$stage\files.sha256" -Force
$lines=foreach($f in Get-ChildItem -LiteralPath $stage -Recurse -File|Where-Object{$_.Name -ne 'files.sha256'}|Sort-Object FullName){
  "$(& $hash $f.FullName)  $($f.FullName.Substring($stage.Length+1).Replace('\','/'))"
}
[IO.File]::WriteAllText("$stage\files.sha256",(($lines -join "`n")+"`n"),[Text.UTF8Encoding]::new($false))
Write-Host "SPIKE-002 stage ready: $stage (commit $($m.source_commit), dirty=$($m.source_dirty))"
