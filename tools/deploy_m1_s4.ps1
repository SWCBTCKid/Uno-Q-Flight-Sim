[CmdletBinding()]
param(
 [Parameter(Mandatory)][ValidatePattern('^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$')][string]$RunId,
 [Parameter(Mandatory)][string]$ArtifactRoot,
 [string]$Board=$env:GX_BOARD,
 [string]$StageDir='',
 # Zero means fly until double-click EXIT; the launcher retains a 3 h backstop.
 [ValidateRange(0,10800)][int]$Seconds=0,
 [switch]$SkipTests
)
$ErrorActionPreference='Stop'
# S4: file-copy deployment and unprivileged board tests only. This script never
# invokes sudo, stops LightDM, opens a KMS window or takes over the display.
$repo=(Resolve-Path "$PSScriptRoot\..").Path
if(!$StageDir){$StageDir=Join-Path $repo 'build\m1-s4\stage'}
function Need($ok,$msg){if(!$ok){throw $msg}}
Need ([bool]$Board) 'Board is required: pass -Board user@hostname or set GX_BOARD'
. "$PSScriptRoot\m1_s1_remote.ps1"
Need ($Board -match '^[A-Za-z0-9._-]+@[A-Za-z0-9._:-]+$') "Unsafe board login: $Board"
Need (Test-Path -LiteralPath "$StageDir\build_manifest.m1-s4.json") "S4 stage missing: $StageDir"
Need (!(Test-Path -LiteralPath $ArtifactRoot)) "ArtifactRoot already exists: $ArtifactRoot"
$null=New-Item -ItemType Directory -Path $ArtifactRoot
$stage=(Resolve-Path -LiteralPath $StageDir).Path
$manifest=Get-Content -Raw -LiteralPath "$stage\build_manifest.m1-s4.json"|ConvertFrom-Json
Need (!$manifest.source_dirty) 'Refusing to deploy a dirty-source build'
foreach($line in [IO.File]::ReadAllLines("$stage\files.sha256")){
  if(!$line){continue}; $hash,$rel=$line -split '  ',2; $path=Join-Path $stage ($rel.Replace('/','\'))
  Need ((Test-Path -LiteralPath $path) -and (Get-FileHash -Algorithm SHA256 -LiteralPath $path).Hash.ToLowerInvariant() -eq $hash) "Stage hash mismatch: $rel"
}
$remote="/home/arduino/global-express-m0/m1/s4/deploy/$RunId"
$runRemote="/home/arduino/global-express-m0/m1/s4/runs/$RunId"
$device='/dev/input/by-id/usb-Logitech_Logitech_Extreme_3D-event-joystick'
$rc=Invoke-M1S1RemoteScript $Board "$PSScriptRoot\m1_s1_remote_prepare.sh" @($remote)
Need ($rc -eq 0) "Remote bundle exists, stale gx-app is running, or mkdir failed: $remote"
foreach($f in Get-ChildItem -LiteralPath $stage -File){
  & scp -q $f.FullName "${Board}:$remote/$($f.Name)"; Need (!$LASTEXITCODE) "scp failed: $($f.Name)"
}
foreach($d in 'bin','tests','jsbsim','spike002'){
  if(!(Test-Path -LiteralPath (Join-Path $stage $d))){Need ($d -eq 'spike002') "stage dir missing: $d"; continue}
  & scp -q -r (Join-Path $stage $d) "${Board}:$remote/"; Need (!$LASTEXITCODE) "scp failed: $d"
}
$rc=Invoke-M1S1RemoteScript $Board "$PSScriptRoot\m1_s4_remote_verify.sh" @($remote,$device,$manifest.visible_controller.sha256)
Need ($rc -eq 0) 'Remote hash, CLI, input, DRM permission or realtime (candidate-unit envelope) preflight failed'
if(!$SkipTests){
  $rc=Invoke-M1S1RemoteScript $Board "$PSScriptRoot\m1_s4_board_tests.sh" @($remote)
  Need ($rc -eq 0) 'Board tests failed'
}
foreach($f in 'files.sha256','elf_report.json','build_manifest.m1-s4.json'){Copy-Item -LiteralPath "$stage\$f" -Destination $ArtifactRoot}
$statusCommand="ssh $Board $remote/m1_s4_launch.sh status"
$runCommand=$(if($Seconds){"ssh -t $Board sudo $remote/m1_s4_launch.sh run $RunId $Seconds"}else{"ssh -t $Board sudo $remote/m1_s4_launch.sh run $RunId"})
$relinkCommand="ssh $Board $remote/m1_s4_launch.sh relink"
$restoreCommand="ssh -t $Board sudo $remote/m1_s4_launch.sh restore"
$abortCommand="ssh -t $Board sudo systemctl kill -s TERM m1-s4-window"
$deadmanCommand="ssh -t $Board sudo $remote/m1_s4_launch.sh stop-deadman"
$smokeCommand="ssh $Board sh $remote/m1_s4_smoke.sh $remote 60"
[ordered]@{
  schema='gx-s4-deploy/1';run_id=$RunId;board=$Board;remote_path=$remote;run_remote_path=$runRemote
  device=$device;source_commit=$manifest.source_commit;gx_app_sha256=$manifest.gx_app.sha256
  controller_sha256=$manifest.visible_controller.sha256;seconds=$(if($Seconds){$Seconds}else{$null})
  status_command=$statusCommand;run_command=$runCommand;relink_command=$relinkCommand
  restore_command=$restoreCommand;abort_command=$abortCommand;deadman_command=$deadmanCommand;smoke_command=$smokeCommand
  deployed_local=(Get-Date).ToString('o')
}|ConvertTo-Json|Set-Content -LiteralPath "$ArtifactRoot\deployment.json" -Encoding utf8
Write-Host 'S4 deploy, non-disruptive preflight and board tests passed. The display was not taken over.'
Write-Host "CHECK DISPLAY READINESS:`n$statusCommand"
Write-Host "CHIEF VISIBLE RUN (sudo + physical observation):`n$runCommand"
Write-Host "IF THE MONITOR STAYS BLANK AFTER THE DESKTOP IS BACK (no sudo):`n$relinkCommand"
Write-Host "MANUAL RESTORE (R-5/R-6):`n$restoreCommand"
Write-Host "ABORT EARLY:`n$abortCommand"
Write-Host "COLLECT AFTER RESTORE:`n& '$PSScriptRoot\collect_m1_s4.ps1' -ArtifactRoot '$ArtifactRoot'"
