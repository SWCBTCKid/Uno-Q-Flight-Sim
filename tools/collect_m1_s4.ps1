[CmdletBinding()]
param([Parameter(Mandatory)][string]$ArtifactRoot)
$ErrorActionPreference='Stop'
# After a visible S4 window: refuse while it is active or the desktop restore
# is unverified, then pull every available evidence file with hash checks.
function Need($ok,$msg){if(!$ok){throw $msg}}
. "$PSScriptRoot\m1_s1_remote.ps1"
$receipt=Join-Path $ArtifactRoot 'deployment.json'
Need (Test-Path -LiteralPath $receipt) "Missing deployment.json: $ArtifactRoot"
$d=Get-Content -Raw -LiteralPath $receipt|ConvertFrom-Json
Need ($d.board -match '^[A-Za-z0-9._-]+@[A-Za-z0-9._:-]+$') 'Unsafe board in receipt'
Need ($d.remote_path -match '^/home/arduino/global-express-m0/m1/s4/deploy/[A-Za-z0-9._-]+$') 'Unsafe bundle path'
Need ($d.run_remote_path -match '^/home/arduino/global-express-m0/m1/s4/runs/[A-Za-z0-9._-]+$') 'Unsafe run path'
$rc=Invoke-M1S1RemoteScript $d.board "$PSScriptRoot\m1_s4_remote_collectable.sh" @($d.remote_path,$d.run_remote_path)
Need ($rc -eq 0) 'Run is incomplete, still active, restore is unverified, or evidence is absent'
$files='s4-run.jsonl','sampler.jsonl','bwmon-trace.txt','bwmon-meta.txt','rtprobe.json','runctl.log','state-before.txt','state-after.txt','candidate.out','candidate.err','card0-holders.txt','relink.log','deadman.log','stream-ladder.jsonl','stream-ladder.stdout','stream-ladder.stderr','stream-ladder.pid','stream-ladder.exit','stream-ladder.ready','stream-ladder.go'
$buildManifestPath=Join-Path $ArtifactRoot 'build_manifest.m1-s4.json'
$isSpike002=$false
if(Test-Path -LiteralPath $buildManifestPath){
  $buildManifest=Get-Content -Raw -LiteralPath $buildManifestPath|ConvertFrom-Json
  $isSpike002=($null -ne $buildManifest.spike002)
}
$optional='candidate.err','card0-holders.txt','relink.log','deadman.log','stream-ladder.jsonl','stream-ladder.stdout','stream-ladder.stderr','stream-ladder.pid','stream-ladder.exit','stream-ladder.ready','stream-ladder.go'
if(!$isSpike002){$optional+=@('sampler.jsonl','bwmon-trace.txt','bwmon-meta.txt')}
foreach($f in $files){
  $dst=Join-Path $ArtifactRoot $f
  # Windows PowerShell promotes native stderr to an ErrorRecord.  Keep a
  # missing optional remote file non-terminating so its exit code can be
  # handled by the policy below instead of aborting under ErrorAction=Stop.
  $savedErrorActionPreference=$ErrorActionPreference
  try {
    $ErrorActionPreference='Continue'
    & scp -q "$($d.board):$($d.run_remote_path)/$f" $dst 2>$null
    $scpExit=$LASTEXITCODE
  } finally {
    $ErrorActionPreference=$savedErrorActionPreference
  }
  if($scpExit){if($f -in $optional){continue};throw "Evidence pull failed: $f"}
  $remoteHash=(& ssh $d.board sha256sum "$($d.run_remote_path)/$f").Trim().Split()[0]
  Need ($remoteHash -eq (Get-FileHash -Algorithm SHA256 -LiteralPath $dst).Hash.ToLowerInvariant()) "Pulled file differs: $f"
  $remoteHash|Set-Content -LiteralPath "$dst.sha256" -Encoding ascii
}
Write-Host "Raw S4 logs and platform state collected: $ArtifactRoot"
