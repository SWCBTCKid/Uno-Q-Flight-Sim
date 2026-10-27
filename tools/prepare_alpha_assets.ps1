[CmdletBinding()]
param(
 [Parameter(Mandatory)][string]$Astcenc,
 [string]$Aircraft787='',
 [string]$Acquisition='',
 [string]$Output=''
)
$ErrorActionPreference='Stop'
$repo=(Resolve-Path "$PSScriptRoot\..").Path
function Need($ok,$msg){if(!$ok){throw $msg}}
if(!$Acquisition){$Acquisition=Join-Path $repo 'build\spike-002-assets'}
if(!$Output){$Output=Join-Path $repo 'build\alpha-runtime'}
$base=Join-Path $repo 'build\alpha-runtime-base'
$b787=Join-Path $repo 'build\alpha-runtime-787'
$lidar=Join-Path $repo 'build\spike-002-lidar\sfo-lidar.npz'
$manifest=Join-Path $repo 'assets\spike-002\sfo-sources.v1.json'
Need (Test-Path -LiteralPath $Astcenc -PathType Leaf) "astcenc not found: $Astcenc"

if(!$Aircraft787){
  $Aircraft787=(& py -I "$PSScriptRoot\fetch_b787.py" --cache (Join-Path $repo 'asset-cache')).Trim()
  Need (!$LASTEXITCODE) "FlightGear 787 acquisition failed: $LASTEXITCODE"
}
Need (Test-Path -LiteralPath (Join-Path $Aircraft787 'Models\787-8.ac')) "787 source root is invalid: $Aircraft787"

& py -I "$PSScriptRoot\spike002_assets.py" validate --manifest $manifest --output $Acquisition
Need (!$LASTEXITCODE) "SFO source-manifest validation failed: $LASTEXITCODE"
& py -I "$PSScriptRoot\spike002_assets.py" acquire --manifest $manifest --output $Acquisition
Need (!$LASTEXITCODE) "SFO acquisition failed: $LASTEXITCODE"
& py -I "$PSScriptRoot\spike002_convert.py" --acquisition $Acquisition --manifest $manifest --output $base --astcenc $Astcenc
Need (!$LASTEXITCODE) "SFO conversion failed: $LASTEXITCODE"
& py -I "$PSScriptRoot\b787_pack.py" --base $base --aircraft $Aircraft787 --output $b787 --astcenc $Astcenc
Need (!$LASTEXITCODE) "787 conversion failed: $LASTEXITCODE"
Copy-Item -LiteralPath (Join-Path $base 'manifest.json') -Destination $b787 -Force

& py -I "$PSScriptRoot\spike002_lidar.py" --bbox -122.405 37.598 -122.340 37.636 --output $lidar
Need (!$LASTEXITCODE) "NOAA lidar acquisition failed: $LASTEXITCODE"
& py -I "$PSScriptRoot\spike002_buildings.py" --pack $b787 --acquisition $Acquisition --lidar $lidar --output $Output
Need (!$LASTEXITCODE) "SFO building conversion failed: $LASTEXITCODE"

foreach($f in 'manifest.json','sfo-real.world.txt'){
  Copy-Item -LiteralPath (Join-Path $base $f) -Destination $Output -Force
}
foreach($f in 'b787-manifest.json','livery-bbj.png'){
  Copy-Item -LiteralPath (Join-Path $b787 $f) -Destination $Output -Force
}
Copy-Item -LiteralPath (Join-Path $Aircraft787 'COPYING') -Destination (Join-Path $Output 'FlightGear-787-8-COPYING') -Force
Copy-Item -LiteralPath (Join-Path $Aircraft787 'Authors.txt') -Destination (Join-Path $Output 'FlightGear-787-8-Authors.txt') -Force

& cargo run --locked --manifest-path (Join-Path $repo 'rust\Cargo.toml') --package gx-app -- spike002-world (Join-Path $Output 'sfo-real.world.txt') (Join-Path $Output 'sfo-real.gxworld')
Need (!$LASTEXITCODE) "GXWORLD compilation failed: $LASTEXITCODE"
Write-Host "Full 787/SFO alpha runtime ready: $Output"
