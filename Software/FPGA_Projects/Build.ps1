param([ValidateSet('video','tracking','both')][string]$Image='both',
 [string]$TDHome='C:\Anlogic\TD_6.2.1_Engineer_6.2.168.116')
$ErrorActionPreference='Stop'
$demo=$PSScriptRoot
New-Item -ItemType Directory -Force -Path (Join-Path $demo 'Rebuilt') | Out-Null
$td=Join-Path $TDHome 'bin/td_commands_prompt.exe'
$flow=(Join-Path $TDHome 'doc/scripts/DefaultFlow.tcl').Replace('\','/')
if (!(Test-Path -LiteralPath $td)) {throw "TD not found: $td"}
$images=if($Image -eq 'both') {@('video','tracking')} else {@($Image)}
foreach($name in $images) {
 $lab=Join-Path $demo $(if($name -eq 'video'){'Video_D1'}else{'Tracking_D2'})
 $inputs=Get-ChildItem -LiteralPath (Join-Path $lab 'user_source') -File -Recurse | Get-FileHash -Algorithm SHA256
 foreach($step in @(@('syn_1','camera_to_dsi_display_gate.db'),@('phy_1','camera_to_dsi_display.bit'))) {
  $dir=Join-Path $lab ('td_project/camera_to_dsi_display_Runs/'+$step[0])
  $started=Get-Date
  Write-Output "$name $($step[0]) started $started"
  Push-Location $dir
  try {
   & $td $flow *> build.log
   $code=$LASTEXITCODE
   $file=Get-Item -LiteralPath $step[1] -ErrorAction SilentlyContinue
   if($code -ne 0 -or (Select-String -Path build.log -Pattern '\bERROR:') -or !$file -or $file.LastWriteTime -lt $started) {
    throw "Build failed: $dir (exit $code); see build.log"
   }
  } finally {Pop-Location}
 }
 $qor=Join-Path $lab 'td_project/camera_to_dsi_display_Runs/phy_1/route.qor'
 if (!(Test-Path -LiteralPath $qor)) {throw 'Missing route.qor'}
 foreach($line in Get-Content -LiteralPath $qor) {
  if($line -match 'Num of violated [SH]-EP\s*\|\s*([1-9]\d*)') {throw "Timing violation: $line"}
 }
 # Explicitly enable the chip's Multi Boot gate; the RTL primitive alone is insufficient.
 $bitgen=(Join-Path $PSScriptRoot 'bitgen_multiboot.tcl').Replace('\','/')
 $started=Get-Date
 Push-Location $dir
 try {
  & $td $bitgen *> bitgen_multiboot.log
  if($LASTEXITCODE -ne 0 -or (Select-String -Path bitgen_multiboot.log -Pattern '\bERROR:') -or
     (Get-Item -LiteralPath 'camera_to_dsi_display.bit').LastWriteTime -lt $started) {
   throw "Multi Boot bitgen failed: $dir; see bitgen_multiboot.log"
  }
 } finally {Pop-Location}
 foreach($inputFile in $inputs) {
  if((Get-FileHash -LiteralPath $inputFile.Path -Algorithm SHA256).Hash -ne $inputFile.Hash){throw "Source changed during build: $($inputFile.Path)"}
 }
 $inputs | Select-Object Path,Hash | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $demo "Rebuilt/${name}_source_manifest.json") -Encoding UTF8
 $dest=Join-Path $demo "Rebuilt/demo_$name.bit"
 Copy-Item -LiteralPath (Join-Path $lab 'td_project/camera_to_dsi_display_Runs/phy_1/camera_to_dsi_display.bit') -Destination $dest
 $hash=(Get-FileHash -LiteralPath $dest -Algorithm SHA256).Hash
 @("Image: $name; Demo ID: $(if($name -eq 'video'){'D1'}else{'D2'})", "Generated: $(Get-Date -Format o)", "SHA256: $hash", 'Multi Boot: mult_boot_en=1', 'Explicit home only; no motor or laser auto-start.', '', (Get-Content -LiteralPath $qor)) | Set-Content -LiteralPath (Join-Path $demo "Rebuilt/${name}_build.txt") -Encoding UTF8
 Write-Output "Complete: $dest"
}
