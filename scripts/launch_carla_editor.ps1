param(
  [string]$CarlaRoot = "D:\carla0.9.15",
  [string]$VsDevCmd = "",
  [string]$MapName = "",
  [switch]$CommandPlay,
  [switch]$Visible
)

$ErrorActionPreference = "Stop"

if (!(Test-Path -LiteralPath $CarlaRoot)) {
  throw "Cannot locate CARLA root: $CarlaRoot"
}

if ([string]::IsNullOrWhiteSpace($VsDevCmd)) {
  $candidates = @(
    "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat",
    "C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\Common7\Tools\VsDevCmd.bat",
    "C:\Program Files (x86)\Microsoft Visual Studio\2019\Enterprise\Common7\Tools\VsDevCmd.bat",
    "D:\VS2019IDE\Common7\Tools\VsDevCmd.bat"
  )
  foreach ($candidate in $candidates) {
    if (Test-Path -LiteralPath $candidate) {
      $VsDevCmd = $candidate
      break
    }
  }
}

if ([string]::IsNullOrWhiteSpace($VsDevCmd) -or !(Test-Path -LiteralPath $VsDevCmd)) {
  throw "Cannot locate VS2019 VsDevCmd.bat. Pass -VsDevCmd '<path>\VsDevCmd.bat'."
}

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$launchLog = Join-Path $env:TEMP ("roadtailbench_launch_carla_{0}.log" -f $stamp)

$commandPlayLines = ""
if ($CommandPlay) {
  if ([string]::IsNullOrWhiteSpace($MapName)) {
    throw "CommandPlay requires -MapName."
  }
  $commandPlayLines = @"
set ROADTAILBENCH_UE_MAP=$MapName
set ROADTAILBENCH_COMMAND_PLAY=1
set LEADERBOARD_UE_MAP=$MapName
set LEADERBOARD_COMMAND_PLAY=1
set PYTHONPATH=$repoRoot;%PYTHONPATH%
"@
}

$cmd = @"
call "$VsDevCmd" -arch=amd64 -host_arch=amd64
cd /d "$CarlaRoot"
$commandPlayLines
make launch-only > "$launchLog" 2>&1
"@

$temp = Join-Path $env:TEMP ("roadtailbench_launch_carla_{0}.cmd" -f $stamp)
Set-Content -LiteralPath $temp -Encoding ASCII -Value $cmd

$process = Start-Process `
  -FilePath "cmd.exe" `
  -ArgumentList "/c `"$temp`"" `
  -WorkingDirectory $CarlaRoot `
  -WindowStyle $(if ($Visible) { "Normal" } else { "Hidden" }) `
  -PassThru

"MAKE_LAUNCH_MODE=VS2019_X64_NATIVE_TOOLS_CMD"
"CMD_SCRIPT=$temp"
"CMD_PID=$($process.Id)"
"LAUNCH_LOG=$launchLog"
"CARLA_ROOT=$CarlaRoot"
"VSDEVCMD=$VsDevCmd"
if ($CommandPlay) {
  "COMMAND_PLAY=1"
  "MAP=$MapName"
  "PYTHONPATH_ADD=$repoRoot"
}
