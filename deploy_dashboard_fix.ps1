param(
    [string]$HostName = "jetson-nano.local",
    [string]$UserName = "jetson",
    [string]$RemoteDirectory = "/home/jetson/ai_robot_car",
    [switch]$SafetyConfirmed
)

if (-not $SafetyConfirmed) {
    Write-Error "Update refused. Stop every CSI/dashboard process and physically disconnect the motor battery, then rerun with -SafetyConfirmed."
    exit 1
}
if ($RemoteDirectory -notmatch '^/[A-Za-z0-9._/-]+$') {
    Write-Error "Update refused: RemoteDirectory contains unsupported characters."
    exit 1
}
if ($UserName -notmatch '^[A-Za-z0-9._-]+$' -or $HostName -notmatch '^[A-Za-z0-9._:-]+$') {
    Write-Error "Update refused: UserName or HostName contains unsupported characters."
    exit 1
}

$files = @(
    "robot_dashboard.py",
    "patch_dashboard_camera_rotation.py",
    "gst_camera_bridge.py",
    "make_build_manifest.py"
) | ForEach-Object { Join-Path $PSScriptRoot $_ }

$missing = @($files | Where-Object { -not (Test-Path -LiteralPath $_ -PathType Leaf) })
if ($missing.Count -gt 0) {
    Write-Error ("Update stopped; missing files:`n" + ($missing -join "`n"))
    exit 1
}

$remoteLogin = "${UserName}@${HostName}"
$target = "${remoteLogin}:${RemoteDirectory}/"
$backupStamp = (Get-Date).ToUniversalTime().ToString("yyyyMMddTHHmmssZ")
$dashboardBackup = "${RemoteDirectory}/robot_dashboard.pre_camera_fix_${backupStamp}.py"
$backupCommand = "if [ -f '${RemoteDirectory}/robot_dashboard.py' ]; then cp -p '${RemoteDirectory}/robot_dashboard.py' '${dashboardBackup}'; fi"

Write-Host "Creating remote dashboard recovery copy."
& ssh $remoteLogin $backupCommand
if ($LASTEXITCODE -ne 0) {
    Write-Error "Update stopped because the remote dashboard backup failed."
    exit $LASTEXITCODE
}
Write-Host "Recovery copy: $dashboardBackup"

Write-Host "Transferring the focused dashboard fix to $target"
& scp @files $target
if ($LASTEXITCODE -ne 0) {
    Write-Error "scp failed with exit code $LASTEXITCODE. The remote recovery copy was retained."
    exit $LASTEXITCODE
}

$localDashboardHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $files[0]).Hash.ToLowerInvariant()
$remoteHashOutput = & ssh $remoteLogin "sha256sum '${RemoteDirectory}/robot_dashboard.py'"
if ($LASTEXITCODE -ne 0) {
    Write-Error "Files transferred, but remote hash verification failed. Recovery copy: $dashboardBackup"
    exit $LASTEXITCODE
}
$remoteDashboardHash = (($remoteHashOutput | Select-Object -First 1) -split '\s+')[0].ToLowerInvariant()
if ($remoteDashboardHash -ne $localDashboardHash) {
    Write-Error "Dashboard hash mismatch after transfer. Recovery copy: $dashboardBackup"
    exit 1
}

Write-Host "Dashboard SHA-256 verified: $remoteDashboardHash"
Write-Host "Focused update complete. Do not rerun the configuration installers."
Write-Host "On the Jetson, run:"
Write-Host "  cd ~/ai_robot_car"
Write-Host "  source venv_jetson/bin/activate"
Write-Host "  python -m py_compile robot_dashboard.py patch_dashboard_camera_rotation.py gst_camera_bridge.py make_build_manifest.py"
Write-Host "  python patch_dashboard_camera_rotation.py robot_dashboard.py"
Write-Host "  python make_build_manifest.py"
Write-Host "  python validation_manager.py show"
