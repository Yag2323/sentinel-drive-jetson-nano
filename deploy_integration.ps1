param(
    [string]$HostName = "jetson-nano.local",
    [string]$UserName = "jetson",
    [string]$RemoteDirectory = "/home/jetson/ai_robot_car",
    [switch]$SafetyConfirmed
)

if (-not $SafetyConfirmed) {
    Write-Error "Deployment refused. First stop every Jetson dashboard/autonomy/motor process and physically disconnect the motor battery, then rerun with -SafetyConfirmed."
    exit 1
}

if ($RemoteDirectory -notmatch '^/[A-Za-z0-9._/-]+$') {
    Write-Error "Deployment refused: RemoteDirectory contains unsupported characters."
    exit 1
}
if ($UserName -notmatch '^[A-Za-z0-9._-]+$' -or $HostName -notmatch '^[A-Za-z0-9._:-]+$') {
    Write-Error "Deployment refused: UserName or HostName contains unsupported characters."
    exit 1
}

$files = @(
    "robot_dashboard.py",
    "patch_dashboard_camera_rotation.py",
    "gst_camera_bridge.py",
    "source_integrity.py",
    "prepare_yolov5_v6.py",
    "jetson_perception_preflight.py",
    "yolov5_runtime.py",
    "yolo_csi_benchmark.py",
    "ipm_lane.py",
    "single_line_lane.py",
    "calibrate_ipm.py",
    "install_single_line_profile.py",
    "test_ipm_lane_synthetic.py",
    "test_single_line_synthetic.py",
    "ipm_alignment_diagnostic.py",
    "ipm_live_dry_run.py",
    "outer_circle_position_validation.py",
    "control_core.py",
    "motor_mapping.py",
    "manual_motor_control.py",
    "safe_motor_output.py",
    "integration_self_test.py",
    "integrated_control_dry_run.py",
    "motor_adapter_raised_test.py",
    "steering_floor_validation.py",
    "validation_manager.py",
    "track_run.py",
    "guarded_live_circle_run.py",
    "guarded_straight_line_run.py",
    "analyse_track_run.py",
    "promote_controller_tuning.py",
    "make_build_manifest.py",
    "install_integration_bundle.py",
    "install_motor_calibration.py",
    "controller_config.seed.json",
    "ipm_config.seed.json",
    "integration_gates.seed.json"
) | ForEach-Object { Join-Path $PSScriptRoot $_ }

$missing = @($files | Where-Object { -not (Test-Path -LiteralPath $_ -PathType Leaf) })
if ($missing.Count -gt 0) {
    Write-Error ("Deployment stopped; missing files:`n" + ($missing -join "`n"))
    exit 1
}

$target = "${UserName}@${HostName}:${RemoteDirectory}/"
$remoteLogin = "${UserName}@${HostName}"
$backupStamp = (Get-Date).ToUniversalTime().ToString("yyyyMMddTHHmmssZ")
$dashboardBackup = "${RemoteDirectory}/robot_dashboard.pre_deploy_${backupStamp}.py"
$backupCommand = "if [ -f '${RemoteDirectory}/robot_dashboard.py' ]; then cp -p '${RemoteDirectory}/robot_dashboard.py' '${dashboardBackup}'; fi"
Write-Host "Transferring integration package to $target"
Write-Host "Safety confirmation received: project processes stopped and motor battery physically disconnected."
& ssh $remoteLogin $backupCommand
if ($LASTEXITCODE -ne 0) {
    Write-Error "Deployment stopped because the remote dashboard backup failed."
    exit $LASTEXITCODE
}
Write-Host "Remote dashboard recovery copy: $dashboardBackup"
& scp @files $target
if ($LASTEXITCODE -ne 0) {
    Write-Error "scp failed with exit code $LASTEXITCODE"
    exit $LASTEXITCODE
}


$localDashboard = Join-Path $PSScriptRoot "robot_dashboard.py"
$localDashboardHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $localDashboard).Hash.ToLowerInvariant()
$remoteHashOutput = & ssh $remoteLogin "sha256sum '${RemoteDirectory}/robot_dashboard.py'"
if ($LASTEXITCODE -ne 0) {
    Write-Error "Transfer completed, but remote dashboard hash verification failed. Recovery copy: $dashboardBackup"
    exit $LASTEXITCODE
}
$remoteDashboardHash = (($remoteHashOutput | Select-Object -First 1) -split '\s+')[0].ToLowerInvariant()
if ($remoteDashboardHash -ne $localDashboardHash) {
    Write-Error "Dashboard hash mismatch after transfer. Recovery copy: $dashboardBackup"
    exit 1
}
Write-Host "Dashboard SHA-256 verified: $remoteDashboardHash"

Write-Host "Transfer complete. Existing motor/IPM configs were not sent."
Write-Host "The single-line profile installer preserves the current homography points but invalidates calibration, controller tuning, and gate evidence because detector semantics changed."
Write-Host "On the Jetson, run:"
Write-Host "  cd ~/ai_robot_car"
Write-Host "  source venv_jetson/bin/activate"
Write-Host "  python install_motor_calibration.py"
Write-Host "  python install_integration_bundle.py"
Write-Host "  python install_single_line_profile.py --dry-run"
Write-Host "  python install_single_line_profile.py"
Write-Host "  python prepare_yolov5_v6.py"
Write-Host "  python test_single_line_synthetic.py"
Write-Host "  python validation_manager.py record-self-test"
Write-Host "  python validation_manager.py show"
Write-Host "  # After calibration and the centred IPM dry-run gate:"
Write-Host "  python outer_circle_position_validation.py --seconds-per-position 5"
Write-Host "  `$OUTER_EVIDENCE=`$(ls -1t evidence/outer_circle_positions_*.json | head -n 1)"
Write-Host "  python validation_manager.py record outer_circle_positions `"`$OUTER_EVIDENCE`""
Write-Host "  python make_build_manifest.py"
