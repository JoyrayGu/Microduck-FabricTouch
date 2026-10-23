$ErrorActionPreference = 'Stop'

$local = 'E:\OneDrive\MatNgine\microduck_brain'
$remote = 'pi4@192.168.1.57'
$key = Join-Path $env:USERPROFILE '.ssh\microduck_ed25519'
$dest = '/home/pi4/microduck_brain'

# Files brain actually needs at runtime. Tests and tools are excluded.
$files = @(
    'microduck/src/action_dispatcher.py',
    'microduck/src/action_library.py',
    'microduck/src/command_schema.py',
    'microduck/src/command_server.py',
    'microduck/src/constants.py',
    'microduck/src/servo_runtime.py'
)

$sshArgs = @('-o','BatchMode=yes','-o','ConnectTimeout=15','-i',$key)

# Create remote directory structure first, plus namespace-package markers so
# `python -m microduck.src.command_server` resolves on any Python 3 version.
& ssh @sshArgs $remote "mkdir -p $dest/microduck/src $dest/tools/deployment_validation && touch $dest/microduck/__init__.py $dest/microduck/src/__init__.py"

foreach ($f in $files) {
    $src = Join-Path $local ($f -replace '/','\')
    if (-not (Test-Path $src)) {
        Write-Host "SKIP (missing locally): $f"
        continue
    }
    $remotePath = "$dest/$f"
    & scp @sshArgs $src "${remote}:$remotePath"
    if ($LASTEXITCODE -ne 0) { throw "scp failed for $f" }
    Write-Host "UPLOADED $f"
}

# Pose files uploaded for compatibility with the legacy Pi tools
# (pose_loop.py / microduck_gui.py still read tilt_head_pose.json).
# NOTE: brain itself no longer reads tilt_head_pose.json - tilt_head/head_wag use
# hard-coded calibrated constants (HEAD_ROLL_* / HEAD_YAW_*). The values inside
# that file (ID13=1736, ID14=2048) are STALE and must not be replayed as-is.
$poses = @(
    'tools/deployment_validation/sit_pose.json',
    'tools/deployment_validation/tilt_head_pose.json'
)
foreach ($p in $poses) {
    $src = Join-Path $local ($p -replace '/','\')
    if (-not (Test-Path $src)) {
        Write-Host "SKIP (missing locally): $p"
        continue
    }
    & scp @sshArgs $src "${remote}:$dest/$p"
    if ($LASTEXITCODE -ne 0) { throw "scp failed for $p" }
    Write-Host "UPLOADED $p"
}

Write-Host "`nDone. Verifying remote tree:"
& ssh @sshArgs $remote "find $dest -type f | sort"
