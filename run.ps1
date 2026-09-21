param(
    [ValidateSet('smoke', 'vehicle', 'lane', 'train', 'evaluate')]
    [string]$Mode = 'train',
    [int]$Steps = 32768,
    [int]$NumEnvs = 16,
    [string]$Checkpoint = '',
    [int]$Seed = 42,
    [string]$Output = 'outputs/waypoint',
    [switch]$HoldOpen,
    [switch]$Gui
)
$ErrorActionPreference = 'Stop'
$simPath = 'C:\isaacsim'
$labPath = Join-Path $env:USERPROFILE 'Documents\IsaacLab'
Push-Location $PSScriptRoot
try {
    if ($Mode -eq 'vehicle') {
        $vehicleArgs = @('scripts\smoke_vehicle.py')
        if ($Gui) { $vehicleArgs += '--gui' }
        & "$simPath\python.bat" @vehicleArgs
    } elseif ($Mode -eq 'lane') {
        $laneArgs = @('scripts\follow_circular_lane.py', '--steps', "$Steps")
        if ($Gui) { $laneArgs += '--gui' }
        if ($HoldOpen) { $laneArgs += '--hold-open' }
        & "$simPath\python.bat" @laneArgs
    } elseif ($Mode -eq 'smoke') {
        & "$simPath\python.bat" 'scripts\smoke_sim.py'
    } else {
        $trainArgs = @('-p', 'scripts\train_waypoint.py', '--mode', $Mode,
                       '--steps', "$Steps", '--num_envs', "$NumEnvs",
                       '--seed', "$Seed", '--output', $Output)
        if ($Checkpoint) { $trainArgs += @('--checkpoint', $Checkpoint) }
        if ($Gui) { $trainArgs += @('--viz', 'kit') }
        if ($HoldOpen) { $trainArgs += '--hold_open' }
        & "$labPath\isaaclab.bat" @trainArgs
    }
    if ($LASTEXITCODE -ne 0) { throw "Simulation exited with code $LASTEXITCODE" }
} finally {
    Pop-Location
}
