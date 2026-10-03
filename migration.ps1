param([ValidateSet('physics','render','vehicle')][string]$Check = 'physics')
$ErrorActionPreference = 'Stop'
$python = Join-Path $PSScriptRoot '.venv-isaac7\Scripts\python.exe'
if (-not (Test-Path $python)) { throw 'The experimental Isaac Sim 7 environment is missing.' }
Push-Location $PSScriptRoot
try {
    switch ($Check) {
        'physics' { & $python -u scripts/smoke_isaac7.py }
        'render' { & $python -u scripts/smoke_ovrtx.py --with-physics }
        'vehicle' { & $python -u scripts/smoke_vehicle_isaac7.py }
    }
    if ($LASTEXITCODE -ne 0) { throw "Migration check '$Check' failed (exit $LASTEXITCODE). See documentation/migration-2026-09-21.md." }
} finally { Pop-Location }
