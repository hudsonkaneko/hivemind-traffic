$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$source = Join-Path $root 'runtimes\IsaacSim7'
if (-not (Test-Path (Join-Path $source 'source\libraries\build.bat'))) {
    throw 'Isaac Sim v7.0.0a1 source checkout is missing from runtimes\IsaacSim7.'
}
Push-Location $source
try {
    & .\source\libraries\build.bat -r --wheel --output-dir _build/libraries/wheels 2>&1 |
        Tee-Object -FilePath (Join-Path $root 'logs\isaacsim7-build.log')
    if ($LASTEXITCODE -ne 0) { throw "Isaac Sim library build failed: $LASTEXITCODE" }
} finally {
    Pop-Location
}
