$ErrorActionPreference = 'Stop'
$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location -LiteralPath $ProjectRoot

function Test-PythonVersion([string]$Executable, [string[]]$PrefixArgs) {
    try {
        $versionText = (& $Executable @PrefixArgs -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')" 2>$null | Select-Object -First 1)
        if (-not $versionText) { return $false }
        $parts = $versionText.Trim().Split('.')
        $major = [int]$parts[0]
        $minor = [int]$parts[1]
        return ($major -eq 3 -and $minor -ge 11 -and $minor -le 13)
    } catch { return $false }
}

$VenvPython = Join-Path $ProjectRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $VenvPython)) {
    $Created = $false
    if (Get-Command py.exe -ErrorAction SilentlyContinue) {
        foreach ($Minor in @(12, 11, 13)) {
            if (Test-PythonVersion 'py.exe' @("-3.$Minor")) {
                & py.exe "-3.$Minor" -m venv .venv
                $Created = ($LASTEXITCODE -eq 0)
                if ($Created) { break }
            }
        }
    }
    if (-not $Created -and (Get-Command python.exe -ErrorAction SilentlyContinue) -and (Test-PythonVersion 'python.exe' @())) {
        & python.exe -m venv .venv
        $Created = ($LASTEXITCODE -eq 0)
    }
    if (-not $Created) {
        throw 'Python 3.11, 3.12, or 3.13 is required. Install one and rerun setup.bat.'
    }
}

Write-Host 'Installing pinned PyBullet project requirements...'
& $VenvPython -m pip install --upgrade pip
if ($LASTEXITCODE -ne 0) { throw 'Could not upgrade pip.' }
& $VenvPython -m pip install -r requirements.txt
if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed.' }

Write-Host 'Checking PyBullet and OpenCV AprilTag support...'
& $VenvPython -c "import cv2, numpy, pybullet as p; assert hasattr(cv2, 'aruco') and hasattr(cv2.aruco, 'DICT_APRILTAG_36h11'); c=p.connect(p.DIRECT); assert c>=0; p.disconnect(c); print('OpenCV AprilTag and PyBullet DIRECT checks passed')"
if ($LASTEXITCODE -ne 0) { throw 'Post-install verification failed. See the error above.' }

Write-Host 'Setup complete. Run .\run.bat check, then .\run.bat 1.'
