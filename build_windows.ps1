# Run in Windows PowerShell. Produces the offline/SSH edition (no ROS subscriber).
param([switch]$Console)
$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot
if ($env:OS -ne 'Windows_NT') { throw 'Build this package on Windows.' }
function Invoke-Checked([scriptblock]$Command) {
    & $Command
    if ($LASTEXITCODE -ne 0) { throw "Command failed (exit $LASTEXITCODE). Build stopped." }
}
Invoke-Checked { py -3.13 -m venv .venv-windows }
$python = Join-Path $PSScriptRoot '.venv-windows\Scripts\python.exe'
Invoke-Checked { & $python -c "import struct, platform; assert struct.calcsize('P') == 8 and platform.machine().upper() in ('AMD64', 'X86_64'), 'Install x64 Python'" }
Invoke-Checked { & $python -m pip install --upgrade pip }
Invoke-Checked { & $python -m pip install -r requirements.txt 'pyinstaller>=6.11,<7' }
Invoke-Checked { & $python -m pip check }
Invoke-Checked { & $python main.py --self-test }
$mode = if ($Console) { '--console' } else { '--windowed' }
$arguments = @('-m', 'PyInstaller', '--noconfirm', '--clean', '--onedir', $mode,
    '--name', 'NAVStation', '--add-data', 'assets;assets', '--add-data', 'configs;configs',
    '--collect-data', 'rasterio', '--collect-submodules', 'rasterio',
    '--exclude-module', 'rospy', 'main.py')
Invoke-Checked { & $python @arguments }
$executable = Join-Path $PSScriptRoot 'dist\NAVStation\NAVStation.exe'
$check = Start-Process -FilePath $executable -ArgumentList '--self-test' -Wait -PassThru
if ($check.ExitCode -ne 0) { throw 'Packaged self-test failed. Rebuild with -Console for diagnostics.' }
Copy-Item 'packaging\install-shortcut.ps1' 'dist\NAVStation\'
Copy-Item 'packaging\WINDOWS.md' 'dist\NAVStation\'
$archive = Join-Path $PSScriptRoot 'dist\NAVStation-2.0.0-windows-x64.zip'
Compress-Archive -Path 'dist\NAVStation' -DestinationPath $archive -Force
$hash = (Get-FileHash -Algorithm SHA256 $archive).Hash.ToLower()
"$hash  NAVStation-2.0.0-windows-x64.zip" | Set-Content "$archive.sha256" -Encoding ASCII
Write-Host "Built: $archive"
Write-Host 'Includes UI, simulation, GeoTIFF and SSH. Live ROS subscriptions are not included.'
