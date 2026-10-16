# Run from the extracted NAVStation folder. No administrator permission required.
$ErrorActionPreference = 'Stop'
$target = Join-Path $PSScriptRoot 'NAVStation.exe'
if (!(Test-Path $target)) { throw 'Keep this script next to NAVStation.exe.' }
$shell = New-Object -ComObject WScript.Shell
$desktop = [Environment]::GetFolderPath('Desktop')
$link = $shell.CreateShortcut((Join-Path $desktop 'NAV Station V2.0.lnk'))
$link.TargetPath = $target
$link.WorkingDirectory = $PSScriptRoot
$link.Description = 'NAV Station V2.0 - offline simulation and SSH edition'
$link.Save()
Write-Host 'Desktop shortcut created. Keep the application folder in its current location.'
