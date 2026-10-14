# Build a portable Windows executable. Run from a clean virtual environment.
python -m pip install --upgrade pip
python -m pip install . pyinstaller
pyinstaller --noconfirm --clean --windowed --name NAVStation --add-data "assets;assets" --add-data "configs;configs" main.py
Write-Host "Built dist/NAVStation/NAVStation.exe"
