#!/usr/bin/env bash
set -euo pipefail
python3 -m pip install --upgrade pip
python3 -m pip install . pyinstaller
pyinstaller --noconfirm --clean --windowed --name nav-station --add-data 'assets:assets' --add-data 'configs:configs' main.py
echo 'Built dist/nav-station/nav-station'
