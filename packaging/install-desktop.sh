#!/usr/bin/env bash
set -euo pipefail
app_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$HOME/.local/share/applications"
cat > "$HOME/.local/share/applications/nav-station.desktop" <<DESKTOP
[Desktop Entry]
Type=Application
Name=NAV Station V2.0
Comment=Visual navigation ground station
Exec="$app_dir/nav-station"
Path=$app_dir
Terminal=false
Categories=Science;Education;
DESKTOP
chmod +x "$app_dir/nav-station"
printf '%s\n' '已添加到应用程序菜单：NAV Station V2.0。请保留当前程序目录。'
