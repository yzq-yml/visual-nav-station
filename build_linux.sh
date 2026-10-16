#!/usr/bin/env bash
# Build on Ubuntu 20.04 with the system Python and installed ROS Noetic modules.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
build_python="${NAV_BUILD_PYTHON:-/usr/bin/python3}"
if [[ -f /opt/ros/noetic/setup.bash ]]; then
    set +u
    source /opt/ros/noetic/setup.bash
    set -u
fi
mkdir -p build/tooling
"$build_python" -m pip install --upgrade --target build/tooling 'pyinstaller==6.11.1' 'pyinstaller-hooks-contrib==2024.10' 'setuptools==69.5.1'
export PYTHONPATH="$PWD/build/tooling${PYTHONPATH:+:$PYTHONPATH}"
"$build_python" -c 'import PyQt5, cv2, rasterio, paramiko, rospy; from mavros_msgs.msg import State; from sensor_msgs.msg import Imu'
"$build_python" -m PyInstaller --noconfirm --clean --windowed --name nav-station \
  --add-data 'assets:assets' --add-data 'configs:configs' \
  --collect-data rasterio --collect-submodules rasterio --collect-submodules rospy --collect-submodules rosgraph \
  --collect-submodules roslib --collect-submodules rospkg \
  --collect-submodules sensor_msgs.msg --collect-submodules geometry_msgs.msg \
  --collect-submodules mavros_msgs.msg --hidden-import genpy --hidden-import rospkg.os_detect \
  main.py
cp packaging/install-desktop.sh dist/nav-station/
cp packaging/使用说明.txt dist/nav-station/
chmod +x dist/nav-station/install-desktop.sh
QT_QPA_PLATFORM=offscreen dist/nav-station/nav-station --self-test --require-ros
archive="nav-station-2.0.0-ubuntu20.04-$(uname -m).tar.gz"
tar -czf "dist/$archive" -C dist nav-station
(cd dist && sha256sum "$archive" > "$archive.sha256")
printf '安装包已生成：dist/%s\n' "$archive"
