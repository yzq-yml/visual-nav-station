"""Visual Navigation Ground Station - first PC shell.

The UI is intentionally usable without a board: telemetry and device actions
come from ``DeviceAdapter`` and can be replaced by the ROS/SSH implementation
without changing any page widgets.
"""
from __future__ import annotations

import math
import json
import random
import shlex
import sys
import threading
import time
import os
import base64
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

try:
    from PyQt5.QtCore import QObject, QTimer, Qt, QEvent, QPointF, QRectF, pyqtSignal, QLibraryInfo
    from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPixmap, QPolygonF, QPen
    from PyQt5.QtWidgets import (
        QApplication, QComboBox, QDialog, QFileDialog, QFrame, QGridLayout,
        QHBoxLayout, QLabel, QLineEdit, QListWidget, QListWidgetItem,
        QMainWindow, QMessageBox, QPlainTextEdit, QProgressBar, QPushButton,
        QScrollArea, QStackedWidget, QTableWidget, QTableWidgetItem,
        QVBoxLayout, QWidget, QSizePolicy,
    )
except ImportError as exc:  # Keep a clear error when dependencies are absent.
    raise SystemExit("缺少 PyQt5，请执行: pip install -r requirements.txt") from exc

# OpenCV bundles another Qt plugin tree. Pin Qt to PyQt5's plugins before cv2
# is imported, otherwise Linux may load cv2's incompatible xcb plugin.
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = QLibraryInfo.location(QLibraryInfo.PluginsPath)
import cv2
import numpy as np
# cv2 may overwrite the variable while importing its Qt helpers.
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = QLibraryInfo.location(QLibraryInfo.PluginsPath)
os.environ["QT_PLUGIN_PATH"] = QLibraryInfo.location(QLibraryInfo.PluginsPath)

try:
    import paramiko
except ImportError:
    paramiko = None

try:
    import rasterio
    from rasterio import Affine
    from rasterio.warp import transform as transform_coordinates
except ImportError:
    rasterio = None
    Affine = None
    transform_coordinates = None

try:
    import rospy
    from sensor_msgs.msg import CompressedImage, Imu, NavSatFix, Range
    from geometry_msgs.msg import TwistStamped
    from mavros_msgs.msg import State
except ImportError:
    rospy = None


ROOT = Path(__file__).resolve().parent


@dataclass
class Telemetry:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    altitude: float = 0.0
    speed_fcu: float = 0.0
    speed_algo: float = 0.0
    lat: float = 0.0
    lon: float = 0.0
    fcu_lat: float = 0.0
    fcu_lon: float = 0.0
    algo_lat: float = 0.0
    algo_lon: float = 0.0
    fcu_gps_valid: bool = False
    algo_gps_valid: bool = False
    range_m: float = 0.0
    imu: str = "--"
    link_ms: int = 0
    received: dict = None
    flight_state: str = "等待飞控状态"
    simulated: bool = False


class DeviceAdapter(QObject):
    """Board boundary. Replace methods here with ROS/API calls later."""

    telemetry_changed = pyqtSignal(object)
    log = pyqtSignal(str)
    connection_changed = pyqtSignal(bool)
    config_loaded = pyqtSignal(str, str)
    config_listed = pyqtSignal(object)

    def __init__(self) -> None:
        super().__init__(); self.ssh = None; self.host = "155.155.0.14"; self.username = "firefly"; self.password = "firefly"; self.connected = False; self.sensors_running = False; self.algorithm_running = False

    def connect_board(self, address: str, user: str, _password: str) -> None:
        if paramiko is None: self.log.emit("Paramiko 未安装，无法连接板卡"); return
        self.host, self.username, self.password = address.strip(), user.strip(), _password
        def worker() -> None:
            try:
                client = paramiko.SSHClient(); client.set_missing_host_key_policy(paramiko.WarningPolicy()); client.connect(self.host, username=self.username, password=self.password, timeout=6, look_for_keys=True, allow_agent=True); self.ssh = client; self.connected = True; self.connection_changed.emit(True); self.log.emit(f"板卡 SSH 连接成功: {self.username}@{self.host}")
            except Exception as exc: self.connected = False; self.connection_changed.emit(False); self.log.emit(f"板卡连接失败: {exc}")
        threading.Thread(target=worker, daemon=True).start()

    def disconnect(self) -> None:
        if self.ssh:
            try: self.ssh.close()
            except Exception: pass
            self.ssh = None
        self.connected = False
        self.connection_changed.emit(False)
        self.log.emit("板卡已断开")

    def start_roscore(self) -> None:
        self._exec("source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=http://155.155.0.14:11311; export ROS_IP=155.155.0.14; mkdir -p ~/results/ground_station_logs; if pgrep -x rosmaster >/dev/null; then echo 'roscore already running'; else nohup roscore > ~/results/ground_station_logs/roscore.log 2>&1 & echo 'roscore started'; fi", "roscore 启动")

    def start_sensors(self, enabled: bool = True) -> None:
        self.sensors_running = enabled
        # Use bracketed process patterns so pkill never matches its own shell command.
        command = "source /opt/ros/noetic/setup.bash && source ~/sensor_ws/devel/setup.bash && export ROS_MASTER_URI=http://155.155.0.14:11311 && export ROS_IP=155.155.0.14 && mkdir -p ~/results/ground_station_logs && (nohup roslaunch veye_camera veye_camera.launch > ~/results/ground_station_logs/rgb.log 2>&1 &) && (nohup roslaunch airui_ir_camera airui_ir_camera.launch > ~/results/ground_station_logs/infrared.log 2>&1 &) && (nohup roslaunch sfa1500c_tof_high sfa1500c_tof_high.launch > ~/results/ground_station_logs/range.log 2>&1 &)" if enabled else "pkill -TERM -f '[r]oslaunch veye_camera veye_camera.launch' 2>/dev/null || true; pkill -TERM -f '[r]oslaunch airui_ir_camera airui_ir_camera.launch' 2>/dev/null || true; pkill -TERM -f '[r]oslaunch sfa1500c_tof_high sfa1500c_tof_high.launch' 2>/dev/null || true; pkill -TERM -x veye_camera 2>/dev/null || true; pkill -TERM -x airui_ir_camera 2>/dev/null || true; pkill -TERM -x sfa1500c_tof_high 2>/dev/null || true; pkill -TERM -f '[r]oslaunch mavros px4.launch' 2>/dev/null || true; pkill -TERM -x mavros_node 2>/dev/null || true; sleep 1; echo 'sensor and mavros processes stopped (roscore kept running)'"
        self._exec(command, f"传感器组{'启动' if enabled else '停止'}")

    def start_algorithm(self, mode: str = "rgb") -> None:
        config = "1.260810_veye_km220" if mode == "rgb" else "2.260810_airui_km220"
        launch = "subscribe_veye_km220.launch" if mode == "rgb" else "subscribe_airui_km220.launch"
        command = f"pkill -f 'roslaunch.*subscribe_.*km220.launch' 2>/dev/null || true; pkill -f run_subscribe_msckf 2>/dev/null || true; source /opt/ros/noetic/setup.bash && source ~/estimators/geovins_qingdao_ws/devel/setup.bash && export ROS_MASTER_URI=http://155.155.0.14:11311 && export ROS_IP=155.155.0.14 && mkdir -p ~/results/ground_station_logs && nohup roslaunch ov_msckf {launch} config:={config} > ~/results/ground_station_logs/algorithm_{mode}.log 2>&1 &"
        self.algorithm_running = True; self._exec(command, f"{mode.upper()} 导航算法启动")

    def stop_algorithm(self) -> None:
        self.algorithm_running = False; self._exec("pkill -f 'roslaunch.*subscribe_.*km220.launch' 2>/dev/null || true; pkill -f run_subscribe_msckf 2>/dev/null || true", "导航算法停止")

    def start_flight_controller(self, enabled: bool = True) -> None:
        command = "source /opt/ros/noetic/setup.bash && export ROS_MASTER_URI=http://155.155.0.14:11311 && export ROS_IP=155.155.0.14 && mkdir -p ~/results/ground_station_logs && nohup roslaunch mavros px4.launch" if enabled else "pkill -TERM -f '[r]oslaunch mavros px4.launch' 2>/dev/null || true; pkill -TERM -x mavros_node 2>/dev/null || true; echo 'mavros stopped (roscore kept running)'"
        if enabled: command += " > ~/results/ground_station_logs/mavros.log 2>&1 &"
        self._exec(command, f"飞控驱动{'启动' if enabled else '停止'}")

    def _exec(self, command: str, label: str) -> None:
        if not self.ssh: self.log.emit(f"{label}失败：尚未连接板卡"); return
        def worker() -> None:
            try:
                _, stdout, stderr = self.ssh.exec_command("bash -lc " + shlex.quote(command), timeout=10); code = stdout.channel.recv_exit_status(); output = stdout.read().decode("utf-8", "replace").strip(); error = stderr.read().decode("utf-8", "replace").strip(); details = output or error; self.log.emit(f"{label}完成，退出码 {code}" + (f": {details[-800:]}" if details else ""))
            except Exception as exc: self.log.emit(f"{label}失败: {exc}")
        threading.Thread(target=worker, daemon=True).start()

    def sensor_command(self, name: str, enabled: bool, topic: str = "") -> None:
        if not enabled or not topic: return
        if "image_raw" in topic:
            command = f"source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=http://155.155.0.14:11311; timeout 4 rostopic hz -w 5 {shlex.quote(topic)}"
        else:
            command = f"source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=http://155.155.0.14:11311; timeout 4 rostopic echo -n 1 {shlex.quote(topic)}"
        self._exec(command, f"测试传感器 {name}")

    def list_remote_configs(self, directory: str) -> None:
        self._exec_config(f"find {shlex.quote(directory)} -maxdepth 3 -type f \\( -name '*.yaml' -o -name '*.yml' -o -name '*.json' -o -name '*.xml' \\) -printf '%P\\n' | sort", directory, list_mode=True)

    def read_remote_config(self, path: str) -> None:
        self._exec_config(f"base64 -w0 {shlex.quote(path)}", path)

    def write_remote_config(self, path: str, content: str) -> None:
        encoded = base64.b64encode(content.encode('utf-8')).decode('ascii')
        self._exec_config(f"echo {shlex.quote(encoded)} | base64 -d > {shlex.quote(path)}", path, write_mode=True)

    def _exec_config(self, command: str, path: str, list_mode: bool = False, write_mode: bool = False) -> None:
        if not self.ssh:
            self.log.emit("远程配置操作失败：尚未连接板卡"); return
        def worker() -> None:
            try:
                _, stdout, stderr = self.ssh.exec_command("bash -lc " + shlex.quote(command), timeout=10)
                code = stdout.channel.recv_exit_status(); out = stdout.read().decode('utf-8', 'replace').strip(); err = stderr.read().decode('utf-8', 'replace').strip()
                if list_mode: self.config_listed.emit(out.splitlines() if out else []); return
                if write_mode: self.log.emit(f"板卡配置已保存: {path} (退出码 {code})"); return
                if code != 0: self.log.emit(f"读取板卡配置失败: {err or out}"); return
                self.config_loaded.emit(path, base64.b64decode(out).decode('utf-8', 'replace'))
            except Exception as exc: self.log.emit(f"远程配置操作失败: {exc}")
        threading.Thread(target=worker, daemon=True).start()

    def sensor_launch(self, name: str, package: str, launch: str, enabled: bool) -> None:
        node = {"RGB camera": "veye_camera", "IR camera": "airui_ir_camera", "ToF range (5-1200 m)": "sfa1500c_tof_high"}.get(name, package)
        if enabled:
            command = f"source /opt/ros/noetic/setup.bash; source ~/sensor_ws/devel/setup.bash; export ROS_MASTER_URI=http://155.155.0.14:11311; export ROS_IP=155.155.0.14; mkdir -p ~/results/ground_station_logs; nohup roslaunch --wait {package} {launch} > ~/results/ground_station_logs/{node}.log 2>&1 &"
        else:
            command = f"pkill -f '/{node}.*__name:={node}' 2>/dev/null || true; pkill -f 'roslaunch.*{launch}' 2>/dev/null || true"
        self._exec(command, f"{name}{'启动' if enabled else '停止'}")

    def upload_assets(self, paths: list[str]) -> None:
        self._upload_files(paths, "/home/firefly/estimators/datasets", "地图资产")

    def upload_calibration(self, paths: list[str]) -> None:
        self._upload_files(paths, "/home/firefly/estimators/geovins_qingdao_ws/src/geovins_rk3588/config", "标定结果")

    def _upload_files(self, paths: list[str], remote_dir: str, label: str) -> None:
        if not self.ssh: self.log.emit(f"{label}上传失败：尚未连接板卡"); return
        def worker() -> None:
            try:
                sftp = self.ssh.open_sftp()
                _, out, _ = self.ssh.exec_command(f"mkdir -p {remote_dir}"); out.channel.recv_exit_status()
                for path in paths: sftp.put(path, f"{remote_dir}/{Path(path).name}")
                sftp.close(); self.log.emit(f"{label}已上传 {len(paths)} 个文件到板卡 {remote_dir}")
            except Exception as exc: self.log.emit(f"{label}上传失败: {exc}")
        threading.Thread(target=worker, daemon=True).start()

class RosBridge(QObject):
    telemetry = pyqtSignal(object)
    image = pyqtSignal(str, object)
    error = pyqtSignal(str)

    def __init__(self) -> None:
        super().__init__(); self.subscribers = []; self.started = False; self.t = Telemetry(received={})

    def start(self, board_ip: str) -> None:
        if not rospy: self.error.emit("当前 Python 没有 ROS 环境，请使用 /usr/bin/python3.8 启动"); return
        if self.started: return
        try:
            import socket
            probe = socket.create_connection((board_ip, 11311), timeout=1.5); probe.close()
            os.environ["ROS_MASTER_URI"] = f"http://{board_ip}:11311"
            route = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); route.connect((board_ip, 11311)); os.environ["ROS_IP"] = route.getsockname()[0]; route.close()
            if not rospy.core.is_initialized(): rospy.init_node("nav_station_ui", anonymous=True, disable_signals=True)
            self.subscribers = [rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps), rospy.Subscriber("/mavros/imu/data", Imu, self.imu), rospy.Subscriber("/sfa1500c_tof_high/range", Range, self.range), rospy.Subscriber("/veye_camera/image_raw/compressed", CompressedImage, lambda m: self.on_image(m, "rgb")), rospy.Subscriber("/airui_ir_camera/image_raw/compressed", CompressedImage, lambda m: self.on_image(m, "ir")), rospy.Subscriber("/geovins_gps", NavSatFix, self.algo_gps)]
            self.subscribers.extend([rospy.Subscriber("/mavros/state", State, self.state), rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.velocity)])
            self.started = True; self.error.emit("ROS 真实话题订阅已建立")
        except Exception as exc: self.error.emit(f"ROS 订阅失败（板卡 master 不可达或未启动）: {exc}")

    def stop(self) -> None:
        for sub in self.subscribers:
            try: sub.unregister()
            except Exception: pass
        self.subscribers = []; self.started = False

    def publish(self, source):
        self.t.received[source] = time.monotonic()
        self.telemetry.emit(self.t)

    def gps(self, msg):
        self.t.fcu_gps_valid = msg.status.status >= 0 and math.isfinite(msg.latitude) and math.isfinite(msg.longitude)
        self.t.fcu_lat, self.t.fcu_lon = msg.latitude, msg.longitude
        self.t.altitude = msg.altitude
        self.publish("gps")

    def algo_gps(self, msg):
        self.t.algo_gps_valid = msg.status.status >= 0 and math.isfinite(msg.latitude) and math.isfinite(msg.longitude)
        self.t.algo_lat, self.t.algo_lon = msg.latitude, msg.longitude
        self.publish("algorithm")

    def imu(self, msg):
        q = msg.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if msg.orientation_covariance[0] < 0 or norm < 1e-8: return
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        self.t.roll = math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)))
        self.t.pitch = math.degrees(math.asin(max(-1, min(1, 2*(w*y-z*x)))))
        self.t.yaw = math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
        self.publish("imu")

    def range(self, msg):
        self.t.range_m = msg.range if msg.min_range <= msg.range <= msg.max_range else float("nan")
        self.publish("range")

    def state(self, msg):
        self.t.flight_state = f"{msg.mode} · {'已解锁' if msg.armed else '未解锁'}" if msg.connected else "飞控未连接"
        self.publish("state")

    def velocity(self, msg):
        v = msg.twist.linear
        self.t.speed_fcu = math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
        self.publish("velocity")

    def on_image(self, msg, tag):
        try:
            frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                if frame.ndim == 2: frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
                else: frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                self.image.emit(tag, frame)
        except Exception as exc: self.error.emit(f"{tag} 图像解码失败: {exc}")


class Metric(QFrame):
    def __init__(self, title: str, value: str = "--", accent: str = "#59d9ff", table_row: bool = False) -> None:
        super().__init__()
        self.setObjectName("telemetryRow" if table_row else "metric")
        layout = QHBoxLayout(self) if table_row else QVBoxLayout(self)
        layout.setContentsMargins(14, 11, 14, 11)
        label = QLabel(title.upper()); label.setObjectName("metricTitle")
        self.value = QLabel(value); self.value.setObjectName("metricValue")
        self.accent = accent
        if table_row:
            label.setFixedWidth(54)
            self.value.setWordWrap(True)
            self.value.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
            layout.addWidget(label); layout.addWidget(self.value, 1)
        else:
            layout.addWidget(label); layout.addWidget(self.value)

    def set_value(self, value: str) -> None:
        self.value.setText(value)


class ImagePanel(QFrame):
    def __init__(self, title: str) -> None:
        super().__init__(); self.setObjectName("panel")
        layout = QVBoxLayout(self); layout.setContentsMargins(10, 10, 10, 10)
        head = QHBoxLayout(); head.addWidget(QLabel(title)); head.addStretch()
        self.status = QLabel("● STANDBY"); self.status.setObjectName("muted"); head.addWidget(self.status)
        self.image = QLabel("NO SIGNAL"); self.image.setAlignment(Qt.AlignCenter); self.image.setMinimumHeight(180); self.image.setObjectName("image")
        layout.addLayout(head); layout.addWidget(self.image)

    def set_online(self, online: bool) -> None:
        self.status.setText("● LIVE" if online else "● STANDBY")
        self.status.setStyleSheet("color:#43e6a0;" if online else "color:#7e8ca3;")
        if not online: self.image.clear(); self.image.setText("NO SIGNAL")

    def set_frame(self, frame: np.ndarray) -> None:
        self.set_online(True); h, w, ch = frame.shape; pix = QPixmap.fromImage(QImage(frame.data, w, h, ch * w, QImage.Format_RGB888).copy()); self.image.setPixmap(pix.scaled(self.image.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))


class MapPanel(QFrame):
    def __init__(self) -> None:
        super().__init__(); self.setObjectName("panel")
        self.map_image = None; self.map_transform = None; self.map_crs = None
        self.fcu_path = []; self.algo_path = []; self.last_fcu = None; self.last_algo = None
        self.last_telemetry = None
        self.follow_gps = True
        self.pan_center = None
        self.drag_position = None
        self.rendered_center = None
        self.source_per_pixel = (1.0, 1.0)
        self.large_window = None
        self.demo_coordinates = False
        # Keep the original gps_map_node-style fixed 800x800 source ROI.
        # Zoom buttons change the source area, never the widget geometry.
        self.zoom = 1.0
        layout = QVBoxLayout(self); layout.setContentsMargins(12, 12, 12, 12)
        head = QHBoxLayout(); head.setContentsMargins(0, 0, 0, 0); title = QLabel("GPS / ALGORITHM TRACK"); title.setObjectName("mapTitle"); head.addWidget(title); head.addStretch()
        self.follow_button = QPushButton("跟随 GPS"); self.follow_button.setCheckable(True); self.follow_button.setChecked(True); self.follow_button.setFixedHeight(28); self.follow_button.clicked.connect(self.toggle_follow); head.addWidget(self.follow_button)
        self.large_button = QPushButton("大地图"); self.large_button.setFixedHeight(28); self.large_button.clicked.connect(self.open_large); head.addWidget(self.large_button)
        self.zoom_out_button = QPushButton("−"); self.zoom_out_button.setToolTip("缩小地图"); self.zoom_out_button.setFixedSize(32, 28); self.zoom_out_button.clicked.connect(self.zoom_out)
        self.zoom_reset_button = QPushButton("1:1"); self.zoom_reset_button.setToolTip("显示完整地图"); self.zoom_reset_button.setFixedSize(42, 28); self.zoom_reset_button.clicked.connect(self.zoom_reset)
        self.zoom_in_button = QPushButton("+"); self.zoom_in_button.setToolTip("放大飞行区域"); self.zoom_in_button.setFixedSize(32, 28); self.zoom_in_button.clicked.connect(self.zoom_in)
        head.addWidget(self.zoom_out_button); head.addWidget(self.zoom_reset_button); head.addWidget(self.zoom_in_button); head.addWidget(QLabel("BASEMAP • DSM"))
        self.view = QLabel("MAP VIEW\n\n        ·  ·  ·\n   ·  ?  ·\n        ·  ·  ·\n\nWAITING FOR ROS GPS")
        self.view.setCursor(Qt.OpenHandCursor)
        self.view.setToolTip("按住鼠标左键拖动地图，滚轮缩放；点击跟随 GPS 恢复跟随")
        self.view.installEventFilter(self); self.view.setAlignment(Qt.AlignCenter); self.view.setObjectName("mapView"); self.view.setMinimumSize(320, 300); self.view.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        layout.addLayout(head); layout.addWidget(self.view, 1)

    def eventFilter(self, watched, event):
        if watched is self.view:
            if event.type() == QEvent.Wheel:
                if event.angleDelta().y() > 0: self.zoom_in()
                elif event.angleDelta().y() < 0: self.zoom_out()
                event.accept()
                return True
            if event.type() == QEvent.MouseButtonPress and event.button() == Qt.LeftButton and self.map_image is not None:
                self.drag_position = event.pos()
                self.view.setCursor(Qt.ClosedHandCursor)
                return True
            if event.type() == QEvent.MouseMove and self.drag_position is not None and event.buttons() & Qt.LeftButton:
                delta = event.pos() - self.drag_position
                self.drag_position = event.pos()
                if delta.isNull(): return True
                center = self.rendered_center or (self.map_image.shape[1]/2, self.map_image.shape[0]/2)
                self.pan_center = (center[0] - delta.x()*self.source_per_pixel[0],
                                   center[1] - delta.y()*self.source_per_pixel[1])
                self.follow_gps = False
                self.follow_button.setChecked(False)
                self.follow_button.setText("固定视图")
                self.render_map()
                return True
            if event.type() == QEvent.MouseButtonRelease and event.button() == Qt.LeftButton:
                self.drag_position = None
                self.view.setCursor(Qt.OpenHandCursor)
                return True
        return super().eventFilter(watched, event)

    def toggle_follow(self, enabled: bool) -> None:
        self.follow_gps = enabled
        if not enabled: self.pan_center = self.rendered_center
        self.follow_button.setText("跟随 GPS" if enabled else "固定视图")
        self.render_map()

    def open_large(self) -> None:
        if self.large_window is None:
            self.large_window = QDialog(self); self.large_window.setWindowTitle("GPS / ALGORITHM TRACK"); self.large_window.resize(1100, 760)
            box = QVBoxLayout(self.large_window); self.large_view = QLabel(); self.large_view.setAlignment(Qt.AlignCenter); self.large_view.setObjectName("mapView"); box.addWidget(self.large_view)
            self.large_window.finished.connect(lambda _=0: setattr(self, "large_window", None))
        self.large_window.show(); self.large_window.raise_(); self.large_window.activateWindow(); self.render_map()

    def load_basemap(self, path: str) -> tuple[bool, str]:
        if rasterio is None:
            return False, "当前 Python 缺少 rasterio；请使用 /usr/bin/python3.8 main.py 启动"
        try:
            with rasterio.open(path) as dataset:
                max_side = 1800
                scale = max(1.0, max(dataset.width, dataset.height) / max_side)
                out_w = max(1, int(dataset.width / scale)); out_h = max(1, int(dataset.height / scale))
                indexes = list(range(1, min(dataset.count, 3) + 1))
                data = dataset.read(indexes, out_shape=(len(indexes), out_h, out_w))
                if len(indexes) == 1: data = np.repeat(data, 3, axis=0)
                self.map_image = np.moveaxis(data[:3], 0, 2).astype(np.uint8)
                self.map_transform = dataset.transform * Affine.scale(dataset.width / out_w, dataset.height / out_h)
                self.map_crs = dataset.crs
            self.fcu_path.clear(); self.algo_path.clear(); self.last_fcu = None; self.last_algo = None
            self.pan_center = None; self.drag_position = None
            self.render_map()
            crs = str(self.map_crs) if self.map_crs else "无 CRS"
            return True, f"{Path(path).name} · {out_w}×{out_h} · {crs}"
        except Exception as exc:
            return False, str(exc)

    def gps_to_pixel(self, lat: float, lon: float):
        if self.demo_coordinates:
            return (int(round((lon - 120.0) / 0.00001)), int(round((36.01 - lat) / 0.00001)))
        if self.map_image is None or self.map_transform is None: return None
        x, y = lon, lat
        if self.map_crs and str(self.map_crs).upper() not in ("EPSG:4326", "OGC:CRS84"):
            x_values, y_values = transform_coordinates("EPSG:4326", self.map_crs, [lon], [lat]); x, y = x_values[0], y_values[0]
        col, row = ~self.map_transform * (x, y); point = (int(round(col)), int(round(row)))
        h, w = self.map_image.shape[:2]
        return point if 0 <= point[0] < w and 0 <= point[1] < h else None

    def update_position(self, t: Telemetry) -> None:
        self.last_telemetry = t
        if self.map_image is None:
            self.view.setText(f"尚未加载 GPS 对比底图\n\nFCU  {t.fcu_lat:.6f}, {t.fcu_lon:.6f}\nALGO {t.algo_lat:.6f}, {t.algo_lon:.6f}")
            return
        if t.fcu_gps_valid:
            point = self.gps_to_pixel(t.fcu_lat, t.fcu_lon)
            if point and point != self.last_fcu: self.fcu_path.append(point); self.fcu_path = self.fcu_path[-10000:]; self.last_fcu = point
        if t.algo_gps_valid:
            point = self.gps_to_pixel(t.algo_lat, t.algo_lon)
            if point and point != self.last_algo: self.algo_path.append(point); self.algo_path = self.algo_path[-10000:]; self.last_algo = point
        self.render_map(t)

    def render_map(self, t: Telemetry | None = None) -> None:
        if self.map_image is None: return
        t = t or self.last_telemetry
        image = cv2.cvtColor(self.map_image.copy(), cv2.COLOR_RGB2BGR)
        if len(self.algo_path) > 1: cv2.polylines(image, [np.asarray(self.algo_path, np.int32)], False, (0, 0, 255), 3)
        if len(self.fcu_path) > 1: cv2.polylines(image, [np.asarray(self.fcu_path, np.int32)], False, (255, 0, 0), 3)
        if self.algo_path: cv2.circle(image, self.algo_path[-1], 7, (0, 255, 0), -1)
        if self.fcu_path: cv2.circle(image, self.fcu_path[-1], 7, (0, 255, 255), -1)
        center = (self.algo_path[-1] if self.algo_path else (self.fcu_path[-1] if self.fcu_path else (image.shape[1] // 2, image.shape[0] // 2))) if self.follow_gps else (self.pan_center or (image.shape[1] // 2, image.shape[0] // 2))
        # Crop an in-bounds ROI matching the widget aspect ratio. The widget
        # keeps its fixed size; only the visible source area changes with zoom.
        map_h, map_w = image.shape[:2]
        view_w = max(320, self.view.width()); view_h = max(240, self.view.height())
        aspect = view_w / float(view_h)
        roi_h = min(map_h, max(2, int(800 / self.zoom)))
        roi_w = max(2, int(roi_h * aspect))
        if roi_w > map_w:
            roi_w = map_w
            roi_h = min(map_h, max(2, int(roi_w / aspect)))
        cx = min(max(int(center[0]), roi_w // 2), map_w - (roi_w - roi_w // 2))
        cy = min(max(int(center[1]), roi_h // 2), map_h - (roi_h - roi_h // 2))
        self.rendered_center = (cx, cy)
        if not self.follow_gps: self.pan_center = self.rendered_center
        x0, y0 = cx - roi_w // 2, cy - roi_h // 2
        crop = image[y0:y0 + roi_h, x0:x0 + roi_w]
        image = crop
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB); h, w, ch = rgb.shape
        pix = QPixmap.fromImage(QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy())
        displayed = pix.scaled(self.view.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.source_per_pixel = (roi_w / max(1, displayed.width()), roi_h / max(1, displayed.height()))
        # Screen-space annotation stays small and anchored while zooming/panning.
        painter = QPainter(displayed)
        painter.setRenderHint(QPainter.Antialiasing)
        font = painter.font(); font.setPixelSize(12); font.setBold(True); painter.setFont(font)
        painter.setPen(Qt.NoPen); painter.setBrush(QColor(255, 255, 255, 205))
        bottom = displayed.height() - 12
        painter.drawRoundedRect(QRectF(10, bottom-48, 280, 48), 6, 6)
        painter.setPen(QColor("#18222c"))
        error_text = "GPS error: --"
        if t and t.fcu_gps_valid and t.algo_gps_valid:
            timestamps = t.received or {}
            fresh = all(time.monotonic()-timestamps.get(key, 0) < 3 for key in ("gps", "algorithm"))
            if fresh:
                error = self.haversine(t.fcu_lat, t.fcu_lon, t.algo_lat, t.algo_lon)
                error_text = f"GPS error: {error:.2f} m"
        painter.drawText(20, bottom-29, error_text)
        painter.drawText(20, bottom-11, "FCU: blue/yellow   ALGO: red/green")
        painter.end()
        self.view.setPixmap(displayed)
        if self.large_window is not None and self.large_window.isVisible():
            self.large_view.setPixmap(pix.scaled(self.large_view.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def zoom_in(self) -> None:
        self.zoom = min(8.0, self.zoom * 1.35); self.render_map()

    def zoom_out(self) -> None:
        self.zoom = max(0.1, self.zoom / 1.35); self.render_map()

    def zoom_reset(self) -> None:
        self.pan_center = None
        self.zoom = min(800 / self.map_image.shape[0], 800 * self.view.width() / max(1, self.view.height()) / self.map_image.shape[1]) if self.map_image is not None else 1.0; self.render_map()

    @staticmethod
    def haversine(lat1, lon1, lat2, lon2) -> float:
        import math
        p1, p2 = math.radians(lat1), math.radians(lat2); dp = math.radians(lat2-lat1); dl = math.radians(lon2-lon1)
        a = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
        return 6371000.0 * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        if self.map_image is not None: self.render_map()


class SparkMetric(Metric):
    def __init__(self, title: str, accent: str) -> None:
        super().__init__(title, "NO DATA", accent); self.history = []
        self.chart = QLabel(); self.chart.setMinimumHeight(58); self.chart.setObjectName("spark"); self.layout().addWidget(self.chart)

    def push(self, value: float) -> None:
        self.history.append(float(value)); self.history = self.history[-50:]
        if len(self.history) < 2: return
        pix = QPixmap(260, 58); pix.fill(QColor("#0a1421")); painter = QPainter(pix); painter.setRenderHint(QPainter.Antialiasing); painter.setPen(QColor(self.accent))
        lo, hi = min(self.history), max(self.history); span = hi - lo or 1.0; points = []
        for i, item in enumerate(self.history): points.append((int(i * 258 / max(1, len(self.history)-1)), int(54 - (item-lo) / span * 48)))
        for a, b in zip(points, points[1:]): painter.drawLine(a[0], a[1], b[0], b[1])
        painter.end(); self.chart.setPixmap(pix)


class CameraPair(QFrame):
    def __init__(self) -> None:
        super().__init__(); self.setObjectName("panel"); self.setMinimumWidth(430); self.setMaximumWidth(560); self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding); layout = QVBoxLayout(self); layout.setContentsMargins(10, 10, 10, 10); layout.addWidget(QLabel("DUAL CAMERA PREVIEW")); row = QHBoxLayout(); self.rgb = ImagePanel("RGB"); self.ir = ImagePanel("IR"); row.addWidget(self.rgb); row.addWidget(self.ir); layout.addLayout(row)


class EmblemBackground:
    def paintEvent(self, event):
        super().paintEvent(event)
        pix = QPixmap(str(ROOT / "assets" / "tsinghua-emblem.jpg"))
        if pix.isNull(): return
        painter = QPainter(self)
        painter.setOpacity(0.13)
        target = pix.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        painter.drawPixmap((self.width()-target.width())//2, (self.height()-target.height())//2, target)
        painter.end()


class EmblemDialog(EmblemBackground, QDialog):
    pass


class PageBase(EmblemBackground, QWidget):
    def __init__(self, title: str, subtitle: str) -> None:
        super().__init__(); layout = QVBoxLayout(self); layout.setContentsMargins(26, 24, 26, 24); layout.setSpacing(16)
        h = QHBoxLayout(); title_box = QVBoxLayout(); a = QLabel(title); a.setObjectName("pageTitle"); b = QLabel(subtitle); b.setObjectName("muted"); title_box.addWidget(a); title_box.addWidget(b); h.addLayout(title_box); h.addStretch(); layout.addLayout(h)
        self.body = QVBoxLayout(); self.body.setSpacing(14); layout.addLayout(self.body); layout.addStretch()


class AircraftAttitude(QFrame):
    """Small projected aircraft, driven by the same Euler angles as telemetry."""
    def __init__(self):
        super().__init__()
        self.setObjectName("aircraftMini")
        self.setFixedSize(72, 70)
        self.angles = (0.0, 0.0, 0.0)
        self.valid = False
        # Body coordinates: forward, right, down. Faces form wings/body/tail.
        self.faces = [
            [(1.15,0,0), (-.8,-.13,0), (-.8,.13,0)],
            [(.35,0,0), (-.3,-1,0), (-.5,-1,0), (-.3,0,0)],
            [(.35,0,0), (-.3,1,0), (-.5,1,0), (-.3,0,0)],
            [(-.55,0,0), (-.9,-.43,0), (-1,-.43,0), (-.85,0,0)],
            [(-.55,0,0), (-.9,.43,0), (-1,.43,0), (-.85,0,0)],
            [(-.5,0,0), (-.9,0,-.4), (-1,0,0)],
        ]

    def set_attitude(self, roll, pitch, yaw, valid):
        self.valid = valid and all(math.isfinite(v) for v in (roll, pitch, yaw))
        self.angles = (roll, pitch, yaw) if self.valid else (0, 0, 0)
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self); painter.setRenderHint(QPainter.Antialiasing)
        font = painter.font(); font.setPixelSize(10); painter.setFont(font)
        painter.setPen(QColor("#465364"))
        if not self.valid:
            painter.drawText(QRectF(0, 0, self.width(), 16), Qt.AlignCenter, "无姿态数据")
        painter.translate(self.width()/2, 39)
        painter.scale(0.48, 0.48)
        painter.setPen(QPen(QColor(120,140,160,90), 1))
        painter.drawEllipse(QRectF(-58,-28,116,56))
        painter.drawLine(-66,0,66,0)
        roll, pitch, yaw = [math.radians(v) for v in self.angles]
        cr,sr,cp,sp,cy,sy = math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch),math.cos(yaw),math.sin(yaw)
        rotation = np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                             [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],
                             [-sp,cp*sr,cp*cr]])
        faces = [np.asarray(face) @ rotation.T for face in self.faces]
        faces.sort(key=lambda face: float(np.mean(face[:,0]*.8+face[:,2]*.55)))
        for face in faces:
            polygon = QPolygonF([QPointF(float(y*46), float((-x*.55+z*.8)*46)) for x,y,z in face])
            painter.setBrush(QColor("#c5deef" if self.valid else "#d3d8de"))
            painter.setPen(QPen(QColor("#375d7a" if self.valid else "#87929c"), 1.2))
            painter.drawPolygon(polygon)
        # A contrasting nose makes heading easy to identify.
        nose = rotation @ np.array([1.15,0,0])
        painter.setPen(Qt.NoPen); painter.setBrush(QColor("#e88549" if self.valid else "#a1a8b0"))
        painter.drawEllipse(QPointF(float(nose[1]*46),float((-nose[0]*.55+nose[2]*.8)*46)), 3, 3)
        painter.end()


class MainPage(QWidget):
    def __init__(self, device, stop_all_callback=None):
        super().__init__()
        self.device = device
        self.last = Telemetry()
        self.last_update = 0
        self.images_at = {}
        layers = QGridLayout(self); layers.setContentsMargins(0, 0, 0, 0)
        self.map = MapPanel(); self.map.large_button.hide()
        layers.addWidget(self.map, 0, 0)
        overlay = QWidget(); overlay.setObjectName("missionOverlay")
        overlay.setAttribute(Qt.WA_TranslucentBackground)
        box = QVBoxLayout(overlay); box.setContentsMargins(18, 58, 18, 18)
        self.telemetry_table = QFrame(); self.telemetry_table.setObjectName("telemetryTable")
        self.telemetry_table.setFixedWidth(310)
        table = QVBoxLayout(self.telemetry_table)
        table.setContentsMargins(0, 0, 0, 0); table.setSpacing(0)
        self.att = Metric("姿态信息", table_row=True)
        self.fcu = Metric("导航信息", table_row=True)
        self.sensor = Metric("传感器状态", table_row=True)
        self.speed = Metric("飞行信息", table_row=True)
        for panel in (self.att, self.fcu, self.sensor, self.speed):
            panel.setFixedHeight(100)
            table.addWidget(panel)
        self.aircraft = AircraftAttitude()
        self.att.layout().addWidget(self.aircraft)
        box.addWidget(self.telemetry_table, 0, Qt.AlignRight)
        box.addStretch()
        # Only the actual panels intercept input; empty overlay passes wheel events.
        overlay.setAttribute(Qt.WA_TransparentForMouseEvents)
        layers.addWidget(overlay, 0, 0)
        # Buttons are a separate top layer, preserving mouse access to the map.
        tool_widget = QWidget(); tool_widget.setObjectName("missionTools")
        tool_box = QHBoxLayout(tool_widget); tool_box.setContentsMargins(18, 54, 18, 0)
        for title, target in [("控制窗口", "control_window"), ("RGB / 红外图像", "camera_window")]:
            btn = QPushButton(title); btn.clicked.connect(lambda checked=False, key=target: self.show_window(getattr(self, key))); tool_box.addWidget(btn)
        layers.addWidget(tool_widget, 0, 0, Qt.AlignTop | Qt.AlignLeft)
        self.camera_window = EmblemDialog(self); self.camera_window.setWindowTitle("RGB / 红外实时图像")
        self.camera_window.setFixedSize(960, 400)
        camera_layout = QHBoxLayout(self.camera_window)
        self.rgb, self.ir = ImagePanel("RGB"), ImagePanel("红外 IR")
        camera_layout.addWidget(self.rgb); camera_layout.addWidget(self.ir)
        self.control_window = EmblemDialog(self); self.control_window.setWindowTitle("控制窗口")
        self.control_window.setFixedSize(420, 550)
        control_layout = QVBoxLayout(self.control_window)
        control_layout.addWidget(QLabel("设备控制 / 算法模式"))
        actions = QVBoxLayout(); self.sensor_btn = QPushButton("▶  START ALL SENSORS"); self.sensor_btn.setObjectName("primary"); self.sensor_btn.clicked.connect(lambda: device.start_sensors(True)); self.fcu_btn = QPushButton("▶  START FLIGHT DRIVER"); self.fcu_btn.clicked.connect(lambda: device.start_flight_controller(True)); self.rgb_algo_btn = QPushButton("▶  DAY / RGB ALGORITHM"); self.rgb_algo_btn.setObjectName("primary"); self.rgb_algo_btn.setToolTip("白天模式：启动 RGB GeoVINS"); self.rgb_algo_btn.clicked.connect(lambda: device.start_algorithm("rgb")); self.ir_algo_btn = QPushButton("▶  NIGHT / IR ALGORITHM"); self.ir_algo_btn.setToolTip("黑夜模式：启动红外 GeoVINS"); self.ir_algo_btn.clicked.connect(lambda: device.start_algorithm("ir")); self.stop_algo_btn = QPushButton("■  STOP ALGORITHM"); self.stop_algo_btn.clicked.connect(device.stop_algorithm); actions.addWidget(self.sensor_btn); actions.addWidget(self.fcu_btn); actions.addWidget(self.rgb_algo_btn); actions.addWidget(self.ir_algo_btn); actions.addWidget(self.stop_algo_btn); control_layout.addLayout(actions)
        self.stop_sensor_btn = QPushButton("■  STOP ALL SENSORS"); self.stop_sensor_btn.clicked.connect(stop_all_callback or (lambda: device.start_sensors(False))); actions.addWidget(self.stop_sensor_btn)

        self.detail = QPlainTextEdit(); self.detail.setReadOnly(True); control_layout.addWidget(self.detail)
        self.timer = QTimer(self); self.timer.timeout.connect(self.refresh); self.timer.start(500)
        self.refresh()

    @staticmethod
    def show_window(window):
        window.show(); window.raise_(); window.activateWindow()

    def update(self, t):
        self.last = t; self.last_update = time.monotonic()
        self.map.update_position(t); self.refresh()

    def refresh(self):
        t = self.last; now = time.monotonic(); received = t.received or {}
        def live(key): return (self.device.connected or t.simulated) and now - received.get(key, 0) < 3
        def status(key): return "在线" if live(key) else "无数据 / 超时"
        self.aircraft.set_attitude(t.roll, t.pitch, t.yaw, live("imu"))
        self.att.set_value(f"R {t.roll:+.1f}°  P {t.pitch:+.1f}°\nY {t.yaw:.1f}°" if live("imu") else "等待姿态数据")
        fcu = f"{t.fcu_lat:.6f}, {t.fcu_lon:.6f}" if live("gps") and t.fcu_gps_valid else "等待有效定位"
        algo = f"{t.algo_lat:.6f}, {t.algo_lon:.6f}" if live("algorithm") and t.algo_gps_valid else "等待有效定位"
        self.fcu.set_value(f"FCU  {fcu}\n算法  {algo}\nGPS / GeoVINS 实时轨迹对比")
        cameras = []
        for key, panel in (("rgb", self.rgb), ("ir", self.ir)):
            online = (self.device.connected or t.simulated) and now-self.images_at.get(key, 0) < 3
            if not online: panel.set_online(False)
            cameras.append(f"{key.upper()} {'在线' if online else '无信号'}")
        self.sensor.set_value(f"{' · '.join(cameras)}\nIMU {status('imu')}\nToF {status('range')}")
        if t.simulated:
            self.sensor.set_value(f"[模拟] RGB / IR 在线 · 10 Hz\nIMU 在线 · ToF {t.range_m:.1f} m\n演示数据 / 非真实设备")
        state = t.flight_state if live("state") else "飞控状态无数据 / 超时"
        altitude = f"{t.altitude:.1f} m" if live("gps") and t.fcu_gps_valid else "--"
        velocity = f"{t.speed_fcu:.1f} m/s" if live("velocity") else "--"
        self.speed.set_value(f"{state}\nGPS 海拔 {altitude} · 速度 {velocity}\n" + ("模拟数据 · 未连接板卡" if t.simulated else f"SSH {'已连接' if self.device.connected else '未连接'}"))
        self.detail.setPlainText(("模拟仿真：所有数值和图像均为演示数据\n" if t.simulated else "") + f"SSH: {'已连接' if self.device.connected else '未连接'}\n飞控: {state}\n数据超过 3 秒未更新显示超时")

    def clear_telemetry(self):
        self.last = Telemetry(); self.last_update = 0; self.images_at.clear(); self.refresh()


class SensorPage(PageBase):
    def __init__(self, device: DeviceAdapter) -> None:
        super().__init__("SENSOR LAB", "Independent topic health checks and sensor controls")
        self.device = device
        self.table = QTableWidget(0, 7); self.table.setHorizontalHeaderLabels(["SENSOR", "ROS TOPIC", "RATE", "STATUS", "START", "STOP", "TEST"]); self.table.horizontalHeader().setStretchLastSection(False); self.table.setColumnWidth(0, 160); self.table.setColumnWidth(1, 330); self.table.setColumnWidth(2, 75); self.table.setColumnWidth(3, 110); self.table.setColumnWidth(4, 90); self.table.setColumnWidth(5, 90); self.table.setColumnWidth(6, 90); self.table.verticalHeader().setDefaultSectionSize(46); self.table.setMinimumHeight(285)
        sensors = [("RGB camera", "/veye_camera/image_raw/compressed", "30 Hz", "veye_camera", "veye_camera.launch"), ("IR camera", "/airui_ir_camera/image_raw/compressed", "25 Hz", "airui_ir_camera", "airui_ir_camera.launch"), ("IMU", "/imu/data", "200 Hz", "", ""), ("ToF range (5-1200 m)", "/sfa1500c_tof_high/range", "20 Hz*", "sfa1500c_tof_high", "sfa1500c_tof_high.launch"), ("FCU GPS (outdoor)", "/mavros/global_position/global", "10 Hz*", "", "")]
        for name, topic, rate, package, launch in sensors:
            r = self.table.rowCount(); self.table.insertRow(r); self.table.setItem(r, 0, QTableWidgetItem(name)); self.table.setItem(r, 1, QTableWidgetItem(topic)); self.table.setItem(r, 2, QTableWidgetItem(rate)); self.table.setItem(r, 3, QTableWidgetItem("NOT TESTED")); start = QPushButton("START"); stop = QPushButton("STOP"); test = QPushButton("TEST"); start.setMinimumHeight(32); stop.setMinimumHeight(32); test.setMinimumHeight(32); start.setEnabled(bool(package)); stop.setEnabled(bool(package)); start.clicked.connect(lambda _=False, n=name, p=package, l=launch: device.sensor_launch(n, p, l, True)); stop.clicked.connect(lambda _=False, n=name, p=package, l=launch: device.sensor_launch(n, p, l, False)); test.clicked.connect(lambda _=False, n=name, t=topic, row=r: self.test_topic(n, t, row)); self.table.setCellWidget(r, 4, start); self.table.setCellWidget(r, 5, stop); self.table.setCellWidget(r, 6, test)
        self.body.addWidget(self.table)
        self.body.addWidget(QLabel("ALGORITHM CONFIGURATION  •  BOARD"))
        self.config_dir = "/home/firefly/estimators/geovins_qingdao_ws/src/geovins_rk3588/config"
        self.config_editor = QPlainTextEdit("连接板卡后点击刷新，选择板卡配置文件"); self.config_editor.setMinimumHeight(150); self.config_path = None; self.body.addWidget(self.config_editor)
        config_row = QHBoxLayout(); refresh = QPushButton("刷新板卡配置"); refresh.clicked.connect(lambda: device.list_remote_configs(self.config_dir)); self.config_combo = QComboBox(); self.config_combo.setMinimumWidth(260); self.config_combo.currentTextChanged.connect(self.remote_config_selected); load = QPushButton("打开板卡文件"); load.clicked.connect(self.load_remote_config); save = QPushButton("保存到板卡"); save.setObjectName("primary"); save.clicked.connect(self.save_remote_config); config_row.addWidget(refresh); config_row.addWidget(self.config_combo); config_row.addWidget(load); config_row.addWidget(save); self.body.addLayout(config_row)
        device.config_listed.connect(self.on_config_listed); device.config_loaded.connect(self.on_config_loaded)

    def test_topic(self, name: str, topic: str, row: int) -> None:
        self.table.item(row, 3).setText("TESTING..."); self.device.sensor_command(name, True, topic)

    def load_config(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Open algorithm config", str(ROOT), "Config (*.yaml *.yml *.json);;All files (*)")
        if path:
            self.config_path = path
            try: self.config_editor.setPlainText(Path(path).read_text(encoding="utf-8"))
            except OSError as exc: self.config_editor.setPlainText(f"# read error: {exc}")

    def save_config(self) -> None:
        path = self.config_path
        if not path: path, _ = QFileDialog.getSaveFileName(self, "Save algorithm config", str(ROOT / "estimator.yaml"), "Config (*.yaml *.yml *.json)")
        if path:
            try:
                Path(path).write_text(self.config_editor.toPlainText(), encoding="utf-8"); self.config_path = path; self.device.log.emit(f"算法配置已保存: {path}")
            except OSError as exc: self.device.log.emit(f"配置保存失败: {exc}")

    def on_config_listed(self, names: object) -> None:
        self.config_combo.blockSignals(True); self.config_combo.clear(); self.config_combo.addItems(list(names)); self.config_combo.blockSignals(False); self.device.log.emit(f"发现板卡配置 {len(list(names))} 个")

    def remote_config_selected(self, name: str) -> None:
        self.config_path = f"{self.config_dir}/{name}" if name else None

    def load_remote_config(self) -> None:
        if self.config_path: self.device.read_remote_config(self.config_path)

    def on_config_loaded(self, path: str, content: str) -> None:
        self.config_path = path; self.config_editor.setPlainText(content); self.device.log.emit(f"已打开板卡配置: {path}")

    def save_remote_config(self) -> None:
        if self.config_path: self.device.write_remote_config(self.config_path, self.config_editor.toPlainText())
        else: self.device.log.emit("请先刷新并选择一个板卡配置文件")


class UploadPage(PageBase):
    basemap_selected = pyqtSignal(str)

    def __init__(self, device: DeviceAdapter) -> None:
        super().__init__("MAP & ASSET DEPOT", "Upload matching map, DSM and choose the comparison basemap")
        self.files = QPlainTextEdit(); self.files.setReadOnly(True); self.files.setPlaceholderText("Selected assets will appear here")
        browse = QPushButton("＋  SELECT ASSETS"); browse.clicked.connect(self.select_files); upload = QPushButton("↑  UPLOAD TO BOARD"); upload.setObjectName("primary"); upload.clicked.connect(self.upload); self.browse_button = browse; self.upload_button = upload
        self.body.addWidget(QLabel("板卡地图资产：匹配图 / DSM / 地图数据")); self.body.addWidget(self.files); row = QHBoxLayout(); row.addWidget(browse); row.addWidget(upload); self.body.addLayout(row); self.paths: list[str] = []; self.device = device
        self.body.addWidget(QLabel("客户端 GPS 对比底图（仅用于主页面显示，不上传板卡）")); self.basemap = QLineEdit(); self.basemap.setReadOnly(True); base_browse = QPushButton("选择 GPS 底图"); base_browse.clicked.connect(self.select_basemap); self.basemap_button = base_browse; base_row = QHBoxLayout(); base_row.addWidget(self.basemap); base_row.addWidget(base_browse); self.body.addLayout(base_row); self.basemap_path = None
        self.calibration_files = QPlainTextEdit(); self.calibration_files.setReadOnly(True); self.calibration_files.setMaximumHeight(80); self.calibration_files.setPlaceholderText("Calibration files: intrinsics / extrinsics"); cal_browse = QPushButton("＋  SELECT CALIBRATION"); cal_browse.clicked.connect(self.select_calibration); cal_upload = QPushButton("↑  UPLOAD CALIBRATION"); cal_upload.setObjectName("primary"); cal_upload.clicked.connect(self.upload_calibration); self.cal_browse_button = cal_browse; self.cal_upload_button = cal_upload; self.body.addWidget(QLabel("CALIBRATION RESULTS")); self.body.addWidget(self.calibration_files); cal_row = QHBoxLayout(); cal_row.addWidget(cal_browse); cal_row.addWidget(cal_upload); self.body.addLayout(cal_row); self.calibration_paths: list[str] = []

    def select_files(self) -> None:
        paths, _ = QFileDialog.getOpenFileNames(self, "Select map assets", str(ROOT), "Map assets (*.tif *.tiff *.png *.jpg *.yaml);;All files (*)")
        if paths: self.paths = paths; self.files.setPlainText("\n".join(paths))

    def upload(self) -> None:
        if self.paths: self.device.upload_assets(self.paths)

    def select_basemap(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "选择客户端 GPS 对比底图", str(ROOT), "底图 (*.png *.jpg *.jpeg *.tif *.tiff);;所有文件 (*)")
        if path:
            self.basemap_path = path; self.basemap.setText(path); self.basemap_selected.emit(path)

    def select_calibration(self) -> None:
        paths, _ = QFileDialog.getOpenFileNames(self, "Select calibration results", str(ROOT), "Calibration (*.yaml *.yml *.json *.xml *.txt);;All files (*)")
        if paths: self.calibration_paths = paths; self.calibration_files.setPlainText("\n".join(paths))

    def upload_calibration(self) -> None:
        if self.calibration_paths: self.device.upload_calibration(self.calibration_paths)


class ModePage(PageBase):
    def __init__(self, device: DeviceAdapter) -> None:
        super().__init__("POSITIONING MODES", "Select the algorithm replacement GPS source")
        self.mode = QComboBox(); self.mode.addItems(["MODE 1 · Serial GPS / NMEA or ULBOX", "MODE 2 · MAVLink HIL_GPS"]); self.body.addWidget(QLabel("ACTIVE SOURCE")); self.body.addWidget(self.mode); self.port = QLineEdit("/dev/ttyUSB0"); self.port.setPlaceholderText("Serial device (mode 1)"); self.body.addWidget(self.port); apply = QPushButton("APPLY POSITIONING MODE"); apply.setObjectName("primary"); apply.clicked.connect(lambda: device.log.emit(f"定位模式已选择: {self.mode.currentText()}")); self.body.addWidget(apply)


class SshTerminal(QObject):
    output = pyqtSignal(str)
    state = pyqtSignal(bool, str)

    def __init__(self) -> None:
        super().__init__(); self.client = None; self.channel = None; self.running = False

    def connect(self, host: str, username: str, password: str) -> None:
        if paramiko is None:
            self.state.emit(False, "paramiko 未安装，请执行 python -m pip install paramiko"); return
        def worker() -> None:
            try:
                self.client = paramiko.SSHClient(); self.client.set_missing_host_key_policy(paramiko.WarningPolicy())
                self.client.connect(host, username=username, password=password, timeout=6, look_for_keys=True, allow_agent=True)
                self.channel = self.client.invoke_shell(width=120, height=32); self.channel.settimeout(0.2); self.running = True; self.state.emit(True, f"SSH connected: {username}@{host}")
                while self.running:
                    try:
                        if self.channel.recv_ready(): self.output.emit(self.channel.recv(65535).decode("utf-8", "replace"))
                    except Exception: pass
                    time.sleep(0.03)
            except Exception as exc: self.state.emit(False, f"SSH connection failed: {exc}")
        threading.Thread(target=worker, daemon=True).start()

    def send(self, command: str) -> None:
        if self.channel and self.running:
            try: self.channel.send(command + "\n")
            except Exception as exc: self.output.emit(f"\n[send error] {exc}\n")

    def close(self) -> None:
        self.running = False
        try:
            if self.channel: self.channel.close()
            if self.client: self.client.close()
        finally: self.channel = None; self.client = None; self.state.emit(False, "SSH disconnected")


class CommsPage(PageBase):
    def __init__(self, device: DeviceAdapter) -> None:
        super().__init__("LINK DIAGNOSTICS", "Connection quality, reconnect policy and device identity")
        self.device = device; self.terminal = SshTerminal(); self.status = Metric("LINK STATUS", "OFFLINE", "#ff6b7a"); self.latency = Metric("ROUND TRIP", "-- ms"); self.body.addWidget(self.status); self.body.addWidget(self.latency)
        form = QHBoxLayout(); self.host = QLineEdit("155.155.0.14"); self.user = QLineEdit("firefly"); self.password = QLineEdit(); self.password.setEchoMode(QLineEdit.Password); self.password.setPlaceholderText("SSH password"); self.ssh_button = QPushButton("CONNECT SSH"); self.ssh_button.setObjectName("primary"); self.ssh_button.clicked.connect(self.connect_ssh); form.addWidget(QLabel("HOST")); form.addWidget(self.host); form.addWidget(QLabel("USER")); form.addWidget(self.user); form.addWidget(QLabel("PASSWORD")); form.addWidget(self.password); form.addWidget(self.ssh_button); self.body.addLayout(form)
        self.console = QPlainTextEdit(); self.console.setReadOnly(True); self.console.setObjectName("terminal"); self.console.setPlaceholderText("SSH terminal output"); self.body.addWidget(self.console)
        command = QHBoxLayout(); self.command = QLineEdit(); self.command.setPlaceholderText("Enter command, e.g. rosnode list"); self.command.returnPressed.connect(self.send_command); send = QPushButton("SEND"); send.clicked.connect(self.send_command); command.addWidget(self.command); command.addWidget(send); self.body.addLayout(command)
        self.terminal.output.connect(self.console.insertPlainText); self.terminal.state.connect(self.on_ssh_state)

    def update(self, t: Telemetry, connected: bool) -> None:
        self.status.set_value("ONLINE" if connected else "OFFLINE"); self.status.value.setStyleSheet("color:#43e6a0;" if connected else "color:#ff6b7a;"); self.latency.set_value(f"{t.link_ms} ms")

    def connect_ssh(self) -> None:
        if self.terminal.running: self.terminal.close()
        else: self.terminal.connect(self.host.text(), self.user.text(), self.password.text())

    def send_command(self) -> None:
        command = self.command.text().strip()
        if command: self.terminal.send(command); self.command.clear()

    def on_ssh_state(self, online: bool, message: str) -> None:
        self.ssh_button.setText("DISCONNECT SSH" if online else "CONNECT SSH"); self.console.appendPlainText(f"\n[{message}]\n")


class SettingsDialog(EmblemDialog):
    settings_changed = pyqtSignal(str, str)

    def __init__(self, theme: str, language: str, parent=None) -> None:
        super().__init__(parent); self.setWindowTitle("Settings / 设置"); self.setMinimumWidth(360)
        layout = QVBoxLayout(self); layout.addWidget(QLabel("DISPLAY THEME / 界面主题"))
        self.theme = QComboBox(); self.theme.addItem("Dark / 深色", "dark"); self.theme.addItem("Light / 浅色", "light"); self.theme.addItem("Blue / 蓝色", "blue"); self.theme.setCurrentIndex(max(0, self.theme.findData(theme))); layout.addWidget(self.theme)
        layout.addWidget(QLabel("LANGUAGE / 语言")); self.language = QComboBox(); self.language.addItem("中文", "zh"); self.language.addItem("English", "en"); self.language.setCurrentIndex(0 if language == "zh" else 1); layout.addWidget(self.language)
        info = QLabel("Version / 版本: 2.0.0\nProducer / 制作人: VisualNav Team"); info.setObjectName("muted"); layout.addWidget(info)
        apply_btn = QPushButton("APPLY / 应用"); apply_btn.setObjectName("primary"); apply_btn.clicked.connect(self.apply); layout.addWidget(apply_btn)

    def apply(self) -> None:
        self.settings_changed.emit(self.theme.currentData(), self.language.currentData()); self.accept()


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__(); self.simulating = False; self.device = DeviceAdapter(); self.ros_bridge = RosBridge(); self.image_seen = set(); self.setWindowTitle("NAV//STATION  •  Visual Navigation Ground Station"); self.resize(1440, 900); self._build(); self.device.telemetry_changed.connect(self.on_telemetry); self.device.connection_changed.connect(self.on_connection); self.device.log.connect(self.log); self.ros_bridge.telemetry.connect(self.on_telemetry); self.ros_bridge.image.connect(self.on_image); self.ros_bridge.error.connect(self.log)

    def _build(self) -> None:
        root = QWidget(); self.setCentralWidget(root); layout = QHBoxLayout(root); layout.setContentsMargins(0, 0, 0, 0)
        nav = QFrame(); nav.setObjectName("sidebar"); nav.setFixedWidth(210); nv = QVBoxLayout(nav); brand = QLabel("NAV//STATION\nV2.0"); brand.setObjectName("brand"); nv.addWidget(brand)
        self.theme_name, self.language = "light", "zh"
        self.sim_button = QPushButton("▶  模拟仿真"); self.sim_button.setCheckable(True); self.sim_button.clicked.connect(self.toggle_simulation); nv.addWidget(self.sim_button)
        self.sim_timer = QTimer(self); self.sim_timer.setInterval(100); self.sim_timer.timeout.connect(self.simulation_tick)
        self.connection = QPushButton("●  CONNECT SSH"); self.connection.clicked.connect(self.toggle_connection); nv.addWidget(self.connection); self.roscore_button = QPushButton("▶  START ROSCORE"); self.roscore_button.clicked.connect(self.start_roscore); nv.addWidget(self.roscore_button); self.nav = QListWidget(); self.nav.setObjectName("nav"); self.nav_labels = ["00  MISSION CONTROL", "01  SENSOR LAB", "02  MAP & ASSETS", "03  POSITIONING", "04  LINK DIAGNOSTICS"]; labels = self.nav_labels
        for label in labels: self.nav.addItem(QListWidgetItem(label))
        self.nav.setCurrentRow(0); nv.addWidget(self.nav); nv.addStretch(); self.system = QLabel("SYSTEM READY\n等待连接真实设备"); self.system.setObjectName("muted"); nv.addWidget(self.system); self.settings_button = QPushButton("⚙  SETTINGS / 设置"); self.settings_button.clicked.connect(self.open_settings); nv.addWidget(self.settings_button); layout.addWidget(nav)
        self.stack = QStackedWidget(); self.pages = [MainPage(self.device, self.stop_all_sensors), SensorPage(self.device), UploadPage(self.device), ModePage(self.device), CommsPage(self.device)]
        for page in self.pages: self.stack.addWidget(page)
        self.pages[0].control_window.layout().insertWidget(1, self.connection)
        self.pages[0].control_window.layout().insertWidget(2, self.roscore_button)
        self.pages[2].basemap_selected.connect(self.load_basemap)
        self.nav.currentRowChanged.connect(self.stack.setCurrentIndex); layout.addWidget(self.stack, 1)
        self.log_view = QPlainTextEdit(); self.log_view.setReadOnly(True); self.log_view.setFixedHeight(92); self.log_view.setObjectName("log")
        self.statusBar().addPermanentWidget(QLabel("  EVENT STREAM  ")); self.statusBar().addPermanentWidget(self.log_view, 1)

    def toggle_simulation(self, enabled):
        if enabled:
            if self.device.connected or self.ros_bridge.started or self.pages[4].terminal.client:
                self.sim_button.setChecked(False)
                self.log("请先断开板卡和诊断终端，再开启离线模拟仿真")
                return
            self.simulating = True
            self.sim_started = time.monotonic()
            self.sim_frame = 0
            m = self.pages[0].map
            keys = ("map_image", "map_transform", "map_crs", "fcu_path", "algo_path", "last_fcu", "last_algo", "last_telemetry", "zoom", "follow_gps", "demo_coordinates", "pan_center")
            self.saved_map = {key: getattr(m, key) for key in keys}
            # A generated terrain works on fresh clones without GeoTIFF or ROS.
            rng = np.random.default_rng(42)
            terrain = np.full((1000, 1600, 3), (156, 177, 143), np.uint8)
            for y in range(0, 1000, 100):
                for x in range(0, 1600, 120):
                    color = tuple(int(v) for v in rng.integers((110, 135, 100), (185, 200, 165)))
                    cv2.rectangle(terrain, (x+4, y+4), (x+113, y+93), color, -1)
            cv2.line(terrain, (0, 640), (1600, 360), (195, 211, 224), 90)
            cv2.line(terrain, (0, 200), (1600, 800), (221, 218, 204), 22)
            for x in range(120, 1500, 180):
                cv2.rectangle(terrain, (x, 180), (x+70, 240), (211, 205, 193), -1)
            m.map_image = terrain; m.map_transform = None; m.map_crs = None
            m.demo_coordinates = True; m.fcu_path = []; m.algo_path = []
            m.last_fcu = m.last_algo = m.last_telemetry = None
            m.follow_gps = False; m.follow_button.setChecked(False); m.follow_button.setText("固定视图")
            m.zoom_reset()
            self.sim_button.setText("■  退出模拟仿真")
            self.system.setText("SIMULATION / 模拟仿真\n所有数据为离线演示")
            # Leave display windows accessible while preventing board actions.
            self.sim_disabled = []
            for root in (self.pages[0].control_window, *self.pages[1:]):
                for button in root.findChildren(QPushButton):
                    self.sim_disabled.append((button, button.isEnabled()))
                    button.setEnabled(False)
            self.stack.setCurrentIndex(0); self.nav.setCurrentRow(0)
            self.simulation_tick(); self.sim_timer.start()
            self.log("模拟仿真已开启：10 Hz 动态姿态、GPS 双轨迹、测距和 RGB/红外演示图像")
        else:
            if not self.simulating: return
            self.sim_timer.stop(); self.simulating = False
            m = self.pages[0].map
            for key, value in self.saved_map.items(): setattr(m, key, value)
            m.drag_position = None; m.view.setCursor(Qt.OpenHandCursor)
            m.follow_button.setChecked(m.follow_gps)
            m.follow_button.setText("跟随 GPS" if m.follow_gps else "固定视图")
            m.view.clear()
            if m.map_image is not None: m.render_map()
            else: m.view.setText("等待加载地图 / 等待真实 ROS GPS")
            self.pages[0].clear_telemetry(); self.image_seen.clear()
            for button, enabled_before in self.sim_disabled: button.setEnabled(enabled_before)
            self.pages[4].update(Telemetry(), False)
            self.sim_button.setChecked(False); self.sim_button.setText("▶  模拟仿真")
            self.system.setText("SYSTEM READY\n等待连接真实设备")
            self.log("模拟仿真已退出，已恢复原地图和待机状态")

    def simulation_tick(self):
        if not self.simulating: return
        elapsed = time.monotonic() - self.sim_started
        phase = elapsed * 0.24
        x = 800 + 450 * math.cos(phase)
        y = 500 + 230 * math.sin(phase)
        now = time.monotonic()
        t = Telemetry(
            roll=12*math.sin(phase*2), pitch=5*math.cos(phase), yaw=(math.degrees(phase)+90)%360,
            altitude=120+8*math.sin(phase), speed_fcu=12+math.sin(phase), speed_algo=12.1+math.sin(phase),
            fcu_lat=36.01-y*0.00001, fcu_lon=120+x*0.00001,
            algo_lat=36.01-(y+3*math.sin(phase*3))*0.00001,
            algo_lon=120+(x+4*math.cos(phase*2))*0.00001,
            fcu_gps_valid=True, algo_gps_valid=True, range_m=85+5*math.sin(phase),
            received={key: now for key in ("gps", "algorithm", "imu", "range", "state", "velocity")},
            flight_state="[模拟] AUTO · 巡航飞行", simulated=True)
        self.on_telemetry(t)
        m = self.pages[0].map
        # Simulate a moving downward camera over the generated terrain.
        cx, cy = int(x), int(y)
        rgb = cv2.resize(m.map_image[cy-100:cy+100, cx-150:cx+150], (640, 360))
        cv2.putText(rgb, "SIMULATION / RGB", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, .65, (255,255,255), 2)
        cv2.drawMarker(rgb, (320,180), (255,255,255), cv2.MARKER_CROSS, 30, 1)
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        ir = cv2.cvtColor(cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO), cv2.COLOR_BGR2RGB)
        cv2.putText(ir, "SIMULATION / IR", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, .65, (255,255,255), 2)
        for tag, frame, panel in (("rgb", rgb, self.pages[0].rgb), ("ir", ir, self.pages[0].ir)):
            panel.set_frame(frame); panel.status.setText("● SIMULATED / 模拟")
            self.pages[0].images_at[tag] = now
        self.pages[0].refresh()
        self.pages[4].status.set_value("SIMULATION / 模拟")
        self.sim_frame += 1

    def toggle_connection(self) -> None:
        if self.simulating: return
        if self.device.connected: self.device.disconnect()
        else: self.device.connect_board("155.155.0.14", "firefly", "firefly")

    def open_settings(self) -> None:
        dialog = SettingsDialog(self.theme_name, self.language, self)
        dialog.settings_changed.connect(self.apply_settings); dialog.exec_()

    def apply_settings(self, theme: str, language: str) -> None:
        self.theme_name, self.language = theme, language; apply_theme(QApplication.instance(), theme)
        if language == "zh":
            labels = ["00  主控总览", "01  传感器调试", "02  地图与资产", "03  定位模式", "04  通讯诊断"]
            self.connection.setText("●  断开 SSH" if self.device.connected else "●  连接 SSH"); self.roscore_button.setText("▶  启动 ROSCORE"); self.settings_button.setText("⚙  设置"); self.setWindowTitle("NAV//STATION  ·  视觉导航地面站")
        else:
            labels = self.nav_labels; self.connection.setText("●  DISCONNECT SSH" if self.device.connected else "●  CONNECT SSH"); self.roscore_button.setText("▶  START ROSCORE"); self.settings_button.setText("⚙  SETTINGS / 设置"); self.setWindowTitle("NAV//STATION  ·  Visual Navigation Ground Station")
        for i, label in enumerate(labels): self.nav.item(i).setText(label)
        self.apply_chinese_widgets(language == "zh")

    def apply_chinese_widgets(self, chinese: bool) -> None:
        main = self.pages[0]
        if chinese:
            main.sensor_btn.setText("▶  启动全部传感器"); main.stop_sensor_btn.setText("■  关闭全部传感器"); main.fcu_btn.setText("▶  启动飞控驱动"); main.rgb_algo_btn.setText("▶  白天 / RGB 算法"); main.ir_algo_btn.setText("▶  黑夜 / 红外算法"); main.stop_algo_btn.setText("■  停止算法")
            self.pages[1].table.setHorizontalHeaderLabels(["传感器", "ROS 话题", "频率", "状态", "启动", "停止", "测试"])
            self.pages[1].config_editor.setPlaceholderText("编辑算法 YAML/JSON 配置")
            upload = self.pages[2]; upload.files.setPlaceholderText("选择的地图文件将显示在这里"); upload.calibration_files.setPlaceholderText("标定文件：内参 / 外参"); upload.browse_button.setText("＋  选择地图资产"); upload.upload_button.setText("↑  上传到板卡"); upload.cal_browse_button.setText("＋  选择标定文件"); upload.cal_upload_button.setText("↑  上传标定结果")
            self.pages[2].setWindowTitle("地图与资产")
            main.device.log.emit("界面语言已切换为中文")
        else:
            main.sensor_btn.setText("▶  START ALL SENSORS"); main.stop_sensor_btn.setText("■  STOP ALL SENSORS"); main.fcu_btn.setText("▶  START FLIGHT DRIVER"); main.rgb_algo_btn.setText("▶  DAY / RGB ALGORITHM"); main.ir_algo_btn.setText("▶  NIGHT / IR ALGORITHM"); main.stop_algo_btn.setText("■  STOP ALGORITHM")
            self.pages[1].table.setHorizontalHeaderLabels(["SENSOR", "ROS TOPIC", "RATE", "STATUS", "START", "STOP", "TEST"])

    def on_connection(self, connected: bool) -> None:
        if self.simulating and connected: self.toggle_simulation(False)
        if not connected: self.ros_bridge.stop(); self.pages[0].clear_telemetry()
        self.connection.setText("●  DISCONNECT SSH" if connected else "●  CONNECT SSH"); self.system.setText(("SSH ONLINE" if connected else "SYSTEM READY") + "\nClick START ROSCORE for ROS")

    def start_roscore(self) -> None:
        if self.simulating: return
        if not self.device.connected: self.log("请先连接 SSH"); return
        self.device.start_roscore(); QTimer.singleShot(1800, lambda: self.ros_bridge.start(self.device.host))

    def on_image(self, tag: str, frame: np.ndarray) -> None:
        if self.simulating: return
        panel = self.pages[0].rgb if tag == "rgb" else self.pages[0].ir; panel.set_frame(frame); self.pages[0].images_at[tag] = time.monotonic()
        if tag not in self.image_seen:
            self.image_seen.add(tag); self.log(f"{tag.upper()} 图像流正常: {frame.shape[1]} × {frame.shape[0]}")

    def load_basemap(self, path: str) -> None:
        if self.simulating:
            self.log("请退出模拟仿真后更换真实地图"); return
        ok, details = self.pages[0].map.load_basemap(path)
        if ok: self.log(f"GPS 对比底图已显示到主页面: {details}")
        else: self.log(f"GPS 对比底图加载失败: {details}")

    def on_telemetry(self, t: Telemetry) -> None:
        if self.simulating != t.simulated: return
        self.pages[0].update(t); self.pages[4].update(t, self.device.connected)

    def stop_all_sensors(self) -> None:
        if self.simulating: return
        self.device.start_sensors(False)
        self.ros_bridge.stop()
        self.pages[0].clear_telemetry()
        self.log("已停止 RGB、红外、ToF 及 MAVROS；roscore 保持运行")

    def log(self, message: str) -> None:
        self.log_view.appendPlainText(f"[{time.strftime('%H:%M:%S')}] {message}")


def apply_theme(app: QApplication, theme: str = "light") -> None:
    palettes = {"dark": ("#0b111c", "#0e1725", "#101b2a", "#0d1826", "#61dcff"), "light": ("#eef2f6", "#ffffff", "#ffffff", "#f7f9fb", "#16769b"), "blue": ("#071a2d", "#08243d", "#0d2d49", "#0a2238", "#55c8ff")}
    bg, sidebar, panel, field, accent = palettes.get(theme, palettes["dark"])
    app.setStyleSheet(f"""
    * {{ font-family: 'Segoe UI', 'Microsoft YaHei'; color: #dce8f5; font-weight:600; }}
    #aircraftMini {{ background:transparent; border:0; }}
    QMainWindow, QWidget {{ background: {bg}; }}
    QLabel {{ background:transparent; }} #missionOverlay, #missionTools {{ background:transparent; }} QStatusBar {{ background:#080d16; color:#7e8ca3; }}
    #sidebar {{ background:{sidebar}; border-right:1px solid #1d3047; }} #brand {{ color:{accent}; font-size:18px; font-weight:700; padding:18px 12px; }}
    #nav {{ background:transparent; border:0; padding:8px; }} #nav::item {{ padding:14px 10px; color:#91a4bb; }} #nav::item:selected {{ background:#15344b; color:#61dcff; border-left:2px solid #61dcff; }}
    QPushButton {{ background:#215575; border:1px solid #448ab4; padding:5px 9px; border-radius:3px; }} QPushButton:hover {{ border-color:#59d9ff; }} #primary {{ background:#0b6f91; border-color:#3abfe7; }}
    #telemetryTable {{ background:{panel}; border:1px solid #1c3852; border-radius:8px; }}
    #telemetryRow {{ background:transparent; border:0; border-bottom:1px solid #38536b; }}
    #panel, #metric {{ background:{panel}; border:1px solid #1c3852; border-radius:4px; }} #metricTitle, #muted {{ color:#7e8ca3; font-size:11px; }} #metricValue {{ font-size:18px; font-weight:600; }}
    #pageTitle {{ font-size:27px; font-weight:600; color:#f2f7fb; }} #mapView, #image {{ background:#0a1421; border:1px solid #1d415b; color:#4b7895; }}
    QTableWidget, QPlainTextEdit, QLineEdit, QComboBox {{ background:{field}; border:1px solid #203b55; padding:8px; }} QHeaderView::section {{ background:#15283d; padding:8px; border:0; }}
    #log {{ background:#080d16; color:#76e6b0; border:0; font-family:Consolas; font-size:11px; }}
    #terminal {{ background:#050a11; color:#8ff0b5; border:1px solid #28425d; font-family:Consolas; font-size:12px; }}
    #spark {{ background:#0a1421; border:1px solid #1d415b; }} #mapTitle {{ font-size:14px; font-weight:600; }}
    """)

    if theme == "light":
        app.setStyleSheet(app.styleSheet() + """
        * { color:#171b21; font-size:12px; font-weight:600; }
        QMainWindow, QWidget { background:#ffffff; }
        QLabel, #missionOverlay, #missionTools { background:transparent; }
        #sidebar { background:#fafbfc; border-right:1px solid #e5e7eb; }
        #brand { color:#101820; font-size:15px; font-weight:800; padding:16px 8px; }
        #nav { background:transparent; font-weight:700; }
        #nav::item { color:#18222e; padding:12px 8px; }
        #nav::item:selected { color:#111827; background:#edf1f5; border-left:2px solid #7d95aa; }
        QPushButton { color:#101820; font-weight:700; background:rgba(255,255,255,210);
            border:1px solid #d6dce3; border-radius:6px; padding:5px 9px; }
        QPushButton:hover { background:#edf4fa; border-color:#98b5cc; }
        QPushButton:pressed, QPushButton:checked { background:#e3edf6; }
        QPushButton:disabled { color:#a0a6ae; background:#f5f6f8; border-color:#e7eaee; }
        #primary { background:#edf4fa; color:#20262e; border-color:#b9cddd; }
        #panel { background:#ffffff; border:1px solid #e0e5eb; border-radius:8px; }
        #metric, #telemetryTable {
            background:qlineargradient(x1:0,y1:0,x2:0.8,y2:1,
                stop:0 rgba(255,255,255,195), stop:0.48 rgba(255,255,255,140),
                stop:1 rgba(235,244,250,165));
            border:1px solid rgba(255,255,255,225); border-radius:12px;
        }
        #telemetryRow { background:transparent; border:0; border-bottom:1px solid rgba(100,120,140,55); border-radius:0; }
        #metricTitle { color:#414a54; font-size:10px; font-weight:600; }
        #metricValue { color:#111820; font-size:12px; font-weight:700; }
        #muted { color:#374151; font-size:11px; font-weight:700; }
        #pageTitle { color:#181e26; font-size:21px; font-weight:700; }
        #mapTitle { color:#101820; font-size:12px; font-weight:700; }
        #mapView, #image { background:#f0f3f6; border:1px solid #e0e5eb; color:#66717e; }
        QTableWidget, QPlainTextEdit, QLineEdit, QComboBox {
            color:#20262e; background:rgba(255,255,255,220);
            border:1px solid #dfe4ea; selection-background-color:#dcebf6;
            selection-color:#111827; padding:6px;
        }
        QHeaderView::section { color:#414a54; background:#f2f4f7; padding:7px; }
        QStatusBar, #log { color:#253140; background:#f8fafb; border:0; font-size:11px; font-weight:700; }
        QStatusBar QLabel { color:#101820; font-weight:700; }
        #terminal { color:#20262e; background:#f8fafb; border:1px solid #dfe4ea; font-size:11px; }
        #spark { background:#f2f4f7; border:1px solid #dfe4ea; }
        QScrollBar { background:#f3f5f7; }
        QScrollBar::handle { background:#c7cfd8; border-radius:3px; min-height:24px; }
        """)


def main() -> int:
    """Application entry point used by both the CLI and packaged executable."""
    self_test = "--self-test" in sys.argv
    if self_test: os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    app = QApplication(sys.argv)
    app.setApplicationName("NAV//STATION")
    app.setOrganizationName("VisualNav")
    apply_theme(app, "light")
    window = MainWindow()
    window.apply_settings("light", "zh")
    window.setWindowTitle("NAV//STATION V2.0 · 视觉导航地面站")
    if self_test:
        window.show(); app.processEvents()
        window.toggle_simulation(True); app.processEvents()
        assert window.pages[0].last.simulated
        assert not window.pages[0].rgb.image.pixmap().isNull()
        assert not QPixmap(str(ROOT / "assets" / "tsinghua-emblem.jpg")).isNull()
        assert rasterio is not None, "GeoTIFF support missing"
        assert rospy is not None, "ROS client support missing"
        from rasterio.io import MemoryFile
        with MemoryFile() as memory:
            with memory.open(driver="GTiff", width=4, height=4, count=1, dtype="uint8", crs="EPSG:4326", transform=Affine.translation(120, 36)*Affine.scale(.001, -.001)) as dataset:
                dataset.write(np.ones((1, 4, 4), dtype=np.uint8))
            with memory.open() as dataset:
                assert dataset.read(1).sum() == 16
        projected, _ = transform_coordinates("EPSG:4326", "EPSG:3857", [120], [36])
        assert 13000000 < projected[0] < 14000000
        window.toggle_simulation(False); window.close()
        print("SELF-TEST OK: Qt, OpenCV, GeoTIFF, ROS client, assets, simulation")
        return 0
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
