# NAV//STATION V2.0

视觉导航装置 PyQt5 PC 上位机，支持通过 SSH 控制板卡并接收真实 ROS 遥测。未连接时显示等待数据。

## 离线模拟仿真

点击左侧“▶ 模拟仿真”，无需板卡、ROS 或本地地图即可查看 10 Hz 动态姿态、飞行状态、GPS / 算法双轨迹、ToF 测距及模拟 RGB / 红外图像。图像通过主页面“RGB / 红外图像”窗口查看，地图继续支持缩放。仿真使用生成的地形和图像，仅用于界面演示，不代表算法精度或真实红外传感器输出。

仿真期间明确显示“模拟”标识，设备操作按钮暂停使用。再次点击“退出模拟仿真”恢复原地图与待机状态。已连接板卡或诊断终端时，需要先断开再进入仿真。

## V2.0 界面

- 主页面地图作为整个工作区底层，鼠标滚轮缩放，按住鼠标左键可上下左右拖动地图；手动拖动自动退出 GPS 跟随，点击“跟随 GPS”恢复。支持双轨迹对比。
- 右侧固定 310 像素宽的竖向透明表格，依次显示姿态信息 / 导航信息 / 传感器状态 / 飞行信息，地图底部保持通透。
- “RGB / 红外图像”打开 960×400 固定窗口，“控制窗口”集中连接、ROSCORE、传感器、飞控、白天/夜晚算法和停止操作。
- 非主页使用清华大学官方校徽水印，默认白底黑字简约配色，地图信息面板使用高透明白色玻璃渐变，小字号显示。
- 姿态由 IMU 四元数计算；飞控状态、速度分别订阅 `/mavros/state`、`/mavros/local_position/velocity_local`。超过 3 秒未更新显示超时，未订阅的数据不显示为有效数值。
- 在“地图与资产”选择本地 GeoTIFF 后显示背景地图；大地图文件不纳入 Git。

## GitHub 同步

提供仓库 URL，并在本机完成 GitHub SSH 密钥授权，然后可配置远程并推送。不要提供帐号密码或将令牌写入源码。当前 `.gitignore` 排除 GeoTIFF、构建产物和常见凭据文件。开源许可证需由项目所有者选定；校徽权利单独归清华大学。

## 页面

- `00 Mission Control`：连接、地图/GPS 对比、飞控姿态与 GPS、飞控/算法速度、RGB/红外预览、IMU/测距扩展位、传感器和算法控制。
- `01 Sensor Lab`：按传感器检查 ROS 话题、频率和独立测试开关。
- `02 Map & Assets`：选择匹配图、DSM、GeoTIFF 底图并上传到板卡的接口占位。
- `03 Positioning`：串口 NMEA/ULBOX 与 MAVLink HIL_GPS 两种模式。
- `04 Link Diagnostics`：连接质量、延迟和重连入口。
  - 内嵌 SSH 终端：填写主机、用户名和密码后连接，可直接执行板卡诊断命令。

## 两页项目介绍 PPT

已生成项目介绍文件：[visual_nav_station_overview.pptx](visual_nav_station_overview.pptx)。

PPT 共两页：

- 第 1 页：视觉导航地面站定位、系统数据链路、主页面和 GPS 对比地图能力。
- 第 2 页：当前已实现功能、量产化建议和后续开发优先级。

如需修改内容或重新生成，可执行：

```bash
/usr/bin/python3.8 generate_ppt.py
```

后续建议按以下顺序推进：健康监控（节点、话题频率、延迟、板卡资源）→ 设备配置中心（每台设备独立 YAML）→ 轨迹回放与误差报告 → 配置备份、权限和软件安装包封装。

## 运行

```bash
python3 --version  # Python 3.8+
python3 -m pip install --user -r requirements.txt
/usr/bin/python3.8 main.py
```

如果使用 Conda，请先执行 `conda activate <环境名>`，再用 `python -m pip install -r requirements.txt`。确认 `which python` 与 `python -m pip --version` 指向同一个环境；Ubuntu 需要额外安装 `python3.8-venv` 后才能创建 Python 3.8 虚拟环境。

本机 ROS Noetic、`rospy` 和 GeoTIFF 的 `rasterio` 均位于系统 Python 3.8 环境，因此接入板卡和 GPS 对比底图时应使用 `/usr/bin/python3.8 main.py`。地图页面选择的客户端 GeoTIFF 不会上传板卡；它会显示在主页面 GPS 对比区域，并依据 GeoTIFF CRS/仿射变换实时绘制 MAVROS 与 GeoVINS 轨迹和定位误差。

也可以以标准应用命令运行（开发环境）：

```bash
pip install -e .
nav-station
```

## 打包发布

QGC 是 C++/Qt 应用，发布出来是安装包或 AppImage/压缩包；本项目先采用同样的用户体验，但使用 Python/Qt 实现。推荐先用 PyInstaller 做可移植目录包，验证稳定后再做安装器：

- Windows：PowerShell 执行 `./build_windows.ps1`，产物在 `dist/NAVStation/`。
- Linux：执行 `./build_linux.sh`，产物在 `dist/nav-station/`。

发布时把整个 `dist` 目录交给用户，后续可用 Inno Setup（Windows）或 AppImage/DEB（Linux）封装安装、桌面快捷方式和卸载入口。不要把板卡密码写进可执行文件；设备配置应作为 `configs/devices/*.yaml` 或首次启动时导入。

## 推荐演进方式

当前 `main.py` 已有 `main()` 入口，后续应逐步迁移为 `src/visual_nav_station/`：`ui/` 放页面，`application/` 放状态机，`infrastructure/` 放 ROS、设备 API 和上传实现。这样打包命令不变，页面和板卡协议可以独立演进。QGC 式的插件化、日志、版本检查和自动升级属于第二阶段，不需要第一天就全部实现。

默认板卡地址为 `155.155.0.14`。第一版的 `DeviceAdapter` 是模拟实现，连接按钮只更新本地状态；后续将把 ROS 订阅、SSH/API 控制和文件上传集中替换到该类，不在页面中散落命令。

当前板卡已接入真实 SSH 启动命令：传感器按钮启动 RGB、红外和 ToF，飞控按钮启动 `roslaunch mavros px4.launch`，算法按钮启动 `geovins_qingdao_ws` 的 GeoVINS，并显式使用设备现有的 `1.260810_veye_km220` 或 `2.260810_airui_km220` 配置。RGB 是白天模式，红外是黑夜模式，二者互斥，启动一个会先停止另一个。ToF 有效测距范围为 5–1200 m，室内近距离无数据属于正常；GPS 需要室外卫星信号，室内无 fix 属于正常。启动日志写入板卡 `~/results/ground_station_logs/`。

连接板卡后，上位机会通过 SSH 检查并启动板卡上的唯一 `roscore`，随后订阅真实 ROS 话题。传感器页的 `TEST TOPIC` 会在板卡执行一次 `rostopic echo -n 1`，结果和退出码显示在事件流中。

如果在 PC 上单独运行 `rqt_image_view`，需要先把 ROS master 指向板卡（把 `192.168.1.20` 换成 PC 在设备网口上的实际 IP）：

```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://155.155.0.14:11311
export ROS_IP=192.168.1.20
rqt_image_view
```

`rqt_image_view` 只在 ROS master 可访问且 PC 与板卡网络互通时显示图像；`Connection refused` 表示 master 地址或网络尚未就绪，不代表 RGB 节点被删除。

## 目录约定

设备差异配置放在 `configs/devices/*.yaml`，机型默认配置放在 `configs/models/*.yaml`。真实接入前需要补充：ROS master 地址、话题映射、launch/service 名称、串口设备、标定文件和地图上传目标目录。

## 开发检查

```bash
python -m py_compile main.py
```

量产前还需要加入 pytest、日志文件、设备版本查询、SSH 主机密钥校验、systemd 服务控制和上传校验/回滚。
