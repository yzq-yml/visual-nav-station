# Windows 打包与使用（V2.0）

## 功能范围

本脚本生成 Windows x64 便携版，包括界面、模拟仿真、姿态动画、GeoTIFF 地图、SSH 终端、板卡远程命令和文件上传。SSH 功能需要到板卡的网络连接和有效登录凭据。

本包不包含 ROS Python 客户端：真实 GPS、IMU、飞控状态、相机图像不会因 SSH 连接成功而自动显示。完整实机版本目前在 Ubuntu 使用；Windows 原生实时接入需另行适配数据传输。模拟图像不能作为真实相机接入验证。

必须在 Windows 构建，Ubuntu 安装包不能改名为 exe 使用。当前仓库维护环境为 Ubuntu；脚本已准备，尚未在 Windows 执行构建与验收。

## 1. 准备构建电脑

使用 Windows 11 x64，或已有的 Windows 10 x64 环境。首次打包需联网。打开 https://www.python.org/downloads/windows/ ，选择 Python 3.13.x 的 Windows installer (64-bit)，安装时启用 Python Launcher（py）。不要选择 ARM64、32-bit、free-threaded 或 embeddable package。

在 PowerShell 执行：

```powershell
py -3.13 --version
```

应显示 Python 3.13.x。脚本固定使用此版本，不需要改动系统的其他 Python 环境。

## 2. 获取源码

访问 https://github.com/yzq-yml/visual-nav-station ，点击 Code → Download ZIP，解压到例如 `D:\NAVStation-source`。确认该目录内直接包含 main.py、requirements.txt、build_windows.ps1、assets 和 configs。

如果使用 Git，也可以：

```powershell
git clone https://github.com/yzq-yml/visual-nav-station.git D:\NAVStation-source
```

不要复制 Ubuntu 的 .venv、build、dist 或其他 Python 安装目录到 Windows。

## 3. 一键构建

在 PowerShell 执行（路径按实际修改）：

```powershell
cd D:\NAVStation-source
powershell -NoProfile -ExecutionPolicy Bypass -File .\build_windows.ps1
```

此处执行策略只对这次 PowerShell 进程生效，不需要管理员权限。脚本自动创建 `.venv-windows`，安装普通依赖和 PyInstaller，检查依赖，运行源码自检，打包资源和地图数据文件，再运行打包后自检。成功后生成：

```text
dist/
  NAVStation/
    NAVStation.exe
    _internal/
    install-shortcut.ps1
    WINDOWS.md
  NAVStation-2.0.0-windows-x64.zip
  NAVStation-2.0.0-windows-x64.zip.sha256
```

以脚本成功输出 `Built:` 为准。构建失败时不要分发旧的 dist 文件。

## 4. 本机验收

双击 `dist\NAVStation\NAVStation.exe`。检查：

1. 点击模拟仿真，确认地图双轨迹、姿态小飞机与右侧数据更新。
2. 打开 RGB / 红外图像，确认模拟画面更新。
3. 滚轮缩放、左键拖动，退出仿真后检查待机状态。
4. 在地图与资产中加载自己复制的 GeoTIFF。
5. 重启程序，检查文字、校徽背景及图像窗口。

自检包含模拟图像、资源加载、GeoTIFF 内存读写及坐标转换，但不能替代新电脑的图形和网络验收。

## 5. 复制到另一台 Windows 电脑

只需复制 ZIP，解压到固定目录，例如 `D:\Apps\NAVStation`，双击 NAVStation.exe。目标机不需要安装 Python、pip、PyQt 或 OpenCV。必须保留整个文件夹及 `_internal`，不要仅复制 exe，也不要直接在压缩包内启动。

地图不在 ZIP 内，另行复制 jimo.tif，并在软件内选择。地图路径必须是新电脑上的路径。

创建桌面快捷方式：在解压后的目录打开 PowerShell，执行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\install-shortcut.ps1
```

此后不要移动程序目录；移动后重新运行快捷方式脚本。

## 6. 排错

- `py` 找不到 / 没有 Python 3.13：重新检查 Python 安装器中的 Launcher，安装完成后重新打开 PowerShell。
- pip 下载失败：检查网络后重试；完整构建依赖需要首次下载。
- `PermissionError`：关闭已运行的 NAVStation.exe 后重新构建。
- 双击没有界面：构建带控制台版本以查看错误：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\build_windows.ps1 -Console
.\dist\NAVStation\NAVStation.exe
```

- `--self-test` 失败：脚本会停止。通过控制台版本定位缺失模块或资源，不要跳过失败步骤发布。
- SSH 能连接但没有图像/姿态：当前 Windows 包不含 ROS 订阅，这是已知功能范围，不是缺少普通 pip 依赖。

## 7. 后续更新

更新源码后再次运行同一个打包命令，将新 ZIP 复制到目标电脑。便携 ZIP 已能分发；带安装向导和卸载入口的 Setup.exe 属于后续封装步骤，应在 Windows 便携版验收通过后制作。

参考：
- https://pyinstaller.org/en/stable/ （PyInstaller 需要在目标操作系统上构建）
- https://www.python.org/downloads/windows/ （Python 官方 Windows 安装器）
