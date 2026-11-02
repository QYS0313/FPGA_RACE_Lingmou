# 灵眸 · FPGA 实时视觉云台系统

以安路 PH1P35 FPGA 为核心的实时视觉云台：摄像头画面在 FPGA 内完成 ISP 与目标检测，双轴云台按目标偏差自动跟踪，激光振镜在目标周围投射红色矩形框，图像处理与运动控制全部由 FPGA 实时完成，PC 端只负责参数配置、标定和诊断记录。

## 功能概览

**视频处理模式**（Video_D1）：1280×720 RGB 视频流经 HDMI 实时显示，支持加权灰度化、Isodata 自适应二值化、带动态噪声阈值的 Sobel 边缘检测、HSV 多目标色块定位、帧差运动区域检测，以及直方图亮度增强、选区处理和区域特效。

**光点追踪模式**（Tracking_D2）：

- 目标检测：按亮度门限 + 绿色通道优势判据提取绿色激光光点，局部重捕获和紧凑度约束抑制反光干扰；红框投射期间自动关闭白核补充，避免红色反光误检。
- 云台跟踪：基于像素偏差的视觉伺服，每轴约 40 ms 更新一次；分段速度规划、停止/重启滞回、加减速限制、反向过零和俯仰轴起步补偿，减轻中心附近抖动。
- 振镜投影：四点标定建立像素坐标到 DAC 编码的仿射映射，DAC8563 双通道同步更新，128 点矩形扫描框选目标；红框中心限制在标定窗口内，云台始终用原始坐标追踪，二者解耦。
- 其他：上电自动回位、丢点与超时熄光、跨时钟域整组提交、串口状态诊断。

## 硬件

- 主控板 HX1P35A V1.0（PH1P35MDG324），板载 MIPI、DDR、HDMI，开发工具 TD 6.2
- 摄像头 SC500CS（2-lane MIPI RAW10）
- 双轴电机：水平轴（pan）与俯仰轴（tilt），各一路 UART，回传编码器角度、电流和状态
- DAC8563 + 双轴激光振镜与红色光源
- 两自由度云台结构，摄像头与振镜固定在同一俯仰箱体内，一次标定可持续使用

## 目录

```text
Software/
  FPGA_Projects/
    Video_D1/       视频处理模式工程（RTL、IP、约束、TD 工程、位流、报告）
    Tracking_D2/    光点追踪模式工程
  Host_Software/
    App/            上位机：desktop_server.py（入口）、demo_gui.py（Tk 版）、vision.py（命令行标定/跟踪/诊断）
    Web/            HTML/CSS/JS 控制台与三维装配模型
    Config/         calibration.json（视觉标定）、gimbal_model.json
    Tests/          离线单元测试与界面检查
Model/              结构件模型（STL）
```

## 运行上位机

仓库未包含随包 Python 运行时，`Launch_GUI.cmd` 无法直接启动。请使用 Python 3.14，并确保系统已安装 Edge 或 Chrome：

```powershell
cd Software\Host_Software
python App\desktop_server.py
```

FPGA 工程请放在纯英文路径下，用 TD 6.2.1 打开，TD 无法打开中文路径。

## License

本项目团队编写的代码和模型按 [MIT License](LICENSE) 开源。

以下第三方文件的版权归各自所有者，不在 MIT 许可范围内，使用时请遵守其原有条款：

- 安路 TD 生成的 IP（`user_source/ip_source/`）
- 加密 IP（`user_source/hdl_source/` 下带 `pragma protect` 的文件，如 `*.enc.v`、`*.enc.sv`）
- pyserial（`Host_Software/Vendor/`）
- Three.js（`Host_Software/Web/assets/three.module.js`）
