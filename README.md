# 灵眸 FPGA 视觉云台

2026 全国大学生嵌入式芯片与系统设计竞赛，安路 FPGA 创新设计赛道，赛题二：基于安路 PH1P35 的实时数字图像处理系统。

- 板卡：HX1P35A（PH1P35MDG324）
- 摄像头：SC500CS（2-lane MIPI RAW10）
- 开发工具：Anlogic TD 6.2.1

## 目录

```text
Software/
  FPGA_Projects/
    Video_D1/       视频算法、直方图、帧差运动检测、手动云台
    Tracking_D2/    绿色光点追踪、颜色/形状激光描边
  Host_Software/    上位机控制台（Python + HTML/CSS/JS）
Model/              模型文件（待上传）
```

## FPGA 工程

用 TD 6.2.1 打开 `Software/FPGA_Projects/<工程名>/td_project/camera_to_dsi_display.al`。已编译的位流在各工程的 `Bitstreams/` 目录。

TD 无法打开中文路径，请把仓库放在纯英文路径下。

## 上位机

原交付包中的 `Launch_GUI.cmd` 依赖随包 Python 运行时（`Host_Software/Runtime/`）。运行时体积较大，没有上传到本仓库，因此克隆后不能直接双击启动。

请使用 Python 3.14（原运行时版本，单元测试在此版本下通过），并确保系统已安装 Edge 或 Chrome。Python 3.12 下部分单元测试不通过，不建议使用。运行方式：

```powershell
cd Software\Host_Software
python App\desktop_server.py
```

## 未上传的内容

`Software` 下各 README 是原交付包说明，其中提到的以下内容没有上传：

- `Host_Software/Runtime/`：随包 Python 运行时
- `Host_Software/Logs/`：运行日志
- `Host_Software/Backups/`：开发过程备份
- `SHA256SUMS.json`：原交付包校验清单

## 第三方内容

本仓库包含第三方文件，其版权归各自所有者：

- 安路 TD 生成的 IP（`user_source/ip_source/`）
- 加密 IP（`user_source/hdl_source/` 下带 `pragma protect` 的文件，如 `*.enc.v`、`*.enc.sv`）
- pyserial（`Host_Software/Vendor/`）
- Three.js（`Host_Software/Web/assets/three.module.js`）
