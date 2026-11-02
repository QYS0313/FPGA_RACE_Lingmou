灵眸 FPGA · HTML/CSS 三维视觉云台控制台

启动：双击 Launch_GUI.cmd。使用随包 Python 和系统 Edge 应用窗口，无需额外安装 Python 包。
界面、Three.js 和模型资源均保存在本地，不依赖互联网。请保留完整 Host_Software 文件夹。
旧版 Tk 控制台保留在 Launch_Legacy.cmd，作为兼容入口。
启动或打开离线演示不会自动连接串口、回位、使能电机或打开激光。

光点追踪工作区
  模型、双轴编码器反馈、运动曲线和跟踪操作位于同一工作区，不另开模型页面。
  模型直接提取自“装配体2_拆装动画预览_v2.html”：96个网格、170130个三角面。
  原始位置、三角形索引、零件分组和颜色数据完整保留；原文件在 Models/assembly2_original.html。
  可拖动旋转、滚轮缩放、复位视角，并在完整装配与内部结构间切换。
  底座固定；水平支架随PAN旋转；机壳、摄像头和内部板卡绕TILT轴俯仰。
  两轴参考角度与固件一致：PAN约116.922°、TILT180°。可在左下角“模型标定”调整显示零位和方向。
  显示映射不改变电机参数。反馈超时或连接中断时保持最后姿态并显示状态。
  模型显示方向为PAN正向、TILT反向，已修正模型与实物上下运动相反的问题；视频与追踪两页共用。
  未连接时可使用“离线动作演示”，角度、电流和曲线均明确标为演示数据。
  视觉标定使用 Config/calibration.json；更换文件会保存为 Config/calibration_imported.json。
  更改相机、振镜、幕布相对位置后，仍需按原流程重新视觉标定。

视频算法工作区
  右上角“模式选择”用于选择板卡工作模式。
  算法卡片在设备确认切换后显示绿色选中状态；读取设备参数也会同步当前算法。
  加入与光点追踪相同的三维装配模型，使用实际编码器反馈同步PAN/TILT姿态。
  手动运动时直接使用运动指令响应中的角度，页面切换保留模型视角与姿态。
  保留原图、灰度、二值化、Sobel、颜色跟踪、帧差运动检测、附加特效、图像增强、作用范围和选区设置。
  手动云台可按住方向按钮或WASD/方向键，松开减速停车；切换页面或窗口失焦会释放运动请求。
  设备视频仍从HDMI输出，本次未增加视频采集。
  ESC请求停止；回位、停止、失能、形状识别、静态描边、CSV记录和Flash切换沿用原单串口控制逻辑。
  Flash切换仍需先按工程说明烧入双镜像，不代表本次已做实机切换验证。

连接与记录
  选择实际COM端口后点击连接，自动识别D1/D2/E2。首次运动前按原流程回位和使能。
  运行事件与CSV仍保存在 Logs。关闭新版窗口会请求停止并断开；本地页面失联后后台会清理会话。
  页面只通过本机回环地址访问现有Python控制器。FPGA源码和正式位流未修改。

验证
  Self_Test.cmd：启动自检及28项离线单元测试，不连接串口。
  Tests/check_css_console.py：使用隔离的系统Chrome检查浏览器界面，不需要接板。
  浏览器中已检查模型完整性、两轴层级、转角、内部结构切换、WebGL和桌面布局。
  Screenshots/css_console_tracking.png：新版跟踪页（离线演示）。
  Screenshots/css_console_internal.png：内部结构视图（离线演示）。
  Screenshots/css_console_video.png：视频控制工作区。
  Reports/css_console_validation.json：本次验证记录。
  Reports/20261031_ui_update：模式选中和视频页模型更新的浏览器验证记录与预览截图。
  本次未打开设备串口，实际编码器方向与机械联动仍需接板确认。

Web：可编辑的HTML/CSS/JavaScript界面及本地模型资源。
App/desktop_server.py：新版入口，复用App/demo_gui.py中的单串口Worker。
Runtime：随包Python；Vendor：pyserial；Config：视觉及模型标定。
本轮前版本备份：Backups/before_css_console_20261030。
