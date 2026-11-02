FPGA PROJECTS

Video_D1: D1/A9, video algorithms, histogram, frame-difference motion detection and manual gimbal.
Tracking_D2: D2/E7, green-spot tracking and colour/shape laser contour drawing.
Both projects include RTL, IP, constraints, TD project, final bitstream and build report.

Open each td_project/camera_to_dsi_display.al in TD 6.2.1.
Optional rebuild: Build.cmd -Image video (or tracking / both). Requires licensed Anlogic TD.
Rebuild outputs go to Rebuilt; shipped Bitstreams remain unchanged.

JTAG: select one bit to test one project. No programming is performed by this package.
128-Mbit Flash layout: Video_D1/Bitstreams/demo_video.bit at 0x00000000 (PROGRAM_SPI),
then Tracking_D2/Bitstreams/demo_tracking.bit at 0x00800000 (MULTI_BOOT).
Both shipped bits have mult_boot_en=1. Software image switching is not claimed hardware-verified.
Project_Notes.txt is the original development reference; its old workspace paths are historical.
