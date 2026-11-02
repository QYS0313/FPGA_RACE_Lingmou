# Run from either image's phy_1 directory, after successful routing.
# MBOOT_V2 instantiation alone does not enable the configuration controller.
# SWUG105 bitgen -g controls configuration registers; no hardware access here.
import_device ph1_35p.db -package PH1P35MDG324 -speed 3
open_project camera_to_dsi_display.prj -noanalyze
import_db camera_to_dsi_display_pr.db
bitgen -bit camera_to_dsi_display.bit -g {mult_boot_en:1}
exit
