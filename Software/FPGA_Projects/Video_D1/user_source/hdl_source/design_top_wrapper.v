

module design_top_wrapper (
    input wire        I_sys_clk,
    input wire        I_rst_n,

    input wire I_pan_rx,I_tilt_rx, output wire O_pan_tx,O_tilt_tx,
    input wire        I_uart_rx,
    output wire       O_uart_tx,
    output wire       O_dac_clr_n,
    output wire       O_dac_sync_n,
    output wire       O_dac_sclk,
    output wire       O_dac_din,
    output wire       O_dac_ldac_n,
    output wire       O_laser_enable,
      
    output wire       O_cam_scl,
    inout  wire       IO_cam_sda,
    output wire       O_cam_24m,
    output wire       O_cam_rst,
      
    inout wire [1:0]  I_button,

    output wire       O_screen_pwm,
    output wire       O_tmds_ch0_p,
    output wire       O_tmds_ch1_p,
    output wire       O_tmds_ch2_p,
    output wire       O_tmds_clk_p,

    inout wire        IO_rx_clk_pad_n, 
    inout wire        IO_rx_clk_pad_p, 
    inout wire[3:0]   IO_rx_data_pad_n,
    inout wire[3:0]   IO_rx_data_pad_p,

    output wire[12:0] ddr_addr,
    output wire[ 1:0] ddr_ba,
    output wire[ 0:0] ddr_cke,
    output wire[ 0:0] ddr_odt,
    output wire[ 0:0] ddr_cs_n,
    output wire       ddr_ras_n,
    output wire       ddr_cas_n,
    output wire       ddr_we_n,
    output wire[ 0:0] ddr_ck_p,
    output wire[ 0:0] ddr_ck_n,
    inout wire[1:0]   ddr_dm,
    inout wire[15:0]  ddr_dq,
    inout wire[1:0]   ddr_dqs_p,
    inout wire[1:0]   ddr_dqs_n   
);


    wire        	S_100m_clk;
    wire        	S_24m_clk;
    wire        	S_aux_50m_clk;
	wire        	S_10m_clk;
		
	wire        	S_pll_lock;
    wire            S_hdmi_pll_lock;
    wire        	S_rst_n;
	
	wire [15:0] 	S_ae;
    wire [15:0] 	S_ag;
    wire 			S_cam_cfg_done;
    wire 			S_ae_cfg_done;
    wire 			S_ae_req; 
	
	wire        	S_csi_rx_clk;  
    wire        	S_hs_rx_valid;  
    wire[15:0]  	S_hs_rx_data;   
	wire[1:0]   	S_lane_error;
	
	wire        	S_csi_frame_start;       
	wire        	S_csi_frame_end;         
	wire        	S_csi_valid;             
	wire[31:0]  	S_csi_data;  
		
	wire        	S_raw10_frame_start; 
	wire        	S_raw10_frame_end;   
	wire        	S_raw10_valid;       
	wire [39:0] 	S_raw10_data;
	wire        	S_raw720_frame_start;
	wire        	S_raw720_frame_end;
	wire        	S_raw720_valid;
	wire [39:0] 	S_raw720_data;
		
	wire 			S_axis_tlast;  
	wire 			S_axis_tuser;  
	wire [39:0]		S_axis_tdata;  
	wire 			S_axis_tvalid; 
	
	wire [39:0]		S_raw_tdata ;
	wire			S_raw_tlast ;
	wire 			S_raw_tvalid;
	wire 			S_raw_tuser ;
	
	wire			S_ISP_O_tready;
	wire [127:0]	S_ISP_O_tdata ;
	wire 			S_ISP_O_tlast ;
	wire 			S_ISP_O_tuser ;
	wire 			S_ISP_O_tvalid; 
	reg  [2047:0] S_isp_data_delay;
	reg  [15:0]   S_isp_valid_delay;
	wire [127:0]  S_isp_safe_data;
	wire          S_isp_safe_valid;
	

    wire        S_ddr_clk;
    wire        S_hdmi_pixel_clk;
    wire        S_hdmi_serial_clk;
    wire        S_hdmi_rst_n;
    wire        S_video_out_rst_n;


    wire        S_vi_128b_frame_start;
    wire        S_vi_128b_valid;      
    wire[127:0] S_vi_128b_data;    
    
    wire        S_video_out_rd_busy;
    wire        S_video_in_wr_busy; 
    wire[1:0]   S_video_out_rp;     

    wire        S_ddr_user_wr_en;       
    wire        S_ddr_user_rd_en;       
    wire[24:0]  S_ddr_user_addr;        
    wire[127:0] S_ddr_user_wr_data;     
    wire        S_ddr_user_ready;       
    wire        S_ddr_user_rd_valid;    
    wire[127:0] S_ddr_user_rd_data;     

    wire        S_vi_ddr_wr_en;   
    wire[24:0]  S_vi_ddr_wr_addr; 
    wire[127:0] S_vi_ddr_wr_data;
    wire        S_vi_ddr_wr_ready;

    wire        S_vo_ddr_rd_en;   
    wire[24:0]  S_vo_ddr_rd_addr; 
    wire        S_vo_ddr_rd_ready;
    wire        S_vo_dbg_fifo_rst;
    wire        S_vo_dbg_ddr_rd_valid;
    wire        S_vo_dbg_fifo_rd_en;
    wire        S_vo_dbg_fifo_empty;

    wire        S_init_calib_complete; 
    wire[24:0]  S_mc_app_addr;          
    wire[2:0]   S_mc_app_cmd;           
    wire        S_mc_app_en;            
    wire[127:0] S_mc_app_wdf_data;      
    wire        S_mc_app_wdf_end;       
    wire[15:0]  S_mc_app_wdf_mask;      
    wire        S_mc_app_wdf_wren;      
    wire[127:0] S_mc_app_rd_data;       
    wire        S_mc_app_rd_data_end;   
    wire        S_mc_app_rd_data_valid; 
    wire        S_mc_app_rdy;           
    wire        S_mc_app_wdf_rdy;       
    wire        S_mc_dbg_fifo_afull;
    wire        S_mc_dbg_app_rdy;
    wire        S_mc_dbg_wdf_rdy;
    wire        S_mc_dbg_cmd_pop;

    wire        S_hdmi_vsync;
    wire        S_hdmi_hsync;
    wire        S_hdmi_de;
    wire        S_hdmi_user;
    wire        S_hdmi_last;
    wire        S_video_out_vsync;
    wire        S_hdmi_window_rd_en;
    wire        S_hdmi_mix_vsync;
    wire        S_hdmi_mix_hsync;
    wire        S_hdmi_mix_de;
    wire[23:0]  S_hdmi_mix_data;
    wire        S_vision_vsync;
    wire        S_vision_hsync;
    wire        S_vision_de;
    wire[23:0]  S_vision_data;
    wire S_color_valid,S_motion_valid,S_motion_warming,S_motion_scene;
    wire [11:0] S_color_x,S_color_y,S_color_w,S_color_h,S_motion_x,S_motion_y,S_motion_w,S_motion_h;
    wire [2:0] S_motion_boxes;
    wire [35:0] S_motion_x0,S_motion_y0,S_motion_x1,S_motion_y1;
    wire [15:0] S_motion_cells;
    wire [7:0] S_motion_reply,S_motion_threshold,S_motion_minimum;
    wire S_motion_reset;
    reg [1:0] S_motion_pause;
    wire S_motion_vs,S_motion_hs,S_motion_de;
    wire [23:0] S_motion_rgb;
    wire[2:0]   S_video_mode_active;
    wire        S_track_vsync;
    wire        S_track_hsync;
    wire        S_track_de;
    wire[23:0]  S_track_data;
    wire [7:0] S_binary_threshold, S_edge_threshold, S_camera_fps, S_display_fps;
    wire [15:0] S_camera_frames, S_display_frames;
    wire [135:0] S_video_stats;
    wire [79:0] S_options;
    wire [7:0] S_isp_flags,S_options_reply;
    wire S_hist_vs,S_hist_hs,S_hist_de;
    wire [23:0] S_hist_rgb,S_vision_original,S_hist_original;
    wire S_effect_valid;
    wire [11:0] S_effect_x0,S_effect_y0,S_effect_x1,S_effect_y1;
    assign S_effect_valid=!S_options[11]||S_region_valid;
    assign S_effect_x0=S_options[11]?S_region_x0:S_options[27:16];
    assign S_effect_y0=S_options[11]?S_region_y0:S_options[43:32];
    assign S_effect_x1=S_options[11]?S_region_x1:S_options[59:48];
    assign S_effect_y1=S_options[11]?S_region_y1:S_options[75:64];
    wire        S_target_valid;
    wire[11:0]  S_target_center_x;
    wire[11:0]  S_target_center_y;
    wire[11:0]  S_target_width;
    wire[11:0]  S_target_height;
    wire        S_hdmi_out_vsync;
    wire        S_hdmi_out_hsync;
    wire        S_hdmi_out_de;
    wire[23:0]  S_hdmi_out_data;
    wire        S_osd_enable_ctrl;
    wire[2:0]   S_video_mode_ctrl;
    wire[1:0]   S_target_color_ctrl;
    wire[1:0]   S_picture_mode_ctrl;
    wire[2:0]   S_color_perm_ctrl;
    reg [15:0]  S_ae_meta, S_ae_sync;   // AE values resampled into the 100 MHz UART domain
    reg [15:0]  S_ag_meta, S_ag_sync;   // (diagnostic readback only; values change rarely)
    wire        S_range_limited_ctrl;   // 1 = HDMI output mapped to 16..235
    wire[7:0]   S_shade_k_ctrl, S_shade_s_ctrl;      // A37 shading correction (100 MHz)
    wire[1:0]   S_fps_min_ctrl;                       // A39 (100 MHz) -> ae_set syncs it to 24 MHz
    wire[1:0]   S_fps_mode_24m;                       // A39 from ae_set (24 MHz)
    wire[15:0]  S_vts_24m;
    reg [1:0]   S_fps_mode_meta100, S_fps_mode_sync100, S_fps_mode_metapx, S_fps_mode_syncpx;
    reg [7:0]   S_shade_k_meta, S_shade_k_sync, S_shade_s_meta, S_shade_s_sync;
    wire        S_shd_vsync, S_shd_hsync, S_shd_de;
    wire[23:0]  S_shd_data;
    wire[23:0]  S_track_clean_data;
    // A20 region effects
    wire        S_vision_edge, S_track_clean_edge;
    wire        S_region_valid;
    wire[11:0]  S_region_x0, S_region_y0, S_region_x1, S_region_y1;
    wire        S_fx_vsync, S_fx_hsync, S_fx_de;
    wire[23:0]  S_fx_data, S_fx_clean;
    wire[2:0]   S_fx_mode_ctrl;
    wire        S_region_union_ctrl;
    wire[1:0]   S_mosaic_sel_ctrl;
    reg [2:0]   S_fx_mode_meta, S_fx_mode_sync;
    reg         S_region_union_meta, S_region_union_sync;
    reg [1:0]   S_mosaic_sel_meta, S_mosaic_sel_sync;
    wire        S_region_lock_ctrl;
    reg         S_region_lock_meta, S_region_lock_sync;
    wire[7:0]   S_hue_center_ctrl, S_hue_halfw_ctrl, S_sat_min_ctrl, S_val_min_ctrl;   // HSV (100 MHz)
    reg [7:0]   S_hue_center_meta, S_hue_halfw_meta, S_sat_min_meta, S_val_min_meta;
    reg [7:0]   S_hue_center_sync, S_hue_halfw_sync, S_sat_min_sync, S_val_min_sync;   // pixel clock
    wire[15:0]  S_ag_max_ctrl;          // AE gain limit from UART (100 MHz) ...
    reg [15:0]  S_ag_max_meta, S_ag_max_sync;   // ... resampled into the 24 MHz sensor-control domain
    reg [2:0]   S_color_perm_meta;
    reg [2:0]   S_color_perm_sync;
    reg [23:0]  S_video_rd_data_fixed;
    reg [1:0]   S_osd_enable_sync;
    reg [2:0]   S_video_mode_meta;
    reg [2:0]   S_video_mode_sync;
    reg [1:0]   S_target_color_meta;
    reg [1:0]   S_target_color_sync;

    wire[23:0]  S_video_rd_data;
    wire        S_dbg_ddr_write_count_ok;
    wire        S_dbg_ddr_read_count_ok;
    reg [19:0]  S_dbg_raw10_word_count;
    reg [18:0]  S_dbg_raw720_word_count;
    reg [18:0]  S_dbg_isp_word_count;
    reg         S_dbg_raw10_count_ok;
    reg         S_dbg_raw720_count_ok;
    reg         S_dbg_isp_count_ok;
    reg [1:0]   S_dbg_raw10_count_ok_sync;
    reg [1:0]   S_dbg_raw720_count_ok_sync;
    reg [1:0]   S_dbg_isp_count_ok_sync;
    reg [1:0]   S_dbg_ddr_write_count_ok_sync;
    reg [1:0]   S_dbg_ddr_read_count_ok_sync;
    reg         S_dbg_raw_seen;
    reg         S_dbg_isp_seen;
    reg         S_dbg_ddr_wr_seen;
    reg         S_dbg_ddr_rd_seen;
    reg         S_dbg_cam_cfg_seen;
    reg         S_dbg_hs_seen;
    reg         S_dbg_csi_seen;
    reg         S_dbg_raw10_seen;
    reg         S_dbg_raw720_seen;
    reg         S_dbg_isp_fs_seen;
    reg         S_dbg_isp_valid_seen;
    reg         S_dbg_ddr_req_seen;
    reg         S_dbg_ddr_ready_seen;
    reg [1:0]   S_dbg_raw_seen_sync;
    reg [1:0]   S_dbg_isp_seen_sync;
    reg [1:0]   S_dbg_ddr_wr_seen_sync;
    reg [1:0]   S_dbg_ddr_rd_seen_sync;
    reg [1:0]   S_dbg_cam_cfg_seen_sync;
    reg [1:0]   S_dbg_hs_seen_sync;
    reg [1:0]   S_dbg_csi_seen_sync;
    reg [1:0]   S_dbg_raw10_seen_sync;
    reg [1:0]   S_dbg_raw720_seen_sync;
    reg [1:0]   S_dbg_isp_fs_seen_sync;
    reg [1:0]   S_dbg_isp_valid_seen_sync;
    reg [1:0]   S_dbg_ddr_req_seen_sync;
    reg [1:0]   S_dbg_ddr_ready_seen_sync;
    reg         S_dbg_video_nonzero_seen;
    reg         S_dbg_rd_req_seen;
    reg         S_dbg_rd_valid_seen;
    reg         S_dbg_rd_nonzero_seen;
    reg [1:0]   S_dbg_rd_req_seen_sync;
    reg [1:0]   S_dbg_rd_valid_seen_sync;
    reg [1:0]   S_dbg_rd_nonzero_seen_sync;
    reg         S_dbg_mc_init_seen;
    reg         S_dbg_mc_app_rdy_seen;
    reg         S_dbg_mc_wdf_rdy_seen;
    reg         S_dbg_mc_cmd_pop_seen;
    reg [1:0]   S_dbg_mc_init_seen_sync;
    reg [1:0]   S_dbg_mc_app_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_wdf_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_cmd_pop_seen_sync;
    wire[9:0]   S_hdmi_debug_status;
    wire        S_lane_error_any;


    assign O_screen_pwm = 1'b1;
    assign S_video_out_vsync = ~S_hdmi_vsync;
	
	
    assign S_rst_n 		= S_pll_lock;
    assign S_hdmi_rst_n = S_pll_lock & S_hdmi_pll_lock;

    assign O_laser_enable=1'b0;
    assign O_dac_clr_n=1'b1, O_dac_sync_n=1'b1, O_dac_ldac_n=1'b1;
    assign O_dac_sclk=1'b0, O_dac_din=1'b0;
    wire S_demo_valid,S_demo_blank,S_demo_boot,S_demo_safe,S_manual_motion;
    wire [7:0] S_demo_cmd,S_demo_data,S_demo_reply,S_demo_motor_status;
    wire [31:0] S_demo_address;
    wire [24:0] S_demo_request;
    wire S_demo_request_toggle,S_demo_ack,S_demo_home,S_demo_heartbeat;
    wire [63:0] S_demo_response;
    demo_control #(.IMAGE(0)) demo_control_i(
      .clk(S_100m_clk),.rst_n(S_rst_n),.valid(S_demo_valid),.cmd(S_demo_cmd),.data(S_demo_data),.reply(S_demo_reply),
      .motor_request(S_demo_request),.motor_toggle(S_demo_request_toggle),
      .motor_response_async(S_demo_response),.motor_ack_async(S_demo_ack),.motor_status_async(S_demo_motor_status),
      .home_toggle(S_demo_home),.heartbeat_toggle(S_demo_heartbeat),.switch_safe_async(S_demo_safe),
      .blank(S_demo_blank),.boot_en(S_demo_boot),.boot_address(S_demo_address));
    demo_mboot demo_mboot_i(.enable(S_demo_boot),.address(S_demo_address));
    demo_manual demo_manual_i(.clk(I_sys_clk),.rst_n(S_rst_n),
      .pan_rx(I_pan_rx),.pan_tx(O_pan_tx),.tilt_rx(I_tilt_rx),.tilt_tx(O_tilt_tx),
      .request_async(S_demo_request),.request_toggle_async(S_demo_request_toggle),
      .home_toggle_async(S_demo_home),.heartbeat_toggle_async(S_demo_heartbeat),
      .response(S_demo_response),.ack(S_demo_ack),.status(S_demo_motor_status),.switch_safe(S_demo_safe),.motion_active(S_manual_motion));

    contest_uart_control #(
        .CLK_FREQ_HZ ( 100000000 ),
        .UART_BPS    ( 115200    )
    ) u_contest_uart_control (
        .demo_valid(S_demo_valid),.demo_cmd(S_demo_cmd),.demo_data(S_demo_data),.demo_reply(S_demo_reply),
        .clk         ( S_100m_clk       ),
        .rst_n       ( S_rst_n          ),
        .uart_rxd    ( I_uart_rx        ),
        .uart_txd    ( O_uart_tx        ),
        .button_n    ( 1'b1             ),
        .options_reply(S_demo_cmd>=8'h76 ? S_motion_reply : S_options_reply),
        .video_stats(S_video_stats),
        .target_valid( S_video_stats[0] ),
        .target_center_x ( S_video_stats[19:8] ),
        .target_center_y ( S_video_stats[35:24] ),
        .target_width    ( S_video_stats[51:40]    ),
        .target_height   ( S_video_stats[67:56]   ),
        .ae_value    ( S_ae_sync        ),
        .ag_value    ( S_ag_sync        ),
        .osd_enable  ( S_osd_enable_ctrl),
        .video_mode  ( S_video_mode_ctrl),
        .target_color( S_target_color_ctrl),
        .picture_mode( S_picture_mode_ctrl),
        .ag_max      ( S_ag_max_ctrl      ),
        .range_limited( S_range_limited_ctrl ),
        .shade_k      ( S_shade_k_ctrl ),
        .shade_s      ( S_shade_s_ctrl ),
        .fps_min      ( S_fps_min_ctrl ),
        .fps_mode     ( S_fps_mode_sync100 ),
        .hue_center8   ( S_hue_center_ctrl ),
        .hue_halfwidth8( S_hue_halfw_ctrl  ),
        .sat_min       ( S_sat_min_ctrl    ),
        .val_min       ( S_val_min_ctrl    ),
        .fx_mode       ( S_fx_mode_ctrl    ),
        .region_union  ( S_region_union_ctrl ),
        .mosaic_sel    ( S_mosaic_sel_ctrl ),
        .pip_enable    (),
        .pip_zoom      (),
        .region_lock   ( S_region_lock_ctrl ),
        .color_perm  ( S_color_perm_ctrl  )
    );

    // HSV thresholds are quasi-static; two-stage resampling into the pixel clock.
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if (!S_hdmi_rst_n) begin
            S_hue_center_meta <= 8'd0;  S_hue_center_sync <= 8'd0;
            S_hue_halfw_meta  <= 8'd18; S_hue_halfw_sync  <= 8'd18;
            S_sat_min_meta    <= 8'd48; S_sat_min_sync    <= 8'd48;
            S_val_min_meta    <= 8'd40; S_val_min_sync    <= 8'd40;
            S_fx_mode_meta <= 3'd0; S_fx_mode_sync <= 3'd0;
            S_region_union_meta <= 1'b1; S_region_union_sync <= 1'b1;
            S_mosaic_sel_meta <= 2'd1; S_mosaic_sel_sync <= 2'd1;
            S_shade_k_meta <= 8'd4; S_shade_k_sync <= 8'd4; S_shade_s_meta <= 8'd20; S_shade_s_sync <= 8'd20;
            S_region_lock_meta <= 1'b0; S_region_lock_sync <= 1'b0;
        end else begin
            S_fx_mode_meta <= S_fx_mode_ctrl; S_fx_mode_sync <= S_fx_mode_meta;
            S_region_union_meta <= S_region_union_ctrl; S_region_union_sync <= S_region_union_meta;
            S_mosaic_sel_meta <= S_mosaic_sel_ctrl; S_mosaic_sel_sync <= S_mosaic_sel_meta;
            S_shade_k_meta <= S_shade_k_ctrl; S_shade_k_sync <= S_shade_k_meta;
            S_shade_s_meta <= S_shade_s_ctrl; S_shade_s_sync <= S_shade_s_meta;
            S_region_lock_meta <= S_region_lock_ctrl; S_region_lock_sync <= S_region_lock_meta;
            S_hue_center_meta <= S_hue_center_ctrl; S_hue_center_sync <= S_hue_center_meta;
            S_hue_halfw_meta  <= S_hue_halfw_ctrl;  S_hue_halfw_sync  <= S_hue_halfw_meta;
            S_sat_min_meta    <= S_sat_min_ctrl;    S_sat_min_sync    <= S_sat_min_meta;
            S_val_min_meta    <= S_val_min_ctrl;    S_val_min_sync    <= S_val_min_meta;
        end
    end

    always @(posedge S_100m_clk) begin S_fps_mode_meta100 <= S_fps_mode_24m; S_fps_mode_sync100 <= S_fps_mode_meta100; end
    always @(posedge S_hdmi_pixel_clk) begin S_fps_mode_metapx <= S_fps_mode_24m; S_fps_mode_syncpx <= S_fps_mode_metapx; end

    always @(posedge S_24m_clk or negedge S_rst_n) begin
        if (!S_rst_n) begin
            S_ag_max_meta <= 16'd127;
            S_ag_max_sync <= 16'd127;
        end else begin
            S_ag_max_meta <= S_ag_max_ctrl;
            S_ag_max_sync <= S_ag_max_meta;
        end
    end

    always @(posedge S_100m_clk or negedge S_rst_n) begin
        if (!S_rst_n) begin
            S_ae_meta <= 16'd0; S_ae_sync <= 16'd0;
            S_ag_meta <= 16'd0; S_ag_sync <= 16'd0;
        end else begin
            S_ae_meta <= S_ae; S_ae_sync <= S_ae_meta;
            S_ag_meta <= S_ag; S_ag_sync <= S_ag_meta;
        end
    end

    // DDR readout delivers each pixel's bytes rotated (128-bit words hold
    // 5 1/3 pixels, so a one-word offset in the read FIFO rotates channels).
    // Reorder the bytes here so everything downstream (video processor,
    // colour tracker, OSD, AE meter, HDMI) sees true R,G,B.  The order is
    // selectable over UART for diagnosis; the reset default is the measured
    // correction.  hdmi_mixer registers this value twice, so the extra
    // combinational mux is not on a critical path.
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if (!S_hdmi_rst_n) begin
            S_color_perm_meta <= 3'd0;
            S_color_perm_sync <= 3'd0;
        end else begin
            S_color_perm_meta <= S_color_perm_ctrl;
            S_color_perm_sync <= S_color_perm_meta;
        end
    end
    always @* begin
        case (S_color_perm_sync)
            3'd1:    S_video_rd_data_fixed = {S_video_rd_data[23:16], S_video_rd_data[7:0],   S_video_rd_data[15:8]};
            3'd2:    S_video_rd_data_fixed = {S_video_rd_data[15:8],  S_video_rd_data[23:16], S_video_rd_data[7:0]};
            3'd3:    S_video_rd_data_fixed = {S_video_rd_data[15:8],  S_video_rd_data[7:0],   S_video_rd_data[23:16]};
            3'd4:    S_video_rd_data_fixed = {S_video_rd_data[7:0],   S_video_rd_data[23:16], S_video_rd_data[15:8]};
            3'd5:    S_video_rd_data_fixed = {S_video_rd_data[7:0],   S_video_rd_data[15:8],  S_video_rd_data[23:16]};
            default: S_video_rd_data_fixed = S_video_rd_data;
        endcase
    end

    // Control values remain stable between commands. Two-stage sampling
    // transfers them safely into the HDMI pixel clock domain.
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if (!S_hdmi_rst_n) begin
            S_osd_enable_sync <= 2'b11;
            S_video_mode_meta <= 3'd0;
            S_video_mode_sync <= 3'd0;
            S_target_color_meta <= 2'd0;
            S_target_color_sync <= 2'd0;
        end else begin
            S_osd_enable_sync <= {S_osd_enable_sync[0], S_osd_enable_ctrl};
            S_video_mode_meta <= S_video_mode_ctrl;
            S_video_mode_sync <= S_video_mode_meta;
            S_target_color_meta <= S_target_color_ctrl;
            S_target_color_sync <= S_target_color_meta;
        end
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n) begin
            S_dbg_raw10_word_count <= 20'd0;
            S_dbg_raw10_count_ok   <= 1'b0;
        end else if(S_raw10_frame_start) begin
            S_dbg_raw10_count_ok   <= (S_dbg_raw10_word_count == 20'd518400);
            S_dbg_raw10_word_count <= S_raw10_valid ? 20'd1 : 20'd0;
        end else if(S_raw10_valid) begin
            S_dbg_raw10_word_count <= S_dbg_raw10_word_count + 20'd1;
        end
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n) begin
            S_dbg_raw720_word_count <= 19'd0;
            S_dbg_raw720_count_ok   <= 1'b0;
        end else if(S_raw720_frame_start) begin
            S_dbg_raw720_count_ok   <= (S_dbg_raw720_word_count == 19'd518400);
            S_dbg_raw720_word_count <= S_raw720_valid ? 19'd1 : 19'd0;
        end else if(S_raw720_valid) begin
            S_dbg_raw720_word_count <= S_dbg_raw720_word_count + 19'd1;
        end
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n) begin
            S_dbg_isp_word_count <= 19'd0;
            S_dbg_isp_count_ok   <= 1'b0;
        end else if(S_ISP_O_tuser) begin
            S_dbg_isp_count_ok   <= (S_dbg_isp_word_count == 19'd230400);  // A5: 4 pixels per word
            S_dbg_isp_word_count <= S_ISP_O_tvalid ? 19'd1 : 19'd0;
        end else if(S_ISP_O_tvalid) begin
            S_dbg_isp_word_count <= S_dbg_isp_word_count + 19'd1;
        end
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw_seen <= 1'b0;
        else if(S_raw10_frame_start | S_axis_tvalid)
            S_dbg_raw_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_seen <= 1'b0;
        else if(S_ISP_O_tuser | S_ISP_O_tvalid)
            S_dbg_isp_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_wr_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_wr_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_rd_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_ddr_rd_seen <= 1'b1;
    end

    always @(posedge S_24m_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_cam_cfg_seen <= 1'b0;
        else if(S_cam_cfg_done)
            S_dbg_cam_cfg_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_hs_seen <= 1'b0;
        else if(S_hs_rx_valid)
            S_dbg_hs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_csi_seen <= 1'b0;
        else if(S_csi_frame_start | S_csi_valid)
            S_dbg_csi_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw10_seen <= 1'b0;
        else if(S_raw10_frame_start | S_raw10_valid)
            S_dbg_raw10_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw720_seen <= 1'b0;
        else if(S_raw720_frame_start | S_raw720_valid)
            S_dbg_raw720_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_fs_seen <= 1'b0;
        else if(S_ISP_O_tuser)
            S_dbg_isp_fs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_valid_seen <= 1'b0;
        else if(S_ISP_O_tvalid)
            S_dbg_isp_valid_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_req_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_ready_seen <= 1'b0;
        else if(S_ddr_user_ready)
            S_dbg_ddr_ready_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_vo_ddr_rd_en)
            S_dbg_rd_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_rd_valid_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
            S_dbg_rd_nonzero_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_init_seen <= 1'b0;
        else if(S_init_calib_complete)
            S_dbg_mc_init_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_app_rdy_seen <= 1'b0;
        else if(S_mc_dbg_app_rdy)
            S_dbg_mc_app_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_wdf_rdy_seen <= 1'b0;
        else if(S_mc_dbg_wdf_rdy)
            S_dbg_mc_wdf_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_cmd_pop_seen <= 1'b0;
        else if(S_mc_dbg_cmd_pop)
            S_dbg_mc_cmd_pop_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_dbg_cam_cfg_seen_sync <= 2'b00;
            S_dbg_hs_seen_sync      <= 2'b00;
            S_dbg_csi_seen_sync     <= 2'b00;
            S_dbg_raw10_seen_sync   <= 2'b00;
            S_dbg_raw720_seen_sync  <= 2'b00;
            S_dbg_raw_seen_sync    <= 2'b00;
            S_dbg_isp_seen_sync    <= 2'b00;
            S_dbg_ddr_wr_seen_sync <= 2'b00;
            S_dbg_ddr_rd_seen_sync <= 2'b00;
            S_dbg_isp_fs_seen_sync  <= 2'b00;
            S_dbg_isp_valid_seen_sync <= 2'b00;
            S_dbg_ddr_req_seen_sync <= 2'b00;
            S_dbg_ddr_ready_seen_sync <= 2'b00;
            S_dbg_video_nonzero_seen <= 1'b0;
            S_dbg_rd_req_seen_sync <= 2'b00;
            S_dbg_rd_valid_seen_sync <= 2'b00;
            S_dbg_rd_nonzero_seen_sync <= 2'b00;
            S_dbg_mc_init_seen_sync <= 2'b00;
            S_dbg_mc_app_rdy_seen_sync <= 2'b00;
            S_dbg_mc_wdf_rdy_seen_sync <= 2'b00;
            S_dbg_mc_cmd_pop_seen_sync <= 2'b00;
            S_dbg_raw10_count_ok_sync <= 2'b00;
            S_dbg_raw720_count_ok_sync <= 2'b00;
            S_dbg_isp_count_ok_sync <= 2'b00;
            S_dbg_ddr_write_count_ok_sync <= 2'b00;
            S_dbg_ddr_read_count_ok_sync <= 2'b00;
        end
        else begin
            S_dbg_cam_cfg_seen_sync <= {S_dbg_cam_cfg_seen_sync[0], S_dbg_cam_cfg_seen};
            S_dbg_hs_seen_sync      <= {S_dbg_hs_seen_sync[0],      S_dbg_hs_seen};
            S_dbg_csi_seen_sync     <= {S_dbg_csi_seen_sync[0],     S_dbg_csi_seen};
            S_dbg_raw10_seen_sync   <= {S_dbg_raw10_seen_sync[0],   S_dbg_raw10_seen};
            S_dbg_raw720_seen_sync  <= {S_dbg_raw720_seen_sync[0],  S_dbg_raw720_seen};
            S_dbg_raw_seen_sync    <= {S_dbg_raw_seen_sync[0],    S_dbg_raw_seen};
            S_dbg_isp_seen_sync    <= {S_dbg_isp_seen_sync[0],    S_dbg_isp_seen};
            S_dbg_ddr_wr_seen_sync <= {S_dbg_ddr_wr_seen_sync[0], S_dbg_ddr_wr_seen};
            S_dbg_ddr_rd_seen_sync <= {S_dbg_ddr_rd_seen_sync[0], S_dbg_ddr_rd_seen};
            S_dbg_isp_fs_seen_sync  <= {S_dbg_isp_fs_seen_sync[0],  S_dbg_isp_fs_seen};
            S_dbg_isp_valid_seen_sync <= {S_dbg_isp_valid_seen_sync[0], S_dbg_isp_valid_seen};
            S_dbg_ddr_req_seen_sync <= {S_dbg_ddr_req_seen_sync[0], S_dbg_ddr_req_seen};
            S_dbg_ddr_ready_seen_sync <= {S_dbg_ddr_ready_seen_sync[0], S_dbg_ddr_ready_seen};
            S_dbg_rd_req_seen_sync <= {S_dbg_rd_req_seen_sync[0], S_dbg_rd_req_seen};
            S_dbg_rd_valid_seen_sync <= {S_dbg_rd_valid_seen_sync[0], S_dbg_rd_valid_seen};
            S_dbg_rd_nonzero_seen_sync <= {S_dbg_rd_nonzero_seen_sync[0], S_dbg_rd_nonzero_seen};
            S_dbg_mc_init_seen_sync <= {S_dbg_mc_init_seen_sync[0], S_dbg_mc_init_seen};
            S_dbg_mc_app_rdy_seen_sync <= {S_dbg_mc_app_rdy_seen_sync[0], S_dbg_mc_app_rdy_seen};
            S_dbg_mc_wdf_rdy_seen_sync <= {S_dbg_mc_wdf_rdy_seen_sync[0], S_dbg_mc_wdf_rdy_seen};
            S_dbg_mc_cmd_pop_seen_sync <= {S_dbg_mc_cmd_pop_seen_sync[0], S_dbg_mc_cmd_pop_seen};
            S_dbg_raw10_count_ok_sync <= {S_dbg_raw10_count_ok_sync[0], S_dbg_raw10_count_ok};
            S_dbg_raw720_count_ok_sync <= {S_dbg_raw720_count_ok_sync[0], S_dbg_raw720_count_ok};
            S_dbg_isp_count_ok_sync <= {S_dbg_isp_count_ok_sync[0], S_dbg_isp_count_ok};
            S_dbg_ddr_write_count_ok_sync <= {S_dbg_ddr_write_count_ok_sync[0], S_dbg_ddr_write_count_ok};
            S_dbg_ddr_read_count_ok_sync <= {S_dbg_ddr_read_count_ok_sync[0], S_dbg_ddr_read_count_ok};
            if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
                S_dbg_video_nonzero_seen <= 1'b1;
        end
    end

    assign S_lane_error_any = |S_lane_error;

    assign S_vo_dbg_fifo_rst = 1'b0;
    assign S_vo_dbg_ddr_rd_valid = 1'b0;
    assign S_vo_dbg_fifo_rd_en = 1'b0;
    assign S_vo_dbg_fifo_empty = 1'b0;
    assign S_mc_dbg_fifo_afull = 1'b0;
    assign S_mc_dbg_app_rdy = 1'b0;
    assign S_mc_dbg_wdf_rdy = 1'b0;
    assign S_mc_dbg_cmd_pop = 1'b0;

    // Left-to-right diagnostic stages in the HDMI status strip:
    // camera config, MIPI HS, exact RAW10 frame, exact scaled RAW frame,
    // exact ISP/packer frame, DDR calibration, exact DDR write/read frames,
    // DDR read data activity, nonzero displayed pixels.
    assign S_hdmi_debug_status = {
        S_dbg_video_nonzero_seen,
        S_dbg_ddr_rd_seen_sync[1],
        S_dbg_ddr_read_count_ok_sync[1],
        S_dbg_ddr_write_count_ok_sync[1],
        S_dbg_mc_init_seen_sync[1],
        S_dbg_isp_count_ok_sync[1],
        S_dbg_raw720_count_ok_sync[1],
        S_dbg_raw10_count_ok_sync[1],
        S_dbg_hs_seen_sync[1],
        S_dbg_cam_cfg_seen_sync[1]
    };
	
	assign O_cam_rst 	= 1'b1;
    assign O_cam_24m 	= S_24m_clk;
	
	
	
	
    PLL u_PLL(
        .refclk      ( I_sys_clk         ),
        .reset    	 ( ~I_rst_n          ),
        
        .clk0_out    ( S_100m_clk        ),
        .clk1_out    ( S_24m_clk         ),

        .clk4_out    (                    ),
        .clk5_out    (                    ),

        .lock        ( S_pll_lock        )

    );

    // A dedicated 75/375 MHz PLL drives native 1280x720 HDMI. Keeping
    // the original PLL on DDR/camera clocks avoids disturbing DDR2 timing.
    pll_hdmi u_PLL_hdmi_720p(
        .refclk      ( I_sys_clk          ),
        .clk0_out    ( S_hdmi_pixel_clk   ),
        .clk1_out    ( S_hdmi_serial_clk  ),
        .clk2_out    (                    ),
        .clk3_out    (                    ),
        .lock        ( S_hdmi_pll_lock    )
    );


	

  
  ae_set u_ae_set (
      .I_clk(S_24m_clk),
      .I_rst(~S_rst_n),
      .I_btn({I_button,2'b11}),
      .I_ag_max(S_ag_max_sync),
      .I_fps_min(S_fps_min_ctrl),
      .O_fps_mode(S_fps_mode_24m),
      .O_vts(S_vts_24m),
      .I_meter_clk(S_hdmi_pixel_clk),
      .I_meter_vsync(S_hdmi_mix_vsync),
      .I_meter_de(S_hdmi_mix_de),
      .I_meter_rgb(S_hdmi_mix_data),
      .I_cam_cfg_done(S_cam_cfg_done),
      .I_ae_cfg_done(S_ae_cfg_done),
      .O_ae_req(S_ae_req),
      .O_ae(S_ae),
      .O_ag(S_ag)
  );
  
    uicfgcs500 #(
      .CLK_DIV(24000000 / 100000 - 1)
  ) u_uicfgcs500 (
      .I_clk(S_24m_clk),  //系统时钟输入
      .I_rst_n(S_rst_n),  //系统复位输入
      .I_ae_req(S_ae_req),
      .I_ae(S_ae),
      .I_ag(S_ag),
      .I_vts(S_vts_24m),
      .O_cam_scl(O_cam_scl),  //I2C总线，SCL时钟
      .IO_cam_sda(IO_cam_sda),  //I2C总线，SDA数据
      .O_cfg_done(S_cam_cfg_done),  //摄像头寄存器初始化完成
      .O_ae_cfg_done(S_ae_cfg_done)  //AE配置完成
  );
	
    mipi_dphy_rx_ph1p_mipiio_wrapper#(
        .DPHY_RX_LOCATION      ( "DPHY0" ),
        .HS_EQUALIZER          ( "0dB"   ),//"0dB","1.5dB","3dB","4.5dB","6dB","7.5dB","9dB","10.5dB"
        .HS_VGA_GAIN           ( "0dB" ),//"-3dB","-1.5dB","0dB","1.5dB","3dB","4.5dB","6dB","7.5dB"
        .LANE_NUM              ( 2       ),
        .BYTE_NUM              ( 1       )
    )u_mipi_dphy_rx_ph1p_mipiio_wrapper(
        .I_lp_clk              ( S_100m_clk       ),
        .I_rst                 ( ~S_rst_n         ),

        .I_clk_lane_in_delay   ( 6'd0             ),
        .I_data_lane0_in_delay ( 6'd0             ),
        .I_data_lane1_in_delay ( 6'd0             ),
        .I_data_lane2_in_delay ( 6'd0             ),
        .I_data_lane3_in_delay ( 6'd0             ),

        .I_lane_invert         ( 4'b0000          ),
     
        .O_hs_rx_clk           ( S_csi_rx_clk     ),
        .O_hs_rx_valid         ( S_hs_rx_valid    ),
        .O_hs_rx_data          ( S_hs_rx_data     ),
      
        .O_lp_rx_lane0_p       (  ),
        .O_lp_rx_lane0_n       (  ),
      
        .I_lp_tx_en            ( 1'b0             ),
        .I_lp_tx_lane0_p       ( 1'b1             ),
        .I_lp_tx_lane0_n       ( 1'b1             ),
      
        .O_lane_match_error    (                  ),
        .O_lane_error          ( S_lane_error     ),
      
        .IO_rx_clk_pad_n       ( IO_rx_clk_pad_n  ),
        .IO_rx_clk_pad_p       ( IO_rx_clk_pad_p  ),
        .IO_rx_data_pad_n      ( IO_rx_data_pad_n ),
        .IO_rx_data_pad_p      ( IO_rx_data_pad_p )
    );
	
	
 //  cwc cwc_inst
 //(
 //    .probe0(S_hs_rx_valid),
 //    .probe1(S_hs_rx_data),
 //    .probe2(S_lane_error),
 //    .clk(S_csi_rx_clk)
 //);

 //csi 解码为RAW数据
csi_unpacket_2lane u_csi_unpacket(
.I_clk                 ( S_csi_rx_clk       ),
.I_rst_n               ( S_rst_n        	),
.I_hs_valid            ( S_hs_rx_valid     ),
.I_hs_data             ( S_hs_rx_data      ),

.O_csi_frame_start     ( S_csi_frame_start ),
.O_csi_frame_end       ( S_csi_frame_end   ),
.O_csi_valid           ( S_csi_valid       ),
.O_csi_data            ( S_csi_data        )
);


//解码为RAW8
raw10_unpacket_2lane u_raw10_unpacket (
.I_clk  (S_csi_rx_clk),
.I_rst_n(S_rst_n),

.I_csi_frame_start(S_csi_frame_start),
.I_csi_frame_end  (S_csi_frame_end),
.I_csi_valid      (S_csi_valid),
.I_csi_data       (S_csi_data),

.O_raw10_frame_start(S_raw10_frame_start),
.O_raw10_frame_end  (S_raw10_frame_end),
.O_raw10_valid      (S_raw10_valid),
.O_raw10_data       (S_raw10_data)
  );

// Preserve the complete 1920x1080 Bayer mosaic.  Non-integer scaling is done
// only after demosaic, in the RGB domain inside isp_top.
assign S_raw720_frame_start = S_raw10_frame_start;
assign S_raw720_frame_end   = S_raw10_frame_end;
assign S_raw720_valid       = S_raw10_valid;
assign S_raw720_data        = S_raw10_data;


//将数据转为stream流
uial2axis #(
.IMG_WIDTH(1920),
.IMG_HEIGHT(1080),
.INPUT_DATA_WIDTH(40)
) 
u_uial2axis (
.I_native_clk(S_csi_rx_clk),
.I_rst_n     (S_rst_n),
.I_data      (S_raw720_data       ),
.I_data_valid(S_raw720_valid      ),
.I_data_start(S_raw720_frame_start),
.I_data_end  (S_raw720_frame_end  ),
.axis_tvalid (S_axis_tvalid),
.axis_tdata  (S_axis_tdata ),
.axis_tuser  (S_axis_tuser ),
.axis_tlast  (S_axis_tlast )
);
	

image_correction #(
    .DATA_WIDTH(40)
)(
    .I_clk   		(S_csi_rx_clk),
    .I_rst_n 		(S_rst_n),
					
    .I_raw_data 	(S_axis_tdata),
    .I_raw_frame_end 	(S_axis_tlast),
    .I_raw_valid	(S_axis_tvalid),
    .I_raw_frame_start 	(S_axis_tuser),
					
    .O_raw_tdata 	(S_raw_tdata ),
    .O_raw_tlast 	(S_raw_tlast ),
    .O_raw_tvalid	(S_raw_tvalid),
    .O_raw_tuser 	(S_raw_tuser )
);


//cwc1 cwc1_Inst
//  (
//      .probe0(S_axis_tuser),
//      .probe1(S_axis_tvalid),
//      .probe2(S_axis_tlast),
//      .probe3(S_axis_tdata),
//      .probe4(S_ISP_O_tuser),
//      .probe5(S_ISP_O_tvalid),
//      .probe6(S_ISP_O_tlast),
//      .probe7(S_ISP_O_tdata),
//      .probe8 (S_raw_tdata ),
//      .probe9 (S_raw_tlast ),
//      .probe10(S_raw_tvalid),
//      .probe11(S_raw_tuser ),
//      .clk(S_csi_rx_clk)
//  );


//ISP算法顶层模块
// Demosaic at 1920x1080, then scale RGB to the native 1280x720 display frame.
isp_top u_isp_top (
    .I_algorithm_flags(S_isp_flags),
    .axi4s_video_aclk (S_csi_rx_clk),
    .I_rst_n          (S_rst_n),
    .I_picture_mode   (S_picture_mode_ctrl),
    .I_tlast          (S_axis_tlast),
    .I_tuser          (S_axis_tuser),
    .I_tdata          (S_axis_tdata),
    .I_tvalid         (S_axis_tvalid),
    .I_tdest          (10'd0),
    .O_tready         (1'b1),
    .O_tdata          (S_ISP_O_tdata),
    .O_tlast          (S_ISP_O_tlast),
    .O_tuser          (S_ISP_O_tuser),
    .O_tvalid         (S_ISP_O_tvalid),
    .I_tready         ()
);

assign S_ISP_O_tready = 1'b1;
wire [127:0] S_latency_tagged_data;
wire [23:0] S_displayed_frame_stamp;
wire S_displayed_frame_stamp_valid, S_latency_valid;
wire [11:0] S_latency_ms;
video_latency u_video_latency (
 .ref_clk(S_100m_clk),.camera_clk(S_csi_rx_clk),.pixel_clk(S_hdmi_pixel_clk),.rst_n(S_hdmi_rst_n),
 .camera_sof(S_axis_tuser),.packed_sof(S_ISP_O_tuser),.packed_rgb(S_ISP_O_tdata),
 .tagged_rgb(S_latency_tagged_data),.displayed_stamp(S_displayed_frame_stamp),
 .displayed_stamp_valid(S_displayed_frame_stamp_valid),.display_vs(S_motion_vs),.display_de(S_motion_de),
 .latency_ms(S_latency_ms),.latency_valid(S_latency_valid)
);


// video_in resets its asynchronous FIFO from the frame-start pulse.  Keep
// payload data away from that reset window so the first DDR word is retained
// and the RGB byte phase remains identical on every frame.
always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
    if(!S_rst_n) begin
        S_isp_data_delay  <= 2048'd0;
        S_isp_valid_delay <= 16'd0;
    end else begin
        S_isp_data_delay  <= {S_isp_data_delay[1919:0], S_latency_tagged_data};
        S_isp_valid_delay <= {S_isp_valid_delay[14:0], S_ISP_O_tvalid};
    end
end

assign S_isp_safe_data  = S_isp_data_delay[2047:1920];
assign S_isp_safe_valid = S_isp_valid_delay[15];


    video_in u_video_in(
        .I_rst_n              (  S_rst_n             ),

        .I_camera_clk         ( S_csi_rx_clk          ),
        .I_camera_frame_start ( S_ISP_O_tuser  ),
        .I_camera_valid       ( S_isp_safe_valid ),
        .I_camera_data        ( S_isp_safe_data  ),
        .I_mipi_rx_error      ( 1'b0			      ),

        .I_ddr_clk            ( S_ddr_clk             ),
		.I_display_pause      ( 1'b0                  ),
        .I_video_out_rd_busy  ( S_video_out_rd_busy   ),
        .O_video_in_wr_busy   ( S_video_in_wr_busy    ),
        .O_video_out_rp       ( S_video_out_rp        ),

        .O_ddr_user_wr_en     ( S_vi_ddr_wr_en        ),
        .O_ddr_user_addr      ( S_vi_ddr_wr_addr      ),
        .O_ddr_user_wr_data   ( S_vi_ddr_wr_data      ),
        .I_ddr_user_ready     ( S_ddr_user_ready      ),
        .O_debug_frame_words_ok ( S_dbg_ddr_write_count_ok )
    );



    assign S_ddr_user_wr_en    = S_vi_ddr_wr_en;

    assign S_ddr_user_rd_en    = S_vo_ddr_rd_en;

    assign S_ddr_user_addr     = S_vi_ddr_wr_en ? S_vi_ddr_wr_addr :
                                 S_vo_ddr_rd_en ? S_vo_ddr_rd_addr : 'd0;

    assign S_ddr_user_wr_data  = S_vi_ddr_wr_data;


    assign S_video_out_rst_n = S_hdmi_rst_n;

    video_out u_video_out(
        .O_frame_stamp(S_displayed_frame_stamp), .O_frame_stamp_valid(S_displayed_frame_stamp_valid),
        .I_rst_n             ( S_video_out_rst_n    ),
        .I_ddr_clk           ( S_ddr_clk            ),
 
        .O_video_out_rd_busy ( S_video_out_rd_busy  ),
        .I_video_in_wr_busy  ( S_video_in_wr_busy   ),
        .I_video_out_rp      ( S_video_out_rp       ),
 
        .O_ddr_user_rd_en    ( S_vo_ddr_rd_en       ),
        .O_ddr_user_addr     ( S_vo_ddr_rd_addr     ),
        .I_ddr_user_ready    ( S_ddr_user_ready     ),
        .I_ddr_user_rd_valid ( S_ddr_user_rd_valid  ),
        .I_ddr_user_rd_data  ( S_ddr_user_rd_data   ),

        .I_dsi_clk           ( S_hdmi_pixel_clk     ),
        .I_video_vsync       ( S_video_out_vsync    ),
        .I_video_rd_en       ( S_hdmi_window_rd_en  ),
        .O_vdieo_data        ( S_video_rd_data      ),
        .O_debug_frame_words_ok ( S_dbg_ddr_read_count_ok )
    );



    mc_to_user_interface u_mc_to_user_interface(
        .I_clk                   ( S_ddr_clk               ),
        .I_rst_n                 ( S_rst_n                 ),

        .I_ddr_user_wr_en        ( S_ddr_user_wr_en        ),
        .I_ddr_user_rd_en        ( S_ddr_user_rd_en        ),
        .I_ddr_user_addr         ( S_ddr_user_addr         ),
        .I_ddr_user_wr_data      ( S_ddr_user_wr_data      ),
        .O_ddr_user_ready        ( S_ddr_user_ready        ),
        .O_ddr_user_rd_valid     ( S_ddr_user_rd_valid     ),
        .O_ddr_user_rd_data      ( S_ddr_user_rd_data      ),
            
        .O_mc_app_en             ( S_mc_app_en             ),
        .O_mc_app_addr           ( S_mc_app_addr           ),
        .O_mc_app_cmd            ( S_mc_app_cmd            ),
        .I_mc_app_rdy            ( S_mc_app_rdy            ),
        .O_mc_app_wdf_wren       ( S_mc_app_wdf_wren       ),
        .O_mc_app_wdf_data       ( S_mc_app_wdf_data       ),
        .O_mc_app_wdf_end        ( S_mc_app_wdf_end        ),
        .O_mc_app_wdf_mask       ( S_mc_app_wdf_mask       ),
        .I_mc_app_wdf_rdy        ( S_mc_app_wdf_rdy        ),
        .I_mc_app_rd_data        ( S_mc_app_rd_data        ),
        .I_mc_app_rd_data_end    ( S_mc_app_rd_data_end    ),
        .I_mc_app_rd_data_valid  ( S_mc_app_rd_data_valid  )
    );



    ph1p35_324_ddr_wrapper u_ph1p35_324_ddr_wrapper(
        .I_sys_clk              ( I_sys_clk              ),
        .I_sys_rst_n            ( S_rst_n                ),

        .O_ddr_clk              ( S_ddr_clk              ),
        .O_init_calib_complete  ( S_init_calib_complete  ),
        .I_mc_app_addr          ( S_mc_app_addr          ),
        .I_mc_app_cmd           ( S_mc_app_cmd           ),
        .I_mc_app_en            ( S_mc_app_en            ),
        .I_mc_app_wdf_data      ( S_mc_app_wdf_data      ),
        .I_mc_app_wdf_end       ( S_mc_app_wdf_end       ),
        .I_mc_app_wdf_mask      ( S_mc_app_wdf_mask      ),
        .I_mc_app_wdf_wren      ( S_mc_app_wdf_wren      ),
        .O_mc_app_rd_data       ( S_mc_app_rd_data       ),
        .O_mc_app_rd_data_end   ( S_mc_app_rd_data_end   ),
        .O_mc_app_rd_data_valid ( S_mc_app_rd_data_valid ),
        .O_mc_app_rdy           ( S_mc_app_rdy           ),
        .O_mc_app_wdf_rdy       ( S_mc_app_wdf_rdy       ),

        .ddr_addr               ( ddr_addr               ),
        .ddr_ba                 ( ddr_ba                 ),
        .ddr_cke                ( ddr_cke                ),
        .ddr_odt                ( ddr_odt                ),
        .ddr_cs_n               ( ddr_cs_n               ),
        .ddr_ras_n              ( ddr_ras_n              ),
        .ddr_cas_n              ( ddr_cas_n              ),
        .ddr_we_n               ( ddr_we_n               ),
        .ddr_ck_p               ( ddr_ck_p               ),
        .ddr_ck_n               ( ddr_ck_n               ),
        .ddr_dm                 ( ddr_dm                 ),
        .ddr_dq                 ( ddr_dq                 ),
        .ddr_dqs_p              ( ddr_dqs_p              ),
        .ddr_dqs_n              ( ddr_dqs_n              )
    );



    uivtc #(
        .H_ActiveSize ( 1280 ),
        .H_FrameSize  ( 1650 ),
        .H_SyncStart  ( 1390 ),
        .H_SyncEnd    ( 1430 ),
        .V_ActiveSize ( 720  ),
        .V_FrameSize  ( 750  ),
        .V_SyncStart  ( 725  ),
        .V_SyncEnd    ( 730  )
    )u_hdmi_vtc(
        .I_vtc_rstn    ( S_hdmi_rst_n    ),
        .I_vtc_clk     ( S_hdmi_pixel_clk ),
        .O_vtc_vs      ( S_hdmi_vsync     ),
        .O_vtc_hs      ( S_hdmi_hsync     ),
        .O_vtc_de_valid( S_hdmi_de        ),
        .O_vtc_user    ( S_hdmi_user      ),
        .O_vtc_last    ( S_hdmi_last      )
    );

    hdmi_mixer #(
        .H_OFFSET   ( 0    ),
        .V_OFFSET   ( 0    ),
        .IMG_WIDTH  ( 1280 ),
        .IMG_HEIGHT ( 720  ),
        .DEBUG_MODE ( 0    )
    )u_hdmi_mixer(
        .I_clk           ( S_hdmi_pixel_clk   ),
        .I_rst_n         ( S_hdmi_rst_n       ),
        .I_video_vsync   ( S_hdmi_vsync       ),
        .I_video_hsync   ( S_hdmi_hsync       ),
        .I_video_de      ( S_hdmi_de          ),
        .I_video_user    ( S_hdmi_user        ),
        .I_video_last    ( S_hdmi_last        ),
        .I_debug_status  ( S_hdmi_debug_status),
        .O_video_rd_en   ( S_hdmi_window_rd_en),
        .I_video_rd_data ( S_video_rd_data_fixed ),
        .O_hdmi_vsync    ( S_hdmi_mix_vsync  ),
        .O_hdmi_hsync    ( S_hdmi_mix_hsync  ),
        .O_hdmi_de       ( S_hdmi_mix_de     ),
        .O_hdmi_data     ( S_hdmi_mix_data   )
    );

    // A37: radial colour-shading correction ahead of all processing.
    contest_shading u_contest_shading (
        .clk       ( S_hdmi_pixel_clk  ),
        .rst_n     ( S_hdmi_rst_n      ),
        .in_vsync  ( S_hdmi_mix_vsync  ),
        .in_hsync  ( S_hdmi_mix_hsync  ),
        .in_de     ( S_hdmi_mix_de     ),
        .in_rgb    ( S_hdmi_mix_data   ),
        .k_gain    ( S_options[5] ? S_shade_k_sync : 8'd0 ),
        .s_att     ( S_options[5] ? S_shade_s_sync : 8'd0 ),
        .out_vsync ( S_shd_vsync       ),
        .out_hsync ( S_shd_hsync       ),
        .out_de    ( S_shd_de          ),
        .out_rgb   ( S_shd_data        )
    );

    video_options u_video_options (
        .clk(S_100m_clk),.pixel_clk(S_hdmi_pixel_clk),.camera_clk(S_csi_rx_clk),.rst_n(S_hdmi_rst_n),
        .pixel_vs(S_hdmi_mix_vsync),.camera_sof(S_axis_tuser),
        .valid(S_demo_valid),.cmd(S_demo_cmd),.data(S_demo_data),.reply(S_options_reply),
        .pixel_config(S_options),.isp_flags(S_isp_flags)
    );
    video_histogram u_video_histogram (
        .clk(S_hdmi_pixel_clk),.rst_n(S_hdmi_rst_n),.in_vs(S_shd_vsync),.in_hs(S_shd_hsync),
        .in_de(S_shd_de),.in_rgb(S_shd_data),.enable(S_options[4]),.roi_only(S_options[10]),
        .roi_valid(S_effect_valid),.x0(S_effect_x0),.y0(S_effect_y0),.x1(S_effect_x1),.y1(S_effect_y1),
        .out_vs(S_hist_vs),.out_hs(S_hist_hs),.out_de(S_hist_de),.out_rgb(S_hist_rgb),.original_rgb(S_hist_original)
    );
    video_stats u_video_stats (
        .ref_clk(S_100m_clk), .pixel_clk(S_hdmi_pixel_clk), .camera_clk(S_csi_rx_clk),
        .rst_n(S_hdmi_rst_n), .camera_sof(S_axis_tuser), .display_vs(S_hdmi_mix_vsync),
        .target_valid(S_target_valid), .target_x(S_target_center_x), .target_y(S_target_center_y),
        .target_w(S_target_width), .target_h(S_target_height),
        .binary_threshold(S_binary_threshold), .edge_threshold(S_edge_threshold),
        .camera_fps(S_camera_fps), .display_fps(S_display_fps),
        .camera_frames(S_camera_frames), .display_frames(S_display_frames), .uart_stats(S_video_stats)
    );

    contest_video_processor #(
        .IMAGE_WIDTH      ( 1280   ),
        .BINARY_THRESHOLD ( 8'd128 )
    ) u_contest_video_processor (
        .detection_rgb(S_hist_original),
        .roi_only(S_options[8]),.roi_valid(S_effect_valid),.original_rgb(S_vision_original),
        .roi_x0(S_effect_x0),.roi_y0(S_effect_y0),.roi_x1(S_effect_x1),.roi_y1(S_effect_y1),
        .binary_threshold(S_binary_threshold), .edge_threshold(S_edge_threshold),
        .clk            ( S_hdmi_pixel_clk  ),
        .rst_n          ( S_hdmi_rst_n      ),
        .in_vsync       ( S_hist_vs       ),
        .in_hsync       ( S_hist_hs       ),
        .in_de          ( S_hist_de          ),
        .in_rgb         ( S_hist_rgb        ),
        .requested_mode ( S_video_mode_sync ),
        .out_vsync      ( S_vision_vsync    ),
        .out_hsync      ( S_vision_hsync    ),
        .out_de         ( S_vision_de       ),
        .out_rgb        ( S_vision_data     ),
        .out_edge       ( S_vision_edge     ),
        .active_mode    ( S_video_mode_active )
    );

    wire S_region_quad;
    wire [11:0] S_qx0,S_qy0,S_qx1,S_qy1,S_qx2,S_qy2,S_qx3,S_qy3;
    contest_target_tracker #(
        .IMAGE_WIDTH    ( 1280 ),
        .IMAGE_HEIGHT   ( 720  ),
        .MIN_RUN_PIXELS ( 32   )
    ) u_contest_target_tracker (
        .region_quad(S_region_quad),
        .region_qx0(S_qx0),.region_qy0(S_qy0),.region_qx1(S_qx1),.region_qy1(S_qy1),
        .region_qx2(S_qx2),.region_qy2(S_qy2),.region_qx3(S_qx3),.region_qy3(S_qy3),
        .detection_rgb(S_vision_original),
        .clk             ( S_hdmi_pixel_clk    ),
        .rst_n           ( S_hdmi_rst_n        ),
        .in_vsync        ( S_vision_vsync      ),
        .in_hsync        ( S_vision_hsync      ),
        .in_de           ( S_vision_de         ),
        .in_rgb          ( S_vision_data       ),
        .in_edge         ( S_vision_edge       ),
        .draw_boxes      ( S_fx_mode_sync == 3'd0 ),
        .region_union    ( S_region_union_sync ),
        .region_lock     ( S_region_lock_sync  ),
        .video_mode      ( S_video_mode_active ),
        .target_color    ( S_target_color_sync ),
        .hue_center8     ( S_hue_center_sync   ),
        .hue_halfwidth8  ( S_hue_halfw_sync    ),
        .sat_min         ( S_sat_min_sync      ),
        .val_min         ( S_val_min_sync      ),
        .out_vsync       ( S_track_vsync       ),
        .out_hsync       ( S_track_hsync       ),
        .out_de          ( S_track_de          ),
        .out_rgb         ( S_track_data        ),
        .clean_rgb       ( S_track_clean_data  ),
        .clean_edge      ( S_track_clean_edge  ),
        .region_valid    ( S_region_valid      ),
        .region_x0       ( S_region_x0         ),
        .region_y0       ( S_region_y0         ),
        .region_x1       ( S_region_x1         ),
        .region_y1       ( S_region_y1         ),
        .target_valid    ( S_color_valid      ),
        .target_center_x ( S_color_x   ),
        .target_center_y ( S_color_y   ),
        .target_width    ( S_color_w      ),
        .target_height   ( S_color_h     )
    );

    // A20: effects inside the tracked region (mosaic / cartoon / edges / tint).
    contest_region_fx u_contest_region_fx (
        .region_quad(S_options[9] && S_options[11] && S_region_union_sync && S_region_quad),
        .region_qx0(S_qx0),.region_qy0(S_qy0),.region_qx1(S_qx1),.region_qy1(S_qy1),
        .region_qx2(S_qx2),.region_qy2(S_qy2),.region_qx3(S_qx3),.region_qy3(S_qy3),

        .clk          ( S_hdmi_pixel_clk    ),
        .rst_n        ( S_hdmi_rst_n        ),
        .in_vsync     ( S_track_vsync       ),
        .in_hsync     ( S_track_hsync       ),
        .in_de        ( S_track_de          ),
        .in_rgb       ( S_track_data        ),
        .clean_rgb    ( S_track_clean_data  ),
        .in_edge      ( S_track_clean_edge  ),
        .enable       ( 1'b1 ),
        .fx_mode      ( S_fx_mode_sync      ),
        .mosaic_sel   ( S_mosaic_sel_sync   ),
        .region_valid ( (!S_options[9] || S_effect_valid)      ),
        .region_x0    ( (S_options[9] ? S_effect_x0 : 12'd0)         ),
        .region_y0    ( (S_options[9] ? S_effect_y0 : 12'd0)         ),
        .region_x1    ( (S_options[9] ? S_effect_x1 : 12'd1279)         ),
        .region_y1    ( (S_options[9] ? S_effect_y1 : 12'd719)         ),
        .out_vsync    ( S_fx_vsync          ),
        .out_hsync    ( S_fx_hsync          ),
        .out_de       ( S_fx_de             ),
        .out_rgb      ( S_fx_data           ),
        .out_clean    ( S_fx_clean          )
    );

    // A8 motion works on unannotated, pre-histogram RGB and fresh camera stamps.
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n)
      if(!S_hdmi_rst_n)S_motion_pause<=0;else S_motion_pause<={S_motion_pause[0],S_manual_motion};
    video_target_visibility u_target_visibility(.mode(S_video_mode_active),.fx_mode(S_fx_mode_sync),
      .color_valid(S_color_valid),.motion_valid(S_motion_valid),.visible(S_target_valid));
    assign S_target_center_x=S_video_mode_active==5 ? S_motion_x : S_color_x;
    assign S_target_center_y=S_video_mode_active==5 ? S_motion_y : S_color_y;
    assign S_target_width=S_video_mode_active==5 ? S_motion_w : S_color_w;
    assign S_target_height=S_video_mode_active==5 ? S_motion_h : S_color_h;
    video_motion_control u_motion_control(.clk(S_100m_clk),.pixel_clk(S_hdmi_pixel_clk),.rst_n(S_hdmi_rst_n),
      .pixel_vs(S_vision_vsync),.valid(S_demo_valid),.cmd(S_demo_cmd),.data(S_demo_data),.reply(S_motion_reply),
      .threshold(S_motion_threshold),.min_tiles(S_motion_minimum),.reset_toggle(S_motion_reset),
      .paused(S_motion_pause[1]),.warming(S_motion_warming),.scene_change(S_motion_scene),
      .boxes_valid(S_motion_boxes),.changed_cells(S_motion_cells));
    video_motion u_motion(.clk(S_hdmi_pixel_clk),.rst_n(S_hdmi_rst_n),
      .roi_only(S_options[8]),.roi_valid(S_effect_valid),
      .roi_x0(S_effect_x0),.roi_y0(S_effect_y0),.roi_x1(S_effect_x1),.roi_y1(S_effect_y1),
      .vs(S_vision_vsync),.de(S_vision_de),.rgb(S_vision_original),
      .enabled(S_video_mode_active==5),.paused(S_motion_pause[1]),.reset_toggle(S_motion_reset),
      .threshold(S_motion_threshold),.min_tiles(S_motion_minimum),
      .frame_stamp(S_displayed_frame_stamp),.stamp_valid(S_displayed_frame_stamp_valid),
      .boxes_valid(S_motion_boxes),.boxes_x0(S_motion_x0),.boxes_y0(S_motion_y0),
      .boxes_x1(S_motion_x1),.boxes_y1(S_motion_y1),.target_valid(S_motion_valid),
      .target_x(S_motion_x),.target_y(S_motion_y),.target_w(S_motion_w),.target_h(S_motion_h),
      .changed_cells(S_motion_cells),.warming(S_motion_warming),.scene_change(S_motion_scene));
    video_motion_overlay u_motion_overlay(.clk(S_hdmi_pixel_clk),.rst_n(S_hdmi_rst_n),
      .enabled(S_video_mode_active==5),.vs(S_fx_vsync),.hs(S_fx_hsync),.de(S_fx_de),.rgb(S_fx_data),
      .boxes_valid(S_motion_boxes),.x0(S_motion_x0),.y0(S_motion_y0),.x1(S_motion_x1),.y1(S_motion_y1),
      .out_vs(S_motion_vs),.out_hs(S_motion_hs),.out_de(S_motion_de),.out_rgb(S_motion_rgb));

    contest_osd u_contest_osd (
        .motion_count({1'b0,S_motion_boxes[0]}+{1'b0,S_motion_boxes[1]}+{1'b0,S_motion_boxes[2]}),
        .latency_ms(S_latency_ms), .latency_valid(S_latency_valid),
        .target_color(S_target_color_sync),
        .target_x(S_target_center_x), .target_y(S_target_center_y),
        .target_w(S_target_width), .target_h(S_target_height),
        .camera_fps(S_camera_fps), .display_fps(S_display_fps),
        .binary_threshold(S_binary_threshold), .edge_threshold(S_edge_threshold),
        .in_vsync   ( S_motion_vs ),
        .in_hsync   ( S_motion_hs ),
        .in_de      ( S_motion_de    ),
        .in_rgb     ( S_motion_rgb  ),
        .clk        ( S_hdmi_pixel_clk    ),
        .rst_n      ( S_hdmi_rst_n        ),
        .enable     ( S_osd_enable_sync[1]),
        .mode       ( S_video_mode_active ),
        .target_valid ( S_target_valid    ),
        .fps_mode     ( S_fps_mode_syncpx ),
        .out_vsync  ( S_hdmi_out_vsync    ),
        .out_hsync  ( S_hdmi_out_hsync    ),
        .out_de     ( S_hdmi_out_de       ),
        .out_rgb    ( S_hdmi_out_data     )
    );

    // A8: HDMI quantization range.  CEA-861 video timings (720p) default to
    // limited-range RGB, and both the capture card and most monitors expand
    // 16..235 to full scale: measured OSD green 208 arrived as 248 and panel
    // grey 32 as 0.  Everything above 235 therefore displayed as glaring
    // pure white and everything below 16 as crushed black.  Map the final
    // picture into 16..235 so standard displays reproduce it faithfully.
    // Host-switchable (0x34/0x35) for monitors set to full range.
    function [7:0] to_limited_range;
        input [7:0] x;
        reg   [15:0] p;
        begin
            p = x * 8'd219;                       // 0..55845
            to_limited_range = 8'd16 + p[15:8] + {7'd0, p[7]};   // 16..235, rounded
        end
    endfunction
    reg        S_range_limited_meta, S_range_limited_sync;
    reg        S_rng_vsync, S_rng_hsync, S_rng_de;
    reg [23:0] S_rng_data;
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if (!S_hdmi_rst_n) begin
            S_range_limited_meta <= 1'b0;
            S_range_limited_sync <= 1'b0;
            S_rng_vsync <= 1'b0; S_rng_hsync <= 1'b0; S_rng_de <= 1'b0;
            S_rng_data  <= 24'd0;
        end else begin
            S_range_limited_meta <= S_range_limited_ctrl;
            S_range_limited_sync <= S_range_limited_meta;
            S_rng_vsync <= S_hdmi_out_vsync;
            S_rng_hsync <= S_hdmi_out_hsync;
            S_rng_de    <= S_hdmi_out_de;
            S_rng_data  <= S_range_limited_sync ?
                           {to_limited_range(S_hdmi_out_data[23:16]),
                            to_limited_range(S_hdmi_out_data[15:8]),
                            to_limited_range(S_hdmi_out_data[7:0])} :
                           S_hdmi_out_data;
        end
    end

    hdmi_tx u_hdmi_tx(
        .I_pixel_clk        ( S_hdmi_pixel_clk  ),
        .I_serial_clk       ( S_hdmi_serial_clk ),
        .I_rst              ( ~S_hdmi_rst_n     ),
        .I_key_in           ( 1'b0              ),
        .I_edid_read_trig   ( 1'b0              ),
        .O_edid_read_valid  (                   ),
        .O_edid_read_data   (                   ),
        .I_video_rgb_enable ( 1'b1              ),
        .I_video_in_vs      ( S_rng_vsync       ),
        .I_video_in_de      ( S_rng_de          ),
        .I_video_in_user    ( 1'b0              ),
        .I_video_in_valid   ( 1'b0              ),
        .I_video_in_last    ( 1'b0              ),
        .O_video_in_ready   (                   ),
        .I_video_in_data    ( S_rng_data        ),
        .I_audio_valid      ( 1'b0              ),
        .I_audio_left_data  ( 24'd0             ),
        .I_audio_right_data ( 24'd0             ),
        .I_i2s_BCLK         ( 1'b0              ),
        .I_i2s_LRCK         ( 1'b0              ),
        .I_i2s_DOUT         ( 1'b0              ),
        .O_ddc_scl          (                   ),
        .O_hdmi_clk_p       (                   ),
        .O_hdmi_tx_p        (                   ),
        .O_tmds_ch0_p       ( O_tmds_ch0_p      ),
        .O_tmds_ch1_p       ( O_tmds_ch1_p      ),
        .O_tmds_ch2_p       ( O_tmds_ch2_p      ),
        .O_tmds_clk_p       ( O_tmds_clk_p      )
    );


    
endmodule
