/*******************************MILIANKE*******************************
*Company : MiLianKe Electronic Technology Co., Ltd.
*WebSite:https://www.milianke.com
*TechWeb:https://www.uisrc.com
*tmall-shop:https://milianke.tmall.com
*jd-shop:https://milianke.jd.com
*taobao-shop1: https://milianke.taobao.com
*Create Date: 2023/03/23
*Module Name:
*File Name:
*Description: 
*The reference demo provided by Milianke is only used for learning. 
*We cannot ensure that the demo itself is free of bugs, so users 
*should be responsible for the technical problems and consequences
*caused by the use of their own products.
*Copyright: Copyright (c) MiLianKe
*All rights reserved.
*Revision: 1.0
*Signal description
*1) I_ input
*2) O_ output
*3) IO_ input output
*4) S_ system internal signal
*5) _n activ low
*6) _dg debug signal 
*7) _r delay or register
*8) _s state mechine
*********************************************************************/
/*********ae_set �ֶ��������AE ***********
--�汾��1.0
*********************************************************************/

module ae_set
(
input wire I_clk,
input wire I_rst,
input wire [3:0]I_btn,
input wire I_meter_clk,
input wire I_meter_vsync,
input wire I_meter_de,
input wire [23:0] I_meter_rgb,
input wire I_ae_cfg_done,
input wire I_cam_cfg_done,
// A6: upper limit of the logical gain index (79 = 0x073F ... 143 = 0x1F3F).
// Logical index minus 16 is the SC500GainTbl entry.  Host-settable so the
// brightness/noise trade-off in dim rooms can be tuned without rebuilding.
input wire [15:0] I_ag_max,
input wire [1:0]  I_fps_min,     // A39: lowest frame rate AE may use: 0 = 60 only (off), 1 = 30, 2 = 15
output wire [1:0] O_fps_mode,    // A39: current step 0 = 60 fps, 1 = 30, 2 = 15
output wire [15:0] O_vts,        // A39: sensor frame length (lines) for the current step
output wire O_ae_req,
output wire [15:0] O_ae,
output wire [15:0] O_ag
);

// Frame statistics are measured on one pixel out of every 16.  The result is
// deliberately reduced to a stable two-bit request before crossing into the
// 24 MHz sensor-control domain: 0=hold, 1=brighter, 2=darker.
reg        meter_vsync_d = 1'b0;
reg        meter_de_d = 1'b0;
reg [11:0] meter_x = 12'd0;
reg [11:0] meter_y = 12'd0;
reg [3:0]  meter_sample_div = 4'd0;
reg [16:0] meter_total = 17'd0;
reg [16:0] meter_dark = 17'd0;
reg [16:0] meter_bright = 17'd0;
reg [16:0] meter_full_total = 17'd0;
reg [16:0] meter_full_bright = 17'd0;
reg [24:0] meter_luma_acc = 25'd0;
reg [1:0]  meter_request = 2'd0;
reg        meter_frame_toggle = 1'b0;
wire [9:0] meter_luma_sum = {2'b0,I_meter_rgb[23:16]} +
                             {1'b0,I_meter_rgb[15:8],1'b0} +
                             {2'b0,I_meter_rgb[7:0]};
wire [7:0] meter_luma = meter_luma_sum[9:2];

always @(posedge I_meter_clk or posedge I_rst) begin
	if(I_rst) begin
		meter_vsync_d <= 1'b0;
		meter_de_d <= 1'b0;
		meter_x <= 12'd0;
		meter_y <= 12'd0;
		meter_sample_div <= 4'd0;
		meter_total <= 17'd0;
		meter_dark <= 17'd0;
		meter_bright <= 17'd0;
		meter_full_total <= 17'd0;
		meter_full_bright <= 17'd0;
		meter_luma_acc <= 25'd0;
		meter_request <= 2'd0;
		meter_frame_toggle <= 1'b0;
	end else begin
		meter_vsync_d <= I_meter_vsync;
		meter_de_d <= I_meter_de;
		if(I_meter_vsync && !meter_vsync_d) begin
			// Closed-loop average-luma metering with a wide hysteresis band.
			// The display curve tops out near code 239, so the old >242
			// highlight test could never see normal clipped image pixels.
			// Centre-weighted average controls the subject.  A separate
			// full-frame highlight meter prevents windows, lamps and screens
			// near the border from clipping while the centre is lifted.
			// A4: the full-frame highlight guard used to force "darker" as
			// soon as 1/64 of the frame exceeded 220.  Any lamp or monitor in
			// view then drove the whole scene dark and the tone curve had to
			// lift it back, which is what produced the grey, hazy look.  Light
			// sources are allowed to clip; only large bright areas (1/8 of the
			// frame, e.g. a window) now pull exposure down, and "brighter" is
			// permitted until 1/16 of the frame is bright.
			// A10: mean-luma target band raised from 96..136 to 112..152 so
			// the picture is brighter through real exposure rather than curve.
			if((meter_total != 0) &&
			   ((meter_luma_acc > ({1'b0,meter_total,7'b0} +
			                       {4'b0,meter_total,4'b0} +
			                       {5'b0,meter_total,3'b0})) ||
			    (meter_bright > (meter_total >> 2)) ||
			    ((meter_full_total != 0) &&
			     (meter_full_bright > (meter_full_total >> 3)))))
				meter_request <= 2'd2;
			else if((meter_total != 0) &&
			        (meter_luma_acc < ({2'b0,meter_total,6'b0} +
			                           {3'b0,meter_total,5'b0} +
			                           {4'b0,meter_total,4'b0})) &&
			        (meter_bright < (meter_total >> 2)) &&
			        ((meter_full_total == 0) ||
			         (meter_full_bright < (meter_full_total >> 4))))
				meter_request <= 2'd1;
			else
				meter_request <= 2'd0;
			meter_total <= 17'd0;
			meter_dark <= 17'd0;
			meter_bright <= 17'd0;
			meter_full_total <= 17'd0;
			meter_full_bright <= 17'd0;
			meter_luma_acc <= 25'd0;
			meter_sample_div <= 4'd0;
			meter_x <= 12'd0;
			meter_y <= 12'd0;
			meter_frame_toggle <= ~meter_frame_toggle;
		end else if(I_meter_de) begin
			meter_x <= meter_x + 1'b1;
			meter_sample_div <= meter_sample_div + 1'b1;
			if(meter_sample_div == 4'd0) begin
				meter_full_total <= meter_full_total + 1'b1;
				if(meter_luma > 8'd220)
					meter_full_bright <= meter_full_bright + 1'b1;
				// Centre-weighted 768x432 ROI remains the subject meter.
				if((meter_x >= 12'd256) && (meter_x < 12'd1024) &&
				   (meter_y >= 12'd144) && (meter_y < 12'd576)) begin
					meter_total <= meter_total + 1'b1;
					meter_luma_acc <= meter_luma_acc + meter_luma;
					if(meter_luma < 8'd64)
						meter_dark <= meter_dark + 1'b1;
					if(meter_luma > 8'd220)
						meter_bright <= meter_bright + 1'b1;
				end
			end
		end else begin
			meter_x <= 12'd0;
			if(meter_de_d)
				meter_y <= meter_y + 1'b1;
		end
	end
end

reg [1:0] meter_request_meta = 2'd0;
reg [1:0] meter_request_sync = 2'd0;
reg       meter_toggle_meta = 1'b0;
reg       meter_toggle_sync = 1'b0;
reg       meter_toggle_seen = 1'b0;
reg [2:0] meter_frame_div = 3'd0;
	

reg 	[19:0] 	rc_ae_tick = 0; 
reg 			r_ae_tick = 0; 
reg 	[4:0] 	rc_ag_tick = 0; 
reg 			r_ag_tick = 0; 
	
always @(posedge I_clk or posedge I_rst) begin
	if(I_rst) begin
		rc_ae_tick <= 0; 
		r_ae_tick <= 0; 
		rc_ag_tick <= 0; 
		r_ag_tick <= 0; 
	end else begin
	//	Trigger AE Inc / Dec per 1 ms. 
		rc_ae_tick <= rc_ae_tick + 1; 
		r_ae_tick <= 0; 
		if(rc_ae_tick >= 99999) begin
			rc_ae_tick <= 0; 
			r_ae_tick <= 1; 
		end 
	//	Trigger AG Inc / Dec per 32 ms. 
		rc_ag_tick <= rc_ag_tick + r_ae_tick; 
		r_ag_tick <= (&rc_ag_tick) && r_ae_tick; 
	end
end
	
//	Use button to add / sub idelay values. I_button[0]:IncDatDly, [1]DecDatDly, [2]IncI_clkDly, [3]DecI_clkDly
reg 	[3:0] 	r_button_0 = 0, r_button_1 = 0; 
// I_ag is a logical table index.  79 maps to 0x073f; 80..89 map to the
// validated low-light range 0x0f20..0x0f29.  Never pass a register code
// through I_ag.
reg 	[15:0] 	r_ae_set = 16'd768, r_ag_set = 16'd79;
// A39 auto frame rate.  Exposure values are half-lines; one frame is 1127 lines
// at 60 fps, so the exposure ceiling is 2246 and doubles with each step down.
// Going down (only when the host allows it): exposure at its ceiling and gain
// >= index 96 (about 6x) for four consecutive 8-frame evaluations (~0.5 s).
// At the switch the exposure value doubles and the gain index drops by 32
// (one octave), so brightness is continuous and noise halves.
// Going up: exposure below 3/4 of the faster step's ceiling and gain at the
// floor for ~0.5 s, so the picture fits the faster frame unchanged.
reg  [1:0]  fps_mode = 2'd0;
reg  [2:0]  fps_dn_cnt = 3'd0, fps_up_cnt = 3'd0;
reg  [1:0]  fps_min_meta = 2'd0, fps_min_sync = 2'd0;
wire [15:0] ae_max     = (fps_mode == 2'd0) ? 16'd2246 : (fps_mode == 2'd1) ? 16'd4492 : 16'd8984;
wire [15:0] ae_max_min = (fps_min_sync == 2'd0) ? 16'd2246 : (fps_min_sync == 2'd1) ? 16'd4492 : 16'd8984;
wire [15:0] ae_max_up  = (fps_mode == 2'd1) ? 16'd2246 : 16'd4492;      // ceiling of the next faster step
wire [15:0] ae_up_thr  = ae_max_up - (ae_max_up >> 2);                  // 3/4 of it
assign O_fps_mode = fps_mode;
assign O_vts = (fps_mode == 2'd0) ? 16'd1127 : (fps_mode == 2'd1) ? 16'd2254 : 16'd4508;
always @(posedge I_clk or posedge I_rst) begin
	if(I_rst) begin
		r_button_0 <= 0; 
		r_button_1 <= 0; 
		r_ae_set <= 16'd768;
		r_ag_set <= 16'd79;
		meter_request_meta <= 2'd0;
		meter_request_sync <= 2'd0;
		meter_toggle_meta <= 1'b0;
		meter_toggle_sync <= 1'b0;
		meter_toggle_seen <= 1'b0;
		meter_frame_div <= 3'd0;
		fps_mode <= 2'd0; fps_dn_cnt <= 3'd0; fps_up_cnt <= 3'd0; fps_min_meta <= 2'd0; fps_min_sync <= 2'd0;
	end else begin
		fps_min_meta <= I_fps_min; fps_min_sync <= fps_min_meta;
		// Host lowered the allowance: step up immediately, clamp exposure to the new ceiling.
		if (fps_mode > fps_min_sync) begin
			fps_mode <= fps_min_sync;
			if (r_ae_set > ae_max_min) r_ae_set <= ae_max_min;
			fps_dn_cnt <= 3'd0; fps_up_cnt <= 3'd0;
		end
		meter_request_meta <= meter_request;
		meter_request_sync <= meter_request_meta;
		meter_toggle_meta <= meter_frame_toggle;
		meter_toggle_sync <= meter_toggle_meta;
	//	When button pressed, r_button_1 bit is 1. 
		r_button_0 <= ~I_btn; 
		r_button_1 <= r_button_0; 
	
		if(r_button_1[0]) begin
			r_ae_set <= (r_ae_set > 2) ? r_ae_set - r_ae_tick : r_ae_set; 
		end else if(r_button_1[1]) begin
			r_ae_set <= (r_ae_set < ae_max) ? r_ae_set + r_ae_tick : r_ae_set;
		end else if(meter_toggle_sync != meter_toggle_seen) begin
			meter_toggle_seen <= meter_toggle_sync;
			if(meter_frame_div == 3'd7) begin
				meter_frame_div <= 3'd0;
				// A39: frame-rate step decisions (same 8-frame cadence)
				// A39b: step down as soon as exposure is capped and gain is high (>= index 96,
				// about 6x) -- noise first.  At the switch the exposure value doubles and the
				// gain index drops by 32 (exactly half), so brightness is continuous.
				if ((fps_mode < fps_min_sync) && (r_ae_set >= ae_max) && (r_ag_set >= 16'd96))
					fps_dn_cnt <= fps_dn_cnt + 3'd1;
				else
					fps_dn_cnt <= 3'd0;
				if ((fps_mode != 2'd0) && (r_ae_set <= ae_up_thr) && (r_ag_set <= 16'd81) && (meter_request_sync != 2'd1))
					fps_up_cnt <= fps_up_cnt + 3'd1;
				else
					fps_up_cnt <= 3'd0;
				if (fps_dn_cnt == 3'd3) begin
					fps_mode <= fps_mode + 2'd1; fps_dn_cnt <= 3'd0;
					r_ae_set <= {r_ae_set[14:0], 1'b0};          // x2: the new step's ceiling
					r_ag_set <= r_ag_set - 16'd32;               // /2 (32 table steps per octave)
				end else if (fps_up_cnt == 3'd3) begin
					fps_mode <= fps_mode - 2'd1; fps_up_cnt <= 3'd0;
				end
				if (fps_dn_cnt == 3'd3 || fps_up_cnt == 3'd3) begin
					// frame-rate switch this evaluation: exposure/gain already set above
				end else if(meter_request_sync == 2'd1) begin
					if(r_ae_set < ae_max - 16'd128)
						r_ae_set <= r_ae_set + 16'd32;
					else if(r_ae_set < ae_max)
						r_ae_set <= ae_max;
					// Gain rises only after exposure has reached the frame limit,
					// two table steps (about 6%) per adjustment, up to I_ag_max.
					// A39: above index 96 the frame rate steps down first when allowed.
					else if((fps_mode < fps_min_sync) && (r_ag_set >= 16'd96))
						r_ag_set <= r_ag_set;
					else if(r_ag_set + 16'd2 <= I_ag_max)
						r_ag_set <= r_ag_set + 16'd2;
					else if(r_ag_set < I_ag_max)
						r_ag_set <= I_ag_max;
				end else if(meter_request_sync == 2'd2) begin
					if(r_ag_set > 16'd81)
						r_ag_set <= r_ag_set - 16'd2;
					else if(r_ag_set > 16'd79)
						r_ag_set <= 16'd79;
					else if(r_ae_set > 16'd160)
						r_ae_set <= r_ae_set - 16'd32;
					else if(r_ae_set > 16'd32)
						r_ae_set <= r_ae_set - 16'd32;
				end
			end else begin
				meter_frame_div <= meter_frame_div + 1'b1;
			end
		end
		// Gain buttons are intentionally disabled; AE selects only validated
		// entries 79..89 from SC500GainTbl.
		end
	end
	
//	Update AG / AE when changed. 
reg 	[15:0] 	r_ae ; 
reg 	[15:0] 	r_ag ; 
reg 	[1:0] 	r_fps = 2'd0;
reg 			r_ae_req = 0; //synthesis keep
assign O_ae = r_ae;
assign O_ag = r_ag;
assign O_ae_req = r_ae_req;

always @(posedge I_clk or posedge I_rst) begin
	if(I_rst) begin
		r_ae_req <= 0; 
		// Force one sensor update after the static register table finishes.
		r_ae     <= 16'd0;
		r_ag     <= 16'd0;
	end else if(r_ae_req) begin
		r_ae_req <= 0; 
	end else if(((r_ae != r_ae_set) || (r_ag != r_ag_set) || (r_fps != fps_mode))&&(I_cam_cfg_done==1'b1)&&(I_ae_cfg_done==1'b1)) begin
		r_ae     <= r_ae_set; 
		r_ag     <= r_ag_set; 
		r_fps    <= fps_mode;
		r_ae_req <= 1; 
	end
end


endmodule


	
