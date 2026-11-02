`timescale 1ns / 1ps

// Streaming dominant-colour multi-target detector with one-frame-late overlay.
// Up to four independent objects are drawn. The largest object is also
// exported through the legacy single-target UART/status interface.
module contest_target_tracker #(
    parameter integer IMAGE_WIDTH = 1280,
    parameter integer IMAGE_HEIGHT = 720,
    parameter integer MIN_RUN_PIXELS = 32,
    parameter integer BORDER_MARGIN = 8,
    parameter integer MIN_ROW_HITS = 3,
    parameter integer MAX_GAP_ROWS = 4,
    parameter integer HORIZONTAL_JOIN = 16,
    parameter integer COMPONENT_SLOTS = 6    // A22: 16->8, A34: 8->6 (area); 4 markers + target + 1 spare
) (
    input wire clk, input wire rst_n,
    input wire in_vsync, input wire in_hsync, input wire in_de,
    input wire [23:0] in_rgb,
    input wire [23:0] detection_rgb, // preserve pre-effect detection path
    input wire in_edge,                  // Sobel edge flag from the video processor, in_rgb timing (A20)
    input wire draw_boxes,               // 0 = suppress box/cross overlay (region effects active)
    input wire region_union,             // 1 = region is the union of all boxes, 0 = largest box only
    input wire region_lock,              // A32: 1 = hold the current effect region (hands may leave)
    input wire [2:0] video_mode,
    input wire [1:0] target_color,       // 0 red, 1 green, 2 blue, 3 custom hue
    // HSV thresholds from the host (quasi-static, already in the pixel clock domain).
    // Hue values are on a 0..255 scale (255 = 360 degrees); presets for colours 0..2
    // use fixed centres, colour 3 uses hue_center8.  Half-width applies to all.
    input wire [7:0] hue_center8,
    input wire [7:0] hue_halfwidth8,
    input wire [7:0] sat_min,            // minimum chroma (max-min), 0..255
    input wire [7:0] val_min,            // minimum brightness (max channel), 0..255
    output reg out_vsync, output reg out_hsync, output reg out_de,
    output reg [23:0] out_rgb,
    output reg [23:0] clean_rgb,       // out_rgb timing, without box/cross overlays (for region effects)
    output reg clean_edge,             // in_edge delayed to out_rgb timing (A20)
    // A20: effect region for contest_region_fx, updated at frame start.
    output reg region_valid,
    output reg [11:0] region_x0, output reg [11:0] region_y0,
    output reg [11:0] region_x1, output reg [11:0] region_y1,
    // A33: four-marker quadrilateral (corners TL, TR, BR, BL; valid when region_quad)
    output reg region_quad,
    output reg [11:0] region_qx0, output reg [11:0] region_qy0,
    output reg [11:0] region_qx1, output reg [11:0] region_qy1,
    output reg [11:0] region_qx2, output reg [11:0] region_qy2,
    output reg [11:0] region_qx3, output reg [11:0] region_qy3,
    output reg target_valid,
    output reg [11:0] target_center_x, output reg [11:0] target_center_y,
    output reg [11:0] target_width, output reg [11:0] target_height
);
    reg [19:0] best_count [0:3];
    reg [11:0] best_min_x [0:3],best_max_x[0:3],best_min_y[0:3],best_max_y[0:3];
    wire [7:0] eff_sat_min;
    localparam [2:0] MODE_TRACK = 3'd4;

    // ------------------------------------------------------------------
    // A11: HSV colour classification.  Hue is computed on a 0..1535 scale
    // (six 256-wide sectors).  Chroma C = max-min stands in for saturation
    // and V = max for brightness.  A pixel matches when the circular hue
    // distance to the target centre is within the half-width, C >= sat_min,
    // V >= val_min and C >= V/4 (rejects near-grey highlights).  Compared
    // with the previous fixed RGB thresholds this is largely independent of
    // illumination level, and the target hue can be set from the host.
    // The classifier is a four-stage pipeline; the video signals that feed
    // the run/component logic are delayed by the same four clocks.
    // ------------------------------------------------------------------
    localparam integer HSV_LAT = 4;
    wire [7:0] red=detection_rgb[23:16], green=detection_rgb[15:8], blue=detection_rgb[7:0];

    // 65536/C for C = 1..255 (C = 0 never reaches the multiplier).
    // A35: block RAM initialised from recip.hex (was a 256-entry case statement in LUTs).
    (* ram_style = "bram" *) reg [15:0] recip_mem [0:255];
    initial $readmemh("recip.hex", recip_mem);

    // stage 1: max, chroma, hue sector and sector numerator
    reg  [7:0] s1_max, s1_c, s1_num;
    reg  [2:0] s1_sector;
    always @(posedge clk) begin
        if (red >= green && red >= blue) begin
            s1_max <= red;
            if (green >= blue) begin s1_sector <= 3'd0; s1_num <= green - blue; s1_c <= red - blue;   end
            else               begin s1_sector <= 3'd5; s1_num <= red - blue;   s1_c <= red - green;  end
        end else if (green >= blue) begin
            s1_max <= green;
            if (red >= blue)   begin s1_sector <= 3'd1; s1_num <= green - red;  s1_c <= green - blue; end
            else               begin s1_sector <= 3'd2; s1_num <= blue - red;   s1_c <= green - red;  end
        end else begin
            s1_max <= blue;
            if (green >= red)  begin s1_sector <= 3'd3; s1_num <= blue - green; s1_c <= blue - red;   end
            else               begin s1_sector <= 3'd4; s1_num <= red - green;  s1_c <= blue - green; end
        end
    end

    // stage 2: reciprocal lookup
    reg  [7:0] s2_max, s2_c, s2_num;
    reg  [2:0] s2_sector;
    reg  [15:0] s2_recip;
    always @(posedge clk) begin
        s2_max <= s1_max; s2_c <= s1_c; s2_num <= s1_num; s2_sector <= s1_sector;
        s2_recip <= recip_mem[s1_c];
    end

    // stage 3: fraction = num * 65536 / C  (24-bit product, take [23:8])
    reg  [7:0] s3_max, s3_c;
    reg  [2:0] s3_sector;
    reg  [23:0] s3_prod;
    reg  [15:0] s3_satp;
    always @(posedge clk) begin
        s3_max <= s2_max; s3_c <= s2_c; s3_sector <= s2_sector;
        s3_prod <= s2_num * s2_recip;
        s3_satp <= s2_max * eff_sat_min;   // A25: V * S_min, compared with C*256 (relative saturation)
    end

    // stage 4: hue assembly and threshold test
    wire [7:0]  s3_frac  = (|s3_prod[23:16]) ? 8'd255 : s3_prod[15:8];
    wire [10:0] s3_hue   = {s3_sector, s3_frac};                 // sector*256 + frac, 0..1535
    wire [7:0]  preset_center8 = (target_color == 2'd1) ? 8'd110 :  // green 155 deg (A24: measured green marker tape renders at 150..166 deg)
                                 (target_color == 2'd2) ? 8'd160 :  // blue  225 deg (A13: everyday blues sit at 200..240)
                                 (target_color == 2'd3) ? hue_center8 : 8'd0;  // red 0 deg
    // A18: per-colour presets for the remaining thresholds.  Red needs a high
    // chroma floor because wood, skin and cardboard sit at the same hue with
    // low chroma (a 72 floor kept the box on a red tag, 48 swallowed the
    // wooden wall); blue and green have few low-chroma neighbours and a pale
    // anodised-blue object measured chroma ~20 under correct white balance.
    // Colour 3 (custom) uses the host-set values unchanged.
    wire [7:0]  eff_halfw8 = (target_color == 2'd3) ? hue_halfwidth8 :
                             (target_color == 2'd2) ? 8'd22 :
                             (target_color == 2'd1) ? 8'd14 : 8'd18;   // green +-20 deg keeps a 184 deg cyan toy out
    // D1 A3: reject weak colour casts without raising the brightness floor.
    // Relative C/V minima: red 35.2%, green 28.1%, blue 31.25%.
    // Custom HSV and the calibrated hue centres/windows remain unchanged.
    assign eff_sat_min = (target_color == 2'd0) ? 8'd90 :
                        (target_color == 2'd1) ? 8'd72 :
                        (target_color == 2'd2) ? 8'd80 : sat_min;
    wire [7:0]  eff_val_min = (target_color == 2'd3) ? val_min : 8'd40;
    wire [10:0] hue_center   = {preset_center8, 3'b0} - {2'b0, preset_center8, 1'b0};   // *6 = *8 - *2
    wire [10:0] hue_halfw    = {eff_halfw8, 3'b0} - {2'b0, eff_halfw8, 1'b0};
    wire [10:0] hue_diff_raw = (s3_hue >= hue_center) ? (s3_hue - hue_center) : (hue_center - s3_hue);
    wire [10:0] hue_diff     = (hue_diff_raw > 11'd768) ? (11'd1536 - hue_diff_raw) : hue_diff_raw;
    reg         s4_hit;
    always @(posedge clk) begin
        // A25: saturation test is relative (C/V >= S_min/256) so a red object in a
        // dim scene still passes while skin (~0.31) and wood (~0.33) stay out;
        // a small absolute chroma floor rejects grey noise.
        s4_hit <= (hue_diff <= hue_halfw) &&
                  ({s3_c, 8'b0} >= s3_satp) &&
                  (s3_c >= 8'd20) &&
                  (s3_max >= eff_val_min);
    end

    // video signals delayed to align with s4_hit
    reg  [HSV_LAT-1:0]      dly_vsync, dly_hsync, dly_de, dly_edge;
    reg  [23:0]             dly_rgb [0:HSV_LAT-1];
    integer dly_i;
    always @(posedge clk) begin
        dly_vsync <= {dly_vsync[HSV_LAT-2:0], in_vsync};
        dly_hsync <= {dly_hsync[HSV_LAT-2:0], in_hsync};
        dly_de    <= {dly_de[HSV_LAT-2:0],    in_de};
        dly_edge  <= {dly_edge[HSV_LAT-2:0],  in_edge};
        dly_rgb[0] <= in_rgb;
        for (dly_i = 1; dly_i < HSV_LAT; dly_i = dly_i + 1) dly_rgb[dly_i] <= dly_rgb[dly_i-1];
    end
    wire        d_vsync = dly_vsync[HSV_LAT-1];
    wire        d_hsync = dly_hsync[HSV_LAT-1];
    wire        d_de    = dly_de[HSV_LAT-1];
    wire        d_edge  = dly_edge[HSV_LAT-1];
    wire [23:0] d_rgb   = dly_rgb[HSV_LAT-1];

    // A20: union of the valid best boxes, computed combinationally from the
    // best_* candidates so it can be latched together with them at vsync.
    reg        u_valid;
    reg [2:0]  u_cnt;                    // A32: number of markers in the union
    reg [11:0] u_x0, u_y0, u_x1, u_y1;
    integer u_i;
    always @* begin
        u_valid = 1'b0; u_cnt = 3'd0;
        u_x0 = 12'hFFF; u_y0 = 12'hFFF; u_x1 = 12'd0; u_y1 = 12'd0;
        for (u_i = 0; u_i < 4; u_i = u_i + 1) begin
            if ((best_count[u_i] >= MIN_RUN_PIXELS) && (region_union || u_i == 0)) begin
                u_valid = 1'b1; u_cnt = u_cnt + 3'd1;
                if (best_min_x[u_i] < u_x0) u_x0 = best_min_x[u_i];
                if (best_min_y[u_i] < u_y0) u_y0 = best_min_y[u_i];
                if (best_max_x[u_i] > u_x1) u_x1 = best_max_x[u_i];
                if (best_max_y[u_i] > u_y1) u_y1 = best_max_y[u_i];
            end
        end
        // A32: a framing gesture needs at least two markers; a single blob would
        // shrink the effect region to the marker itself.
        if (region_union && (u_cnt < 3'd2)) u_valid = 1'b0;
    end

    // ------------------------------------------------------------------
    // A21: temporal smoothing of the primary target and of the effect region.
    // Each frame the candidate nearest to the previous smoothed centre is
    // chosen (within 200 px Manhattan distance; otherwise the largest), the
    // box coordinates move a quarter of the way toward the measurement, and a
    // lost target is held for HOLD_FRAMES frames before it disappears.
    // ------------------------------------------------------------------
    localparam integer HOLD_FRAMES = 8;
    reg [11:0] sm_x0, sm_y0, sm_x1, sm_y1;
    reg        sm_valid;
    reg [3:0]  sm_hold;
    reg [11:0] rg_x0, rg_y0, rg_x1, rg_y1;
    reg        rg_valid;
    reg [3:0]  rg_hold;
    wire [11:0] sm_cx = (sm_x0 + sm_x1) >> 1;
    wire [11:0] sm_cy = (sm_y0 + sm_y1) >> 1;

    function [11:0] absdiff12;
        input [11:0] a, b;
        begin absdiff12 = (a > b) ? (a - b) : (b - a); end
    endfunction
    function [11:0] iir4;                 // A34: (old + new + 1) / 2 -- one adder per instance (16 instances)
        input [11:0] old, nw;
        reg   [12:0] s;
        begin
            s = {1'b0, old} + {1'b0, nw} + 13'd1;
            iir4 = s[12:1];
        end
    endfunction

    // A23: the nearest-candidate search and the IIR update run as a short
    // clock sequence at the start of vsync (one add/compare per clock) instead
    // of a single combinational cone; A22 missed timing by 5 ns on that cone.
    //   steps 0..15 : candidate c = step[3:2], phase = step[1:0]
    //                 0 centre, 1 |dx|,|dy|, 2 sum, 3 compare/select
    //   step 16     : fallback selection, latch union box
    //   step 17     : secondary boxes, IIR update of sm_*/rg_*, clear best_*
    //   step 18     : latch outputs from the smoothed registers
    reg [1:0]  sel_i;
    reg        sel_found;
    reg [12:0] sel_d, cand_d;
    reg        cand_ok;
    reg        sm_run;
    reg [4:0]  sm_step;
    reg [11:0] sm_cx_r, sm_cy_r;
    reg [11:0] cc_x, cc_y, cd_x, cd_y;
    reg        u_valid_r;
    reg [11:0] u_x0_r, u_y0_r, u_x1_r, u_y1_r;
    wire [1:0] sm_c = sm_step[3:2];

    // A33: with exactly four markers the effect region is the quadrilateral
    // through their centres.  Markers are assigned to corners by their
    // quadrant around the centroid (TL, TR, BR, BL); if a quadrant is empty
    // the region falls back to the bounding rectangle.
    reg [11:0] mk_x [0:3], mk_y [0:3];
    reg        mk_all;
    reg [13:0] cen_sx, cen_sy;
    wire [11:0] cen_x = cen_sx[13:2], cen_y = cen_sy[13:2];
    reg [11:0] q_x [0:3], q_y [0:3];
    reg        q_valid;
    reg [3:0]  q_hold;
    reg [11:0] nq_x [0:3], nq_y [0:3];
    reg [3:0]  nq_found;
    reg [1:0]  mq [0:3];
    integer    qi, qk;
    always @* begin
        for (qi = 0; qi < 4; qi = qi + 1)
            mq[qi] = (mk_y[qi] < cen_y) ? ((mk_x[qi] < cen_x) ? 2'd0 : 2'd1)
                                        : ((mk_x[qi] < cen_x) ? 2'd3 : 2'd2);
        for (qk = 0; qk < 4; qk = qk + 1) begin
            nq_x[qk] = 12'd0; nq_y[qk] = 12'd0; nq_found[qk] = 1'b0;
            for (qi = 0; qi < 4; qi = qi + 1)
                if (mq[qi] == qk[1:0]) begin nq_x[qk] = mk_x[qi]; nq_y[qk] = mk_y[qi]; nq_found[qk] = 1'b1; end
        end
    end
    wire quad_now = mk_all && (&nq_found);

    // A26: before the selection sequence, candidate boxes that overlap or lie
    // within BOX_JOIN pixels of each other are merged (two passes over the six
    // pairs), so one object broken into several components gets one box.
    localparam integer BOX_JOIN = 24;
    reg        mg_run;
    reg [3:0]  mg_step;
    wire [3:0] mg_pair4 = (mg_step >= 4'd6) ? (mg_step - 4'd6) : mg_step;
    wire [2:0] mg_pair  = mg_pair4[2:0];
    wire [1:0] mg_a = (mg_pair < 3'd3) ? 2'd0 : (mg_pair < 3'd5) ? 2'd1 : 2'd2;
    wire [1:0] mg_b = (mg_pair == 3'd0) ? 2'd1 : (mg_pair == 3'd1) ? 2'd2 : (mg_pair == 3'd2) ? 2'd3 :
                      (mg_pair == 3'd3) ? 2'd2 : 2'd3;
    wire       mg_near = (best_count[mg_a] != 20'd0) && (best_count[mg_b] != 20'd0) &&
                         ({1'b0, best_min_x[mg_a]} <= {1'b0, best_max_x[mg_b]} + BOX_JOIN) &&
                         ({1'b0, best_min_x[mg_b]} <= {1'b0, best_max_x[mg_a]} + BOX_JOIN) &&
                         ({1'b0, best_min_y[mg_a]} <= {1'b0, best_max_y[mg_b]} + BOX_JOIN) &&
                         ({1'b0, best_min_y[mg_b]} <= {1'b0, best_max_y[mg_a]} + BOX_JOIN);

    reg [11:0] pixel_x, pixel_y;
    reg vsync_previous, de_previous;
    wire inside=(pixel_x>=BORDER_MARGIN) && (pixel_x<IMAGE_WIDTH-BORDER_MARGIN) &&
                (pixel_y>=BORDER_MARGIN) && (pixel_y<IMAGE_HEIGHT-BORDER_MARGIN);
    // A27: 3x3 majority filter on the colour mask removes isolated noise pixels
    // before connected-component labelling without thinning porous objects
    // (A25's erode+dilate broke a low-saturation can into crumbs).  The stage
    // delays the mask by 3 clocks and centres on (x-1, y-1); see contest_morph3x3.v.
    localparam integer MORPH_DX = 4;
    localparam integer MORPH_DY = 1;
    wire       hit_m;
    contest_morph3x3 #(.MODE(2)) u_majority (.clk(clk), .in_de(d_de), .in_bit(s4_hit), .out_bit(hit_m));
    wire color_hit = hit_m && inside;
    wire [11:0] px_m = (pixel_x > MORPH_DX) ? pixel_x - MORPH_DX : 12'd0;   // mask pixel's true column
    wire [11:0] py_m = (pixel_y > MORPH_DY) ? pixel_y - MORPH_DY : 12'd0;   // mask pixel's true row

    reg run_active;
    reg [11:0] run_start_x, run_count;
    wire [11:0] run_end_x=px_m-12'd1;

    reg comp_active [0:COMPONENT_SLOTS-1];
    reg [11:0] comp_min_x [0:COMPONENT_SLOTS-1];
    reg [11:0] comp_max_x [0:COMPONENT_SLOTS-1];
    reg [11:0] comp_min_y [0:COMPONENT_SLOTS-1];
    reg [11:0] comp_max_y [0:COMPONENT_SLOTS-1];
    reg [11:0] comp_last_y[0:COMPONENT_SLOTS-1];
    reg [19:0] comp_count [0:COMPONENT_SLOTS-1];

    integer match_index, free_index, search_i;
    always @* begin
        match_index=-1;
        free_index=-1;
        for(search_i=0;search_i<COMPONENT_SLOTS;search_i=search_i+1) begin
            if(!comp_active[search_i] && free_index<0) free_index=search_i;
            if(comp_active[search_i] && match_index<0 && pixel_y>=comp_last_y[search_i] &&
               (pixel_y-comp_last_y[search_i]<=MAX_GAP_ROWS) &&
               ({1'b0,run_end_x}+HORIZONTAL_JOIN>={1'b0,comp_min_x[search_i]}) &&
               ({1'b0,comp_max_x[search_i]}+HORIZONTAL_JOIN>={1'b0,run_start_x}))
                match_index=search_i;
        end
    end

    function candidate_valid;
        input [19:0] count;
        input [11:0] min_x,max_x,min_y,max_y;
        reg [12:0] w,h;
        begin
            w={1'b0,max_x}-{1'b0,min_x}+13'd1;
            h={1'b0,max_y}-{1'b0,min_y}+13'd1;
            candidate_valid=(count>=MIN_RUN_PIXELS) && (w>=13'd8) && (h>=13'd8) &&
                            (w<=(h<<2)+(h<<1)) && (h<=(w<<2)+(w<<1));   // A32: aspect up to 6:1
        end
    endfunction

    reg box_valid[0:3];
    reg [11:0] box_min_x[0:3],box_max_x[0:3],box_min_y[0:3],box_max_y[0:3];
    reg [3:0] gc_index;

    integer box_i;
    reg any_box_edge,any_center_cross;
    reg [11:0] cx,cy;
    always @* begin
        any_box_edge=1'b0;
        any_center_cross=1'b0;
        cx=12'd0; cy=12'd0;
        for(box_i=0;box_i<4;box_i=box_i+1) begin
            cx=(box_min_x[box_i]+box_max_x[box_i])>>1;
            cy=(box_min_y[box_i]+box_max_y[box_i])>>1;
            if(box_valid[box_i]) begin
                if((box_i==0)&&(pixel_x>=box_min_x[box_i])&&(pixel_x<=box_max_x[box_i])&&   // A34: rectangle only for the primary box
                   (((pixel_y>=box_min_y[box_i])&&(pixel_y<=box_min_y[box_i]+12'd1))||
                    ((pixel_y<=box_max_y[box_i])&&(pixel_y+12'd1>=box_max_y[box_i])))) any_box_edge=1'b1;
                if((box_i==0)&&(pixel_y>=box_min_y[box_i])&&(pixel_y<=box_max_y[box_i])&&
                   (((pixel_x>=box_min_x[box_i])&&(pixel_x<=box_min_x[box_i]+12'd1))||
                    ((pixel_x<=box_max_x[box_i])&&(pixel_x+12'd1>=box_max_x[box_i])))) any_box_edge=1'b1;
                if((((pixel_x+12'd8)>=cx)&&(pixel_x<=cx+12'd8)&&(pixel_y>=cy)&&(pixel_y<=cy+12'd1))||
                   (((pixel_y+12'd8)>=cy)&&(pixel_y<=cy+12'd8)&&(pixel_x>=cx)&&(pixel_x<=cx+12'd1))) any_center_cross=1'b1;
            end
        end
    end

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            pixel_x<=0; pixel_y<=0; vsync_previous<=0; de_previous<=0;
            run_active<=0; run_start_x<=0; run_count<=0; gc_index<=0;
            target_valid<=0; target_center_x<=0; target_center_y<=0; target_width<=0; target_height<=0;
            out_vsync<=0; out_hsync<=0; out_de<=0; out_rgb<=0; clean_rgb<=0; clean_edge<=0;
            region_valid<=0; region_x0<=0; region_y0<=0; region_x1<=0; region_y1<=0;
            sm_x0<=0; sm_y0<=0; sm_x1<=0; sm_y1<=0; sm_valid<=0; sm_hold<=0;
            mg_run<=0; mg_step<=0;
            sm_run<=0; sm_step<=0; sel_i<=0; sel_found<=0; sel_d<=0; cand_d<=0; cand_ok<=0;
            sm_cx_r<=0; sm_cy_r<=0; cc_x<=0; cc_y<=0; cd_x<=0; cd_y<=0;
            u_valid_r<=0; u_x0_r<=0; u_y0_r<=0; u_x1_r<=0; u_y1_r<=0;
            mk_all<=0; cen_sx<=0; cen_sy<=0; q_valid<=0; q_hold<=0;
            for(i=0;i<4;i=i+1) begin mk_x[i]<=0; mk_y[i]<=0; q_x[i]<=0; q_y[i]<=0; end
            region_quad<=0; region_qx0<=0; region_qy0<=0; region_qx1<=0; region_qy1<=0;
            region_qx2<=0; region_qy2<=0; region_qx3<=0; region_qy3<=0;
            rg_x0<=0; rg_y0<=0; rg_x1<=0; rg_y1<=0; rg_valid<=0; rg_hold<=0;
            for(i=0;i<COMPONENT_SLOTS;i=i+1) begin
                comp_active[i]<=0; comp_min_x[i]<=0; comp_max_x[i]<=0; comp_min_y[i]<=0;
                comp_max_y[i]<=0; comp_last_y[i]<=0; comp_count[i]<=0;
            end
            for(i=0;i<4;i=i+1) begin
                best_count[i]<=0; best_min_x[i]<=0; best_max_x[i]<=0; best_min_y[i]<=0; best_max_y[i]<=0;
                box_valid[i]<=0; box_min_x[i]<=0; box_max_x[i]<=0; box_min_y[i]<=0; box_max_y[i]<=0;
            end
        end else begin
            vsync_previous<=d_vsync;
            de_previous<=d_de;
            if (mg_run) begin
                if (mg_near) begin
                    if (best_min_x[mg_b] < best_min_x[mg_a]) best_min_x[mg_a] <= best_min_x[mg_b];
                    if (best_max_x[mg_b] > best_max_x[mg_a]) best_max_x[mg_a] <= best_max_x[mg_b];
                    if (best_min_y[mg_b] < best_min_y[mg_a]) best_min_y[mg_a] <= best_min_y[mg_b];
                    if (best_max_y[mg_b] > best_max_y[mg_a]) best_max_y[mg_a] <= best_max_y[mg_b];
                    best_count[mg_a] <= best_count[mg_a] + best_count[mg_b];
                    best_count[mg_b] <= 20'd0;
                end
                if (mg_step == 4'd11) begin mg_run <= 1'b0; sm_run <= 1'b1; sm_step <= 5'd0; end
                else mg_step <= mg_step + 4'd1;
            end
            if (sm_run) begin
                sm_step <= sm_step + 5'd1;
                if (!sm_step[4]) begin
                    case (sm_step[1:0])
                    2'd0: begin
                        cc_x <= (best_min_x[sm_c] + best_max_x[sm_c]) >> 1;
                        cc_y <= (best_min_y[sm_c] + best_max_y[sm_c]) >> 1;
                        cand_ok <= (best_count[sm_c] >= MIN_RUN_PIXELS);
                    end
                    2'd1: begin cd_x <= absdiff12(cc_x, sm_cx_r); cd_y <= absdiff12(cc_y, sm_cy_r); end
                    2'd2: begin cand_d <= {1'b0, cd_x} + {1'b0, cd_y}; end
                    2'd3: begin
                        if (cand_ok) begin
                            if (!sm_valid) begin
                                if (!sel_found) begin sel_i <= sm_c; sel_found <= 1'b1; end   // no history: largest
                            end else if ((cand_d < sel_d) && (cand_d < 13'd200)) begin
                                sel_d <= cand_d; sel_i <= sm_c; sel_found <= 1'b1;
                            end
                        end
                    end
                    endcase
                end else if (sm_step == 5'd16) begin
                    if (!sel_found && sm_valid && (best_count[0] >= MIN_RUN_PIXELS)) begin
                        sel_i <= 2'd0; sel_found <= 1'b1;                                  // everything moved far: re-acquire largest
                    end
                    u_valid_r <= u_valid; u_x0_r <= u_x0; u_y0_r <= u_y0; u_x1_r <= u_x1; u_y1_r <= u_y1;
                    for (i = 0; i < 4; i = i + 1) begin
                        mk_x[i] <= (best_min_x[i] + best_max_x[i]) >> 1;
                        mk_y[i] <= (best_min_y[i] + best_max_y[i]) >> 1;
                    end
                    mk_all <= region_union && (u_cnt == 3'd4);
                end else if (sm_step == 5'd17) begin
                    cen_sx <= {2'b0, mk_x[0]} + {2'b0, mk_x[1]} + {2'b0, mk_x[2]} + {2'b0, mk_x[3]};
                    cen_sy <= {2'b0, mk_y[0]} + {2'b0, mk_y[1]} + {2'b0, mk_y[2]} + {2'b0, mk_y[3]};
                    // Secondary boxes: raw candidates other than the selected one.
                    for(i=1;i<4;i=i+1) begin
                        box_valid[i]<=(best_count[i]>=MIN_RUN_PIXELS) && (sel_i != i[1:0]);
                        box_min_x[i]<=best_min_x[i]; box_max_x[i]<=best_max_x[i];
                        box_min_y[i]<=best_min_y[i]; box_max_y[i]<=best_max_y[i];
                    end
                    for(i=0;i<4;i=i+1) best_count[i]<=0;
                    // Primary target: nearest candidate, smoothed, held when lost.
                    if (sel_found) begin
                        if (sm_valid) begin
                            sm_x0 <= iir4(sm_x0, best_min_x[sel_i]); sm_x1 <= iir4(sm_x1, best_max_x[sel_i]);
                            sm_y0 <= iir4(sm_y0, best_min_y[sel_i]); sm_y1 <= iir4(sm_y1, best_max_y[sel_i]);
                        end else begin
                            sm_x0 <= best_min_x[sel_i]; sm_x1 <= best_max_x[sel_i];
                            sm_y0 <= best_min_y[sel_i]; sm_y1 <= best_max_y[sel_i];
                        end
                        sm_valid <= 1'b1; sm_hold <= 4'd0;
                    end else if (sm_valid) begin
                        if (sm_hold == HOLD_FRAMES-1) sm_valid <= 1'b0;
                        else sm_hold <= sm_hold + 4'd1;
                    end
                    // Effect region: same treatment; frozen while region_lock is set (A32).
                    if (region_lock) begin
                        rg_hold <= 4'd0;                   // keep whatever region we have
                    end else if (u_valid_r) begin
                        if (rg_valid) begin
                            rg_x0 <= iir4(rg_x0, u_x0_r); rg_x1 <= iir4(rg_x1, u_x1_r);
                            rg_y0 <= iir4(rg_y0, u_y0_r); rg_y1 <= iir4(rg_y1, u_y1_r);
                        end else begin
                            rg_x0 <= u_x0_r; rg_x1 <= u_x1_r; rg_y0 <= u_y0_r; rg_y1 <= u_y1_r;
                        end
                        rg_valid <= 1'b1; rg_hold <= 4'd0;
                    end else if (rg_valid) begin
                        if (rg_hold == HOLD_FRAMES-1) rg_valid <= 1'b0;
                        else rg_hold <= rg_hold + 4'd1;
                    end
                end else if (sm_step == 5'd18) begin
                    // Outputs and the primary overlay box use the smoothed values.
                    box_valid[0]<=sm_valid;
                    box_min_x[0]<=sm_x0; box_max_x[0]<=sm_x1; box_min_y[0]<=sm_y0; box_max_y[0]<=sm_y1;
                    region_valid<=rg_valid;
                    region_x0<=rg_x0; region_y0<=rg_y0; region_x1<=rg_x1; region_y1<=rg_y1;
                    target_valid<=sm_valid;
                    target_center_x<=sm_cx;
                    target_center_y<=sm_cy;
                    target_width<=sm_x1-sm_x0+12'd1;
                    target_height<=sm_y1-sm_y0+12'd1;
                    // A33: quadrilateral corners, smoothed like the rectangle, held when lost
                    if (!region_lock) begin
                        if (quad_now) begin
                            for (i = 0; i < 4; i = i + 1) begin
                                q_x[i] <= q_valid ? iir4(q_x[i], nq_x[i]) : nq_x[i];
                                q_y[i] <= q_valid ? iir4(q_y[i], nq_y[i]) : nq_y[i];
                            end
                            q_valid <= 1'b1; q_hold <= 4'd0;
                        end else if (q_valid) begin
                            if (q_hold == HOLD_FRAMES-1) q_valid <= 1'b0;
                            else q_hold <= q_hold + 4'd1;
                        end
                    end
                end else if (sm_step == 5'd19) begin
                    region_quad <= q_valid;
                    region_qx0 <= q_x[0]; region_qy0 <= q_y[0];
                    region_qx1 <= q_x[1]; region_qy1 <= q_y[1];
                    region_qx2 <= q_x[2]; region_qy2 <= q_y[2];
                    region_qx3 <= q_x[3]; region_qy3 <= q_y[3];
                    sm_run <= 1'b0;
                end
            end
            if(d_vsync) begin
                pixel_x<=0; pixel_y<=0; run_active<=0; run_count<=0; gc_index<=0;
                if(!vsync_previous) begin
                    // Start the per-frame selection/smoothing sequence (see A23 note above).
                    mg_run <= 1'b1; mg_step <= 4'd0;                 // A26: merge first, then select
                    sel_found <= 1'b0; sel_i <= 2'd0; sel_d <= 13'h1FFF;
                    sm_cx_r <= sm_cx; sm_cy_r <= sm_cy;
                    for(i=0;i<COMPONENT_SLOTS;i=i+1) comp_active[i]<=0;
                end
            end else if(d_de) begin
                pixel_x<=pixel_x+12'd1;
                if(color_hit) begin
                    if(!run_active) begin run_active<=1; run_start_x<=px_m; run_count<=12'd1; end
                    else run_count<=run_count+12'd1;
                end else if(run_active) begin
                    run_active<=0; run_count<=0;
                    if(run_count>=MIN_ROW_HITS) begin
                        if(match_index>=0) begin
                            if(run_start_x<comp_min_x[match_index]) comp_min_x[match_index]<=run_start_x;
                            if(run_end_x>comp_max_x[match_index]) comp_max_x[match_index]<=run_end_x;
                            comp_max_y[match_index]<=py_m; comp_last_y[match_index]<=pixel_y;
                            comp_count[match_index]<=comp_count[match_index]+run_count;
                        end else if(free_index>=0) begin
                            comp_active[free_index]<=1; comp_min_x[free_index]<=run_start_x; comp_max_x[free_index]<=run_end_x;
                            comp_min_y[free_index]<=py_m; comp_max_y[free_index]<=py_m; comp_last_y[free_index]<=pixel_y;
                            comp_count[free_index]<=run_count;
                        end
                    end
                end
            end else begin
                pixel_x<=0; run_active<=0; run_count<=0;
                if(de_previous) begin pixel_y<=pixel_y+12'd1; gc_index<=0; end
                else begin
                    gc_index<=(gc_index==COMPONENT_SLOTS-1) ? 4'd0 : gc_index+4'd1;   // wrap within the slot count
                    if(comp_active[gc_index] && pixel_y>comp_last_y[gc_index]+MAX_GAP_ROWS) begin
                        comp_active[gc_index]<=0;
                        if(candidate_valid(comp_count[gc_index],comp_min_x[gc_index],comp_max_x[gc_index],comp_min_y[gc_index],comp_max_y[gc_index])) begin
                            if(comp_count[gc_index]>best_count[0]) begin
                                best_count[3]<=best_count[2]; best_min_x[3]<=best_min_x[2]; best_max_x[3]<=best_max_x[2]; best_min_y[3]<=best_min_y[2]; best_max_y[3]<=best_max_y[2];
                                best_count[2]<=best_count[1]; best_min_x[2]<=best_min_x[1]; best_max_x[2]<=best_max_x[1]; best_min_y[2]<=best_min_y[1]; best_max_y[2]<=best_max_y[1];
                                best_count[1]<=best_count[0]; best_min_x[1]<=best_min_x[0]; best_max_x[1]<=best_max_x[0]; best_min_y[1]<=best_min_y[0]; best_max_y[1]<=best_max_y[0];
                                best_count[0]<=comp_count[gc_index]; best_min_x[0]<=comp_min_x[gc_index]; best_max_x[0]<=comp_max_x[gc_index]; best_min_y[0]<=comp_min_y[gc_index]; best_max_y[0]<=comp_max_y[gc_index];
                            end else if(comp_count[gc_index]>best_count[1]) begin
                                best_count[3]<=best_count[2]; best_min_x[3]<=best_min_x[2]; best_max_x[3]<=best_max_x[2]; best_min_y[3]<=best_min_y[2]; best_max_y[3]<=best_max_y[2];
                                best_count[2]<=best_count[1]; best_min_x[2]<=best_min_x[1]; best_max_x[2]<=best_max_x[1]; best_min_y[2]<=best_min_y[1]; best_max_y[2]<=best_max_y[1];
                                best_count[1]<=comp_count[gc_index]; best_min_x[1]<=comp_min_x[gc_index]; best_max_x[1]<=comp_max_x[gc_index]; best_min_y[1]<=comp_min_y[gc_index]; best_max_y[1]<=comp_max_y[gc_index];
                            end else if(comp_count[gc_index]>best_count[2]) begin
                                best_count[3]<=best_count[2]; best_min_x[3]<=best_min_x[2]; best_max_x[3]<=best_max_x[2]; best_min_y[3]<=best_min_y[2]; best_max_y[3]<=best_max_y[2];
                                best_count[2]<=comp_count[gc_index]; best_min_x[2]<=comp_min_x[gc_index]; best_max_x[2]<=comp_max_x[gc_index]; best_min_y[2]<=comp_min_y[gc_index]; best_max_y[2]<=comp_max_y[gc_index];
                            end else if(comp_count[gc_index]>best_count[3]) begin
                                best_count[3]<=comp_count[gc_index]; best_min_x[3]<=comp_min_x[gc_index]; best_max_x[3]<=comp_max_x[gc_index]; best_min_y[3]<=comp_min_y[gc_index]; best_max_y[3]<=comp_max_y[gc_index];
                            end
                        end
                    end
                end
            end

            out_vsync<=d_vsync; out_hsync<=d_hsync; out_de<=d_de;
            clean_rgb<=d_rgb;
            clean_edge<=d_edge;
            if(d_de && video_mode==MODE_TRACK && (draw_boxes || region_union) && any_center_cross) out_rgb<=24'hFFFF00;   // A32: marker crosses stay while effects run
            else if(d_de && video_mode==MODE_TRACK && draw_boxes && any_box_edge) out_rgb<=24'hFF3030;
            else out_rgb<=d_rgb;
        end
    end
endmodule
