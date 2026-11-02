`timescale 1ns / 1ps

// A20: effects applied inside a rectangular region of the TRACK-mode video.
//
// The region is supplied by the tracker: either the largest target box or
// the union of all detected boxes (up to four same-colour markers, e.g. tape
// on the fingertips framing a "viewfinder").  Inside the region one of the
// following is applied; outside, the video passes through.
//
//   fx_mode 0: off (tracker draws its usual boxes)
//   fx_mode 1: outline only
//   fx_mode 2: mosaic (pixelate, block size 8/16/32 selected by mosaic_sel)
//   fx_mode 3: cartoon (colour posterised to 4 levels per channel, Sobel
//              edges drawn black)
//   fx_mode 4: sticker (A35): a 320x180, 256-colour picture stored on chip is
//              shown 4x upscaled behind the region -- the region is a window onto it
//   fx_mode 5: blue negative (A24: luma inverted, mapped to a blue palette:
//              deep blue shadows, teal midtones, white where the picture was dark)
//
// Three-clock pipeline (A35: index ROM read, palette read, output), so the
// video timing is delayed by the same three clocks.
module contest_region_fx #(
    parameter integer IMAGE_WIDTH = 1280
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_vsync,
    input  wire        in_hsync,
    input  wire        in_de,
    input  wire [23:0] in_rgb,          // tracker output (boxes suppressed by the tracker when fx is on)
    input  wire [23:0] clean_rgb,       // same timing, no overlays
    input  wire        in_edge,         // Sobel edge flag, same timing
    input  wire        enable,          // TRACK mode active
    input  wire [2:0]  fx_mode,
    input  wire [1:0]  mosaic_sel,      // 0: 8 px, 1: 16 px, 2: 32 px
    input  wire        region_valid,
    input  wire [11:0] region_x0,
    input  wire [11:0] region_y0,
    input  wire [11:0] region_x1,       // inclusive
    input  wire [11:0] region_y1,
    // A33: quadrilateral corners TL, TR, BR, BL (used instead of the rectangle when region_quad)
    input  wire        region_quad,
    input  wire [11:0] region_qx0, input wire [11:0] region_qy0,
    input  wire [11:0] region_qx1, input wire [11:0] region_qy1,
    input  wire [11:0] region_qx2, input wire [11:0] region_qy2,
    input  wire [11:0] region_qx3, input wire [11:0] region_qy3,
    output reg         out_vsync,
    output reg         out_hsync,
    output reg         out_de,
    output reg  [23:0] out_rgb,
    output reg  [23:0] out_clean       // clean_rgb delayed to out_rgb timing (for the PiP capture)
);
    // ---------------------------------------------------------------- timing
    reg [11:0] x, y;
    reg        de_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x <= 12'd0; y <= 12'd0; de_d <= 1'b0;
        end else begin
            de_d <= in_de;
            if (in_vsync) begin
                x <= 12'd0; y <= 12'd0;
            end else if (in_de) begin
                x <= x + 12'd1;
            end else begin
                x <= 12'd0;
                if(de_d) y <= y + 12'd1;
            end
        end
    end

    // Region and mode are latched at frame start so a frame never mixes.
    reg        f_valid, f_active;
    reg [2:0]  f_mode;
    reg [1:0]  f_msel;
    reg [11:0] f_x0, f_y0, f_x1, f_y1;
    reg        vsync_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            f_valid <= 1'b0; f_active <= 1'b0; f_mode <= 3'd0; f_msel <= 2'd1;
            f_x0 <= 12'd0; f_y0 <= 12'd0; f_x1 <= 12'd0; f_y1 <= 12'd0; vsync_d <= 1'b0;
        end else begin
            vsync_d <= in_vsync;
            if (in_vsync) begin
                f_valid  <= region_valid;
                f_active <= enable && (fx_mode != 3'd0);
                f_mode   <= fx_mode;
                f_msel   <= mosaic_sel;
                f_x0 <= region_x0; f_y0 <= region_y0; f_x1 <= region_x1; f_y1 <= region_y1;
            end
        end
    end

    wire quad_in,quad_active;
    video_quad_mask quad_mask(.clk(clk),.rst_n(rst_n),.vs(in_vsync),.de(in_de),
        .enable(region_quad),.points({region_qy3,region_qx3,region_qy2,region_qx2,region_qy1,region_qx1,region_qy0,region_qx0}),
        .active(quad_active),.hit(quad_in));
    wire rect_in  = (x >= f_x0) && (x <= f_x1) && (y >= f_y0) && (y <= f_y1);
    wire inside   = in_de && f_active && f_valid && (quad_active ? quad_in : rect_in);

    // Outline (A34): a pixel is on the boundary when its inside/outside state
    // differs from the pixel two columns to the left or from the pixel in the
    // row above (one 1-bit line buffer).  Works for the rectangular region
    // alike and draws a ~2 px line on both sides of the boundary.
    (* ram_style = "bram" *) reg ins_lb [0:2047];
    reg        ins_up, ins_l1, ins_l2;
    reg [11:0] x_w;
    reg        de_w, ins_w;
    always @(posedge clk) begin
        if (in_de) begin
            ins_up <= ins_lb[x];
            ins_l1 <= inside; ins_l2 <= ins_l1;
        end
        x_w <= x; de_w <= in_de; ins_w <= inside;
        if (de_w) ins_lb[x_w] <= ins_w;
    end
    wire outline = in_de && f_active && f_valid && ((inside ^ ins_up) || (inside ^ ins_l2));

    // ---------------------------------------------------------------- mosaic
    // One block-RAM row of block colours: the first pixel of every block row
    // is captured, then replicated across the block.  Blocks align to the
    // global grid so the pattern is stable while the region moves.
    wire [3:0] bshift = (f_msel == 2'd0) ? 4'd3 : (f_msel == 2'd2) ? 4'd5 : 4'd4;
    wire [11:0] bmask = (12'd1 << bshift) - 12'd1;
    wire        blk_first = ((x & bmask) == 12'd0) && ((y & bmask) == 12'd0);
    wire [7:0]  blk_col   = x[11:3] >> (bshift - 4'd3);          // block index at the finest (8 px) grid scale
    (* ram_style = "bram" *) reg [15:0] blk_buf [0:159];
    reg  [15:0] blk_q;
    wire [15:0] clean565 = {clean_rgb[23:19], clean_rgb[15:10], clean_rgb[7:3]};
    always @(posedge clk) begin
        if (inside && blk_first) blk_buf[blk_col] <= clean565;
        blk_q <= blk_buf[blk_col];
    end

    // --------------------------------------------------------------- stage 1
    reg        s1_vsync, s1_hsync, s1_de, s1_inside, s1_outline, s1_first, s1_edge;
    reg [2:0]  s1_mode;
    reg [23:0] s1_rgb, s1_clean;
    reg [15:0] s1_own565;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_vsync <= 1'b0; s1_hsync <= 1'b0; s1_de <= 1'b0; s1_inside <= 1'b0; s1_outline <= 1'b0;
            s1_first <= 1'b0; s1_edge <= 1'b0; s1_mode <= 3'd0; s1_rgb <= 24'd0; s1_clean <= 24'd0; s1_own565 <= 16'd0;
        end else begin
            s1_vsync <= in_vsync; s1_hsync <= in_hsync; s1_de <= in_de;
            s1_inside <= inside; s1_outline <= outline; s1_first <= blk_first;
            s1_edge <= in_edge; s1_mode <= f_mode;
            s1_rgb <= in_rgb; s1_clean <= clean_rgb; s1_own565 <= clean565;
        end
    end

    // ------------------------------------------------------------ sticker ROM
    // 320x180 indices (4x4 screen pixels per image pixel) + 256-entry palette,
    // both initialised from hex files generated by tools/make_sticker.py.
    (* ram_style = "bram" *) reg [7:0]  stk_mem [0:57599];
    (* ram_style = "bram" *) reg [23:0] stk_pal [0:255];
    initial $readmemh("sticker_idx.hex", stk_mem);
    initial $readmemh("sticker_pal.hex", stk_pal);
    // Block RAM needs a registered read address and a fabric register on its
    // output.  Chain: address register -> index RAM -> index register ->
    // palette RAM -> colour register, four clocks, so the address is formed
    // for the pixel two columns ahead of the current input. x now describes
    // the input pixel exactly (including the first pixel of each line).
    wire [11:0] pxn      = in_de ? (x + 12'd2) : 12'd0;
    wire [7:0]  stk_row  = y[9:2];                      // 0..179
    wire [8:0]  stk_col  = pxn[10:2];                   // 0..320
    reg  [15:0] stk_addr_r;
    reg  [7:0]  stk_idx, stk_idx_r;
    reg  [23:0] stk_rgb;
    always @(posedge clk) begin
        stk_addr_r <= {stk_row, 8'b0} + {2'b0, stk_row, 6'b0} + {7'b0, stk_col};   // row*320 + col
        stk_idx    <= stk_mem[stk_addr_r];
        stk_idx_r  <= stk_idx;
        stk_rgb    <= stk_pal[stk_idx_r];               // valid with stage 2
    end

    // --------------------------------------------------------------- stage 2
    reg        s2_vsync, s2_hsync, s2_de, s2_inside, s2_outline, s2_first, s2_edge;
    reg [2:0]  s2_mode;
    reg [23:0] s2_rgb, s2_clean;
    reg [15:0] s2_own565, blk_q2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2_vsync <= 1'b0; s2_hsync <= 1'b0; s2_de <= 1'b0; s2_inside <= 1'b0; s2_outline <= 1'b0;
            s2_first <= 1'b0; s2_edge <= 1'b0; s2_mode <= 3'd0; s2_rgb <= 24'd0; s2_clean <= 24'd0; s2_own565 <= 16'd0; blk_q2 <= 16'd0;
        end else begin
            s2_vsync <= s1_vsync; s2_hsync <= s1_hsync; s2_de <= s1_de;
            s2_inside <= s1_inside; s2_outline <= s1_outline; s2_first <= s1_first;
            s2_edge <= s1_edge; s2_mode <= s1_mode;
            s2_rgb <= s1_rgb; s2_clean <= s1_clean; s2_own565 <= s1_own565; blk_q2 <= blk_q;
        end
    end

    // --------------------------------------------------------------- stage 3
    wire [15:0] mosaic565 = s2_first ? s2_own565 : blk_q2;  // the capturing pixel shows itself
    wire [23:0] mosaic_rgb = {mosaic565[15:11], mosaic565[15:13], mosaic565[10:5], mosaic565[10:9], mosaic565[4:0], mosaic565[4:2]};
    wire [23:0] poster_rgb = {s2_clean[23:22], 6'b100000, s2_clean[15:14], 6'b100000, s2_clean[7:6], 6'b100000};
    // A36 blue negative with hard contrast: n = 255 - luma, k = sat(3*(n-100)),
    //   B = 64 + 7k/8 (sat), G = k, R = 2(k-128) for k >= 128 else 0
    // bright picture (lit face) -> deep navy, midtones -> blue, dark picture
    // (hair, shadows) -> white; a steep knee so faces read dark and hair white.
    wire [9:0]  neg_sum = {2'b0, s2_clean[23:16]} + {1'b0, s2_clean[15:8], 1'b0} + {2'b0, s2_clean[7:0]};
    wire [7:0]  neg_n   = ~neg_sum[9:2];
    wire [7:0]  neg_d   = (neg_n > 8'd100) ? (neg_n - 8'd100) : 8'd0;
    wire [9:0]  neg_k3  = {neg_d, 1'b0} + {2'b0, neg_d};
    wire [7:0]  neg_k   = (|neg_k3[9:8]) ? 8'hff : neg_k3[7:0];
    wire [8:0]  neg_bs  = 9'd64 + {1'b0, neg_k} - {4'b0, neg_k[7:3]};
    wire [7:0]  neg_b   = neg_bs[8] ? 8'hff : neg_bs[7:0];
    wire [7:0]  neg_g   = neg_k;
    wire [7:0]  neg_r   = neg_k[7] ? {neg_k[6:0], 1'b0} : 8'd0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_vsync <= 1'b0; out_hsync <= 1'b0; out_de <= 1'b0; out_rgb <= 24'd0; out_clean <= 24'd0;
        end else begin
            out_vsync <= s2_vsync; out_hsync <= s2_hsync; out_de <= s2_de;
            out_clean <= s2_clean;
            if (!s2_de)
                out_rgb <= 24'd0;
            else if (s2_outline)
                out_rgb <= 24'hFFFFFF;
            else if (!s2_inside)
                out_rgb <= s2_rgb;
            else begin
                case (s2_mode)
                    3'd2:    out_rgb <= mosaic_rgb;
                    3'd3:    out_rgb <= s2_edge ? 24'h000000 : poster_rgb;
                    3'd4:    out_rgb <= stk_rgb;          // A35: sticker picture behind the window
                    3'd5:    out_rgb <= {neg_r, neg_g, neg_b};
                    default: out_rgb <= s2_clean;      // 1: outline only
                endcase
            end
        end
    end
endmodule

// Convex quadrilateral raster mask. Coefficients are computed once per frame
// with ONE multiplier; raster scanning uses additions only. Coordinates are
// captured at VS falling edge, after the tracker's blanking-time update.
module video_quad_mask(
 input wire clk,rst_n,vs,de,enable,input wire [95:0] points,
 output reg active,output wire hit);
 reg vd,dd,ready;
 reg [95:0] saved;
 reg [2:0] state;reg [1:0] edge_index;
 wire [1:0] next_edge=edge_index+2'd1;
 wire [23:0] p0=saved[edge_index*24+:24],p1=saved[next_edge*24+:24];
 reg signed [12:0] a[0:3],b[0:3];
 reg signed [23:0] row_value[0:3],pixel_value[0:3];
 reg signed [25:0] first_product;
 wire signed [12:0] mx=state==3 ? $signed({1'b0,p1[11:0]}) : $signed({1'b0,p0[11:0]});
 wire signed [12:0] my=state==3 ? $signed({1'b0,p0[23:12]}) : $signed({1'b0,p1[23:12]});
 wire signed [25:0] product=mx*my;
 wire signed [23:0] c0=dd?pixel_value[0]:row_value[0],c1=dd?pixel_value[1]:row_value[1];
 wire signed [23:0] c2=dd?pixel_value[2]:row_value[2],c3=dd?pixel_value[3]:row_value[3];
 wire positive=c0>=0&&c1>=0&&c2>=0&&c3>=0;
 wire negative=c0<=0&&c1<=0&&c2<=0&&c3<=0;
 assign hit=active&&ready&&de&&(positive||negative);
 integer i;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   vd<=0;dd<=0;ready<=0;active<=0;saved<=0;state<=0;edge_index<=0;first_product<=0;
   for(i=0;i<4;i=i+1)begin a[i]<=0;b[i]<=0;row_value[i]<=0;pixel_value[i]<=0;end
  end else begin
   vd<=vs;dd<=de;
   if(vs&&!vd)ready<=0;
   if(vd&&!vs)begin
    active<=enable;saved<=points;state<=1;edge_index<=0;ready<=0;
   end else case(state)
    1:begin
     a[edge_index]<=$signed({1'b0,p1[11:0]})-$signed({1'b0,p0[11:0]});
     b[edge_index]<=$signed({1'b0,p0[23:12]})-$signed({1'b0,p1[23:12]});
     state<=2;
    end
    2:begin first_product<=product;state<=3;end
    3:begin row_value[edge_index]<=first_product-product;state<=4;end
    4:if(edge_index==3)begin ready<=1;state<=0;end
      else begin edge_index<=edge_index+1'b1;state<=1;end
   endcase
   if(ready&&!vs)for(i=0;i<4;i=i+1)begin
    if(de)pixel_value[i]<=(dd?pixel_value[i]:row_value[i])+b[i];
    else if(dd)row_value[i]<=row_value[i]+a[i];
   end
  end
 end
endmodule
