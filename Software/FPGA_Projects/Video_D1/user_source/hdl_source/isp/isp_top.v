module isp_top (
  input wire [7:0] I_algorithm_flags,
    input 			axi4s_video_aclk,
    input 			I_rst_n			,
    // Picture style, applied only at frame start (see pic_mode below):
    //   0 = V52 look (five-segment WDR curve, fixed WB trims, 1.375x chroma)
    //   1 = natural (gentle gamma, no fixed trims, 1.125x chroma)  [default]
    //   2 = natural curve with highlight knee (saturated lights -> 232), 1.125x chroma
    //   3 = linear after AWB, unity chroma (diagnostic)
    input [1:0]		I_picture_mode	,
    input 			I_tlast			,
    input 			I_tuser			,
    input [39:0] 	I_tdata			,
    input 			I_tvalid		,
    input [9:0] 	I_tdest			,
    input 			O_tready		,
    output [127:0] 	O_tdata			,
    output 			O_tlast			,
    output 			O_tuser			,
    output 			O_tvalid		,
    output 			I_tready
);
 
wire         m_aixs_tvalid;  //synthesis keep 
  wire [127:0] m_aixs_tdata;  //synthesis keep 
  wire         m_aixs_tuser;  //synthesis keep 
  wire         m_aixs_tlast;  //synthesis keep 
  wire         m_aixs_tready;  //synthesis keep 
  
  wire [ 95:0] m_aixs_tdata_96;  //synthesis keep 
  wire [ 95:0] scaled_tdata_96;
  wire          scaled_tvalid;
  wire          scaled_tuser;
  wire          scaled_tlast;
  reg  [ 95:0] wb_tdata_96;
  reg  [ 95:0] color_tdata_96;
  reg  [ 79:0] chroma_stats_tdata_80;
  reg  [ 79:0] vertical_chroma_tdata_80;
  reg  [ 79:0] luma_stats_tdata_80;
  wire [ 79:0] chroma_current_stats_80;
  wire [ 79:0] chroma_previous_stats_80;
  reg  [  8:0] chroma_word_addr;
  reg  [  9:0] chroma_row_index;
  reg           chroma_first_line_d;
  reg  [ 95:0] denoise_tdata_96;
  reg  [ 95:0] vivid_tdata_96;
  reg           wb_tvalid;
  reg           wb_tuser;
  reg           color_tvalid;
  reg           color_tuser;
  reg           chroma_stats_tvalid;
  reg           chroma_stats_tuser;
  reg           vertical_chroma_tvalid;
  reg           vertical_chroma_tuser;
  reg           luma_stats_tvalid;
  reg           luma_stats_tuser;
  reg           denoise_tvalid;
  reg           denoise_tuser;
  reg           vivid_tvalid;
  reg           vivid_tuser;

  // Frame-to-frame gray-world white balance.  Sums from one complete frame
  // produce gains for the following frame, so coefficients never change in
  // the middle of a picture.
  reg  [31:0] wb_sum_r;
  reg  [31:0] wb_sum_g;
  reg  [31:0] wb_sum_b;
  reg  [19:0] wb_gain_r;
  reg  [19:0] wb_gain_b;
  reg  [19:0] wb_pending_r;
  reg  [19:0] wb_pending_b;

  // Picture-mode control crosses from the 100 MHz UART domain.  It is a
  // static two-bit value, so a two-stage sampler is enough; the result is
  // then latched at frame start so one frame never mixes two styles.
  reg  [1:0]  pic_mode_meta;
  reg  [1:0]  pic_mode_sync;
  reg [7:0] algorithm_flags;
  reg [95:0] bypass0,bypass1,bypass2;
  always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
      if(!I_rst_n) begin algorithm_flags<=8'h2f;bypass0<=0;bypass1<=0;bypass2<=0;end
      else begin
          if(scaled_tuser) algorithm_flags<=I_algorithm_flags;
          bypass0<=color_tdata_96;bypass1<=bypass0;bypass2<=bypass1;
      end
  end
  reg  [1:0]  pic_mode;
  always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
      if (!I_rst_n) begin
          pic_mode_meta <= 2'd2;
          pic_mode_sync <= 2'd2;
          pic_mode      <= 2'd2;
      end else begin
          pic_mode_meta <= I_picture_mode;
          pic_mode_sync <= pic_mode_meta;
          if (scaled_tuser) pic_mode <= pic_mode_sync;
      end
  end

  // Source pixels for statistics and white balance.  Channel order here is
  // the demosaic's R,G,B; the DDR word-alignment colour rotation seen on the
  // display is corrected after DDR readout in design_top_wrapper.
  wire [95:0] src_tdata_96 = scaled_tdata_96;

  // A15: gray-world sums use RAW (pre-gain) values so the divider result is
  // an absolute gain, but the neutral-pixel *selection* must look at the
  // pixel after the current white-balance gains.  Raw sensor white has
  // G >> R,B and never passes a chroma test (A13 turned the picture red for
  // exactly that reason).  src_tdata_d is the raw word delayed one stage so
  // it lines up with wb_tdata_96, the same word after the gains.
  reg  [95:0] src_tdata_d;
  // A19: the selection flags are registered one stage before the masked
  // sums (stage A: flags from wb_tdata_96 / src_tdata_d; stage B: sums from
  // src_tdata_d2 masked by the registered flags).  Doing both in one cycle
  // was 13 logic levels and missed the 8.888 ns MIPI byte clock by 0.3 ns.
  reg  [95:0] src_tdata_d2;
  reg  [3:0]  stat_use_q, all_use_q;
  wire [7:0] stat_r0 = src_tdata_d2[95:88];
  wire [7:0] stat_g0 = src_tdata_d2[87:80];
  wire [7:0] stat_b0 = src_tdata_d2[79:72];
  wire [7:0] stat_r1 = src_tdata_d2[71:64];
  wire [7:0] stat_g1 = src_tdata_d2[63:56];
  wire [7:0] stat_b1 = src_tdata_d2[55:48];
  wire [7:0] stat_r2 = src_tdata_d2[47:40];
  wire [7:0] stat_g2 = src_tdata_d2[39:32];
  wire [7:0] stat_b2 = src_tdata_d2[31:24];
  wire [7:0] stat_r3 = src_tdata_d2[23:16];
  wire [7:0] stat_g3 = src_tdata_d2[15:8];
  wire [7:0] stat_b3 = src_tdata_d2[7:0];
  // whole-frame candidate test on the stage-A raw word (src_tdata_d)
  wire [9:0] stat_rgb0 = src_tdata_d[95:88] + src_tdata_d[87:80] + src_tdata_d[79:72];
  wire [9:0] stat_rgb1 = src_tdata_d[71:64] + src_tdata_d[63:56] + src_tdata_d[55:48];
  wire [9:0] stat_rgb2 = src_tdata_d[47:40] + src_tdata_d[39:32] + src_tdata_d[31:24];
  wire [9:0] stat_rgb3 = src_tdata_d[23:16] + src_tdata_d[15:8]  + src_tdata_d[7:0];
  // A12: gray-world statistics use only near-neutral pixels.  Plain
  // gray-world over the whole frame lets a dominant coloured surface (wood
  // wall, coloured cloth) drag the gains: measured white came out
  // R180 G201 B193 in a wood-dominated scene.  A pixel counts when its
  // chroma (max-min) is below 1/6 of its brightness and it is not too dark
  // or clipped.  If too few such pixels exist in a frame the gains hold.
  function neutral_pixel;
      input [7:0] r, g, b;
      reg [7:0]  mx, mn;
      reg [10:0] c6;
      begin
          mx = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
          mn = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);
          c6 = ({3'b0, mx - mn} << 2) + ({3'b0, mx - mn} << 1);   // 6 * chroma, 11 bits
          // A17: only BRIGHT low-chroma pixels count as neutral (white-patch
          // prior).  With the A16 threshold of 48 a wood-panel wall (V ~120,
          // low chroma after gains) was accepted as "gray" and pulled the true
          // white surface to R188 G217 B217.  White objects are normally the
          // brightest neutral surfaces in view; when none exist the
          // whole-frame fallback below takes over.
          neutral_pixel = (mx >= 8'd160) && (mx <= 8'd240) && (c6 <= {3'b0, mx});
      end
  endfunction
  // Neutrality judged on the white-balanced pixel (closed loop: once white
  // renders white it keeps qualifying and the gains settle).
  wire neut0 = neutral_pixel(wb_tdata_96[95:88], wb_tdata_96[87:80], wb_tdata_96[79:72]);
  wire neut1 = neutral_pixel(wb_tdata_96[71:64], wb_tdata_96[63:56], wb_tdata_96[55:48]);
  wire neut2 = neutral_pixel(wb_tdata_96[47:40], wb_tdata_96[39:32], wb_tdata_96[31:24]);
  wire neut3 = neutral_pixel(wb_tdata_96[23:16], wb_tdata_96[15:8],  wb_tdata_96[7:0]);
  wire stat_use0 = stat_use_q[0];
  wire stat_use1 = stat_use_q[1];
  wire stat_use2 = stat_use_q[2];
  wire stat_use3 = stat_use_q[3];
  wire [2:0] stat_add_n = {2'b0, stat_use0} + {2'b0, stat_use1} + {2'b0, stat_use2} + {2'b0, stat_use3};
  reg  [2:0]  stat_add_n_d;
  reg  [19:0] wb_count;          // neutral pixels in the frame being accumulated
  reg         wb_enough;         // previous frame had enough neutral pixels
  localparam [19:0] WB_MIN_NEUTRAL = 20'd4096;
  wire [9:0] stat_add_r = (stat_use0 ? stat_r0 : 8'd0) +
                           (stat_use1 ? stat_r1 : 8'd0) +
                           (stat_use2 ? stat_r2 : 8'd0) +
                           (stat_use3 ? stat_r3 : 8'd0);
  wire [9:0] stat_add_g = (stat_use0 ? stat_g0 : 8'd0) +
                           (stat_use1 ? stat_g1 : 8'd0) +
                           (stat_use2 ? stat_g2 : 8'd0) +
                           (stat_use3 ? stat_g3 : 8'd0);
  wire [9:0] stat_add_b = (stat_use0 ? stat_b0 : 8'd0) +
                           (stat_use1 ? stat_b1 : 8'd0) +
                           (stat_use2 ? stat_b2 : 8'd0) +
                           (stat_use3 ? stat_b3 : 8'd0);
  reg  [9:0] stat_add_r_d;
  reg  [9:0] stat_add_g_d;
  reg  [9:0] stat_add_b_d;
  reg        stat_valid_d;

  // A16: whole-frame gray-world sums as a bootstrap.  The neutral-pixel
  // selection is judged after the current gains, so from a poor starting
  // point (or in a scene without neutral surfaces) it may never see enough
  // candidates; A15 stayed at its reset gains and rendered everything red.
  // When a frame yields fewer than WB_MIN_NEUTRAL neutral pixels the
  // dividers use the plain gray-world sums instead (the pre-A12 behaviour).
  wire allc0 = (stat_rgb0 > 10'd24) && (stat_rgb0 < 10'd720);   // stage A, registered into all_use_q
  wire allc1 = (stat_rgb1 > 10'd24) && (stat_rgb1 < 10'd720);
  wire allc2 = (stat_rgb2 > 10'd24) && (stat_rgb2 < 10'd720);
  wire allc3 = (stat_rgb3 > 10'd24) && (stat_rgb3 < 10'd720);
  wire all_use0 = all_use_q[0];
  wire all_use1 = all_use_q[1];
  wire all_use2 = all_use_q[2];
  wire all_use3 = all_use_q[3];
  wire [9:0] all_add_r = (all_use0 ? stat_r0 : 8'd0) + (all_use1 ? stat_r1 : 8'd0) +
                         (all_use2 ? stat_r2 : 8'd0) + (all_use3 ? stat_r3 : 8'd0);
  wire [9:0] all_add_g = (all_use0 ? stat_g0 : 8'd0) + (all_use1 ? stat_g1 : 8'd0) +
                         (all_use2 ? stat_g2 : 8'd0) + (all_use3 ? stat_g3 : 8'd0);
  wire [9:0] all_add_b = (all_use0 ? stat_b0 : 8'd0) + (all_use1 ? stat_b1 : 8'd0) +
                         (all_use2 ? stat_b2 : 8'd0) + (all_use3 ? stat_b3 : 8'd0);
  reg  [9:0]  all_add_r_d, all_add_g_d, all_add_b_d;
  reg  [31:0] wb_all_r, wb_all_g, wb_all_b;
  wire        wb_enough_now = (wb_count >= WB_MIN_NEUTRAL);   // evaluated at frame start, before the reset
  wire [31:0] div_sum_r = wb_enough_now ? wb_sum_r : wb_all_r;
  wire [31:0] div_sum_g = wb_enough_now ? wb_sum_g : wb_all_g;
  wire [31:0] div_sum_b = wb_enough_now ? wb_sum_b : wb_all_b;

  wire [47:0] wb_ratio_r;
  wire [47:0] wb_ratio_b;
  wire wb_done_r;
  wire wb_done_b;

  // Clamp the raw gray-world result, then approach it gradually.  Applying
  // the full per-frame ratio made the displayed colour alternate whenever a
  // bright window or coloured object moved through the metering population.
  wire [19:0] wb_target_r = (wb_ratio_r < 48'h0000c000) ? 20'h0c000 :
                            (wb_ratio_r > 48'h00030000) ? 20'h30000 :
                            wb_ratio_r[19:0];
  wire [19:0] wb_target_b = (wb_ratio_b < 48'h0000c000) ? 20'h0c000 :
                            (wb_ratio_b > 48'h00030000) ? 20'h30000 :
                            wb_ratio_b[19:0];

  video_awb_div wb_div_shared (
      .clk(axi4s_video_aclk), .rst_n(I_rst_n), .start(scaled_tuser),
      .sum_r(div_sum_r), .sum_g(div_sum_g), .sum_b(div_sum_b),
      .ratio_r(wb_ratio_r), .ratio_b(wb_ratio_b),
      .done_r(wb_done_r), .done_b(wb_done_b)
  );

  always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
      if (!I_rst_n) begin
          wb_sum_r     <= 32'd1;
          wb_sum_g     <= 32'd1;
          wb_sum_b     <= 32'd1;
          wb_gain_r    <= 20'h20000;
          wb_gain_b    <= 20'h20000;
          wb_pending_r <= 20'h20000;
          wb_pending_b <= 20'h20000;
          stat_add_r_d <= 10'd0;
          stat_add_g_d <= 10'd0;
          stat_add_b_d <= 10'd0;
          stat_add_n_d <= 3'd0;
          stat_valid_d <= 1'b0;
          wb_count     <= 20'd0;
          wb_enough    <= 1'b0;
          all_add_r_d  <= 10'd0;
          all_add_g_d  <= 10'd0;
          all_add_b_d  <= 10'd0;
          wb_all_r     <= 32'd1;
          wb_all_g     <= 32'd1;
          wb_all_b     <= 32'd1;
      end else begin
          all_add_r_d  <= all_add_r;
          all_add_g_d  <= all_add_g;
          all_add_b_d  <= all_add_b;
          stat_add_r_d <= stat_add_r;
          stat_add_g_d <= stat_add_g;
          stat_add_b_d <= stat_add_b;
          stat_add_n_d <= stat_add_n;
          // stage A registers (flags + raw copy), stage B is stat_add_*_d above
          stat_use_q   <= {neut3, neut2, neut1, neut0};
          all_use_q    <= {allc3, allc2, allc1, allc0};
          src_tdata_d2 <= src_tdata_d;
          stat_valid_d <= color_tvalid;   // A19: two stages after scaled_tvalid
          if (scaled_tuser) begin
              wb_sum_r  <= 32'd1;
              wb_sum_g  <= 32'd1;
              wb_sum_b  <= 32'd1;
              wb_all_r  <= 32'd1;
              wb_all_g  <= 32'd1;
              wb_all_b  <= 32'd1;
              wb_enough <= wb_enough_now;     // status only; the dividers already picked their sums
              wb_count  <= 20'd0;
              wb_gain_r <= wb_pending_r;
              wb_gain_b <= wb_pending_b;
              stat_valid_d <= 1'b0;
          end else if (stat_valid_d) begin
              wb_sum_r <= wb_sum_r + stat_add_r_d;
              wb_sum_g <= wb_sum_g + stat_add_g_d;
              wb_sum_b <= wb_sum_b + stat_add_b_d;
              wb_all_r <= wb_all_r + all_add_r_d;
              wb_all_g <= wb_all_g + all_add_g_d;
              wb_all_b <= wb_all_b + all_add_b_d;
              wb_count <= wb_count + stat_add_n_d;
          end
          // The dividers start at frame start on either the neutral-pixel
          // sums or the whole-frame sums of the frame just completed.
          if (wb_done_r) begin
              // 1/16 IIR slew plus a 1/64-gain deadband.
              if (wb_target_r > wb_pending_r + 20'h00400)
                  wb_pending_r <= wb_pending_r + ((wb_target_r - wb_pending_r) >> 4);
              else if (wb_pending_r > wb_target_r + 20'h00400)
                  wb_pending_r <= wb_pending_r - ((wb_pending_r - wb_target_r) >> 4);
          end
          if (wb_done_b) begin
              if (wb_target_b > wb_pending_b + 20'h00400)
                  wb_pending_b <= wb_pending_b + ((wb_target_b - wb_pending_b) >> 4);
              else if (wb_pending_b > wb_target_b + 20'h00400)
                  wb_pending_b <= wb_pending_b - ((wb_pending_b - wb_target_b) >> 4);
          end
      end
  end

  // The sensor delivers linear RAW data.  The vendor AWB block used by the
  // original design changes the stream timing and is therefore bypassed in
  // this frame-safe path.  Apply a small fixed daylight white balance and a
  // continuous piecewise gamma curve here.  This is purely combinational, so
  // every valid group entering this block still produces exactly one valid
  // group at the packer.
  function [7:0] gain_sat;
      input [7:0] x;
      input [19:0] gain;
      reg [27:0] y;
      begin
          y = x * gain;
          gain_sat = (|y[27:24]) ? 8'hff : y[23:16];
      end
  endfunction

  function [7:0] gamma_lift;
      input [7:0] x;
      reg [8:0] y;
      reg [8:0] d;
      begin
          // Low-light display curve.  The sensor has little signal below 32
          // in a dim room, so lift that range strongly while retaining a true
          // black point and compressing highlights.
          if (x < 8'd4) begin
              y = 9'd0;
          end else if (x < 8'd16) begin
              y = {1'b0,x} + ({1'b0,x} << 1);
          end else if (x < 8'd64) begin
              d = {1'b0,x} - 9'd16;
              y = 9'd48 + d + (d >> 1);
          end else if (x < 8'd128) begin
              d = {1'b0,x} - 9'd64;
              y = 9'd120 + d;
          end else if (x < 8'd192) begin
              d = {1'b0,x} - 9'd128;
              y = 9'd184 + (d >> 1);
          end else begin
              d = {1'b0,x} - 9'd192;
              y = 9'd216 + (d >> 1);
          end
          gamma_lift = (y > 9'd255) ? 8'hff : y[7:0];
      end
  endfunction

  function [7:0] sat_signed;
      input integer v;
      begin
          if (v < 0) sat_signed = 8'd0;
          else if (v > 255) sat_signed = 8'hff;
          else sat_signed = v[7:0];
      end
  endfunction

  function [39:0] chroma_pair_stats;
      input [47:0] p;
      integer r0;
      integer g0;
      integer b0;
      integer r1;
      integer g1;
      integer b1;
      integer y0;
      integer y1;
      integer ra;
      integer ga;
      integer ba;
      begin
          r0 = p[47:40];
          g0 = p[39:32];
          b0 = p[31:24];
          r1 = p[23:16];
          g1 = p[15:8];
          b1 = p[7:0];
          y0 = (r0 + (g0 << 1) + b0) >> 2;
          y1 = (r1 + (g1 << 1) + b1) >> 2;

          ra = (r0+r1) >> 1;
          ga = (g0+g1) >> 1;
          ba = (b0+b1) >> 1;
          chroma_pair_stats = {y0[7:0],y1[7:0],ra[7:0],ga[7:0],ba[7:0]};
      end
  endfunction

  function [47:0] rebuild_chroma_pair;
      input [39:0] s;
      integer y0;
      integer y1;
      integer ra;
      integer ga;
      integer ba;
      integer ya;
      integer dr;
      integer dg;
      integer db;
      begin
          y0 = s[39:32];
          y1 = s[31:24];
          ra = s[23:16];
          ga = s[15:8];
          ba = s[7:0];
          ya = (ra + (ga << 1) + ba) >> 2;
          dr = ra-ya;
          dg = ga-ya;
          db = ba-ya;
          // Horizontal 4:2:2-style chroma filtering.  Both pixels share
          // their averaged colour difference but retain independent luma.
          rebuild_chroma_pair = {
              sat_signed(y0+dr),sat_signed(y0+dg),sat_signed(y0+db),
              sat_signed(y1+dr),sat_signed(y1+dg),sat_signed(y1+db)};
      end
  endfunction

  // Edge-aware four-pixel chroma filter.  Flat areas share one colour
  // difference across the complete group, which suppresses the red/green/
  // blue speckle produced by high sensor gain.  Across a real luminance edge
  // it falls back to the existing two-pixel filter to avoid colour bleeding.
  function [95:0] rebuild_chroma_quad;
      input [79:0] s;
      integer y0;
      integer y1;
      integer y2;
      integer y3;
      integer ra0;
      integer ga0;
      integer ba0;
      integer ra1;
      integer ga1;
      integer ba1;
      integer ym0;
      integer ym1;
      integer delta;
      integer rav;
      integer gav;
      integer bav;
      integer yav;
      integer dr;
      integer dg;
      integer db;
      begin
          y0  = s[79:72];
          y1  = s[71:64];
          ra0 = s[63:56];
          ga0 = s[55:48];
          ba0 = s[47:40];
          y2  = s[39:32];
          y3  = s[31:24];
          ra1 = s[23:16];
          ga1 = s[15:8];
          ba1 = s[7:0];
          ym0 = (y0+y1) >> 1;
          ym1 = (y2+y3) >> 1;
          delta = (ym0 > ym1) ? (ym0-ym1) : (ym1-ym0);
          // This stage follows the low-light gamma lift, where values below
          // 64 and their noise have already been multiplied by three.  Use a
          // post-gamma threshold so dark flat areas are not mistaken for
          // genuine edges and excluded from the four-pixel chroma average.
          if(delta < 64) begin
              rav = (ra0+ra1) >> 1;
              gav = (ga0+ga1) >> 1;
              bav = (ba0+ba1) >> 1;
              yav = (rav + (gav << 1) + bav) >> 2;
              dr = rav-yav;
              dg = gav-yav;
              db = bav-yav;
              rebuild_chroma_quad = {
                  sat_signed(y0+dr),sat_signed(y0+dg),sat_signed(y0+db),
                  sat_signed(y1+dr),sat_signed(y1+dg),sat_signed(y1+db),
                  sat_signed(y2+dr),sat_signed(y2+dg),sat_signed(y2+db),
                  sat_signed(y3+dr),sat_signed(y3+dg),sat_signed(y3+db)};
          end else begin
              rebuild_chroma_quad = {
                  rebuild_chroma_pair(s[79:40]),
                  rebuild_chroma_pair(s[39:0])};
          end
      end
  endfunction

  function [79:0] merge_vertical_chroma;
      input [79:0] cur;
      input [79:0] prv;
      input        first_line;
      integer k;
      integer cy0,cy1,cr,cg,cb;
      integer py0,py1,pr,pg,pb;
      integer cmean,pmean,ydiff;
      integer rd,gd,bd;
      integer vr,vg,vb;
      reg [39:0] out_pair [0:1];
      begin
          for(k=0;k<2;k=k+1) begin
              if(k==0) begin
                  cy0=cur[79:72]; cy1=cur[71:64]; cr=cur[63:56]; cg=cur[55:48]; cb=cur[47:40];
                  py0=prv[79:72]; py1=prv[71:64]; pr=prv[63:56]; pg=prv[55:48]; pb=prv[47:40];
              end else begin
                  cy0=cur[39:32]; cy1=cur[31:24]; cr=cur[23:16]; cg=cur[15:8]; cb=cur[7:0];
                  py0=prv[39:32]; py1=prv[31:24]; pr=prv[23:16]; pg=prv[15:8]; pb=prv[7:0];
              end
              cmean=(cy0+cy1)>>1;
              pmean=(py0+py1)>>1;
              ydiff=(cmean>pmean)?(cmean-pmean):(pmean-cmean);
              rd=(cr>pr)?(cr-pr):(pr-cr);
              gd=(cg>pg)?(cg-pg):(pg-cg);
              bd=(cb>pb)?(cb-pb):(pb-cb);
              // As above, these are post-gamma differences.  V40 used the
              // pre-lift scale (18/40), causing most low-light pixels to skip
              // vertical denoising.  The corrected gates still reject strong
              // object boundaries while accepting amplified sensor noise.
              if(!first_line && ydiff<56 && rd<112 && gd<112 && bd<112) begin
                  vr=(cr+pr+1)>>1;
                  vg=(cg+pg+1)>>1;
                  vb=(cb+pb+1)>>1;
              end else begin
                  vr=cr;
                  vg=cg;
                  vb=cb;
              end
              out_pair[k]={cy0[7:0],cy1[7:0],vr[7:0],vg[7:0],vb[7:0]};
          end
          merge_vertical_chroma={out_pair[0],out_pair[1]};
      end
  endfunction

  // Mild horizontal luma filtering for amplified dark areas.  The chroma
  // statistics already carry the independent luminance of both pixels, so
  // this stage needs no RGB conversion and stays short.  A 3:1 blend cuts
  // random grain while the darkness and local-difference tests keep bright
  // detail and real edges unchanged.
  function [39:0] smooth_dark_luma_pair;
      input [39:0] s;
      integer y0;
      integer y1;
      integer ym;
      integer yd;
      integer sy0;
      integer sy1;
      begin
          y0 = s[39:32];
          y1 = s[31:24];
          ym = (y0+y1) >> 1;
          yd = (y0>y1) ? (y0-y1) : (y1-y0);
          if(ym < 80 && yd < 24) begin
              // Full pair average only in very dark, locally flat areas.
              sy0 = (y0 + y1 + 1) >> 1;
              sy1 = (y0 + y1 + 1) >> 1;
          end else if(ym < 160 && yd < 36) begin
              sy0 = ((y0 << 1) + y0 + y1 + 2) >> 2;
              sy1 = ((y1 << 1) + y1 + y0 + 2) >> 2;
          end else begin
              sy0 = y0;
              sy1 = y1;
          end
          smooth_dark_luma_pair = {sy0[7:0],sy1[7:0],s[23:0]};
      end
  endfunction

  function [23:0] vivid_pixel;
      input [23:0] p;
      input [1:0]  mode;
      integer r;
      integer g;
      integer b;
      integer y;
      integer dr;
      integer dg;
      integer db;
      begin
          r = p[23:16];
          g = p[15:8];
          b = p[7:0];
          y = (r + (g << 1) + b) >> 2;
          dr = r-y;
          dg = g-y;
          db = b-y;
          // A29: 1.375x chroma for every mode value (single picture style).
          vivid_pixel = algorithm_flags[3] ? {sat_signed(y + dr + (dr/4) + (dr/8)),
                         sat_signed(y + dg + (dg/4) + (dg/8)),
                         sat_signed(y + db + (db/4) + (db/8))} : p;
      end
  endfunction

  function [23:0] wb_pixel;
      input [23:0] p;
      input [1:0]  mode;
      integer wr;
      integer wg;
      integer wb;
      begin
          wr = gain_sat(p[23:16],wb_gain_r);
          wg = p[15:8];
          wb = gain_sat(p[7:0],wb_gain_b);
          // A28: V52 fixed WB trims (mode 0) removed; all modes use the AWB gains only.
          wb_pixel = (scaled_tuser ? I_algorithm_flags[0] : algorithm_flags[0]) ?
                     {sat_signed(wr),sat_signed(wg),sat_signed(wb)} : p;
      end
  endfunction

  // Gentle display curve for modes 1 and 2.  Piecewise linear, monotonic:
  // 0->0, 32->68, 64->108, 128->168, 192->216, 255->255 (A10: midtones
  // lifted slightly from the A1 curve 32->64/64->104/128->160/192->208).
  // Compared with the V46 curve (32->112) this lifts shadows about half as
  // much, so AE raises sensor exposure instead and dark-area noise is not
  // amplified digitally.
  function [7:0] natural_channel;
      input integer x;
      integer d;
      integer y;
      begin
          if (x < 32) begin
              y = (x << 1) + (x >> 3);
          end else if (x < 64) begin
              d = x - 32;
              y = 68 + d + (d >> 2);
          end else if (x < 128) begin
              d = x - 64;
              y = 108 + d - (d >> 4);
          end else if (x < 192) begin
              d = x - 128;
              y = 168 + (d >> 1) + (d >> 2);
          end else begin
              d = x - 192;
              y = 216 + (d >> 1) + (d >> 3);
          end
          natural_channel = sat_signed(y);
      end
  endfunction

  // Mode 2: the natural curve with a highlight knee.  Identical below 192,
  // then 192->216 rises only to 255->228 (slope 0.19).  Sensor-saturated
  // light sources therefore render as light grey instead of glaring pure
  // white, while everything below the knee is untouched.  Clipped pixels
  // carry no detail, so this is cosmetic glare reduction, not HDR.
  function [7:0] natural_knee_channel;
      input integer x;
      integer d;
      begin
          if (x < 192) begin
              natural_knee_channel = natural_channel(x);
          end else begin
              d = x - 192;
              natural_knee_channel = sat_signed(216 + (d >> 3) + (d >> 4));
          end
      end
  endfunction

  function [7:0] wdr_channel;
      input integer x;
      integer d;
      integer y;
      begin
          // V46 monotonic five-segment tone curve:
          // 0->0, 32->112, 64->144, 128->196, 192->228, 255->246.
          // Shadows remain visible while the middle and upper ranges keep
          // enough code space to show texture instead of becoming flat white.
          if (x < 32) begin
              y = x + (x << 1) + (x >> 1);
          end else if (x < 64) begin
              y = 112 + (x - 32);
          end else if (x < 128) begin
              d = x - 64;
              y = 144 + d - (d >> 2) + (d >> 4);
          end else if (x < 192) begin
              y = 196 + ((x - 128) >> 1);
          end else begin
              d = x - 192;
              y = 228 + (d >> 2) + (d >> 4);
          end
          wdr_channel = sat_signed(y);
      end
  endfunction

  function [23:0] gamma_pixel;
      input [23:0] p;
      input [1:0]  mode;
      integer r;
      integer g;
      integer b;
      integer ro;
      integer go;
      integer bo;
      begin
          r = p[23:16];
          g = p[15:8];
          b = p[7:0];
          // A29: a single picture style (natural curve + highlight knee) for
          // every mode value; the V52 WDR, plain-natural and linear paths were
          // removed to free logic (user: keep only this look).
          ro = natural_knee_channel(r);
          go = natural_knee_channel(g);
          bo = natural_knee_channel(b);
          gamma_pixel = algorithm_flags[1] ? {ro[7:0],go[7:0],bo[7:0]} : p;
      end
  endfunction

  assign chroma_current_stats_80 = {
      chroma_pair_stats(color_tdata_96[95:48]),
      chroma_pair_stats(color_tdata_96[47:0])};

  chroma80_line_eram u_chroma_previous_line (
      .clk  (axi4s_video_aclk),
      .we   (color_tvalid),
      .addr (color_tuser ? 9'd0 : chroma_word_addr),
      .din  (chroma_current_stats_80),
      .dout (chroma_previous_stats_80)
  );

  // Registered stages keep the 112 MHz camera path short.  Valid and
  // frame-start travel with the same pixels, including the scaler's bubbles.
  always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
      if (!I_rst_n) begin
          wb_tdata_96    <= 96'd0;
          color_tdata_96 <= 96'd0;
          chroma_stats_tdata_80 <= 80'd0;
          vertical_chroma_tdata_80 <= 80'd0;
          luma_stats_tdata_80 <= 80'd0;
          denoise_tdata_96 <= 96'd0;
          vivid_tdata_96 <= 96'd0;
          wb_tvalid      <= 1'b0;
          wb_tuser       <= 1'b0;
          color_tvalid   <= 1'b0;
          color_tuser    <= 1'b0;
          chroma_stats_tvalid <= 1'b0;
          chroma_stats_tuser  <= 1'b0;
          vertical_chroma_tvalid <= 1'b0;
          vertical_chroma_tuser  <= 1'b0;
          luma_stats_tvalid <= 1'b0;
          luma_stats_tuser  <= 1'b0;
          denoise_tvalid <= 1'b0;
          denoise_tuser  <= 1'b0;
          vivid_tvalid   <= 1'b0;
          vivid_tuser    <= 1'b0;
          chroma_word_addr <= 9'd0;
          chroma_row_index <= 10'd0;
          chroma_first_line_d <= 1'b1;
      end else begin
          wb_tvalid    <= scaled_tvalid;
          wb_tuser     <= scaled_tuser;
          color_tvalid <= wb_tvalid;
          color_tuser  <= wb_tuser;
          chroma_stats_tvalid <= color_tvalid;
          chroma_stats_tuser  <= color_tuser;
          vertical_chroma_tvalid <= chroma_stats_tvalid;
          vertical_chroma_tuser  <= chroma_stats_tuser;
          luma_stats_tvalid <= vertical_chroma_tvalid;
          luma_stats_tuser  <= vertical_chroma_tuser;
          denoise_tvalid <= luma_stats_tvalid;
          denoise_tuser  <= luma_stats_tuser;
          vivid_tvalid <= denoise_tvalid;
          vivid_tuser  <= denoise_tuser;
          src_tdata_d <= src_tdata_96;
          if (scaled_tvalid) begin
              wb_tdata_96 <= {wb_pixel(src_tdata_96[95:72], pic_mode),
                              wb_pixel(src_tdata_96[71:48], pic_mode),
                              wb_pixel(src_tdata_96[47:24], pic_mode),
                              wb_pixel(src_tdata_96[23:0],  pic_mode)};
          end
          if (wb_tvalid) begin
              color_tdata_96 <= {gamma_pixel(wb_tdata_96[95:72], pic_mode),
                                 gamma_pixel(wb_tdata_96[71:48], pic_mode),
                                 gamma_pixel(wb_tdata_96[47:24], pic_mode),
                                 gamma_pixel(wb_tdata_96[23:0],  pic_mode)};
          end
          if (color_tvalid) begin
              chroma_stats_tdata_80 <= chroma_current_stats_80;
              chroma_first_line_d <= color_tuser || (chroma_row_index == 10'd0);
              if(color_tuser) begin
                  chroma_word_addr <= 9'd1;
                  chroma_row_index <= 10'd0;
              end else if(chroma_word_addr == 9'd319) begin
                  chroma_word_addr <= 9'd0;
                  chroma_row_index <= chroma_row_index + 10'd1;
              end else begin
                  chroma_word_addr <= chroma_word_addr + 9'd1;
              end
          end
          if (chroma_stats_tvalid) begin
              vertical_chroma_tdata_80 <= merge_vertical_chroma(
                  chroma_stats_tdata_80, chroma_previous_stats_80,
                  chroma_first_line_d);
          end
          if (vertical_chroma_tvalid) begin
              luma_stats_tdata_80 <= {
                  smooth_dark_luma_pair(vertical_chroma_tdata_80[79:40]),
                  smooth_dark_luma_pair(vertical_chroma_tdata_80[39:0])};
          end
          if (luma_stats_tvalid) begin
              denoise_tdata_96 <= algorithm_flags[2] ? rebuild_chroma_quad(luma_stats_tdata_80) : bypass2;
          end
          if (denoise_tvalid) begin
              vivid_tdata_96 <= {vivid_pixel(denoise_tdata_96[95:72], pic_mode),
                                 vivid_pixel(denoise_tdata_96[71:48], pic_mode),
                                 vivid_pixel(denoise_tdata_96[47:24], pic_mode),
                                 vivid_pixel(denoise_tdata_96[23:0],  pic_mode)};
          end
      end
  end


  wire         awb_O_tuser_128;  //synthesis keep 
  wire         awb_O_tvalid_128;  //synthesis keep 
  wire [127:0] awb_O_tdata_128;  //synthesis keep 

  assign O_tdata  = awb_O_tdata_128;
  // Keep the baseline path frame-length preserving.  The original AWB
  // signal-delay block reconstructs valid/last and does not preserve a full
  // 1280x720 frame reliably on this port.  AWB can be reintroduced after the
  // capture, packer and DDR path have been verified independently.
  assign O_tlast  = scaled_tlast;
  assign O_tuser  = awb_O_tuser_128    ;
  assign O_tvalid = awb_O_tvalid_128   ;
  assign m_aixs_tready = O_tready;
  
  //assign m_aixs_tready = O_tready;
  //assign O_tdata  = m_aixs_tdata;
  //assign O_tlast  = m_aixs_tlast    ;
  //assign O_tuser  = m_aixs_tuser    ;
  //assign O_tvalid = m_aixs_tvalid   ;
 
 
demosaic #(
    .IMG_HEIGHT          (1080),
    .IMG_WIDTH           (1920),
    .data_complete_delay (50  ),
    .BAYER_MODE          ("BGGR")
)
u_demosaic
(
    .I_clk   			(axi4s_video_aclk)	,   // ??????
    .I_rst_n 			(I_rst_n)	,   // ????????????
    .axi4s_video_tdata 	(I_tdata)	,  // AXI4-Stream??????
    .axi4s_video_tdest 	(I_tdest)	,
    .axi4s_video_tlast 	(I_tlast)	,  // ????????
    .axi4s_video_tvalid	(I_tvalid)	,  // ?????????
    .axi4s_video_tuser 	(I_tuser)	,  // ????????
    .axi4s_video_tready	(I_tready)	,  // ???????????????
    .O_tlast  			(m_aixs_tlast)	,  // ???????????
    .O_tuser  			(m_aixs_tuser)	,  // ???????????
    .O_tdata  			(m_aixs_tdata)	,  // ??????
    .O_tvalid 			(m_aixs_tvalid)	,  // ????????????
    .O_tready   		(m_aixs_tready)	   // ??????????????
);
 
data128_96 u_data128_96 (
    .I_tdata(m_aixs_tdata),
    .O_tdata(m_aixs_tdata_96)
);

  rgb888_downscale_3to2 u_rgb888_downscale_3to2 (
      .I_clk   (axi4s_video_aclk),
      .I_rst_n (I_rst_n),
      .I_user  (m_aixs_tuser),
      .I_last  (m_aixs_tlast),
      .I_valid (m_aixs_tvalid),
      .I_data  (m_aixs_tdata_96),
      .O_user  (scaled_tuser),
      .O_last  (scaled_tlast),
      .O_valid (scaled_tvalid),
      .O_data  (scaled_tdata_96)
  );

    data_96bit_to_128bit u_data_96bit_to_128bit(
        .I_clk              ( axi4s_video_aclk  ),
        .I_rst_n            ( I_rst_n           ),
	
        .I_96b_frame_start  ( vivid_tuser       ),
        .I_96b_valid        ( vivid_tvalid      ),
        .I_96b_data         ( vivid_tdata_96    ),
	
        .O_128b_frame_start ( awb_O_tuser_128  	),
        .O_128b_valid       ( awb_O_tvalid_128 	),
        .O_128b_data        ( awb_O_tdata_128  	)
    );



//data96_128 u_data96_128 (
//    .I_tdata(awb_O_tdata),
//    .O_tdata(awb_O_tdata_128)
//);
	
endmodule

// 1920x1080 RGB888 to 1280x720 RGB888 polyphase scaler.
// Four pixels are transported per 96-bit word.  Scaling is deliberately done
// after demosaic so interpolation never mixes or rearranges Bayer phases.
module rgb888_downscale_3to2 (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_user,
    input  wire        I_last,
    input  wire        I_valid,
    input  wire [95:0] I_data,
    output reg         O_user,
    output reg         O_last,
    output reg         O_valid,
    output reg  [95:0] O_data
);
    localparam INPUT_WORDS_PER_LINE = 480;

    reg [8:0]  vertical_word;
    reg [1:0]  vertical_row_mod3;
    reg [1:0]  vertical_phase;
    reg [95:0] vertical_current;
    reg        vertical_valid;
    reg        vertical_last;
    reg [95:0] vertical_filtered_reg;
    reg        filtered_valid;
    reg        filtered_last;
    wire [8:0] ram_addr = I_user ? 9'd0 : vertical_word;
    wire [1:0] row_phase = I_user ? 2'd0 : vertical_row_mod3;
    wire [95:0] previous_line_q;

    reg [1:0]  horizontal_word_mod3;
    reg [95:0] horizontal_previous;
    reg        frame_pending;

    function [23:0] blend31_pixel;
        input [23:0] a;
        input [23:0] b;
        reg [9:0] r;
        reg [9:0] g;
        reg [9:0] bl;
        begin
            r  = {2'b0,a[23:16]} + ({2'b0,a[23:16]} << 1) + {2'b0,b[23:16]} + 10'd2;
            g  = {2'b0,a[15:8]}  + ({2'b0,a[15:8]}  << 1) + {2'b0,b[15:8]}  + 10'd2;
            bl = {2'b0,a[7:0]}   + ({2'b0,a[7:0]}   << 1) + {2'b0,b[7:0]}   + 10'd2;
            blend31_pixel = {r[9:2], g[9:2], bl[9:2]};
        end
    endfunction

    function [95:0] blend31_word;
        input [95:0] a;
        input [95:0] b;
        begin
            blend31_word = {
                blend31_pixel(a[95:72], b[95:72]),
                blend31_pixel(a[71:48], b[71:48]),
                blend31_pixel(a[47:24], b[47:24]),
                blend31_pixel(a[23:0],  b[23:0])
            };
        end
    endfunction

    // Phase 1 produces 3/4(previous)+1/4(current); phase 2 produces
    // 1/4(previous)+3/4(current).  Together, each three source rows yield two
    // evenly placed destination rows.
    wire [95:0] vertical_filtered = (vertical_phase == 2'd1) ?
                                    blend31_word(previous_line_q, vertical_current) :
                                    blend31_word(vertical_current, previous_line_q);

    rgb96_line_eram u_previous_line (
        .clk  (I_clk),
        .we   (I_valid),
        .addr (ram_addr),
        .din  (I_data),
        .dout (previous_line_q)
    );

    // Vertical 3:2 stage. The RAM's read-before-write output and the current
    // word are consumed together one clock later by the horizontal stage.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            vertical_word     <= 9'd0;
            vertical_row_mod3 <= 2'd0;
            vertical_phase    <= 2'd0;
            vertical_current  <= 96'd0;
            vertical_valid    <= 1'b0;
            vertical_last     <= 1'b0;
            vertical_filtered_reg <= 96'd0;
            filtered_valid    <= 1'b0;
            filtered_last     <= 1'b0;
        end else begin
            vertical_filtered_reg <= vertical_filtered;
            filtered_valid <= vertical_valid;
            filtered_last  <= vertical_last;
            vertical_valid <= 1'b0;
            vertical_last  <= 1'b0;
            if(I_user) begin
                vertical_word     <= 9'd0;
                vertical_row_mod3 <= 2'd0;
            end
            if(I_valid) begin
                vertical_current <= I_data;
                vertical_phase   <= row_phase;
                vertical_valid   <= (row_phase != 2'd0);
                vertical_last    <= I_last;
                if(ram_addr == INPUT_WORDS_PER_LINE - 1) begin
                    vertical_word <= 9'd0;
                    vertical_row_mod3 <= (row_phase == 2'd2) ?
                                         2'd0 : row_phase + 2'd1;
                end else begin
                    vertical_word <= ram_addr + 9'd1;
                end
            end
        end
    end

    // Horizontal 3:2 stage. Three 4-pixel input words become two 4-pixel
    // output words, with a quarter-pixel polyphase interpolation at each
    // destination sample location.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            horizontal_word_mod3 <= 2'd0;
            horizontal_previous  <= 96'd0;
            frame_pending         <= 1'b0;
            O_user                <= 1'b0;
            O_last                <= 1'b0;
            O_valid               <= 1'b0;
            O_data                <= 96'd0;
        end else begin
            O_user  <= 1'b0;
            O_last  <= 1'b0;
            O_valid <= 1'b0;
            if(I_user) begin
                horizontal_word_mod3 <= 2'd0;
                horizontal_previous  <= 96'd0;
                frame_pending         <= 1'b1;
            end
            if(filtered_valid) begin
                case(horizontal_word_mod3)
                    2'd0: begin
                        horizontal_previous <= vertical_filtered_reg;
                    end
                    2'd1: begin
                        O_data <= {
                            blend31_pixel(horizontal_previous[95:72], horizontal_previous[71:48]),
                            blend31_pixel(horizontal_previous[47:24], horizontal_previous[71:48]),
                            blend31_pixel(horizontal_previous[23:0],  vertical_filtered_reg[95:72]),
                            blend31_pixel(vertical_filtered_reg[71:48], vertical_filtered_reg[95:72])
                        };
                        O_valid <= 1'b1;
                        O_user  <= frame_pending;
                        if(frame_pending)
                            frame_pending <= 1'b0;
                        horizontal_previous <= vertical_filtered_reg;
                    end
                    2'd2: begin
                        O_data <= {
                            blend31_pixel(horizontal_previous[47:24], horizontal_previous[23:0]),
                            blend31_pixel(vertical_filtered_reg[95:72], horizontal_previous[23:0]),
                            blend31_pixel(vertical_filtered_reg[71:48], vertical_filtered_reg[47:24]),
                            blend31_pixel(vertical_filtered_reg[23:0],  vertical_filtered_reg[47:24])
                        };
                        O_valid <= 1'b1;
                        O_last  <= filtered_last;
                    end
                    default: horizontal_previous <= horizontal_previous;
                endcase
                horizontal_word_mod3 <= (horizontal_word_mod3 == 2'd2) ?
                                        2'd0 : horizontal_word_mod3 + 2'd1;
            end
        end
    end
endmodule

module chroma80_line_eram (
    input  wire        clk,
    input  wire        we,
    input  wire [8:0]  addr,
    input  wire [79:0] din,
    output wire [79:0] dout
);
    wire [79:0] unused_dob;
    PH1P_LOGIC_ERAM #(
        .DATA_WIDTH_A (80), .DATA_WIDTH_B (80),
        .ADDR_WIDTH_A (9),  .ADDR_WIDTH_B (9),
        .DATA_DEPTH_A (512), .DATA_DEPTH_B (512),
        .MODE ("DP"), .REGMODE_A ("NOREG"), .REGMODE_B ("NOREG"),
        .WRITEMODE_A ("READBEFOREWRITE"), .WRITEMODE_B ("NORMAL"),
        .CLKMODE ("SYNC"), .IMPLEMENT ("20K(FAST)"),
        .ECC_ENCODE ("DISABLE"), .ECC_DECODE ("DISABLE"),
        .SSROVERCE ("DISABLE"), .INIT_FILE ("NONE"), .FILL_ALL ("NONE")
    ) u_eram (
        .dia(din), .addra(addr), .cea(1'b1), .ocea(1'b0),
        .clka(clk), .wea(we), .rsta(1'b0), .bea(1'b0), .doa(dout),
        .dib(80'd0), .addrb(9'd0), .ceb(1'b0), .oceb(1'b0),
        .clkb(clk), .web(1'b0), .rstb(1'b0), .beb(1'b0), .dob(unused_dob),
        .ecc_sbiterrinj(1'b0), .ecc_dbiterrinj(1'b0),
        .ecc_sbiterr(), .ecc_dbiterr()
    );
endmodule

// Logical 96x512 read-before-write RAM. TD maps it to native ERAM20K blocks.
module rgb96_line_eram (
    input  wire        clk,
    input  wire        we,
    input  wire [8:0]  addr,
    input  wire [95:0] din,
    output wire [95:0] dout
);
    wire [95:0] unused_dob;
    PH1P_LOGIC_ERAM #(
        .DATA_WIDTH_A (96), .DATA_WIDTH_B (96),
        .ADDR_WIDTH_A (9),  .ADDR_WIDTH_B (9),
        .DATA_DEPTH_A (512), .DATA_DEPTH_B (512),
        .MODE ("DP"), .REGMODE_A ("NOREG"), .REGMODE_B ("NOREG"),
        .WRITEMODE_A ("READBEFOREWRITE"), .WRITEMODE_B ("NORMAL"),
        .CLKMODE ("SYNC"), .IMPLEMENT ("20K(FAST)"),
        .ECC_ENCODE ("DISABLE"), .ECC_DECODE ("DISABLE"),
        .SSROVERCE ("DISABLE"), .INIT_FILE ("NONE"), .FILL_ALL ("NONE")
    ) u_eram (
        .dia(din), .addra(addr), .cea(1'b1), .ocea(1'b0),
        .clka(clk), .wea(we), .rsta(1'b0), .bea(1'b0), .doa(dout),
        .dib(96'd0), .addrb(9'd0), .ceb(1'b0), .oceb(1'b0),
        .clkb(clk), .web(1'b0), .rstb(1'b0), .beb(1'b0), .dob(unused_dob),
        .ecc_sbiterrinj(1'b0), .ecc_dbiterrinj(1'b0),
        .ecc_sbiterr(), .ecc_dbiterr()
    );
endmodule

