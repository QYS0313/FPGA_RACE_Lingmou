`timescale 1ns/1ps
// 64-bin luminance CDF equalization, sampled at a 4x4 grid. Histogram counts
// only the selected area; mapping is double-buffered and switched in blanking.
// A9: quarter-strength CDF, +/-16 luma correction, slope-limited LUT with
// interpolation. Adjacent 4-level knots rise by 3..5 (gain 0.75..1.25).
// RGB receives the same luma delta (hue retained except for saturation).
// Fixed two-clock video latency, including disabled/outside-ROI paths.
module video_histogram (
 input wire clk,rst_n,in_vs,in_hs,in_de,input wire[23:0] in_rgb,
 input wire enable,roi_only,roi_valid,
 input wire[11:0] x0,y0,x1,y1,
 output reg out_vs,out_hs,out_de,output reg[23:0] out_rgb,
 output reg[23:0] original_rgb
);
 localparam CLEAR=0,ACQUIRE=1,ADDRESS=2,READ=3,PREPARE=4,DIVIDE=5,WRITE=6;
 reg [2:0] state;
 reg [5:0] bin;
 reg [11:0] x,y;
 reg de_d,vs_d;
 reg en,region_only,region_valid;
 reg [11:0] rx0,ry0,rx1,ry1;
 wire [11:0] px=de_d?x:12'd0;
 wire selected=!region_only||(region_valid&&px>=rx0&&px<=rx1&&y>=ry0&&y<=ry1);
 wire [11:0] lum_sum={2'b0,in_rgb[23:16],2'b0}+{4'b0,in_rgb[23:16]}+
    {1'b0,in_rgb[15:8],3'b0}+{4'b0,in_rgb[15:8]}+{3'b0,in_rgb[7:0],1'b0};
 wire [7:0] lum=lum_sum[11:4];
 wire sample=in_de&&selected&&px[1:0]==0&&y[1:0]==0&&state==ACQUIRE;
 (* ram_style="distributed" *) reg [15:0] histogram[0:63];
 (* ram_style="distributed" *) reg [7:0] mapping[0:127];
 reg [15:0] hist_q,total,total_frame,cumulative,minimum;
 reg seen,sample_d;
 reg [5:0] sample_bin;
 reg active_bank,ready,mapping_valid;
 reg build_flat,ready_flat,active_flat;
 reg [7:0] previous_map;
 wire [7:0] base_luma={bin,2'b0};
 // No multipliers: keep this display enhancement small and bounded.
 function [7:0] gentle_map;
  input [5:0] b;input [7:0] cdf,previous;
  integer base,wanted,lo,hi,end_lo,end_hi;
  begin
   base={b,2'b0};wanted=((base<<1)+base+cdf+2)>>2;
   lo=base-16;hi=base+16;
   if(lo<0)lo=0;if(hi>255)hi=255;
   if(lo<previous+3)lo=previous+3;
   if(hi>previous+5)hi=previous+5;
   // Reserve enough range to meet the virtual endpoint (256,256).
   end_lo=({1'b0,b}<<2)+b-64;end_hi=64+({1'b0,b}<<1)+b;
   if(lo<end_lo)lo=end_lo;if(hi>end_hi)hi=end_hi;
   if(wanted<lo)wanted=lo;if(wanted>hi)wanted=hi;
   gentle_map=b==0?8'd0:wanted[7:0];
  end
 endfunction
 reg [23:0] numerator;
 reg [15:0] denominator;
 reg div_start;
 wire div_done;wire[23:0] quotient;
 wire [7:0] next_map=denominator==0?base_luma:gentle_map(bin,quotient[7:0],previous_map);
 wire [15:0] next_sum=cumulative+hist_q;
 wire [15:0] next_min=seen?minimum:hist_q;
 wire [15:0] spread=total_frame-next_min;
 wire [15:0] rank=next_sum-next_min;
 contest_div_seq #(.NW(24),.DW(16)) u_div(.clk(clk),.start(div_start),
      .num(numerator),.den(denominator),.quo(quotient),.done(div_done));
 // Reset-free memory ports. Samples are at least four clocks apart, so the
 // increment has completed before the next read of the same bin.
 always @(posedge clk)begin
  hist_q<=histogram[state==ACQUIRE?lum[7:2]:bin];
  if(state==CLEAR||state==PREPARE)histogram[bin]<=0;
  else if(sample_d)histogram[sample_bin]<=hist_q+1'b1;
  if(state==WRITE)mapping[{!active_bank,bin}]<=next_map;
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=CLEAR;bin<=0;x<=0;y<=0;de_d<=0;vs_d<=0;en<=0;
   region_only<=0;region_valid<=0;rx0<=0;ry0<=0;rx1<=0;ry1<=0;
   total<=0;total_frame<=0;cumulative<=0;minimum<=0;seen<=0;
   sample_d<=0;sample_bin<=0;active_bank<=0;ready<=0;mapping_valid<=0;
   previous_map<=0;build_flat<=1;ready_flat<=1;active_flat<=1;
   numerator<=0;denominator<=1;div_start<=0;
  end else begin
   de_d<=in_de;vs_d<=in_vs;div_start<=0;sample_d<=sample;
   if(in_vs)begin
    x<=0;y<=0;en<=enable;region_only<=roi_only;region_valid<=roi_valid;
    rx0<=x0;ry0<=y0;rx1<=x1;ry1<=y1;
    if(ready)begin active_bank<=~active_bank;mapping_valid<=1;active_flat<=ready_flat;ready<=0;end
   end else if(in_de)x<=px+1'b1;
   else if(de_d)begin x<=0;y<=y+1'b1;end
   if(sample)begin sample_bin<=lum[7:2];total<=total+1'b1;end
   case(state)
    CLEAR:if(bin==63)begin state<=ACQUIRE;bin<=0;end else bin<=bin+1'b1;
    ACQUIRE:if(in_vs&&!vs_d)begin
      total_frame<=total;total<=0;cumulative<=0;minimum<=0;seen<=0;bin<=0;state<=ADDRESS;
      previous_map<=0;build_flat<=total==0;
    end
    ADDRESS:state<=READ;
    READ:state<=PREPARE;
    PREPARE:begin
      cumulative<=next_sum;
      if(hist_q==total_frame&&hist_q!=0)build_flat<=1;
      if(!seen&&hist_q!=0)begin minimum<=hist_q;seen<=1;end
      if(!seen&&hist_q==0)begin numerator<=0;denominator<=total_frame;end
      else begin numerator<={rank,8'b0}-{8'b0,rank};denominator<=spread;end
      state<=DIVIDE;div_start<=1;
    end
    DIVIDE:if(div_done)state<=WRITE;
    WRITE:begin
      previous_map<=next_map;
      if(bin==63)begin ready<=1;ready_flat<=build_flat;state<=ACQUIRE;end
      else begin bin<=bin+1'b1;state<=ADDRESS;end
    end
   endcase
  end
 end
 reg [7:0] map_q,map_next_q,lum_a;
 wire [8:0] span=(lum_a[7:2]==63?9'd256:{1'b0,map_next_q})-{1'b0,map_q};
 wire [10:0] fraction=(lum_a[0]?{2'd0,span}:11'd0)+(lum_a[1]?{1'b0,span,1'b0}:11'd0)+11'd2;
 wire [9:0] interpolated={2'd0,map_q}+fraction[10:2];
 wire [7:0] mapped_luma=interpolated>255?8'd255:interpolated[7:0];
 reg [23:0] rgb_a;
 reg vs_a,hs_a,de_a,apply_a;
 function [7:0] channel;
  input[7:0] c,m,l;
  reg signed[10:0] v;
  begin
   v=$signed({3'b0,c})+$signed({3'b0,m})-$signed({3'b0,l});
   channel=v<0?8'd0:v>255?8'd255:v[7:0];
  end
 endfunction
 always @(posedge clk)map_q<=mapping[{active_bank,lum[7:2]}];
 always @(posedge clk)map_next_q<=mapping[{active_bank,(lum[7:2]+6'd1)}];
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin rgb_a<=0;lum_a<=0;vs_a<=0;hs_a<=0;de_a<=0;apply_a<=0;
    out_vs<=0;out_hs<=0;out_de<=0;out_rgb<=0;original_rgb<=0;end
  else begin
   rgb_a<=in_rgb;lum_a<=lum;vs_a<=in_vs;hs_a<=in_hs;de_a<=in_de;
   apply_a<=en&&selected&&mapping_valid&&!active_flat;
   out_vs<=vs_a;out_hs<=hs_a;out_de<=de_a;
   original_rgb<=rgb_a;
   out_rgb<=apply_a?{channel(rgb_a[23:16],mapped_luma,lum_a),channel(rgb_a[15:8],mapped_luma,lum_a),channel(rgb_a[7:0],mapped_luma,lum_a)}:rgb_a;
  end
 end
endmodule
