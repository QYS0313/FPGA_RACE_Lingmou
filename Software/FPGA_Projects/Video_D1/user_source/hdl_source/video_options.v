`timescale 1ns/1ps
// D1-only options: atomic staging/commit, held bundles to both video clocks.
// flags: AWB,tone,denoise,vivid,histogram,shading (bits 0..5).
// scope: basic ROI, effect ROI, histogram ROI, target-region source (0..3).
// Remaining four little-endian uint16 fields: inclusive x0,y0,x1,y1.
module video_options (
 input wire clk,pixel_clk,camera_clk,rst_n,pixel_vs,camera_sof,
 input wire valid,input wire [7:0] cmd,data,
 output reg [7:0] reply, output reg [79:0] pixel_config,
 output reg [7:0] isp_flags
);
 localparam [79:0] DEFAULT={16'd539,16'd959,16'd180,16'd320,8'h02,8'h2f};
 reg [79:0] shadow,committed,pixel_pending;
 reg [7:0] camera_pending,index;
 reg req,pack,cack;
 reg [2:0] ps,cs,pa,ca;
 wire busy=(req!=pa[2])||(req!=ca[2]);
 wire acceptable=(shadow[7:6]==0)&&(shadow[15:12]==0)&&
      shadow[31:16]<=shadow[63:48]&&shadow[47:32]<=shadow[79:64]&&
      shadow[63:48]<1280&&shadow[79:64]<720;
 always @* begin
  reply=0;
  case(cmd)
   8'h70:reply=data<10?data:8'hff;
   8'h71:reply=index<10?data:8'hff;
   8'h72:reply=data!=8'ha5 ? 8'd3 : busy ? 8'd1 : acceptable ? 8'd0 : 8'd2;
   8'h73:reply=data<10?committed[data*8+:8]:8'hff;
   8'h74:reply=8'hc2;
   8'h75:reply={7'd0,busy};
  endcase
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin shadow<=DEFAULT;committed<=DEFAULT;index<=0;req<=0;pa<=0;ca<=0;end
  else begin
   pa<={pa[1:0],pack};ca<={ca[1:0],cack};
   if(valid)case(cmd)
    8'h70:index<=data;
    8'h71:if(index<10)shadow[index*8+:8]<=data;
    8'h72:if(reply==0)begin committed<=shadow;req<=~req;end
   endcase
  end
 end
 always @(posedge pixel_clk or negedge rst_n)begin
  if(!rst_n)begin ps<=0;pack<=0;pixel_pending<=DEFAULT;pixel_config<=DEFAULT;end
  else begin
   ps<={ps[1:0],req};
   if(ps[2]!=pack)begin pixel_pending<=committed;pack<=ps[2];end
   if(pixel_vs)pixel_config<=pixel_pending;
  end
 end
 always @(posedge camera_clk or negedge rst_n)begin
  if(!rst_n)begin cs<=0;cack<=0;camera_pending<=8'h2f;isp_flags<=8'h2f;end
  else begin
   cs<={cs[1:0],req};
   if(cs[2]!=cack)begin camera_pending<=committed[7:0];cack<=cs[2];end
   if(camera_sof)isp_flags<=camera_pending;
  end
 end
endmodule

