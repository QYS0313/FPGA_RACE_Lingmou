// Held frame bundle crossing to 100 MHz. A0: enable/color, A1: threshold,
// A2: status, A3: identity E2. Status bit 0 valid, [2:1] shape, [4:3] color.
module shape_link #(parameter CLK_HZ=100000000)(
 input wire clk,rst_n,command_valid,input wire [7:0] cmd,data,
 output reg [7:0] reply,output reg allow_full_range,output reg [3:0] control,output reg [7:0] threshold,
 input wire range_fault,laser_on,drawing_requested,input wire [31:0] pixel_diagnostic,
 input wire frame_toggle,pixel_valid,input wire [1:0] pixel_kind,pixel_color,
 input wire [3:0] pixel_count,input wire [191:0] pixel_vertices,
 input wire [11:0] pixel_x,pixel_y,pixel_w,pixel_h,
 output reg valid,output reg [1:0] kind,output reg [3:0] count,
 output reg [191:0] vertices,output reg [11:0] x,y,w,h);
 reg [2:0] sync;reg seen;reg [280:0] meta,bundle;reg [31:0] diagnostic;
 reg [1:0] color;reg [23:0] age;
 always @* begin
  reply=8'hff;
  case(cmd)
   8'ha0:reply={3'b0,data[4:0]};8'ha1:reply=data;
   8'ha2:reply={3'b0,color,kind,valid};8'ha3:reply=8'he2;8'ha4:reply=8'he7;8'ha5:reply={laser_on,drawing_requested,allow_full_range,control,range_fault};
   8'ha6:reply=diagnostic[7:0];8'ha7:reply=diagnostic[15:8];8'ha8:reply=diagnostic[23:16];8'ha9:reply=diagnostic[31:24];
  endcase
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   allow_full_range<=0;control<=0;threshold<=120;sync<=0;seen<=0;meta<=0;bundle<=0;
   diagnostic<=0;valid<=0;kind<=0;color<=0;count<=0;vertices<=0;x<=0;y<=0;w<=0;h<=0;age<=0;
  end else begin
   if(command_valid)begin
    if(cmd==8'ha0)begin allow_full_range<=data[4];control<=data[3:0];valid<=0;end
    if(cmd==8'ha1)begin threshold<=data;valid<=0;end
   end
   sync<={sync[1:0],frame_toggle};
   meta<={pixel_diagnostic,pixel_valid,pixel_kind,pixel_color,pixel_count,pixel_vertices,pixel_x,pixel_y,pixel_w,pixel_h};bundle<=meta;
   if(sync[2]!=seen)begin
    seen<=sync[2];age<=0;
    {diagnostic,valid,kind,color,count,vertices,x,y,w,h}<=bundle;
   end else if(age<CLK_HZ/10)age<=age+1'b1;else valid<=0;
   if(!control[0])valid<=0;
  end
 end
endmodule
