`timescale 1ns/1ps
// A5: frame age from FPGA RAW input SOF to first active output pixel.
// A common millisecond clock crosses domains as registered Gray code.
// Timestamp travels WITH the image in the four unused DDR padding bytes;
// no buffer-pointer inference, extra DDR access, or RGB/video latency change.
module video_latency #(parameter TICKS_PER_MS=100000) (
 input wire ref_clk,camera_clk,pixel_clk,rst_n,
 input wire camera_sof,packed_sof,
 input wire [127:0] packed_rgb,
 output wire [127:0] tagged_rgb,
 input wire [23:0] displayed_stamp,
 input wire displayed_stamp_valid,
 input wire display_vs,display_de,
 output reg [11:0] latency_ms,
 output reg latency_valid
);
 reg [16:0] tick;
 reg [23:0] now_ms,now_gray;
 wire [23:0] next_ms=now_ms+24'd1;
 always @(posedge ref_clk or negedge rst_n) begin
  if(!rst_n) begin tick<=0;now_ms<=0;now_gray<=0;end
  else if(tick==TICKS_PER_MS-1) begin
   tick<=0;now_ms<=next_ms;now_gray<=next_ms^(next_ms>>1);
  end else tick<=tick+1'b1;
 end
 (* ASYNC_REG="TRUE" *) reg [23:0] cam_meta,cam_gray,pix_meta,pix_gray;
 function [23:0] ungray;
  input [23:0] value;integer i;
  begin ungray[23]=value[23];for(i=22;i>=0;i=i-1)ungray[i]=ungray[i+1]^value[i];end
 endfunction
 reg raw_d,packed_d,raw_valid,write_valid;
 reg [23:0] raw_stamp,write_stamp;
 always @(posedge camera_clk or negedge rst_n) begin
  if(!rst_n) begin
   cam_meta<=0;cam_gray<=0;raw_d<=0;packed_d<=0;
   raw_stamp<=0;write_stamp<=0;raw_valid<=0;write_valid<=0;
  end else begin
   cam_meta<=now_gray;cam_gray<=cam_meta;
   raw_d<=camera_sof;packed_d<=packed_sof;
   if(camera_sof&&!raw_d) begin raw_stamp<=ungray(cam_gray);raw_valid<=1;end
   if(packed_sof&&!packed_d) begin
    write_stamp<=(camera_sof&&!raw_d) ? ungray(cam_gray) : raw_stamp;
    write_valid<=raw_valid||(camera_sof&&!raw_d);
   end
  end
 end
 assign tagged_rgb={write_valid ? 8'hD5 : 8'h00,packed_rgb[119:96],
                    write_stamp[23:16],packed_rgb[87:64],
                    write_stamp[15:8],packed_rgb[55:32],
                    write_stamp[7:0],packed_rgb[23:0]};
 reg first_pixel;
 wire [23:0] age_ms=ungray(pix_gray)-displayed_stamp;
 always @(posedge pixel_clk or negedge rst_n) begin
  if(!rst_n) begin pix_meta<=0;pix_gray<=0;first_pixel<=0;latency_ms<=0;latency_valid<=0;end
  else begin
   pix_meta<=now_gray;pix_gray<=pix_meta;
   if(display_vs) first_pixel<=1;
   else if(display_de&&first_pixel) begin
    first_pixel<=0;
    latency_valid<=displayed_stamp_valid && (age_ms<=24'd4095);
    latency_ms<=age_ms[11:0];
   end
  end
 end
endmodule

// A7 display-only conditioning. Keep the measurement above unmodified.
// Low-pass valid samples at 1/8 gain, publish every 30 output frames (~0.5s),
// then subtract the requested 50 ms presentation offset with zero saturation.
// Brief missing samples hold the last value; 30 missing frames show dashes.
module video_latency_display #(
 parameter REFRESH_FRAMES=30, LOSS_FRAMES=30
) (
 input wire clk,rst_n,frame_start,
 input wire [11:0] latency_ms,
 input wire latency_valid,
 output reg [11:0] display_ms,
 output reg display_valid
);
 reg [14:0] filtered_q3;
 reg [7:0] refresh_count,missing_count;
 wire [15:0] next_filtered={1'b0,filtered_q3}-{4'b0,filtered_q3[14:3]}+{4'b0,latency_ms};
 wire [12:0] rounded_ms=(next_filtered+16'd4)>>3;
 wire [11:0] smooth_ms=rounded_ms>13'd4095 ? 12'd4095 : rounded_ms[11:0];
 function [11:0] adjusted;
  input [11:0] value;
  begin adjusted=value>12'd50 ? value-12'd50 : 12'd0;end
 endfunction
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   filtered_q3<=0;refresh_count<=0;missing_count<=0;display_ms<=0;display_valid<=0;
  end else if(frame_start) begin
   if(latency_valid) begin
    missing_count<=0;
    if(!display_valid) begin
     filtered_q3<={latency_ms,3'b0};refresh_count<=0;
     display_ms<=adjusted(latency_ms);display_valid<=1;
    end else begin
     filtered_q3<=next_filtered[14:0];
     if(refresh_count==REFRESH_FRAMES-1) begin display_ms<=adjusted(smooth_ms);refresh_count<=0;end
     else refresh_count<=refresh_count+1'b1;
    end
   end else if(display_valid) begin
    if(missing_count==LOSS_FRAMES-1) begin
     display_valid<=0;display_ms<=0;filtered_q3<=0;refresh_count<=0;missing_count<=0;
    end else missing_count<=missing_count+1'b1;
   end
  end
 end
endmodule
