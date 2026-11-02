// Bundled-data transfer: source updates once per video frame and holds data
// until the next frame. Toggle traverses three FFs before capture; payload
// traverses two. No per-bit live-coordinate sampling by the UART.
module spot_cdc #(parameter integer CLK_HZ=100000000)(
    input wire clk, rst_n, frame_toggle, pixel_valid,
    input wire [11:0] pixel_x, pixel_y, pixel_w, pixel_h,
    output reg valid,
    output reg [11:0] x,y,w,h
);
    reg [2:0] sync;
    reg seen;
    reg [48:0] bundle1,bundle2;
    reg [23:0] age;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync<=0; seen<=0; bundle1<=0; bundle2<=0;
            valid<=0; x<=0; y<=0; w<=0; h<=0; age<=0;
        end else begin
            sync<={sync[1:0],frame_toggle};
            bundle1<={pixel_valid,pixel_x,pixel_y,pixel_w,pixel_h};
            bundle2<=bundle1;
            if (sync[2] != seen) begin
                seen<=sync[2]; {valid,x,y,w,h}<=bundle2; age<=0;
            end else if (age < CLK_HZ/10) age<=age+1'b1;
            else valid<=0;
        end
    end
endmodule

// B19: keep drawing while the gimbal catches up. Only the projected rectangle
// centre is clamped to the existing +/-24px optical window. The motor receives
// the original, unclamped target coordinates. Loss/fault/mode still blank light.
module vision_laser_window(
    input wire clk,rst_n,valid,
    input wire [1:0] mode,
    input wire [7:0] status,
    input wire [11:0] x,y,aim_x,aim_y,
    output reg allowed,
    output wire [11:0] limited_x,limited_y
);
    wire ready = status[1:0]==3 && !status[6] && !status[3] && !status[2] && !status[5];
    wire coordinates_ok = x<1280 && y<720 && aim_x<1280 && aim_y<720;
    wire [12:0] left = aim_x<24 ? 13'd0 : {1'b0,aim_x}-13'd24;
    wire [12:0] right = aim_x>1255 ? 13'd1279 : {1'b0,aim_x}+13'd24;
    wire [12:0] top = aim_y<24 ? 13'd0 : {1'b0,aim_y}-13'd24;
    wire [12:0] bottom = aim_y>695 ? 13'd719 : {1'b0,aim_y}+13'd24;
    assign limited_x = {1'b0,x}<left ? left[11:0] : {1'b0,x}>right ? right[11:0] : x;
    assign limited_y = {1'b0,y}<top ? top[11:0] : {1'b0,y}>bottom ? bottom[11:0] : y;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) allowed<=0;
        else allowed<=valid && mode==1 && ready && coordinates_ok;
    end
endmodule
