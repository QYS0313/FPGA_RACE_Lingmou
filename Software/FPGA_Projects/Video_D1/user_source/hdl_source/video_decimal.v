`timescale 1ns/1ps
// Shared 12-bit double-dabble conversion; no divide/modulo logic in pixels.
// Eight values are captured together once per frame. Publish only when all
// eight conversions complete, well inside vertical blanking.
module video_decimal (
    input wire clk,rst_n,start,
    input wire [95:0] values,
    output reg [127:0] digits
);
    reg busy;
    reg [6:0] count;
    reg [95:0] pending;
    reg [127:0] result;
    reg [15:0] bcd;
    reg [15:0] adjusted;
    wire [15:0] next_bcd={adjusted[14:0],pending[95]};
    integer i;
    always @* begin
        adjusted=bcd;
        for(i=0;i<4;i=i+1)
            if(bcd[i*4+:4]>=5) adjusted[i*4+:4]=bcd[i*4+:4]+4'd3;
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin busy<=0; count<=0; pending<=0; result<=0; bcd<=0; digits<=0; end
        else if(start) begin busy<=1; count<=0; pending<=values; result<=0; bcd<=0; end
        else if(busy) begin
            pending<={pending[94:0],1'b0}; count<=count+1'b1;
            if(count==11 || count==23 || count==35 || count==47 || count==59 || count==71 || count==83 || count==95) begin
                result<={result[111:0],next_bcd}; bcd<=0;
                if(count==95) begin digits<={result[111:0],next_bcd}; busy<=0; end
            end else bcd<=next_bcd;
        end
    end
endmodule
