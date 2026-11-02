`timescale 1ns/1ps
// Two exact unsigned Q16 ratios per frame, sharing one restoring divider.
// A new frame supersedes any unfinished calculation. Inputs are snapshotted
// together, so the blue pass never sees the next frame's accumulating sums.
module video_awb_div (
    input wire clk, rst_n, start,
    input wire [31:0] sum_r, sum_g, sum_b,
    output reg [47:0] ratio_r, ratio_b,
    output reg done_r, done_b
);
    reg busy, blue;
    reg [5:0] count;
    reg [47:0] numerator, quotient;
    reg [31:0] denominator, blue_den;
    reg [32:0] remainder;
    wire [32:0] shifted = {remainder[31:0],quotient[47]};
    wire subtract = shifted >= {1'b0,denominator};
    wire [47:0] next_q = {quotient[46:0],subtract};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy<=0; blue<=0; count<=0; numerator<=0; quotient<=0;
            denominator<=1; blue_den<=1; remainder<=0;
            ratio_r<=0; ratio_b<=0; done_r<=0; done_b<=0;
        end else begin
            done_r<=0; done_b<=0;
            if (start) begin
                numerator<={sum_g,16'b0}; quotient<={sum_g,16'b0};
                denominator<=sum_r; blue_den<=sum_b;
                remainder<=0; count<=0; blue<=0; busy<=1;
            end else if (busy) begin
                quotient<=next_q;
                remainder<=subtract ? shifted-{1'b0,denominator} : shifted;
                count<=count+1'b1;
                if (count==47) begin
                    // Zero denominator deliberately saturates high, as the
                    // old unsigned divider did; the ISP clamps to gain 3.
                    if (!blue) begin
                        ratio_r<=next_q; done_r<=1; blue<=1;
                        denominator<=blue_den; quotient<=numerator;
                        remainder<=0; count<=0;
                    end else begin
                        ratio_b<=next_q; done_b<=1; busy<=0;
                    end
                end
            end
        end
    end
endmodule
