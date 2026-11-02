`timescale 1ns / 1ps

// Autonomous circle: 256 samples, radius 4000 DAC codes, center 32768.
// 100 us per sample => 39.0625 circles/second at the default 100 MHz clock.
// Quarter-wave integer LUT avoids multipliers, dividers and external ROM files.
module galvo_circle #(
    parameter integer CLK_HZ = 100000000,
    parameter integer POINT_HZ = 10000
) (
    input wire clk,
    input wire rst_n,
    input wire dac_ready,
    input wire dac_busy,
    output reg [15:0] target_x,
    output reg [15:0] target_y
);
    localparam integer POINT_DIV = CLK_HZ / POINT_HZ;
    reg [$clog2(POINT_DIV)-1:0] point_count;
    reg [7:0] phase;
    wire [7:0] next_phase = phase + 8'd1;
    wire [7:0] next_cos_phase = phase + 8'd65;

    function [15:0] circle_code;
        input [7:0] angle;
        reg [6:0] index;
        reg [12:0] magnitude;
        begin
            index = angle[6] ? (7'd64 - {1'b0, angle[5:0]}) : {1'b0, angle[5:0]};
            case (index)
                7'd0: magnitude = 13'd0;
                7'd1: magnitude = 13'd98;
                7'd2: magnitude = 13'd196;
                7'd3: magnitude = 13'd294;
                7'd4: magnitude = 13'd392;
                7'd5: magnitude = 13'd490;
                7'd6: magnitude = 13'd587;
                7'd7: magnitude = 13'd684;
                7'd8: magnitude = 13'd780;
                7'd9: magnitude = 13'd876;
                7'd10: magnitude = 13'd972;
                7'd11: magnitude = 13'd1067;
                7'd12: magnitude = 13'd1161;
                7'd13: magnitude = 13'd1255;
                7'd14: magnitude = 13'd1348;
                7'd15: magnitude = 13'd1440;
                7'd16: magnitude = 13'd1531;
                7'd17: magnitude = 13'd1621;
                7'd18: magnitude = 13'd1710;
                7'd19: magnitude = 13'd1798;
                7'd20: magnitude = 13'd1886;
                7'd21: magnitude = 13'd1972;
                7'd22: magnitude = 13'd2056;
                7'd23: magnitude = 13'd2140;
                7'd24: magnitude = 13'd2222;
                7'd25: magnitude = 13'd2303;
                7'd26: magnitude = 13'd2383;
                7'd27: magnitude = 13'd2461;
                7'd28: magnitude = 13'd2538;
                7'd29: magnitude = 13'd2613;
                7'd30: magnitude = 13'd2686;
                7'd31: magnitude = 13'd2758;
                7'd32: magnitude = 13'd2828;
                7'd33: magnitude = 13'd2897;
                7'd34: magnitude = 13'd2964;
                7'd35: magnitude = 13'd3029;
                7'd36: magnitude = 13'd3092;
                7'd37: magnitude = 13'd3153;
                7'd38: magnitude = 13'd3213;
                7'd39: magnitude = 13'd3270;
                7'd40: magnitude = 13'd3326;
                7'd41: magnitude = 13'd3379;
                7'd42: magnitude = 13'd3431;
                7'd43: magnitude = 13'd3480;
                7'd44: magnitude = 13'd3528;
                7'd45: magnitude = 13'd3573;
                7'd46: magnitude = 13'd3616;
                7'd47: magnitude = 13'd3657;
                7'd48: magnitude = 13'd3696;
                7'd49: magnitude = 13'd3732;
                7'd50: magnitude = 13'd3766;
                7'd51: magnitude = 13'd3798;
                7'd52: magnitude = 13'd3828;
                7'd53: magnitude = 13'd3855;
                7'd54: magnitude = 13'd3880;
                7'd55: magnitude = 13'd3903;
                7'd56: magnitude = 13'd3923;
                7'd57: magnitude = 13'd3941;
                7'd58: magnitude = 13'd3957;
                7'd59: magnitude = 13'd3970;
                7'd60: magnitude = 13'd3981;
                7'd61: magnitude = 13'd3989;
                7'd62: magnitude = 13'd3995;
                7'd63: magnitude = 13'd3999;
                7'd64: magnitude = 13'd4000;
                default: magnitude = 13'd0;
            endcase
            circle_code = angle[7] ? (16'd32768 - {3'd0, magnitude})
                                   : (16'd32768 + {3'd0, magnitude});
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            point_count <= 0;
            phase <= 0;
            target_x <= 16'd36768;
            target_y <= 16'd32768;
        end else if (!dac_ready) begin
            point_count <= 0;
            phase <= 0;
            target_x <= 16'd36768;
            target_y <= 16'd32768;
        end else if (point_count == POINT_DIV-1) begin
            // Hold until the preceding coordinate pair has been transmitted.
            if (!dac_busy) begin
                point_count <= 0;
                phase <= next_phase;
                target_x <= circle_code(next_cos_phase);
                target_y <= circle_code(next_phase);
            end
        end else point_count <= point_count + 1'b1;
    end
endmodule
