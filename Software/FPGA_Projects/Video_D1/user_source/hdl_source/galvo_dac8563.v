`timescale 1ns / 1ps

// DAC8563 dual-axis output. All logic uses clk; SCLK is a registered output.
// DIN changes on SCLK rising edges and is sampled on falling edges.
// Write both input registers (00/01), then pulse LDAC to update together.
module galvo_dac8563 #(
    parameter integer CLK_HZ = 100000000,
    parameter integer SCLK_HZ = 1000000,
    parameter integer STARTUP_US = 1000
) (
    input wire clk,
    input wire rst_n,
    input wire [15:0] target_x,
    input wire [15:0] target_y,
    output reg ready,
    output wire busy,
    output reg [15:0] applied_x,
    output reg [15:0] applied_y,
    output reg dac_clr_n,
    output reg dac_sync_n,
    output reg dac_sclk,
    output reg dac_din,
    output reg dac_ldac_n
);
    localparam integer HALF_DIV = CLK_HZ / (2 * SCLK_HZ);
    localparam integer WAIT_TICKS = (2 * SCLK_HZ / 1000000) * STARTUP_US;
    localparam [3:0] POWER_WAIT=0, DISPATCH=1, RISE=2, FALL=3,
                     FINISH=4, GAP=5, REF_WAIT=6, LDAC_LOW=7, IDLE=8;
    reg [3:0] state;
    reg [$clog2(HALF_DIV)-1:0] divider;
    reg [$clog2(WAIT_TICKS+1)-1:0] wait_count;
    reg [2:0] command_index;
    reg [4:0] bit_index;
    reg [23:0] shift_data;
    reg [15:0] pair_x, pair_y;
    assign busy = state != IDLE;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= POWER_WAIT;
            divider <= 0; wait_count <= 0;
            command_index <= 0; bit_index <= 0; shift_data <= 0;
            pair_x <= 16'h8000; pair_y <= 16'h8000;
            applied_x <= 16'h8000; applied_y <= 16'h8000;
            ready <= 1'b0;
            dac_clr_n <= 1'b0; dac_sync_n <= 1'b1;
            dac_sclk <= 1'b0; dac_din <= 1'b0; dac_ldac_n <= 1'b1;
        end else if (divider == HALF_DIV-1) begin
            divider <= 0;
            case (state)
                POWER_WAIT: begin
                    if (wait_count == WAIT_TICKS-1) begin
                        wait_count <= 0;
                        dac_clr_n <= 1'b1;
                        state <= DISPATCH;
                    end else wait_count <= wait_count + 1'b1;
                end
                DISPATCH: begin
                    case (command_index)
                        0: shift_data <= 24'h280001; // reset all registers
                        1: shift_data <= 24'h200003; // power up A and B
                        2: shift_data <= 24'h380001; // internal 2.5 V reference
                        3: shift_data <= 24'h020000; // gain = 2, both channels
                        4: shift_data <= 24'h300000; // hardware LDAC enabled
                        5: shift_data <= {8'h00, pair_x};
                        6: shift_data <= {8'h01, pair_y};
                        default: shift_data <= 24'h280001;
                    endcase
                    bit_index <= 0;
                    dac_sync_n <= 1'b0;
                    state <= RISE;
                end
                RISE: begin
                    dac_din <= shift_data[23];
                    dac_sclk <= 1'b1;
                    state <= FALL;
                end
                FALL: begin
                    dac_sclk <= 1'b0;
                    shift_data <= {shift_data[22:0], 1'b0};
                    if (bit_index == 23) state <= FINISH;
                    else begin
                        bit_index <= bit_index + 1'b1;
                        state <= RISE;
                    end
                end
                FINISH: begin
                    dac_sync_n <= 1'b1;
                    dac_din <= 1'b0;
                    state <= GAP;
                end
                GAP: begin
                    if (command_index == 4) begin
                        wait_count <= 0;
                        state <= REF_WAIT;
                    end else if (command_index == 6) begin
                        dac_ldac_n <= 1'b0;
                        state <= LDAC_LOW;
                    end else begin
                        command_index <= command_index + 1'b1;
                        state <= DISPATCH;
                    end
                end
                REF_WAIT: begin
                    if (wait_count == WAIT_TICKS-1) begin
                        pair_x <= target_x; pair_y <= target_y;
                        command_index <= 5;
                        state <= DISPATCH;
                    end else wait_count <= wait_count + 1'b1;
                end
                LDAC_LOW: begin
                    dac_ldac_n <= 1'b1;
                    applied_x <= pair_x; applied_y <= pair_y;
                    ready <= 1'b1;
                    state <= IDLE;
                end
                IDLE: begin
                    if ((target_x != applied_x) || (target_y != applied_y)) begin
                        pair_x <= target_x; pair_y <= target_y;
                        command_index <= 5;
                        state <= DISPATCH;
                    end
                end
                default: begin
                    ready <= 1'b0;
                    dac_clr_n <= 1'b0; dac_sync_n <= 1'b1;
                    dac_sclk <= 1'b0; dac_ldac_n <= 1'b1;
                    command_index <= 0; wait_count <= 0;
                    state <= POWER_WAIT;
                end
            endcase
        end else divider <= divider + 1'b1;
    end
endmodule
