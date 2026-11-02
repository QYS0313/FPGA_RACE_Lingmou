`timescale 1ns / 1ps
// contest_shading (A37): radial colour-shading correction.
//
// Measured on plain white walls (A35): the picture centre has G about 8%
// above R and B while the edges are neutral to slightly red, the optical
// centre sitting near x=760, y=360.  The green excess follows
//   excess(r) ~ -S + K * r^2
// so the green channel is multiplied by
//   gain(r) = 256 - S + min((r^2 * K) >> 16, 2S)      (units of 1/256)
// i.e. 0.92 at the centre rising to 1.08 at the edges for S = 20.  The
// auto white balance then makes the whole frame neutral.  K = 0 disables
// the correction (gain 256 everywhere).  K and S are host-tunable.
//
// Five-clock pipeline; the video timing is delayed by the same five clocks.
module contest_shading #(
    parameter integer CENTRE_X = 760,
    parameter integer CENTRE_Y = 360
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_vsync,
    input  wire        in_hsync,
    input  wire        in_de,
    input  wire [23:0] in_rgb,
    input  wire [7:0]  k_gain,          // slope, default 4 (A38 calibration); 0 = off
    input  wire [7:0]  s_att,           // centre attenuation, default 20
    output reg         out_vsync,
    output reg         out_hsync,
    output reg         out_de,
    output reg  [23:0] out_rgb
);
    // ---------------------------------------------------------------- counters
    reg [11:0] x, y;
    reg        de_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x <= 12'd0; y <= 12'd0; de_d <= 1'b0;
        end else begin
            de_d <= in_de;
            if (in_vsync) begin
                x <= 12'd0; y <= 12'd0;
            end else if (in_de) begin
                x <= de_d ? x + 12'd1 : 12'd0;
            end else if (de_d) begin
                y <= y + 12'd1;
            end
        end
    end
    wire signed [12:0] dx = $signed({1'b0, x}) - CENTRE_X;
    wire signed [12:0] dy = $signed({1'b0, y}) - CENTRE_Y;

    // ---------------------------------------------------------------- pipeline
    reg  [4:0]  p_vsync, p_hsync, p_de;
    reg  [23:0] p_rgb [0:4];
    reg  signed [25:0] dx2, dy2;
    reg  [19:0] r2;
    reg  [27:0] prod;
    reg  [8:0]  gain;
    wire [11:0] delta   = prod[27:16];
    wire [8:0]  s2      = {s_att, 1'b0};
    wire [16:0] g_mul   = p_rgb[4][15:8] * gain;          // G * gain, 8x9 bits
    wire [7:0]  g_new   = (|g_mul[16:16]) ? 8'hff : g_mul[15:8];   // >> 8, saturate
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_vsync <= 5'd0; p_hsync <= 5'd0; p_de <= 5'd0;
            for (i = 0; i < 5; i = i + 1) p_rgb[i] <= 24'd0;
            dx2 <= 26'd0; dy2 <= 26'd0; r2 <= 20'd0; prod <= 28'd0; gain <= 9'd256;
            out_vsync <= 1'b0; out_hsync <= 1'b0; out_de <= 1'b0; out_rgb <= 24'd0;
        end else begin
            p_vsync <= {p_vsync[3:0], in_vsync};
            p_hsync <= {p_hsync[3:0], in_hsync};
            p_de    <= {p_de[3:0],    in_de};
            p_rgb[0] <= in_rgb;
            for (i = 1; i < 5; i = i + 1) p_rgb[i] <= p_rgb[i-1];
            dx2  <= dx * dx;                                   // stage 1
            dy2  <= dy * dy;
            r2   <= dx2[19:0] + dy2[19:0];                     // stage 2 (max ~707k < 2^20)
            prod <= r2 * k_gain;                               // stage 3
            gain <= (k_gain == 8'd0) ? 9'd256 :                // stage 4
                    (9'd256 - {1'b0, s_att}) + ((delta > {3'b0, s2}) ? s2 : delta[8:0]);
            out_vsync <= p_vsync[4]; out_hsync <= p_hsync[4]; out_de <= p_de[4];   // stage 5
            out_rgb   <= {p_rgb[4][23:16], g_new, p_rgb[4][7:0]};
        end
    end
endmodule
