`timescale 1ns / 1ps

// A14: picture-in-picture magnifier for the tracked target.
//
// While TRACK mode has a valid target, a WIN_W x WIN_H window in the top-right
// corner shows the area around the target centre magnified 2x or 4x.  The
// source region is captured from the un-annotated video of frame N into an
// on-chip RGB565 buffer and displayed during frame N+1, so no DDR access is
// added and the capture region may lie anywhere in the frame, including under
// the window itself (two buffer banks alternate).  Nearest-neighbour scaling.
//
// Buffer: 2 banks x 160 x 90 x 16 bit.  2x uses the full 160x90 source
// region; 4x uses 80x45 with the same row stride.  Zoom is chosen per frame:
// automatic (4x when the target box fits in 80x45, else 2x) or forced.
module contest_pip #(
    parameter integer IMAGE_WIDTH  = 1280,
    parameter integer IMAGE_HEIGHT = 720,
    parameter integer WIN_W        = 320,
    parameter integer WIN_H        = 180,
    parameter integer WIN_X        = 1280 - 320 - 12,   // top-right, 12 px margin
    parameter integer WIN_Y        = 12
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_vsync,
    input  wire        in_hsync,
    input  wire        in_de,
    input  wire [23:0] in_rgb,          // annotated video (boxes drawn)
    input  wire [23:0] clean_rgb,       // same timing, without annotations
    input  wire        enable,          // show window when a target is valid
    input  wire [1:0]  zoom_sel,        // 0 auto, 1 force 2x, 2 force 4x
    input  wire        target_valid,
    input  wire [11:0] target_center_x,
    input  wire [11:0] target_center_y,
    input  wire [11:0] target_width,
    input  wire [11:0] target_height,
    output reg         out_vsync,
    output reg         out_hsync,
    output reg         out_de,
    output reg  [23:0] out_rgb
);
    localparam integer SRC_W = WIN_W / 2;     // 160: source columns stored per row
    localparam integer SRC_H = WIN_H / 2;     // 90
    localparam integer BANK  = SRC_W * SRC_H; // 14400 entries per bank

    // ---------------------------------------------------------------- timing
    reg [11:0] x, y;
    reg        de_d, vsync_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x <= 12'd0; y <= 12'd0; de_d <= 1'b0; vsync_d <= 1'b0;
        end else begin
            de_d    <= in_de;
            vsync_d <= in_vsync;
            if (in_vsync) begin
                x <= 12'd0; y <= 12'd0;
            end else if (in_de) begin
                x <= de_d ? x + 12'd1 : 12'd0;
            end else if (de_d) begin
                y <= y + 12'd1;
            end
        end
    end
    wire frame_start = in_vsync & ~vsync_d;

    // ------------------------------------------------ per-frame capture setup
    reg        wbank;                 // bank being written this frame
    reg        cap_zoom4;             // this frame captures an 80x45 region
    reg [11:0] cap_x0, cap_y0;        // capture origin
    reg [7:0]  cap_w, cap_h;          // 160x90 or 80x45
    reg        cap_valid;             // a target existed when the frame started
    // read side (previous frame's capture)
    reg        rd_zoom4, rd_valid;

    wire auto_zoom4 = (target_width <= 12'd80) && (target_height <= 12'd45);
    wire want_zoom4 = (zoom_sel == 2'd2) || ((zoom_sel == 2'd0) && auto_zoom4);
    wire [7:0]  nxt_w = want_zoom4 ? 8'd80 : 8'd160;
    wire [7:0]  nxt_h = want_zoom4 ? 8'd45 : 8'd90;
    wire [11:0] half_w = {5'd0, nxt_w[7:1]};
    wire [11:0] half_h = {5'd0, nxt_h[7:1]};
    wire [11:0] max_x0 = IMAGE_WIDTH  - {4'd0, nxt_w};
    wire [11:0] max_y0 = IMAGE_HEIGHT - {4'd0, nxt_h};
    wire [11:0] nxt_x0 = (target_center_x < half_w) ? 12'd0 :
                         ((target_center_x - half_w) > max_x0) ? max_x0 : (target_center_x - half_w);
    wire [11:0] nxt_y0 = (target_center_y < half_h) ? 12'd0 :
                         ((target_center_y - half_h) > max_y0) ? max_y0 : (target_center_y - half_h);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wbank <= 1'b0; cap_zoom4 <= 1'b0; cap_x0 <= 12'd0; cap_y0 <= 12'd0;
            cap_w <= 8'd160; cap_h <= 8'd90; cap_valid <= 1'b0;
            rd_zoom4 <= 1'b0; rd_valid <= 1'b0;
        end else if (frame_start) begin
            // Hand the bank just written to the display side, start a new capture.
            rd_zoom4  <= cap_zoom4;
            rd_valid  <= cap_valid;
            wbank     <= ~wbank;
            cap_zoom4 <= want_zoom4;
            cap_w     <= nxt_w;
            cap_h     <= nxt_h;
            cap_x0    <= nxt_x0;
            cap_y0    <= nxt_y0;
            cap_valid <= target_valid & enable;
        end
    end
    wire rbank = ~wbank;

    // --------------------------------------------------------------- capture
    wire in_cap = in_de && cap_valid &&
                  (x >= cap_x0) && (x < cap_x0 + {4'd0, cap_w}) &&
                  (y >= cap_y0) && (y < cap_y0 + {4'd0, cap_h});
    wire [7:0]  cap_col = x[7:0] - cap_x0[7:0];      // < 160, wraps safely
    wire [7:0]  cap_row = y[7:0] - cap_y0[7:0];      // < 90
    wire [14:0] wr_addr = (wbank ? BANK : 0) + ({7'd0, cap_row} << 7) + ({7'd0, cap_row} << 5) + {7'd0, cap_col};
    wire [15:0] wr_data = {clean_rgb[23:19], clean_rgb[15:10], clean_rgb[7:3]};   // RGB565

    (* ram_style = "bram" *) reg [15:0] pip_buf [0:2*BANK-1];
    always @(posedge clk) begin
        if (in_cap) pip_buf[wr_addr] <= wr_data;
    end

    // --------------------------------------------------------------- display
    localparam [8:0] WIN_X_LO = WIN_X % 512;
    localparam [7:0] WIN_Y_LO = WIN_Y % 256;
    wire in_win = (x >= WIN_X) && (x < WIN_X + WIN_W) && (y >= WIN_Y) && (y < WIN_Y + WIN_H);
    wire [8:0] px = x[8:0] - WIN_X_LO;               // 0..319 (modulo arithmetic is exact inside the window)
    wire [7:0] py = y[7:0] - WIN_Y_LO;               // 0..179
    wire [7:0] sx = rd_zoom4 ? {1'b0, px[8:2]} : px[8:1];   // 0..79 or 0..159
    wire [7:0] sy = rd_zoom4 ? {2'b0, py[7:2]} : {1'b0, py[7:1]};   // 0..44 or 0..89
    wire [14:0] rd_addr = (rbank ? BANK : 0) + ({7'd0, sy} << 7) + ({7'd0, sy} << 5) + {7'd0, sx};
    reg  [15:0] rd_data;
    always @(posedge clk) begin
        rd_data <= pip_buf[rd_addr];
    end

    // stage 1 registers (RAM latency), stage 2 output
    reg        s1_vsync, s1_hsync, s1_de, s1_win, s1_border, s1_show;
    reg [23:0] s1_rgb;
    wire border = in_win && ((px < 9'd2) || (px >= WIN_W - 2) || (py < 8'd2) || (py >= WIN_H - 2));
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_vsync <= 1'b0; s1_hsync <= 1'b0; s1_de <= 1'b0; s1_win <= 1'b0; s1_border <= 1'b0; s1_show <= 1'b0;
            s1_rgb <= 24'd0;
            out_vsync <= 1'b0; out_hsync <= 1'b0; out_de <= 1'b0; out_rgb <= 24'd0;
        end else begin
            s1_vsync  <= in_vsync;
            s1_hsync  <= in_hsync;
            s1_de     <= in_de;
            s1_rgb    <= in_rgb;
            s1_win    <= in_win;
            s1_border <= border;
            s1_show   <= rd_valid & enable;

            out_vsync <= s1_vsync;
            out_hsync <= s1_hsync;
            out_de    <= s1_de;
            if (s1_de && s1_show && s1_border)
                out_rgb <= 24'hF0F0F0;
            else if (s1_de && s1_show && s1_win)
                out_rgb <= {rd_data[15:11], rd_data[15:13], rd_data[10:5], rd_data[10:9], rd_data[4:0], rd_data[4:2]};
            else
                out_rgb <= s1_rgb;
        end
    end
endmodule
