`timescale 1ns / 1ps

// Streaming 1280x720 video processor.
// Mode 0: live RGB, mode 1: grayscale, mode 2: binary, mode 3: Sobel edge.
// Mode 4 remains RGB and is augmented by the following target tracker.
// Mode changes are committed during vertical blanking so a frame never mixes modes.
module contest_video_processor #(
    parameter integer IMAGE_WIDTH = 1280,
    parameter [7:0] BINARY_THRESHOLD = 8'd128
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_vsync,
    input  wire        in_hsync,
    input  wire        in_de,
    input  wire [23:0] in_rgb,
    input  wire [2:0]  requested_mode,
    output reg         out_vsync,
    output reg         out_hsync,
    output reg         out_de,
    output reg  [23:0] out_rgb,
    output reg         out_edge,       // A20: Sobel edge flag (magnitude >= EDGE_FLAG_THRESHOLD) in all modes, out_rgb timing
    output wire [2:0]  active_mode
);
    localparam [7:0] EDGE_FLAG_THRESHOLD = 8'd64;

    // A28 binary mode: isodata threshold.  Each frame accumulates sum/count of
    // the central-window pixels below and above the current threshold; in
    // vertical blanking two serial dividers give the two class means and the
    // threshold moves a quarter of the way toward their midpoint (close to
    // Otsu, no histogram).  Applied to the 3x3 mean of the gray window, then
    // cleaned by a 3x3 majority filter.
    reg  [26:0] sum_lo, sum_hi;     // 2^19 samples * 255 < 2^27
    reg  [19:0] cnt_lo, cnt_hi;
    reg  [7:0]  bin_thr;
    reg  [2:0]  thr_state;          // 0 idle, 1 dividing, 2 update, 3 done for this blank
    reg         div_start;
    wire [26:0] q_lo, q_hi;
    wire        d_hi_done;
    reg  [26:0] n_lo, n_hi;
    reg  [19:0] c_lo, c_hi;
    // A37: one divider shared by the two class means (lo first, then hi)
    reg         div_sel;                // 0: dividing lo, 1: dividing hi
    reg  [26:0] q_lo_r;
    wire [26:0] q_div;
    wire        d_div_done;
    contest_div_seq #(.NW(27), .DW(20)) u_div (.clk(clk), .start(div_start),
        .num(div_sel ? n_hi : n_lo), .den(div_sel ? c_hi : c_lo), .quo(q_div), .done(d_div_done));
    assign q_lo = q_lo_r;
    assign q_hi = q_div;
    assign d_hi_done = d_div_done & div_sel;
    wire        acc_window = in_de && (pixel_x >= 12'd128) && (pixel_x < 12'd1152) &&
                             (pixel_y >= 12'd104) && (pixel_y < 12'd616);
    // A28 edge mode: noise floor = 2 * previous frame's mean Sobel magnitude
    // (clamped 8..96) is subtracted and the remainder amplified 3x, so real
    // edges show as bright lines and sensor grain goes black.
    reg  [26:0] mag_acc;
    reg  [7:0]  edge_floor;
    reg         acc_window_a, acc_window_b;   // acc_window delayed to the stage where edge_level exists

    wire [7:0] red_now   = in_rgb[23:16];
    wire [7:0] green_now = in_rgb[15:8];
    wire [7:0] blue_now  = in_rgb[7:0];
    // (5R + 9G + 2B) / 16; shift/add implementation, coefficients sum to one.
    wire [11:0] gray_sum = {2'b00,red_now,2'b00} + {4'b0000,red_now} +
                           {1'b0,green_now,3'b000} + {4'b0000,green_now} +
                           {3'b000,blue_now,1'b0};
    wire [7:0] gray_now = gray_sum[11:4];

    reg [2:0] mode_latched;
    assign active_mode = mode_latched;

    reg [11:0] pixel_x;
    reg [11:0] pixel_y;
    reg        de_previous;

    // Two read-first line memories hold y-1 and y-2. They are intentionally
    // left unreset; the first two rows are masked while valid history is built.
    (* ram_style = "bram" *) reg [7:0] previous_line [0:IMAGE_WIDTH-1];
    (* ram_style = "bram" *) reg [7:0] two_rows_back [0:IMAGE_WIDTH-1];
    reg [7:0]  previous_q;
    reg [7:0]  two_rows_q;
    reg [11:0] px_nr;                  // pixel_x without async reset (block RAM read address)
    reg [11:0] px_d1;
    reg        de_d1;
    reg [7:0]  gray_d1;
    always @(posedge clk) begin
        if (in_vsync || !in_de) px_nr <= 12'd0; else px_nr <= px_nr + 12'd1;
        // A36: reads and writes live in this reset-free block so TD maps the
        // memories to block RAM (with reset on the output register they had
        // been falling back to distributed RAM).  Same timing as before.
        previous_q <= previous_line[px_nr];
        two_rows_q <= two_rows_back[px_nr];
        px_d1 <= px_nr; de_d1 <= in_de; gray_d1 <= gray_now;
        if (de_d1) begin
            previous_line[px_d1] <= gray_d1;      // row y   at column x-1
            two_rows_back[px_d1] <= previous_q;   // row y-1 at column x-1 (read one clock ago)
        end
    end
    reg [7:0] current_q;
    reg [11:0] sample_x_q;
    reg        window_valid_q;

    reg [7:0] top_left, top_center;
    reg [7:0] mid_left, mid_center;
    reg [7:0] bot_left, bot_center;

    // Pipeline A: original/gray/binary values and timing.
    reg        vs_a, hs_a, de_a;
    reg [23:0] rgb_a;
    reg [7:0]  gray_a;
    reg [2:0]  mode_a;

    // Pipeline B: Sobel positive and negative sums, plus matched bypass paths.
    reg [10:0] gx_pos_b, gx_neg_b;
    reg [10:0] gy_pos_b, gy_neg_b;
    reg        edge_valid_b;
    reg        vs_b, hs_b, de_b;
    reg [23:0] rgb_b;
    reg [7:0]  gray_b;
    reg [7:0]  binary_b;
    reg [2:0]  mode_b;

    wire [10:0] gx_abs = (gx_pos_b >= gx_neg_b) ?
                          (gx_pos_b - gx_neg_b) : (gx_neg_b - gx_pos_b);
    wire [10:0] gy_abs = (gy_pos_b >= gy_neg_b) ?
                          (gy_pos_b - gy_neg_b) : (gy_neg_b - gy_pos_b);
    wire [11:0] edge_sum = gx_abs + gy_abs;
    wire [7:0] edge_level = (|edge_sum[11:8]) ? 8'hff : edge_sum[7:0];

    function [7:0] clamp_thr;
        input [10:0] v;
        begin clamp_thr = (v < 11'd24) ? 8'd24 : (v > 11'd232) ? 8'd232 : v[7:0]; end
    endfunction
    // midpoint of the two class means; an empty class (den forced to 1, sum 0) uses bin_thr
    wire [8:0]  mean_lo_w = (n_lo == 27'd0) ? {1'b0, bin_thr} : {1'b0, q_lo[7:0]};
    wire [8:0]  mean_hi_w = (n_hi == 27'd0) ? {1'b0, bin_thr} : {1'b0, q_hi[7:0]};
    wire [9:0]  iso_mid   = ({1'b0, mean_lo_w} + {1'b0, mean_hi_w}) >> 1;

    // Pipeline C (A28): registered edge level so the display maths has its own stage.
    reg        vs_c, hs_c, de_c, edge_valid_c, bin_c;
    reg [23:0] rgb_c;
    reg [7:0]  gray_c, edge_c;
    reg [2:0]  mode_c;
    wire       bin_maj;                      // majority-filtered binary (3 clocks + 1 line behind bin_c)
    contest_morph3x3 #(.MODE(2)) u_bin_majority (.clk(clk), .in_de(de_b), .in_bit(binary_b[0]), .out_bit(bin_maj));
    wire [7:0]  edge_diff = (edge_c > edge_floor) ? (edge_c - edge_floor) : 8'd0;
    wire [9:0]  edge_x3   = {edge_diff, 1'b0} + {2'b0, edge_diff};
    wire [7:0]  edge_disp = (|edge_x3[9:8]) ? 8'hff : edge_x3[7:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode_latched <= 3'd0;
            pixel_x <= 12'd0;
            pixel_y <= 12'd0;
            de_previous <= 1'b0;
            current_q <= 8'd0;
            sample_x_q <= 12'd0;
            window_valid_q <= 1'b0;
            top_left <= 8'd0; top_center <= 8'd0;
            mid_left <= 8'd0; mid_center <= 8'd0;
            bot_left <= 8'd0; bot_center <= 8'd0;
            vs_a <= 1'b0; hs_a <= 1'b0; de_a <= 1'b0;
            rgb_a <= 24'd0; gray_a <= 8'd0; mode_a <= 3'd0;
            sum_lo <= 27'd0; sum_hi <= 27'd0; cnt_lo <= 20'd0; cnt_hi <= 20'd0; bin_thr <= BINARY_THRESHOLD;
            thr_state <= 3'd0; div_start <= 1'b0; div_sel <= 1'b0; q_lo_r <= 27'd0; n_lo <= 27'd0; n_hi <= 27'd0; c_lo <= 20'd1; c_hi <= 20'd1;
            mag_acc <= 27'd0; edge_floor <= 8'd16; acc_window_a <= 1'b0; acc_window_b <= 1'b0;
            vs_c <= 1'b0; hs_c <= 1'b0; de_c <= 1'b0; rgb_c <= 24'd0; gray_c <= 8'd0; edge_c <= 8'd0; edge_valid_c <= 1'b0; mode_c <= 3'd0; bin_c <= 1'b0;
            gx_pos_b <= 11'd0; gx_neg_b <= 11'd0;
            gy_pos_b <= 11'd0; gy_neg_b <= 11'd0;
            edge_valid_b <= 1'b0;
            vs_b <= 1'b0; hs_b <= 1'b0; de_b <= 1'b0;
            rgb_b <= 24'd0; gray_b <= 8'd0; binary_b <= 8'd0; mode_b <= 3'd0;
            out_vsync <= 1'b0; out_hsync <= 1'b0; out_de <= 1'b0;
            out_rgb <= 24'd0; out_edge <= 1'b0;
        end else begin
            // Counters describe the pixel presented at this clock edge.
            de_previous <= in_de;
            if (in_vsync) begin
                pixel_x <= 12'd0;
                pixel_y <= 12'd0;
                mode_latched <= (requested_mode <= 3'd4) ? requested_mode : 3'd0;
            end else begin
                if (!in_de)
                    pixel_x <= 12'd0;
                else
                    pixel_x <= pixel_x + 12'd1;
                if (!in_de && de_previous)
                    pixel_y <= pixel_y + 12'd1;
            end

            // A28: isodata threshold sequence in vertical blanking
            div_start <= 1'b0;
            if (in_vsync) begin
                case (thr_state)
                    3'd0: begin
                        if (cnt_lo != 20'd0 || cnt_hi != 20'd0) begin
                            n_lo <= sum_lo; n_hi <= sum_hi;
                            c_lo <= (cnt_lo == 20'd0) ? 20'd1 : cnt_lo;
                            c_hi <= (cnt_hi == 20'd0) ? 20'd1 : cnt_hi;
                            div_start <= 1'b1; div_sel <= 1'b0; thr_state <= 3'd1;
                            sum_lo <= 27'd0; sum_hi <= 27'd0; cnt_lo <= 20'd0; cnt_hi <= 20'd0;
                        end
                    end
                    3'd1: if (d_div_done) begin                  // lo mean ready -> divide hi
                        q_lo_r <= q_div; div_sel <= 1'b1; div_start <= 1'b1; thr_state <= 3'd4;
                    end
                    3'd4: if (d_div_done) thr_state <= 3'd2;     // hi mean ready
                    3'd2: begin
                        // midpoint of the two class means (an empty class contributes the
                        // current threshold), quarter-step IIR toward it, clamp
                        bin_thr <= clamp_thr(({1'b0, bin_thr, 1'b0} + {2'b0, bin_thr} + iso_mid) >> 2);
                        thr_state <= 3'd3;
                    end
                    default: ;
                endcase
                if (mag_acc != 27'd0) begin
                    edge_floor <= (mag_acc[25:19] < 7'd4)  ? 8'd8 :
                                  (mag_acc[25:19] > 7'd48) ? 8'd96 : {mag_acc[25:19], 1'b0};   // 2 * mean magnitude
                    mag_acc <= 27'd0;
                end
            end else begin
                thr_state <= 3'd0;
                if (acc_window) begin
                    if (gray_now < bin_thr) begin sum_lo <= sum_lo + {19'd0, gray_now}; cnt_lo <= cnt_lo + 20'd1; end
                    else                    begin sum_hi <= sum_hi + {19'd0, gray_now}; cnt_hi <= cnt_hi + 20'd1; end
                end
                if (acc_window_b) mag_acc <= mag_acc + {19'd0, edge_level};
            end
            acc_window_b <= acc_window_a;
            acc_window_a <= acc_window;

            current_q <= gray_now;
            sample_x_q <= pixel_x;
            window_valid_q <= in_de && (pixel_x >= 12'd2) && (pixel_y >= 12'd2);

            vs_a <= in_vsync;
            hs_a <= in_hsync;
            de_a <= in_de;
            rgb_a <= in_rgb;
            gray_a <= gray_now;
            mode_a <= mode_latched;

            // The memory outputs and current_q belong to the same input pixel.
            if (de_a) begin
                if (sample_x_q == 12'd0) begin
                    top_left <= 8'd0; top_center <= two_rows_q;
                    mid_left <= 8'd0; mid_center <= previous_q;
                    bot_left <= 8'd0; bot_center <= current_q;
                end else begin
                    top_left <= top_center; top_center <= two_rows_q;
                    mid_left <= mid_center; mid_center <= previous_q;
                    bot_left <= bot_center; bot_center <= current_q;
                end
            end

            gx_pos_b <= {3'b000,two_rows_q} + {2'b00,previous_q,1'b0} + {3'b000,current_q};
            gx_neg_b <= {3'b000,top_left} + {2'b00,mid_left,1'b0} + {3'b000,bot_left};
            gy_pos_b <= {3'b000,bot_left} + {2'b00,bot_center,1'b0} + {3'b000,current_q};
            gy_neg_b <= {3'b000,top_left} + {2'b00,top_center,1'b0} + {3'b000,two_rows_q};
            edge_valid_b <= window_valid_q;
            vs_b <= vs_a;
            hs_b <= hs_a;
            de_b <= de_a;
            rgb_b <= rgb_a;
            gray_b <= gray_a;
            // 3x3 mean vs threshold: sum9 >= 9*thr  (window columns x-2..x, rows y-2..y)
            binary_b <= (window_valid_q &&
                         ({4'd0,top_left} + {4'd0,top_center} + {4'd0,two_rows_q} +
                          {4'd0,mid_left} + {4'd0,mid_center} + {4'd0,previous_q} +
                          {4'd0,bot_left} + {4'd0,bot_center} + {4'd0,current_q})
                         >= ({4'd0,bin_thr,3'b0} + {4'd0,bin_thr})) ? 8'hff : 8'h00;
            mode_b <= mode_a;

            vs_c <= vs_b; hs_c <= hs_b; de_c <= de_b; rgb_c <= rgb_b; gray_c <= gray_b; mode_c <= mode_b;
            edge_valid_c <= edge_valid_b; edge_c <= edge_level; bin_c <= binary_b[0];

            out_vsync <= vs_c;
            out_hsync <= hs_c;
            out_de <= de_c;
            out_edge <= de_c && edge_valid_c && (edge_c >= EDGE_FLAG_THRESHOLD);
            case (mode_c)
                3'd1: out_rgb <= {gray_c, gray_c, gray_c};
                3'd2: out_rgb <= bin_maj ? 24'hFFFFFF : 24'h000000;
                3'd3: out_rgb <= edge_valid_c ? {edge_disp, edge_disp, edge_disp} : 24'd0;
                default: out_rgb <= rgb_c;
            endcase
        end
    end
endmodule
