// 1920x1080 RAW10 Bayer to 1280x720 RAW10 Bayer scaler.
//
// Horizontal conversion is phase-aware: only samples belonging to the same
// Bayer colour are averaged before the 3:2 decimation.  The old implementation
// simply kept pixels 0,1,4,5 out of every six.  That created strong stair-steps,
// moire and false-colour blocks on diagonal and repetitive detail.
//
// Vertical conversion uses two read-before-write line stores.  Rows of the
// same Bayer phase are averaged before the 3:2 decimation, removing the old
// 0,1,4,5 hard row-drop pattern. Pixel 0 is [9:0].
module raw10_downscale_3to2 (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_frame_start,
    input  wire        I_frame_end,
    input  wire        I_valid,
    input  wire [39:0] I_data,
    output reg         O_frame_start,
    output reg         O_frame_end,
    output reg         O_valid,
    output reg  [39:0] O_data
);
    localparam INPUT_WORDS_PER_LINE = 480;
    reg [8:0]  input_word;
    reg [1:0]  word_mod3;
    reg [39:0] previous_word;

    // Two rows are sufficient because Bayer phase repeats every two lines.
    // Each store is 512 x 40 and is forced into one PH1P ERAM20K.
    reg [8:0]  vertical_word;
    reg [2:0]  vertical_row_mod6;
    reg [39:0] vertical_current;
    reg        vertical_even;
    reg        vertical_valid;
    wire [39:0] even_line_q;
    wire [39:0] odd_line_q;

    raw10_line_eram u_even_line (
        .clk  (I_clk),
        .we   (I_valid && (vertical_row_mod6[0] == 1'b0)),
        .addr (vertical_word),
        .din  (I_data),
        .dout (even_line_q)
    );

    raw10_line_eram u_odd_line (
        .clk  (I_clk),
        .we   (I_valid && (vertical_row_mod6[0] == 1'b1)),
        .addr (vertical_word),
        .din  (I_data),
        .dout (odd_line_q)
    );

    function [9:0] avg_same_colour;
        input [9:0] a;
        input [9:0] b;
        reg [10:0] sum;
        begin
            sum = {1'b0,a} + {1'b0,b} + 11'd1;
            avg_same_colour = sum[10:1];
        end
    endfunction

    wire [39:0] vertical_filtered = {
        avg_same_colour(vertical_even ? even_line_q[39:30] : odd_line_q[39:30], vertical_current[39:30]),
        avg_same_colour(vertical_even ? even_line_q[29:20] : odd_line_q[29:20], vertical_current[29:20]),
        avg_same_colour(vertical_even ? even_line_q[19:10] : odd_line_q[19:10], vertical_current[19:10]),
        avg_same_colour(vertical_even ? even_line_q[9:0]   : odd_line_q[9:0],   vertical_current[9:0])
    };

    // Read the previous row of identical Bayer phase and replace it with the
    // current row. Rows 0/1 of each six-row group prime the memories; rows
    // 2/3 and 4/5 generate the four filtered output rows.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            vertical_word     <= 9'd0;
            vertical_row_mod6 <= 3'd0;
            vertical_current  <= 40'd0;
            vertical_even     <= 1'b0;
            vertical_valid    <= 1'b0;
        end else begin
            vertical_valid <= 1'b0;
            if(I_frame_start) begin
                vertical_word     <= 9'd0;
                vertical_row_mod6 <= 3'd0;
            end else if(I_valid) begin
                vertical_current <= I_data;
                vertical_even  <= (vertical_row_mod6[0] == 1'b0);
                vertical_valid <= (vertical_row_mod6 >= 3'd2);

                if(vertical_word == INPUT_WORDS_PER_LINE - 1) begin
                    vertical_word <= 9'd0;
                    vertical_row_mod6 <= (vertical_row_mod6 == 3'd5) ?
                                         3'd0 : vertical_row_mod6 + 3'd1;
                end else begin
                    vertical_word <= vertical_word + 9'd1;
                end
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            input_word    <= 9'd0;
            word_mod3     <= 2'd0;
            previous_word <= 40'd0;
            O_frame_start <= 1'b0;
            O_frame_end   <= 1'b0;
            O_valid       <= 1'b0;
            O_data        <= 40'd0;
        end else begin
            O_frame_start <= I_frame_start;
            O_frame_end   <= I_frame_end;
            O_valid       <= 1'b0;

            if(I_frame_start) begin
                input_word <= 9'd0;
                word_mod3  <= 2'd0;
                previous_word <= 40'd0;
            end else if(vertical_valid) begin
                case(word_mod3)
                        // A contains source pixels 0..3.
                        2'd0: previous_word <= vertical_filtered;
                        // B contains pixels 4..7. Generate output pixels 0..3:
                        // avg(0,2), avg(1,3), avg(2,4), avg(3,5).
                        2'd1: begin
                            O_data <= {
                                avg_same_colour(previous_word[39:30], vertical_filtered[19:10]),
                                avg_same_colour(previous_word[29:20], vertical_filtered[9:0]),
                                avg_same_colour(previous_word[19:10], previous_word[39:30]),
                                avg_same_colour(previous_word[9:0],   previous_word[29:20])
                            };
                            O_valid    <= 1'b1;
                            previous_word <= vertical_filtered;
                        end
                        // C contains pixels 8..11. Generate output pixels 4..7:
                        // avg(6,8), avg(7,9), avg(8,10), avg(9,11).
                        2'd2: begin
                            O_data <= {
                                avg_same_colour(vertical_filtered[19:10], vertical_filtered[39:30]),
                                avg_same_colour(vertical_filtered[9:0],   vertical_filtered[29:20]),
                                avg_same_colour(previous_word[39:30], vertical_filtered[19:10]),
                                avg_same_colour(previous_word[29:20], vertical_filtered[9:0])
                            };
                            O_valid <= 1'b1;
                        end
                        default: previous_word <= previous_word;
                endcase

                if(input_word == INPUT_WORDS_PER_LINE - 1) begin
                    input_word <= 9'd0;
                    word_mod3  <= 2'd0;
                end else begin
                    input_word <= input_word + 9'd1;
                    word_mod3  <= (word_mod3 == 2'd2) ? 2'd0 : word_mod3 + 2'd1;
                end
            end
        end
    end
endmodule

// One native 20-Kbit embedded RAM used as a 40-bit by 512-word line store.
// READBEFOREWRITE returns the old sample at an address while the current row
// replaces it, which is exactly the two-line Bayer-phase delay required above.
module raw10_line_eram (
    input  wire        clk,
    input  wire        we,
    input  wire [8:0]  addr,
    input  wire [39:0] din,
    output wire [39:0] dout
);
    wire [39:0] unused_dob;

    PH1P_LOGIC_ERAM #(
        .DATA_WIDTH_A (40),
        .DATA_WIDTH_B (40),
        .ADDR_WIDTH_A (9),
        .ADDR_WIDTH_B (9),
        .DATA_DEPTH_A (512),
        .DATA_DEPTH_B (512),
        .MODE          ("DP"),
        .REGMODE_A     ("NOREG"),
        .REGMODE_B     ("NOREG"),
        .WRITEMODE_A   ("READBEFOREWRITE"),
        .WRITEMODE_B   ("NORMAL"),
        .CLKMODE       ("SYNC"),
        .IMPLEMENT     ("20K(FAST)"),
        .ECC_ENCODE    ("DISABLE"),
        .ECC_DECODE    ("DISABLE"),
        .SSROVERCE     ("DISABLE"),
        .INIT_FILE     ("NONE"),
        .FILL_ALL      ("NONE")
    ) u_eram (
        .dia(din), .addra(addr), .cea(1'b1), .ocea(1'b0),
        .clka(clk), .wea(we), .rsta(1'b0), .bea(1'b0), .doa(dout),
        .dib(40'd0), .addrb(9'd0), .ceb(1'b0), .oceb(1'b0),
        .clkb(clk), .web(1'b0), .rstb(1'b0), .beb(1'b0), .dob(unused_dob),
        .ecc_sbiterrinj(1'b0), .ecc_dbiterrinj(1'b0),
        .ecc_sbiterr(), .ecc_dbiterr()
    );
endmodule
