// contest_morph3x3 : 3x3 binary filter on a 1-bit raster stream.
//   MODE 0 dilation (any of 9), 1 erosion (all 9), 2 majority (at least 5 of 9),
//   3 despeckle (centre set and at least 3 of 9 set: keeps 1-px lines, drops dots).
// Majority removes isolated noise like erosion but does not thin porous
// objects (A27).  Two 1-bit line buffers hold the previous two
// rows; a 3-column shift register forms the window.
//
// Timing: the output for the window centred on (x-1, y-1) is registered
// two clocks after the input pixel (x, y) is presented, so relative to a
// pixel counter that advances with in_de the result is offset by
//   dx = 3 columns, dy = 1 row.
// The consumer subtracts these (contest_target_tracker: MORPH_DX/MORPH_DY).
// Row wrap: the first columns of a row see the previous row's last columns
// in the window; the caller already ignores a border margin.
module contest_morph3x3 #(
    parameter MODE       = 2,
    parameter ADDR_BITS  = 11          // line buffer depth 2048 >= 1280
) (
    input  wire clk,
    input  wire in_de,
    input  wire in_bit,
    output reg  out_bit
);
    reg [ADDR_BITS-1:0] x;             // column of the input pixel
    reg [ADDR_BITS-1:0] x_d1;
    reg                 de_d1, b_d1;
    reg                 q1, q2;         // row y-1 / row y-2 at column x
    reg                 lb1 [0:(1<<ADDR_BITS)-1];
    reg                 lb2 [0:(1<<ADDR_BITS)-1];
    reg [2:0]           r0, r1, r2;     // columns x, x-1, x-2 of rows y, y-1, y-2
    wire [3:0]          pop9 = {3'd0,r0[0]} + {3'd0,r0[1]} + {3'd0,r0[2]} +
                               {3'd0,r1[0]} + {3'd0,r1[1]} + {3'd0,r1[2]} +
                               {3'd0,r2[0]} + {3'd0,r2[1]} + {3'd0,r2[2]};

    always @(posedge clk) begin
        // read the two previous rows at column x, write column x-1 one clock later
        if (in_de) begin
            q1 <= lb1[x];
            q2 <= lb2[x];
            x  <= x + 1'b1;
        end else begin
            x  <= {ADDR_BITS{1'b0}};
        end
        de_d1 <= in_de;
        b_d1  <= in_bit;
        x_d1  <= x;
        if (de_d1) begin
            lb1[x_d1] <= b_d1;          // row y   -> becomes "row y-1" for the next row
            lb2[x_d1] <= q1;            // row y-1 -> becomes "row y-2" for the next row
            r0 <= {r0[1:0], b_d1};
            r1 <= {r1[1:0], q1};
            r2 <= {r2[1:0], q2};
        end
        case (MODE)
            0:       out_bit <= (|r0 | |r1 | |r2);
            1:       out_bit <= (&r0 & &r1 & &r2);
            3:       out_bit <= r1[1] && (pop9 >= 4'd3);
            default: out_bit <= (pop9 >= 4'd5);
        endcase
    end
endmodule
