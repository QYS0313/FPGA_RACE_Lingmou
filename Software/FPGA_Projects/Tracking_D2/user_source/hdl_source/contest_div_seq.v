// contest_div_seq : small restoring integer divider, one quotient bit per
// clock (NW clocks).  Used once per frame in vertical blanking, so area
// matters more than speed.  den == 0 must be guarded by the caller.
module contest_div_seq #(
    parameter integer NW = 27,     // numerator / quotient width
    parameter integer DW = 20      // denominator width
) (
    input  wire          clk,
    input  wire          start,
    input  wire [NW-1:0] num,
    input  wire [DW-1:0] den,
    output reg  [NW-1:0] quo,
    output reg           done
);
    reg          busy;
    reg [5:0]    cnt;
    reg [DW:0]   rem;
    reg [NW-1:0] q;
    reg [DW-1:0] d;
    wire [DW:0]  rem_sh = {rem[DW-1:0], q[NW-1]};

    wire         qbit   = (rem_sh >= {1'b0, d});

    always @(posedge clk) begin
        done <= 1'b0;
        if (start) begin
            busy <= 1'b1; cnt <= 6'd0; rem <= {(DW+1){1'b0}}; q <= num; d <= den;
        end else if (busy) begin
            rem <= qbit ? (rem_sh - {1'b0, d}) : rem_sh;
            q   <= {q[NW-2:0], qbit};
            cnt <= cnt + 6'd1;
            if (cnt == NW-1) begin
                busy <= 1'b0; done <= 1'b1;
                quo  <= {q[NW-2:0], qbit};      // valid together with done
            end
        end
    end
endmodule
