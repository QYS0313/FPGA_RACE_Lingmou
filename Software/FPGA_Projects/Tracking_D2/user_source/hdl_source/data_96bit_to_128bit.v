

module data_96bit_to_128bit (
    input wire        I_clk,
    input wire        I_rst_n,
   
    input wire        I_96b_frame_start,
    input wire        I_96b_valid,
    input wire[95:0]  I_96b_data,
  
    output reg        O_128b_frame_start,
    output reg        O_128b_valid,
    output reg[127:0] O_128b_data
);



    reg[95:0]  S_96b_data_1d;
    reg[1:0]   S_cnt;


    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_96b_data_1d <= 96'd0;
            S_cnt <= 2'd0;
            O_128b_frame_start <= 1'b0;
            O_128b_valid <= 1'b0;
            O_128b_data <= 128'd0;
        end else begin
            O_128b_frame_start <= I_96b_frame_start;
            O_128b_valid <= 1'b0;
            if(I_96b_frame_start)
                S_cnt <= 2'd0;
            if(I_96b_valid) begin
                S_96b_data_1d <= I_96b_data;
                S_cnt <= S_cnt + 2'd1;
                case(S_cnt)
                    2'd1: begin O_128b_valid <= 1'b1; O_128b_data <= {S_96b_data_1d,I_96b_data[95:64]}; end
                    2'd2: begin O_128b_valid <= 1'b1; O_128b_data <= {S_96b_data_1d[63:0],I_96b_data[95:32]}; end
                    2'd3: begin O_128b_valid <= 1'b1; O_128b_data <= {S_96b_data_1d[31:0],I_96b_data}; end
                    default: O_128b_data <= 128'd0;
                endcase
            end
        end
    end



    

endmodule
