

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



    // A5 pixel format: one 128-bit DDR word carries exactly four pixels,
    // each padded to 32 bits as {8'h00, R, G, B}.  A 1280x720 frame is
    // therefore 230400 words (was 172800 with the 3-words-per-16-pixel
    // packing).  The dense packing let any one-word offset in the DDR read
    // FIFO rotate the colour channels of every pixel (128 mod 24 = 8 bits),
    // which was observed on hardware as an intermittent boot-to-boot colour
    // rotation.  With word-aligned pixels an offset can only shift the image
    // by whole pixels; it can never mix channels.  Extra DDR traffic is
    // 33% (about 440 MB/s total against a 2 GB/s DDR2 interface).
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_128b_frame_start <= 1'b0;
            O_128b_valid       <= 1'b0;
            O_128b_data        <= 128'd0;
        end else begin
            O_128b_frame_start <= I_96b_frame_start;
            O_128b_valid       <= I_96b_valid;
            if(I_96b_valid)
                O_128b_data <= {8'd0, I_96b_data[95:72],
                                8'd0, I_96b_data[71:48],
                                8'd0, I_96b_data[47:24],
                                8'd0, I_96b_data[23:0]};
        end
    end

    

endmodule







/*
module data96_128 (

    input wire [95:0] I_tdata,

    output wire [127:0] O_tdata

);

  assign O_tdata = {
    {8'd0}, I_tdata[95:72], {8'd0}, I_tdata[71:48], {8'd0}, I_tdata[47:24], {8'd0}, I_tdata[23:0]
  };
 
endmodule
*/

