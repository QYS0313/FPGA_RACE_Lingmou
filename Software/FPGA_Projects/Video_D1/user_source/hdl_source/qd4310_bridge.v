`timescale 1ns / 1ps

// Host request: A5 axis id command lo hi crc8 (CRC covers all first 6 bytes).
// Host reply:   5A result axis [10 motor feedback bytes] crc8 (14 bytes total).
// result: 0 valid feedback, 1 timeout, 2 bad motor CRC/ID, 3 bad request.
// Only one transaction is outstanding across both axes. Never replay a command.
module qd4310_bridge #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD = 115200,
    parameter integer TIMEOUT_MS = 20
) (
    input wire clk, input wire rst_n,
    input wire host_rx, output wire host_tx,
    input wire pan_rx, output wire pan_tx,
    input wire tilt_rx, output wire tilt_tx,
    input wire auto_active, auto_valid, auto_axis,
    input wire [7:0] auto_command,
    input wire [15:0] auto_value,
    output reg auto_accepted, auto_response,
    output reg [7:0] auto_result,
    output reg auto_enabled,
    output reg [15:0] auto_angle,
    output reg signed [15:0] auto_speed,
    output reg signed [15:0] auto_current,
    output reg home_start, home_axis, home_cancel,
    output reg home_release,home_release_axis,
    input wire [1:0] home_homed,home_fault_axes,
    input wire [7:0] home_fault_code,
    input wire home_fault_axis
);
    function [7:0] crc8;
        input [7:0] prior, value;
        reg [7:0] c;
        integer k;
        begin
            c = prior ^ value;
            for (k=0; k<8; k=k+1)
                c = c[7] ? ((c << 1) ^ 8'h07) : (c << 1);
            crc8 = c;
        end
    endfunction
    wire host_valid, pan_valid, tilt_valid, host_rx_valid;
    wire [7:0] host_data, pan_data, tilt_data, host_rx_data;
    reg host_start, motor_start;
    reg [7:0] host_out, motor_out;
    wire host_busy, pan_busy, tilt_busy;
    reg axis;
    uart_rx_8n1 #(.CLK_FREQ_HZ(CLK_HZ),.UART_BPS(BAUD)) hr
        (.clk(clk),.rst_n(rst_n),.rxd(host_rx),.data_valid(host_rx_valid),.data_out(host_rx_data));
    uart_rx_8n1 #(.CLK_FREQ_HZ(CLK_HZ),.UART_BPS(BAUD)) pr
        (.clk(clk),.rst_n(rst_n),.rxd(pan_rx),.data_valid(pan_valid),.data_out(pan_data));
    uart_rx_8n1 #(.CLK_FREQ_HZ(CLK_HZ),.UART_BPS(BAUD)) tr
        (.clk(clk),.rst_n(rst_n),.rxd(tilt_rx),.data_valid(tilt_valid),.data_out(tilt_data));
    uart_tx_8n1 #(.CLK_FREQ_HZ(CLK_HZ),.UART_BPS(BAUD)) ht
        (.clk(clk),.rst_n(rst_n),.start(host_start),.data_in(host_out),.busy(host_busy),.txd(host_tx));
    uart_tx_8n1 #(.CLK_FREQ_HZ(CLK_HZ),.UART_BPS(BAUD)) pt
        (.clk(clk),.rst_n(rst_n),.start(motor_start && !axis),.data_in(motor_out),.busy(pan_busy),.txd(pan_tx));
    uart_tx_8n1 #(.CLK_FREQ_HZ(CLK_HZ),.UART_BPS(BAUD)) tt
        (.clk(clk),.rst_n(rst_n),.start(motor_start && axis),.data_in(motor_out),.busy(tilt_busy),.txd(tilt_tx));
    wire motor_busy = axis ? tilt_busy : pan_busy;
    wire motor_valid = axis ? tilt_valid : pan_valid;
    wire [7:0] motor_data = axis ? tilt_data : pan_data;
    localparam [3:0] IDLE=0, SEND=1, TX_START=2, TX_END=3, RECEIVE=4,
                     REPLY=5, REPLY_START=6, REPLY_END=7;
    localparam integer TIMEOUT_CLOCKS = CLK_HZ/1000*TIMEOUT_MS;
    localparam integer HOST_GAP_CLOCKS = CLK_HZ/200; // incomplete host frame: 5 ms
    reg [3:0] state;
    // Buffer the host frame while an autonomous motor transaction is in flight.
    reg [7:0] host_fifo[0:15];
    reg [3:0] host_rd,host_wr;
    reg [4:0] host_count;
    wire host_push=host_rx_valid && (host_count<16 || host_valid);
    assign host_valid=(state==IDLE && host_count!=0);
    assign host_data=host_fifo[host_rd];
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin host_rd<=0;host_wr<=0;host_count<=0;end
        else begin
            if(host_push) begin host_fifo[host_wr]<=host_rx_data;host_wr<=host_wr+1'b1;end
            if(host_valid) host_rd<=host_rd+1'b1;
            case({host_push,host_valid})
                2'b10:host_count<=host_count+1'b1;
                2'b01:host_count<=host_count-1'b1;
                default:host_count<=host_count;
            endcase
        end
    end
    reg [2:0] request_index;
    reg [7:0] request_crc, axis_byte, motor_id, command, value_lo, value_hi;
    reg [7:0] motor_crc, feedback_crc, result, reply_crc;
    reg [7:0] feedback [0:9];
    reg [3:0] feedback_count, reply_index;
    reg [2:0] send_index;
    reg [$clog2(TIMEOUT_CLOCKS+1)-1:0] timeout_count;
    reg [$clog2(HOST_GAP_CLOCKS+1)-1:0] host_gap_count;
    reg [7:0] reply_byte;
    integer i;
    reg auto_transaction;
    wire motor_feedback_ok=feedback[0]==motor_id && feedback[9]==feedback_crc;
    always @* begin
        if (reply_index == 0) reply_byte = 8'h5A;
        else if (reply_index == 1) reply_byte = result;
        else if (reply_index == 2) reply_byte = {7'd0,axis};
        else if (reply_index == 13) reply_byte = reply_crc;
        else reply_byte = feedback[reply_index-3];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=IDLE; axis<=0; request_index<=0; request_crc<=0;
            auto_transaction<=0;auto_accepted<=0;auto_response<=0;
            auto_result<=0;auto_enabled<=0;auto_angle<=0;auto_speed<=0;auto_current<=0;
            home_start<=0;home_axis<=0;home_cancel<=0;
            home_release<=0;home_release_axis<=0;
            axis_byte<=0; motor_id<=0; command<=0; value_lo<=0; value_hi<=0;
            motor_crc<=0; feedback_crc<=0; result<=0; reply_crc<=0;
            feedback_count<=0; reply_index<=0; send_index<=0;
            timeout_count<=0; host_gap_count<=0;
            host_start<=0; motor_start<=0; host_out<=8'hFF; motor_out<=8'hFF;
            for(i=0;i<10;i=i+1) feedback[i]<=0;
        end else begin
            host_start<=0; motor_start<=0;
            auto_accepted<=0;auto_response<=0;home_start<=0;home_cancel<=0;
            home_release<=0;
            // Capture a quick reply even if it begins while the last TX stops.
            if ((state==SEND || state==TX_START || state==TX_END || state==RECEIVE)
                && motor_valid && feedback_count<10) begin
                feedback[feedback_count]<=motor_data;
                feedback_count<=feedback_count+1'b1;
                if(feedback_count<9) feedback_crc<=crc8(feedback_crc,motor_data);
            end
            case(state)
                IDLE: begin
                    if(auto_valid && request_index==0 && !host_valid) begin
                        axis<=auto_axis;motor_id<=0;command<=auto_command;
                        value_lo<=auto_value[7:0];value_hi<=auto_value[15:8];
                        motor_crc<=crc8(crc8(crc8(crc8(0,0),auto_command),auto_value[7:0]),auto_value[15:8]);
                        auto_transaction<=1;auto_accepted<=1;
                        feedback_count<=0;feedback_crc<=0;send_index<=0;timeout_count<=0;
                        for(i=0;i<10;i=i+1) feedback[i]<=0;
                        state<=SEND;
                    end
                    if(request_index!=0) begin
                        if(host_gap_count==HOST_GAP_CLOCKS-1) begin
                            request_index<=0; host_gap_count<=0;
                        end else host_gap_count<=host_gap_count+1'b1;
                    end
                    if(host_valid) begin
                        host_gap_count<=0;
                        if(request_index==0) begin
                            if(host_data==8'hA5) begin
                                request_index<=1; request_crc<=crc8(0,host_data);
                            end
                        end else if(request_index==6) begin
                            auto_transaction<=0;
                            request_index<=0; feedback_count<=0; feedback_crc<=0;
                            reply_index<=0; reply_crc<=0; timeout_count<=0; send_index<=0;
                            for(i=0;i<10;i=i+1) feedback[i]<=0;
                            axis<=axis_byte[0];
                            if(host_data!=request_crc || axis_byte>1 || motor_id>15) begin
                                result<=3; state<=REPLY;
                            end else if(command==8'hF0)begin
                                // FPGA-local homing diagnostics, not native motor feedback.
                                feedback[0]<={6'd0,home_homed};
                                feedback[1]<={6'd0,home_fault_axes};
                                feedback[2]<={7'd0,auto_active};
                                feedback[3]<={7'd0,home_fault_axis};
                                feedback[4]<=home_fault_code;
                                result<=0;state<=REPLY;
                            end else if(auto_active && command!=0 && command!=2) begin
                                // Read/disable remain available during autonomous homing.
                                result<=4;state<=REPLY;
                            end else if(command==1 && motor_id==0) begin
                                // Clear any stale position/velocity mode BEFORE enabling.
                                command<=4;value_lo<=0;value_hi<=0;
                                motor_crc<=crc8(crc8(crc8(crc8(0,0),4),0),0);
                                home_start<=1;home_axis<=axis_byte[0];
                                result<=0;state<=SEND;
                            end else begin
                                if(command!=0)begin home_release<=1;home_release_axis<=axis_byte[0];end
                                if(command==2 && auto_active) home_cancel<=1;
                                motor_crc<=crc8(crc8(crc8(crc8(0,motor_id),command),value_lo),value_hi);
                                result<=0; state<=SEND;
                            end
                        end else begin
                            request_crc<=crc8(request_crc,host_data);
                            case(request_index)
                                1: axis_byte<=host_data;
                                2: motor_id<=host_data;
                                3: command<=host_data;
                                4: value_lo<=host_data;
                                5: value_hi<=host_data;
                            endcase
                            request_index<=request_index+1'b1;
                        end
                    end
                end
                SEND: if(!motor_busy) begin
                    case(send_index)
                        0: motor_out<=motor_id;
                        1: motor_out<=command;
                        2: motor_out<=value_lo;
                        3: motor_out<=value_hi;
                        default: motor_out<=motor_crc;
                    endcase
                    motor_start<=1; state<=TX_START;
                end
                TX_START: if(motor_busy) state<=TX_END;
                TX_END: if(!motor_busy) begin
                    if(send_index==4) begin timeout_count<=0; state<=RECEIVE; end
                    else begin send_index<=send_index+1'b1; state<=SEND; end
                end
                RECEIVE: begin
                    if(feedback_count==10) begin
                        result <= motor_feedback_ok ? 0 : 2;
                        if(auto_transaction) begin
                            auto_result<=motor_feedback_ok ? 0 : 2;
                            auto_enabled<=feedback[1][0];
                            auto_angle<={feedback[8],feedback[7]};
                            auto_speed<={feedback[6],feedback[5]};
                            auto_current<={feedback[4],feedback[3]};
                            auto_response<=1;state<=IDLE;
                        end else state<=REPLY;
                    end else if(timeout_count==TIMEOUT_CLOCKS-1) begin
                        result<=1;
                        if(auto_transaction) begin auto_result<=1;auto_response<=1;state<=IDLE;end
                        else state<=REPLY;
                    end else timeout_count<=timeout_count+1'b1;
                end
                REPLY: if(!host_busy) begin
                    host_out<=reply_byte; host_start<=1;
                    if(reply_index<13) reply_crc<=crc8(reply_crc,reply_byte);
                    state<=REPLY_START;
                end
                REPLY_START: if(host_busy) state<=REPLY_END;
                REPLY_END: if(!host_busy) begin
                    if(reply_index==13) state<=IDLE;
                    else begin reply_index<=reply_index+1'b1; state<=REPLY; end
                end
                default: begin state<=IDLE; request_index<=0; end
            endcase
        end
    end
endmodule
