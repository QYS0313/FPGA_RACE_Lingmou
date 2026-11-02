`timescale 1ns/1ps
// Angle homing without range rejection; apply angle limits after homing.
// Arm both axes at their measured poses before boot motion.
module qd4310_home #(
    parameter integer CLK_HZ=50000000, BOOT_HOME=1, START_BOTH=0, BOOT_MS=2000,
    parameter integer PERIOD_MS=20, MAX_HOME_MS=45000, HOLD_MS=200,
    parameter [15:0] PAN_HOME=16'd21285, TILT_HOME=16'd32768
) (
    input wire clk,rst_n,start,start_axis,cancel,
    input wire release_axis_valid,release_axis,
    output wire active,
    output reg [1:0] homed,
    output wire fault,
    output reg [1:0] fault_axes,
    output reg [7:0] fault_code,
    output reg fault_axis,
    output wire request_valid,
    output reg request_axis,
    output wire [7:0] request_command,
    output wire [15:0] request_value,
    input wire accepted,response_valid,
    input wire [7:0] response_result,
    input wire response_enabled,
    input wire [15:0] response_angle,
    input wire signed [15:0] response_speed
);
    localparam [2:0] IDLE=0,BOOT=1,PREPARE=2,REQUEST=3,WAIT_REPLY=4,DELAY=5;
    reg [2:0] state,phase;
    reg [1:0] pending,retries;
    reg [2:0] stable[0:1];
    reg [1:0] launched;
    reg [1:0] limit_axes;
    wire [1:0] monitored=homed | limit_axes;
    reg [15:0] captured_angle;
    reg arming;
    reg abort_latched,cleaning,clean_all,holding,hold_next;
    reg queued_start,queued_axis;
    reg [1:0] cleanup_step;
    reg [31:0] timer,age;
    wire [15:0] home_code=request_axis ? TILT_HOME : PAN_HOME;
    wire signed [15:0] offset=response_angle-home_code;
    wire [15:0] distance=offset[15] ? -offset : offset;
    // Tilt: positive encoder offset is upward; allow -30 to +120 degrees.
    // Default home is 180 degrees: post-home encoder range is ~150..300.
    // Signed modular offset preserves the permitted arc across encoder zero.
    wire out_of_range=request_axis ?
        (offset < -16'sd5461 || offset > 16'sd21845) : distance>16'd10922;
    assign fault=|fault_axes;
    assign active=state!=IDLE && !holding;
    assign request_valid=state==REQUEST;
    assign request_command=cleaning ? (cleanup_step[0] ? 8'd2 : 8'd4) :
                           (phase==1 ? 8'd5 : phase==2 ? 8'd1 : 8'd0);
    assign request_value=(!cleaning && phase==1) ? (arming ? captured_angle : home_code) : 0;

    task cleanup;
        input all_axes;
        begin
            cleaning<=1;clean_all<=all_axes;cleanup_step<=0;
            if(all_axes)begin request_axis<=0;homed<=0;limit_axes<=0;pending<=0;end
            else begin homed[request_axis]<=0;limit_axes[request_axis]<=0;pending[request_axis]<=0;end
            holding<=0;retries<=0;state<=PREPARE;
        end
    endtask
    task fail_axis;
        input [7:0] reason;
        begin
            fault_axes[request_axis]<=1;fault_axis<=request_axis;fault_code<=reason;
            cleanup(0);
        end
    endtask
    task next_axis;
        begin
            if(arming ? (!request_axis && pending[1]) : pending[!request_axis])begin
                request_axis<=!request_axis;
                phase<=(!arming && launched[!request_axis]) ? 3 : 0;
                holding<=0;retries<=0;state<=PREPARE;
            end else if(arming && (~fault_axes)!=2'b00)begin
                arming<=0;pending<=~fault_axes;request_axis<=fault_axes[0];
                phase<=0;age<=0;holding<=0;retries<=0;state<=PREPARE;
            end else begin arming<=0;pending<=0;timer<=0;holding<=0;state<=IDLE;end
        end
    endtask
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=BOOT_HOME ? BOOT : IDLE;phase<=0;pending<=BOOT_HOME ? 3 : 0;
            request_axis<=0;stable[0]<=0;stable[1]<=0;launched<=0;limit_axes<=0;
            captured_angle<=0;arming<=BOOT_HOME;homed<=0;fault_axes<=0;fault_code<=0;fault_axis<=0;
            abort_latched<=0;cleaning<=0;clean_all<=0;holding<=0;hold_next<=0;
            queued_start<=0;queued_axis<=0;cleanup_step<=0;timer<=0;age<=0;retries<=0;
        end else begin
            if(start)begin queued_start<=1;queued_axis<=start_axis;end
            if(release_axis_valid)homed[release_axis]<=0;
            if(cancel && active)abort_latched<=1;
            if(active && state!=BOOT && !cleaning && age<CLK_HZ/1000*MAX_HOME_MS)age<=age+1;
            case(state)
                IDLE:begin
                    holding<=0;
                    if(queued_start || start)begin
                        request_axis<=start ? start_axis : queued_axis;
                        pending<=START_BOTH ? 3 : ((start ? start_axis : queued_axis) ? 2 : 1);
                        if(START_BOTH)begin homed<=0;limit_axes<=0;fault_axes<=0;end
                        else begin
                            homed[start ? start_axis : queued_axis]<=0;
                            limit_axes[start ? start_axis : queued_axis]<=0;
                            fault_axes[start ? start_axis : queued_axis]<=0;
                        end
                        queued_start<=0;phase<=0;stable[0]<=0;stable[1]<=0;launched<=0;arming<=START_BOTH;age<=0;
                        abort_latched<=0;cleaning<=0;retries<=0;timer<=0;state<=PREPARE;
                    end else if(timer>=CLK_HZ/1000*HOLD_MS-1)begin
                        timer<=0;
                        if(monitored!=0 && !release_axis_valid)begin
                            request_axis<=monitored[hold_next] ? hold_next : !hold_next;
                            hold_next<=!hold_next;holding<=1;phase<=5;retries<=0;state<=PREPARE;
                        end
                    end else timer<=timer+1;
                end
                BOOT:if(abort_latched || cancel)cleanup(1);
                     else if(timer>=CLK_HZ/1000*BOOT_MS-1)begin timer<=0;age<=0;state<=PREPARE;end
                     else timer<=timer+1;
                PREPARE:if(abort_latched && !cleaning)cleanup(1);
                        else if(!cleaning && !holding && age>=CLK_HZ/1000*MAX_HOME_MS-1)fail_axis(4);
                        else if(holding && !monitored[request_axis])begin holding<=0;timer<=0;state<=IDLE;end
                        else state<=REQUEST;
                REQUEST:if(accepted)state<=WAIT_REPLY;
                        else if(holding && !monitored[request_axis])begin holding<=0;timer<=0;state<=IDLE;end
                WAIT_REPLY:if(response_valid)begin
                    if(cleaning)begin
                        if((!clean_all && cleanup_step==1) || cleanup_step==3)begin
                            cleaning<=0;abort_latched<=0;
                            if(clean_all)begin arming<=0;pending<=0;timer<=0;state<=IDLE;end
                            else next_axis;
                        end else begin
                            cleanup_step<=cleanup_step+1;
                            if(clean_all)request_axis<=cleanup_step>=1;
                            state<=PREPARE;
                        end
                    end else if(abort_latched || cancel)cleanup(1);
                    else if(response_result!=0)begin
                        stable[request_axis]<=0;
                        if(retries<2)begin retries<=retries+1;timer<=0;state<=DELAY;end
                        else fail_axis(response_result==1 ? 1 : 2);
                    end else if(holding)begin
                        retries<=0;
                        if(!response_enabled && !homed[request_axis])begin
                            limit_axes[request_axis]<=0;holding<=0;timer<=0;state<=IDLE;
                        end else if(!response_enabled)fail_axis(5);
                        else if(out_of_range)fail_axis(3);
                        else begin holding<=0;timer<=0;state<=IDLE;end
                    end else if((phase==3 || phase==4) && !response_enabled)begin
                        if(retries<2)begin retries<=retries+1;timer<=0;state<=DELAY;end
                        else fail_axis(5);
                    end else begin
                        retries<=0;
                        if(phase==0)begin captured_angle<=response_angle;phase<=1;state<=PREPARE;end
                        else if(phase==1)begin phase<=2;state<=PREPARE;end
                        else if(phase==2 && !response_enabled)fail_axis(5);
                        else if(phase==2 && arming)next_axis;
                        else if(phase==2)begin
                            launched[request_axis]<=1;timer<=0;
                            if(pending[!request_axis] && !launched[!request_axis])begin
                                request_axis<=!request_axis;phase<=0;state<=PREPARE;
                            end else begin phase<=3;state<=DELAY;end
                        end
                        else if(phase==3)begin
                            timer<=0;
                            if(distance<=91 && response_speed<=10 && response_speed>=-10)begin
                                if(stable[request_axis]==4)begin phase<=4;state<=PREPARE;end
                                else begin stable[request_axis]<=stable[request_axis]+1;state<=DELAY;end
                            end else begin stable[request_axis]<=0;state<=DELAY;end
                        end else begin
                            homed[request_axis]<=1;limit_axes[request_axis]<=1;
                            pending[request_axis]<=0;next_axis;
                        end
                    end
                end
                DELAY:if(abort_latched || cancel)cleanup(1);
                      else if(timer>=CLK_HZ/1000*PERIOD_MS-1)begin
                          if(!arming && !holding && phase==3 && retries==0 &&
                             pending[!request_axis] && launched[!request_axis])request_axis<=!request_axis;
                          state<=PREPARE;
                      end
                      else timer<=timer+1;
                default:state<=IDLE;
            endcase
        end
    end
endmodule
