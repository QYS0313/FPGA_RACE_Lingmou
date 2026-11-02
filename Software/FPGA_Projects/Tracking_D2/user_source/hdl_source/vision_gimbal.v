// 50 MHz motor subsystem, using the proven motion_lab UART bridge/home FSM.
module vision_gimbal #(
    // Shared logical homes for native angle homing and visual travel limits.
    parameter [15:0] PAN_HOME=16'd21285, TILT_HOME=16'd32768
)(
    input wire clk,rst_n,pan_rx,tilt_rx,
    output wire pan_tx,tilt_tx,
    input wire [7:0] control_async,
    input wire control_toggle_async,
    input wire restart_async,
    input wire [255:0] config_async,
    input wire config_toggle_async,
    input wire [1:0] mode_async,
    input wire frame_toggle,pixel_valid,
    input wire [11:0] pixel_x,pixel_y,pixel_w,pixel_h,
    output wire switch_safe,
    output wire [7:0] status,test_status,
    output reg [191:0] diag_data,output reg diag_toggle,
    input wire diag_ack_async
);
    reg [7:0] control1,control2,control;
    reg [2:0] control_sync;
    reg control_seen;
    reg [2:0] restart_sync,config_sync;
    reg restart_seen,config_seen;
    reg [255:0] config1,config2;
    reg [1:0] mode1,mode2,mode;
    reg [11:0] aim_x,aim_y;
    reg restart;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            control1<=0;control2<=0;control<=0;control_sync<=0;control_seen<=0;
            restart_sync<=0;restart_seen<=0;restart<=0;
            config_sync<=0;config_seen<=0;config1<=0;config2<=0;
            mode1<=0;mode2<=0;mode<=0;aim_x<=546;aim_y<=252;
        end else begin
            // B15 bundled control: apply the entire byte only after its commit
            // crosses three flops. Request, axis and profile must become visible
            // together. The UART source holds data until the next full packet
            // (>= 347 us at 115200), far longer than this four-cycle transfer.
            control1<=control_async;control2<=control1;
            control_sync<={control_sync[1:0],control_toggle_async};
            if(control_sync[2]!=control_seen) begin
                control<=control2;control_seen<=control_sync[2];
            end
            restart_sync<={restart_sync[1:0],restart_async};
            restart<=restart_sync[2]!=restart_seen;restart_seen<=restart_sync[2];
            config_sync<={config_sync[1:0],config_toggle_async};
            config1<=config_async;config2<=config1;mode1<=mode_async;mode2<=mode1;
            if(config_sync[2]!=config_seen) begin
                config_seen<=config_sync[2];mode<=mode2;
                aim_x<=config2[243:232];aim_y<=config2[255:244];
            end
        end
    end
    wire valid;
    wire [11:0] x,y;
    spot_cdc #(.CLK_HZ(50000000)) coordinates(
        .clk(clk),.rst_n(rst_n),.frame_toggle(frame_toggle),.pixel_valid(pixel_valid),
        .pixel_x(pixel_x),.pixel_y(pixel_y),.pixel_w(pixel_w),.pixel_h(pixel_h),
        .valid(valid),.x(x),.y(y),.w(),.h());
    wire motor_rst=rst_n && !restart;
    wire home_active,home_valid,home_axis,home_fault;
    wire [1:0] homed,fault_axes;
    wire [7:0] home_cmd,home_code;
    wire [15:0] home_value;
    wire home_fault_axis;
    wire sv,sa,sfault;
    wire [7:0] sc;
    wire [15:0] value;
    wire ack,resp,en;
    wire [7:0] result;
    wire [15:0] angle;
    wire signed [15:0] speed,current;
    wire sample_valid;
    wire [191:0] sample_data;
    reg [1:0] diag_ack_sync;
    // A full handshake keeps the payload stable across the clock boundary.
    // Telemetry can drop a sample; it never stalls the motor control FSM.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin diag_data<=0;diag_toggle<=0;diag_ack_sync<=0;end
        else begin
            diag_ack_sync<={diag_ack_sync[0],diag_ack_async};
            if(sample_valid && diag_ack_sync[1]==diag_toggle) begin
                diag_data<=sample_data;diag_toggle<=!diag_toggle;
            end
        end
    end
    // Latch the requester until response: background home reads may overlap
    // a pending visual request, but feedback must never change owners.
    reg owner,owner_pending;
    reg start_after_reset;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)start_after_reset<=0;else start_after_reset<=restart;
    reg [1:0] safe_hold, safe_quiet, previous_homed;
    reg [25:0] safe_age0,safe_age1;
    reg issued_axis;
    reg [7:0] issued_command;
    always @(posedge clk or negedge motor_rst)begin
        if(!motor_rst)begin safe_hold<=0;safe_quiet<=0;previous_homed<=0;safe_age0<=50000000;safe_age1<=50000000;issued_axis<=0;issued_command<=0;end
        else begin
            previous_homed<=homed;
            if(homed[0] && !previous_homed[0])safe_hold[0]<=1;
            if(homed[1] && !previous_homed[1])safe_hold[1]<=1;
            if(safe_age0<50000000)safe_age0<=safe_age0+1;
            if(safe_age1<50000000)safe_age1<=safe_age1+1;
            if(ack && !owner)begin
                issued_axis<=sa;issued_command<=sc;
                if(sc==4)safe_hold[sa]<=0;
            end
            if(resp && !owner)begin
                if(issued_axis)safe_age1<=0;else safe_age0<=0;
                safe_quiet[issued_axis]<=result==0 && en && speed<=10 && speed>=-10;
                if(result!=0 || !en)safe_hold[issued_axis]<=0;
                else if(issued_command==5)safe_hold[issued_axis]<=1;
            end
        end
    end
    assign switch_safe=(safe_hold==3 && safe_quiet==3 && safe_age0<50000000 && safe_age1<50000000 &&
        homed==3 && !home_active && !home_fault && !sfault && !control[0] && !control[1] && !control[2]);

    wire choose_home=home_valid && !control[1];
    wire choose_servo=sv && (!home_active || control[1]) && !choose_home;
    always @(posedge clk or negedge motor_rst) begin
        if(!motor_rst) begin owner<=0;owner_pending<=0;end
        else begin
            if(!owner_pending && (choose_home || choose_servo)) begin
                owner<=choose_home;owner_pending<=1;
            end
            if(resp) owner_pending<=0;
        end
    end
    // Bridge consumes the request one cycle before its registered ack arrives.
    // Selection is locked by owner_pending, and stays locked through response.
    wire use_home=owner_pending ? owner : choose_home;
    wire dispatch=owner_pending ? (owner ? home_valid : sv) : choose_home || choose_servo;
    qd4310_home #(.BOOT_HOME(0),.START_BOTH(1),.PAN_HOME(PAN_HOME),.TILT_HOME(TILT_HOME)) home(
        .clk(clk),.rst_n(motor_rst && !control[1]),.start(start_after_reset),.start_axis(1'b0),.cancel(1'b0),
        .release_axis_valid(1'b0),.release_axis(1'b0),.active(home_active),.homed(homed),
        .fault(home_fault),.fault_axes(fault_axes),.fault_code(home_code),.fault_axis(home_fault_axis),
        .request_valid(home_valid),.request_axis(home_axis),.request_command(home_cmd),.request_value(home_value),
        .accepted(ack && owner),.response_valid(resp && owner),.response_result(result),
        .response_enabled(en),.response_angle(angle),.response_speed(speed));
    vision_gimbal_servo #(.PAN_HOME(PAN_HOME),.TILT_HOME(TILT_HOME)) servo(
        .clk(clk),.rst_n(motor_rst),.run(control[0] && (mode==1 || mode==3)),
        .ready(homed==3 && !home_active && !home_fault),.disabled(control[1]),
        .spot_valid(valid),.x(x),.y(y),.aim_x(aim_x),.aim_y(aim_y),
        .invert_pan(control[4]),.invert_tilt(control[5]),
        .test_request(control[2]),.test_axis(control[3]),.test_dynamic(control[6]),.test_status(test_status),
        .request_valid(sv),.axis(sa),.command(sc),.value(value),
        .accepted(ack && !owner),.response(resp && !owner),.result(result),.enabled(en),.angle(angle),.speed(speed),.current(current),.fault(sfault),
        .sample_valid(sample_valid),.sample_data(sample_data));
    qd4310_bridge bridge(
        .clk(clk),.rst_n(motor_rst),.host_rx(1'b1),.host_tx(),
        .pan_rx(pan_rx),.pan_tx(pan_tx),.tilt_rx(tilt_rx),.tilt_tx(tilt_tx),
        .auto_active(home_active),.auto_valid(dispatch),.auto_axis(use_home?home_axis:sa),
        .auto_command(use_home?home_cmd:sc),.auto_value(use_home?home_value:value),
        .auto_accepted(ack),.auto_response(resp),.auto_result(result),.auto_enabled(en),.auto_angle(angle),.auto_speed(speed),.auto_current(current),
        .home_start(),.home_axis(),.home_cancel(),.home_release(),.home_release_axis(),
        .home_homed(homed),.home_fault_axes(fault_axes),.home_fault_code(home_code),.home_fault_axis(home_fault_axis));
    assign status={test_status[7],sfault,control[1],control[0],home_fault,home_active,homed};
endmodule
