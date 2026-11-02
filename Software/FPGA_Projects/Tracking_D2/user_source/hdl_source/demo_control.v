// Common 100 MHz UART extension. Commands 80..94 are reserved for demo control.
// Motor request/response cross as held bundles with toggle handshakes.
module demo_control #(
 parameter integer CLK_HZ=100000000, IMAGE=0,
 parameter [31:0] VIDEO_ADDRESS=32'h00000000, TRACK_ADDRESS=32'h00800000
)(
 input wire clk,rst_n,valid,input wire [7:0] cmd,data,
 output reg [7:0] reply,
 output reg [24:0] motor_request,output reg motor_toggle,
 input wire [63:0] motor_response_async,input wire motor_ack_async,
 input wire [7:0] motor_status_async,
 output reg home_toggle,heartbeat_toggle,
 input wire switch_safe_async,
 output wire blank,output reg boot_en,output wire [31:0] boot_address
);
 reg [2:0] ack_sync;
 reg [63:0] response_meta,response_sync,response;
 reg [7:0] status_meta,status_sync;
 reg [1:0] safe_sync;
 reg axis;
 reg [7:0] operation,lo,hi;
 reg [1:0] boot_state;
 reg destination;
 reg [31:0] timer;
 wire busy=ack_sync[2]!=motor_toggle;
 assign blank=boot_state!=0;
 assign boot_address=destination ? TRACK_ADDRESS : VIDEO_ADDRESS;
 always @* begin
  reply=8'hFF;
  case(cmd)
   8'h80,8'h81,8'h82,8'h83:reply=(busy || blank) ? 8'hFE : data;
   8'h84:reply=(IMAGE!=0 || busy || blank || data!=8'hA5) ? 8'hFE : 0;
   8'h85:reply={7'd0,busy};
   8'h86:reply=data<8 ? response[data*8 +: 8] : 8'hFF;
   8'h87:reply=(IMAGE!=0 || busy || blank || data!=8'hA5) ? 8'hFE : 0;
   8'h88:reply=data;
   8'h89:reply=status_sync;
   8'h90:reply=IMAGE==0 ? 8'hD1 : 8'hD2;
   8'h91:reply={4'd0,safe_sync[1],busy,boot_state};
   8'h92:reply=(data>1 || data==IMAGE || busy || !safe_sync[1] || boot_state!=0) ? 8'hFE : 0;
   8'h93:reply=(data!=8'hA5 || boot_state!=1 || !safe_sync[1] || busy) ? 8'hFE : 0;
   8'h94:reply=0;
  endcase
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   ack_sync<=0;response_meta<=0;response_sync<=0;response<=0;
   status_meta<=0;status_sync<=0;safe_sync<=0;
   axis<=0;operation<=0;lo<=0;hi<=0;motor_request<=0;motor_toggle<=0;
   home_toggle<=0;heartbeat_toggle<=0;boot_state<=0;destination<=0;timer<=0;boot_en<=0;
  end else begin
   ack_sync<={ack_sync[1:0],motor_ack_async};
   response_meta<=motor_response_async;response_sync<=response_meta;
   if(ack_sync[2]==motor_toggle) response<=response_sync;
   status_meta<=motor_status_async;status_sync<=status_meta;
   safe_sync<={safe_sync[0],switch_safe_async};
   if(boot_state==1) begin
    // Feedback polling can temporarily make the motor bus non-idle.
    // Keep the arm bounded; commit still requires fresh safe feedback.
    if(timer>=CLK_HZ*2-1) begin boot_state<=0;timer<=0;end
    else timer<=timer+1;
   end
   if(boot_state==2) begin
    // Wait at least 20 ms for ACK, then trigger only at a safe idle boundary.
    // Do not cancel on a normal read-only poll during the ACK delay.
    // Stale feedback, movement, faults or a busy bus keep the gate closed;
    // after two seconds cancel without automatic retry.
    if(timer>=CLK_HZ*2-1) begin boot_state<=0;timer<=0;end
    else if(timer>=CLK_HZ/50-1 && safe_sync[1] && !busy) begin boot_en<=1;boot_state<=3;end
    else timer<=timer+1;
   end
   // boot_en remains high until reconfiguration (>1 us PH1P35 minimum).
   if(valid) begin
    if(cmd==8'h88 && data==8'h5A)heartbeat_toggle<=!heartbeat_toggle;
    if(!busy && !blank) case(cmd)
     8'h80:axis<=data[0];
     8'h81:operation<=data;
     8'h82:lo<=data;
     8'h83:hi<=data;
     8'h84:if(IMAGE==0 && data==8'hA5) begin
      motor_request<={axis,operation,hi,lo};motor_toggle<=!motor_toggle;
     end
     8'h87:if(IMAGE==0 && data==8'hA5)home_toggle<=!home_toggle;
    endcase
    if(cmd==8'h92 && reply==0)begin destination<=data[0];boot_state<=1;timer<=0;end
    if(cmd==8'h93 && reply==0)begin boot_state<=2;timer<=0;end
    if(cmd==8'h94 && boot_state!=3)begin boot_state<=0;timer<=0;end
   end
  end
 end
endmodule
