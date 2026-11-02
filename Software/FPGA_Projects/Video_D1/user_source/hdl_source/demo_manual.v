// Manual video-mode motors. Native drive speed/angle loops are retained.
// On boot no motion is issued. Home is explicit and simultaneous on both axes.
module demo_manual #(parameter integer CLK_HZ=50000000)(
 input wire clk,rst_n,pan_rx,tilt_rx,output wire pan_tx,tilt_tx,
 input wire [24:0] request_async,input wire request_toggle_async,
 input wire home_toggle_async,heartbeat_toggle_async,
 output reg [63:0] response,output reg ack,
 output wire [7:0] status,output wire switch_safe,output wire motion_active
);
 reg [24:0] req1,req2,req;
 reg [2:0] req_sync,home_sync,hb_sync;
 reg home_seen,hb_seen,home_start,home_cancel;
 wire release_axis;
 reg [31:0] heartbeat_age;
 wire home_active,home_fault,hvalid,haxis;
 wire [1:0] homed,hfault_axes;
 wire [7:0] hcmd,hcode;
 wire [15:0] hvalue;
 wire hfaxis;
 reg owner,owner_pending;
 reg mvalid,maxis;
 reg [7:0] mcmd;
 reg [15:0] mvalue;
 wire accepted,done,enabled;
 wire [7:0] result;
 wire [15:0] angle;
 wire signed [15:0] speed,current;
 assign release_axis=done && !owner && result==0 && mcmd==2;
 reg [1:0] velocity,held,disabled,quiet;
 reg fault;
 reg [15:0] last_angle[0:1];
 reg [31:0] pose_age[0:1];
 localparam IDLE=0,ISSUE=1,WAIT=2,STOP_DELAY=3;
 reg [2:0] state;
 reg watchdog;
 reg [2:0] stop_phase;
 reg [2:0] stable;
 reg [31:0] stop_timer,delay_timer;
 wire signed [15:0] delta=angle-last_angle[maxis];
 wire signed [15:0] requested_offset=req2[15:0]-(req2[24] ? 16'd32768 : 16'd21285);
 wire angle_allowed=req2[24] ? (requested_offset>=-5461 && requested_offset<=21845) :
                                            (requested_offset>=-10923 && requested_offset<=10923);
 wire signed [15:0] requested_speed=req2[15:0];
 wire choose_home=hvalid;
 wire dispatch=owner_pending ? (owner ? hvalid : mvalid) : hvalid || mvalid;
 wire use_home=owner_pending ? owner : choose_home;
 assign status={fault,home_fault,home_active,watchdog,disabled,homed};
 assign motion_active=home_active || velocity!=0 || state==STOP_DELAY;
 // Disabled axes or explicitly confirmed native angle holds are safe.
 assign switch_safe=!home_active && !home_fault && !fault && !watchdog && state==IDLE &&
  !owner_pending && (held|disabled)==3 && velocity==0 && quiet==3 &&
  pose_age[0]<CLK_HZ && pose_age[1]<CLK_HZ;
 qd4310_home #(.CLK_HZ(CLK_HZ),.BOOT_HOME(0),.START_BOTH(1)) home(
  .clk(clk),.rst_n(rst_n),.start(home_start),.start_axis(1'b0),.cancel(home_cancel),
  .release_axis_valid(release_axis),.release_axis(maxis),.active(home_active),.homed(homed),
  .fault(home_fault),.fault_axes(hfault_axes),.fault_code(hcode),.fault_axis(hfaxis),
  .request_valid(hvalid),.request_axis(haxis),.request_command(hcmd),.request_value(hvalue),
  .accepted(accepted && owner),.response_valid(done && owner),.response_result(result),
  .response_enabled(enabled),.response_angle(angle),.response_speed(speed));
 qd4310_bridge #(.CLK_HZ(CLK_HZ)) bridge(
  .clk(clk),.rst_n(rst_n),.host_rx(1'b1),.host_tx(),
  .pan_rx(pan_rx),.pan_tx(pan_tx),.tilt_rx(tilt_rx),.tilt_tx(tilt_tx),
  .auto_active(home_active),.auto_valid(dispatch),.auto_axis(use_home?haxis:maxis),
  .auto_command(use_home?hcmd:mcmd),.auto_value(use_home?hvalue:mvalue),
  .auto_accepted(accepted),.auto_response(done),.auto_result(result),.auto_enabled(enabled),
  .auto_angle(angle),.auto_speed(speed),.auto_current(current),
  .home_start(),.home_axis(),.home_cancel(),.home_release(),.home_release_axis(),
  .home_homed(homed),.home_fault_axes(hfault_axes),.home_fault_code(hcode),.home_fault_axis(hfaxis));
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin owner<=0;owner_pending<=0;end
  else begin
   if(!owner_pending && (hvalid || mvalid))begin owner<=hvalid;owner_pending<=1;end
   if(done)owner_pending<=0;
  end
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   req1<=0;req2<=0;req<=0;req_sync<=0;home_sync<=0;hb_sync<=0;
   home_seen<=0;hb_seen<=0;home_start<=0;home_cancel<=0;heartbeat_age<=0;
   response<=0;ack<=0;state<=IDLE;mvalid<=0;maxis<=0;mcmd<=0;mvalue<=0;
   velocity<=0;held<=0;disabled<=0;quiet<=0;fault<=0;watchdog<=0;stop_phase<=0;
   stable<=0;stop_timer<=0;delay_timer<=0;
   last_angle[0]<=0;last_angle[1]<=0;pose_age[0]<=CLK_HZ;pose_age[1]<=CLK_HZ;
  end else begin
   req1<=request_async;req2<=req1;req_sync<={req_sync[1:0],request_toggle_async};
   home_sync<={home_sync[1:0],home_toggle_async};hb_sync<={hb_sync[1:0],heartbeat_toggle_async};
   home_start<=0;home_cancel<=0;
   if(heartbeat_age<CLK_HZ)heartbeat_age<=heartbeat_age+1;
   if(hb_sync[2]!=hb_seen)begin hb_seen<=hb_sync[2];heartbeat_age<=0;end
   if(pose_age[0]<CLK_HZ)pose_age[0]<=pose_age[0]+1;
   if(pose_age[1]<CLK_HZ)pose_age[1]<=pose_age[1]+1;
   if(done && owner && result==0)begin
    last_angle[haxis]<=angle;pose_age[haxis]<=0;disabled[haxis]<=!enabled;
    quiet[haxis]<=speed<=10 && speed>=-10;
    if(hcmd==5)held[haxis]<=1;
   end
   if(home_active)begin held<=0;velocity<=0;end
   if(watchdog && stop_timer<CLK_HZ*3)stop_timer<=stop_timer+1;
   case(state)
    IDLE:begin
     if(home_sync[2]!=home_seen)begin
      home_seen<=home_sync[2];
      if(!home_active && velocity==0)begin home_start<=1;fault<=0;held<=0;disabled<=0;end
     end else if(velocity!=0 && heartbeat_age>=CLK_HZ/2 && !home_active)begin
      watchdog<=1;stop_phase<=0;maxis<=0;mcmd<=4;mvalue<=0;mvalid<=1;
      stable<=0;stop_timer<=0;state<=ISSUE;
     end else if(req_sync[2]!=ack)begin
      req<=req2;maxis<=req2[24];mcmd<=req2[23:16];mvalue<=req2[15:0];
      if(home_active && req2[23:16]==2)begin
       home_cancel<=1;response<={54'd0,req2[24],1'b0,8'd0};ack<=req_sync[2];
      end else if(home_active || watchdog || (req2[23:16]!=0 && req2[23:16]!=2 &&
       (fault || home_fault || homed!=3)) ||
       !(req2[23:16]==0 || req2[23:16]==2 || req2[23:16]==4 || req2[23:16]==5) ||
       (req2[23:16]==5 && !angle_allowed) ||
       (req2[23:16]==4 && (requested_speed>328 || requested_speed< -328)))begin
       response<=64'd4;ack<=req_sync[2];
      end else begin mvalid<=1;state<=ISSUE;heartbeat_age<=0;end
     end
    end
    ISSUE:if(accepted && !owner)begin mvalid<=0;state<=WAIT;end
    WAIT:if(done && !owner)begin
     if(result==0)begin
      last_angle[maxis]<=angle;pose_age[maxis]<=0;disabled[maxis]<=!enabled;
      quiet[maxis]<=speed<=10 && speed>=-10;
      if(mcmd==4)begin velocity[maxis]<=mvalue!=0;held[maxis]<=0;end
      if(mcmd==5)begin velocity[maxis]<=0;held[maxis]<=enabled;end
      if(mcmd==2)begin velocity[maxis]<=0;held[maxis]<=0;end
     end
     if(!watchdog)begin
      response<={current,speed,angle,6'd0,maxis,enabled,result};ack<=req_sync[2];state<=IDLE;
      if(result!=0 && velocity!=0)heartbeat_age<=CLK_HZ/2;
     end else if(result!=0 || stop_timer>=CLK_HZ*2)begin
      // If feedback is unusable, attempt disable of both axes and latch fault.
      if(stop_phase<6)begin stop_phase<=6;maxis<=0;mcmd<=2;mvalue<=0;mvalid<=1;state<=ISSUE;fault<=1;end
      else if(stop_phase==6)begin stop_phase<=7;maxis<=1;mvalid<=1;state<=ISSUE;end
      else begin watchdog<=0;velocity<=0;state<=IDLE;end
     end else if(stop_phase==0)begin
      stop_phase<=1;maxis<=1;mcmd<=4;mvalue<=0;mvalid<=1;state<=ISSUE;
     end else if(stop_phase==1)begin
      stop_phase<=2;maxis<=0;stable<=0;mcmd<=0;delay_timer<=0;state<=STOP_DELAY;
     end else if(stop_phase==2 || stop_phase==4)begin
      if(enabled && speed<=10 && speed>=-10 && delta<=18 && delta>=-18)stable<=stable+1;
      else stable<=0;
      if(stable>=4 && enabled && speed<=10 && speed>=-10 && delta<=18 && delta>=-18)begin
       stop_phase<=stop_phase+1;mcmd<=5;mvalue<=angle;mvalid<=1;state<=ISSUE;
      end else begin mcmd<=0;delay_timer<=0;state<=STOP_DELAY;end
     end else if(stop_phase==3)begin
      stop_phase<=4;maxis<=1;stable<=0;mcmd<=0;delay_timer<=0;state<=STOP_DELAY;
     end else begin watchdog<=0;state<=IDLE;end
    end
    STOP_DELAY:if(delay_timer>=CLK_HZ/50-1)begin mvalid<=1;state<=ISSUE;end
               else delay_timer<=delay_timer+1;
    default:state<=IDLE;
   endcase
  end
 end
endmodule
