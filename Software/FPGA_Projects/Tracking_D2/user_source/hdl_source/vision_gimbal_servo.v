// Native velocity tracking (CMD4: 32767 = 1000 rpm), measured angle hold.
module vision_gimbal_servo #(
 parameter integer CLK_HZ=50000000,
 parameter [15:0] PAN_HOME=16'd21285, TILT_HOME=16'd32768
)(
 input wire clk,rst_n,run,ready,disabled,spot_valid,
 input wire [11:0] x,y,aim_x,aim_y,
 input wire invert_pan,invert_tilt,
 input wire test_request,test_axis,test_dynamic,
 output wire [7:0] test_status,
 output wire request_valid,output reg axis,
 output wire [7:0] command,output wire [15:0] value,
 input wire accepted,response,input wire [7:0] result,
 input wire enabled,input wire [15:0] angle,
 input wire signed [15:0] speed,current,output reg fault,
 output reg sample_valid,output reg [191:0] sample_data
);
 localparam IDLE=0,READ=1,WAIT_READ=2,PLAN=3,RAMP=4,WRITE=5,WAIT_WRITE=6,GAIN=7;
 reg [2:0] state;
 reg [31:0] timer;
 reg [1:0] tracking,velocity_mode,disable_sent;
 reg [1:0] stop_pending,start_negative;
 reg [1:0] start_count[0:1];
 reg [15:0] settle_anchor[0:1];
 reg signed [15:0] velocity[0:1];
 reg [3:0] settled[0:1];
 reg [5:0] stop_ticks[0:1];
 reg signed [15:0] measured,measured_speed;
 reg [15:0] measured_angle;
 reg signed [13:0] error;
 wire signed [13:0] raw_error=axis ? $signed({1'b0,aim_y})-$signed({1'b0,y}) :
                                                  $signed({1'b0,aim_x})-$signed({1'b0,x});
 reg [11:0] measured_x,measured_y;
 reg want_track;
 reg signed [15:0] desired,desired_reg,next_velocity,slew;
 reg signed [16:0] lo,hi,remaining,limit_speed;
 reg [7:0] write_command;
 reg [15:0] write_value;
 reg read_disable;
 wire emergency=disabled || fault;
 reg [15:0] previous_angle[0:1];
 reg signed [15:0] previous_speed[0:1];
 reg [1:0] history_valid;
 reg moving_positive,moving_negative;
 wire signed [15:0] encoder_delta=angle-previous_angle[axis];
 // Upper-axis encoder rate from two feedback intervals (~80ms).
 // Native rpm sometimes spikes or stays at zero during encoder motion.
 // At 80ms, encoder counts * 3/8 ~= native speed units (32767=1000rpm).
 reg [15:0] tilt_previous,tilt_older;
 reg [31:0] diag_tick;
 reg [31:0] tilt_previous_tick,tilt_older_tick;
 reg [1:0] tilt_history;
 reg signed [10:0] tilt_delta;
 reg signed [9:0] tilt_rate;
 wire signed [15:0] tilt_delta_80=angle-tilt_older;
 wire [31:0] tilt_interval=diag_tick-tilt_older_tick;
 wire [10:0] tilt_delta_abs=tilt_delta<0 ? -tilt_delta : tilt_delta;
 wire [12:0] tilt_scaled={1'b0,tilt_delta_abs,1'b0}+{2'b0,tilt_delta_abs}+13'd4;
 reg [9:0] toward_rate,tilt_brake_threshold,tilt_excess;
 reg [7:0] tilt_base,tilt_boost,tilt_brake;
 reg signed [10:0] tilt_drive;
 // B17: a single 400ms startup pulse, rearmed only by measured angle hold.
 // Encoder slowdowns and brief target losses must not restart the pulse.
 reg [3:0] tilt_start_ticks;
 reg tilt_start_negative;
 reg tilt_start_now;
 reg [4:0] tilt_brake_room;
 reg [12:0] magnitude;
 reg [7:0] gain,gain_reg;
 reg [4:0] chase_extra;
 wire [6:0] chase_delta=magnitude[6:0]-7'd40;
 // Narrow, explicit shift/add arithmetic; avoid constant multipliers in DSPs.
 // Each delta is used only in its 20- or 40-pixel segment (modulo subtraction).
 wire [4:0] delta20=magnitude[4:0]-5'd20;
 wire [4:0] delta40=magnitude[4:0]-5'd8;
 wire [5:0] delta60=magnitude[5:0]-6'd60;
 wire [7:0] near_scale={delta20,3'b0}+{3'd0,delta20}+8'd8;
 wire [6:0] mid_scale={delta40,2'b0}+{2'd0,delta40};
 wire [7:0] outer_scale={1'b0,delta60,1'b0}+{2'd0,delta60};
 // Explicit, edge-triggered, bounded test. No camera or USB timing involved.
 // Fixed: seven 2-second stages. Dynamic: nine 2-second stages.
 // Both finish with at most 2 seconds to stop and hold; profile is latched.
 reg test_seen,test_active,test_selected,test_failed,test_done,test_profile;
 reg [3:0] test_stage;
 reg [31:0] test_timer;
 reg signed [15:0] test_speed;
 wire visual_run=run && !test_request && !test_active;
 wire testing_axis=test_active && axis==test_selected;
 wire [3:0] test_end_stage=test_profile ? 4'd9 : 4'd7;
 wire test_driving=testing_axis && test_stage<(test_profile ? 4'd8 : 4'd6);
 wire test_timeout=test_active && test_stage==test_end_stage && test_timer==2*CLK_HZ-1 && velocity_mode!=0;
 assign test_status={test_active,test_failed,test_done,test_selected,test_stage};
 always @* begin
  if(test_profile) case(test_stage)
   1,3:test_speed=10; 2:test_speed=20;
   5,7:test_speed=-10; 6:test_speed=-20;
   default:test_speed=0;
  endcase
  else case(test_stage)
   1:test_speed=16; 2:test_speed=33;
   4:test_speed=-16; 5:test_speed=-33;
   default:test_speed=0;
  endcase
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   test_seen<=0;test_active<=0;test_selected<=0;test_failed<=0;test_done<=0;test_profile<=0;test_stage<=0;test_timer<=0;
  end else begin
   test_seen<=test_request;
   if(test_request && !test_seen && !test_active) begin
    test_selected<=test_axis;test_profile<=test_dynamic;test_stage<=0;test_timer<=0;test_done<=0;
    test_failed<=!(ready && !emergency && !run && velocity_mode==0);
    test_active<=ready && !emergency && !run && velocity_mode==0;
   end else if(test_active) begin
    if(emergency || !ready || test_timeout) begin
     test_active<=0;test_failed<=1;
    end else if((!test_request || run) && test_stage!=test_end_stage) begin
     test_stage<=test_end_stage;test_timer<=0;test_failed<=1;
    end else if(test_stage==test_end_stage && velocity_mode==0) begin
     test_active<=0;test_done<=!test_failed;
    end else if(test_timer==2*CLK_HZ-1) begin
     test_timer<=0;
     if(test_stage<test_end_stage) test_stage<=test_stage+1'b1;
    end else test_timer<=test_timer+1'b1;
   end
  end
 end
 // Read-only diagnostic snapshot: PLAN inputs + RAMP's planned command.
 // Feedback belongs to the preceding READ, not the response to this command.
 reg [15:0] diag_seq[0:1];
 reg signed [15:0] measured_current;
 reg diag_pending;
 wire [15:0] home=axis?TILT_HOME:PAN_HOME;
 wire signed [15:0] feedback_offset=angle-home;
 wire outside=axis ? (feedback_offset< -5461 || feedback_offset>21845) :
                     (feedback_offset< -10922 || feedback_offset>10922);
 wire signed [15:0] settle_delta=measured_angle-settle_anchor[axis];
 wire hold_allowed=!test_driving && (test_active || stop_pending[axis] || !visual_run || !spot_valid || (error<=20 && error>=-20));
 wire [3:0] hold_samples=(test_active || stop_pending[axis]) ? 4'd2 : 4'd11;
 assign request_valid=state==READ || state==WRITE;
 assign command=state==READ ? (emergency ? 8'd2 : 8'd0) : write_command;
 assign value=state==WRITE ? write_value : 16'd0;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   diag_tick<=0;diag_seq[0]<=0;diag_seq[1]<=0;
   sample_valid<=0;sample_data<=0;diag_pending<=0;measured_current<=0;
  end else begin
   diag_tick<=diag_tick+1'b1;
   sample_valid<=0;diag_pending<=state==RAMP;
   if(state==WAIT_READ && response) measured_current<=current;
   if(state==PLAN) begin
    sample_data<={8'd0,measured_y,measured_x,measured_current,measured_speed,measured_angle,
                  16'd0,desired,{{2{error[13]}},error},
                  disabled,fault,stop_pending[axis],velocity_mode[axis],want_track,ready,run,spot_valid,
                  test_active,test_stage[2:0],1'b0,test_profile,test_stage[3],axis,diag_seq[axis]+16'd1,diag_tick};
   end
   if(diag_pending) begin
    sample_valid<=1;diag_seq[axis]<=diag_seq[axis]+1'b1;
    sample_data[191:184]<=state==WRITE ? write_command : 8'd0;
    sample_data[111:96]<=state==WRITE && write_command==4 ? write_value : 16'd0;
   end
  end
 end
 always @* begin
  want_track=tracking[axis];
  if(!visual_run || !spot_valid || (error<=12 && error>=-12)) want_track=0;
  else if((error>20 || error< -20) && start_count[axis]>=1 && start_negative[axis]==(error<0)) want_track=1;
  // B12: stronger near/mid-field response, same taper to zero and 3 rpm cap.
  // 20/30/40/60/100 px -> raw 4/10/15/21/36 (0.12/0.31/0.46/0.64/1.10 rpm).
  magnitude=error<0 ? -error : error;
  if(magnitude<=12) gain=0;
  else if(magnitude<=20) gain=(magnitude-13'd12)>>1;
  else if(magnitude<=40) gain=8'd4+{4'd0,near_scale[7:4]};
  else if(magnitude<=60) gain=8'd15+{5'd0,mid_scale[6:4]};
  else if(magnitude<100) gain=8'd21+{3'd0,outer_scale[7:3]};
  else if(magnitude<=226) gain=8+((magnitude-13'd44)>>1);
  else gain=99;
  // B18: upper-axis fine motion. 20/30/40/60px -> 8/14/19/21 raw
  // (0.24/0.43/0.58/0.64rpm). Join the unchanged outer curve at 60px.
  // Preserve the zero band; no minimum-speed jump near the centre.
  if(axis && visual_run) begin
   if(magnitude>12 && magnitude<=20) gain=magnitude-13'd12;
   else if(magnitude>20 && magnitude<=40) gain=8'd8+{4'd0,near_scale[7:4]};
   else if(magnitude>40 && magnitude<=60) gain=8'd19+{6'd0,delta40[4:3]};
  end
  // B20: retain the entire <=40px fine/stop region. Above it, smoothly add
  // up to 25% of the B19 base speed; same 99-unit cap and acceleration limits.
  // Below 136px, the seven-bit modulo delta is exactly magnitude-40 (0..95).
  // At/above 136px, the distance allowance is already >= the largest extra (24).
  chase_extra=0;
  if(visual_run && magnitude>40) begin
   chase_extra=gain>>2;
   if(magnitude<136 && chase_extra>chase_delta[6:2]) chase_extra=chase_delta[6:2];
   gain=gain+{3'd0,chase_extra};
   if(gain>99)gain=99;
  end
  desired=!want_track ? 16'sd0 : error<0 ? -$signed({8'd0,gain_reg}) : $signed({8'd0,gain_reg});
  // Keep a one-unit crawl outside the stop band despite integer truncation.
  if(want_track && error>12 && desired==0) desired=1;
  if(want_track && error< -12 && desired==0) desired=-1;
  // Startup from angle hold: +50% for 200ms, +25% for 120ms, +12.5%
  // for 80ms. Mid-field then follows the base curve, independent of rpm.
  // Entering the near zone, reversing, or losing the target cancels boost.
  tilt_start_now=axis && visual_run && want_track && !stop_pending[1] &&
                 !velocity_mode[1] && magnitude>40;
  tilt_boost=0;
  if(magnitude>40 && (tilt_start_now || (tilt_start_ticks!=0 && tilt_start_negative==(error<0)))) begin
   if(tilt_start_now || tilt_start_ticks>=6) tilt_boost=gain_reg>>1;
   else if(tilt_start_ticks>=3) tilt_boost=gain_reg>>2;
   else tilt_boost=gain_reg>>3;
  end
  // Brake only within 40px and above base+4 raw (~0.12rpm headroom).
  // Correct half the excess speed, not half the entire measured speed:
  // a motor that already lags its target must keep the full base command.
  // Retain the distance fade and half-base cap to prevent a reverse kick.
  toward_rate=0;
  if((error>0 && tilt_rate>0) || (error<0 && tilt_rate<0))
   toward_rate=tilt_rate<0 ? -tilt_rate : tilt_rate;
  tilt_base=gain_reg==0 ? 8'd1 : gain_reg;
  tilt_brake_threshold={2'd0,tilt_base}+10'd4;
  tilt_excess=toward_rate>tilt_brake_threshold ? toward_rate-tilt_brake_threshold : 10'd0;
  tilt_brake_room=0;tilt_brake=0;
  if(magnitude>12 && magnitude<40) begin
   tilt_brake_room=(13'd40-magnitude)>>1;
   tilt_brake=tilt_excess>>1;
   if(tilt_brake>tilt_brake_room) tilt_brake={3'd0,tilt_brake_room};
   if(tilt_brake>(tilt_base>>1)) tilt_brake=tilt_base>>1;
  end
  tilt_drive=$signed({3'd0,tilt_base})+$signed({3'd0,tilt_boost})-
             $signed({3'd0,tilt_brake});
  if(axis && visual_run && want_track) begin
   if(tilt_drive<=0) desired=0;
   else if(tilt_drive>=99) desired=error<0 ? -16'sd99 : 16'sd99;
   else desired=error<0 ? -$signed({5'd0,tilt_drive}) : $signed({5'd0,tilt_drive});
  end
  if(test_active) desired=testing_axis ? test_speed : 16'sd0;
  // Explicit off completes its stop. Tracking reversal passes through zero
  // and measured low speed without entering position hold or a dwell.
  if(stop_pending[axis] ||
     (velocity_mode[axis] && ((desired>0 && (velocity[axis]<0 || moving_negative)) ||
                              (desired<0 && (velocity[axis]>0 || moving_positive))))) begin
   desired=0;
  end
  lo=axis ? -5461 : -10922;hi=axis ? 21845 : 10922;
  // Reduce outward speed approaching limits, stop 2 degrees before limit.
  remaining=desired>=0 ? hi-$signed(measured)-364 : $signed(measured)-lo-364;
  limit_speed=remaining>0 ? remaining/8 : 0;
  if(desired>0 && desired>limit_speed) desired=limit_speed;
  if(desired<0 && -desired>limit_speed) desired=-limit_speed;
  slew=((velocity[axis]>=0 && desired_reg>velocity[axis]) ||
        (velocity[axis]<=0 && desired_reg<velocity[axis])) ?
          (axis && visual_run ? 16'sd12 : 16'sd8) : 16'sd16;
  if(velocity[axis]<desired_reg-slew) next_velocity=velocity[axis]+slew;
  else if(velocity[axis]>desired_reg+slew) next_velocity=velocity[axis]-slew;
  else next_velocity=desired_reg;
  if((measured>=hi-364 && next_velocity>0) ||
     (measured<=lo+364 && next_velocity<0)) next_velocity=0;
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   state<=IDLE;timer<=0;axis<=0;fault<=0;tracking<=0;velocity_mode<=0;disable_sent<=0;
   stop_pending<=0;start_negative<=0;start_count[0]<=0;start_count[1]<=0;settle_anchor[0]<=0;settle_anchor[1]<=0;
   velocity[0]<=0;velocity[1]<=0;settled[0]<=0;settled[1]<=0;stop_ticks[0]<=0;stop_ticks[1]<=0;
   measured<=0;measured_speed<=0;measured_angle<=0;desired_reg<=0;write_command<=0;write_value<=0;read_disable<=0;
   error<=0;gain_reg<=0;measured_x<=0;measured_y<=0;
   previous_angle[0]<=0;previous_angle[1]<=0;previous_speed[0]<=0;previous_speed[1]<=0;
   history_valid<=0;moving_positive<=0;moving_negative<=0;
   tilt_previous<=0;tilt_older<=0;tilt_previous_tick<=0;tilt_older_tick<=0;
   tilt_history<=0;tilt_delta<=0;tilt_rate<=0;
   tilt_start_ticks<=0;tilt_start_negative<=0;
  end else begin
   if(!ready || emergency) begin tilt_history<=0;tilt_delta<=0;tilt_rate<=0;tilt_start_ticks<=0;end
   if(!emergency) disable_sent<=0;
   if(timer!=0) timer<=timer-1'b1;
   // Loss of readiness cannot leave a velocity command running.
   if(!ready && velocity_mode!=0 && !disabled) fault<=1;
   if(test_timeout) fault<=1;
   case(state)
    IDLE: if(timer==0 && (emergency || ready)) begin
     timer<=CLK_HZ/50-1;axis<=!axis;
     if(!emergency || !disable_sent[!axis]) state<=READ;
    end
    READ: if(accepted) begin read_disable<=emergency;state<=WAIT_READ;end
    WAIT_READ: if(response) begin
     if(read_disable) begin
      disable_sent[axis]<=result==0 && !enabled;velocity[axis]<=0;
      velocity_mode[axis]<=0;tracking[axis]<=0;settled[axis]<=0;stop_ticks[axis]<=0;state<=IDLE;
      stop_pending[axis]<=0;start_count[axis]<=0;settle_anchor[axis]<=0;
     end else if(emergency) state<=IDLE;
     else if(result!=0 || !enabled || !ready || outside) begin fault<=1;state<=IDLE;end
     else begin
      measured<=feedback_offset;measured_angle<=angle;measured_speed<=speed;state<=GAIN;
      measured_x<=x;measured_y<=y;error<=(axis?invert_tilt:invert_pan) ? -raw_error : raw_error;
      // A single speed spike cannot interrupt tracking. Require two consecutive
      // reports AND encoder displacement in that direction (~40 ms interval).
      moving_positive<=history_valid[axis] && speed>10 && previous_speed[axis]>10 && encoder_delta>2;
      moving_negative<=history_valid[axis] && speed< -10 && previous_speed[axis]< -10 && encoder_delta< -2;
      previous_angle[axis]<=angle;previous_speed[axis]<=speed;history_valid[axis]<=1;
      if(axis) begin
       tilt_previous<=angle;tilt_older<=tilt_previous;
       tilt_previous_tick<=diag_tick;tilt_older_tick<=tilt_previous_tick;
       if(tilt_history<2) tilt_history<=tilt_history+1'b1;
       // Discard stale/short intervals; never differentiate across a pause.
       if(tilt_history==2 && tilt_interval>=(CLK_HZ/1000)*72 && tilt_interval<=(CLK_HZ/1000)*88) begin
        if(tilt_delta_80>512) tilt_delta<=11'sd512;
        else if(tilt_delta_80< -512) tilt_delta<=-11'sd512;
        else tilt_delta<=tilt_delta_80[10:0];
       end else tilt_delta<=0;
      end
     end
    end
    GAIN: begin
     gain_reg<=gain;state<=PLAN;
     if(axis) tilt_rate<=tilt_delta<0 ? -$signed(tilt_scaled[12:3]) : $signed(tilt_scaled[12:3]);
    end
    PLAN: begin
     tracking[axis]<=want_track;desired_reg<=desired;state<=RAMP;
     if(axis) begin
      if(emergency || !ready || !visual_run || !want_track || stop_pending[1] || magnitude<=40)
       tilt_start_ticks<=0;
      else if(tilt_start_now) begin tilt_start_ticks<=9;tilt_start_negative<=error<0;end
      else if(tilt_start_negative!=(error<0)) tilt_start_ticks<=0;
      else if(tilt_start_ticks!=0) tilt_start_ticks<=tilt_start_ticks-1'b1;
     end
     if(!visual_run || !spot_valid || stop_pending[axis] || (error<=20 && error>=-20)) start_count[axis]<=0;
     else if(start_negative[axis]!=(error<0)) begin start_negative[axis]<=error<0;start_count[axis]<=1;end
     else if(start_count[axis]<2) start_count[axis]<=start_count[axis]+1'b1;
     if(velocity_mode[axis] && !visual_run && !test_driving) stop_pending[axis]<=1;
    end
    RAMP: begin
     if(emergency || !ready) state<=IDLE;
     else begin
      velocity[axis]<=next_velocity;
      if(next_velocity!=0 || desired_reg!=0) begin
       settled[axis]<=0;stop_ticks[axis]<=0;
       write_command<=4;write_value<=next_velocity;state<=WRITE;
      end else if(velocity_mode[axis]) begin
       // Centre/loss holds require ~480ms stable position; explicit off ~120ms.
       // During reversal only send zero until motion can resume, never hold.
       if(velocity[axis]==0 && measured_speed<=10 && measured_speed>=-10 && hold_allowed &&
          (settled[axis]==0 || (settle_delta<=18 && settle_delta>=-18))) begin
        if(settled[axis]==0) settle_anchor[axis]<=measured_angle;
        if(settled[axis]>=hold_samples) begin
         write_command<=5;write_value<=measured_angle;state<=WRITE;
        end else begin
         settled[axis]<=settled[axis]+1'b1;
         write_command<=4;write_value<=0;state<=WRITE;
        end
       end else begin
        settled[axis]<=0;write_command<=4;write_value<=0;state<=WRITE;
       end
       if(test_driving && test_speed==0) stop_ticks[axis]<=0;
       else if(stop_ticks[axis]==49) begin fault<=1;state<=IDLE;end
       else stop_ticks[axis]<=stop_ticks[axis]+1'b1;
      end else state<=IDLE;
     end
    end
    WRITE: if(accepted) state<=WAIT_WRITE;
    WAIT_WRITE: if(response) begin
     if(result!=0 || !enabled) fault<=1;
     else begin
      velocity_mode[axis]<=write_command==4;
      if(write_command==5) begin
       settled[axis]<=0;stop_ticks[axis]<=0;stop_pending[axis]<=0;
       start_count[axis]<=0;tracking[axis]<=0;
      end
     end
     state<=IDLE;
    end
    default:state<=IDLE;
   endcase
  end
 end
endmodule

