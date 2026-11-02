// A8 motion configuration and coherent telemetry mailbox. No motor commands.
// 76 threshold write,77 min tiles write,78/79 read,7A indexed 4-byte snapshot,
// 7B reset reference,7C revision D8. Snapshot 0 latches; byte0 flags,byte1 count,
// bytes2..3 changed reduced samples (a cell covers 25 original pixel positions).
module video_motion_control(
 input wire clk,pixel_clk,rst_n,pixel_vs,
 input wire valid,input wire [7:0] cmd,data,output reg [7:0] reply,
 output wire [7:0] threshold,min_tiles,output wire reset_toggle,
 input wire paused,warming,scene_change,input wire [2:0] boxes_valid,
 input wire [15:0] changed_cells
);
 reg [7:0] threshold_reg,minimum_reg;reg reset_reg;
 reg [16:0] config_meta,config_sync;
 assign threshold=config_sync[7:0],min_tiles=config_sync[15:8],reset_toggle=config_sync[16];
 reg [31:0] mailbox,snapshot,latest;
 reg req,ack;reg [2:0] rs,as;
 reg vs_d;
 wire [7:0] count={7'd0,boxes_valid[0]}+{7'd0,boxes_valid[1]}+{7'd0,boxes_valid[2]};
 always @*begin
  reply=0;
  case(cmd)
   8'h76:reply=data==0?8'd1:data;
   8'h77:reply=data==0?8'd1:data>32?8'd32:data;
   8'h78:reply=threshold_reg;
   8'h79:reply=minimum_reg;
   8'h7a:reply=data==0?latest[7:0]:data<4?snapshot[data*8+:8]:8'hff;
   8'h7b:reply=0;
   8'h7c:reply=8'hd8;
  endcase
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin threshold_reg<=18;minimum_reg<=3;reset_reg<=0;as<=0;ack<=0;latest<=0;snapshot<=0;end
  else begin
   as<={as[1:0],req};
   if(as[2]!=ack)begin latest<=mailbox;ack<=as[2];end
   if(valid)case(cmd)
    8'h76:threshold_reg<=reply;
    8'h77:minimum_reg<=reply;
    8'h7a:if(data==0)snapshot<=latest;
    8'h7b:reset_reg<=~reset_reg;
   endcase
  end
 end
 always @(posedge pixel_clk or negedge rst_n)begin
  if(!rst_n)begin config_meta<={1'b0,8'd3,8'd18};config_sync<={1'b0,8'd3,8'd18};
   rs<=0;req<=0;mailbox<=0;vs_d<=0;end
  else begin
   config_meta<={reset_reg,minimum_reg,threshold_reg};config_sync<=config_meta;
   rs<={rs[1:0],ack};vs_d<=pixel_vs;
   if(vs_d&&!pixel_vs && req==rs[2])begin
    mailbox<={changed_cells,count,4'd0,scene_change,paused,warming,|boxes_valid};req<=~req;
   end
  end
 end
endmodule
