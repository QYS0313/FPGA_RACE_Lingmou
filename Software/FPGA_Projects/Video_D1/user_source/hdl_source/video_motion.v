`timescale 1ns/1ps
// D1 A8: 256x144 temporal difference, one 8-bit reference frame.
// Average a 4x4 aperture in each 5x5 source cell. No RGB/DDR writes.
// A tile is 4x4 reduced samples (20x20 screen pixels), active with >=4 hits.
// Three streaming accumulators join boxes separated by up to two empty tiles.
// This is bounded region grouping, not unlimited connected-component labeling.
module video_motion #(
 parameter WIDTH=1280, HEIGHT=720, HOLD_FRAMES=6, SETTLE_FRAMES=12
)(
 input wire clk,rst_n,vs,de,input wire [23:0] rgb,
 input wire enabled,paused,reset_toggle,
 input wire roi_only,roi_valid,input wire [11:0] roi_x0,roi_y0,roi_x1,roi_y1,
 input wire [7:0] threshold,min_tiles,
 input wire [23:0] frame_stamp,input wire stamp_valid,
 output reg [2:0] boxes_valid,
 output reg [35:0] boxes_x0,boxes_y0,boxes_x1,boxes_y1,
 output wire target_valid,
 output wire [11:0] target_x,target_y,target_w,target_h,
 output reg [15:0] changed_cells,
 output reg warming,scene_change
);
 localparam CW=WIDTH/5, CH=HEIGHT/5, TW=CW/4, CELLS=CW*CH;
 reg [7:0] reference[0:CELLS-1];
 reg [11:0] columns[0:CW-1];
 reg [4:0] tiles[0:TW-1];
 reg [11:0] x,y;
 reg [2:0] xp,yp;
 reg [7:0] cx,cy;
 reg vs_d,de_d,first,run_frame,ref_valid,have_stamp,reset_seen;
 reg [23:0] last_stamp;
 reg [7:0] cooldown,thr,minimum,stale_frames;
 reg [9:0] hsum;
 wire [9:0] gray_sum={2'b0,rgb[23:16]}+{1'b0,rgb[15:8],1'b0}+{2'b0,rgb[7:0]};
 wire [7:0] gray=gray_sum[9:2];
 wire [9:0] htotal=hsum+gray;
 reg p1,p2,p3;
 reg [11:0] old_col;
 reg [9:0] row_sum;
 reg [7:0] ax,ay,bx,by;
 reg [2:0] yp_saved;
 wire [11:0] total=(yp_saved==0) ? {2'b0,row_sum} : old_col+row_sum;
 reg [15:0] addr;
 reg [7:0] prev_gray,new_gray;
 reg compare_valid;
 wire [7:0] diff=new_gray>=prev_gray ? new_gray-prev_gray : prev_gray-new_gray;
 wire [11:0] sample_x={4'd0,bx}*5+2,sample_y={4'd0,by}*5+2;
 wire roi_hit=!roi_only||(roi_valid&&sample_x>=roi_x0&&sample_x<=roi_x1&&sample_y>=roi_y0&&sample_y<=roi_y1);
 reg [15:0] hits,eligible;
 wire broad_change=hits>(eligible>>2);
 reg [15:0] samples;
 reg [4:0] old_tile;
 reg hit;
 reg [7:0] tx_sample,ty_sample;
 wire [5:0] tile_total=(ty_sample[1:0]==0 && tx_sample[1:0]==0 ? 6'd0 : {1'b0,old_tile})+hit;
 wire tile_event=p3 && tx_sample[1:0]==3 && ty_sample[1:0]==3 && tile_total>=4;
 wire [5:0] tile_x=tx_sample[7:2],tile_y=ty_sample[7:2];
 reg [2:0] used;
 reg [5:0] left[0:2],top[0:2],right[0:2],bottom[0:2];
 reg [11:0] count[0:2];
 reg [2:0] match_mask,qualified;
 reg [5:0] ml,mt,mr,mb;
 reg [13:0] mc;
 integer i,slot,largest;
 reg [11:0] largest_count;
 reg [1:0] confirmed;
 reg [7:0] missing;
 reg [1:0] biggest;
 assign target_valid=|boxes_valid;
 wire [11:0] lx=boxes_x0[biggest*12+:12],ly=boxes_y0[biggest*12+:12];
 wire [11:0] rx=boxes_x1[biggest*12+:12],ry=boxes_y1[biggest*12+:12];
 assign target_x=({1'b0,lx}+{1'b0,rx})>>1;
 assign target_y=({1'b0,ly}+{1'b0,ry})>>1;
 assign target_w=rx-lx+1'b1,target_h=ry-ly+1'b1;
 always @* begin
  match_mask=0;slot=-1;ml=tile_x;mr=tile_x;mt=tile_y;mb=tile_y;mc=1;
  qualified=0;largest=0;largest_count=0;
  for(i=0;i<3;i=i+1) begin
   if(used[i] && count[i]>=minimum) begin
    qualified[i]=1;
    if(count[i]>largest_count) begin largest=i;largest_count=count[i];end
   end
   if(used[i] && {1'b0,tile_x}+3>={1'b0,left[i]} && {1'b0,tile_x}<={1'b0,right[i]}+3 &&
      {1'b0,tile_y}+3>={1'b0,top[i]} && {1'b0,tile_y}<={1'b0,bottom[i]}+3) begin
    match_mask[i]=1;if(slot<0)slot=i;
    if(left[i]<ml)ml=left[i];if(right[i]>mr)mr=right[i];
    if(top[i]<mt)mt=top[i];if(bottom[i]>mb)mb=bottom[i];mc=mc+count[i];
   end
  end
  if(slot<0)for(i=2;i>=0;i=i-1)
   if(!used[i] || (count[i]<minimum && {1'b0,tile_y}>{1'b0,bottom[i]}+3))slot=i;
 end
 // Synchronous read/write RAMs, intentionally not reset.
 always @(posedge clk) begin
  if(de && run_frame && xp==3 && yp<4)old_col<=columns[cx];
  if(p1)columns[ax]<=total;
  if(p1 && yp_saved==3) begin
   prev_gray<=reference[ay*CW+ax];
   reference[ay*CW+ax]<=total[11:4];
  end
  if(p2)old_tile<=tiles[bx[7:2]];
  if(p3)tiles[tx_sample[7:2]]<=tile_total[4:0];
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   x<=0;y<=0;xp<=0;yp<=0;cx<=0;cy<=0;vs_d<=0;de_d<=0;first<=1;
   run_frame<=0;ref_valid<=0;have_stamp<=0;last_stamp<=0;reset_seen<=0;cooldown<=0;stale_frames<=0;
   thr<=18;minimum<=3;hsum<=0;p1<=0;p2<=0;p3<=0;row_sum<=0;
   ax<=0;ay<=0;bx<=0;by<=0;yp_saved<=0;addr<=0;new_gray<=0;compare_valid<=0;
   hit<=0;tx_sample<=0;ty_sample<=0;hits<=0;eligible<=0;samples<=0;used<=0;
   boxes_valid<=0;boxes_x0<=0;boxes_y0<=0;boxes_x1<=0;boxes_y1<=0;
   changed_cells<=0;warming<=1;scene_change<=0;confirmed<=0;missing<=0;biggest<=0;
   for(i=0;i<3;i=i+1)begin left[i]<=0;top[i]<=0;right[i]<=0;bottom[i]<=0;count[i]<=0;end
  end else begin
   vs_d<=vs;de_d<=de;
   p1<=de && run_frame && xp==3 && yp<4;
   p2<=p1 && yp_saved==3;p3<=p2;
   if(de) begin
    if(xp==0)hsum<=gray;else if(xp<4)hsum<=hsum+gray;
    if(xp==3)begin row_sum<=htotal;ax<=cx;ay<=cy;yp_saved<=yp;end
    if(xp==4)begin xp<=0;cx<=cx+1'b1;end else xp<=xp+1'b1;
    x<=x+1'b1;
   end
   if(de_d&&!de)begin x<=0;xp<=0;cx<=0;y<=y+1'b1;
    if(yp==4)begin yp<=0;cy<=cy+1'b1;end else yp<=yp+1'b1;
   end
   if(p1 && yp_saved==3)begin new_gray<=total[11:4];bx<=ax;by<=ay;compare_valid<=ref_valid;end
   if(p2)begin
    samples<=samples+1'b1;
    if(compare_valid && roi_hit)eligible<=eligible+1'b1;
    hit<=compare_valid && diff>=thr && roi_hit;
    tx_sample<=bx;ty_sample<=by;
    if(compare_valid && diff>=thr && roi_hit)hits<=hits+1'b1;
   end
   if(tile_event && slot>=0)begin
    for(i=0;i<3;i=i+1)if(match_mask[i])begin used[i]<=0;count[i]<=0;end
    used[slot]<=1;left[slot]<=ml;right[slot]<=mr;top[slot]<=mt;bottom[slot]<=mb;
    count[slot]<=mc>4095 ? 12'd4095 : mc[11:0];
   end
   if(vs&&!vs_d) begin
    x<=0;y<=0;xp<=0;yp<=0;cx<=0;cy<=0;first<=1;run_frame<=0;
    if(run_frame && samples==CELLS)begin
     ref_valid<=1;warming<=!ref_valid;changed_cells<=hits;
     scene_change<=ref_valid && broad_change;
     if(ref_valid && !broad_change && |qualified)begin
      if(confirmed<2)confirmed<=confirmed+1'b1;
      missing<=0;
      if(confirmed>=1)begin
       boxes_valid<=qualified;biggest<=largest;
       for(i=0;i<3;i=i+1)begin
        boxes_x0[i*12+:12]<=left[i]*20;boxes_y0[i*12+:12]<=top[i]*20;
        boxes_x1[i*12+:12]<=(right[i]+12'd1)*20-1;
        boxes_y1[i*12+:12]<=(bottom[i]+12'd1)*20-1;
       end
      end
     end else begin
      confirmed<=0;
      if(broad_change || missing>=HOLD_FRAMES-1)boxes_valid<=0;
      else missing<=missing+1'b1;
     end
    end
    if(run_frame && samples!=CELLS)begin ref_valid<=0;boxes_valid<=0;confirmed<=0;warming<=1;end
    hits<=0;eligible<=0;samples<=0;used<=0;
   end
   if(first&&de)begin
    first<=0;thr<=threshold==0?8'd1:threshold;minimum<=min_tiles==0?8'd1:min_tiles;
    if(enabled&&!paused&&cooldown==0&&stamp_valid)begin
     if(!have_stamp||frame_stamp!=last_stamp)begin
      run_frame<=1;have_stamp<=1;last_stamp<=frame_stamp;stale_frames<=0;
     end else if(stale_frames<119)stale_frames<=stale_frames+1'b1;
     else begin boxes_valid<=0;ref_valid<=0;confirmed<=0;warming<=1;end
    end
    if(!stamp_valid)begin
      ref_valid<=0;have_stamp<=0;boxes_valid<=0;confirmed<=0;warming<=1;
     end
    if(cooldown!=0)cooldown<=cooldown-1'b1;
   end
   // Invalid/disabled streams must never compare partial or old reference frames.
   if(!enabled||paused||reset_seen!=reset_toggle)begin
    run_frame<=0;ref_valid<=0;have_stamp<=0;boxes_valid<=0;warming<=1;scene_change<=0;
    p1<=0;p2<=0;p3<=0;confirmed<=0;missing<=0;hits<=0;eligible<=0;samples<=0;used<=0;
    cooldown<=paused ? SETTLE_FRAMES : 0;reset_seen<=reset_toggle;stale_frames<=0;
   end
  end
 end
endmodule

// Overlay after existing effects, one clock with identical RGB/sync latency.
module video_motion_overlay(
 input wire clk,rst_n,enabled,vs,hs,de,input wire [23:0] rgb,
 input wire [2:0] boxes_valid,input wire [35:0] x0,y0,x1,y1,
 output reg out_vs,out_hs,out_de,output reg [23:0] out_rgb
);
 reg [11:0] x,y;reg de_d;integer j;reg border;
 reg [11:0] l,t,r,b;
 always @*begin
  border=0;l=0;t=0;r=0;b=0;
  for(j=0;j<3;j=j+1)begin
   l=x0[j*12+:12];t=y0[j*12+:12];r=x1[j*12+:12];b=y1[j*12+:12];
   if(boxes_valid[j]&&x>=l&&x<=r&&y>=t&&y<=b &&
     (x<l+2||x+2>r||y<t+2||y+2>b))border=1;
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin x<=0;y<=0;de_d<=0;out_vs<=0;out_hs<=0;out_de<=0;out_rgb<=0;end
  else begin
   de_d<=de;out_vs<=vs;out_hs<=hs;out_de<=de;
   out_rgb<=enabled&&de&&border ? 24'h00ffff : rgb;
   if(vs)begin x<=0;y<=0;end
   else if(de)x<=x+1'b1;
   else if(de_d)begin x<=0;y<=y+1'b1;end
  end
 end
endmodule
