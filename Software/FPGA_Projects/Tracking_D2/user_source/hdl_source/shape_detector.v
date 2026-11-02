// E7 robust stationary printed-shape detector (E2 image identity retained).
// One sampled ROI RAM, alternating acquisition/analysis frames. Flood fill
// separates 4-connected colour components; only the largest solid blob is fitted.
// Bounded queue and processing deadline fail closed. No full video-frame buffer.
module shape_detector #(parameter X0=320,X1=959,Y0=180,Y1=539)(
 input wire clk,rst_n,enable,auto_color,drawing,input wire [1:0] select_color,
 input wire [7:0] threshold,input wire vs,de,input wire [23:0] rgb,
 output reg toggle,valid,output reg [1:0] kind,color,
 output reg [3:0] count,output reg [191:0] vertices,
 output reg [11:0] cx,cy,width,height,output reg [31:0] diagnostic);
 localparam GW=(X1-X0+1)/4, GH=(Y1-Y0+1)/4, CELLS=GW*GH;
 reg [11:0] x,y;reg vd,dd,capturing;
 wire fs=vs&&!vd;
 wire [7:0] r=rgb[23:16],g=rgb[15:8],b=rgb[7:0];
 wire [7:0] vmax=r>g?(r>b?r:b):(g>b?g:b);
 wire [7:0] vmin=r<g?(r<b?r:b):(g<b?g:b);
 wire [7:0] chroma=vmax-vmin;
 wire [7:0] margin=(vmax>>4)<6?8'd6:(vmax>>4);
 wire chromatic=vmax>=32 && chroma>=12 && {3'b0,chroma,3'b0}>={6'b0,vmax};
 wire red=chromatic&&{1'b0,r}>={1'b0,g}+margin&&{1'b0,r}>={1'b0,b}+margin;
 wire green=chromatic&&{1'b0,g}>={1'b0,r}+margin&&{1'b0,g}>={1'b0,b}+margin;
 wire blue=chromatic&&{1'b0,b}>={1'b0,r}+margin&&{1'b0,b}>={1'b0,g}+margin;
 wire [2:0] ink=red?1:green?2:blue?3:0;
 wire [9:0] lum={2'b0,r}+{1'b0,g,1'b0}+{2'b0,b};
 wire [2:0] sample_color=auto_color?ink:select_color==0?(lum<({2'b0,threshold}<<2)?3'd4:3'd0):
     ink=={1'b0,select_color}?ink:3'd0;
 wire sample=enable&&capturing&&de&&x>=X0&&x<=X1&&y>=Y0&&y<=Y1&&
     ((x-X0)&3)==0&&((y-Y0)&3)==0;
 reg [13:0] captured,scan_addr,read_addr,write_addr;
 reg [2:0] pixels[0:CELLS-1];reg [2:0] read_data;reg erase;
 // Independent synchronous read and write ports, infer block RAM. No RAM reset.
 always @(posedge clk)begin
   read_data<=pixels[read_addr];
   if(sample)pixels[captured]<=sample_color;
   else if(erase)pixels[write_addr]<=0;
 end
 reg [14:0] queue[0:1023];reg [9:0] head,tail;reg [10:0] queued;
 reg [14:0] qread;reg push;reg [14:0] push_point;
 always @(posedge clk)begin qread<=queue[head];if(push)queue[tail]<=push_point;end
 reg [7:0] sx,px,nx;reg [6:0] sy,py,ny;reg [2:0] component_color;
 reg [1:0] neighbor;reg [6:0] state;
 reg bad,done;
 reg [14:0] hits,largest;reg [7:0] xmin,xmax,bxmin,bxmax;reg [6:0] ymin,ymax,bymin,bymax;
 wire [7:0] bw=bxmax-bxmin+1;wire [6:0] bh=bymax-bymin+1;
 // Extent of the straight segment on each bounding side. Colour loss at
 // corners must not turn a rectangle into a circle solely by hull area.
 reg [6:0] left_lo,left_hi,right_lo,right_hi;
 reg [7:0] top_lo,top_hi,bottom_lo,bottom_hi;
 reg [6:0] left_span,right_span;reg [7:0] top_span,bottom_span;
 wire flat_left=({3'b0,left_span}<<3)>=({3'b0,bh}-1)*3;
 wire flat_right=({3'b0,right_span}<<3)>=({3'b0,bh}-1)*3;
 wire flat_top=({3'b0,top_span}<<3)>=({3'b0,bw}-1)*3;
 wire flat_bottom=({3'b0,bottom_span}<<3)>=({3'b0,bw}-1)*3;
 wire [2:0] flat_sides={2'b0,flat_left}+{2'b0,flat_right}+{2'b0,flat_top}+{2'b0,flat_bottom};
 reg [2:0] bcolor;
 reg signed [10:0] best[0:7];reg [14:0] point[0:7],saved[0:7],poly[0:7];
 wire signed [10:0] gx=$signed({3'b0,px}),gy=$signed({4'b0,py});
 wire signed [10:0] proj[0:7];
 assign proj[0]=gx;assign proj[1]=gx+gy;assign proj[2]=gy;assign proj[3]=gy-gx;
 assign proj[4]=-gx;assign proj[5]=-gx-gy;assign proj[6]=-gy;assign proj[7]=gx-gy;
 reg [3:0] index,np,corner,remove_at,pass;
 // Search all 70 ordered four-point subsets of the eight original supports.
 // Reuse the existing multiplier; no frame buffer or parallel fit multipliers.
 reg [2:0] qa,qb,qc,qd;reg [1:0] qe;
 reg signed [24:0] quad_area;reg [24:0] quad_best;
 reg [59:0] quad_points;reg quad_tried,quad_testing;
 wire [2:0] qi=qe==0?qa:qe==1?qb:qe==2?qc:qd;
 wire [2:0] qj=qe==0?qb:qe==1?qc:qe==2?qd:qa;
 wire [24:0] quad_abs=quad_area<0?-quad_area:quad_area;
 wire [3:0] i1=corner+1>=np?0:corner+1;
 wire [3:0] i2=i1+1>=np?0:i1+1;
 reg duplicate,orthogonal;
 reg signed [10:0] ex,ey,fx,fy,ma,mb;
 wire signed [21:0] product=ma*mb;
 reg signed [21:0] cross1,cross2,dot1,dot2;
 reg [21:0] norm;
 wire [10:0] aex=ex<0?-ex:ex,aey=ey<0?-ey:ey,afx=fx<0?-fx:fx,afy=fy<0?-fy:fy;
 wire [11:0] len1=aex+aey,len2=afx+afy;
 wire [10:0] short1=aex>aey?aex:aey,short2=afx>afy?afx:afy;
 wire signed [22:0] crossval=$signed(cross1)-$signed(cross2),dotval=$signed(dot1)+$signed(dot2);
 wire [22:0] ac=crossval<0?-crossval:crossval,ad=dotval<0?-dotval:dotval;
 wire [7:0] maxdim=bw>bh?bw:{1'b0,bh};
 wire [7:0] tolerance=maxdim>=24?3:maxdim>=16?2:1;
 reg [17:0] box_area;
 wire [17:0] geometric_box_area=box_area-bw-bh+1;
 reg signed [24:0] area2,support_area2;
 wire [24:0] hull_area2=support_area2<0?-support_area2:support_area2;
 wire quad_covers_hull=quad_best!=0&&({3'b0,quad_best}<<3)>=(({3'b0,hull_area2}<<3)-hull_area2);
 reg [1:0] result_kind;reg result_valid;
 reg [3:0] result_count;reg [191:0] result_points;
 reg [11:0] result_x,result_y,result_w,result_h;
 reg [1:0] result_color;
 reg [1:0] confirmations,misses,previous_kind,previous_color;
 reg [11:0] previous_x,previous_y;
 reg [10:0] frame_config;wire [10:0] config_now={threshold,auto_color,select_color};
 reg draw_seen,draw_lock;
 wire near_previous=result_x+16>=previous_x&&result_x<=previous_x+16&&result_y+16>=previous_y&&result_y<=previous_y+16;
 integer i,j;
 always @* begin
  duplicate=0;
  for(j=0;j<8;j=j+1)begin
   if(j<np&&saved[index[2:0]]==poly[j])duplicate=1;
  end
 end
 // pack via explicitly sized functions (avoid unsized parameter concatenations).
 function [23:0] fullpoint;input [14:0] p;reg [11:0] xx,yy;begin
 xx=X0+({4'b0,p[14:7]}<<2);yy=Y0+({5'b0,p[6:0]}<<2);fullpoint={yy,xx};end endfunction
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin
  x<=0;y<=0;vd<=0;dd<=0;capturing<=1;captured<=0;scan_addr<=0;read_addr<=0;write_addr<=0;erase<=0;
  head<=0;tail<=0;queued<=0;push<=0;push_point<=0;sx<=0;sy<=0;px<=0;py<=0;nx<=0;ny<=0;neighbor<=0;
  state<=0;bad<=0;done<=0;hits<=0;largest<=0;xmin<=255;xmax<=0;ymin<=127;ymax<=0;bxmin<=0;bxmax<=0;bymin<=0;bymax<=0;
  left_lo<=0;left_hi<=0;right_lo<=0;right_hi<=0;top_lo<=0;top_hi<=0;bottom_lo<=0;bottom_hi<=0;
  left_span<=0;right_span<=0;top_span<=0;bottom_span<=0;
  component_color<=0;bcolor<=0;index<=0;np<=0;corner<=0;remove_at<=0;pass<=0;orthogonal<=0;
  ex<=0;ey<=0;fx<=0;fy<=0;ma<=0;mb<=0;cross1<=0;cross2<=0;dot1<=0;dot2<=0;norm<=0;box_area<=0;area2<=0;support_area2<=0;
  diagnostic<=0;toggle<=0;valid<=0;kind<=0;color<=0;count<=0;vertices<=0;cx<=0;cy<=0;width<=0;height<=0;
  qa<=0;qb<=1;qc<=2;qd<=3;qe<=0;quad_area<=0;quad_best<=0;quad_points<=0;quad_tried<=0;quad_testing<=0;
  result_kind<=0;result_valid<=0;result_color<=0;result_count<=0;result_points<=0;result_x<=0;result_y<=0;result_w<=0;result_h<=0;
  confirmations<=0;misses<=0;previous_kind<=0;previous_color<=0;previous_x<=0;previous_y<=0;
  frame_config<=0;draw_seen<=0;draw_lock<=0;
  for(i=0;i<8;i=i+1)begin best[i]<=-1024;point[i]<=0;saved[i]<=0;poly[i]<=0;end
 end else begin
  vd<=vs;dd<=de;erase<=0;push<=0;
  if(de)x<=x+1'b1;else begin x<=0;if(dd)y<=y+1'b1;end
  if(sample&&captured<CELLS)captured<=captured+1'b1;
  if(!drawing)begin draw_seen<=0;draw_lock<=0;end
  // An arm request may race one invalid observation. Stay armed, but dark,
  // until a fresh confirmed candidate exists; do not latch failure forever.
  else if(!draw_seen && valid)begin draw_seen<=1;draw_lock<=1;end
  if(fs)begin
   x<=0;y<=0;toggle<=~toggle;frame_config<=config_now;
   if(!enable||frame_config!=config_now)begin
    state<=0;capturing<=1;captured<=0;valid<=0;done<=0;confirmations<=0;misses<=0;draw_lock<=0;
   end else if(capturing)begin
    captured<=0;
    if(captured==CELLS)begin
     capturing<=0;state<=1;scan_addr<=0;sx<=0;sy<=0;largest<=0;bad<=0;done<=0;
    end else if(!draw_lock)valid<=0;
   end else begin
    // Publish only on a frame boundary. Late processing is an invalid observation.
    diagnostic<={4'b0,draw_lock,!done,bad,result_valid,result_kind,result_color,result_count,1'b0,largest};
    capturing<=1;captured<=0;state<=0;done<=0;
    // Static draw owns the frozen contour until explicit off/reconfiguration.
    // The lit image remains diagnostic; it cannot cancel and restart drawing.
    if(drawing&&draw_lock)begin valid<=1;end
    else if(done&&result_valid&&!bad)begin
     previous_kind<=result_kind;previous_color<=result_color;previous_x<=result_x;previous_y<=result_y;misses<=0;
     if(previous_kind==result_kind&&previous_color==result_color&&near_previous)begin
      if(confirmations<3)confirmations<=confirmations+1'b1;
      // Two observations acquire; changing an existing label needs three.
      if(confirmations>=1&&(!valid||(result_kind==kind&&result_color==color)||confirmations>=2))begin
       valid<=1;
       if(!drawing||!draw_lock)begin kind<=result_kind;color<=result_color;count<=result_count;vertices<=result_points;
        cx<=result_x;cy<=result_y;width<=result_w;height<=result_h;end
      end
     end else begin confirmations<=1;if(!near_previous)valid<=0;end
    end else begin
     confirmations<=0;
     if(misses<3)misses<=misses+1'b1;
     if(misses>=2||bad||!done)valid<=0;
    end
   end
  end else case(state)
   0:begin end
   1:begin read_addr<=scan_addr;state<=2;end
   2:state<=3;
   3:if(read_data!=0)begin
    component_color<=read_data;hits<=0;xmin<=255;xmax<=0;ymin<=127;ymax<=0;
    for(i=0;i<8;i=i+1)best[i]<=-1024;
    head<=0;tail<=0;queued<=1;push<=1;push_point<={sx,sy};erase<=1;write_addr<=scan_addr;state<=4;
   end else state<=18;
   4:begin tail<=1;state<=5;end
   5:state<=6;
   6:begin px<=qread[14:7];py<=qread[6:0];head<=head+1'b1;queued<=queued-1'b1;state<=7;end
   7:begin
    hits<=hits+1'b1;
    if(hits==0||px<xmin)begin left_lo<=py;left_hi<=py;end
    else if(px==xmin)begin if(py<left_lo)left_lo<=py;if(py>left_hi)left_hi<=py;end
    if(hits==0||px>xmax)begin right_lo<=py;right_hi<=py;end
    else if(px==xmax)begin if(py<right_lo)right_lo<=py;if(py>right_hi)right_hi<=py;end
    if(hits==0||py<ymin)begin top_lo<=px;top_hi<=px;end
    else if(py==ymin)begin if(px<top_lo)top_lo<=px;if(px>top_hi)top_hi<=px;end
    if(hits==0||py>ymax)begin bottom_lo<=px;bottom_hi<=px;end
    else if(py==ymax)begin if(px<bottom_lo)bottom_lo<=px;if(px>bottom_hi)bottom_hi<=px;end
    if(px<xmin)xmin<=px;if(px>xmax)xmax<=px;if(py<ymin)ymin<=py;if(py>ymax)ymax<=py;
    for(i=0;i<8;i=i+1)if(proj[i]>best[i])begin best[i]<=proj[i];point[i]<={px,py};end
    neighbor<=0;state<=8;
   end
   8:begin
    nx<=px;ny<=py;
    case(neighbor)
     0:if(px==0)state<=14;else begin nx<=px-1'b1;state<=9;end
     1:if(px==GW-1)state<=14;else begin nx<=px+1'b1;state<=9;end
     2:if(py==0)state<=14;else begin ny<=py-1'b1;state<=9;end
     3:if(py==GH-1)state<=14;else begin ny<=py+1'b1;state<=9;end
    endcase
   end
   9:begin read_addr<=ny*GW+nx;state<=10;end
   10:state<=11;
   11:begin
    if(read_data==component_color)begin
     if(queued>=1023)begin bad<=1;state<=60;end
     else begin push<=1;push_point<={nx,ny};erase<=1;write_addr<=read_addr;queued<=queued+1'b1;state<=12;end
    end else state<=14;
   end
   12:begin tail<=tail+1'b1;state<=14;end
   14:if(neighbor==3)state<=15;else begin neighbor<=neighbor+1'b1;state<=8;end
   15:if(queued!=0)state<=5;else state<=16;
   16:begin
    // An ROI-border background patch is ineligible; do not let it hide
    // a smaller, complete coloured target inside the curtain area.
    if((!drawing||!draw_lock||component_color[1:0]==color)&&hits>largest&&hits>=24&&xmin>0&&ymin>0&&xmax<GW-1&&ymax<GH-1&&xmax-xmin>=4&&ymax-ymin>=4)begin
     left_span<=left_hi-left_lo;right_span<=right_hi-right_lo;top_span<=top_hi-top_lo;bottom_span<=bottom_hi-bottom_lo;
     largest<=hits;bxmin<=xmin;bxmax<=xmax;bymin<=ymin;bymax<=ymax;bcolor<=component_color;
     for(i=0;i<8;i=i+1)saved[i]<=point[i];
    end
    state<=18;
   end
   18:if(scan_addr==CELLS-1)begin state<=20;np<=0;index<=0;quad_tried<=0;quad_testing<=0;end
      else begin scan_addr<=scan_addr+1'b1;if(sx==GW-1)begin sx<=0;sy<=sy+1'b1;end else sx<=sx+1'b1;state<=1;end
   20:begin
    if(!duplicate)begin poly[np[2:0]]<=saved[index[2:0]];np<=np+1'b1;end
    if(index==7)begin state<=50;corner<=0;pass<=0;support_area2<=0;end else index<=index+1'b1;
   end
   21:begin
    ex<=$signed({3'b0,poly[i1[2:0]][14:7]})-$signed({3'b0,poly[corner[2:0]][14:7]});
    ey<=$signed({4'b0,poly[i1[2:0]][6:0]})-$signed({4'b0,poly[corner[2:0]][6:0]});
    fx<=$signed({3'b0,poly[i2[2:0]][14:7]})-$signed({3'b0,poly[i1[2:0]][14:7]});
    fy<=$signed({4'b0,poly[i2[2:0]][6:0]})-$signed({4'b0,poly[i1[2:0]][6:0]});state<=22;
   end
   22:begin ma<=ex;mb<=fy;state<=23;end
   23:begin cross1<=product;ma<=ey;mb<=fx;state<=24;end
   24:begin cross2<=product;state<=25;end
   25:if(np>3&&pass<8&&(short1<=tolerance||short2<=tolerance||ac<=len1+len2))begin
     remove_at<=i1;state<=26;
    end else if(corner+1>=np)begin corner<=0;orthogonal<=1;area2<=0;state<=30;end
    else begin corner<=corner+1'b1;state<=21;end
   26:begin
    for(i=0;i<7;i=i+1)if(i>=remove_at)poly[i]<=poly[i+1];
    np<=np-1'b1;corner<=0;pass<=pass+1'b1;state<=21;
   end
   30:begin
    ex<=$signed({3'b0,poly[i1[2:0]][14:7]})-$signed({3'b0,poly[corner[2:0]][14:7]});
    ey<=$signed({4'b0,poly[i1[2:0]][6:0]})-$signed({4'b0,poly[corner[2:0]][6:0]});
    fx<=$signed({3'b0,poly[i2[2:0]][14:7]})-$signed({3'b0,poly[i1[2:0]][14:7]});
    fy<=$signed({4'b0,poly[i2[2:0]][6:0]})-$signed({4'b0,poly[i1[2:0]][6:0]});state<=31;
   end
   31:begin ma<=ex;mb<=fx;state<=32;end
   32:begin dot1<=product;ma<=ey;mb<=fy;state<=33;end
   33:begin dot2<=product;ma<=len1;mb<=len2;state<=34;end
   34:begin norm<=product;ma<=$signed({3'b0,poly[corner[2:0]][14:7]});mb<=$signed({4'b0,poly[i1[2:0]][6:0]});state<=35;end
   35:begin cross1<=product;ma<=$signed({4'b0,poly[corner[2:0]][6:0]});mb<=$signed({3'b0,poly[i1[2:0]][14:7]});state<=36;end
   36:begin area2<=area2+$signed(cross1)-$signed(product);if(ad>(norm>>2))orthogonal<=0;
    if(corner+1>=np)state<=40;else begin corner<=corner+1'b1;state<=30;end
   end
   40:begin box_area<=bw*bh;state<=41;end
   41:begin
    state<=60;
    result_valid<=0;result_kind<=0;result_color<=bcolor[1:0];result_count<=np;
    for(i=0;i<8;i=i+1)result_points[i*24+:24]<=fullpoint(poly[i]);
    result_x<=X0+(({4'b0,bxmin}+{4'b0,bxmax})<<1);result_y<=Y0+(({5'b0,bymin}+{5'b0,bymax})<<1);
    result_w<=({4'b0,bxmax}-{4'b0,bxmin})<<2;result_h<=({5'b0,bymax}-{5'b0,bymin})<<2;
    if(largest>=24&&bxmin>0&&bymin>0&&bxmax<GW-1&&bymax<GH-1&&bw>=5&&bh>=5&&!bad)begin
     // Raster area must approximately fill the fitted polygon; reject hollow laser lines.
     if(np==3 && largest*4>=(area2<0?-area2:area2) && largest*4<=(area2<0?-area2:area2)*3)begin result_valid<=1;result_kind<=1;end
     else if(!quad_tried&&np>=4&&quad_covers_hull)begin
      for(i=0;i<4;i=i+1)poly[i]<=quad_points[i*15+:15];
      np<=4;quad_tried<=1;quad_testing<=1;corner<=0;orthogonal<=1;area2<=0;state<=30;
     end
     else if(np==4&&orthogonal&&largest*4>=(area2<0?-area2:area2)&&largest*4<=(area2<0?-area2:area2)*3)begin result_valid<=1;result_kind<=2;end
     else if(quad_testing)begin
      // Failed right-angle test: reconstruct original supports and run normal
      // triangle/rectangle/circle logic, rather than forcing a quadrilateral.
      quad_testing<=0;np<=0;index<=0;state<=20;
     end
     // A mostly rectangular HULL stays rectangular even when blue chroma
     // drops out inside it. Pixel occupancy alone cannot distinguish this from a circle.
     // Geometric (not inclusive raster) box avoids a resolution-dependent threshold.
     else if(np>=4&&((hull_area2*16>=geometric_box_area*27&&largest*8>=geometric_box_area*5)||
       (flat_sides>=3&&hull_area2*2>=geometric_box_area*3&&largest*2>=geometric_box_area)))begin
      result_valid<=1;result_kind<=2;result_count<=4;
      result_points[23:0]<=fullpoint({bxmin,bymin});
      result_points[47:24]<=fullpoint({bxmax,bymin});
      result_points[71:48]<=fullpoint({bxmax,bymax});
      result_points[95:72]<=fullpoint({bxmin,bymax});
     end
     else if(np>=5&&hull_area2*16<geometric_box_area*27&&hull_area2*4>=geometric_box_area*5&&
        largest*8>=box_area*5&&largest*8<=box_area*7&&bw<=bh*2&&bh<=bw*2)begin result_valid<=1;result_kind<=3;end
    end
   end
   // Preserve support-hull area BEFORE simplifying corner points. A cut/blurred
   // corner may remove a short edge; it must not make the original hull look round.
   50:begin ma<=$signed({3'b0,poly[corner[2:0]][14:7]});mb<=$signed({4'b0,poly[i1[2:0]][6:0]});state<=51;end
   51:begin cross1<=product;ma<=$signed({4'b0,poly[corner[2:0]][6:0]});mb<=$signed({3'b0,poly[i1[2:0]][14:7]});state<=52;end
   52:begin
    support_area2<=support_area2+$signed(cross1)-$signed(product);
    if(corner+1>=np)begin
     corner<=0;
     if(quad_tried)state<=21;
     else begin qa<=0;qb<=1;qc<=2;qd<=3;qe<=0;quad_area<=0;quad_best<=0;state<=70;end
    end else begin corner<=corner+1'b1;state<=50;end
   end
   70:begin ma<=$signed({3'b0,saved[qi][14:7]});mb<=$signed({4'b0,saved[qj][6:0]});state<=71;end
   71:begin cross1<=product;ma<=$signed({4'b0,saved[qi][6:0]});mb<=$signed({3'b0,saved[qj][14:7]});state<=72;end
   72:begin quad_area<=quad_area+$signed(cross1)-$signed(product);
    if(qe==3)state<=73;else begin qe<=qe+1'b1;state<=70;end
   end
   73:begin
    if(quad_abs>quad_best)begin quad_best<=quad_abs;quad_points<={saved[qd],saved[qc],saved[qb],saved[qa]};end
    state<=74;
   end
   74:begin
    qe<=0;quad_area<=0;state<=70;
    if(qd<7)qd<=qd+1'b1;
    else if(qc<6)begin qc<=qc+1'b1;qd<=qc+2;end
    else if(qb<5)begin qb<=qb+1'b1;qc<=qb+2;qd<=qb+3;end
    else if(qa<4)begin qa<=qa+1'b1;qb<=qa+2;qc<=qa+3;qd<=qa+4;end
    else begin corner<=0;state<=21;end
   end
   60:begin done<=1;state<=0;end
   default:state<=0;
  endcase
  if(draw_seen&&!drawing)begin valid<=0;confirmations<=0;misses<=0;end
 end
 end
endmodule
