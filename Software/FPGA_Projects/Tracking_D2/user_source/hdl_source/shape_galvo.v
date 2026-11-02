// Independent contour renderer. One affine multiplier, full bounding-box
// preflight, one frozen frame per closed stroke, immediate stale blanking.
module shape_galvo #(parameter integer CLK_HZ=100000000,DAC_MIN=28768,DAC_MAX=36768)(
 input wire clk,rst_n,input wire [255:0] config_data,input wire [1:0] config_mode,input wire config_toggle,
 input wire spot_valid,input wire [11:0] spot_x,spot_y,
 input wire shape_enable,shape_valid,allow_full_range,input wire [1:0] shape_kind,input wire [3:0] shape_count,
 input wire [191:0] shape_vertices,input wire [11:0] shape_x,shape_y,shape_w,shape_h,
 input wire dac_ready,dac_busy,input wire [15:0] applied_x,applied_y,
 output reg [15:0] target_x,target_y,output wire laser,output reg range_fault);
 reg [255:0] cfg;reg [1:0] mode;reg seen;
 reg [5:0] state;reg [16:0] timer;reg [19:0] blank;
 reg lit,path_ok,probing,shape_saved,full_saved,probe_clipped;reg [1:0] probe,kind;
 reg [3:0] count,edge_index;reg [5:0] step_index;
 reg [7:0] phase;reg [191:0] points;
 reg [11:0] cx,cy,ww,hh,xlo,xhi,ylo,yhi;
 reg signed [12:0] px,py;
 reg signed [36:0] ax,by,dx,ey;reg signed [38:0] mx,my;
 reg signed [23:0] ma;reg signed [12:0] mb;
 wire signed [36:0] product=ma*mb;
 wire signed [23:0] a=cfg[23:0],b=cfg[55:32],d=cfg[119:96],e=cfg[151:128];
 wire signed [31:0] c=cfg[95:64],f=cfg[191:160];
 wire [7:0] radius=cfg[231:224]<4?8'd4:cfg[231:224]>64?8'd64:cfg[231:224];
 wire [12:0] advance=radius*phase[4:0];
 wire green_ok=spot_valid&&spot_x>=radius&&spot_x+radius<1280&&spot_y>=radius&&spot_y+radius<720;
 wire input_ok=shape_enable?(shape_valid&&shape_count>=3&&shape_count<=8):green_ok;
 wire dac_code_ok=mx>=0&&mx<=65535&&my>=0&&my<=65535;
 wire mapped_ok=mx>=DAC_MIN&&mx<=DAC_MAX&&my>=DAC_MIN&&my<=DAC_MAX;
 wire [3:0] next_edge=edge_index+1>=count?0:edge_index+1'b1;
 wire [23:0] p0=points[edge_index*24+:24],p1=points[next_edge*24+:24];
 wire signed [12:0] vx=$signed({1'b0,p1[11:0]})-$signed({1'b0,p0[11:0]});
 wire signed [12:0] vy=$signed({1'b0,p1[23:12]})-$signed({1'b0,p0[23:12]});
 wire signed [19:0] ix=vx*$signed({1'b0,step_index}),iy=vy*$signed({1'b0,step_index});
 function [8:0] sine_q;
 input [5:0] q;
 begin case(q)
 0:sine_q=0;1:sine_q=13;2:sine_q=25;3:sine_q=38;4:sine_q=50;5:sine_q=62;6:sine_q=74;7:sine_q=86;
 8:sine_q=98;9:sine_q=109;10:sine_q=121;11:sine_q=132;12:sine_q=142;13:sine_q=152;14:sine_q=162;15:sine_q=172;
 16:sine_q=181;17:sine_q=190;18:sine_q=198;19:sine_q=206;20:sine_q=213;21:sine_q=220;22:sine_q=226;23:sine_q=231;
 24:sine_q=237;25:sine_q=241;26:sine_q=245;27:sine_q=248;28:sine_q=251;29:sine_q=253;30:sine_q=255;31:sine_q=256;
 default:sine_q=256;endcase end endfunction
 wire [8:0] sin_u=sine_q({1'b0,phase[4:0]}),cos_u=sine_q(32-{1'b0,phase[4:0]});
 wire [8:0] frac_x=phase[5]?sin_u:cos_u,frac_y=phase[5]?cos_u:sin_u;
 wire [20:0] circ_x=ww*frac_x,circ_y=hh*frac_y;
 assign laser=rst_n&&dac_ready&&lit&&path_ok&&blank==0&&shape_enable==shape_saved&&
   ((mode==1&&input_ok&&(!shape_saved||allow_full_range==full_saved))||mode==2);
 always @* begin
  ma=a;mb=px;
  case(state)
   11:begin ma=b;mb=py;end
   12:begin ma=d;mb=px;end
   13:begin ma=e;mb=py;end
  endcase
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   cfg<=0;mode<=0;seen<=0;state<=0;timer<=0;blank<=0;lit<=0;path_ok<=0;probing<=0;probe<=0;
   full_saved<=0;probe_clipped<=0;range_fault<=0;shape_saved<=0;kind<=0;count<=0;edge_index<=0;step_index<=0;phase<=0;points<=0;
   cx<=0;cy<=0;ww<=0;hh<=0;xlo<=0;xhi<=0;ylo<=0;yhi<=0;
   px<=0;py<=0;ax<=0;by<=0;dx<=0;ey<=0;mx<=0;my<=0;target_x<=32768;target_y<=32768;
  end else if(config_toggle!=seen)begin
   range_fault<=0;seen<=config_toggle;cfg<=config_data;mode<=config_mode;state<=0;timer<=0;lit<=0;path_ok<=0;blank<=CLK_HZ/500;
  end else begin
   if(timer!=0)timer<=timer-1'b1;if(blank!=0)blank<=blank-1'b1;
   if((mode!=1&&mode!=2)||(mode==1&&!input_ok)||!dac_ready)begin
    state<=0;lit<=0;path_ok<=0;timer<=0;blank<=CLK_HZ/500;
   end else if(state!=0&&mode==1&&(shape_saved!=shape_enable||(shape_saved&&full_saved!=allow_full_range)))begin state<=0;lit<=0;path_ok<=0;end
   else case(state)
    0:begin
     shape_saved<=shape_enable;full_saved<=allow_full_range;probe_clipped<=0;kind<=shape_kind;count<=shape_count;points<=shape_vertices;
     cx<=shape_enable?shape_x:spot_x;cy<=shape_enable?shape_y:spot_y;
     ww<=shape_w;hh<=shape_h;phase<=0;edge_index<=0;step_index<=0;
     lit<=0;path_ok<=1;blank<=CLK_HZ/500;probe<=0;probing<=1;state<=1;
    end
    1:begin
     xlo<=shape_saved?cx-(ww>>1):cx-radius;xhi<=shape_saved?cx+((ww+1)>>1):cx+radius;
     ylo<=shape_saved?cy-(hh>>1):cy-radius;yhi<=shape_saved?cy+((hh+1)>>1):cy+radius;
     state<=2;
    end
    2:begin px<=$signed({1'b0,probe[0]?xhi:xlo});py<=$signed({1'b0,probe[1]?yhi:ylo});state<=10;end
    3:if(timer==0&&!dac_busy)begin
     timer<=CLK_HZ/10000-1;
     if(shape_saved&&mode==1)begin
      if(kind==3)begin
       px<=$signed({1'b0,cx})+((phase[6]^phase[5])?-$signed({1'b0,circ_x[20:9]}):$signed({1'b0,circ_x[20:9]}));
       py<=$signed({1'b0,cy})+(phase[6]?-$signed({1'b0,circ_y[20:9]}):$signed({1'b0,circ_y[20:9]}));
      end else begin px<=$signed({1'b0,p0[11:0]})+(ix>>>5);py<=$signed({1'b0,p0[23:12]})+(iy>>>5);end
     end else case(phase[6:5])
      0:begin px<=$signed({1'b0,cx})-radius+advance[12:4];py<=$signed({1'b0,cy})-radius;end
      1:begin px<=$signed({1'b0,cx})+radius;py<=$signed({1'b0,cy})-radius+advance[12:4];end
      2:begin px<=$signed({1'b0,cx})+radius-advance[12:4];py<=$signed({1'b0,cy})+radius;end
      3:begin px<=$signed({1'b0,cx})-radius;py<=$signed({1'b0,cy})+radius-advance[12:4];end
     endcase
     state<=10;
    end
    10:begin ax<=product;state<=11;end
    11:begin by<=product;state<=12;end
    12:begin dx<=product;state<=13;end
    13:begin ey<=product;state<=14;end
    14:begin
     mx<=($signed(ax)+$signed(by)+$signed(c))>>>8;my<=($signed(dx)+$signed(ey)+$signed(f))>>>8;
     if(mode==2)begin
      if(probing)begin mx<=$signed({1'b0,cfg[207:192]})+(probe[0]?128:-128);my<=$signed({1'b0,cfg[223:208]})+(probe[1]?128:-128);end
      else case(phase[6:5])
       0:begin mx<=$signed({1'b0,cfg[207:192]})-128+{phase[4:0],3'b0};my<=$signed({1'b0,cfg[223:208]})-128;end
       1:begin mx<=$signed({1'b0,cfg[207:192]})+128;my<=$signed({1'b0,cfg[223:208]})-128+{phase[4:0],3'b0};end
       2:begin mx<=$signed({1'b0,cfg[207:192]})+128-{phase[4:0],3'b0};my<=$signed({1'b0,cfg[223:208]})+128;end
       3:begin mx<=$signed({1'b0,cfg[207:192]})-128;my<=$signed({1'b0,cfg[223:208]})+128-{phase[4:0],3'b0};end
      endcase
     end
     state<=15;
    end
    15:begin
     if(!dac_code_ok||(!mapped_ok&&!(shape_saved&&mode==1&&full_saved)))begin range_fault<=1;path_ok<=0;lit<=0;state<=0;end
     else if(probing)begin
      if(!mapped_ok)probe_clipped<=1;
      if(probe==3)begin range_fault<=probe_clipped||!mapped_ok;probing<=0;state<=3;end else begin probe<=probe+1'b1;state<=2;end
     end else begin
      target_x<=mx[15:0];
      target_y<=my[15:0];
      if(!mapped_ok)range_fault<=1;
      state<=16;
     end
    end
    16:state<=17;
    17:if(!dac_busy&&applied_x==target_x&&applied_y==target_y)begin lit<=1;state<=18;end
    18:if(blank==0)begin
     if(shape_saved&&mode==1&&kind!=3)begin
      if(step_index==32)begin step_index<=0;if(next_edge==0)state<=0;else begin edge_index<=next_edge;state<=3;end end
      else begin step_index<=step_index+1'b1;state<=3;end
     end else begin phase<=phase+1'b1;state<=phase==128?0:3;end
    end
    default:state<=0;
   endcase
  end
 end
endmodule
