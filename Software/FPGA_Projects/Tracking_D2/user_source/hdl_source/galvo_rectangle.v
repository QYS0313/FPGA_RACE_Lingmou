// Autonomous 128-point rectangle, 10 kpoint/s. Affine camera->DAC mapping
// is calibrated by the host; all following tracking runs without the host.
// config bytes 0..23: six signed LE int32 Q8 (a,b,c,d,e,f).
// Slopes a,b,d,e use signed 24-bit values; offsets c,f use signed 32-bit.
// bytes 24..27 manual X/Y LE uint16, byte 28 pixel half-size (4..64).
// mode 0 dark, 1 green tracking, 2 small manual DAC calibration square.
module galvo_rectangle #(
    parameter integer CLK_HZ=100000000,
    parameter integer DAC_MIN=28768, DAC_MAX=36768
)(
    input wire clk, rst_n,
    input wire [255:0] config_data,
    input wire [1:0] config_mode,
    input wire config_toggle,
    input wire spot_valid,
    input wire [11:0] spot_x,spot_y,
    input wire dac_ready,dac_busy,
    input wire [15:0] applied_x,applied_y,
    output reg [15:0] target_x,target_y,
    output wire laser
);
    reg [255:0] cfg;
    reg [1:0] mode;
    reg seen;
    reg [6:0] phase;
    reg [11:0] cx,cy;
    reg [3:0] state;
    reg [31:0] timer;
    reg [31:0] blank;
    reg lit, inside;
    reg signed [12:0] px,py;
    reg signed [36:0] ax,by,dx,ey;
    reg signed [38:0] mx,my;
    wire signed [23:0] a=cfg[23:0], b=cfg[55:32], d=cfg[119:96], e=cfg[151:128];
    wire signed [31:0] c=cfg[95:64], f=cfg[191:160];
    wire [7:0] radius = cfg[231:224]<4 ? 8'd4 : cfg[231:224]>64 ? 8'd64 : cfg[231:224];
    wire [12:0] advance = radius * phase[4:0];
    wire tracking_ok = spot_valid && spot_x>=radius && spot_x+radius<1280 &&
        spot_y>=radius && spot_y+radius<720;
    assign laser = rst_n && dac_ready && lit && inside && blank==0 &&
        ((mode==1 && tracking_ok) || mode==2);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cfg<=0; mode<=0; seen<=0; phase<=0; cx<=0; cy<=0; state<=0;
            timer<=0; blank<=0; lit<=0; inside<=0;
            target_x<=32768; target_y<=32768; px<=0; py<=0;
            ax<=0; by<=0; dx<=0; ey<=0; mx<=0; my<=0;
        end else if (config_toggle != seen) begin
            seen<=config_toggle; cfg<=config_data; mode<=config_mode;
            phase<=0; state<=0; timer<=0; lit<=0; blank<=CLK_HZ/500;
        end else begin
            if (blank!=0) blank<=blank-1'b1;
            if (timer!=0) timer<=timer-1'b1;
            if (mode==0 || (mode==1 && !tracking_ok) || !dac_ready) begin
                lit<=0; phase<=0; state<=0; timer<=0; blank<=CLK_HZ/500;
            end else case(state)
                0: if (timer==0 && !dac_busy) begin
                    timer<=CLK_HZ/10000-1;
                    if (phase==0) begin
                        cx<=spot_x; cy<=spot_y;
                        // Blank travel to the beginning of each new rectangle.
                        blank<=CLK_HZ/500; lit<=0;
                        state<=1;
                    end else state<=2;
                end
                1: state<=2;
                2: begin
                    case(phase[6:5])
                        0: begin px<=$signed({1'b0,cx})-$signed({1'b0,radius})+$signed({1'b0,advance[12:4]}); py<=$signed({1'b0,cy})-$signed({1'b0,radius}); end
                        1: begin px<=$signed({1'b0,cx})+$signed({1'b0,radius}); py<=$signed({1'b0,cy})-$signed({1'b0,radius})+$signed({1'b0,advance[12:4]}); end
                        2: begin px<=$signed({1'b0,cx})+$signed({1'b0,radius})-$signed({1'b0,advance[12:4]}); py<=$signed({1'b0,cy})+$signed({1'b0,radius}); end
                        3: begin px<=$signed({1'b0,cx})-$signed({1'b0,radius}); py<=$signed({1'b0,cy})+$signed({1'b0,radius})-$signed({1'b0,advance[12:4]}); end
                    endcase
                    state<=3;
                end
                3: begin ax<=a*px; by<=b*py; dx<=d*px; ey<=e*py; state<=4; end
                4: begin
                    mx<=($signed(ax)+$signed(by)+$signed(c)) >>> 8;
                    my<=($signed(dx)+$signed(ey)+$signed(f)) >>> 8;
                    state<=5;
                    if (mode==2) begin
                        // +/-128 DAC units; no stationary bright calibration dot.
                        case(phase[6:5])
                            0: begin mx<=$signed({1'b0,cfg[207:192]})-128+{phase[4:0],3'b0}; my<=$signed({1'b0,cfg[223:208]})-128; end
                            1: begin mx<=$signed({1'b0,cfg[207:192]})+128; my<=$signed({1'b0,cfg[223:208]})-128+{phase[4:0],3'b0}; end
                            2: begin mx<=$signed({1'b0,cfg[207:192]})+128-{phase[4:0],3'b0}; my<=$signed({1'b0,cfg[223:208]})+128; end
                            3: begin mx<=$signed({1'b0,cfg[207:192]})-128; my<=$signed({1'b0,cfg[223:208]})+128-{phase[4:0],3'b0}; end
                        endcase
                    end
                end
                5: begin
                    inside <= mx>=DAC_MIN && mx<=DAC_MAX && my>=DAC_MIN && my<=DAC_MAX;
                    if (mx>=DAC_MIN && mx<=DAC_MAX && my>=DAC_MIN && my<=DAC_MAX) begin
                        target_x<=mx[15:0]; target_y<=my[15:0];
                    end
                    lit<=0; state<=6;
                end
                // Allow DAC busy to assert before testing completion.
                6: state<=7;
                7: if (!dac_busy && applied_x==target_x && applied_y==target_y) begin
                    lit<=1;
                    if (phase==0) state<=8;
                    else begin phase<=phase+1'b1; state<=0; end
                end
                8: if (blank==0) begin phase<=phase+1'b1; state<=0; end
                default: state<=0;
            endcase
        end
    end
endmodule
