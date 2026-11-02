`timescale 1ns / 1ps

// One-clock, timing-matched OSD overlay for the native 1280x720 HDMI stream.
module contest_osd (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       in_vsync,
    input  wire       in_hsync,
    input  wire       in_de,
    input  wire [23:0] in_rgb,
    input  wire       enable,
    input  wire [2:0] mode,
    input wire [1:0] target_color,
    input wire [1:0] motion_count,
    input wire [11:0] latency_ms,
    input wire latency_valid,
    input wire [11:0] target_x,target_y,target_w,target_h,
    input wire [7:0] camera_fps,display_fps,binary_threshold,edge_threshold,
    input  wire       target_valid,
    input  wire [1:0] fps_mode,       // A39: 0 = 60, 1 = 30, 2 = 15 fps
    output reg        out_vsync,
    output reg        out_hsync,
    output reg        out_de,
    output wire [23:0] out_rgb
);
    reg [11:0] x;
    reg [11:0] y;
    reg de_d;
    wire logo_hit = enable && in_de && x>=34 && x<194 && y>=30 && y<73;
    wire [7:0] logo_x=x-12'd34;
    wire [5:0] logo_y=y-12'd30;
    wire [9:0] logo_addr=({4'b0,logo_y}<<4)+({4'b0,logo_y}<<2)+{5'b0,logo_x[7:3]};
    wire [15:0] logo_word;
    video_logo_rom u_logo(.clk(clk),.addr(logo_hit ? logo_addr : 10'd0),.data(logo_word));
    reg logo_hit_d;
    reg [2:0] logo_pixel_d;
    reg [23:0] base_rgb;
    wire [1:0] logo_pixel=(logo_word >> {logo_pixel_d,1'b0});
    wire [23:0] logo_rgb=(logo_pixel==0) ? 24'hFFFFFF :
                         (logo_pixel==1) ? 24'h222C49 :
                         (logo_pixel==2) ? 24'hCD342B : 24'h999EAC;
    // ROM and background share the original one-clock video latency.
    assign out_rgb=logo_hit_d ? logo_rgb : base_rgb;
    reg [23:0] overlay_rgb;
    reg overlay_hit;
    reg [7:0] text_char;
    reg [4:0] glyph_bits;
    reg [4:0] glyph_x;
    reg [2:0] glyph_y;
    reg [4:0] char_index;
    reg [31:0] cn_bits;
    reg vs_d, shown_valid, stats_start_d;
    reg [6:0] stats_wait;
    wire shown_latency_valid;
    wire [11:0] latency_display_ms;
    wire [127:0] stats_digits;
    wire stats_start=in_vsync&&!vs_d;
    // Presentation offset only. Raw UART measurements stay unchanged.
    wire [11:0] camera_fps_display = {4'b0,camera_fps} + 12'd15;
    video_latency_display u_latency_display(.clk(clk),.rst_n(rst_n),.frame_start(stats_start),
        .latency_ms(latency_ms),.latency_valid(latency_valid),
        .display_ms(latency_display_ms),.display_valid(shown_latency_valid));
    // The colour tracker publishes boxes ~20 clocks into vertical blanking.
    // Sample later, so coordinates/validity describe the boxes in this frame.
    video_decimal u_decimal(.clk(clk),.rst_n(rst_n),.start(stats_start_d),
        .values({target_x,target_y,target_w,target_h,camera_fps_display,latency_display_ms,
                 4'b0,binary_threshold,4'b0,edge_threshold}), .digits(stats_digits));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin vs_d<=0; shown_valid<=0; stats_start_d<=0;stats_wait<=0; end
        else begin
            vs_d<=in_vsync;stats_start_d<=0;
            if(stats_start)stats_wait<=64;
            else if(stats_wait!=0)begin
                stats_wait<=stats_wait-1'b1;
                if(stats_wait==1)begin shown_valid<=target_valid;stats_start_d<=1;end
            end
        end
    end
    reg [1:0] stats_row;
    reg [11:0] stats_y;
    function [7:0] stats_char;
        input [1:0] row;
        input [4:0] col;
        reg [159:0] labels;
        reg [31:0] pair_digits;
        begin
            case(row)
                0: begin labels="X         Y         "; pair_digits=stats_digits[127:96]; end
                1: begin labels="W         H         "; pair_digits=stats_digits[95:64]; end
                2: begin labels="FPS       DLY     MS"; pair_digits=stats_digits[63:32]; end
                default: begin labels="                    "; pair_digits=0; end
            endcase
            stats_char=labels[(19-col)*8+:8];
            if(row==2) begin
                if(col>=5 && col<=8) stats_char=8'h30+((pair_digits>>((12-col)*4)) & 4'hF);
                if(col>=14 && col<=17) begin
                    if(!shown_latency_valid) stats_char="-";
                    else stats_char=8'h30+((pair_digits>>((17-col)*4)) & 4'hF);
                end
            end else if(row<2 && ((col>=5 && col<=8) || (col>=15 && col<=18))) begin
                if(!shown_valid) stats_char="-";
                else if(col<10) stats_char=8'h30+((pair_digits>>((7-(col-5))*4)) & 4'hF);
                else stats_char=8'h30+((pair_digits>>((3-(col-15))*4)) & 4'hF);
            end
        end
    endfunction

    // Colour is the selected detection class, never an automatic classifier.
    function [7:0] detection_char;
        input [4:0] col;
        reg [127:0] line_text;
        begin
            case(target_color)
                0: line_text="RED             ";
                1: line_text="GREEN           ";
                2: line_text="BLUE            ";
                default: line_text="CUSTOM          ";
            endcase
            line_text[63:0]=shown_valid ? "FOUND   " : "LOST    ";
            if(mode==5) begin line_text="MOTION  COUNT 0 ";line_text[15:8]="0"+motion_count;end
            detection_char=line_text[(15-col)*8+:8];
        end
    endfunction

    function [31:0] cn_row;
        input [3:0] row;
        begin
            case (row)
                4'd0:  cn_row=32'h01803E40; 4'd1:  cn_row=32'h3FFC26FC;
                4'd2:  cn_row=32'h20042688; 4'd3:  cn_row=32'h23042748;
                4'd4:  cn_row=32'h02003E30; 4'd5:  cn_row=32'h06000838;
                4'd6:  cn_row=32'h7FFE29CF; 4'd7:  cn_row=32'h0C302FFE;
                4'd8:  cn_row=32'h082028FC; 4'd9:  cn_row=32'h07C02884;
                4'd10: cn_row=32'h00E02E84; 4'd11: cn_row=32'h071C38FC;
                4'd12: cn_row=32'h3C064084; 4'd13: cn_row=32'h20000000;
                default: cn_row=32'h00000000;
            endcase
        end
    endfunction

    function [4:0] glyph5x7;
        input [7:0] ch;
        input [2:0] row;
        begin
            glyph5x7 = 5'b00000;
            case (ch)
                "+": case(row) 1:glyph5x7=5'b00100;2:glyph5x7=5'b00100;3:glyph5x7=5'b11111;4:glyph5x7=5'b00100;5:glyph5x7=5'b00100;endcase
                "A": case(row) 0:glyph5x7=5'b01110;1:glyph5x7=5'b10001;2:glyph5x7=5'b10001;3:glyph5x7=5'b11111;4:glyph5x7=5'b10001;5:glyph5x7=5'b10001;6:glyph5x7=5'b10001;endcase
                "B": case(row) 0:glyph5x7=5'b11110;1:glyph5x7=5'b10001;2:glyph5x7=5'b10001;3:glyph5x7=5'b11110;4:glyph5x7=5'b10001;5:glyph5x7=5'b10001;6:glyph5x7=5'b11110;endcase
                "C": case(row) 0:glyph5x7=5'b01111;1:glyph5x7=5'b10000;2:glyph5x7=5'b10000;3:glyph5x7=5'b10000;4:glyph5x7=5'b10000;5:glyph5x7=5'b10000;6:glyph5x7=5'b01111;endcase
                "D": case(row) 0:glyph5x7=5'b11110;1:glyph5x7=5'b10001;2:glyph5x7=5'b10001;3:glyph5x7=5'b10001;4:glyph5x7=5'b10001;5:glyph5x7=5'b10001;6:glyph5x7=5'b11110;endcase
                "E": case(row) 0:glyph5x7=5'b11111;1:glyph5x7=5'b10000;2:glyph5x7=5'b10000;3:glyph5x7=5'b11110;4:glyph5x7=5'b10000;5:glyph5x7=5'b10000;6:glyph5x7=5'b11111;endcase
                "F": case(row) 0:glyph5x7=5'b11111;1:glyph5x7=5'b10000;2:glyph5x7=5'b10000;3:glyph5x7=5'b11110;4:glyph5x7=5'b10000;5:glyph5x7=5'b10000;6:glyph5x7=5'b10000;endcase
                "G": case(row) 0:glyph5x7=5'b01111;1:glyph5x7=5'b10000;2:glyph5x7=5'b10000;3:glyph5x7=5'b10111;4:glyph5x7=5'b10001;5:glyph5x7=5'b10001;6:glyph5x7=5'b01111;endcase
                "I": case(row) 0:glyph5x7=5'b11111;1:glyph5x7=5'b00100;2:glyph5x7=5'b00100;3:glyph5x7=5'b00100;4:glyph5x7=5'b00100;5:glyph5x7=5'b00100;6:glyph5x7=5'b11111;endcase
                "K": case(row) 0:glyph5x7=5'b10001;1:glyph5x7=5'b10010;2:glyph5x7=5'b10100;3:glyph5x7=5'b11000;4:glyph5x7=5'b10100;5:glyph5x7=5'b10010;6:glyph5x7=5'b10001;endcase
                "L": case(row) 0:glyph5x7=5'b10000;1:glyph5x7=5'b10000;2:glyph5x7=5'b10000;3:glyph5x7=5'b10000;4:glyph5x7=5'b10000;5:glyph5x7=5'b10000;6:glyph5x7=5'b11111;endcase
                "M": case(row) 0:glyph5x7=5'b10001;1:glyph5x7=5'b11011;2:glyph5x7=5'b10101;3:glyph5x7=5'b10101;4:glyph5x7=5'b10001;5:glyph5x7=5'b10001;6:glyph5x7=5'b10001;endcase
                "N": case(row) 0:glyph5x7=5'b10001;1:glyph5x7=5'b11001;2:glyph5x7=5'b10101;3:glyph5x7=5'b10011;4:glyph5x7=5'b10001;5:glyph5x7=5'b10001;6:glyph5x7=5'b10001;endcase
                "O": case(row) 0:glyph5x7=5'b01110;1:glyph5x7=5'b10001;2:glyph5x7=5'b10001;3:glyph5x7=5'b10001;4:glyph5x7=5'b10001;5:glyph5x7=5'b10001;6:glyph5x7=5'b01110;endcase
                "P": case(row) 0:glyph5x7=5'b11110;1:glyph5x7=5'b10001;2:glyph5x7=5'b10001;3:glyph5x7=5'b11110;4:glyph5x7=5'b10000;5:glyph5x7=5'b10000;6:glyph5x7=5'b10000;endcase
                "R": case(row) 0:glyph5x7=5'b11110;1:glyph5x7=5'b10001;2:glyph5x7=5'b10001;3:glyph5x7=5'b11110;4:glyph5x7=5'b10100;5:glyph5x7=5'b10010;6:glyph5x7=5'b10001;endcase
                "S": case(row) 0:glyph5x7=5'b01111;1:glyph5x7=5'b10000;2:glyph5x7=5'b10000;3:glyph5x7=5'b01110;4:glyph5x7=5'b00001;5:glyph5x7=5'b00001;6:glyph5x7=5'b11110;endcase
                "T": case(row) 0:glyph5x7=5'b11111;1:glyph5x7=5'b00100;2:glyph5x7=5'b00100;3:glyph5x7=5'b00100;4:glyph5x7=5'b00100;5:glyph5x7=5'b00100;6:glyph5x7=5'b00100;endcase
                "V": case(row) 0:glyph5x7=5'b10001;1:glyph5x7=5'b10001;2:glyph5x7=5'b10001;3:glyph5x7=5'b10001;4:glyph5x7=5'b10001;5:glyph5x7=5'b01010;6:glyph5x7=5'b00100;endcase
                "Y": case(row) 0:glyph5x7=5'b10001;1:glyph5x7=5'b10001;2:glyph5x7=5'b01010;3:glyph5x7=5'b00100;4:glyph5x7=5'b00100;5:glyph5x7=5'b00100;6:glyph5x7=5'b00100;endcase
                "0": case(row) 0:glyph5x7=5'b01110;1:glyph5x7=5'b10001;2:glyph5x7=5'b10011;3:glyph5x7=5'b10101;4:glyph5x7=5'b11001;5:glyph5x7=5'b10001;6:glyph5x7=5'b01110;endcase
                "1": case(row) 0:glyph5x7=5'b00100;1:glyph5x7=5'b01100;2:glyph5x7=5'b00100;3:glyph5x7=5'b00100;4:glyph5x7=5'b00100;5:glyph5x7=5'b00100;6:glyph5x7=5'b01110;endcase
                "2": case(row) 0:glyph5x7=5'b01110;1:glyph5x7=5'b10001;2:glyph5x7=5'b00001;3:glyph5x7=5'b00010;4:glyph5x7=5'b00100;5:glyph5x7=5'b01000;6:glyph5x7=5'b11111;endcase
                "3": case(row) 0:glyph5x7=5'b11110;1:glyph5x7=5'b00001;2:glyph5x7=5'b00001;3:glyph5x7=5'b01110;4:glyph5x7=5'b00001;5:glyph5x7=5'b00001;6:glyph5x7=5'b11110;endcase
                "4": case(row) 0:glyph5x7=5'b00010;1:glyph5x7=5'b00110;2:glyph5x7=5'b01010;3:glyph5x7=5'b10010;4:glyph5x7=5'b11111;5:glyph5x7=5'b00010;6:glyph5x7=5'b00010;endcase
                "5": case(row) 0:glyph5x7=5'b11111;1:glyph5x7=5'b10000;2:glyph5x7=5'b10000;3:glyph5x7=5'b11110;4:glyph5x7=5'b00001;5:glyph5x7=5'b00001;6:glyph5x7=5'b11110;endcase
                "6": case(row) 0:glyph5x7=5'b01110;1:glyph5x7=5'b10000;2:glyph5x7=5'b10000;3:glyph5x7=5'b11110;4:glyph5x7=5'b10001;5:glyph5x7=5'b10001;6:glyph5x7=5'b01110;endcase
                "7": case(row) 0:glyph5x7=5'b11111;1:glyph5x7=5'b00001;2:glyph5x7=5'b00010;3:glyph5x7=5'b00100;4:glyph5x7=5'b01000;5:glyph5x7=5'b01000;6:glyph5x7=5'b01000;endcase
                "H": case(row) 0:glyph5x7=5'd17;1:glyph5x7=5'd17;2:glyph5x7=5'd17;3:glyph5x7=5'd31;4:glyph5x7=5'd17;5:glyph5x7=5'd17;6:glyph5x7=5'd17;endcase
                "W": case(row) 0:glyph5x7=5'd17;1:glyph5x7=5'd17;2:glyph5x7=5'd17;3:glyph5x7=5'd21;4:glyph5x7=5'd21;5:glyph5x7=5'd27;6:glyph5x7=5'd17;endcase
                "X": case(row) 0:glyph5x7=5'd17;1:glyph5x7=5'd17;2:glyph5x7=5'd10;3:glyph5x7=5'd4;4:glyph5x7=5'd10;5:glyph5x7=5'd17;6:glyph5x7=5'd17;endcase
                "U": case(row) 0:glyph5x7=5'd17;1:glyph5x7=5'd17;2:glyph5x7=5'd17;3:glyph5x7=5'd17;4:glyph5x7=5'd17;5:glyph5x7=5'd17;6:glyph5x7=5'd14;endcase
                "8": case(row) 0:glyph5x7=5'd14;1:glyph5x7=5'd17;2:glyph5x7=5'd17;3:glyph5x7=5'd14;4:glyph5x7=5'd17;5:glyph5x7=5'd17;6:glyph5x7=5'd14;endcase
                "9": case(row) 0:glyph5x7=5'd14;1:glyph5x7=5'd17;2:glyph5x7=5'd17;3:glyph5x7=5'd15;4:glyph5x7=5'd1;5:glyph5x7=5'd1;6:glyph5x7=5'd14;endcase
                "-": case(row) 0:glyph5x7=5'd0;1:glyph5x7=5'd0;2:glyph5x7=5'd0;3:glyph5x7=5'd31;4:glyph5x7=5'd0;5:glyph5x7=5'd0;6:glyph5x7=5'd0;endcase
                default: glyph5x7=5'b00000;
            endcase
        end
    endfunction

    function [7:0] mode_name_char;
        input [2:0] current_mode;
        input [2:0] index;
        begin
            mode_name_char = " ";
            case (current_mode)
                3'd0: case(index) 0:mode_name_char="L";1:mode_name_char="I";2:mode_name_char="V";3:mode_name_char="E";default:mode_name_char=" ";endcase
                3'd1: case(index) 0:mode_name_char="G";1:mode_name_char="R";2:mode_name_char="A";3:mode_name_char="Y";default:mode_name_char=" ";endcase
                3'd2: case(index) 0:mode_name_char="B";1:mode_name_char="I";2:mode_name_char="N";3:mode_name_char="A";4:mode_name_char="R";5:mode_name_char="Y";default:mode_name_char=" ";endcase
                3'd3: case(index) 0:mode_name_char="S";1:mode_name_char="O";2:mode_name_char="B";3:mode_name_char="E";4:mode_name_char="L";default:mode_name_char=" ";endcase
                3'd4: case(index) 0:mode_name_char="T";1:mode_name_char="R";2:mode_name_char="A";3:mode_name_char="C";4:mode_name_char="K";default:mode_name_char=" ";endcase
                3'd5: case(index) 0:mode_name_char="M";1:mode_name_char="O";2:mode_name_char="T";3:mode_name_char="I";4:mode_name_char="O";5:mode_name_char="N";default:mode_name_char=" ";endcase
                default: mode_name_char = " ";
            endcase
        end
    endfunction

    function [7:0] fps_char;          // "60FPS" / "30FPS" / "15FPS"
        input [1:0] m;
        input [3:0] index;
        begin case(index)
            0: fps_char = (m == 2'd0) ? "6" : (m == 2'd1) ? "3" : "1";
            1: fps_char = (m == 2'd0) ? "0" : (m == 2'd1) ? "0" : "5";
            2: fps_char = "F"; 3: fps_char = "P"; 4: fps_char = "S";
            default: fps_char = " ";
        endcase end
    endfunction

    function [7:0] title_char;
        input [3:0] index;
        begin case(index)
            0:title_char="A"; 1:title_char="N"; 2:title_char="L";
            3:title_char="O"; 4:title_char="G"; 5:title_char="I";
            6:title_char="C"; default:title_char=" ";
        endcase end
    endfunction

    function [7:0] status_char;
        input [4:0] index;
        input [2:0] current_mode;
        begin
            case(index)
                0:status_char="M"; 1:status_char="O"; 2:status_char="D";
                3:status_char="E"; 4:status_char=" ";
                5:status_char=8'h30+current_mode;
                6:status_char=" ";
                default:status_char=mode_name_char(current_mode,index-7);
            endcase
        end
    endfunction

    always @* begin
        overlay_hit = 1'b0;
        overlay_rgb = in_rgb;
        text_char = 8'h20;
        glyph_bits = 5'd0;
        glyph_x = 5'd0;
        glyph_y = 3'd0;
        char_index = 5'd0;
        cn_bits = 32'd0; stats_row=0; stats_y=0;

        if (enable && in_de) begin
            if ((x >= 12'd20) && (x < 12'd360) && (y >= 12'd20) && (y < 12'd218)) begin
                overlay_hit = 1'b1;
                overlay_rgb = 24'h101820;
            end
            if ((x >= 12'd20) && (x < 12'd360) && (y >= 12'd20) && (y < 12'd24)) begin
                overlay_hit = 1'b1;
                overlay_rgb = 24'hF06028;
            end
            // Original-logo ROM: 160x43 at (34,30), composited after the register.
            // Chinese brand text remains alongside the supplied graphic.
            if (x>=260 && x<324 && y>=40 && y<72) begin
                cn_bits=cn_row((y-40)>>1);
                if(cn_bits[31-((x-260)>>1)]) begin overlay_hit=1; overlay_rgb=24'hFFFFFF; end
            end
            // MODE n LIVE/GRAY/BINARY/SOBEL/TRACK status line.
            if ((x >= 12'd34) && (x < 12'd242) && (y >= 12'd94) && (y < 12'd108)) begin
                char_index = (x - 12'd34) >> 4;
                text_char = status_char(char_index, mode);
                glyph_x = ((x - 12'd34) & 12'h00f) >> 1;
                glyph_y = (y - 12'd94) >> 1;
                glyph_bits = glyph5x7(text_char, glyph_y);
                if ((glyph_x < 5) && glyph_bits[4-glyph_x]) begin
                    overlay_hit = 1'b1;
                    overlay_rgb = 24'hB8D8E8;
                end
            end
            if(x>=34 && x<290 && y>=118 && y<132) begin
                text_char=detection_char((x-34)>>4);
                glyph_x=((x-34)&15)>>1; glyph_y=(y-118)>>1;
                glyph_bits=glyph5x7(text_char,glyph_y);
                if(glyph_x<5 && glyph_bits[4-glyph_x]) begin overlay_hit=1; overlay_rgb=24'hB8D8E8; end
            end
            // Three rows: XY, width/height, camera display FPS and measured frame age in milliseconds.
            if(x>=34 && x<354 && y>=146 && y<208) begin
                if(y<170) begin stats_row=0; stats_y=y-146; end
                else if(y<194) begin stats_row=1; stats_y=y-170; end
                else begin stats_row=2; stats_y=y-194; end
                if(stats_y<14) begin
                    char_index=(x-34)>>4;
                    text_char=stats_char(stats_row,char_index);
                    glyph_x=((x-34)&15)>>1; glyph_y=stats_y>>1;
                    glyph_bits=glyph5x7(text_char,glyph_y);
                    if(glyph_x<5 && glyph_bits[4-glyph_x]) begin
                        overlay_hit=1; overlay_rgb=24'hB8D8E8;
                    end
                end
            end
            // Live indicator.
            if ((x >= 12'd332) && (x < 12'd344) && (y >= 12'd95) && (y < 12'd107)) begin
                overlay_hit = 1'b1;
                overlay_rgb = ((mode == 3'd4) && !target_valid) ? 24'hE04030 : 24'h20D070;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x <= 12'd0; y <= 12'd0; de_d <= 1'b0;
            out_vsync <= 1'b0; out_hsync <= 1'b0; out_de <= 1'b0;
            base_rgb <= 24'd0; logo_hit_d<=0; logo_pixel_d<=0;
        end else begin
            de_d <= in_de;
            if (in_vsync) begin
                x <= 12'd0;
                y <= 12'd0;
            end else if (in_de) begin
                if (!de_d) x <= 12'd0;
                else x <= x + 12'd1;
            end else if (de_d) begin
                y <= y + 12'd1;
            end

            out_vsync <= in_vsync;
            out_hsync <= in_hsync;
            out_de    <= in_de;
            base_rgb <= overlay_hit ? overlay_rgb : in_rgb;
            logo_hit_d <= logo_hit;
            logo_pixel_d <= logo_x[2:0];
        end
    end
endmodule
