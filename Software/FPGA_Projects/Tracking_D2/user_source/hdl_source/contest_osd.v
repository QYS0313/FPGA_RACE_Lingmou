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
    input  wire       target_valid,
    input wire shape_enable,shape_drawing,shape_range_fault,shape_allow_full_range,
    input wire [1:0] shape_kind,shape_color,
    input  wire [1:0] fps_mode,       // A39: 0 = 60, 1 = 30, 2 = 15 fps
    output reg        out_vsync,
    output reg        out_hsync,
    output reg        out_de,
    output reg [23:0] out_rgb
);
    reg [11:0] x;
    reg [11:0] y;
    reg de_d;
    reg [23:0] overlay_rgb;
    reg overlay_hit;
    reg [7:0] text_char;
    reg [4:0] glyph_bits;
    reg [4:0] glyph_x;
    reg [2:0] glyph_y;
    reg [4:0] char_index;
    reg [31:0] cn_bits;

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
                "A": case(row) 0:glyph5x7=5'b01110;1:glyph5x7=5'b10001;2:glyph5x7=5'b10001;3:glyph5x7=5'b11111;4:glyph5x7=5'b10001;5:glyph5x7=5'b10001;6:glyph5x7=5'b10001;endcase
                "B": case(row) 0:glyph5x7=5'b11110;1:glyph5x7=5'b10001;2:glyph5x7=5'b10001;3:glyph5x7=5'b11110;4:glyph5x7=5'b10001;5:glyph5x7=5'b10001;6:glyph5x7=5'b11110;endcase
                "C": case(row) 0:glyph5x7=5'b01111;1:glyph5x7=5'b10000;2:glyph5x7=5'b10000;3:glyph5x7=5'b10000;4:glyph5x7=5'b10000;5:glyph5x7=5'b10000;6:glyph5x7=5'b01111;endcase
                "D": case(row) 0:glyph5x7=5'b11110;1:glyph5x7=5'b10001;2:glyph5x7=5'b10001;3:glyph5x7=5'b10001;4:glyph5x7=5'b10001;5:glyph5x7=5'b10001;6:glyph5x7=5'b11110;endcase
                "E": case(row) 0:glyph5x7=5'b11111;1:glyph5x7=5'b10000;2:glyph5x7=5'b10000;3:glyph5x7=5'b11110;4:glyph5x7=5'b10000;5:glyph5x7=5'b10000;6:glyph5x7=5'b11111;endcase
                "F": case(row) 0:glyph5x7=5'b11111;1:glyph5x7=5'b10000;2:glyph5x7=5'b10000;3:glyph5x7=5'b11110;4:glyph5x7=5'b10000;5:glyph5x7=5'b10000;6:glyph5x7=5'b10000;endcase
                "G": case(row) 0:glyph5x7=5'b01111;1:glyph5x7=5'b10000;2:glyph5x7=5'b10000;3:glyph5x7=5'b10111;4:glyph5x7=5'b10001;5:glyph5x7=5'b10001;6:glyph5x7=5'b01111;endcase
                "H": case(row) 0,1,2,4,5,6:glyph5x7=5'b10001;3:glyph5x7=5'b11111;endcase
                "U": case(row) 0,1,2,3,4,5:glyph5x7=5'b10001;6:glyph5x7=5'b01110;endcase
                "W": case(row) 0,1,2:glyph5x7=5'b10001;3,4,5:glyph5x7=5'b10101;6:glyph5x7=5'b01010;endcase
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

    function [7:0] shape_text;
      input [4:0] idx;input status_line;
      reg [255:0] text;
      begin
        text="                                ";
        if(!target_valid)text="SEARCHING                       ";
        else if(status_line)begin
          if(shape_drawing&&shape_range_fault)text=shape_allow_full_range?"FULL RANGE DRAWING              ":"OUT OF RANGE                    ";
          else if(shape_drawing)text="LOCKED DRAWING                  ";
          else text="READY                           ";
        end else begin
          case(shape_color)
            1:text[255:208]="RED   ";2:text[255:208]="GREEN ";
            3:text[255:208]="BLUE  ";default:text[255:208]="DARK  ";
          endcase
          case(shape_kind)
            1:text[207:80]="TRIANGLE        ";2:text[207:80]="RECTANGLE       ";
            3:text[207:80]="CIRCLE          ";default:text[207:80]="UNKNOWN         ";
          endcase
        end
        shape_text=text[255-idx*8-:8];
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
        cn_bits = 32'd0;

        if (enable && in_de) begin
            if ((x >= 12'd20) && (x < 12'd360) && (y >= 12'd20) && (y < 12'd106)) begin
                overlay_hit = 1'b1;
                overlay_rgb = 24'h101820;
            end
            if ((x >= 12'd20) && (x < 12'd360) && (y >= 12'd20) && (y < 12'd24)) begin
                overlay_hit = 1'b1;
                overlay_rgb = 24'hF06028;
            end
            // 32x16 Chinese bitmap, enlarged 2x to 64x32 pixels.
            if ((x >= 12'd34) && (x < 12'd98) && (y >= 12'd33) && (y < 12'd65)) begin
                cn_bits = cn_row((y - 12'd33) >> 1);
                if (cn_bits[31 - ((x - 12'd34) >> 1)]) begin
                    overlay_hit = 1'b1;
                    overlay_rgb = 24'hFFFFFF;
                end
            end
            // ANLOGIC, 5x7 font at 2x scale in 16-pixel cells.
            if ((x >= 12'd116) && (x < 12'd228) && (y >= 12'd34) && (y < 12'd48)) begin
                char_index = (x - 12'd116) >> 4;
                text_char = title_char(char_index[3:0]);
                glyph_x = ((x - 12'd116) & 12'h00f) >> 1;
                glyph_y = (y - 12'd34) >> 1;
                glyph_bits = glyph5x7(text_char, glyph_y);
                if ((glyph_x < 5) && glyph_bits[4-glyph_x]) begin
                    overlay_hit = 1'b1;
                    overlay_rgb = 24'hFFFFFF;
                end
            end
            // A39: frame rate, title line right of ANLOGIC (grey, orange when reduced).
            if ((x >= 12'd244) && (x < 12'd324) && (y >= 12'd34) && (y < 12'd48)) begin
                char_index = (x - 12'd244) >> 4;
                text_char = fps_char(fps_mode, char_index[3:0]);
                glyph_x = ((x - 12'd244) & 12'h00f) >> 1;
                glyph_y = (y - 12'd34) >> 1;
                glyph_bits = glyph5x7(text_char, glyph_y);
                if ((glyph_x < 5) && glyph_bits[4-glyph_x]) begin
                    overlay_hit = 1'b1;
                    overlay_rgb = (fps_mode == 2'd0) ? 24'h8898A8 : 24'hF0A040;
                end
            end
            // MODE n LIVE/GRAY/BINARY/SOBEL/TRACK status line.
            if ((x >= 12'd116) && (x < 12'd324) && (y >= 12'd68) && (y < 12'd82)) begin
                char_index = (x - 12'd116) >> 4;
                text_char = status_char(char_index, mode);
                glyph_x = ((x - 12'd116) & 12'h00f) >> 1;
                glyph_y = (y - 12'd68) >> 1;
                glyph_bits = glyph5x7(text_char, glyph_y);
                if ((glyph_x < 5) && glyph_bits[4-glyph_x]) begin
                    overlay_hit = 1'b1;
                    overlay_rgb = 24'hB8D8E8;
                end
            end
            if(shape_enable && x>=20 && x<400 && y>=108 && y<162)begin
                overlay_hit=1;overlay_rgb=24'h101820;
                if(x>=34&&x<386&&((y>=114&&y<128)||(y>=140&&y<154)))begin
                    char_index=(x-34)>>4;
                    text_char=shape_text(char_index,y>=140);
                    glyph_x=((x-34)&15)>>1;
                    glyph_y=(y>=140?y-140:y-114)>>1;
                    glyph_bits=glyph5x7(text_char,glyph_y);
                    if(glyph_x<5&&glyph_bits[4-glyph_x])begin
                        overlay_rgb=!target_valid?24'hb0b0b0:shape_color==1?24'hff9090:
                            shape_color==2?24'h70e090:shape_color==3?24'h80b0ff:24'hffffff;
                    end
                end
            end
            // Live indicator.
            if ((x >= 12'd332) && (x < 12'd344) && (y >= 12'd69) && (y < 12'd81)) begin
                overlay_hit = 1'b1;
                overlay_rgb = ((mode == 3'd4) && !target_valid) ? 24'hE04030 : 24'h20D070;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x <= 12'd0; y <= 12'd0; de_d <= 1'b0;
            out_vsync <= 1'b0; out_hsync <= 1'b0; out_de <= 1'b0;
            out_rgb <= 24'd0;
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
            out_rgb   <= overlay_hit ? overlay_rgb : in_rgb;
        end
    end
endmodule
