`timescale 1ns / 1ps

// Colour-seeded bright-spot tracker. White cores are accepted only near a
// recently colour-confirmed target. target_valid never represents a held overlay.
module contest_target_tracker #(
    parameter integer IMAGE_WIDTH = 1280,
    parameter integer IMAGE_HEIGHT = 720,
    parameter integer MIN_RUN_PIXELS = 32,
    parameter integer BORDER_MARGIN = 8,
    parameter integer MIN_ROW_HITS = 3,
    parameter integer MAX_GAP_ROWS = 4,
    parameter integer HORIZONTAL_JOIN = 16,
    parameter integer COMPONENT_SLOTS = 6    // A22: 16->8, A34: 8->6 (area); 4 markers + target + 1 spare
) (
    input wire clk, input wire rst_n,
    input wire in_vsync, input wire in_hsync, input wire in_de,
    input wire [23:0] in_rgb,
    input wire in_edge,                  // Sobel edge flag from the video processor, in_rgb timing (A20)
    input wire draw_boxes,               // 0 = suppress box/cross overlay (region effects active)
    input wire region_union,             // 1 = region is the union of all boxes, 0 = largest box only
    input wire region_lock,              // A32: 1 = hold the current effect region (hands may leave)
    input wire [2:0] video_mode,
    input wire [1:0] target_color,       // 0 red, 1 green, 2 blue, 3 custom hue
    // HSV thresholds from the host (quasi-static, already in the pixel clock domain).
    // Hue values are on a 0..255 scale (255 = 360 degrees); presets for colours 0..2
    // use fixed centres, colour 3 uses hue_center8.  Half-width applies to all.
    input wire [7:0] hue_center8,
    input wire [7:0] hue_halfwidth8,
    input wire [7:0] sat_min,            // minimum chroma (max-min), 0..255
    input wire [7:0] val_min,            // minimum brightness (max channel), 0..255
    input wire projection_active,       // synchronized tracking-laser mode, not PWM gate
    output reg out_vsync, output reg out_hsync, output reg out_de,
    output reg [23:0] out_rgb,
    output reg [23:0] clean_rgb,       // out_rgb timing, without box/cross overlays (for region effects)
    output reg clean_edge,             // in_edge delayed to out_rgb timing (A20)
    // A20: effect region for contest_region_fx, updated at frame start.
    output reg region_valid,
    output reg [11:0] region_x0, output reg [11:0] region_y0,
    output reg [11:0] region_x1, output reg [11:0] region_y1,
    output reg frame_toggle,
    output reg target_valid,
    output reg [11:0] target_center_x, output reg [11:0] target_center_y,
    output reg [11:0] target_width, output reg [11:0] target_height
);
    // One compact bright spot on a neutral screen. Separated green objects
    // enlarge the bounding box and are deliberately rejected, not averaged.
    reg [11:0] px, py, xmin, xmax, ymin, ymax;
    reg [15:0] hits;
    reg [11:0] lx0,lx1,ly0,ly1;
    reg [15:0] local_hits, colour_hits;
    reg locked;
    reg [3:0] colour_age;
    reg [1:0] misses, frame_color;
    reg vs_d, de_d;
    reg projection_seen;
    reg [2:0] projection_tail;
    wire allow_white = target_color!=1 || (!projection_active && !projection_seen && projection_tail==0);
    wire [7:0] r=in_rgb[23:16], g=in_rgb[15:8], b=in_rgb[7:0];
    wire red_calibration = target_color == 0;
    wire [8:0] dominant = red_calibration ? {1'b0,r} : {1'b0,g};
    wire [8:0] other = red_calibration ? {1'b0,g} : {1'b0,r};
    wire white_calibration = target_color == 2;
    wire calibration_pixel;
    wire match_pixel = white_calibration ? calibration_pixel : dominant >= {1'b0,val_min} &&
        dominant >= other + {1'b0,sat_min} && dominant >= {1'b0,b} + {1'b0,sat_min};
    wire [7:0] local_chroma = sat_min>16 ? sat_min>>1 : 8'd8;
    wire [7:0] local_value = val_min>96 ? val_min-8'd32 : 8'd64;
    wire colour_pixel = white_calibration ? calibration_pixel : dominant >= {1'b0,local_value} &&
        dominant >= other+{1'b0,local_chroma} && dominant >= {1'b0,b}+{1'b0,local_chroma};
    wire white_pixel = r>=200 && g>=200 && b>=200 &&
        {1'b0,r}<={1'b0,g}+48 && {1'b0,g}<={1'b0,r}+48 &&
        {1'b0,r}<={1'b0,b}+48 && {1'b0,b}<={1'b0,r}+48 &&
        {1'b0,g}<={1'b0,b}+48 && {1'b0,b}<={1'b0,g}+48;
    // Explicit calibration-only acquisition; never enabled for green tracking.
    assign calibration_pixel = white_pixel && r>=val_min && g>=val_min && b>=val_min;
    wire near_target = locked && px+32>=target_center_x && px<=target_center_x+32 &&
        py+32>=target_center_y && py<=target_center_y+32;
    wire frame_start = in_vsync && !vs_d;
    wire good = hits >= 2 && hits <= (white_calibration ? 1024 : 4096) && xmax >= xmin && ymax >= ymin &&
        xmax-xmin <= (white_calibration ? 48 : 128) && ymax-ymin <= (white_calibration ? 48 : 128);
    wire local_good = locked && local_hits>=3 && local_hits<=1024 &&
        (allow_white || local_hits==colour_hits) &&
        lx1>=lx0 && ly1>=ly0 && lx1-lx0<=48 && ly1-ly0<=48 &&
        (colour_hits>=2 || (allow_white && colour_age<8));
    wire accepted = frame_color==target_color && (local_good || good);
    wire [11:0] bx0=local_good?lx0:xmin, bx1=local_good?lx1:xmax;
    wire [11:0] by0=local_good?ly0:ymin, by1=local_good?ly1:ymax;
    wire cross_pixel = locked && frame_color==target_color &&
        ((px == target_center_x && py+8 >= target_center_y && py <= target_center_y+8) ||
         (py == target_center_y && px+8 >= target_center_x && px <= target_center_x+8));
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            px<=0; py<=0; xmin<=4095; ymin<=4095; xmax<=0; ymax<=0;
            hits<=0; vs_d<=0; de_d<=0; frame_toggle<=0;
            lx0<=4095; ly0<=4095; lx1<=0; ly1<=0; local_hits<=0; colour_hits<=0;
            locked<=0; colour_age<=0; misses<=0; frame_color<=1;
            projection_seen<=0;projection_tail<=0;
            target_valid<=0; target_center_x<=0; target_center_y<=0;
            target_width<=0; target_height<=0;
            region_valid<=0; region_x0<=0; region_x1<=0; region_y0<=0; region_y1<=0;
            out_vsync<=0; out_hsync<=0; out_de<=0; out_rgb<=0; clean_rgb<=0; clean_edge<=0;
        end else begin
            vs_d<=in_vsync; de_d<=in_de;
            if(projection_active) projection_seen<=1;
            out_vsync<=in_vsync; out_hsync<=in_hsync; out_de<=in_de;
            // Amber is a display-only hold after a missed frame; laser uses valid.
            out_rgb <= cross_pixel ? (target_valid ? 24'hffffff : 24'hffb000) : in_rgb;
            clean_rgb<=in_rgb; clean_edge<=0;
            if (frame_start) begin
                frame_toggle<=~frame_toggle;
                projection_seen<=projection_active;
                if(projection_active || projection_seen) projection_tail<=4;
                else if(projection_tail!=0) projection_tail<=projection_tail-1'b1;
                frame_color<=target_color;
                target_valid<=accepted; region_valid<=accepted;
                if (accepted) begin
                    locked<=1; misses<=0;
                    if (!local_good || colour_hits>=2) colour_age<=0;
                    else if (colour_age<15) colour_age<=colour_age+1'b1;
                    target_center_x <= ({1'b0,bx0}+{1'b0,bx1}) >> 1;
                    target_center_y <= ({1'b0,by0}+{1'b0,by1}) >> 1;
                    target_width<=bx1-bx0+1'b1; target_height<=by1-by0+1'b1;
                    region_x0<=bx0; region_x1<=bx1; region_y0<=by0; region_y1<=by1;
                end else begin
                    if (colour_age<15) colour_age<=colour_age+1'b1;
                    if (misses==2 || frame_color!=target_color) locked<=0;
                    else misses<=misses+1'b1;
                end
                lx0<=4095; ly0<=4095; lx1<=0; ly1<=0; local_hits<=0; colour_hits<=0;
                hits<=0; xmin<=4095; ymin<=4095; xmax<=0; ymax<=0; px<=0; py<=0;
            end else if (in_de) begin
                px<=px+1'b1;
                if (match_pixel && px<IMAGE_WIDTH && py<IMAGE_HEIGHT) begin
                    if (hits != 65535) hits<=hits+1'b1;
                    if (px<xmin) xmin<=px;
                    if (px>xmax) xmax<=px;
                    if (py<ymin) ymin<=py;
                    if (py>ymax) ymax<=py;
                end
                if (near_target && (colour_pixel || (allow_white && white_pixel)) && px<IMAGE_WIDTH && py<IMAGE_HEIGHT) begin
                    if (local_hits!=65535) local_hits<=local_hits+1'b1;
                    if (colour_pixel && colour_hits!=65535) colour_hits<=colour_hits+1'b1;
                    if (px<lx0) lx0<=px;
                    if (px>lx1) lx1<=px;
                    if (py<ly0) ly0<=py;
                    if (py>ly1) ly1<=py;
                end
            end else begin
                px<=0;
                if (de_d) py<=py+1'b1;
            end
        end
    end
endmodule
