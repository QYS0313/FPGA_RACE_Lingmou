`timescale 1ns/1ps
// Presentation validity follows the overlay, not the always-running ROI detector.
module video_target_visibility(input wire [2:0] mode,fx_mode,
 input wire color_valid,motion_valid,output wire visible);
 assign visible=mode==3'd5 ? motion_valid : mode==3'd4 && fx_mode==0 && color_valid;
endmodule

// Frame events, not selected AE settings. One-second window uses the board's
// 100 MHz reference. Camera and HDMI counters are independent (DDR repeats
// frames when the camera runs slower). Counts wrap at 65536, not latency.
module video_stats #(parameter REF_HZ=100000000) (
    input wire ref_clk, pixel_clk, camera_clk, rst_n,
    input wire camera_sof, display_vs,
    input wire target_valid,
    input wire [11:0] target_x,target_y,target_w,target_h,
    input wire [7:0] binary_threshold,edge_threshold,
    output reg [7:0] camera_fps,display_fps,
    output reg [15:0] camera_frames,display_frames,
    output reg [135:0] uart_stats
);
    reg [26:0] timer;
    reg second_toggle, camera_toggle, camera_d;
    reg [2:0] second_sync,camera_sync;
    reg display_d;
    reg [7:0] camera_count,display_count;
    wire cam_event=camera_sync[2]^camera_sync[1];
    wire display_event=display_vs&&!display_d;
    wire second_event=second_sync[2]^second_sync[1];
    wire [7:0] cam_next=(camera_count==255) ? 8'd255 : camera_count+cam_event;
    wire [7:0] disp_next=(display_count==255) ? 8'd255 : display_count+display_event;
    // Bundled-data CDC: source holds the complete record until destination
    // acknowledges it. Destination waits two synchronizer cycles before read.
    reg [135:0] held;
    reg request,ack;
    reg [2:0] req_sync,ack_sync;
    always @(posedge camera_clk or negedge rst_n) begin
        if (!rst_n) begin camera_toggle<=0; camera_d<=0; end
        else begin
            camera_d<=camera_sof;
            if(camera_sof&&!camera_d) camera_toggle<=~camera_toggle;
        end
    end
    always @(posedge ref_clk or negedge rst_n) begin
        if (!rst_n) begin
            timer<=0; second_toggle<=0; req_sync<=0; ack<=0; uart_stats<=0;
        end else begin
            if(timer==REF_HZ-1) begin timer<=0; second_toggle<=~second_toggle; end
            else timer<=timer+1'b1;
            req_sync<={req_sync[1:0],request};
            if(req_sync[2]!=ack) begin uart_stats<=held; ack<=req_sync[2]; end
        end
    end
    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            second_sync<=0; camera_sync<=0; display_d<=0;
            camera_count<=0; display_count<=0; camera_fps<=0; display_fps<=0;
            camera_frames<=0; display_frames<=0; ack_sync<=0; request<=0; held<=0;
        end else begin
            second_sync<={second_sync[1:0],second_toggle};
            camera_sync<={camera_sync[1:0],camera_toggle}; display_d<=display_vs;
            if(cam_event) camera_frames<=camera_frames+1'b1;
            if(display_event) display_frames<=display_frames+1'b1;
            if(second_event) begin
                camera_fps<=cam_next; display_fps<=disp_next;
                camera_count<=0; display_count<=0;
            end else begin camera_count<=cam_next; display_count<=disp_next; end
            ack_sync<={ack_sync[1:0],ack};
            if(ack_sync[2]==request) begin
                held<={edge_threshold,binary_threshold,display_frames,camera_frames,
                       display_fps,camera_fps,4'b0,target_h,4'b0,target_w,
                       4'b0,target_y,4'b0,target_x,7'b0,target_valid};
                request<=~request;
            end
        end
    end
endmodule
