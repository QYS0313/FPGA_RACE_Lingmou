`timescale 1ns / 1ps

// Contest control channel, 115200-8-N-1.
// Host request : A5 CMD DATA CHECKSUM, CHECKSUM = A5 ^ CMD ^ DATA
// FPGA response: 5A (CMD|80) STATUS CHECKSUM
// STATUS[0] is OSD enable, STATUS[6:4] is the selected video mode.
module contest_uart_control #(
    parameter integer CLK_FREQ_HZ = 100000000,
    parameter integer UART_BPS    = 115200
) (
    output wire demo_valid, output wire [7:0] demo_cmd,demo_data,
    input wire [7:0] demo_reply,
    input  wire       clk,
    input  wire       rst_n,
    input  wire       uart_rxd,
    output wire       uart_txd,
    input  wire       button_n,
    input  wire       target_valid,
    input  wire [11:0] target_center_x,
    input  wire [11:0] target_center_y,
    input  wire [11:0] target_width,
    input  wire [11:0] target_height,
    // Current sensor exposure / gain-table index, for diagnostics (0x40..0x43).
    input  wire [15:0] ae_value,
    input  wire [15:0] ag_value,
    output reg        osd_enable,
    output reg [2:0]  video_mode,
    output reg [1:0]  target_color,
    output reg [1:0]  picture_mode,
    // Upper limit of the AE gain index (command 0x44/0x45), 79..143.
    output reg [15:0] ag_max,
    // 1 = HDMI RGB mapped to limited range 16..235 (command 0x34/0x35).
    output reg        range_limited,
    output reg [7:0]  shade_k,          // A37: 0x36 write / 0x37 read, radial shading slope (0 = off)
    output reg [7:0]  shade_s,          // A37: 0x38 write / 0x39 read, centre attenuation
    output reg [1:0]  fps_min,          // A39: 0x3A write / 0x3B read, lowest frame rate AE may use (0 = 60 only)
    input  wire [1:0] fps_mode,         // A39: 0x3C read, current AE frame-rate step
    // HSV target thresholds (commands 0x50..0x53 set, 0x54..0x57 read).
    output reg [7:0]  hue_center8,      // custom hue for target_color 3, 0..255 = 0..360 deg
    output reg [7:0]  hue_halfwidth8,   // hue tolerance, same scale
    output reg [7:0]  sat_min,          // minimum chroma
    output reg [7:0]  val_min,          // minimum brightness
    // PiP magnifier (0x60 enable set / 0x62 read, 0x61 zoom set / 0x63 read; zoom 0 auto 1 2x 2 4x)
    output reg        pip_enable,
    output reg [1:0]  pip_zoom,
    // Region effects (A20): 0x64 fx_mode (0 off 1 outline 2 mosaic 3 cartoon 4 edges 5 tint),
    // 0x65 region source (0 largest box, 1 union of all boxes), 0x68 mosaic block (0 8px 1 16px 2 32px)
    // reads: 0x66 fx_mode, 0x67 region source, 0x69 mosaic block
    output reg [2:0]  fx_mode,
    output reg        region_union,
    output reg [1:0]  mosaic_sel,
    output reg        region_lock,     // A32: 0x6A write / 0x6B read, 1 = freeze the effect region
    // Channel order applied to DDR readout pixels (command 0x32/0x33):
    // 0 {R,G,B} 1 {R,B,G} 2 {G,R,B} 3 {G,B,R} 4 {B,R,G} 5 {B,G,R}
    // where the letters name the byte positions of the stored pixel.
    output reg [255:0] galvo_config,
    output reg [7:0] gimbal_control,
    output reg gimbal_control_toggle,
    output reg gimbal_restart,
    input wire [7:0] gimbal_status,
    input wire [7:0] gimbal_test_status,
    input wire [191:0] diag_data_async,
    input wire diag_toggle_async,
    output reg diag_ack,
    output reg [1:0] galvo_mode,
    output reg galvo_commit,
    output reg [2:0]  color_perm
);

    // Firmware identity returned by command 0x26.  0xA1 marks the post-V52
    // contest line (teammate builds answered with their version number).
 localparam [7:0] FW_VERSION = 8'hC4;

    wire       rx_valid;
    wire [7:0] rx_data;
    reg        tx_start;
    reg  [7:0] tx_data;
    wire       tx_busy;

    uart_rx_8n1 #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .UART_BPS(UART_BPS)) u_rx (
        .clk(clk), .rst_n(rst_n), .rxd(uart_rxd),
        .data_valid(rx_valid), .data_out(rx_data)
    );

    uart_tx_8n1 #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .UART_BPS(UART_BPS)) u_tx (
        .clk(clk), .rst_n(rst_n), .start(tx_start), .data_in(tx_data),
        .busy(tx_busy), .txd(uart_txd)
    );

    localparam [1:0] RX_SYNC = 2'd0, RX_CMD = 2'd1,
                     RX_DATA = 2'd2, RX_CHECK = 2'd3;
    reg [1:0] rx_state;
    reg [7:0] cmd_reg;
    reg [7:0] data_reg;
    assign demo_valid=rx_valid && rx_state==RX_CHECK && rx_data==(8'hA5 ^ cmd_reg ^ data_reg);
    assign demo_cmd=cmd_reg; assign demo_data=data_reg;

    reg       response_pending;
    reg [4:0] response_index;
    reg [7:0] response_cmd;
    reg [7:0] response_status;
    reg [7:0] response_xor;
    reg [191:0] response_diag;
    reg [2:0] diag_sync;
    reg [191:0] diag_meta,diag_sync_data;
    reg [191:0] diag_pan,diag_tilt;
    reg [1:0] diag_available;
    wire [4:0] response_last=response_cmd==8'hF8 ? 5'd27 : 5'd3;
    reg [7:0] response_byte;
    always @* begin
        case(response_index)
            0:response_byte=8'h5A;
            1:response_byte=response_cmd;
            2:response_byte=response_status;
            default:response_byte=response_index==response_last ? response_xor : response_diag[7:0];
        endcase
    end
    // Bundled-data handshake; source holds all 192 bits until this ACK.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            diag_sync<=0;diag_meta<=0;diag_sync_data<=0;diag_ack<=0;
            diag_pan<=0;diag_tilt<=0;diag_available<=0;
        end else begin
            diag_sync<={diag_sync[1:0],diag_toggle_async};
            diag_meta<=diag_data_async;diag_sync_data<=diag_meta;
            if(diag_sync[2]!=diag_ack) begin
                diag_ack<=diag_sync[2];
                if(diag_sync_data[48]) begin diag_tilt<=diag_sync_data;diag_available[1]<=1;end
                else begin diag_pan<=diag_sync_data;diag_available[0]<=1;end
            end
        end
    end
    reg [4:0] config_index;
    reg        snapshot_valid;
    reg [11:0] snapshot_x, snapshot_y;
    reg [11:0] snapshot_width, snapshot_height;

    // About 20 ms of stable low level is treated as one button press.
    localparam integer BUTTON_COUNT_MAX = CLK_FREQ_HZ / 50;
    reg [21:0] button_count;
    reg        button_latched;

    wire [7:0] live_status = {1'b0, video_mode, 3'b000, osd_enable};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            osd_enable       <= 1'b1;
            galvo_config <= 0; galvo_mode <= 0; galvo_commit <= 0; config_index <= 0;
            gimbal_control<=0;gimbal_control_toggle<=0;gimbal_restart<=0;
            video_mode <= 3'd4;
            target_color <= 2'd1;
            // Defaults chosen by the user on 2026-09-13 after live A/B on the
            // capture card: natural curve with highlight knee, full-range output.
            picture_mode     <= 2'd2;
            ag_max           <= 16'd127;   // table entry 111 = 0x1F2F, about 11.75x
            range_limited    <= 1'b0;
            shade_k          <= 8'd4;    // A38: calibrated on a white wall (R-G spread 21 -> 4)
            shade_s          <= 8'd20;
            fps_min          <= 2'd0;
            hue_center8      <= 8'd0;
            hue_halfwidth8   <= 8'd18;    // about +-25 degrees (A13; 13 missed a 212-degree blue)
            sat_min <= 8'd40;
            val_min <= 8'd160;
            pip_enable       <= 1'b0;
            pip_zoom         <= 2'd0;
            fx_mode          <= 3'd0;
            region_union     <= 1'b1;
            mosaic_sel       <= 2'd1;
            region_lock      <= 1'b0;
            // Normal state is identity.  Hardware 2026-09-13: one boot out of
            // eleven came up with the stored bytes rotated (displayed R,G,B
            // from true B,R,G); selecting 3 = {G,B,R} corrects that state
            // until the DDR pixel format is made alignment-proof.
            color_perm       <= 3'd0;
            rx_state         <= RX_SYNC;
            cmd_reg          <= 8'd0;
            data_reg         <= 8'd0;
            response_pending <= 1'b0;
            response_index   <= 3'd0;
            response_cmd     <= 8'd0;
            response_status  <= 8'd0;
            response_xor<=0;response_diag<=0;
            snapshot_valid   <= 1'b0;
            snapshot_x       <= 12'd0;
            snapshot_y       <= 12'd0;
            snapshot_width   <= 12'd0;
            snapshot_height  <= 12'd0;
            tx_start         <= 1'b0;
            tx_data          <= 8'd0;
            button_count     <= 22'd0;
            button_latched   <= 1'b0;
        end else begin
            tx_start <= 1'b0;

            if (button_n) begin
                button_count   <= 22'd0;
                button_latched <= 1'b0;
            end else if (!button_latched) begin
                if (button_count == BUTTON_COUNT_MAX - 1) begin
                    button_count   <= button_count;
                    button_latched <= 1'b1;
                    osd_enable     <= ~osd_enable;
                end else begin
                    button_count <= button_count + 22'd1;
                end
            end

            if (rx_valid) begin
                case (rx_state)
                    RX_SYNC: begin
                        if (rx_data == 8'hA5)
                            rx_state <= RX_CMD;
                    end
                    RX_CMD: begin
                        cmd_reg  <= rx_data;
                        rx_state <= RX_DATA;
                    end
                    RX_DATA: begin
                        data_reg <= rx_data;
                        rx_state <= RX_CHECK;
                    end
                    default: begin
                        rx_state <= RX_SYNC;
                        if (rx_data == (8'hA5 ^ cmd_reg ^ data_reg)) begin
                            if (cmd_reg == 8'h74) begin
                                gimbal_control <= data_reg;
                                gimbal_control_toggle <= ~gimbal_control_toggle;
                                if(data_reg[2]) begin galvo_mode<=0;galvo_commit<=~galvo_commit;end
                            end
                            else if(cmd_reg == 8'h77) begin
                                gimbal_control<=0;gimbal_control_toggle<=~gimbal_control_toggle;
                                gimbal_restart<=~gimbal_restart;
                            end
                            else if (cmd_reg == 8'h70) config_index <= data_reg[4:0];
                            else if (cmd_reg == 8'h71) galvo_config[config_index*8 +: 8] <= data_reg;
                            else if (cmd_reg == 8'h72) begin
                                galvo_mode <= gimbal_control[2] || gimbal_status[7] ? 2'd0 : data_reg[1:0]; galvo_commit <= ~galvo_commit;
                            end
                            else if (cmd_reg == 8'h10)
                                osd_enable <= data_reg[0];
                            else if (cmd_reg == 8'h11)
                                video_mode <= (data_reg[2:0] <= 3'd4) ? data_reg[2:0] : 3'd0;
                            else if (cmd_reg == 8'h21)
                                target_color <= data_reg[1:0];     // 3 = custom hue
                            else if (cmd_reg == 8'h50)
                                hue_center8 <= data_reg;
                            else if (cmd_reg == 8'h51)
                                hue_halfwidth8 <= data_reg;
                            else if (cmd_reg == 8'h52)
                                sat_min <= data_reg;
                            else if (cmd_reg == 8'h53)
                                val_min <= data_reg;
                            else if (cmd_reg == 8'h60)
                                pip_enable <= 1'b0; // Removed feature: legacy command is a no-op.
                            else if (cmd_reg == 8'h61)
                                pip_zoom <= 2'd0;
                            else if (cmd_reg == 8'h64)
                                fx_mode <= (data_reg[2:0] <= 3'd5) ? data_reg[2:0] : 3'd0;
                            else if (cmd_reg == 8'h65)
                                region_union <= data_reg[0];
                            else if (cmd_reg == 8'h68)
                                mosaic_sel <= (data_reg[1:0] <= 2'd2) ? data_reg[1:0] : 2'd1;
                            else if (cmd_reg == 8'h6A)
                                region_lock <= data_reg[0];
                            else if (cmd_reg == 8'h30)
                                picture_mode <= data_reg[1:0];
                            else if (cmd_reg == 8'h32)
                                color_perm <= (data_reg[2:0] <= 3'd5) ? data_reg[2:0] : 3'd0;
                            else if (cmd_reg == 8'h44)
                                ag_max <= (data_reg < 8'd79) ? 16'd79 :
                                          (data_reg > 8'd143) ? 16'd143 : {8'd0, data_reg};
                            else if (cmd_reg == 8'h34)
                                range_limited <= data_reg[0];
                            else if (cmd_reg == 8'h36)
                                shade_k <= data_reg;
                            else if (cmd_reg == 8'h38)
                                shade_s <= (data_reg > 8'd64) ? 8'd64 : data_reg;
                            else if (cmd_reg == 8'h3A)
                                fps_min <= (data_reg[1:0] > 2'd2) ? 2'd2 : data_reg[1:0];

                            if ((cmd_reg == 8'h20) && (data_reg == 8'd0)) begin
                                snapshot_valid  <= target_valid;
                                snapshot_x      <= target_center_x;
                                snapshot_y      <= target_center_y;
                                snapshot_width  <= target_width;
                                snapshot_height <= target_height;
                            end

                            // Snapshot status after applying the accepted command.
                            response_cmd <= cmd_reg | 8'h80;
                            response_xor<=0;
                            if(cmd_reg==8'h78)
                                response_diag<=data_reg==0 ? diag_pan : data_reg==1 ? diag_tilt : 192'd0;
                            if((cmd_reg>=8'h80 && cmd_reg<=8'h94)||(cmd_reg>=8'ha0 && cmd_reg<=8'haf)) response_status<=demo_reply;
                            else if (cmd_reg == 8'h10)
                                response_status <= {1'b0, video_mode, 3'b000, data_reg[0]};
                            else if (cmd_reg == 8'h11)
                                response_status <= {1'b0,
                                    ((data_reg[2:0] <= 3'd4) ? data_reg[2:0] : 3'd0),
                                    3'b000, osd_enable};
                            else if (cmd_reg == 8'h20) begin
                                case (data_reg)
                                    8'd0: response_status <= {7'd0,target_valid};
                                    8'd1: response_status <= snapshot_x[7:0];
                                    8'd2: response_status <= {4'd0,snapshot_x[11:8]};
                                    8'd3: response_status <= snapshot_y[7:0];
                                    8'd4: response_status <= {4'd0,snapshot_y[11:8]};
                                    8'd5: response_status <= snapshot_width[7:0];
                                    8'd6: response_status <= {4'd0,snapshot_width[11:8]};
                                    8'd7: response_status <= snapshot_height[7:0];
                                    8'd8: response_status <= {4'd0,snapshot_height[11:8]};
                                    8'd9: response_status <= {6'd0,target_color};
                                    default: response_status <= 8'd0;
                                endcase
                            end
                            else if (cmd_reg == 8'h30)
                                response_status <= {6'd0, data_reg[1:0]};
                            else if (cmd_reg == 8'h31)
                                response_status <= {6'd0, picture_mode};
                            else if (cmd_reg == 8'h32)
                                response_status <= {5'd0, ((data_reg[2:0] <= 3'd5) ? data_reg[2:0] : 3'd0)};
                            else if (cmd_reg == 8'h33)
                                response_status <= {5'd0, color_perm};
                            else if (cmd_reg == 8'h74) response_status<=data_reg;
                            else if (cmd_reg == 8'h75) response_status<=gimbal_status;
                            else if (cmd_reg == 8'h79) response_status<=gimbal_test_status;
                            else if (cmd_reg == 8'h78)
                                response_status<=data_reg<2 ? {6'd0,data_reg[0],diag_available[data_reg[0]]} : 8'h80;
                            else if (cmd_reg == 8'h73)
                                response_status <= galvo_config[config_index*8 +: 8];
                            else if (cmd_reg == 8'h26)
                                response_status <= FW_VERSION;
                            else if (cmd_reg == 8'h40)
                                response_status <= ae_value[7:0];
                            else if (cmd_reg == 8'h41)
                                response_status <= ae_value[15:8];
                            else if (cmd_reg == 8'h42)
                                response_status <= ag_value[7:0];
                            else if (cmd_reg == 8'h43)
                                response_status <= ag_value[15:8];
                            else if (cmd_reg == 8'h44)
                                response_status <= (data_reg < 8'd79) ? 8'd79 :
                                                   (data_reg > 8'd143) ? 8'd143 : data_reg;
                            else if (cmd_reg == 8'h45)
                                response_status <= ag_max[7:0];
                            else if (cmd_reg == 8'h34)
                                response_status <= {7'd0, data_reg[0]};
                            else if (cmd_reg == 8'h35)
                                response_status <= {7'd0, range_limited};
                            else if (cmd_reg == 8'h36)
                                response_status <= data_reg;
                            else if (cmd_reg == 8'h37)
                                response_status <= shade_k;
                            else if (cmd_reg == 8'h38)
                                response_status <= (data_reg > 8'd64) ? 8'd64 : data_reg;
                            else if (cmd_reg == 8'h39)
                                response_status <= shade_s;
                            else if (cmd_reg == 8'h3A)
                                response_status <= {6'd0, ((data_reg[1:0] > 2'd2) ? 2'd2 : data_reg[1:0])};
                            else if (cmd_reg == 8'h3B)
                                response_status <= {6'd0, fps_min};
                            else if (cmd_reg == 8'h3C)
                                response_status <= {6'd0, fps_mode};
                            else if (cmd_reg >= 8'h50 && cmd_reg <= 8'h53)
                                response_status <= data_reg;
                            else if (cmd_reg == 8'h54)
                                response_status <= hue_center8;
                            else if (cmd_reg == 8'h55)
                                response_status <= hue_halfwidth8;
                            else if (cmd_reg == 8'h56)
                                response_status <= sat_min;
                            else if (cmd_reg == 8'h57)
                                response_status <= val_min;
                            else if (cmd_reg == 8'h60)
                                response_status <= 8'd0;
                            else if (cmd_reg == 8'h61)
                                response_status <= 8'd0;
                            else if (cmd_reg == 8'h62)
                                response_status <= {7'd0, pip_enable};
                            else if (cmd_reg == 8'h63)
                                response_status <= {6'd0, pip_zoom};
                            else if (cmd_reg == 8'h64)
                                response_status <= {5'd0, ((data_reg[2:0] <= 3'd5) ? data_reg[2:0] : 3'd0)};
                            else if (cmd_reg == 8'h65)
                                response_status <= {7'd0, data_reg[0]};
                            else if (cmd_reg == 8'h66)
                                response_status <= {5'd0, fx_mode};
                            else if (cmd_reg == 8'h67)
                                response_status <= {7'd0, region_union};
                            else if (cmd_reg == 8'h68)
                                response_status <= {6'd0, ((data_reg[1:0] <= 2'd2) ? data_reg[1:0] : 2'd1)};
                            else if (cmd_reg == 8'h69)
                                response_status <= {6'd0, mosaic_sel};
                            else if (cmd_reg == 8'h6A)
                                response_status <= {7'd0, data_reg[0]};
                            else if (cmd_reg == 8'h6B)
                                response_status <= {7'd0, region_lock};
                            else
                                response_status <= live_status;
                            response_pending <= 1'b1;
                            response_index   <= 3'd0;
                        end
                    end
                endcase
            end

            if (response_pending && !tx_busy && !tx_start) begin
                tx_data<=response_byte;
                response_xor<=response_xor ^ response_byte;
                if(response_cmd==8'hF8 && response_index>=3)
                    response_diag<={8'd0,response_diag[191:8]};
                tx_start <= 1'b1;
                if (response_index == response_last) begin
                    response_pending <= 1'b0;
                    response_index   <= 3'd0;
                end else begin
                    response_index <= response_index + 3'd1;
                end
            end
        end
    end
endmodule

module uart_rx_8n1 #(
    parameter integer CLK_FREQ_HZ = 100000000,
    parameter integer UART_BPS    = 115200
) (
    input wire clk, input wire rst_n, input wire rxd,
    output reg data_valid, output reg [7:0] data_out
);
    localparam integer CLKS_PER_BIT = CLK_FREQ_HZ / UART_BPS;
    localparam [1:0] IDLE=2'd0, START=2'd1, DATA=2'd2, STOP=2'd3;
    reg rx_meta, rx_sync;
    reg [1:0] state;
    reg [15:0] count;
    reg [2:0] bit_index;
    reg [7:0] shift;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin rx_meta <= 1'b1; rx_sync <= 1'b1; end
        else begin rx_meta <= rxd; rx_sync <= rx_meta; end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE; count <= 16'd0; bit_index <= 3'd0;
            shift <= 8'd0; data_valid <= 1'b0; data_out <= 8'd0;
        end else begin
            data_valid <= 1'b0;
            case (state)
                IDLE: begin count <= 16'd0; bit_index <= 3'd0; if (!rx_sync) state <= START; end
                START: begin
                    if (count == (CLKS_PER_BIT/2)-1) begin
                        count <= 16'd0; state <= rx_sync ? IDLE : DATA;
                    end else count <= count + 16'd1;
                end
                DATA: begin
                    if (count == CLKS_PER_BIT-1) begin
                        count <= 16'd0; shift[bit_index] <= rx_sync;
                        if (bit_index == 3'd7) begin bit_index <= 3'd0; state <= STOP; end
                        else bit_index <= bit_index + 3'd1;
                    end else count <= count + 16'd1;
                end
                default: begin
                    if (count == CLKS_PER_BIT-1) begin
                        count <= 16'd0; state <= IDLE;
                        if (rx_sync) begin data_out <= shift; data_valid <= 1'b1; end
                    end else count <= count + 16'd1;
                end
            endcase
        end
    end
endmodule

module uart_tx_8n1 #(
    parameter integer CLK_FREQ_HZ = 100000000,
    parameter integer UART_BPS    = 115200
) (
    input wire clk, input wire rst_n, input wire start, input wire [7:0] data_in,
    output wire busy, output reg txd
);
    localparam integer CLKS_PER_BIT = CLK_FREQ_HZ / UART_BPS;
    reg active;
    reg [15:0] count;
    reg [3:0] bit_index;
    reg [9:0] frame;
    assign busy = active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active <= 1'b0; count <= 16'd0; bit_index <= 4'd0;
            frame <= 10'h3ff; txd <= 1'b1;
        end else if (!active) begin
            txd <= 1'b1; count <= 16'd0; bit_index <= 4'd0;
            if (start) begin
                frame <= {1'b1, data_in, 1'b0};
                txd <= 1'b0;
                active <= 1'b1;
            end
        end else if (count == CLKS_PER_BIT-1) begin
            count <= 16'd0;
            if (bit_index == 4'd9) begin
                active <= 1'b0; txd <= 1'b1;
            end else begin
                bit_index <= bit_index + 4'd1;
                txd <= frame[bit_index + 4'd1];
            end
        end else begin
            count <= count + 16'd1;
        end
    end
endmodule
