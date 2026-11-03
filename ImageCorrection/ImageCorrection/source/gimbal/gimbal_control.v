`timescale 1ns / 1ps
// R17 host preset motion. IMU telemetry is read-only, never a servo feedback input.
module r17_gimbal_control #(
    parameter integer CLOCK_HZ = 27000000,
    parameter integer BAUD = 115200,
    parameter integer WATCHDOG_MS = 2000
)(
    input wire clk, input wire rst_n, input wire rs232_rx,
    input wire [15:0] imu_roll,imu_pitch,imu_yaw,imu_age,imu_bad,imu_idle_ms,
    input wire [31:0] imu_samples,imu_bytes,imu_edges,imu_min_ticks,
    input wire [7:0] imu_baud,
    input wire imu_has_sample,imu_fresh,imu_rx_level,
    input wire tx_grant, output wire tx_pending,tx_owned,
    output wire rs232_tx, output wire servo_pan, output wire servo_tilt
);
    // rst_n already has synchronized release in the 27 MHz system domain.
    // Reuse R16's single JY61 receiver; telemetry cannot set PWM targets.
    wire reset_ok=rst_n;
    wire [7:0] rx_byte;
    wire rx_valid, tx_busy, tx_done;
    reg tx_start;
    reg [7:0] tx_byte;
    r15_imu_uart_rx #(.CLOCK_HZ(CLOCK_HZ), .BAUD(BAUD)) u_rx
        (.clk(clk), .rst_n(reset_ok), .rx(rs232_rx), .data(rx_byte), .valid(rx_valid));
    r17_host_uart_tx #(.CLOCK_HZ(CLOCK_HZ), .BAUD(BAUD)) u_tx
        (.clk(clk), .rst_n(reset_ok), .start(tx_start), .data(tx_byte),
         .tx(rs232_tx), .busy(tx_busy), .done(tx_done));

    function [7:0] crc_byte;
        input [7:0] previous, value;
        reg [7:0] c;
        integer k;
        begin
            c = previous ^ value;
            for (k=0; k<8; k=k+1)
                c = c[7] ? (c << 1) ^ 8'h07 : c << 1;
            crc_byte = c;
        end
    endfunction
    reg [7:0] request [2:8];
    reg [3:0] rx_index;
    reg [7:0] rx_crc;
    reg [31:0] rx_gap;
    reg enabled, watchdog_latched;
    reg [15:0] pan_target, tilt_target;
    wire [15:0] pan_current, tilt_current;
    reg [7:0] step_us,step_frames;
    reg [31:0] watchdog_count;
    localparam integer WATCHDOG_TICKS = (CLOCK_HZ/1000)*WATCHDOG_MS;
    wire watchdog_expire = enabled && !watchdog_latched &&
        watchdog_count == WATCHDOG_TICKS-1;
    reg response_pending, tx_active, tx_wait;
    assign tx_pending=response_pending||tx_active;
    assign tx_owned=tx_active;
    reg [4:0] tx_index, tx_last;
    reg [1:0] response_kind;
    reg [7:0] response_seq, response_result;
    reg [7:0] response [0:23];
    wire [15:0] requested_pan = {request[5],request[4]};
    wire [15:0] requested_tilt = {request[7],request[6]};
    wire frame_received = rx_valid && rx_index == 9;
    wire response_slot=!response_pending && !tx_active;
    // STOP/DISABLE take effect even while an earlier reply waits behind J160.
    // A busy reply is never overwritten; host queries/retries for confirmation.
    wire actionable_frame=frame_received && (response_slot || request[3]==3 || request[3]==4);
    wire crc_ok = rx_byte == rx_crc;
    // Conservative commissioning envelope, NOT measured mechanical calibration.
    localparam [15:0] PAN_MIN_US=16'd1000, PAN_MAX_US=16'd2000, PAN_CENTER_US=16'd1500;
    localparam [15:0] TILT_MIN_US=16'd1000, TILT_MAX_US=16'd2000, TILT_CENTER_US=16'd1500;
    wire set_range_ok = requested_pan >= PAN_MIN_US && requested_pan <= PAN_MAX_US &&
        requested_tilt >= TILT_MIN_US && requested_tilt <= TILT_MAX_US &&
        request[8] != 0 && request[8] <= 20; // 1..20 us per 20 ms frame for initial direction/limit testing.
    wire legal_command = request[3] >= 1 && request[3] <= 8;
    wire rate_range_ok=request[8]>=1 && request[8]<=100;
    wire valid_command = actionable_frame && crc_ok && legal_command &&
        (request[3] != 8 || rate_range_ok) &&
        (request[3] != 1 || (set_range_ok && enabled));
    wire freeze = (valid_command && (request[3] == 3 || request[3] == 4)) ||
        (watchdog_expire && !valid_command);
    wire moving = enabled && (pan_current != pan_target || tilt_current != tilt_target);
    r17_gimbal_pwm #(.CLOCK_HZ(CLOCK_HZ),.PAN_CENTER_US(PAN_CENTER_US),
                 .TILT_CENTER_US(TILT_CENTER_US)) u_pwm
        (.clk(clk), .rst_n(reset_ok), .enable(enabled), .freeze(freeze),
         .pan_target(pan_target), .tilt_target(tilt_target), .step_us(step_us),.step_frames(step_frames),
         .pan_current(pan_current), .tilt_current(tilt_current),
         .pan_pwm(servo_pan), .tilt_pwm(servo_tilt));

    reg [7:0] response_crc;
    integer init_index;
    always @(posedge clk) begin
        if (!reset_ok) begin
            rx_index <= 0; rx_crc <= 0; rx_gap <= 0;
            enabled <= 0; watchdog_latched <= 0; watchdog_count <= 0;
            pan_target <= PAN_CENTER_US; tilt_target <= TILT_CENTER_US; step_us <= 1; step_frames <= 2;
            response_pending <= 0; tx_active <= 0; tx_wait <= 0;
            tx_index <= 0; tx_last<=13; response_kind<=0; tx_start <= 0; tx_byte <= 0;
            response_seq <= 0; response_result <= 0;
            response_crc<=0;
            for (init_index=2; init_index<9; init_index=init_index+1) request[init_index] <= 0;
            for (init_index=0; init_index<24; init_index=init_index+1) response[init_index] <= 0;
        end else begin
            tx_start <= 0;
            if (watchdog_count < WATCHDOG_TICKS-1)
                watchdog_count <= watchdog_count + 1'b1;
            if (watchdog_expire && !valid_command) begin
                pan_target <= pan_current; tilt_target <= tilt_current;
                watchdog_latched <= 1;
            end
            if (valid_command) begin
                watchdog_count <= 0;
                case (request[3])
                    1: begin
                        pan_target <= requested_pan; tilt_target <= requested_tilt;
                        step_us <= request[8]; watchdog_latched <= 0;
                    end
                    2: begin
                        enabled <= 1;
                        if(!enabled) begin pan_target <= pan_current; tilt_target <= tilt_current; end
                        watchdog_latched <= 0;
                    end
                    3: begin
                        enabled <= 0; pan_target <= pan_current; tilt_target <= tilt_current;
                        watchdog_latched <= 0;
                    end
                    4: begin
                        pan_target <= pan_current; tilt_target <= tilt_current;
                        watchdog_latched <= 0;
                    end
                    5: begin end // Query is also the communication heartbeat.
                    6: begin end // IMU snapshot query; never changes servo targets.
                    7: begin end // Read-only electrical input diagnostics.
                    8: step_frames <= request[8]; // 1..100 PWM frames per step.
                    default: begin end
                endcase
            end
            if (rx_valid) begin
                rx_gap <= 0;
                case (rx_index)
                    0: if (rx_byte == 8'ha5) begin
                        rx_index <= 1; rx_crc <= crc_byte(0,rx_byte);
                    end
                    1: if (rx_byte == 8'h5a) begin
                        rx_index <= 2; rx_crc <= crc_byte(rx_crc,rx_byte);
                    end else if (rx_byte != 8'ha5) rx_index <= 0;
                    9: begin
                        rx_index <= 0;
                        if (frame_received && response_slot) begin
                            response_seq <= request[2]; response_pending <= 1;
                            response_kind <= request[3]==6 ? 1 : request[3]==7 ? 2 : 0;
                            if (!crc_ok) response_result <= 1;
                            else if (!legal_command) response_result <= 3;
                            else if ((request[3] == 1 && !set_range_ok) || (request[3] == 8 && !rate_range_ok)) response_result <= 2;
                            else if (request[3] == 1 && !enabled) response_result <= 4;
                            else response_result <= 0;
                        end
                    end
                    default: begin
                        request[rx_index] <= rx_byte;
                        rx_crc <= crc_byte(rx_crc,rx_byte);
                        rx_index <= rx_index + 1'b1;
                    end
                endcase
            end else if (rx_index != 0) begin
                if (rx_gap == CLOCK_HZ/100-1) begin rx_index <= 0; rx_gap <= 0; end
                else rx_gap <= rx_gap + 1'b1;
            end
            // Snapshot AFTER command register updates, then serialize a whole response.
            if (response_pending && tx_grant) begin
                response_pending <= 0; tx_active <= 1; tx_wait <= 0; tx_index <= 0;
                response_crc<=0;
                response[0] <= 8'h5a; response[1] <= 8'ha5;
                response[2] <= response_seq; response[3] <= response_result;
                response[4] <= {2'b10,3'b000,watchdog_latched,moving,enabled};
                response[5] <= pan_target[7:0]; response[6] <= pan_target[15:8];
                response[7] <= tilt_target[7:0]; response[8] <= tilt_target[15:8];
                response[9] <= pan_current[7:0]; response[10] <= pan_current[15:8];
                response[11] <= tilt_current[7:0]; response[12] <= tilt_current[15:8];
                tx_last<=13;
                if(response_kind==1) begin
                    // Atomic angle snapshot; no unsolicited serial data or ACK interleaving.
                    tx_last<=24;
                    response[1]<=8'ha6;
                    response[4]<={2'b10,4'b0000,imu_fresh,imu_has_sample};
                    response[5]<=imu_roll[7:0]; response[6]<=imu_roll[15:8];
                    response[7]<=imu_pitch[7:0]; response[8]<=imu_pitch[15:8];
                    response[9]<=imu_yaw[7:0]; response[10]<=imu_yaw[15:8];
                    response[11]<=imu_age[7:0]; response[12]<=imu_age[15:8];
                    response[13]<=imu_samples[7:0]; response[14]<=imu_samples[15:8];
                    response[15]<=imu_samples[23:16]; response[16]<=imu_samples[31:24];
                    response[17]<=imu_bytes[7:0]; response[18]<=imu_bytes[15:8];
                    response[19]<=imu_bytes[23:16]; response[20]<=imu_bytes[31:24];
                    response[21]<=imu_bad[7:0]; response[22]<=imu_bad[15:8];
                    response[23]<=imu_baud;
                end
                if(response_kind==2) begin
                    tx_last<=15;
                    response[1]<=8'ha7;
                    response[4]<={2'b10,4'b0000,(imu_edges!=0),imu_rx_level};
                    response[5]<=imu_edges[7:0]; response[6]<=imu_edges[15:8];
                    response[7]<=imu_edges[23:16]; response[8]<=imu_edges[31:24];
                    response[9]<=imu_min_ticks[7:0]; response[10]<=imu_min_ticks[15:8];
                    response[11]<=imu_min_ticks[23:16]; response[12]<=imu_min_ticks[31:24];
                    response[13]<=imu_idle_ms[7:0]; response[14]<=imu_idle_ms[15:8];
                end
            end
            if (tx_active) begin
                if (!tx_wait && !tx_busy) begin
                    tx_byte <= tx_index == tx_last ? response_crc : response[tx_index];
                    if(tx_index<tx_last) response_crc<=crc_byte(response_crc,response[tx_index]);
                    tx_start <= 1; tx_wait <= 1;
                end else if (tx_wait && tx_done) begin
                    tx_wait <= 0;
                    if (tx_index == tx_last) tx_active <= 0;
                    else tx_index <= tx_index + 1'b1;
                end
            end
        end
    end
endmodule
