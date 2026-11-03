`timescale 1ns / 1ps
// Continuous 20 ms frames; width changes only at a frame boundary.
// freeze is combinationally asserted by the validated STOP/watchdog event.
module r17_gimbal_pwm #(
    parameter integer CLOCK_HZ = 27000000,
    parameter [15:0] PAN_CENTER_US = 16'd1500,
    parameter [15:0] TILT_CENTER_US = 16'd1500
)(
    input wire clk, input wire rst_n, input wire enable, input wire freeze,
    input wire [15:0] pan_target, input wire [15:0] tilt_target,
    input wire [7:0] step_us,step_frames,
    output reg [15:0] pan_current, output reg [15:0] tilt_current,
    output reg pan_pwm, output reg tilt_pwm
);
    localparam integer US_TICKS = CLOCK_HZ / 1000000;
    reg [((US_TICKS<=1)?1:$clog2(US_TICKS))-1:0] divider;
    reg [6:0] step_count;
    reg [14:0] frame_us;
    reg active, freeze_pending;
    wire us_tick = divider == US_TICKS-1;
    wire frame_end = us_tick && frame_us == 19999;
    // Only flip-flops drive the PWM pins. Decode transitions are sampled,
    // never exposed at the pins. Pulse timing remains identical to R17.
    wire [15:0] next_us = {1'b0,frame_us} + 16'd1;
    always @(posedge clk) begin
        if (!rst_n) begin
            pan_pwm <= 1'b0;
            tilt_pwm <= 1'b0;
        end else if (frame_end) begin
            pan_pwm <= enable;
            tilt_pwm <= enable;
        end else if (us_tick) begin
            if (next_us == pan_current) pan_pwm <= 1'b0;
            if (next_us == tilt_current) tilt_pwm <= 1'b0;
        end
    end
    function [15:0] approach;
        input [15:0] current_value, target_value;
        input [7:0] amount;
        begin
            if (current_value < target_value)
                approach = target_value-current_value > amount ?
                    current_value+amount : target_value;
            else if (current_value > target_value)
                approach = current_value-target_value > amount ?
                    current_value-amount : target_value;
            else approach = current_value;
        end
    endfunction
    always @(posedge clk) begin
        if (!rst_n) begin
            divider <= 0; frame_us <= 0; active <= 0;
            pan_current <= PAN_CENTER_US; tilt_current <= TILT_CENTER_US; freeze_pending <= 0; step_count<=0;
        end else begin
            if (us_tick) begin
                divider <= 0;
                if (frame_end) frame_us <= 0;
                else frame_us <= frame_us + 1'b1;
            end else divider <= divider + 1'b1;
            if (freeze) freeze_pending <= 1;
            if (frame_end) begin
                active <= enable;
                freeze_pending <= 0;
                if (!enable || !active || freeze || freeze_pending) begin
                    step_count<=0; // Retain the last pulse width across disable/re-enable.
                end else if(step_count+1>=step_frames) begin
                    step_count<=0;
                    pan_current <= approach(pan_current, pan_target, step_us);
                    tilt_current <= approach(tilt_current, tilt_target, step_us);
                end else step_count<=step_count+1'b1;
            end
        end
    end
endmodule
