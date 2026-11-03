// Deterministic RGB565 source for isolating FIFO/DDR/HDMI from the OV5640 DVP.
// It uses the same camera-clock-side RGB565 packer and async FIFO as live data.
module rgb565_internal_test_pattern #(
    parameter integer WIDTH = 1920,
    parameter integer HEIGHT = 1080
) (
    input wire clk,
    input wire rst_n,
    input wire enable,
    output reg [15:0] pixel,
    output reg valid, sof, eol, eof,
    output reg frame_done, frame_good, frame_bad,
    output reg [11:0] x,
    output reg [10:0] y,
    output reg [11:0] measured_lines,
    output reg [12:0] last_line_bytes,
    output reg [3:0] error_flags,
    output reg [11:0] line_ok, line_bad,
    output reg [12:0] line_bytes_min, line_bytes_max,
    output reg [10:0] first_bad_line_y,
    output reg [12:0] first_bad_line_bytes,
    output reg [10:0] bad0_y, bad1_y, bad2_y,
    output reg [12:0] bad0_href, bad1_href, bad2_href,
    output reg [12:0] bad0_accept, bad1_accept, bad2_accept,
    output reg [12:0] bad0_bytes, bad1_bytes, bad2_bytes,
    output reg [12:0] bad0_raw_href, bad1_raw_href, bad2_raw_href,
    output reg [12:0] bad0_zero, bad1_zero, bad2_zero,
    output reg [12:0] bad0_ff, bad1_ff, bad2_ff,
    output reg frame_start_pulse
);
    localparam integer BAR_WIDTH = WIDTH / 8;
    reg active, pending_done;

    function [15:0] bar_color;
        input [11:0] xpos;
        begin
            if (xpos < BAR_WIDTH)       bar_color = 16'hF800;
            else if (xpos < 2*BAR_WIDTH) bar_color = 16'h07E0;
            else if (xpos < 3*BAR_WIDTH) bar_color = 16'h001F;
            else if (xpos < 4*BAR_WIDTH) bar_color = 16'hFFFF;
            else if (xpos < 5*BAR_WIDTH) bar_color = 16'h07FF;
            else if (xpos < 6*BAR_WIDTH) bar_color = 16'hF81F;
            else if (xpos < 7*BAR_WIDTH) bar_color = 16'hFFE0;
            else                         bar_color = 16'h0000;
        end
    endfunction

    always @(posedge clk) begin
        if (!rst_n) begin
            active <= 1'b0; pending_done <= 1'b0;
            pixel <= 16'd0;
            valid <= 1'b0; sof <= 1'b0; eol <= 1'b0; eof <= 1'b0;
            frame_done <= 1'b0; frame_good <= 1'b0; frame_bad <= 1'b0;
            x <= 12'd0; y <= 11'd0;
            measured_lines <= 12'd0; last_line_bytes <= 13'd0;
            error_flags <= 4'd0;
            line_ok <= 12'd0; line_bad <= 12'd0;
            line_bytes_min <= 13'd0; line_bytes_max <= 13'd0;
            first_bad_line_y <= 11'd0; first_bad_line_bytes <= 13'd0;
            bad0_y<=0; bad1_y<=0; bad2_y<=0;
            bad0_href<=0; bad1_href<=0; bad2_href<=0;
            bad0_accept<=0; bad1_accept<=0; bad2_accept<=0;
            bad0_bytes<=0; bad1_bytes<=0; bad2_bytes<=0;
            bad0_raw_href<=0; bad1_raw_href<=0; bad2_raw_href<=0;
            bad0_zero<=0; bad1_zero<=0; bad2_zero<=0;
            bad0_ff<=0; bad1_ff<=0; bad2_ff<=0;
            frame_start_pulse <= 1'b0;
        end else begin
            valid <= 1'b0; sof <= 1'b0; eol <= 1'b0; eof <= 1'b0;
            frame_done <= 1'b0; frame_good <= 1'b0; frame_bad <= 1'b0;
            frame_start_pulse <= 1'b0;
            if (!enable) begin
                active <= 1'b0;
                pending_done <= 1'b0;
                x <= 12'd0; y <= 11'd0;
            end else if (pending_done) begin
                frame_done <= 1'b1;
                frame_good <= 1'b1;
                pending_done <= 1'b0;
                measured_lines <= HEIGHT;
                last_line_bytes <= WIDTH;
                line_ok <= HEIGHT;
                line_bad <= 12'd0;
                line_bytes_min <= WIDTH;
                line_bytes_max <= WIDTH;
                error_flags <= 4'd0;
            end else if (!active) begin
                // One idle cycle separates frames so frame_done never shares
                // a cycle with the last valid pixel.
                active <= 1'b1;
                x <= 12'd0; y <= 11'd0;
                frame_start_pulse <= 1'b1;
            end else begin
                pixel <= bar_color(x);
                valid <= 1'b1;
                sof <= (x == 0 && y == 0);
                eol <= (x == WIDTH-1);
                eof <= (x == WIDTH-1 && y == HEIGHT-1);
                if (x == WIDTH-1) begin
                    x <= 12'd0;
                    if (y == HEIGHT-1) begin
                        y <= 11'd0;
                        active <= 1'b0;
                        pending_done <= 1'b1;
                    end else y <= y + 11'd1;
                end else x <= x + 12'd1;
                if (x == WIDTH-1 && y == HEIGHT-1) begin
                    measured_lines <= HEIGHT;
                    last_line_bytes <= WIDTH;
                    line_ok <= HEIGHT;
                    line_bad <= 12'd0;
                    line_bytes_min <= WIDTH;
                    line_bytes_max <= WIDTH;
                    error_flags <= 4'd0;
                end
            end
        end
    end
endmodule
