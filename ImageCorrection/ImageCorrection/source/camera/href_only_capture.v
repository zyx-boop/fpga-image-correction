// OV5640 RAW8 front end. Camera VSYNC is deliberately absent from this port.
// A long HREF-low interval marks the boundary; every line and frame is
// validated before the existing RGB packer receives frame_done/frame_good.
module href_only_capture #(
    parameter integer ACTIVE_W=1920,
    parameter integer ACTIVE_H=1080,
    parameter integer SYS_CLK_HZ=27000000,
    parameter integer CAM_PCLK_HZ=84000000,
    parameter integer H_TOTAL=2500,
    // A real OV5640 line can end a few PCLKs early at the HREF edge.  Keep
    // the historical strict behaviour by default; the board top enables a
    // small bounded tolerance and repeats the final sample to restore the
    // fixed-width RAW stream expected by Bayer/DDR.
    parameter integer SHORT_LINE_TOL=0,
    // A one-cycle HREF edge race can occasionally expose one or two extra
    // samples.  They are ignored after x=ACTIVE_W-1; larger excursions are
    // still rejected as malformed lines.
    parameter integer LONG_LINE_TOL=0,
    parameter integer GAP_LINE_PERIODS=4,
    // Divide before multiplication to keep Verilog's 32-bit integer math in
    // range. For the configured 2500-cycle line this is about 119 us at 27 MHz.
    parameter integer GAP_SYS_CYCLES=
        ((SYS_CLK_HZ/1000)*H_TOTAL*GAP_LINE_PERIODS)/(CAM_PCLK_HZ/1000)
) (
    input wire sys_clk, cam_pclk, rst_n, configured,
    input wire cam_href,
    input wire [7:0] cam_data,
    output reg [7:0] raw,
    output reg valid, sof, eol, eof,
    output reg [11:0] x,
    output reg [10:0] y,
    output reg frame_done, frame_good, frame_bad,
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
    // Kept for interface parity with dvp_frame_capture.  HREF-only capture
    // has no separate sampled-vs-raw rejection recorder in this build; the
    // outputs remain zero rather than affecting the HREF data path.
    output reg [12:0] first_bad_reject_count,
    output reg [11:0] first_bad_rej_x0, first_bad_rej_x1,
    output reg [11:0] first_bad_rej_x2, first_bad_rej_x3,
    output reg [3:0] first_bad_rej_r0, first_bad_rej_r1,
    output reg [3:0] first_bad_rej_r2, first_bad_rej_r3,
    output reg frame_start_pulse,
    output reg [16:0] gap_pclk_before_frame_end,
    output reg line_start,line_end,
    output reg coord_error,
    output reg boundary_event,
    output reg [10:0] boundary_line_y,
    output reg boundary_raw_href,
    output reg boundary_sync_href,
    output reg boundary_prev_href,
    output reg [15:0] boundary_gap_count,
    output reg [2:0] boundary_source,
    output wire frame_start,frame_end,frame_valid
);
    assign frame_start=frame_start_pulse;
    assign frame_end=frame_done;
    assign frame_valid=frame_good;
    // HREF lasts about 23 us high and 7 us low in the configured mode; both
    // are much longer than a 27 MHz system cycle. The sys-clock counter also
    // advances if the camera stops PCLK during vertical blank.
    (* ASYNC_REG="TRUE" *) reg [1:0] sys_rst_pipe;
    (* ASYNC_REG="TRUE" *) reg [1:0] href_sys_pipe;
    reg [31:0] low_cycles;
    reg long_gap_sys;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            sys_rst_pipe<=0; href_sys_pipe<=0;
            low_cycles<=0; long_gap_sys<=0;
        end else begin
            sys_rst_pipe<={sys_rst_pipe[0],1'b1};
            href_sys_pipe<={href_sys_pipe[0],cam_href};
            if (!sys_rst_pipe[1] || !configured || href_sys_pipe[1]) begin
                low_cycles<=0;
                long_gap_sys<=0;
            end else if (!long_gap_sys) begin
                if (low_cycles>=GAP_SYS_CYCLES-1)
                    long_gap_sys<=1;
                else low_cycles<=low_cycles+32'd1;
            end
        end
    end

    // A long-lived system-domain gap asynchronously CLEARS a single camera
    // flip-flop. This uses the device's supported async-clear primitive, not
    // an unclocked latch, and preserves the event if PCLK pauses in blanking.
    // Its output is consumed only after a camera-clock edge. No video bus
    // crosses here; an early gap indication has many line times to settle.
    wire gap_capture_n=rst_n && !long_gap_sys;
    reg gap_pending_n;
    always @(posedge cam_pclk or negedge gap_capture_n) begin
        if (!gap_capture_n) gap_pending_n<=1'b0;
        else gap_pending_n<=1'b1;
    end
    wire gap_pending=!gap_pending_n;
    reg gap_seen;
    wire gap_event=gap_pending && !gap_seen;

    (* ASYNC_REG="TRUE" *) reg [1:0] cfg_pipe;
    reg href_q, href_prev;
    reg [7:0] data_q;
    reg armed, active, bad_line_seen;
    reg [12:0] pad_remaining, pad_x;
    reg [10:0] pad_y;
    reg pad_last_line;
    reg [7:0] last_raw;
    reg [12:0] line_bytes;
    reg [12:0] href_high_pclk_count, capture_accept_count;
    reg [12:0] raw_href_pclk_count, raw_zero_pixel_count, raw_ff_pixel_count;
    reg [2:0] bad_record_count;
    reg [11:0] lines;
    reg [16:0] post_last_line_clocks;
    reg [15:0] href_gap_count;
    wire href_rise=href_q && !href_prev;
    wire href_fall=!href_q && href_prev;
    wire starting=armed && href_rise;
    wire taking=href_q && (active || starting);
    wire frame_complete=(lines==ACTIVE_H) && !bad_line_seen &&
                        (pad_remaining==0) && !href_q && !href_prev;

    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n) begin
            cfg_pipe<=0; href_q<=0; href_prev<=0; data_q<=0;
            gap_seen<=0; armed<=0; active<=0; bad_line_seen<=0;
            line_bytes<=0; lines<=0; raw<=0;
            href_high_pclk_count<=0; capture_accept_count<=0;
            raw_href_pclk_count<=0; raw_zero_pixel_count<=0; raw_ff_pixel_count<=0;
            bad_record_count<=0;
            pad_remaining<=0; pad_x<=0; pad_y<=0; pad_last_line<=0;
            last_raw<=0;
            post_last_line_clocks<=0;
            valid<=0; sof<=0; eol<=0; eof<=0;
            x<=0; y<=0; frame_done<=0; frame_good<=0; frame_bad<=0;
            measured_lines<=0; last_line_bytes<=0; error_flags<=0;
            line_ok<=0; line_bad<=0;
            line_bytes_min<=13'h1fff; line_bytes_max<=0;
            first_bad_line_y<=0; first_bad_line_bytes<=0;
            bad0_y<=0; bad1_y<=0; bad2_y<=0;
            bad0_href<=0; bad1_href<=0; bad2_href<=0;
            bad0_accept<=0; bad1_accept<=0; bad2_accept<=0;
            bad0_bytes<=0; bad1_bytes<=0; bad2_bytes<=0;
            bad0_raw_href<=0; bad1_raw_href<=0; bad2_raw_href<=0;
            bad0_zero<=0; bad1_zero<=0; bad2_zero<=0;
            bad0_ff<=0; bad1_ff<=0; bad2_ff<=0;
            first_bad_reject_count<=0;
            first_bad_rej_x0<=0; first_bad_rej_x1<=0;
            first_bad_rej_x2<=0; first_bad_rej_x3<=0;
            first_bad_rej_r0<=0; first_bad_rej_r1<=0;
            first_bad_rej_r2<=0; first_bad_rej_r3<=0;
            frame_start_pulse<=0;
            gap_pclk_before_frame_end<=0;
            line_start<=0;line_end<=0;
            coord_error<=1'b0;
            boundary_event<=0; boundary_line_y<=0;
            boundary_raw_href<=0; boundary_sync_href<=0;
            boundary_prev_href<=0; boundary_gap_count<=0;
            boundary_source<=0; href_gap_count<=0;
        end else begin
            cfg_pipe<={cfg_pipe[0],configured};
            href_q<=cam_href;
            href_prev<=href_q;
            data_q<=cam_data;
            valid<=0; sof<=0; eol<=0; eof<=0;
            frame_done<=0; frame_good<=0; frame_bad<=0;
            frame_start_pulse<=0;
            line_start<=0;line_end<=0;
            boundary_event<=0;
            if (!href_q) begin
                if (href_gap_count!=16'hffff)
                    href_gap_count<=href_gap_count+16'd1;
            end else href_gap_count<=0;
            // Raw-domain observability only. Count direct cam_href/cam_data
            // on every cam_pclk edge; these counters never gate acceptance.
            if (cam_href) begin
                if (raw_href_pclk_count != 13'h1fff)
                    raw_href_pclk_count <= raw_href_pclk_count + 13'd1;
                if (cam_data == 8'h00 && raw_zero_pixel_count != 13'h1fff)
                    raw_zero_pixel_count <= raw_zero_pixel_count + 13'd1;
                if (cam_data == 8'hff && raw_ff_pixel_count != 13'h1fff)
                    raw_ff_pixel_count <= raw_ff_pixel_count + 13'd1;
            end
            if (!cfg_pipe[1]) begin
                gap_seen<=0; armed<=0; active<=0; bad_line_seen<=0;
                line_bytes<=0; lines<=0;
                pad_remaining<=0; pad_x<=0; pad_y<=0; pad_last_line<=0;
                post_last_line_clocks<=0;
            end else begin
                if (!gap_pending) gap_seen<=0;
                else if (gap_event) gap_seen<=1;

                if (gap_event) begin
                    boundary_event<=1'b1;
                    boundary_line_y<=lines[10:0];
                    boundary_raw_href<=cam_href;
                    boundary_sync_href<=href_q;
                    boundary_prev_href<=href_prev;
                    boundary_gap_count<=href_gap_count;
                    boundary_source<=3'd2; // HREF-only long-gap boundary
                    if (active) begin
                        frame_done<=1;
                        frame_good<=frame_complete;
                        frame_bad<=!frame_complete;
                        measured_lines<=lines;
                        gap_pclk_before_frame_end<=post_last_line_clocks;
                        error_flags<={1'b0,(lines!=ACTIVE_H),
                                      (line_bad!=0),bad_line_seen};
                    end
                    armed<=1;
                    active<=0;
                    bad_line_seen<=0;
                    line_bytes<=0;
                    lines<=0;
                    first_bad_line_y<=0;
                    first_bad_line_bytes<=0;
                    bad_record_count<=0;
                    bad0_y<=0; bad1_y<=0; bad2_y<=0;
                    bad0_href<=0; bad1_href<=0; bad2_href<=0;
                    bad0_accept<=0; bad1_accept<=0; bad2_accept<=0;
                    bad0_bytes<=0; bad1_bytes<=0; bad2_bytes<=0;
                    bad0_raw_href<=0; bad1_raw_href<=0; bad2_raw_href<=0;
                    bad0_zero<=0; bad1_zero<=0; bad2_zero<=0;
                    bad0_ff<=0; bad1_ff<=0; bad2_ff<=0;
                    href_high_pclk_count<=0;
                    capture_accept_count<=0;
                    raw_href_pclk_count<=0;
                    raw_zero_pixel_count<=0;
                    raw_ff_pixel_count<=0;
                    pad_remaining<=0;
                    pad_x<=0; pad_y<=0; pad_last_line<=0;
                    post_last_line_clocks<=0;
                    coord_error<=1'b0;
                end else begin
                    // Complete a tolerated short line before accepting the
                    // next HREF rise.  Repeating the last captured sample
                    // preserves one RAW token per x-coordinate, so Bayer
                    // parity and the RGB565 packer remain aligned.
                    if (pad_remaining!=0) begin
                        raw<=last_raw;
                        valid<=1;
                        x<=pad_x[11:0];
                        y<=pad_y;
                        sof<=0;
                        eol<=(pad_remaining==1);
                        eof<=(pad_remaining==1 && pad_last_line);
                        if (pad_remaining==1) begin
                            pad_remaining<=0;
                        end else begin
                            pad_remaining<=pad_remaining-13'd1;
                            pad_x<=pad_x+13'd1;
                        end
                    end else if (starting) begin
                        if (line_bytes != 0 || lines >= ACTIVE_H)
                            coord_error<=1'b1;
                        armed<=0;
                        active<=1;
                        frame_start_pulse<=1;
                        line_ok<=0;
                        line_bad<=0;
                        first_bad_line_y<=0;
                        first_bad_line_bytes<=0;
                        line_bytes_min<=13'h1fff;
                        line_bytes_max<=0;
                        last_line_bytes<=0;
                    end
                    if (href_rise && (active || starting))
                        line_start<=1;
                    if (taking) begin
                        if (href_high_pclk_count!=13'h1fff)
                            href_high_pclk_count<=href_high_pclk_count+13'd1;
                        if (line_bytes!=13'h1fff)
                            line_bytes<=line_bytes+13'd1;
                        else bad_line_seen<=1;
                        if (line_bytes<ACTIVE_W && lines<ACTIVE_H) begin
                            raw<=data_q;
                            last_raw<=data_q;
                            valid<=1;
                            if (capture_accept_count!=13'h1fff)
                                capture_accept_count<=capture_accept_count+13'd1;
                            x<=line_bytes[11:0];
                            y<=lines[10:0];
                            sof<=(line_bytes==0 && lines==0);
                            eol<=(line_bytes==ACTIVE_W-1);
                            eof<=(line_bytes==ACTIVE_W-1 && lines==ACTIVE_H-1);
                        end else if (line_bytes > ACTIVE_W + LONG_LINE_TOL)
                            bad_line_seen<=1;
                    end else if (href_fall && active) begin
                        line_end<=1;
                        last_line_bytes<=line_bytes;
                        if (line_bytes<line_bytes_min)
                            line_bytes_min<=line_bytes;
                        if (line_bytes>line_bytes_max)
                            line_bytes_max<=line_bytes;
                        if ((line_bytes<=ACTIVE_W + LONG_LINE_TOL) &&
                            (line_bytes >= ACTIVE_W - SHORT_LINE_TOL)) begin
                            if (line_ok!=12'hfff) line_ok<=line_ok+12'd1;
                            if (line_bytes<ACTIVE_W) begin
                                pad_remaining<=ACTIVE_W-line_bytes;
                                pad_x<=line_bytes[12:0];
                                pad_y<=lines[10:0];
                                pad_last_line<=(lines==ACTIVE_H-1);
                            end
                        end else begin
                            bad_line_seen<=1;
                            if (line_bad!=12'hfff) line_bad<=line_bad+12'd1;
                            if (bad_record_count==0) begin
                                bad0_y<=lines[10:0]; bad0_href<=href_high_pclk_count;
                                bad0_accept<=capture_accept_count; bad0_bytes<=line_bytes;
                                bad0_raw_href<=raw_href_pclk_count;
                                bad0_zero<=raw_zero_pixel_count; bad0_ff<=raw_ff_pixel_count;
                            end else if (bad_record_count==1) begin
                                bad1_y<=lines[10:0]; bad1_href<=href_high_pclk_count;
                                bad1_accept<=capture_accept_count; bad1_bytes<=line_bytes;
                                bad1_raw_href<=raw_href_pclk_count;
                                bad1_zero<=raw_zero_pixel_count; bad1_ff<=raw_ff_pixel_count;
                            end else if (bad_record_count==2) begin
                                bad2_y<=lines[10:0]; bad2_href<=href_high_pclk_count;
                                bad2_accept<=capture_accept_count; bad2_bytes<=line_bytes;
                                bad2_raw_href<=raw_href_pclk_count;
                                bad2_zero<=raw_zero_pixel_count; bad2_ff<=raw_ff_pixel_count;
                            end
                            if (bad_record_count!=3'd3)
                                bad_record_count<=bad_record_count+3'd1;
                            if (line_bad==0) begin
                                first_bad_line_y<=lines[10:0];
                                first_bad_line_bytes<=line_bytes;
                            end
                        end
                        if (lines!=12'hfff) lines<=lines+12'd1;
                        else bad_line_seen<=1;
                        if (lines==ACTIVE_H-1)
                            post_last_line_clocks<=0;
                        line_bytes<=0;
                        href_high_pclk_count<=0;
                        capture_accept_count<=0;
                        raw_href_pclk_count<=0;
                        raw_zero_pixel_count<=0;
                        raw_ff_pixel_count<=0;
                    end else if (active && lines==ACTIVE_H && !href_q &&
                                 post_last_line_clocks!=17'h1ffff) begin
                        post_last_line_clocks<=post_last_line_clocks+17'd1;
                    end
                end
            end
        end
    end
endmodule
