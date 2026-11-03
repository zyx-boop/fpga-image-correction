// Source-synchronous RAW8 capture. A frame is committed only at the next
// VSYNC rising edge after exactly HEIGHT complete WIDTH-byte HREF lines.
// HREF alone qualifies pixels between boundaries. The VSYNC level is NOT
// a pixel gate; its rising edge must occur in the frame blanking interval.
// Optional warmup matches the same-module reference. Optional end delay
// lets the existing Bayer bottom-row replay drain before the DDR end token.
module dvp_frame_capture #(
    parameter integer WIDTH = 1920,
    parameter integer HEIGHT = 1080,
    parameter integer WARMUP_FRAMES = 0,
    parameter integer FRAME_DRAIN_CYCLES = 0,
    // Board diagnostic only: OV5640 DVP signals may have insufficient
    // setup/hold margin when sampled on the rising PCLK edge. Keep the
    // production/default path at 0; edge selection is independent from the
    // HREF polarity diagnostic below.
    parameter integer SAMPLE_NEGEDGE = 0,
    // OV5640 0x4740 bit1=0 is the documented HREF active-low mode.  Keep
    // the legacy high-active interpretation by default for existing tests,
    // and let the board A diagnostic select the documented polarity without
    // changing the sensor register or any downstream video stage.
    parameter integer HREF_ACTIVE_LOW = 0,
    // A measured board line can end a few PCLKs early.  When the deficit is
    // within this bound, repeat the last RAW sample after HREF falls so the
    // Bayer/DDR path still receives exactly WIDTH samples.  Zero preserves
    // the historical strict frame gate.
    parameter integer SHORT_LINE_TOL = 0
) (
    input wire pclk, rst_n, configured,
    input wire vsync, href,
    input wire [7:0] data,
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
    // Per-bad-line raw-domain diagnostics. raw_href counts every PCLK for
    // which the unregistered input HREF is active; zero/ff count data values
    // only and never gate capture validity.
    output reg [12:0] bad0_raw_href, bad1_raw_href, bad2_raw_href,
    output reg [12:0] bad0_zero, bad1_zero, bad2_zero,
    output reg [12:0] bad0_ff, bad1_ff, bad2_ff,
    // First bad line: capture-side rejected pixel diagnostics.
    output reg [12:0] first_bad_reject_count,
    output reg [11:0] first_bad_rej_x0, first_bad_rej_x1,
    output reg [11:0] first_bad_rej_x2, first_bad_rej_x3,
    output reg [3:0] first_bad_rej_r0, first_bad_rej_r1,
    output reg [3:0] first_bad_rej_r2, first_bad_rej_r3,
    output reg frame_start_pulse,
    output reg coord_error,
    // Boundary-only diagnostics. These are latched on a VSYNC boundary
    // observation and never depend on pixel value or DDR activity.
    output reg boundary_event,
    output reg [10:0] boundary_line_y,
    output reg boundary_raw_href,
    output reg boundary_sync_href,
    output reg boundary_prev_href,
    output reg [15:0] boundary_gap_count,
    output reg [2:0] boundary_source,
    // Observer-only diagnostics. None of these signals feed capture control.
    output reg [31:0] early_vsync_count,
    output reg [31:0] early_vsync_in_href_count,
    output reg [31:0] drop_by_early_vsync_count,
    output reg bad0_vsync_rise, bad0_early_boundary, bad0_event_href,
    output reg bad0_armed, bad0_end_pending
);
    (* ASYNC_REG = "TRUE" *) reg [1:0] cfg_pipe;
    // Native ADS property, not Vivado IOB. These registers are directly
    // connected to input ports; inspect Device Map to confirm IO packing.
    reg vs_q /* synthesis syn_useioff=1 */;
    reg href_q /* synthesis syn_useioff=1 */;
    reg [7:0] data_q /* synthesis syn_useioff=1 */;
    reg vs_prev, href_prev, armed, bad_line;
    reg [12:0] line_bytes;
    reg [12:0] href_high_pclk_count, capture_accept_count;
    reg [12:0] raw_href_pclk_count, raw_zero_pixel_count, raw_ff_pixel_count;
    reg [12:0] line_reject_count;
    reg [11:0] line_rej_x0, line_rej_x1, line_rej_x2, line_rej_x3;
    reg [3:0] line_rej_r0, line_rej_r1, line_rej_r2, line_rej_r3;
    reg [2:0] bad_record_count;
    reg [12:0] completed_line_bytes;
    reg [15:0] href_gap_count;
    reg [12:0] pad_remaining, pad_x;
    reg [10:0] pad_y;
    reg pad_last_line;
    reg [7:0] last_raw;
    reg [11:0] lines;
    reg [11:0] running_ok, running_bad;
    reg [12:0] running_min, running_max;
    localparam WARMUP_BITS = WARMUP_FRAMES < 2 ? 1 : $clog2(WARMUP_FRAMES+1);
    localparam DRAIN_BITS = FRAME_DRAIN_CYCLES < 2 ? 1 : $clog2(FRAME_DRAIN_CYCLES+1);
    reg [WARMUP_BITS-1:0] warmup_count;
    reg [DRAIN_BITS-1:0] drain_count;
    reg end_pending, pending_good;
    reg [3:0] pending_flags;
    wire vsync_rise = vs_q && !vs_prev;
    wire href_active = HREF_ACTIVE_LOW ? !href_q : href_q;
    wire href_prev_active = HREF_ACTIVE_LOW ? !href_prev : href_prev;
    // HREF entering before the Bayer drain ends is unsafe at either VSYNC
    // level. Keep this consistent with the HREF-only pixel qualification.
    wire drain_interrupted = vsync_rise || href_active;
    wire line_end = href_prev_active && !href_active;
    wire line_start = href_active && !href_prev_active;
    wire [11:0] final_lines = lines + (line_end ? 12'd1 : 12'd0);
    // Use a lower-bound compare instead of adding the tolerance on the
    // cam_pclk path.  It is algebraically identical for the non-negative
    // bounded tolerance used here, but avoids a 13-bit adder in timing.
    wire line_short_ok = (line_bytes <= WIDTH) &&
                         (line_bytes >= (WIDTH - SHORT_LINE_TOL));
    wire line_bad_now = line_end && !line_short_ok;
    wire boundary_bad = href_active || href_prev_active;
    // A raw HREF high cycle with the sampled HREF low (after the first
    // pipeline-prime cycle) is a capture-side sampling miss. It is logged as
    // a reason, never used to qualify or reject the pixel stream itself.
    wire raw_sample_gap = (HREF_ACTIVE_LOW ? !href : href) &&
                          !href_q && (raw_href_pclk_count != 0);
    wire [11:0] raw_sample_gap_x = raw_href_pclk_count[11:0];
    wire raw_count_mismatch = (raw_href_pclk_count != href_high_pclk_count);
    wire [11:0] raw_mismatch_x = raw_href_pclk_count != 0 ?
                                  (raw_href_pclk_count - 13'd1) : 12'd0;
    wire early_boundary = vsync_rise && armed && !end_pending &&
                          (final_lines < HEIGHT);
    wire complete = (final_lines == HEIGHT) && !bad_line &&
                    !line_bad_now && !boundary_bad && (pad_remaining == 0);

    // Keep row evidence separate from the legacy state machine. A new HREF
    // row always clears the previous row; an event on that very first cycle
    // wins over the clear. Blanking events never prime the following row.
    reg line_early_event_seen;
    reg [4:0] line_early_event_state;
    wire early_diag_event = vsync_rise && early_boundary;
    wire diag_first_pixel = (!vsync_rise || early_boundary) && armed && !end_pending &&
                            pad_remaining == 0 && href_active &&
                            line_bytes == 0 && lines == 0;
    wire diag_bad0_close = (!vsync_rise || early_boundary) && armed && !end_pending &&
                           pad_remaining == 0 && !href_active && line_end &&
                           !line_short_ok && bad_record_count == 0;
    always @(posedge pclk) begin
        if (!rst_n) begin
            early_vsync_count <= 0;
            early_vsync_in_href_count <= 0;
            drop_by_early_vsync_count <= 0;
        end else begin
            if (early_diag_event)
                early_vsync_count <= early_vsync_count + 32'd1;
            if (early_diag_event && href_q)
                early_vsync_in_href_count <= early_vsync_in_href_count + 32'd1;
            if (early_diag_event && href_q && armed && !end_pending)
                drop_by_early_vsync_count <= drop_by_early_vsync_count + 32'd1;
        end
    end
    always @(posedge pclk) begin
        if (!rst_n || !cfg_pipe[1]) begin
            line_early_event_seen <= 0;
            line_early_event_state <= 0;
            {bad0_vsync_rise,bad0_early_boundary,bad0_event_href,
             bad0_armed,bad0_end_pending} <= 0;
        end else begin
            if (line_start) begin
                line_early_event_seen <= 0;
                line_early_event_state <= 0;
            end
            if (early_diag_event && (href_active || line_end) &&
                (!line_early_event_seen || line_start)) begin
                line_early_event_seen <= 1;
                line_early_event_state <= {vsync_rise,early_boundary,
                                           href_q,armed,end_pending};
            end
            // Exactly the same record lifetime as the existing BAD0 bytes.
            // Do NOT clear row evidence here: an early VSYNC on the first
            // accepted pixel belongs to this row, not the previous frame.
            if (diag_first_pixel)
                {bad0_vsync_rise,bad0_early_boundary,bad0_event_href,
                 bad0_armed,bad0_end_pending} <= 0;
            if (diag_bad0_close)
                {bad0_vsync_rise,bad0_early_boundary,bad0_event_href,
                 bad0_armed,bad0_end_pending} <=
                    line_early_event_seen ? line_early_event_state :
                    (early_diag_event ? {vsync_rise,early_boundary,
                                          href_q,armed,end_pending} : 5'b00000);
            if (line_end) begin
                line_early_event_seen <= 0;
                line_early_event_state <= 0;
            end
        end
    end

    generate
        if (SAMPLE_NEGEDGE != 0) begin : g_sample_negedge
            always @(negedge pclk) begin
                if (!rst_n) cfg_pipe <= 0;
                else cfg_pipe <= {cfg_pipe[0], configured};
                vs_q <= vsync;
                href_q <= href;
                data_q <= data;
            end
        end else begin : g_sample_posedge
            always @(posedge pclk) begin
                if (!rst_n) cfg_pipe <= 0;
                else cfg_pipe <= {cfg_pipe[0], configured};
                vs_q <= vsync;
                href_q <= href;
                data_q <= data;
            end
        end
    endgenerate

    always @(posedge pclk) begin
        if (!rst_n || !cfg_pipe[1]) begin
            vs_prev <= 1'b0;
            // In the active-low mode an idle HREF is high, so seed the
            // previous level to inactive. This avoids a phantom line end at
            // the first configured clock after reset.
            href_prev <= HREF_ACTIVE_LOW ? 1'b1 : 1'b0;
            armed <= 1'b0; bad_line <= 1'b0;
            line_bytes <= 13'd0; completed_line_bytes <= 13'd0;
            href_high_pclk_count <= 13'd0; capture_accept_count <= 13'd0;
            raw_href_pclk_count <= 13'd0;
            raw_zero_pixel_count <= 13'd0;
            raw_ff_pixel_count <= 13'd0;
            bad_record_count <= 3'd0;
            pad_remaining <= 13'd0; pad_x <= 13'd0; pad_y <= 11'd0;
            pad_last_line <= 1'b0; last_raw <= 8'd0;
            lines <= 12'd0;
            raw <= 8'd0; valid <= 1'b0; sof <= 1'b0;
            eol <= 1'b0; eof <= 1'b0;
            x <= 12'd0; y <= 11'd0;
            frame_done <= 1'b0; frame_good <= 1'b0; frame_bad <= 1'b0;
            measured_lines <= 12'd0; last_line_bytes <= 13'd0;
            error_flags <= 4'd0;
            warmup_count <= 0; drain_count <= 0;
            end_pending <= 0; pending_good <= 0; pending_flags <= 0;
            running_ok <= 0; running_bad <= 0;
            running_min <= 13'h1fff; running_max <= 0;
            line_ok <= 0; line_bad <= 0;
            line_bytes_min <= 0; line_bytes_max <= 0;
            first_bad_line_y <= 0; first_bad_line_bytes <= 0;
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
            line_reject_count<=0;
            line_rej_x0<=0; line_rej_x1<=0; line_rej_x2<=0; line_rej_x3<=0;
            line_rej_r0<=0; line_rej_r1<=0; line_rej_r2<=0; line_rej_r3<=0;
            frame_start_pulse <= 0;
            coord_error <= 1'b0;
            boundary_event<=1'b0; boundary_line_y<=0;
            boundary_raw_href<=0; boundary_sync_href<=0;
            boundary_prev_href<=0; boundary_gap_count<=0;
            boundary_source<=0; href_gap_count<=0;
        end else begin
            vs_prev <= vs_q;
            href_prev <= href_q;
            valid <= 1'b0; sof <= 1'b0; eol <= 1'b0; eof <= 1'b0;
            frame_done <= 1'b0; frame_good <= 1'b0; frame_bad <= 1'b0;
            frame_start_pulse <= 1'b0;
            boundary_event<=1'b0;
            if (!href_active) begin
                if (href_gap_count != 16'hffff)
                    href_gap_count <= href_gap_count + 16'd1;
            end else href_gap_count<=0;
            // Raw-domain observability only. These counters use the direct
            // cam_href/cam_data pins on every cam_pclk edge and do not depend
            // on valid, accepted pixels, FIFO writes, or data transitions.
            if (HREF_ACTIVE_LOW ? !href : href) begin
                if (raw_href_pclk_count != 13'h1fff)
                    raw_href_pclk_count <= raw_href_pclk_count + 13'd1;
                if (data == 8'h00 && raw_zero_pixel_count != 13'h1fff)
                    raw_zero_pixel_count <= raw_zero_pixel_count + 13'd1;
                if (data == 8'hff && raw_ff_pixel_count != 13'h1fff)
                    raw_ff_pixel_count <= raw_ff_pixel_count + 13'd1;
            end
            if (armed && !end_pending && raw_sample_gap) begin
                if (line_reject_count != 13'h1fff)
                    line_reject_count <= line_reject_count + 13'd1;
                if (line_reject_count == 0) begin
                    line_rej_x0 <= raw_sample_gap_x; line_rej_r0 <= 4'd1;
                end else if (line_reject_count == 1) begin
                    line_rej_x1 <= raw_sample_gap_x; line_rej_r1 <= 4'd1;
                end else if (line_reject_count == 2) begin
                    line_rej_x2 <= raw_sample_gap_x; line_rej_r2 <= 4'd1;
                end else if (line_reject_count == 3) begin
                    line_rej_x3 <= raw_sample_gap_x; line_rej_r3 <= 4'd1;
                end
            end
            if (vsync_rise && armed) begin
                boundary_event<=1'b1;
                boundary_line_y<=final_lines[10:0];
                boundary_raw_href<=href;
                boundary_sync_href<=href_q;
                boundary_prev_href<=href_prev;
                boundary_gap_count<=href_gap_count;
                boundary_source<=3'd1; // VSYNC rising edge
            end
            if (end_pending) begin
                if (drain_interrupted) begin
                    pending_good <= 1'b0;
                    pending_flags[3] <= 1'b1;
                    armed <= 1'b0;
                end
                if (drain_count == 0) begin
                    end_pending <= 1'b0;
                    frame_done <= 1'b1;
                    frame_good <= pending_good && !drain_interrupted;
                    frame_bad <= !pending_good || drain_interrupted;
                    error_flags <= pending_flags | (drain_interrupted ? 4'b1000 : 4'b0000);
                end else drain_count <= drain_count - 1'b1;
            end
            // Only a non-early VSYNC consumes the boundary branch. An early
            // edge remains observable above, but must fall through to the
            // normal pixel/line-end path instead of skipping this PCLK beat.
            if (vsync_rise && !early_boundary) begin
                if (armed && !end_pending) begin
                    if (FRAME_DRAIN_CYCLES == 0) begin
                        frame_done <= 1'b1;
                        frame_good <= complete;
                        frame_bad <= !complete;
                    end else begin
                        end_pending <= 1'b1;
                        drain_count <= FRAME_DRAIN_CYCLES > 0 ? FRAME_DRAIN_CYCLES-1 : 0;
                        pending_good <= complete;
                        pending_flags <= {boundary_bad, (final_lines != HEIGHT),
                                          line_bad_now, bad_line};
                    end
                    measured_lines <= final_lines;
                    last_line_bytes <= line_end ? line_bytes : completed_line_bytes;
                    error_flags <= {boundary_bad, (final_lines != HEIGHT),
                                    (line_end && line_bad_now), bad_line};
                    line_ok <= running_ok + (line_end && !line_bad_now);
                    line_bad <= running_bad + line_bad_now;
                    line_bytes_min <= line_end && line_bytes < running_min ? line_bytes :
                                      (final_lines == 0 ? 13'd0 : running_min);
                    line_bytes_max <= line_end && line_bytes > running_max ? line_bytes : running_max;
                    if (running_bad == 0 && line_bad_now) begin
                        first_bad_line_y <= lines[10:0];
                        first_bad_line_bytes <= line_bytes;
                    end
                end
                if (!end_pending) begin
                    if (warmup_count < WARMUP_FRAMES) begin
                        warmup_count <= warmup_count + 1'b1;
                        armed <= 1'b0;
                    // A boundary inside HREF cannot establish the next
                    // first line: its tail may coincidentally be WIDTH.
                    // Wait for a later clean boundary instead of promoting
                    // that partial physical line into a valid frame.
                    end else armed <= !boundary_bad;
                end
                line_bytes <= 13'd0;
                completed_line_bytes <= 13'd0;
                raw_href_pclk_count <= 13'd0;
                raw_zero_pixel_count <= 13'd0;
                raw_ff_pixel_count <= 13'd0;
                pad_remaining <= 13'd0;
                pad_x <= 13'd0; pad_y <= 11'd0; pad_last_line <= 1'b0;
                lines <= 12'd0;
                bad_line <= 1'b0;
                running_ok <= 0; running_bad <= 0;
                running_min <= 13'h1fff; running_max <= 0;
                coord_error <= 1'b0;
            end else if (armed && !end_pending) begin
                // Complete a tolerated short line before the next HREF
                // window.  Repeating the final captured RAW sample preserves
                // x/y parity for the existing Bayer pipeline.
                if (pad_remaining != 0) begin
                    raw <= last_raw;
                    valid <= 1'b1;
                    x <= pad_x[11:0];
                    y <= pad_y;
                    eol <= (pad_remaining == 1);
                    eof <= (pad_remaining == 1 && pad_last_line);
                    if (pad_remaining == 1) begin
                        pad_remaining <= 13'd0;
                    end else begin
                        pad_remaining <= pad_remaining - 13'd1;
                        pad_x <= pad_x + 13'd1;
                    end
                end else if (href_active) begin
                    // The previous frame's verdict is emitted only after
                    // the drain interval.  Clear the diagnostic record on
                    // the first pixel of the next frame, after that verdict
                    // has been sampled by the top-level latch.
                    if (line_bytes == 0 && lines == 0) begin
                        first_bad_line_y <= 11'd0;
                        first_bad_line_bytes <= 13'd0;
                        bad_record_count <= 3'd0;
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
                        line_reject_count<=0;
                        line_rej_x0<=0; line_rej_x1<=0; line_rej_x2<=0; line_rej_x3<=0;
                        line_rej_r0<=0; line_rej_r1<=0; line_rej_r2<=0; line_rej_r3<=0;
                    end
                    if (href_high_pclk_count != 13'h1fff)
                        href_high_pclk_count <= href_high_pclk_count + 13'd1;
                    if (line_start && (line_bytes != 0 || lines >= HEIGHT))
                        coord_error <= 1'b1;
                    if (line_bytes < 13'h1fff)
                        line_bytes <= line_bytes + 13'd1;
                    else bad_line <= 1'b1;
                    if (line_bytes < WIDTH && lines < HEIGHT) begin
                        raw <= data_q;
                        last_raw <= data_q;
                        valid <= 1'b1;
                        if (capture_accept_count != 13'h1fff)
                            capture_accept_count <= capture_accept_count + 13'd1;
                        x <= line_bytes[11:0];
                        y <= lines[10:0];
                        sof <= (line_bytes == 0 && lines == 0);
                        frame_start_pulse <= (line_bytes == 0 && lines == 0);
                        eol <= (line_bytes == WIDTH - 1);
                        eof <= (line_bytes == WIDTH - 1 && lines == HEIGHT - 1);
                    end else begin
                        bad_line <= 1'b1;
                        if (line_reject_count != 13'h1fff)
                            line_reject_count <= line_reject_count + 13'd1;
                        if (line_reject_count == 0) begin
                            line_rej_x0 <= line_bytes[11:0];
                            line_rej_r0 <= lines >= HEIGHT ? 4'd3 : 4'd2;
                        end else if (line_reject_count == 1) begin
                            line_rej_x1 <= line_bytes[11:0];
                            line_rej_r1 <= lines >= HEIGHT ? 4'd3 : 4'd2;
                        end else if (line_reject_count == 2) begin
                            line_rej_x2 <= line_bytes[11:0];
                            line_rej_r2 <= lines >= HEIGHT ? 4'd3 : 4'd2;
                        end else if (line_reject_count == 3) begin
                            line_rej_x3 <= line_bytes[11:0];
                            line_rej_r3 <= lines >= HEIGHT ? 4'd3 : 4'd2;
                        end
                    end
                end else if (line_end) begin
                    // A raw-HREF/sample-count mismatch identifies a pixel
                    // lost between the physical DVP window and the sampled
                    // capture window.  This is diagnostic only: it never
                    // changes valid, raw, or downstream write enables.
                    // The next expected x is the first x not accepted by
                    // the sampled path; reason 4 means SAMPLE_COUNT_MISMATCH.
                    if (raw_count_mismatch) begin
                        if (line_reject_count != 13'h1fff)
                            line_reject_count <= line_reject_count + 13'd1;
                        if (line_reject_count == 0) begin
                            line_rej_x0 <= capture_accept_count[11:0];
                            line_rej_r0 <= 4'd4;
                        end else if (line_reject_count == 1) begin
                            line_rej_x1 <= capture_accept_count[11:0];
                            line_rej_r1 <= 4'd4;
                        end else if (line_reject_count == 2) begin
                            line_rej_x2 <= capture_accept_count[11:0];
                            line_rej_r2 <= 4'd4;
                        end else if (line_reject_count == 3) begin
                            line_rej_x3 <= capture_accept_count[11:0];
                            line_rej_r3 <= 4'd4;
                        end
                    end
                    if (!line_short_ok) bad_line <= 1'b1;
                    if (!line_short_ok) begin
                        if (running_bad != 12'hfff) running_bad <= running_bad + 1'b1;
                        if (bad_record_count == 0) begin
                            bad0_y<=lines[10:0]; bad0_href<=href_high_pclk_count;
                            bad0_accept<=capture_accept_count; bad0_bytes<=line_bytes;
                            bad0_raw_href<=raw_href_pclk_count;
                            bad0_zero<=raw_zero_pixel_count; bad0_ff<=raw_ff_pixel_count;
                        end else if (bad_record_count == 1) begin
                            bad1_y<=lines[10:0]; bad1_href<=href_high_pclk_count;
                            bad1_accept<=capture_accept_count; bad1_bytes<=line_bytes;
                            bad1_raw_href<=raw_href_pclk_count;
                            bad1_zero<=raw_zero_pixel_count; bad1_ff<=raw_ff_pixel_count;
                        end else if (bad_record_count == 2) begin
                            bad2_y<=lines[10:0]; bad2_href<=href_high_pclk_count;
                            bad2_accept<=capture_accept_count; bad2_bytes<=line_bytes;
                            bad2_raw_href<=raw_href_pclk_count;
                            bad2_zero<=raw_zero_pixel_count; bad2_ff<=raw_ff_pixel_count;
                        end
                        if (bad_record_count != 3'd3)
                            bad_record_count <= bad_record_count + 3'd1;
                        if (running_bad == 0) begin
                            first_bad_line_y <= lines[10:0];
                            first_bad_line_bytes <= line_bytes;
                            first_bad_reject_count <= line_reject_count +
                                                       (raw_count_mismatch ? 13'd1 : 13'd0);
                            // Copy the per-line record.  When the only
                            // rejection is the end-of-line sample mismatch,
                            // expose it directly in slot 0 despite the
                            // nonblocking assignments above.
                            first_bad_rej_x0 <= line_rej_x0;
                            first_bad_rej_r0 <= line_rej_r0;
                            first_bad_rej_x1 <= line_rej_x1;
                            first_bad_rej_r1 <= line_rej_r1;
                            first_bad_rej_x2 <= line_rej_x2;
                            first_bad_rej_r2 <= line_rej_r2;
                            first_bad_rej_x3 <= line_rej_x3;
                            first_bad_rej_r3 <= line_rej_r3;
                            // The mismatch slot is written on this same
                            // line-end edge, so override the copied record
                            // explicitly for every possible prior reject
                            // count (nonblocking assignments otherwise expose
                            // the previous line-reject slot).
                            if (raw_count_mismatch) begin
                                if (line_reject_count == 0) begin
                                    first_bad_rej_x0 <= capture_accept_count[11:0];
                                    first_bad_rej_r0 <= 4'd4;
                                end else if (line_reject_count == 1) begin
                                    first_bad_rej_x1 <= capture_accept_count[11:0];
                                    first_bad_rej_r1 <= 4'd4;
                                end else if (line_reject_count == 2) begin
                                    first_bad_rej_x2 <= capture_accept_count[11:0];
                                    first_bad_rej_r2 <= 4'd4;
                                end else if (line_reject_count == 3) begin
                                    first_bad_rej_x3 <= capture_accept_count[11:0];
                                    first_bad_rej_r3 <= 4'd4;
                                end
                            end
                        end
                    end else if (running_ok != 12'hfff) running_ok <= running_ok + 1'b1;
                    if (line_bytes < running_min) running_min <= line_bytes;
                    if (line_bytes > running_max) running_max <= line_bytes;
                    if (lines < 12'hfff) lines <= lines + 12'd1;
                    else bad_line <= 1'b1;
                    completed_line_bytes <= line_bytes;
                    href_high_pclk_count <= 13'd0;
                    capture_accept_count <= 13'd0;
                    raw_href_pclk_count <= 13'd0;
                    raw_zero_pixel_count <= 13'd0;
                    raw_ff_pixel_count <= 13'd0;
                    if (line_bytes < WIDTH && line_short_ok) begin
                        pad_remaining <= WIDTH - line_bytes;
                        pad_x <= line_bytes;
                        pad_y <= lines[10:0];
                        pad_last_line <= (lines == HEIGHT - 1);
                    end
                    line_bytes <= 13'd0;
                    // Reject records are per-line diagnostics.  Clear them
                    // only after the line-end snapshot above has consumed
                    // the current line's values.
                    line_reject_count <= 13'd0;
                    line_rej_x0 <= 12'd0; line_rej_x1 <= 12'd0;
                    line_rej_x2 <= 12'd0; line_rej_x3 <= 12'd0;
                    line_rej_r0 <= 4'd0; line_rej_r1 <= 4'd0;
                    line_rej_r2 <= 4'd0; line_rej_r3 <= 4'd0;
                end
            end
        end
    end
endmodule
