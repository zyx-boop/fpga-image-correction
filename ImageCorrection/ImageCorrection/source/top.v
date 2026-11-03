// Product-only D55-derived continuous correction. No UART/snapshot control.
module top #(
    parameter USE_HREF_ONLY = 1'b0,
    parameter CAMERA_TEST_PATTERN = 1'b0,
    // Diagnostic-only: select the lower OV5640 clock profile. Resolution
    // remains 1920x1080; the frame rate is allowed to fall temporarily.
    parameter LOW_PCLK_DIAG = 1'b1,
    // Keep this build focused on the requested pin-side counters.  The
    // optional RAW tile UART dump is disabled so the counters are visible
    // immediately after the legacy status fields; capture/DDR/HDMI paths are
    // unchanged.
    parameter RAW_TILE_DIAGNOSTIC = 1'b0,
    // Compile-time debug only. Default is fixed Bayer colour, no key mux.
    parameter DEBUG_RAW_GRAY = 1'b0,
    // Diagnostic-only strictness: 1 requires exactly 1920 bytes per line;
    // 0 restores the board-proven 16-byte short-line padding.
    parameter STRICT_LINE_LENGTH = 1'b1,
    // Compile-time PCLK sampling selection for A/B capture tests.
    parameter CAPTURE_NEGEDGE = 1'b0,
    // Diagnostic only: allow the DDR writer to publish a malformed/partial
    // frame for visual observation while strict line verdicts remain logged.
    parameter DEBUG_FORCE_COMMIT = 1'b0,
    // 1 bypasses OV5640 data and drives a known RGB565 frame into the live
    // packer/FIFO/DDR/HDMI path. It does not alter DDR or HDMI timing.
    parameter INTERNAL_TEST_PATTERN = 1'b0,
    // 0=RGGB, 1=BGGR (default, pending board colour check), 2=GRBG, 3=GBRG.
    parameter CFA_PHASE = 1
) (
    input wire sys_clk, key_n,
    input wire key_snapshot_n, uart_rx,
    input wire imu_rx,
    output wire servo_pan,servo_tilt,
    input wire clk_p, clk_n,
    input wire cam_pclk, cam_vsync, cam_href,
    input wire [7:0] cam_data,
    output wire cam_rst_n, cam_pwdn,
    inout wire cam_scl, cam_sda,
    output wire uart_tx,
    output wire [7:0] led,
    output wire pix_clk, hs_out, vs_out, de_out,
    output wire [7:0] r_out, g_out, b_out,
    output wire rstn_out, iic_tx_scl,
    inout wire iic_tx_sda,
    output wire mem_rst_n, mem_ck, mem_ck_n, mem_cke,
    output wire mem_cs_n, mem_ras_n, mem_cas_n, mem_we_n, mem_odt,
    output wire [14:0] mem_a,
    output wire [2:0] mem_ba,
    inout wire [3:0] mem_dqs, mem_dqs_n,
    inout wire [31:0] mem_dq,
    output wire [3:0] mem_dm
);
    wire cfg_clk, unused_clk, pll_lock;
    pll u_pll (.clkin1(sys_clk), .clkout0(pix_clk), .clkout1(cfg_clk),
               .clkout2(unused_clk), .lock(pll_lock));

    // The board demo deasserts the HDMI/DDR reset 1 ms after PLL lock.
    reg [13:0] reset_wait;
    always @(posedge cfg_clk) begin
        if (!pll_lock || !key_n) reset_wait <= 14'd0;
        else if (reset_wait != 14'd10000) reset_wait <= reset_wait + 14'd1;
    end
    reg board_reset_n = 1'b0;
    always @(posedge cfg_clk) begin
        if (!pll_lock || !key_n) board_reset_n <= 1'b0;
        else board_reset_n <= (reset_wait == 14'd10000);
    end
    assign rstn_out = board_reset_n;
    (* ASYNC_REG = "TRUE" *) reg [1:0] pix_reset_pipe;
    (* ASYNC_REG = "TRUE" *) reg [1:0] sys_reset_pipe;
    // The first synchronizer stages sample only the external key and PLL-lock
    // condition, never a 10 MHz register into a faster clock domain.
    wire reset_request_n = key_n && pll_lock;
    always @(posedge pix_clk)
        pix_reset_pipe <= {pix_reset_pipe[0], reset_request_n};
    always @(posedge sys_clk)
        sys_reset_pipe <= {sys_reset_pipe[0], reset_request_n};

    wire hdmi_init_done;
    ms72xx_ctl u_hdmi_i2c (
        .clk(cfg_clk), .rst_n(rstn_out), .init_over(hdmi_init_done),
        .init_over_rx(), .iic_scl(iic_tx_scl), .iic_sda(iic_tx_sda)
    );

    wire [11:0] x;
    wire [10:0] y;
    wire frame_start, vblank_start;
    wire hs_raw, vs_raw, de_raw;
    hdmi_1080p30_timing u_timing (
        .clk(pix_clk), .rst_n(pix_reset_pipe[1]), .hs(hs_raw), .vs(vs_raw),
        .de(de_raw), .x(x), .y(y), .frame_start(frame_start),
        .vblank_start(vblank_start)
    );
    reg hs_d, vs_d, de_d;
    reg [11:0] x_d;
    reg [10:0] y_d;
    always @(posedge pix_clk) begin
        if (!pix_reset_pipe[1]) begin
            hs_d <= 1'b1;
            vs_d <= 1'b1;
            de_d <= 1'b0;
            x_d <= 12'd0; y_d <= 0;
        end else begin
            hs_d <= hs_raw;
            vs_d <= vs_raw;
            de_d <= de_raw;
            x_d <= x; y_d <= y;
        end
    end


    wire [15:0] ddr_rgb565;
    wire ddr_pixel_valid, ddr_underflow;
    wire fifo_ready, fifo_empty, fifo_full;
    wire [255:0] fifo_read_data, fifo_write_data;
    wire [9:0] fifo_used;
    wire fifo_read, fifo_write;
    wire [27:0] awaddr, araddr;
    wire [3:0] awlen, arlen;
    wire awvalid, awready, arvalid, arready;
    wire [255:0] wdata, rdata;
    wire [31:0] wstrb;
    wire wready, wlast, rvalid, rlast;
    wire ddr_read_error, ddr_write_error;
    wire writer_addr_error, reader_addr_error, ddr_addr_error;
    wire [15:0] writer_addr_error_count, reader_addr_error_count;
    wire [27:0] writer_addr_error_actual, writer_addr_error_expected;
    wire [27:0] reader_addr_error_actual, reader_addr_error_expected;
    wire [3:0] writer_addr_error_code, reader_addr_error_code;
    wire [16:0] ddr_addr_error_sum =
        {1'b0,writer_addr_error_count}+{1'b0,reader_addr_error_count};
    wire [15:0] ddr_addr_error_count = ddr_addr_error_sum[16] ?
        16'hffff : ddr_addr_error_sum[15:0];
    assign ddr_addr_error = writer_addr_error | reader_addr_error;
    wire ddr_core_clk, ddr_init_done;
    (* ASYNC_REG = "TRUE" *) reg [1:0] core_reset_pipe = 2'b00;
    always @(posedge ddr_core_clk)
        core_reset_pipe <= {core_reset_pipe[0], reset_request_n};
    wire stream_reset_n = core_reset_pipe[1] && pix_reset_pipe[1];

    // Pixel-domain vertical blank enters the DDR domain as a toggle event.
    reg vblank_toggle;
    always @(posedge pix_clk)
        if (!pix_reset_pipe[1]) vblank_toggle <= 1'b0;
        else if (vblank_start) vblank_toggle <= ~vblank_toggle;
    (* ASYNC_REG = "TRUE" *) reg [2:0] vblank_sync;
    always @(posedge ddr_core_clk)
        if (!core_reset_pipe[1]) vblank_sync <= 3'b000;
        else vblank_sync <= {vblank_sync[1:0],vblank_toggle};
    wire core_vblank = vblank_sync[2] ^ vblank_sync[1];

    wire [258:0] camera_token, camera_fifo_data;
    wire camera_token_valid, camera_fifo_full, camera_fifo_empty;
    wire camera_fifo_wr_overflow, camera_fifo_rd_underflow;
    wire [9:0] camera_fifo_used_core;
    wire video_fifo_wr_overflow, video_fifo_rd_underflow;
    wire camera_fifo_read, camera_packer_overflow, camera_packer_row_error;
    wire camera_packer_fifo_full_stall;
    wire writer_active, display_valid, pending_valid, dropped_frame;
    wire [1:0] writer_bank, display_bank, pending_bank;
    wire writer_frame_start, writer_frame_finish, writer_frame_good;
    wire writer_frame_end_pulse, writer_frame_end_token_good;
    wire [17:0] writer_frame_end_word_count;
    wire [27:0] writer_frame_end_addr_actual, writer_frame_end_addr_expected;
    wire continuous_core_reset_n,continuous_cam_reset_n,continuous_pixel_reset_n,continuous_sys_reset_n;
    wire continuous_transport_ready,continuous_camera_fifo_write;
    (* ASYNC_REG="TRUE" *) reg [2:0] snapshot_key_core=3'b111;
    reg snapshot_key_stable=1'b1,single_key_pulse;
    reg [20:0] snapshot_key_count;
    always @(posedge ddr_core_clk)begin
        if(!core_reset_pipe[1])begin snapshot_key_core<=3'b111;snapshot_key_stable<=1;
            snapshot_key_count<=0;single_key_pulse<=0;end
        else begin
            snapshot_key_core<={snapshot_key_core[1:0],key_snapshot_n};single_key_pulse<=0;
            if(snapshot_key_core[2]==snapshot_key_stable)snapshot_key_count<=0;
            else if(snapshot_key_count==21'd1249999)begin
                single_key_pulse<=snapshot_key_stable&&!snapshot_key_core[2];
                snapshot_key_stable<=snapshot_key_core[2];snapshot_key_count<=0;
            end else snapshot_key_count<=snapshot_key_count+1'b1;
        end
    end
    // Continuous Scheme-A integration. No legacy one-shot owner/export FSM
    // participates in allocation. Key1 is sticky-start only inside Manager.
    wire [31:0] continuous_job,continuous_fid,continuous_publish,continuous_input,continuous_drop;
    wire [1:0] continuous_target;
    wire [4:0] continuous_stage;
    wire continuous_error,continuous_display_lease;
    wire [7:0] continuous_error_code;
    wire continuous_start_requested;
    continuous_start_request u_start_request(.clk(ddr_core_clk),.system_reset_n(core_reset_pipe[1]),
        .key_pulse(single_key_pulse),.start_request(continuous_start_requested));
    assign pending_valid=1'b0;
    assign writer_bank=2'd0;
    assign pending_bank=2'd0;
    assign dropped_frame=continuous_error;
    wire [63:0] observed_latency;
    wire [31:0] observed_count;
    wire observed_error;
    wire [15:0] imu_roll,imu_pitch,imu_yaw;
    wire imu_fresh;wire [31:0] imu_sequence;
    wire imu_has_sample,imu_rx_level;
    wire [15:0] imu_age,imu_bad,imu_idle_ms;
    wire [31:0] imu_bytes,imu_edges,imu_min_ticks;
    wire [7:0] imu_baud;
    r15_jy61_uart imu_receiver(.clk(sys_clk),.rst_n(sys_reset_pipe[1]),.rx(imu_rx),
      .roll(imu_roll),.pitch(imu_pitch),.yaw(imu_yaw),.fresh(imu_fresh),.sample_count(imu_sequence),
      .has_sample(imu_has_sample),.age_ms(imu_age),.byte_count(imu_bytes),.bad_count(imu_bad),.baud_code(imu_baud),.rx_level(imu_rx_level),.edge_count(imu_edges),.min_edge_ticks(imu_min_ticks),.idle_ms(imu_idle_ms));
    continuous_video_pipeline u_continuous(
        .observed_latency(observed_latency),.observed_count(observed_count),.observed_error(observed_error),
        .imu_pose_sys({imu_fresh,imu_sequence,imu_roll,imu_pitch,imu_yaw}),
        .core_clk(ddr_core_clk),.pixel_clk(pix_clk),.cam_clk(cam_pclk),.sys_clk(sys_clk),.rst_n(reset_request_n),
        .core_reset_n(continuous_core_reset_n),.cam_reset_n(continuous_cam_reset_n),
        .pixel_reset_n(continuous_pixel_reset_n),.sys_reset_n(continuous_sys_reset_n),
        .transport_ready_core(continuous_transport_ready),.camera_fifo_write(continuous_camera_fifo_write),
        .init_done(ddr_init_done),.key_pulse(continuous_start_requested),.vblank(core_vblank),.de(de_raw),.frame_start(frame_start),
        .capture_sof(stream_frame_start),.capture_done(stream_frame_done),.capture_good(stream_frame_good),
        .token_wr_en(camera_token_valid),.token_wr_data(camera_token),
        .camera_fifo_empty(camera_fifo_empty),.camera_fifo_data(camera_fifo_data),.camera_fifo_read(camera_fifo_read),
        .camera_fifo_used_core(camera_fifo_used_core),
        .arvalid(arvalid),.arready(arready),.araddr(araddr),.arlen(arlen),.rvalid(rvalid),.rlast(rlast),.rdata(rdata),
        .awvalid(awvalid),.awready(awready),.awaddr(awaddr),.awlen(awlen),.wdata(wdata),.wstrb(wstrb),.wready(wready),.wlast(wlast),
        .rgb565(ddr_rgb565),.pixel_valid(ddr_pixel_valid),.error(continuous_error),.error_code(continuous_error_code),
        .job_seq(continuous_job),.source_frame_id(continuous_fid),.publish_count(continuous_publish),.input_frame(continuous_input),.drop_total(continuous_drop),
        .display_valid(display_valid),.display_bank(display_bank),.target_bank(continuous_target),.bank_stage(continuous_stage),.display_lease(continuous_display_lease),
        .writer_addr_error(writer_addr_error),.writer_addr_error_count(writer_addr_error_count),.writer_addr_error_code(writer_addr_error_code),
        .writer_addr_error_actual(writer_addr_error_actual),.writer_addr_error_expected(writer_addr_error_expected),
        .ddr_write_error(ddr_write_error),.ddr_read_error(ddr_read_error),.underflow(ddr_underflow),
        .reader_addr_error(reader_addr_error),.reader_addr_error_count(reader_addr_error_count),.reader_addr_actual(reader_addr_error_actual),
        .reader_addr_expected(reader_addr_expected),.reader_addr_code(reader_addr_error_code),
        .video_fifo_wr_overflow(video_fifo_wr_overflow),.video_fifo_rd_underflow(video_fifo_rd_underflow),.fifo_ready(fifo_ready),
        .camera_writer_active(writer_active),.camera_frame_start(writer_frame_start),.camera_frame_finish(writer_frame_finish),.camera_frame_good(writer_frame_good));
    (* ASYNC_REG="TRUE" *)reg [1:0] batch_display_pix;
    always @(posedge pix_clk)begin
        if(!pix_reset_pipe[1])batch_display_pix<=0;
        else batch_display_pix<={batch_display_pix[0],display_valid&&!continuous_error};
    end
    wire [23:0] display_rgb;
    wire [19:0] latency_digits;
    wire latency_valid,latency_overflow;
    latency_readout u_latency_readout(.core_clk(ddr_core_clk),.core_rst_n(continuous_core_reset_n),
        .pixel_clk(pix_clk),.pixel_rst_n(continuous_pixel_reset_n),.frame_start(frame_start),
        .latency_cycles(observed_latency),.displayed_count(observed_count),.measurement_error(observed_error),
        .digits(latency_digits),.valid(latency_valid),.overflow(latency_overflow));
    latency_osd u_latency_osd(.clk(pix_clk),.rst_n(pix_reset_pipe[1]),
        .x(x_d),.y(y_d),.hs(hs_d),.vs(vs_d),.de(de_d),.rgb(display_rgb),
        .digits(latency_digits),.valid(latency_valid),.overflow(latency_overflow),
        .out_hs(hs_out),.out_vs(vs_out),.out_de(de_out),.out_rgb({r_out,g_out,b_out}));
    hdmi_rgb_out u_rgb(
        .de(de_d),.x(x_d),.rgb565(batch_display_pix[1]&&ddr_pixel_valid?ddr_rgb565:16'd0),.pixel_valid(1'b1),
        .r(display_rgb[23:16]),.g(display_rgb[15:8]),.b(display_rgb[7:0]));
    // UART protocol removed; retain the existing legal idle pad/FDC binding.
    wire gimbal_tx,gimbal_pending,gimbal_owned,diag_serial_busy;
    r17_gimbal_control u_gimbal(.clk(sys_clk),.rst_n(sys_reset_pipe[1]),.rs232_rx(uart_rx),
      .imu_roll(imu_roll),.imu_pitch(imu_pitch),.imu_yaw(imu_yaw),.imu_age(imu_age),.imu_bad(imu_bad),
      .imu_idle_ms(imu_idle_ms),.imu_samples(imu_sequence),.imu_bytes(imu_bytes),.imu_edges(imu_edges),
      .imu_min_ticks(imu_min_ticks),.imu_baud(imu_baud),.imu_has_sample(imu_has_sample),
      .imu_fresh(imu_fresh),.imu_rx_level(imu_rx_level),
      .tx_grant(!diag_serial_busy),.tx_pending(gimbal_pending),.tx_owned(gimbal_owned),
      .rs232_tx(gimbal_tx),.servo_pan(servo_pan),.servo_tilt(servo_tilt));
    assign uart_tx=gimbal_owned?gimbal_tx:diag_uart_tx;

    ddr3_bridge u_ddr (
        .clk_p(clk_p), .clk_n(clk_n), .reset_n(rstn_out),
        .core_clk(ddr_core_clk), .init_done(ddr_init_done),
        .awaddr(awaddr), .awlen(awlen), .awvalid(awvalid),
        .awready(awready), .wdata(wdata), .wstrb(wstrb),
        .wready(wready), .wlast(wlast), .araddr(araddr),
        .arlen(arlen), .arvalid(arvalid), .arready(arready),
        .rdata(rdata), .rvalid(rvalid), .rlast(rlast),
        .mem_rst_n(mem_rst_n), .mem_ck(mem_ck), .mem_ck_n(mem_ck_n),
        .mem_cke(mem_cke), .mem_cs_n(mem_cs_n), .mem_ras_n(mem_ras_n),
        .mem_cas_n(mem_cas_n), .mem_we_n(mem_we_n), .mem_odt(mem_odt),
        .mem_a(mem_a), .mem_ba(mem_ba), .mem_dqs(mem_dqs),
        .mem_dqs_n(mem_dqs_n), .mem_dq(mem_dq), .mem_dm(mem_dm)
    );

    wire camera_power_ready, camera_id_ok, camera_id_error;
    wire [1:0] camera_power_state;
    wire [31:0] camera_power_count;
    wire [15:0] camera_id;
    wire [3:0] camera_id_code;
    wire camera_id_low_failed;
    camera_power_seq u_cam_power (
        .clk(sys_clk),.rst_n(sys_reset_pipe[1]),
        .cam_pwdn(cam_pwdn),.cam_rst_n(cam_rst_n),
        .ready(camera_power_ready),
        .debug_state(camera_power_state),.debug_count(camera_power_count)
    );
    wire id_start,init_start,init_read,cam_configured,cam_init_failed;
    wire diag_bus_start,diag_uart_tx;wire[15:0]diag_bus_addr;
    wire [8:0] init_failed_index;
    wire [15:0] init_failed_addr;
    wire [15:0] id_addr,init_addr;
    wire [7:0] init_data,sccb_data;
    wire sccb_done,sccb_error,sccb_busy,sccb_scl;
    wire [3:0] sccb_error_code;
    wire [7:0] pre_r4740,pre_r4741;
    wire pre_read_ok,pre_scl_idle,pre_sda_idle;
    ov5640_id_reader u_cam_id (
        .clk(sys_clk),.rst_n(sys_reset_pipe[1]),
        .power_ready(camera_power_ready),.start(id_start),
        .reg_addr(id_addr),.done(sccb_done),.bus_error(sccb_error),
        .bus_error_code(sccb_error_code),.rd_data(sccb_data),
        .id_ok(camera_id_ok),.id_error(camera_id_error),
        .chip_id(camera_id),.diagnostic_code(camera_id_code),
        .failed_low_register(camera_id_low_failed)
    );
    ov5640_init_raw1080 #(.TEST_COLORBAR(CAMERA_TEST_PATTERN),
                          .LOW_PCLK_DIAG(LOW_PCLK_DIAG)) u_cam_init (
        .clk(sys_clk),.rst_n(sys_reset_pipe[1]),.enable(camera_id_ok),
        .scl_idle_in(cam_scl),.sda_idle_in(cam_sda),
        .start(init_start),.read_en(init_read),.reg_addr(init_addr),
        .wr_data(init_data),.bus_done(sccb_done),.bus_error(sccb_error),
        .rd_data(sccb_data),.configured(cam_configured),
        .failed(cam_init_failed),.failed_index(init_failed_index),
        .failed_addr(init_failed_addr),
        .expected_data(),.observed_data(),
        .pre_r4740(pre_r4740),.pre_r4741(pre_r4741),
        .pre_read_ok(pre_read_ok),.pre_scl_idle(pre_scl_idle),
        .pre_sda_idle(pre_sda_idle)
    );
    // SCL and SDA use open-drain drive; the board provides pull-ups.
    assign cam_scl = sccb_scl ? 1'bz : 1'b0;
    sccb_master u_cam_sccb (
        .clk(sys_clk),.rst_n(sys_reset_pipe[1]),
        .start(cam_configured ? diag_bus_start : (camera_id_ok ? init_start : id_start)),
        .read_en(cam_configured ? 1'b1 : (camera_id_ok ? init_read : 1'b1)),
        .reg_addr(cam_configured ? diag_bus_addr : (camera_id_ok ? init_addr : id_addr)),
        .wr_data(init_data),.rd_data(sccb_data),
        .busy(sccb_busy),.done(sccb_done),.error(sccb_error),
        .error_code(sccb_error_code),.scl(sccb_scl),
        .scl_in(cam_scl),.sda(cam_sda),.probe_enable(1'b0),
        .probe_windows(),.probe_flags(),.probe_first_valid(),
        .probe_result_valid(),.probe_pattern(),.probe_position(),
        .probe_expected(),.probe_result_code(),.probe_result_data()
    );

    // PCLK may be absent while the camera is held in reset. Assert reset
    // without PCLK; release it only after two camera-clock edges.
    (* ASYNC_REG = "TRUE" *) reg [1:0] cam_reset_pipe = 2'b00;
    always @(posedge cam_pclk or negedge reset_request_n)
        if (!reset_request_n) cam_reset_pipe <= 2'b00;
        else cam_reset_pipe <= {cam_reset_pipe[0],1'b1};
    wire [7:0] capture_raw_pixel;
    wire capture_raw_valid,capture_raw_sof,capture_raw_eol,capture_raw_eof;
    wire [11:0] capture_raw_x,capture_measured_lines;
    wire [10:0] capture_raw_y;
    wire [12:0] capture_last_line_bytes;
    wire [3:0] capture_frame_error_flags;
    wire capture_frame_done,capture_frame_good,capture_frame_bad;
    wire capture_boundary_event;
    wire [10:0] capture_boundary_line_y;
    wire capture_boundary_raw_href,capture_boundary_sync_href,capture_boundary_prev_href;
    wire [15:0] capture_boundary_gap_count;
    wire [2:0] capture_boundary_source;
    wire [11:0] capture_line_ok,capture_line_bad;
    wire [12:0] capture_line_min,capture_line_max;
    wire [10:0] capture_first_bad_y;
    wire [12:0] capture_first_bad_bytes;
    wire [10:0] capture_bad0_y,capture_bad1_y,capture_bad2_y;
    wire [12:0] capture_bad0_href,capture_bad1_href,capture_bad2_href;
    wire [12:0] capture_bad0_accept,capture_bad1_accept,capture_bad2_accept;
    wire [12:0] capture_bad0_bytes,capture_bad1_bytes,capture_bad2_bytes;
    wire [12:0] capture_bad0_raw_href,capture_bad1_raw_href,capture_bad2_raw_href;
    wire [12:0] capture_bad0_zero,capture_bad1_zero,capture_bad2_zero;
    wire [12:0] capture_bad0_ff,capture_bad1_ff,capture_bad2_ff;
    wire [12:0] capture_first_bad_reject_count;
    wire [11:0] capture_first_bad_rej_x0,capture_first_bad_rej_x1;
    wire [11:0] capture_first_bad_rej_x2,capture_first_bad_rej_x3;
    wire [3:0] capture_first_bad_rej_r0,capture_first_bad_rej_r1;
    wire [3:0] capture_first_bad_rej_r2,capture_first_bad_rej_r3;
    wire [16:0] capture_gap_pclk;
    wire capture_frame_start;
    wire capture_coord_error;
    wire [31:0] capture_early_vsync_count,capture_early_vsync_in_href_count;
    wire [31:0] capture_drop_by_early_vsync_count;
    wire [4:0] capture_bad0_early_state;
    wire [7:0] raw_pixel;
    wire raw_valid,raw_sof,raw_eol,raw_eof;
    wire [11:0] raw_x,measured_lines;
    wire [10:0] raw_y;
    wire [12:0] last_line_bytes;
    wire [3:0] frame_error_flags;
    wire raw_frame_done,raw_frame_good,raw_frame_bad;
    wire [11:0] href_line_ok,href_line_bad;
    wire [12:0] href_line_min,href_line_max;
    wire [10:0] href_first_bad_y;
    wire [12:0] href_first_bad_bytes;
    wire [10:0] href_bad0_y,href_bad1_y,href_bad2_y;
    wire [12:0] href_bad0_href,href_bad1_href,href_bad2_href;
    wire [12:0] href_bad0_accept,href_bad1_accept,href_bad2_accept;
    wire [12:0] href_bad0_bytes,href_bad1_bytes,href_bad2_bytes;
    wire [12:0] href_bad0_raw_href,href_bad1_raw_href,href_bad2_raw_href;
    wire [12:0] href_bad0_zero,href_bad1_zero,href_bad2_zero;
    wire [12:0] href_bad0_ff,href_bad1_ff,href_bad2_ff;
    wire [16:0] href_gap_pclk;
    wire href_frame_start;
    wire coord_error;
    wire [15:0] pattern_pixel;
    wire pattern_valid,pattern_sof,pattern_eol,pattern_eof;
    wire pattern_frame_done,pattern_frame_good,pattern_frame_bad;
    wire [11:0] pattern_x,pattern_measured_lines,pattern_line_ok,pattern_line_bad;
    wire [10:0] pattern_y;
    wire [12:0] pattern_last_line_bytes,pattern_line_min,pattern_line_max;
    wire [10:0] pattern_first_bad_y;
    wire [12:0] pattern_first_bad_bytes;
    wire [10:0] pattern_bad0_y,pattern_bad1_y,pattern_bad2_y;
    wire [12:0] pattern_bad0_href,pattern_bad1_href,pattern_bad2_href;
    wire [12:0] pattern_bad0_accept,pattern_bad1_accept,pattern_bad2_accept;
    wire [12:0] pattern_bad0_bytes,pattern_bad1_bytes,pattern_bad2_bytes;
    wire [12:0] pattern_bad0_raw_href,pattern_bad1_raw_href,pattern_bad2_raw_href;
    wire [12:0] pattern_bad0_zero,pattern_bad1_zero,pattern_bad2_zero;
    wire [12:0] pattern_bad0_ff,pattern_bad1_ff,pattern_bad2_ff;
    wire [3:0] pattern_error_flags;
    wire pattern_frame_start;
    wire stream_frame_done,stream_frame_good,stream_frame_bad;
    wire [15:0] cam_pclk_blocks,cam_vsync_edges,cam_href_edges;
    wire [15:0] cam_frame_edges,cam_good_edges;
    generate if (USE_HREF_ONLY) begin : g_href_capture
        assign capture_early_vsync_count=0;
        assign capture_early_vsync_in_href_count=0;
        assign capture_drop_by_early_vsync_count=0;
        assign capture_bad0_early_state=0;
        // Keep HREF-only available as a diagnostic fallback, but accept the
        // measured 1908..1920-byte line range.  Strict zero tolerance made
        // every short physical line invalidate the whole frame and left HDMI
        // displaying the previous frame (the observed distance-dependent
        // freeze).
        href_only_capture #(.SHORT_LINE_TOL(STRICT_LINE_LENGTH ? 0 : 16),.LONG_LINE_TOL(0)) u_capture (
            .sys_clk(sys_clk),.cam_pclk(cam_pclk),
            .rst_n(cam_reset_pipe[1]),.configured(cam_configured),
            .cam_href(cam_href),.cam_data(cam_data),.raw(capture_raw_pixel),
            .valid(capture_raw_valid),.sof(capture_raw_sof),.eol(capture_raw_eol),.eof(capture_raw_eof),
            .x(capture_raw_x),.y(capture_raw_y),.frame_done(capture_frame_done),
            .frame_good(capture_frame_good),.frame_bad(capture_frame_bad),
            .measured_lines(capture_measured_lines),.last_line_bytes(capture_last_line_bytes),
            .error_flags(capture_frame_error_flags),.line_ok(capture_line_ok),
            .line_bad(capture_line_bad),.line_bytes_min(capture_line_min),
            .line_bytes_max(capture_line_max),
            .first_bad_line_y(capture_first_bad_y),
            .first_bad_line_bytes(capture_first_bad_bytes),
            .bad0_y(capture_bad0_y),.bad1_y(capture_bad1_y),.bad2_y(capture_bad2_y),
            .bad0_href(capture_bad0_href),.bad1_href(capture_bad1_href),.bad2_href(capture_bad2_href),
            .bad0_accept(capture_bad0_accept),.bad1_accept(capture_bad1_accept),.bad2_accept(capture_bad2_accept),
            .bad0_bytes(capture_bad0_bytes),.bad1_bytes(capture_bad1_bytes),.bad2_bytes(capture_bad2_bytes),
            .bad0_raw_href(capture_bad0_raw_href),.bad1_raw_href(capture_bad1_raw_href),.bad2_raw_href(capture_bad2_raw_href),
            .bad0_zero(capture_bad0_zero),.bad1_zero(capture_bad1_zero),.bad2_zero(capture_bad2_zero),
            .bad0_ff(capture_bad0_ff),.bad1_ff(capture_bad1_ff),.bad2_ff(capture_bad2_ff),
            .first_bad_reject_count(capture_first_bad_reject_count),
            .first_bad_rej_x0(capture_first_bad_rej_x0),.first_bad_rej_x1(capture_first_bad_rej_x1),
            .first_bad_rej_x2(capture_first_bad_rej_x2),.first_bad_rej_x3(capture_first_bad_rej_x3),
            .first_bad_rej_r0(capture_first_bad_rej_r0),.first_bad_rej_r1(capture_first_bad_rej_r1),
            .first_bad_rej_r2(capture_first_bad_rej_r2),.first_bad_rej_r3(capture_first_bad_rej_r3),
            .frame_start_pulse(capture_frame_start),.coord_error(capture_coord_error),
            .gap_pclk_before_frame_end(capture_gap_pclk),
            .boundary_event(capture_boundary_event),
            .boundary_line_y(capture_boundary_line_y),
            .boundary_raw_href(capture_boundary_raw_href),
            .boundary_sync_href(capture_boundary_sync_href),
            .boundary_prev_href(capture_boundary_prev_href),
            .boundary_gap_count(capture_boundary_gap_count),
            .boundary_source(capture_boundary_source)
        );
    end else begin : g_vsync_capture
        // Keep the board-proven high-active HREF interpretation.  The UART
        // baseline shows 1908-1920-byte lines; tolerate only that measured
        // short-line range and repeat the final sample to restore 1920.
        dvp_frame_capture #(.WARMUP_FRAMES(8),.FRAME_DRAIN_CYCLES(1936),
            // The board baseline has measured short lines (about 1908..1920
            // bytes).  Pad only this bounded deficit; long lines remain bad.
            .SAMPLE_NEGEDGE(CAPTURE_NEGEDGE),.HREF_ACTIVE_LOW(0),
            .SHORT_LINE_TOL(STRICT_LINE_LENGTH ? 0 : 16)) u_capture (
            .pclk(cam_pclk),.rst_n(cam_reset_pipe[1]),
            .configured(cam_configured),.vsync(cam_vsync),
            .early_vsync_count(capture_early_vsync_count),
            .early_vsync_in_href_count(capture_early_vsync_in_href_count),
            .drop_by_early_vsync_count(capture_drop_by_early_vsync_count),
            .bad0_vsync_rise(capture_bad0_early_state[4]),
            .bad0_early_boundary(capture_bad0_early_state[3]),
            .bad0_event_href(capture_bad0_early_state[2]),
            .bad0_armed(capture_bad0_early_state[1]),
            .bad0_end_pending(capture_bad0_early_state[0]),
            .href(cam_href),.data(cam_data),.raw(capture_raw_pixel),
            .valid(capture_raw_valid),.sof(capture_raw_sof),.eol(capture_raw_eol),.eof(capture_raw_eof),
            .x(capture_raw_x),.y(capture_raw_y),.frame_done(capture_frame_done),
            .frame_good(capture_frame_good),.frame_bad(capture_frame_bad),
            .measured_lines(capture_measured_lines),.last_line_bytes(capture_last_line_bytes),
            .error_flags(capture_frame_error_flags),.line_ok(capture_line_ok),
            .line_bad(capture_line_bad),.line_bytes_min(capture_line_min),
            .line_bytes_max(capture_line_max),
            .first_bad_line_y(capture_first_bad_y),
            .first_bad_line_bytes(capture_first_bad_bytes),
            .bad0_y(capture_bad0_y),.bad1_y(capture_bad1_y),.bad2_y(capture_bad2_y),
            .bad0_href(capture_bad0_href),.bad1_href(capture_bad1_href),.bad2_href(capture_bad2_href),
            .bad0_accept(capture_bad0_accept),.bad1_accept(capture_bad1_accept),.bad2_accept(capture_bad2_accept),
            .bad0_bytes(capture_bad0_bytes),.bad1_bytes(capture_bad1_bytes),.bad2_bytes(capture_bad2_bytes),
            .bad0_raw_href(capture_bad0_raw_href),.bad1_raw_href(capture_bad1_raw_href),.bad2_raw_href(capture_bad2_raw_href),
            .bad0_zero(capture_bad0_zero),.bad1_zero(capture_bad1_zero),.bad2_zero(capture_bad2_zero),
            .bad0_ff(capture_bad0_ff),.bad1_ff(capture_bad1_ff),.bad2_ff(capture_bad2_ff),
            .first_bad_reject_count(capture_first_bad_reject_count),
            .first_bad_rej_x0(capture_first_bad_rej_x0),.first_bad_rej_x1(capture_first_bad_rej_x1),
            .first_bad_rej_x2(capture_first_bad_rej_x2),.first_bad_rej_x3(capture_first_bad_rej_x3),
            .first_bad_rej_r0(capture_first_bad_rej_r0),.first_bad_rej_r1(capture_first_bad_rej_r1),
            .first_bad_rej_r2(capture_first_bad_rej_r2),.first_bad_rej_r3(capture_first_bad_rej_r3),
            .frame_start_pulse(capture_frame_start),
            .coord_error(capture_coord_error),
            .boundary_event(capture_boundary_event),
            .boundary_line_y(capture_boundary_line_y),
            .boundary_raw_href(capture_boundary_raw_href),
            .boundary_sync_href(capture_boundary_sync_href),
            .boundary_prev_href(capture_boundary_prev_href),
            .boundary_gap_count(capture_boundary_gap_count),
            .boundary_source(capture_boundary_source)
        );
        assign capture_gap_pclk=17'd0;
    end endgenerate
    assign raw_pixel = capture_raw_pixel;
    assign raw_valid = capture_raw_valid;
    assign raw_sof = capture_raw_sof;
    assign raw_eol = capture_raw_eol;
    assign raw_eof = capture_raw_eof;
    assign raw_x = capture_raw_x;
    assign raw_y = capture_raw_y;
    assign measured_lines = capture_measured_lines;
    assign last_line_bytes = capture_last_line_bytes;
    assign frame_error_flags = capture_frame_error_flags;
    assign raw_frame_done = capture_frame_done;
    assign raw_frame_good = capture_frame_good;
    assign raw_frame_bad = capture_frame_bad;
    assign href_line_ok = capture_line_ok;
    assign href_line_bad = capture_line_bad;
    assign href_line_min = capture_line_min;
    assign href_line_max = capture_line_max;
    assign href_first_bad_y = capture_first_bad_y;
    assign href_first_bad_bytes = capture_first_bad_bytes;
    assign href_bad0_y=capture_bad0_y; assign href_bad1_y=capture_bad1_y; assign href_bad2_y=capture_bad2_y;
    assign href_bad0_href=capture_bad0_href; assign href_bad1_href=capture_bad1_href; assign href_bad2_href=capture_bad2_href;
    assign href_bad0_accept=capture_bad0_accept; assign href_bad1_accept=capture_bad1_accept; assign href_bad2_accept=capture_bad2_accept;
    assign href_bad0_bytes=capture_bad0_bytes; assign href_bad1_bytes=capture_bad1_bytes; assign href_bad2_bytes=capture_bad2_bytes;
    assign href_bad0_raw_href=capture_bad0_raw_href; assign href_bad1_raw_href=capture_bad1_raw_href; assign href_bad2_raw_href=capture_bad2_raw_href;
    assign href_bad0_zero=capture_bad0_zero; assign href_bad1_zero=capture_bad1_zero; assign href_bad2_zero=capture_bad2_zero;
    assign href_bad0_ff=capture_bad0_ff; assign href_bad1_ff=capture_bad1_ff; assign href_bad2_ff=capture_bad2_ff;
    assign href_gap_pclk = capture_gap_pclk;
    assign href_frame_start = capture_frame_start;
    assign coord_error = capture_coord_error;
    rgb565_internal_test_pattern u_internal_pattern (
        .clk(cam_pclk),.rst_n(cam_reset_pipe[1]),.enable(INTERNAL_TEST_PATTERN),
        .pixel(pattern_pixel),.valid(pattern_valid),.sof(pattern_sof),
        .eol(pattern_eol),.eof(pattern_eof),.frame_done(pattern_frame_done),
        .frame_good(pattern_frame_good),.frame_bad(pattern_frame_bad),
        .x(pattern_x),.y(pattern_y),.measured_lines(pattern_measured_lines),
        .last_line_bytes(pattern_last_line_bytes),.error_flags(pattern_error_flags),
        .line_ok(pattern_line_ok),.line_bad(pattern_line_bad),
        .line_bytes_min(pattern_line_min),.line_bytes_max(pattern_line_max),
        .first_bad_line_y(pattern_first_bad_y),
        .first_bad_line_bytes(pattern_first_bad_bytes),
        .bad0_y(pattern_bad0_y),.bad1_y(pattern_bad1_y),.bad2_y(pattern_bad2_y),
        .bad0_href(pattern_bad0_href),.bad1_href(pattern_bad1_href),.bad2_href(pattern_bad2_href),
        .bad0_accept(pattern_bad0_accept),.bad1_accept(pattern_bad1_accept),.bad2_accept(pattern_bad2_accept),
        .bad0_bytes(pattern_bad0_bytes),.bad1_bytes(pattern_bad1_bytes),.bad2_bytes(pattern_bad2_bytes),
        .bad0_raw_href(pattern_bad0_raw_href),.bad1_raw_href(pattern_bad1_raw_href),.bad2_raw_href(pattern_bad2_raw_href),
        .bad0_zero(pattern_bad0_zero),.bad1_zero(pattern_bad1_zero),.bad2_zero(pattern_bad2_zero),
        .bad0_ff(pattern_bad0_ff),.bad1_ff(pattern_bad1_ff),.bad2_ff(pattern_bad2_ff),
        .frame_start_pulse(pattern_frame_start)
    );
    assign stream_frame_done = INTERNAL_TEST_PATTERN ? pattern_frame_done : raw_frame_done;
    assign stream_frame_good = INTERNAL_TEST_PATTERN ? pattern_frame_good : raw_frame_good;
    assign stream_frame_bad = INTERNAL_TEST_PATTERN ? pattern_frame_bad : raw_frame_bad;
    wire [11:0] stream_measured_lines = INTERNAL_TEST_PATTERN ? pattern_measured_lines : measured_lines;
    wire [12:0] stream_last_line_bytes = INTERNAL_TEST_PATTERN ? pattern_last_line_bytes : last_line_bytes;
    wire [3:0] stream_error_flags = INTERNAL_TEST_PATTERN ? pattern_error_flags : frame_error_flags;
    wire [11:0] stream_line_ok = INTERNAL_TEST_PATTERN ? pattern_line_ok : href_line_ok;
    wire [11:0] stream_line_bad = INTERNAL_TEST_PATTERN ? pattern_line_bad : href_line_bad;
    wire [12:0] stream_line_min = INTERNAL_TEST_PATTERN ? pattern_line_min : href_line_min;
    wire [12:0] stream_line_max = INTERNAL_TEST_PATTERN ? pattern_line_max : href_line_max;
    wire [10:0] stream_first_bad_y = INTERNAL_TEST_PATTERN ? pattern_first_bad_y : href_first_bad_y;
    wire [12:0] stream_first_bad_bytes = INTERNAL_TEST_PATTERN ? pattern_first_bad_bytes : href_first_bad_bytes;
    wire [10:0] stream_bad0_y = INTERNAL_TEST_PATTERN ? pattern_bad0_y : href_bad0_y;
    wire [10:0] stream_bad1_y = INTERNAL_TEST_PATTERN ? pattern_bad1_y : href_bad1_y;
    wire [10:0] stream_bad2_y = INTERNAL_TEST_PATTERN ? pattern_bad2_y : href_bad2_y;
    wire [12:0] stream_bad0_href = INTERNAL_TEST_PATTERN ? pattern_bad0_href : href_bad0_href;
    wire [12:0] stream_bad1_href = INTERNAL_TEST_PATTERN ? pattern_bad1_href : href_bad1_href;
    wire [12:0] stream_bad2_href = INTERNAL_TEST_PATTERN ? pattern_bad2_href : href_bad2_href;
    wire [12:0] stream_bad0_accept = INTERNAL_TEST_PATTERN ? pattern_bad0_accept : href_bad0_accept;
    wire [12:0] stream_bad1_accept = INTERNAL_TEST_PATTERN ? pattern_bad1_accept : href_bad1_accept;
    wire [12:0] stream_bad2_accept = INTERNAL_TEST_PATTERN ? pattern_bad2_accept : href_bad2_accept;
    wire [12:0] stream_bad0_bytes = INTERNAL_TEST_PATTERN ? pattern_bad0_bytes : href_bad0_bytes;
    wire [12:0] stream_bad1_bytes = INTERNAL_TEST_PATTERN ? pattern_bad1_bytes : href_bad1_bytes;
    wire [12:0] stream_bad2_bytes = INTERNAL_TEST_PATTERN ? pattern_bad2_bytes : href_bad2_bytes;
    wire [12:0] stream_bad0_raw_href = INTERNAL_TEST_PATTERN ? pattern_bad0_raw_href : href_bad0_raw_href;
    wire [12:0] stream_bad1_raw_href = INTERNAL_TEST_PATTERN ? pattern_bad1_raw_href : href_bad1_raw_href;
    wire [12:0] stream_bad2_raw_href = INTERNAL_TEST_PATTERN ? pattern_bad2_raw_href : href_bad2_raw_href;
    wire [12:0] stream_bad0_zero = INTERNAL_TEST_PATTERN ? pattern_bad0_zero : href_bad0_zero;
    wire [12:0] stream_bad1_zero = INTERNAL_TEST_PATTERN ? pattern_bad1_zero : href_bad1_zero;
    wire [12:0] stream_bad2_zero = INTERNAL_TEST_PATTERN ? pattern_bad2_zero : href_bad2_zero;
    wire [12:0] stream_bad0_ff = INTERNAL_TEST_PATTERN ? pattern_bad0_ff : href_bad0_ff;
    wire [12:0] stream_bad1_ff = INTERNAL_TEST_PATTERN ? pattern_bad1_ff : href_bad1_ff;
    wire [12:0] stream_bad2_ff = INTERNAL_TEST_PATTERN ? pattern_bad2_ff : href_bad2_ff;
    wire [12:0] stream_first_bad_reject_count = INTERNAL_TEST_PATTERN ? 13'd0 : capture_first_bad_reject_count;
    wire [11:0] stream_first_bad_rej_x0 = INTERNAL_TEST_PATTERN ? 12'd0 : capture_first_bad_rej_x0;
    wire [11:0] stream_first_bad_rej_x1 = INTERNAL_TEST_PATTERN ? 12'd0 : capture_first_bad_rej_x1;
    wire [11:0] stream_first_bad_rej_x2 = INTERNAL_TEST_PATTERN ? 12'd0 : capture_first_bad_rej_x2;
    wire [11:0] stream_first_bad_rej_x3 = INTERNAL_TEST_PATTERN ? 12'd0 : capture_first_bad_rej_x3;
    wire [3:0] stream_first_bad_rej_r0 = INTERNAL_TEST_PATTERN ? 4'd0 : capture_first_bad_rej_r0;
    wire [3:0] stream_first_bad_rej_r1 = INTERNAL_TEST_PATTERN ? 4'd0 : capture_first_bad_rej_r1;
    wire [3:0] stream_first_bad_rej_r2 = INTERNAL_TEST_PATTERN ? 4'd0 : capture_first_bad_rej_r2;
    wire [3:0] stream_first_bad_rej_r3 = INTERNAL_TEST_PATTERN ? 4'd0 : capture_first_bad_rej_r3;
    wire [16:0] stream_gap_pclk = INTERNAL_TEST_PATTERN ? 17'd0 : href_gap_pclk;
    wire stream_frame_start = INTERNAL_TEST_PATTERN ? pattern_frame_start : href_frame_start;
    wire stream_coord_error = INTERNAL_TEST_PATTERN ? 1'b0 : coord_error;
    wire stream_boundary_event = INTERNAL_TEST_PATTERN ? 1'b0 : capture_boundary_event;
    wire [10:0] stream_boundary_line_y = INTERNAL_TEST_PATTERN ? 11'd0 : capture_boundary_line_y;
    wire stream_boundary_raw_href = INTERNAL_TEST_PATTERN ? 1'b0 : capture_boundary_raw_href;
    wire stream_boundary_sync_href = INTERNAL_TEST_PATTERN ? 1'b0 : capture_boundary_sync_href;
    wire stream_boundary_prev_href = INTERNAL_TEST_PATTERN ? 1'b0 : capture_boundary_prev_href;
    wire [15:0] stream_boundary_gap_count = INTERNAL_TEST_PATTERN ? 16'd0 : capture_boundary_gap_count;
    wire [2:0] stream_boundary_source = INTERNAL_TEST_PATTERN ? 3'd0 : capture_boundary_source;
    wire [15:0] camera_rgb565;
    wire color_valid,color_sof,color_eol,color_eof;
    wire [11:0] color_x;
    wire [10:0] color_y;
    wire [23:0] camera_rgb888;
    wire color_line_start;
    wire packer_frame_done,packer_frame_good;
    wire[12:0]joint_r,joint_g,joint_b;wire[7:0]joint_tone;
    wire[15:0]joint_n,joint_h;wire[23:0]joint_v;
    generate
        if (INTERNAL_TEST_PATTERN) begin : g_internal_pattern_display
            // Known RGB565 bars bypass OV5640, RAW8 and Bayer, but still use
            // the live RGB565 packer, async FIFO, DDR writer and HDMI reader.
            assign camera_rgb565 = pattern_pixel;
            assign color_valid = pattern_valid;
            assign color_sof = pattern_sof;
            assign color_eol = pattern_eol;
            assign color_eof = pattern_eof;
            assign color_x = pattern_x;
            assign color_y = pattern_y;
            assign camera_rgb888 = {pattern_pixel[15:11],3'b0,pattern_pixel[10:5],2'b0,pattern_pixel[4:0],3'b0};
            assign color_line_start = pattern_valid && pattern_x==0;
            assign {joint_r,joint_g,joint_b,joint_tone,joint_n,joint_h,joint_v}=0;
        end else if (DEBUG_RAW_GRAY) begin : g_raw_gray_display
            assign camera_rgb565 = {raw_pixel[7:3],raw_pixel[7:2],raw_pixel[7:3]};
            assign camera_rgb888 = {raw_pixel,raw_pixel,raw_pixel};
            assign color_valid = raw_valid;
            assign color_sof = raw_sof;
            assign color_eol = raw_eol;
            assign color_eof = raw_eof;
            assign color_x = raw_x;
            assign color_y = raw_y;
            assign color_line_start = raw_valid && raw_x==0;
            assign {joint_r,joint_g,joint_b,joint_tone,joint_n,joint_h,joint_v}=0;
        end else begin : g_bayer_display
            wire [23:0] linear_rgb;
            wire [15:0] unused_linear565;
            wire linear_valid,linear_sof,linear_eol,linear_eof,unused_linear_start;
            wire [11:0] linear_x;
            wire [10:0] linear_y;
            bayer_bilinear_rgb888 #(.WIDTH(1920),.HEIGHT(1080),.CFA_PHASE(CFA_PHASE)) u_demosaic (
                .pclk(cam_pclk),.rst_n(cam_reset_pipe[1]),
                .raw(raw_pixel),.raw_valid(raw_valid),.raw_sof(raw_sof),
                .raw_eol(raw_eol),.raw_eof(raw_eof),.raw_x(raw_x),.raw_y(raw_y),
                .rgb888(linear_rgb),.rgb565(unused_linear565),.valid(linear_valid),.sof(linear_sof),
                .eol(linear_eol),.eof(linear_eof),.line_start(unused_linear_start),.x(linear_x),.y(linear_y)
            );
            wire [23:0] denoised_rgb;
            wire [11:0] denoised_x;
            wire [10:0] denoised_y;
            wire denoised_valid,denoised_sof,denoised_eol,denoised_eof;
            rgb_edge_denoise u_denoise(.clk(cam_pclk),.rst_n(cam_reset_pipe[1]),
                .in_valid(linear_valid),.in_rgb(linear_rgb),.in_x(linear_x),.in_y(linear_y),
                .in_sof(linear_sof),.in_eol(linear_eol),.in_eof(linear_eof),
                .out_valid(denoised_valid),.out_rgb(denoised_rgb),.out_x(denoised_x),.out_y(denoised_y),
                .out_sof(denoised_sof),.out_eol(denoised_eol),.out_eof(denoised_eof));
            color_restore_pipeline u_color_restore (
                .clk(cam_pclk),.rst_n(cam_reset_pipe[1]),
                .in_valid(denoised_valid),.in_rgb(denoised_rgb),.in_x(denoised_x),.in_y(denoised_y),
                .in_sof(denoised_sof),.in_eol(denoised_eol),.in_eof(denoised_eof),
                .frame_done(packer_frame_done),.frame_good(packer_frame_good),
                .out_valid(color_valid),.out_rgb(camera_rgb888),.out565(camera_rgb565),
                .out_x(color_x),.out_y(color_y),.out_sof(color_sof),.out_eol(color_eol),.out_eof(color_eof),
                .debug_r(joint_r),.debug_g(joint_g),.debug_b(joint_b),.debug_tone(joint_tone),.debug_n(joint_n),.debug_h(joint_h),.debug_v(joint_v)
            );
            assign color_line_start=color_valid && color_x==0;
        end
    endgenerate
    // Join the unchanged capture verdict with delayed RGB EOF. This is the
    // processing adapter, not a change to capture, packer, FIFO or DDR.
    generate if (!DEBUG_RAW_GRAY && !INTERNAL_TEST_PATTERN) begin : g_bayer_frame_end
        raw_image_frame_end u_image_end (
            .clk(cam_pclk),.rst_n(cam_reset_pipe[1]),.in_sof(raw_sof),
            .capture_done(stream_frame_done),.capture_good(stream_frame_good),
            .pixel_eof(color_valid && color_eof),
            .out_done(packer_frame_done),.out_good(packer_frame_good)
        );
    end else begin : g_original_frame_end
        assign packer_frame_done=stream_frame_done;
        assign packer_frame_good=stream_frame_good;
    end endgenerate
    rgb565_word_packer #(.ROW_CHECK_ENABLE(1),.DEBUG_FORCE_COMMIT(DEBUG_FORCE_COMMIT)) u_cam_packer (
        .clk(cam_pclk),.rst_n(cam_reset_pipe[1]),
        .pixel(camera_rgb565),.valid(color_valid),.sof(color_sof),
        .eol(color_eol),.eof(color_eof),.frame_done(packer_frame_done),
        .frame_good(packer_frame_good),.fifo_full(camera_fifo_full),
        .fifo_wr_en(camera_token_valid),.fifo_wr_data(camera_token),
        .overflow(camera_packer_overflow),.row_error(camera_packer_row_error),
        .fifo_full_stall(camera_packer_fifo_full_stall)
    );
    async_word_fifo #(.WIDTH(259),.ADDR_BITS(9),.SPLIT_RESET(1)) u_camera_fifo (
        .wr_clk(cam_pclk),.rd_clk(ddr_core_clk),.rst_n(reset_request_n),
        .wr_rst_n(continuous_cam_reset_n),.rd_rst_n(continuous_core_reset_n),
        .wr_en(continuous_camera_fifo_write),.wr_data(camera_token),
        .wr_full(camera_fifo_full),.wr_used(),
        .wr_overflow(camera_fifo_wr_overflow),
        .rd_en(camera_fifo_read),.rd_data(camera_fifo_data),
        .rd_empty(camera_fifo_empty),.rd_underflow(camera_fifo_rd_underflow),.rd_used(camera_fifo_used_core)
    );
    // Preserve the original sticky LED fault conditions, not UART frame holds.
    reg cam_bad_seen,packer_bad_seen;
    always @(posedge cam_pclk) begin
        if (!cam_reset_pipe[1]) begin cam_bad_seen<=0;packer_bad_seen<=0;end
        else begin
            if (stream_frame_done && stream_frame_bad) cam_bad_seen<=1;
            if (camera_packer_overflow) packer_bad_seen<=1;
        end
    end
    reg [25:0] heartbeat;
    always @(posedge sys_clk)
        if (!sys_reset_pipe[1]) heartbeat <= 26'd0;
        else heartbeat <= heartbeat + 26'd1;
    (* ASYNC_REG = "TRUE" *) reg [1:0] displayed_sys, underflow_sys;
    (* ASYNC_REG = "TRUE" *) reg [1:0] read_error_sys, write_error_sys;
    (* ASYNC_REG = "TRUE" *) reg [1:0] packer_error_sys, camera_bad_sys;
    (* ASYNC_REG = "TRUE" *) reg [1:0] camera_fifo_ovf_sys, camera_fifo_udf_sys;
    (* ASYNC_REG = "TRUE" *) reg [1:0] video_fifo_ovf_sys, video_fifo_udf_sys;
    always @(posedge sys_clk) begin
        displayed_sys <= {displayed_sys[0], display_valid};
        underflow_sys <= {underflow_sys[0], ddr_underflow};
        read_error_sys <= {read_error_sys[0], ddr_read_error};
        write_error_sys <= {write_error_sys[0], ddr_write_error};
        packer_error_sys <= {packer_error_sys[0], packer_bad_seen};
        camera_bad_sys <= {camera_bad_sys[0], cam_bad_seen};
        camera_fifo_ovf_sys <= {camera_fifo_ovf_sys[0],
                                camera_fifo_wr_overflow | camera_packer_fifo_full_stall};
        camera_fifo_udf_sys <= {camera_fifo_udf_sys[0], camera_fifo_rd_underflow};
        video_fifo_ovf_sys <= {video_fifo_ovf_sys[0], video_fifo_wr_overflow};
        video_fifo_udf_sys <= {video_fifo_udf_sys[0], video_fifo_rd_underflow};
    end
    wire fifo_overflow_status = packer_error_sys[1] |
        camera_fifo_ovf_sys[1] | video_fifo_ovf_sys[1];
    wire fifo_underflow_status = underflow_sys[1] |
        camera_fifo_udf_sys[1] | video_fifo_udf_sys[1];
    assign led = {camera_id_error | cam_init_failed | read_error_sys[1] |
                  write_error_sys[1] | packer_error_sys[1],
                  underflow_sys[1], displayed_sys[1], cam_configured,
                  camera_id_ok, hdmi_init_done,
                  ddr_init_done, heartbeat[25]};
    joint_diagnostic u_joint_diagnostic (
        .clk(sys_clk),.rst_n(sys_reset_pipe[1]),.pclk(cam_pclk),.p_rst_n(cam_reset_pipe[1]),
        .serial_available(!gimbal_pending),.serial_busy(diag_serial_busy),
        .enabled(cam_configured),.sensor_info({camera_id,14'd0,cam_init_failed,cam_configured}),
        .bus_busy(sccb_busy),.bus_done(sccb_done),.bus_error(sccb_error),.bus_code(sccb_error_code),.bus_data(sccb_data),
        .bus_start(diag_bus_start),.bus_addr(diag_bus_addr),.tx(diag_uart_tx),
        .raw_valid(raw_valid),.raw_sof(raw_sof),.raw(raw_pixel),.raw_x(raw_x),.raw_y(raw_y),
        .rgb_valid(color_valid),.rgb_sof(color_sof),.rgb_eof(color_eof),.rgb565(camera_rgb565),.rgb_x(color_x),.rgb_y(color_y),
        .frame_done(packer_frame_done),.frame_good(packer_frame_good),
        .gain_r(joint_r),.gain_g(joint_g),.gain_b(joint_b),.tone(joint_tone),.stat_n(joint_n),.stat_h(joint_h),.stat_v(joint_v)
    );
endmodule
