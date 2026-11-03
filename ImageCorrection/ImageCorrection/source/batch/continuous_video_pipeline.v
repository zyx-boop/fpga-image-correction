// Connection-only integration shared by top and the T4 joint scoreboard.
// Source3 mapping and transaction arbitration; frozen F24/Reader/Writer math unchanged.
module continuous_video_pipeline #(
 parameter OUTPUT_WIDTH=1920,OUTPUT_HEIGHT=1080,SNAPSHOT_CYCLES=125000000,
 parameter WATCHDOG_CYCLES=125000000
)(
 input wire core_clk,pixel_clk,cam_clk,sys_clk,rst_n,init_done,key_pulse,vblank,de,frame_start,
 input wire [80:0] imu_pose_sys,
 input wire capture_sof,capture_done,capture_good,token_wr_en,input wire [258:0] token_wr_data,
 input wire camera_fifo_empty,input wire [258:0] camera_fifo_data,output wire camera_fifo_read,
 input wire [9:0] camera_fifo_used_core,
 output wire arvalid,input wire arready,output wire [27:0] araddr,output wire [3:0] arlen,
 input wire rvalid,rlast,input wire [255:0] rdata,
 output wire awvalid,input wire awready,output wire [27:0] awaddr,output wire [3:0] awlen,
 output wire [255:0] wdata,output wire [31:0] wstrb,input wire wready,wlast,
 output wire [15:0] rgb565,output wire pixel_valid,error,output wire [7:0] error_code,
 output wire [31:0] job_seq,source_frame_id,publish_count,input_frame,drop_total,
 output wire display_valid,output wire [1:0] display_bank,target_bank,
 output wire [4:0] bank_stage,output wire display_lease,
 output wire [31:0] display_job,display_fid,display_context_epoch,
 output wire [63:0] render_cycles,source_sof_cycle,
 output wire [63:0] observed_latency,output wire [31:0] observed_count,output wire observed_error,
 output wire [31:0] input_good,drop_bad,drop_busy,drop_write,
 output wire [31:0] source_wait_max,display_wait_max,
 output wire [15:0] writer_addr_error_count,output wire [3:0] writer_addr_error_code,
 output wire [27:0] writer_addr_error_actual,writer_addr_error_expected,
 output wire writer_addr_error,ddr_write_error,ddr_read_error,underflow,
 output wire reader_addr_error,output wire [15:0] reader_addr_error_count,
 output wire [27:0] reader_addr_actual,reader_addr_expected,output wire [3:0] reader_addr_code,
 output wire video_fifo_wr_overflow,video_fifo_rd_underflow,
 output wire fifo_ready,camera_writer_active,camera_frame_start,camera_frame_finish,camera_frame_good,
 output wire core_reset_n,cam_reset_n,pixel_reset_n,sys_reset_n,
  output wire transport_ready_core,transport_ready_cam,camera_stream_active,camera_fifo_write
);
 localparam PIXELS=OUTPUT_WIDTH*OUTPUT_HEIGHT,WORDS=PIXELS/16;
 wire admitted_sof,boot_fault;
 wire [80:0] camera_pose,metadata_pose,source_pose;
 pose_mailbox pose_cdc(.source_clk(sys_clk),.dest_clk(cam_clk),.reset_n(rst_n),
  .source_data(imu_pose_sys),.dest_data(camera_pose));
 continuous_reset_domains resets(.core_clk(core_clk),.cam_clk(cam_clk),.pixel_clk(pixel_clk),.sys_clk(sys_clk),
 .reset_request_n(rst_n),.init_done(init_done),.capture_sof(capture_sof),.token_valid(token_wr_en),
 .token_eof(token_wr_data[258:257]==1||token_wr_data[258:257]==2),
 .core_rst_n(core_reset_n),.cam_rst_n(cam_reset_n),.pixel_rst_n(pixel_reset_n),.sys_rst_n(sys_reset_n),
 .ready_core(transport_ready_core),.ready_cam(transport_ready_cam),.stream_active(camera_stream_active),
 .accept_sof(admitted_sof),.accept_token(camera_fifo_write),.boot_fault(boot_fault));
 wire camera_token_consume; wire [258:0] camera_consumed_token;
 wire writer_active,writer_start,writer_finish,writer_good,camera_quiet,camera_pending,camera_error;
 wire [1:0] writer_bank,source_bank;wire [7:0] camera_code,bank_code,identity_code,session_code,arb_code,render_code,display_code;
 wire bank_error,identity_error,session_error,arb_error,render_error,display_error;
 wire write_arb_error,completed_descriptor_accepted;wire [7:0] write_arb_code;
 wire c_awready,c_wready,c_wlast,t_awready,t_wready,t_wlast;
 wire [63:0] completed_display_sof;
 wire render_start,render_owner,source_locked,target_locked,source_lease_active,allow_ar,rearm_req,rearm_ack;
 wire [31:0] source_epoch;wire [63:0] meta_time;wire [31:0] meta_fid;wire meta_valid;
 wire [17:0] camera_words;wire [27:0] camera_actual,camera_expected;wire camera_eof;
 wire [27:0] ca,sa,da,next_addr;wire [3:0] cl,sl,dl;wire cav,sav,dav,sready,dready;
 wire [255:0] cw,sw,sr,dr;wire [31:0] cs,ss;wire srv,srl,drv,drl;
 wire rav;wire [27:0] ra;wire [3:0] ralen;
 wire done,rquiet,wquiet,return_pending,rinflight,winflight,eof_seen;
 wire [21:0] pixels,consumer_pixels;wire [17:0] words;wire [5:0] tokens;wire ack_count_complete;
 wire [31:0] hits,misses,ci,co,defer_count;wire [3:0] render_stage;
 wire engine_reset,engine_start,session_ready;wire [1:0] engine_source,engine_target;wire [15:0] engine_fid,engine_epoch;
 wire display_quiet,consumer_quiet,launch_enable;wire [15:0] consumer_epoch,scan_epoch;
 wire [1:0] reader_bank;wire [17:0] display_words;
 wire fifo_full,fifo_empty,fifo_write,fifo_read,consumer_ready;wire [9:0] fifo_used;
 wire [255:0] fifo_write_data,fifo_read_data;
 wire metrics_error;wire [31:0] displayed_count,first_job,first_fid,input_fps,output_fps,display_fps;
 assign observed_latency=latency_cycles;
 assign observed_count=displayed_count;
 assign observed_error=error;
 wire [63:0] latency_cycles;reg [63:0] display_stamp;reg [31:0] stamp_job;
 // Sticky Pixel-domain faults are status CDC, not synchronous Core controls.
 // Keep raw diagnostics at their output ports; only stage two may abort the
 // Core session/Bank manager. Never use these flops to reset or release leases.
 (* ASYNC_REG="TRUE" *) reg [1:0] pixel_fault_meta,pixel_fault_core;
 always @(posedge core_clk or negedge core_reset_n)begin
  if(!core_reset_n)begin pixel_fault_meta<=0;pixel_fault_core<=0;end
  else begin pixel_fault_meta<={video_fifo_rd_underflow,underflow};pixel_fault_core<=pixel_fault_meta;end
 end
 assign error=write_arb_error||bank_error||identity_error||session_error||arb_error||camera_error||render_error||display_error||pixel_fault_core[0]||video_fifo_wr_overflow||pixel_fault_core[1]||metrics_error||boot_fault;
 assign error_code=identity_error?identity_code:write_arb_error?write_arb_code:arb_error?arb_code:session_error?session_code:
  camera_error?camera_code:render_error?render_code:display_error?display_code:bank_error?bank_code:pixel_fault_core[0]?8'h41:metrics_error?8'hb1:boot_fault?8'hb2:8'd0;
 assign drop_total=drop_bad+drop_busy+drop_write;
 assign ddr_read_error=display_error;
 assign fifo_ready=consumer_ready;
 assign camera_writer_active=writer_active;assign camera_frame_start=writer_start;
 assign camera_frame_finish=writer_finish;assign camera_frame_good=writer_good;
 always @(posedge core_clk)begin
  if(!core_reset_n)begin display_stamp<=0;stamp_job<=0;end
  else if(display_job!=stamp_job)begin display_stamp<=completed_display_sof;stamp_job<=display_job;end
 end
 continuous_metrics #(.WINDOW_CYCLES(SNAPSHOT_CYCLES)) metrics(.core_clk(core_clk),.pixel_clk(pixel_clk),.rst_n(core_reset_n),
 .pixel_rst_n(pixel_reset_n),.startup_ready(transport_ready_core),
 .frame_start_pix(frame_start),.pixel_valid_pix(pixel_valid),.scan_epoch(scan_epoch),.display_job(display_job),.display_fid(display_fid),
 .display_sof_cycle(display_stamp),.input_frame(input_frame),.publish_count(publish_count),.displayed_count(displayed_count),
 .first_display_job(first_job),.first_display_fid(first_fid),.latency_cycles(latency_cycles),.input_fps(input_fps),.output_fps(output_fps),.display_fps(display_fps),.error(metrics_error));
 camera_frame_identity #(.SNAPSHOT_CYCLES(SNAPSHOT_CYCLES)) identity(
 .cam_clk(cam_clk),.core_clk(core_clk),.rst_n(core_reset_n),.cam_rst_n(cam_reset_n),.startup_ready(transport_ready_core),
 .camera_pose(camera_pose),.frame_pose(metadata_pose),.capture_sof(admitted_sof),.capture_done(capture_done&&camera_stream_active),.capture_good(capture_good),
 .token_wr_en(camera_fifo_write),.token_wr_data(token_wr_data),.token_rd_en(camera_token_consume),.token_rd_empty(1'b0),.token_rd_data(camera_consumed_token),
 .writer_frame_start(writer_start),.writer_frame_finish(writer_finish),.writer_frame_good(writer_good),.writer_active(writer_active),
 .sof_metadata_valid(meta_valid),.frame_id(meta_fid),.sof_cycle(meta_time),.input_frame_snapshot(input_frame),.input_good_snapshot(input_good),
 .drop_capture_bad(drop_bad),.drop_schedule(drop_busy),.drop_write_bad(drop_write),.error(identity_error),.error_code(identity_code));
 continuous_bank_manager #(.FRAME_PIXELS(PIXELS),.FRAME_WORDS(WORDS),.LEASE_WATCHDOG_CYCLES(WATCHDOG_CYCLES),.USE_ACK_CERT(1)) manager(
 .clk(core_clk),.rst_n(core_reset_n),.init_done(init_done&&transport_ready_core),.key_pulse(key_pulse),.vblank(vblank),
 .completed_descriptor_accepted(completed_descriptor_accepted),.display_sof_cycle(completed_display_sof),
 .camera_frame_start(writer_start),.camera_frame_finish(writer_finish),.camera_frame_good(writer_good),
 .camera_end_words(camera_words),.camera_end_actual(camera_actual),.camera_end_expected(camera_expected),.camera_end_good(camera_eof),
 .camera_write_quiet(camera_quiet),.camera_metadata_valid(meta_valid),.camera_pose(metadata_pose),.source_pose(source_pose),.camera_frame_id(meta_fid),.camera_sof_cycle(meta_time),
 .camera_writer_active(writer_active),.camera_writer_bank(writer_bank),.continuous_run(),.render_start(render_start),.render_bus_owner(render_owner),
 .source_bank(source_bank),.target_bank(target_bank),.source_frame_id(source_frame_id),.job_seq(job_seq),.source_epoch(source_epoch),.source_sof_cycle(source_sof_cycle),
 .source_locked(source_locked),.target_locked(target_locked),.source_lease_active(source_lease_active),.allow_new_ar(allow_ar),
 .render_done(done),.render_error(error),.render_pixels(pixels),.render_word_acks(words),.render_ack_count_complete(ack_count_complete),.render_next_addr(next_addr),.render_tokens(tokens),
 .render_eof_seen(eof_seen),.render_reader_quiet(rquiet),.render_writer_quiet(wquiet),.render_arvalid(sav),.render_arready(sready),
 .render_araddr(sa),.render_arlen(sl),.render_rvalid(srv),.render_rlast(srl),.render_return_pending(return_pending),.render_awvalid(rav),.render_write_inflight(winflight),
 .display_epoch_quiet(display_quiet),.display_lease_active(display_lease),.display_valid(display_valid),.display_bank(display_bank),.display_launch_enable(launch_enable),
 .display_frame_id(display_fid),.display_job_seq(display_job),.display_context_epoch(display_context_epoch),
 .rearm_req(rearm_req),.rearm_ack(rearm_ack),.bank_error(bank_error),.error_code(bank_code),.stage(bank_stage),.publish_count(publish_count),.capture_bad_count(),.capture_denied_count());
 // Board binding replaces only the old one-shot session/renderer. Source Owner,
 // camera/target write ownership, display drain and DDR arbiters remain intact.
 assign session_error=1'b0;assign session_code=8'd0;
 board_prefetch_renderer #(.PIPELINED_MODE(1)) render(
 .clk(core_clk),.rst_n(core_reset_n),.start(render_start),.abort_req(error),.source_bank(source_bank),.target_bank(target_bank),
 .source_pose(source_pose),.source_frame_id(source_frame_id),.source_epoch(source_epoch),.job_seq(job_seq),
 .rearm_req(rearm_req),.rearm_ack(rearm_ack),.publication_confirmed(display_valid&&display_job==job_seq),
 .completed_descriptor_accepted(completed_descriptor_accepted),
 .source_lease_active(source_lease_active),.allow_new_ar(allow_ar),.araddr(sa),.arlen(sl),.arvalid(sav),.arready(sready),
 .rdata(sr),.rvalid(srv),.rlast(srl),.awaddr(ra),.awlen(ralen),.awvalid(rav),.awready(t_awready),
 .wdata(sw),.wstrb(ss),.wready(t_wready),.wlast(t_wlast),.done(done),.error(render_error),.error_code(render_code),
 .reader_quiet(rquiet),.writer_quiet(wquiet),.return_pending(return_pending),.read_inflight(rinflight),.write_inflight(winflight),
 .pixel_count(pixels),.word_ack_count(words),.next_write_addr(next_addr),.ack_count_complete(ack_count_complete),.token_count(tokens),.eof_seen(eof_seen),
 .render_cycles(render_cycles),.cache_hits(hits),.cache_misses(misses),.coord_inputs(ci),.coord_outputs(co),.stage(render_stage));
 camera_write_guard #(.BURST_MODE(1),.WATCHDOG_CYCLES(WATCHDOG_CYCLES)) camera_guard(
 .clk(core_clk),.rst_n(core_reset_n),.render_owner(1'b0),.writer_active(writer_active),.awvalid(cav),.awready(c_awready),
 .awlen(cl),.wready(c_wready),.write_ack(c_wlast),.pending(camera_pending),.quiet(camera_quiet),.error(camera_error),.error_code(camera_code));
 // Metadata follows consumed tokens, not speculative FIFO reads.
 // Fatal fault stops new token consumption; offered native transactions still drain.
 camera_burst_writer #(.DUAL_SOURCE(1)) camera_writer(.clk(core_clk),.rst_n(core_reset_n),.init_done(init_done&&transport_ready_core&&!error),
 .fifo_empty(camera_fifo_empty),.fifo_rd_data(camera_fifo_data),.fifo_rd_en(camera_fifo_read),
 .token_consume_en(camera_token_consume),.token_consume_data(camera_consumed_token),.writer_active(writer_active),.writer_bank(writer_bank),
 .frame_start(writer_start),.frame_finish(writer_finish),.frame_good(writer_good),.awaddr(ca),.awlen(cl),.awvalid(cav),.awready(c_awready),
 .wdata(cw),.wstrb(cs),.wready(c_wready),.wlast(c_wlast),.error(ddr_write_error),
 .addr_error(writer_addr_error),.addr_error_count(writer_addr_error_count),.addr_error_actual(writer_addr_error_actual),.addr_error_expected(writer_addr_error_expected),
 .addr_error_code(writer_addr_error_code),.frame_end_pulse(),.frame_end_word_count(camera_words),.frame_end_addr_actual(camera_actual),
 .frame_end_addr_expected(camera_expected),.frame_end_token_good(camera_eof));
 // Native transaction ownership replaces whole-frame exclusive ownership.
 ddr_write_arbiter write_arbiter(.clk(core_clk),.rst_n(core_reset_n),.abort_req(error),.camera_fifo_used(camera_fifo_used_core),
 .c_active(writer_active),.c_bank(writer_bank),.c_awvalid(cav),.c_awready(c_awready),.c_awaddr(ca),.c_awlen(cl),.c_wdata(cw),.c_wstrb(cs),.c_wready(c_wready),.c_wlast(c_wlast),
 .t_active(target_locked),.t_bank(target_bank),.t_awvalid(rav),.t_awready(t_awready),.t_awaddr(ra),.t_awlen(ralen),.t_wdata(sw),.t_wstrb(ss),.t_wready(t_wready),.t_wlast(t_wlast),
 .awvalid(awvalid),.awready(awready),.awaddr(awaddr),.awlen(awlen),.wdata(wdata),.wstrb(wstrb),.wready(wready),.wlast(wlast),.quiet(),.error(write_arb_error),.error_code(write_arb_code));
 ddr_read_arbiter #(.DUAL_SOURCE(1)) arbiter(.clk(core_clk),.rst_n(core_reset_n),.source_lease_active(source_lease_active),.source_epoch(source_epoch),.source_bank(source_bank),
 .s_arvalid(sav),.s_arready(sready),.s_araddr(sa),.s_arlen(sl),.s_rdata(sr),.s_rvalid(srv),.s_rlast(srl),
 .display_lease_active(display_lease),.display_bank(reader_bank),.display_epoch(scan_epoch),.fifo_used(fifo_used),
 .d_arvalid(dav),.d_arready(dready),.d_araddr(da),.d_arlen(dl),.d_rdata(dr),.d_rvalid(drv),.d_rlast(drl),
 .arvalid(arvalid),.arready(arready),.araddr(araddr),.arlen(arlen),.rdata(rdata),.rvalid(rvalid),.rlast(rlast),
 .error(arb_error),.error_code(arb_code),.quiet(),.source_wait_max(source_wait_max),.display_wait_max(display_wait_max),.completed_bursts());
 hdmi_frame_read_guard #(.FRAME_WIDTH(OUTPUT_WIDTH),.FRAME_HEIGHT(OUTPUT_HEIGHT),.BANK_ADDR_UNITS(28'hfd200),.USE_LAUNCH_ENABLE(1),.WATCHDOG_CYCLES(WATCHDOG_CYCLES)) display_guard(
 .clk(core_clk),.pixel_clk(pixel_clk),.rst_n(core_reset_n),.vblank(vblank),.display_valid(display_valid),.launch_enable(launch_enable),.requested_bank(display_bank),
 .fifo_used(fifo_used),.fifo_full(fifo_full),.consumer_quiet_pix(consumer_quiet),.consumer_error_pix(underflow),.consumer_pixels_pix(consumer_pixels),.consumer_epoch_pix(consumer_epoch),
 .araddr(da),.arlen(dl),.arvalid(dav),.arready(dready),.rdata(dr),.rvalid(drv),.rlast(drl),.fifo_wr_en(fifo_write),.fifo_wr_data(fifo_write_data),
 .epoch_quiet(display_quiet),.error(display_error),.error_code(display_code),.lease_active(display_lease),.display_epoch(scan_epoch),.reader_bank(reader_bank),
 .words_received(display_words),.defer_count(defer_count),.reader_addr_error(reader_addr_error),.reader_addr_error_count(reader_addr_error_count),
 .reader_addr_actual(reader_addr_actual),.reader_addr_expected(reader_addr_expected),.reader_addr_code(reader_addr_code));
 async_word_fifo #(.WIDTH(256),.ADDR_BITS(9),.SPLIT_RESET(1)) video_fifo(.wr_clk(core_clk),.rd_clk(pixel_clk),.rst_n(rst_n),
 .wr_rst_n(core_reset_n),.rd_rst_n(pixel_reset_n),
 .wr_en(fifo_write),.wr_data(fifo_write_data),.wr_full(fifo_full),.wr_used(fifo_used),.wr_overflow(video_fifo_wr_overflow),
 .rd_en(fifo_read),.rd_data(fifo_read_data),.rd_empty(fifo_empty),.rd_underflow(video_fifo_rd_underflow));
 (* ASYNC_REG="TRUE" *) reg [1:0] display_active_pix;
 always @(posedge pixel_clk or negedge pixel_reset_n)begin
  if(!pixel_reset_n)display_active_pix<=0;else display_active_pix<={display_active_pix[0],display_lease};
 end
 ddr_word_to_pixel consumer(.clk(pixel_clk),.rst_n(pixel_reset_n),.de(de&&display_active_pix[1]),.frame_start(frame_start),
 .fifo_empty(fifo_empty),.fifo_rd_data(fifo_read_data),.fifo_rd_en(fifo_read),.ready(consumer_ready),.rgb565(rgb565),
 .pixel_valid(pixel_valid),.underflow(underflow),.consumer_quiet(consumer_quiet),.consumed_pixels(consumer_pixels),.consumed_epoch(consumer_epoch));
endmodule
