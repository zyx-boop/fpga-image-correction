// Processing-only board integration. Real DDR source lease remains with the
// existing continuous_bank_manager. Frozen processing modules are unchanged.
module board_prefetch_renderer #(parameter PIPELINED_MODE=0)(
 input wire clk,rst_n,start,abort_req,rearm_req,publication_confirmed,source_lease_active,allow_new_ar,
 input wire completed_descriptor_accepted,
 input wire [80:0] source_pose,
 input wire [1:0] source_bank,target_bank,input wire [31:0] source_frame_id,source_epoch,job_seq,
 output wire [27:0] araddr,output wire [3:0] arlen,output wire arvalid,input wire arready,
 input wire [255:0] rdata,input wire rvalid,rlast,
 output wire [27:0] awaddr,output wire [3:0] awlen,output wire awvalid,input wire awready,
 output wire [255:0] wdata,output wire [31:0] wstrb,input wire wready,wlast,
 output wire done,error,rearm_ack,output wire [7:0] error_code,
 output wire reader_quiet,writer_quiet,return_pending,output reg read_inflight,output wire write_inflight,
 output wire [21:0] pixel_count,output wire [17:0] word_ack_count,output wire [27:0] next_write_addr,
 output wire ack_count_complete,
 output wire [5:0] token_count,output reg eof_seen,output reg [63:0] render_cycles,
  output wire [31:0] cache_hits,cache_misses,coord_inputs,coord_outputs,output wire [3:0] stage
);
 wire descriptor_valid,descriptor_ready;wire [1:0] descriptor_bank,held_target_bank;
 wire [31:0] descriptor_fid,descriptor_job,descriptor_epoch,next_job,next_epoch,ctx_fid,ctx_epoch,ctx_job;
 wire [1:0] ctx_bank;wire [31:0] launched_count,reader_input_count,retired_count;
 wire cache_row_mode;
 wire caller_ctx_ready,caller_active,caller_closing,caller_in_ready,context_open,context_active,stop_new_rows;
 wire caller_close,caller_abort,reader_close,reader_abort,window_close,lease_release_req;
 wire [31:0] caller_reserved,caller_results;
 wire map_valid,map_ready,map_sof,map_eol,map_eof,map_oor,map_ioor,map_sat;
 wire [31:0] map_fid,map_job,map_epoch,map_index;
 wire signed [27:0] map_xq,map_yq;wire signed [12:0] map_x,map_y;
 wire reader_ctx_ready,reader_active,reader_closing,reader_aborted,reader_error;wire [31:0] reader_reserved,reader_pending;
 wire mem_valid,mem_ready,mem_rsp_valid;wire [10:0] mem_x,mem_y;wire [31:0] mem_fid,mem_epoch;wire [15:0] mem_rsp_data;
 wire [15:0] mem_fx,mem_fy;
 wire [255:0] tap_rsp_word;
 wire tap_valid,tap_ready,tap_rsp_valid,sampler_busy,sampler_fault;
 wire [10:0] tap_x,tap_y;wire [31:0] tap_fid,tap_epoch;wire [15:0] tap_rsp_data;
 wire adapter_quiet,adapter_fault;wire [7:0] adapter_code;wire [31:0] ram_refs,ram_requests,ram_responses;
 wire ram_rd_valid,ram_rd_ready,ram_rsp_valid,ram_rsp_hit,ram_error;wire [10:0] ram_rd_x,ram_rd_y,ram_rsp_x,ram_rsp_y;
 wire [31:0] ram_rd_epoch,ram_rsp_epoch;wire [15:0] ram_rsp_data;
 wire window_launch,window_ready,window_idle,window_rows_done,window_stopped,window_fault;
 wire [10:0] window_lo,window_hi,output_y,row_req_y,row_done_y,lookup_tag_y;wire [31:0] row_done_epoch,lookup_epoch;
 wire row_req_valid,row_req_ready,row_done_valid,row_done_ready,lookup_ready;
 wire pf_busy,pf_pending,pf_fault,pf_timeout;wire [7:0] pf_code;
 wire wr_begin,wr_begin_ready,wr_valid,wr_ready;wire [10:0] wr_y;wire [31:0] wr_epoch;wire [6:0] wr_word;wire [255:0] wr_data;
 wire controller_fault;wire [7:0] controller_code;
 wire [31:0] ar_count,r_count,burst_count,word_count,rows_count;
 wire out_valid,out_ready,out_sof,out_eol,out_eof,out_oor;wire [15:0] out_pixel;
 wire [31:0] out_fid,out_job,out_epoch,out_index;
 wire core_job_done,core_job_done_ready,core_aborted,binding_fault;wire [7:0] binding_code;
 wire w_done,w_error,w_pixel_ready,writer_reset_n;wire [7:0] w_code;wire [21:0] w_pixels;
 wire caller_quiet=!caller_active&&caller_reserved==0&&caller_results==0&&!map_valid;
 wire pipeline_quiet=caller_reserved==0&&caller_results==0&&!map_valid&&reader_reserved==0&&reader_pending==0&&!sampler_busy&&adapter_quiet;
 wire prefetch_quiet=!pf_busy&&!pf_pending;
 wire processing_quiet=pipeline_quiet&&prefetch_quiet&&window_idle&&!arvalid&&!rvalid;
 wire launch_valid=window_launch&&!stop_new_rows;
 wire launch_take=launch_valid&&caller_in_ready;
 wire map_take=map_valid&&map_ready;
 wire output_take=out_valid&&out_ready;
 // D58: one complete comparison per identity field, bounded byte boundaries.
 // The original same-cycle error -> global abort -> stop chain is unchanged.
 (* syn_keep=1 *) wire [3:0] output_fid_eq8;
 (* syn_keep=1 *) wire [3:0] output_job_eq8;
 (* syn_keep=1 *) wire [3:0] output_epoch_eq8;
 genvar output_identity_byte;
 generate for(output_identity_byte=0;output_identity_byte<4;output_identity_byte=output_identity_byte+1)begin:g_output_identity
  assign output_fid_eq8[output_identity_byte]=out_fid[output_identity_byte*8+:8]==ctx_fid[output_identity_byte*8+:8];
  assign output_job_eq8[output_identity_byte]=out_job[output_identity_byte*8+:8]==ctx_job[output_identity_byte*8+:8];
  assign output_epoch_eq8[output_identity_byte]=out_epoch[output_identity_byte*8+:8]==ctx_epoch[output_identity_byte*8+:8];
 end endgenerate
 wire identity_bad=out_valid&&(!(&output_fid_eq8)||!(&output_job_eq8)||!(&output_epoch_eq8));
 wire geometry_bad=map_valid&&(map_ioor||map_sat);
 wire external_fault=sampler_fault||pf_fault||adapter_fault||window_fault||ram_error||reader_error||identity_bad||geometry_bad||binding_fault||w_error;
 assign error=controller_fault||external_fault;
 assign error_code=sampler_fault?8'hb1:pf_fault?pf_code:binding_fault?binding_code:w_error?w_code:adapter_fault?{4'ha,adapter_code[3:0]}:controller_code;
 wire modules_open_ready=caller_ctx_ready&&reader_ctx_ready&&adapter_quiet&&prefetch_quiet&&window_idle;
 wire discard_output=abort_req||core_aborted||error;
 assign out_ready=discard_output||w_pixel_ready;
 assign reader_quiet=processing_quiet;
 assign return_pending=sampler_busy||pf_pending||ram_refs!=0||ram_rsp_valid||mem_rsp_valid||row_done_valid;
 // Legacy Bank manager consumes only zero/nonzero outstanding indication.
 assign token_count=(caller_reserved!=0||caller_results!=0||map_valid||reader_reserved!=0||reader_pending!=0||ram_refs!=0)?6'd1:6'd0;
 assign pixel_count=retired_count[21:0];assign coord_inputs=launched_count;assign coord_outputs=reader_input_count;

 assign stage=done?4'd6:context_active?4'd1:4'd0;
 board_render_binding #(.PIPELINED_MODE(PIPELINED_MODE),.USE_ACK_CERT(1)) binding(.clk(clk),.rst_n(rst_n),.start(start),.abort_req(abort_req),.rearm_req(rearm_req),
  .completed_descriptor_accepted(completed_descriptor_accepted),
  .publication_confirmed(publication_confirmed),.source_lease_active(source_lease_active),.source_bank(source_bank),.target_bank(target_bank),
  .source_fid(source_frame_id),.source_epoch(source_epoch),.job_seq(job_seq),.next_job(next_job),.next_epoch(next_epoch),
  .modules_open_ready(modules_open_ready),.context_open(context_open),.core_release_req(lease_release_req),.core_job_done(core_job_done),
  .core_aborted(core_aborted),.core_quiet(processing_quiet),.writer_done(w_done),.writer_quiet(writer_quiet),.writer_words(word_ack_count),.writer_ack_count_complete(ack_count_complete),
  .output_pixels(pixel_count),.next_write_addr(next_write_addr),.eof_seen(eof_seen),.descriptor_valid(descriptor_valid),.descriptor_bank(descriptor_bank),
  .held_target_bank(held_target_bank),.descriptor_fid(descriptor_fid),.descriptor_job(descriptor_job),.descriptor_epoch(descriptor_epoch),
  .writer_reset_n(writer_reset_n),.job_done_ready(core_job_done_ready),.done(done),.rearm_ack(rearm_ack),.error(binding_fault),.error_code(binding_code));
 prefetch_frame_control controller(.clk(clk),.rst(!rst_n),.run_enable(1'b1),.app_abort(abort_req),.external_fault(external_fault),
  .descriptor_valid(descriptor_valid),.descriptor_ready(descriptor_ready),.descriptor_bank(descriptor_bank),.descriptor_fid(descriptor_fid),.descriptor_job(descriptor_job),.descriptor_epoch(descriptor_epoch),
  .source_request(),.next_job(next_job),.next_epoch(next_epoch),.modules_open_ready(modules_open_ready),
  .caller_quiet(caller_quiet),.reader_closed(reader_ctx_ready),.adapter_quiet(adapter_quiet),.prefetch_quiet(prefetch_quiet),.window_idle(window_idle),.window_rows_done(window_rows_done),.lease_active(source_lease_active),
  .launch_take(launch_take),.reader_take(map_take),.output_take(output_take),.output_eof(out_eof),
  .context_open(context_open),.context_active(context_active),.stop_new_rows(stop_new_rows),.caller_close(caller_close),.caller_abort(caller_abort),
  .reader_close(reader_close),.reader_abort(reader_abort),.window_close(window_close),.lease_release_req(lease_release_req),
  .job_done_valid(core_job_done),.job_done_ready(core_job_done_ready),.aborted(core_aborted),.fault(controller_fault),.error_code(controller_code),
  .ctx_bank(ctx_bank),.ctx_fid(ctx_fid),.ctx_job(ctx_job),.ctx_epoch(ctx_epoch),.launched_count(launched_count),.reader_input_count(reader_input_count),.retired_count(retired_count));
 streaming_f24_caller caller(.clk(clk),.rst(!rst_n),.ctx_valid(context_open),.ctx_ready(caller_ctx_ready),
  .ctx_pose(source_pose),.cache_row_mode(cache_row_mode),.ctx_source_frame_id(descriptor_fid),.ctx_job(descriptor_job),.ctx_epoch(descriptor_epoch),.ctx_first_index(32'd0),
  .in_valid(launch_valid),.in_ready(caller_in_ready),.close_req(caller_close),.abort_req(caller_abort),.active(caller_active),.closing(caller_closing),
  .out_valid(map_valid),.out_ready(map_ready),.out_source_frame_id(map_fid),.out_job(map_job),.out_epoch(map_epoch),.out_pixel_index(map_index),
  .out_sof(map_sof),.out_eol(map_eol),.out_eof(map_eof),.src_x_q(map_xq),.src_y_q(map_yq),.src_x_nn(map_x),.src_y_nn(map_y),
  .source_oob(map_oor),.input_oob(map_ioor),.saturation_seen(map_sat),.reserved_count(caller_reserved),.result_count(caller_results));
 nn_stream_reader #(.BILINEAR(1)) reader(.clk(clk),.rst(!rst_n),.ctx_valid(context_open),.ctx_ready(reader_ctx_ready),.ctx_fid(descriptor_fid),.ctx_job(descriptor_job),.ctx_epoch(descriptor_epoch),.ctx_first_index(32'd0),
  .active(reader_active),.closing(reader_closing),.aborted(reader_aborted),.error(reader_error),.reserved_count(reader_reserved),.pending_count(reader_pending),
  .close_req(reader_close),.abort_req(reader_abort),.in_valid(map_valid),.in_ready(map_ready),.in_x(map_x),.in_y(map_y),.in_x_q(map_xq),.in_y_q(map_yq),.mem_fx(mem_fx),.mem_fy(mem_fy),
  .in_fid(map_fid),.in_job(map_job),.in_epoch(map_epoch),.in_index(map_index),.in_sof(map_sof),.in_eol(map_eol),.in_eof(map_eof),.in_oor(map_oor),
  .mem_valid(mem_valid),.mem_ready(mem_ready),.mem_x(mem_x),.mem_y(mem_y),.mem_fid(mem_fid),.mem_epoch(mem_epoch),.mem_rsp_valid(mem_rsp_valid),.mem_rsp_data(mem_rsp_data),
  .out_valid(out_valid),.out_ready(out_ready),.out_fid(out_fid),.out_job(out_job),.out_epoch(out_epoch),.out_index(out_index),.out_sof(out_sof),.out_eol(out_eol),.out_eof(out_eof),.out_oor(out_oor),.out_pixel(out_pixel));
 // Dynamic request scheduling has no static source-row window.
 reg dynamic_open;
 wire cache_initialized,cache_quiet,cache_fault;
 always @(posedge clk)begin
  if(!rst_n)dynamic_open<=0;
  else if(context_open)dynamic_open<=1;
  else if(window_close)dynamic_open<=0;
 end
 assign window_idle=!dynamic_open;
 assign window_launch=dynamic_open&&cache_initialized&&(launched_count<2073600);
 assign window_rows_done=dynamic_open&&(retired_count==2073600)&&pipeline_quiet&&prefetch_quiet;
 assign window_fault=0;
 assign pf_busy=0;assign pf_pending=0;assign pf_fault=0;assign pf_code=0;
 assign row_done_valid=0;assign ram_rsp_valid=0;assign ram_error=0;
 assign adapter_quiet=cache_quiet;assign adapter_fault=cache_fault;assign adapter_code=8'hd1;
 assign ram_refs=cache_quiet?0:1;
 dynamic_burst_cache #(.ROW_REUSE(1)) cache(.clk(clk),.rst(!rst_n),.frame_open(context_open),.frame_close(window_close),.prefer_rows(cache_row_mode),
  .cancel(discard_output),.ctx_active(context_active),.allow_new_ar(allow_new_ar&&!error),
  .ctx_bank(ctx_bank),.ctx_fid(ctx_fid),.ctx_epoch(ctx_epoch),
  .req_valid(tap_valid),.req_ready(tap_ready),.req_x(tap_x),.req_y(tap_y),.req_fid(tap_fid),.req_epoch(tap_epoch),
  .rsp_valid(tap_rsp_valid),.rsp_data(tap_rsp_data),.rsp_word(tap_rsp_word),.quiet(cache_quiet),.initialized(cache_initialized),.fault(cache_fault),
  .hits(cache_hits),.misses(cache_misses),.arvalid(arvalid),.arready(arready),.araddr(araddr),.arlen(arlen),
  .rvalid(rvalid),.rlast(rlast),.rdata(rdata));
 bilinear_rgb565_sampler #(.WORD_CACHE(1)) sampler(.clk(clk),.rst(!rst_n),
  .in_valid(mem_valid),.in_ready(mem_ready),.in_x(mem_x),.in_y(mem_y),.in_fx(mem_fx),.in_fy(mem_fy),.in_fid(mem_fid),.in_epoch(mem_epoch),
  .tap_valid(tap_valid),.tap_ready(tap_ready),.tap_x(tap_x),.tap_y(tap_y),.tap_fid(tap_fid),.tap_epoch(tap_epoch),
  .tap_rsp_valid(tap_rsp_valid),.tap_rsp_data(tap_rsp_data),.tap_rsp_word(tap_rsp_word),.rsp_valid(mem_rsp_valid),.rsp_data(mem_rsp_data),.busy(sampler_busy),.fault(sampler_fault));
 corrected_frame_writer #(.H_MIRROR(1)) target_writer(.clk(clk),.rst_n(writer_reset_n),.start(context_open),.target_bank(held_target_bank),.abort_req(discard_output),
  .pixel_valid(out_valid&&!discard_output),.pixel_ready(w_pixel_ready),.pixel(out_pixel),.pixel_eof(out_eof),
  .awaddr(awaddr),.awlen(awlen),.awvalid(awvalid),.awready(awready),.wdata(wdata),.wstrb(wstrb),.wready(wready),.wlast(wlast),
  .quiet(writer_quiet),.write_inflight(write_inflight),.done(w_done),.error(w_error),.error_code(w_code),
  .pixel_count(w_pixels),.word_ack_count(word_ack_count),.next_addr(next_write_addr),.ack_count_complete(ack_count_complete));
 always @(posedge clk)begin
  if(!rst_n)begin eof_seen<=0;render_cycles<=0;read_inflight<=0;end
  else begin
   if(start)begin eof_seen<=0;render_cycles<=0;end
   else if(context_active&&!done)render_cycles<=render_cycles+1'b1;
   if(output_take&&out_eof)eof_seen<=1;
   if(arvalid&&arready)read_inflight<=1;
   if(rvalid&&rlast)read_inflight<=0;
  end
 end
endmodule
