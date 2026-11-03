// D144: two immutable source banks (0/3), two completed target banks (1/2).
// Capture, Render and display own independent roles; only real certificates
// release them. Complete descriptors separate render rearm from HDMI vblank.
module continuous_bank_manager #(
 parameter FRAME_PIXELS=2073600,FRAME_WORDS=129600,FRAME_ADDR_UNITS=28'hfd200,
 parameter LEASE_WATCHDOG_CYCLES=125000000,USE_ACK_CERT=0
)(
 input wire clk,rst_n,init_done,key_pulse,vblank,
 input wire camera_frame_start,camera_frame_finish,camera_frame_good,
 input wire [17:0] camera_end_words,
 input wire [27:0] camera_end_actual,camera_end_expected,
 input wire camera_end_good,camera_write_quiet,camera_metadata_valid,
 input wire [80:0] camera_pose,output reg [80:0] source_pose,
 input wire [31:0] camera_frame_id,input wire [63:0] camera_sof_cycle,
 output reg camera_writer_active,output reg [1:0] camera_writer_bank,
 output reg continuous_run,render_start,render_bus_owner,
 output reg [1:0] source_bank,target_bank,
 output reg [31:0] source_frame_id,job_seq,source_epoch,
 output reg [63:0] source_sof_cycle,
 output reg source_locked,target_locked,output wire source_lease_active,allow_new_ar,
 input wire render_done,render_error,
 input wire [21:0] render_pixels,input wire [17:0] render_word_acks,input wire render_ack_count_complete,
 input wire [27:0] render_next_addr,input wire [5:0] render_tokens,
 input wire render_eof_seen,render_reader_quiet,render_writer_quiet,
 input wire render_arvalid,render_arready,input wire [27:0] render_araddr,input wire [3:0] render_arlen,
 input wire render_rvalid,render_rlast,render_return_pending,render_awvalid,render_write_inflight,
 input wire display_epoch_quiet,display_lease_active,
 output reg display_valid,output reg [1:0] display_bank,
 output wire display_launch_enable,
 output reg [31:0] display_frame_id,display_job_seq,display_context_epoch,
 output wire rearm_req,input wire rearm_ack,
 output reg bank_error,output reg [7:0] error_code,
 output reg [4:0] stage,output reg [31:0] publish_count,capture_bad_count,capture_denied_count,
 output reg completed_descriptor_accepted,output reg [63:0] display_sof_cycle
);
 localparam WAIT_START=0,WAIT_SOURCE=1,RESERVE=4,RENDER=5,SEAL_TARGET=6,
  QUEUE_COMPLETE=7,COMMIT=8,REARM=9,ERROR_HELD=15;
 reg [1:0] source_ready;
 reg [31:0] fid0,fid3,capture_fid;
 reg [80:0] pose0,pose3,capture_pose;
 reg [63:0] stamp0,stamp3,capture_stamp;
 reg oldest3;
 reg pending_valid;
 reg [1:0] pending_bank;
 reg [31:0] pending_fid,pending_job,pending_epoch;
 reg [63:0] pending_stamp;
 reg display_started;
 wire lease_close_ack,lease_error,lease_closing,lease_inflight;
 wire [7:0] lease_code;wire [1:0] lease_bank;wire [31:0] lease_epoch;
 wire render_drained=render_reader_quiet&&render_writer_quiet&&render_tokens==0&&
  !render_arvalid&&!render_rvalid&&!render_return_pending&&!render_awvalid&&!render_write_inflight&&!lease_inflight;
 wire [27:0] camera_base=camera_writer_bank==3?3*FRAME_ADDR_UNITS:28'd0;
 // Input remains a full 1080p even when a test renders a short target.
 wire camera_complete=camera_frame_good&&camera_end_good&&camera_end_words==18'd129600&&
  camera_end_actual==camera_end_expected&&camera_end_expected==camera_base+FRAME_ADDR_UNITS;
 wire [27:0] target_end=(target_bank==1?FRAME_ADDR_UNITS:2*FRAME_ADDR_UNITS)+FRAME_WORDS*8;
 // Actual board path receives the exact current counter predicate from the
 // genuine ACK producer. Legacy users retain the original full comparison.
 wire render_words_complete=USE_ACK_CERT?render_ack_count_complete:(render_word_acks==FRAME_WORDS);
 wire target_complete=render_done&&render_eof_seen&&render_pixels==FRAME_PIXELS&&
  render_words_complete&&render_next_addr==target_end&&render_drained&&!render_error;
 wire free_s0=!source_ready[0]&&!(source_locked&&source_bank==0)&&!(camera_writer_active&&camera_writer_bank==0);
 wire free_s3=!source_ready[1]&&!(source_locked&&source_bank==3)&&!(camera_writer_active&&camera_writer_bank==3);
 wire free_t1=!(display_valid&&display_bank==1)&&!(pending_valid&&pending_bank==1)&&!(target_locked&&target_bank==1);
 wire free_t2=!(display_valid&&display_bank==2)&&!(pending_valid&&pending_bank==2)&&!(target_locked&&target_bank==2);
 wire choose3=source_ready[1]&&(!source_ready[0]||oldest3);
 wire invalid_roles=(camera_writer_active&&(camera_writer_bank!=0&&camera_writer_bank!=3))||
  (camera_writer_active&&source_locked&&camera_writer_bank==source_bank)||
  (target_locked&&(target_bank==0||target_bank>2||(display_valid&&display_bank==target_bank)||(pending_valid&&pending_bank==target_bank)))||
  (pending_valid&&(pending_bank==0||pending_bank>2||(display_valid&&pending_bank==display_bank)))||
  (source_lease_active&&(!source_locked||lease_bank!=source_bank||lease_epoch!=source_epoch));
 wire camera_admission=(continuous_run||key_pulse)&&init_done&&(free_s0||free_s3);
 wire camera_metadata_fault=camera_frame_start&&!camera_writer_active&&camera_admission&&!camera_metadata_valid;
 wire camera_role_fault=(camera_frame_start&&(camera_writer_active||(camera_admission&&!camera_write_quiet)))||
  (camera_frame_finish&&camera_writer_active&&!camera_write_quiet);
 wire fatal_now=invalid_roles||lease_error||render_error||camera_metadata_fault||camera_role_fault;
 assign rearm_req=stage==REARM&&!bank_error&&!fatal_now;
 assign display_launch_enable=display_valid&&!bank_error&&!fatal_now;
 render_read_lease_guard #(.WATCHDOG_CYCLES(LEASE_WATCHDOG_CYCLES),.DUAL_SOURCE(1)) u_source_lease(
  .clk(clk),.rst_n(rst_n),.grant_req(stage==RESERVE&&!bank_error&&!fatal_now),
  .grant_bank(source_bank),.grant_epoch(source_epoch),.close_req(stage==SEAL_TARGET),.abort_req(bank_error||fatal_now),
  .arvalid(render_arvalid),.arready(render_arready),.araddr(render_araddr),.arlen(render_arlen),
  .rvalid(render_rvalid),.rlast(render_rlast),.return_pending(render_return_pending),
  .client_quiet(render_reader_quiet&&render_writer_quiet&&render_tokens==0&&!render_awvalid&&!render_write_inflight),
  .active(source_lease_active),.closing(lease_closing),.close_ack(lease_close_ack),.error(lease_error),
  .error_code(lease_code),.bank(lease_bank),.epoch(lease_epoch),.allow_new_ar(allow_new_ar),.inflight(lease_inflight));
 task fault(input [7:0] code);begin bank_error<=1;if(!bank_error)error_code<=code;stage<=ERROR_HELD;end endtask
 always @(posedge clk)begin
  if(!rst_n)begin
   stage<=WAIT_START;continuous_run<=0;camera_writer_active<=0;camera_writer_bank<=0;
   render_start<=0;render_bus_owner<=0;source_bank<=0;target_bank<=1;
   source_frame_id<=0;job_seq<=0;source_epoch<=0;source_sof_cycle<=0;
   source_locked<=0;target_locked<=0;display_valid<=0;display_bank<=1;
   display_frame_id<=0;display_job_seq<=0;display_context_epoch<=0;display_sof_cycle<=0;
   source_pose<=0;pose0<=0;pose3<=0;capture_pose<=0;source_ready<=0;fid0<=0;fid3<=0;stamp0<=0;stamp3<=0;capture_fid<=0;capture_stamp<=0;oldest3<=0;
   pending_valid<=0;pending_bank<=1;pending_fid<=0;pending_job<=0;pending_epoch<=0;pending_stamp<=0;completed_descriptor_accepted<=0;display_started<=0;
   publish_count<=0;capture_bad_count<=0;capture_denied_count<=0;bank_error<=0;error_code<=0;
  end else begin
   render_start<=0;
   if(key_pulse)continuous_run<=1;
   if(fatal_now)fault(lease_error?lease_code:camera_metadata_fault?8'h72:camera_role_fault?8'h73:8'h71);
   else if(!bank_error)begin
    // First token is held in CameraWriter's WAIT_BANK until this allocation.
    // Capture FID/stamp are separate from the immutable active Render context.
    if(camera_frame_start)begin
     if(camera_admission)begin
       camera_writer_active<=1;camera_writer_bank<=free_s0?2'd0:2'd3;
       capture_pose<=camera_pose;capture_fid<=camera_frame_id;capture_stamp<=camera_sof_cycle;
     end else capture_denied_count<=capture_denied_count+1'b1;
    end
    if(camera_frame_finish&&camera_writer_active)begin
     begin
      camera_writer_active<=0;
      if(camera_complete)begin
       if(camera_writer_bank==0)begin source_ready[0]<=1;pose0<=capture_pose;fid0<=capture_fid;stamp0<=capture_stamp;if(!source_ready[1])oldest3<=0;end
       else begin source_ready[1]<=1;pose3<=capture_pose;fid3<=capture_fid;stamp3<=capture_stamp;if(!source_ready[0])oldest3<=1;end
      end else capture_bad_count<=capture_bad_count+1'b1;
     end
    end
    // A descriptor is enqueued only after target completion and source close.
    // vblank consumes the previous immutable descriptor, never live job wires.
    if(display_valid&&display_lease_active)display_started<=1;
    // At first publication HDMI has not launched a read yet. Quiet alone must
    // not let a faster second job replace a frame that has never been scanned.
    if(vblank&&display_epoch_quiet&&pending_valid&&(!display_valid||display_started))begin
     display_bank<=pending_bank;display_valid<=1;
     display_frame_id<=pending_fid;display_job_seq<=pending_job;display_context_epoch<=pending_epoch;display_sof_cycle<=pending_stamp;
     publish_count<=publish_count+1'b1;pending_valid<=0;display_started<=0;
    end
    case(stage)
     WAIT_START:if((continuous_run||key_pulse)&&init_done)stage<=WAIT_SOURCE;
     WAIT_SOURCE:if(source_ready!=0&&(free_t1||free_t2)&&render_drained&&!source_lease_active)begin
      source_pose<=choose3?pose3:pose0;source_bank<=choose3?2'd3:2'd0;source_frame_id<=choose3?fid3:fid0;source_sof_cycle<=choose3?stamp3:stamp0;
      if(choose3)begin source_ready[1]<=0;oldest3<=0;end else begin source_ready[0]<=0;oldest3<=1;end
      target_bank<=free_t1?2'd1:2'd2;source_locked<=1;target_locked<=1;
      job_seq<=job_seq+1'b1;source_epoch<=source_epoch+1'b1;render_bus_owner<=1;completed_descriptor_accepted<=0;stage<=RESERVE;
     end
     RESERVE:if(source_lease_active&&lease_epoch==source_epoch)begin render_start<=1;stage<=RENDER;end
     RENDER:if(target_complete)stage<=SEAL_TARGET;
     SEAL_TARGET:if(lease_close_ack&&render_drained&&target_complete)stage<=QUEUE_COMPLETE;
     QUEUE_COMPLETE:if(!pending_valid&&target_complete&&!source_lease_active)begin
      pending_valid<=1;pending_bank<=target_bank;pending_fid<=source_frame_id;pending_job<=job_seq;pending_epoch<=source_epoch;pending_stamp<=source_sof_cycle;
      completed_descriptor_accepted<=1;target_locked<=0;source_locked<=0;stage<=COMMIT;
     end
     COMMIT:stage<=REARM;
     REARM:if(rearm_ack&&render_drained&&!source_lease_active)begin render_bus_owner<=0;stage<=WAIT_SOURCE;end
     ERROR_HELD:begin end
     default:fault(8'h7f);
    endcase
   end
  end
 end
endmodule
