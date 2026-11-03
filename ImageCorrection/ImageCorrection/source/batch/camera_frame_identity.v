// Passive identity channel paired with the unchanged pixel FIFO boundary
// tokens. Includes bad EOF without a SOF Word; never gates pixel writes.
module camera_frame_identity #(parameter SNAPSHOT_CYCLES=125000000,INITIAL_FRAME_ID=0)(
 input wire cam_clk,core_clk,rst_n,cam_rst_n,startup_ready,capture_sof,capture_done,capture_good,
 input wire [80:0] camera_pose,output wire [80:0] frame_pose,
 input wire token_wr_en,input wire [258:0] token_wr_data,
 input wire token_rd_en,token_rd_empty,input wire [258:0] token_rd_data,
 input wire writer_frame_start,writer_frame_finish,writer_frame_good,writer_active,
 output wire sof_metadata_valid,output wire [31:0] frame_id,output wire [63:0] sof_cycle,
 output reg [31:0] input_frame_snapshot,input_good_snapshot,
 output reg [31:0] drop_capture_bad,drop_schedule,drop_write_bad,
  output wire error,output wire [7:0] error_code
);
 reg [31:0] fid_cam,frames_cam,good_cam;
 reg time_req,time_busy,time_ready;reg [63:0] stamp_cam;
 (* ASYNC_REG="TRUE" *) reg [1:0] time_ack_sync;
 reg time_ack;reg [63:0] stamp_core,core_cycles;
 (* ASYNC_REG="TRUE" *) reg [1:0] time_req_sync;
 reg cam_error;reg [7:0] cam_code;
 (* ASYNC_REG="TRUE" *) reg [1:0] cam_error_sync;
 reg core_error;reg [7:0] core_code;
 wire write_sof=token_wr_data[258:257]==0&&token_wr_data[256];
 wire write_eof=token_wr_data[258:257]==1||token_wr_data[258:257]==2;
 wire write_boundary=token_wr_en&&(write_sof||write_eof);
 wire meta_full,meta_empty;wire [178:0] meta_data;
 reg [80:0] pose_cam,held_pose;
 wire meta_write=write_boundary&&time_ready&&!meta_full&&!cam_error;
 reg rd_pending,meta_pending;reg [1:0] requested_kind;
 wire read_sof=token_rd_data[258:257]==0&&token_rd_data[256];
 wire read_eof=token_rd_data[258:257]==1||token_rd_data[258:257]==2;
 wire read_boundary=rd_pending&&(read_sof||read_eof);
 wire meta_read=read_boundary&&!meta_empty&&!core_error;
 reg open_frame,admitted,admission_wait;
 reg [31:0] held_fid,last_end_fid;reg [63:0] held_stamp;
 wire pending_sof=meta_pending&&requested_kind==0;
 assign error=core_error||cam_error_sync[1];
 assign error_code=core_error?core_code:cam_error_sync[1]?cam_code:8'd0;
 assign sof_metadata_valid=pending_sof&&meta_data[97:96]==0&&!error;
 assign frame_id=pending_sof?meta_data[95:64]:held_fid;
 assign sof_cycle=pending_sof?meta_data[63:0]:held_stamp;
 assign frame_pose=pending_sof?meta_data[178:98]:held_pose;
 async_word_fifo #(.WIDTH(179),.ADDR_BITS(4),.SPLIT_RESET(1)) metadata_fifo(
  .wr_clk(cam_clk),.rd_clk(core_clk),.rst_n(rst_n),.wr_en(meta_write),
  .wr_rst_n(cam_rst_n),.rd_rst_n(rst_n),
  .wr_data({pose_cam,write_sof?2'd0:token_wr_data[258:257],fid_cam,stamp_cam}),
  .wr_full(meta_full),.wr_used(),.wr_overflow(),
  .rd_en(meta_read),.rd_data(meta_data),.rd_empty(meta_empty),.rd_underflow());
 // Separate held snapshots: no running multi-bit count read across clocks.
 reg snap_req,snap_busy,snap_ack;reg [31:0] snap_ticks;
 reg [63:0] counts_cam;
 (* ASYNC_REG="TRUE" *) reg [1:0] snap_req_sync,snap_ack_sync;
 task cam_fault(input [7:0] code);begin cam_error<=1;if(!cam_error)cam_code<=code;end endtask
 task core_fault(input [7:0] code);begin core_error<=1;if(!core_error)core_code<=code;end endtask
 always @(posedge cam_clk or negedge cam_rst_n)begin
  if(!cam_rst_n)begin
   pose_cam<=0;fid_cam<=INITIAL_FRAME_ID;frames_cam<=0;good_cam<=0;time_req<=0;time_busy<=0;time_ready<=0;
   time_ack_sync<=0;stamp_cam<=0;cam_error<=0;cam_code<=0;
   snap_req_sync<=0;snap_ack<=0;counts_cam<=0;
  end else begin
   time_ack_sync<={time_ack_sync[0],time_ack};snap_req_sync<={snap_req_sync[0],snap_req};
   if(capture_sof)begin
    pose_cam<=camera_pose;
    fid_cam<=fid_cam+1'b1;frames_cam<=frames_cam+1'b1;
    if(time_busy)cam_fault(8'h81);
    else begin time_req<=~time_req;time_busy<=1;time_ready<=0;end
   end
   if(time_busy&&time_ack_sync[1]==time_req)begin
    // stamp_core was frozen before its ack traversed two camera registers.
    stamp_cam<=stamp_core;time_busy<=0;time_ready<=1;
   end
   if(capture_done&&capture_good)good_cam<=good_cam+1'b1;
   if(write_boundary)begin
    if(meta_full)cam_fault(8'h82);
    else if(!time_ready||capture_sof)cam_fault(8'h83);
   end
   if(snap_req_sync[1]!=snap_ack)begin counts_cam<={frames_cam,good_cam};snap_ack<=snap_req_sync[1];end
  end
 end
 always @(posedge core_clk or negedge rst_n)begin
  if(!rst_n)begin
   core_cycles<=0;time_req_sync<=0;time_ack<=0;stamp_core<=0;cam_error_sync<=0;
   rd_pending<=0;meta_pending<=0;requested_kind<=0;open_frame<=0;admitted<=0;admission_wait<=0;
   held_pose<=0;held_fid<=INITIAL_FRAME_ID;last_end_fid<=INITIAL_FRAME_ID;held_stamp<=0;
   core_error<=0;core_code<=0;drop_capture_bad<=0;drop_schedule<=0;drop_write_bad<=0;
   snap_req<=0;snap_busy<=0;snap_ticks<=0;snap_ack_sync<=0;input_frame_snapshot<=0;input_good_snapshot<=0;
  end else begin
   core_cycles<=core_cycles+1'b1;
   time_req_sync<={time_req_sync[0],time_req};cam_error_sync<={cam_error_sync[0],cam_error};
   if(time_req_sync[1]!=time_ack)begin stamp_core<=core_cycles;time_ack<=time_req_sync[1];end
   snap_ack_sync<={snap_ack_sync[0],snap_ack};
   if(snap_busy&&snap_ack_sync[1]==snap_req)begin
    input_frame_snapshot<=counts_cam[63:32];input_good_snapshot<=counts_cam[31:0];snap_busy<=0;
   end
   if(!startup_ready)snap_ticks<=0;
   else if(snap_ticks>=SNAPSHOT_CYCLES-1)begin
    if(!snap_busy)begin snap_req<=~snap_req;snap_busy<=1;snap_ticks<=0;end
   end else snap_ticks<=snap_ticks+1'b1;
   rd_pending<=token_rd_en&&!token_rd_empty;
   meta_pending<=meta_read;
   if(read_boundary)begin
    requested_kind<=read_sof?2'd0:token_rd_data[258:257];
    if(meta_empty)core_fault(8'h84);
   end
   if(admission_wait)begin admitted<=writer_active;admission_wait<=0;end
   if(meta_pending)begin
    if(meta_data[97:96]!=requested_kind||meta_data[95:64]!=(last_end_fid+1'b1))core_fault(8'h85);
    else if(requested_kind==0)begin
     if(open_frame||!writer_frame_start)core_fault(8'h86);
     else begin open_frame<=1;held_pose<=meta_data[178:98];held_fid<=meta_data[95:64];held_stamp<=meta_data[63:0];admission_wait<=1;end
    end else begin
     if(open_frame&&held_fid!=meta_data[95:64])core_fault(8'h87);
     else if(!open_frame&&requested_kind==1)core_fault(8'h88);
     else begin
      if(requested_kind==2)drop_capture_bad<=drop_capture_bad+1'b1;
      else if(!(admission_wait?writer_active:admitted))drop_schedule<=drop_schedule+1'b1;
      else if(!writer_frame_finish||!writer_frame_good)drop_write_bad<=drop_write_bad+1'b1;
      open_frame<=0;admitted<=0;last_end_fid<=meta_data[95:64];
     end
    end
   end
  end
 end
endmodule
