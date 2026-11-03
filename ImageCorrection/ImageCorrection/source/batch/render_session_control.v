// Reusable session shell around the unchanged one-shot pixel renderer.
// Local reset is a consequence of an actual completion/lease certificate,
// never a substitute for drain and never reaches shared FIFOs or DDR.
module render_session_control #(parameter PIXELS=2073600,WORDS=129600)(
 input wire clk,rst_n,start,rearm_req,publication_confirmed,source_lease_active,abort_req,
 input wire [1:0] source_bank,target_bank,
 input wire [31:0] source_frame_id,source_epoch,job_seq,
 input wire engine_done,engine_error,input wire [21:0] engine_pixels,input wire [17:0] engine_words,
 input wire [27:0] engine_next_addr,input wire [5:0] engine_tokens,
 input wire engine_eof,engine_reader_quiet,engine_writer_quiet,engine_arvalid,engine_rvalid,
 input wire engine_return_pending,engine_read_inflight,engine_awvalid,engine_write_inflight,
 output reg engine_rst_n,engine_start,
 output reg [1:0] engine_source_bank,engine_target_bank,
 output wire [15:0] engine_frame_id,engine_epoch,
 output wire ready,output reg rearm_ack,error,output reg [7:0] error_code,output reg [2:0] stage
);
 localparam RESET=0,SETTLE=1,READY=2,BUSY=3,ERROR_HELD=4;
 reg reset_tick,reset_for_rearm,completion_saved,rearm_seen;
 reg [31:0] held_fid,held_epoch,held_job;
 assign engine_frame_id=held_fid[15:0];assign engine_epoch=held_epoch[15:0];
 assign ready=stage==READY&&!error;
 wire drained=engine_reader_quiet&&engine_writer_quiet&&engine_tokens==0&&!engine_arvalid&&!engine_rvalid&&
  !engine_return_pending&&!engine_read_inflight&&!engine_awvalid&&!engine_write_inflight;
 wire [27:0] expected_end=(engine_target_bank==1?28'hfd200:28'h1fa400)+WORDS*8;
 wire certificate=engine_done&&!engine_error&&engine_eof&&engine_pixels==PIXELS&&engine_words==WORDS&&
  engine_next_addr==expected_end&&drained;
 wire metadata_changed=source_bank!=engine_source_bank||target_bank!=engine_target_bank||
  source_frame_id!=held_fid||source_epoch!=held_epoch||job_seq!=held_job;
 task fault(input [7:0] code);begin error<=1;if(!error)error_code<=code;stage<=ERROR_HELD;end endtask
 always @(posedge clk)begin
  if(!rst_n)begin
   stage<=RESET;engine_rst_n<=0;engine_start<=0;reset_tick<=0;reset_for_rearm<=0;
   completion_saved<=0;rearm_seen<=0;rearm_ack<=0;error<=0;error_code<=0;
   engine_source_bank<=0;engine_target_bank<=1;held_fid<=0;held_epoch<=0;held_job<=0;
  end else begin
   engine_start<=0;rearm_ack<=0;
   if(!rearm_req)rearm_seen<=0;
   if(abort_req||engine_error)fault(8'ha1);
   else case(stage)
    RESET:if(reset_tick)begin engine_rst_n<=1;stage<=SETTLE;end else reset_tick<=1;
    SETTLE:begin stage<=READY;if(reset_for_rearm)rearm_ack<=1;reset_for_rearm<=0;end
    READY:if(start)begin
     if(!source_lease_active||!drained||source_bank!=0||target_bank==0||target_bank>2)fault(8'ha2);
     else begin
      engine_source_bank<=source_bank;engine_target_bank<=target_bank;
      held_fid<=source_frame_id;held_epoch<=source_epoch;held_job<=job_seq;
      completion_saved<=0;engine_start<=1;stage<=BUSY;
     end
    end
    BUSY:begin
     if(metadata_changed||start)fault(8'ha3);
     else begin
      if(certificate)completion_saved<=1;
      if(rearm_req&&!rearm_seen)begin
       if(!completion_saved||!certificate||!publication_confirmed||source_lease_active)fault(8'ha4);
       else begin
        engine_rst_n<=0;reset_tick<=0;reset_for_rearm<=1;rearm_seen<=1;stage<=RESET;
       end
      end
     end
    end
    ERROR_HELD:begin end // No local reset or ACK after any fault.
    default:fault(8'haf);
   endcase
  end
 end
endmodule
