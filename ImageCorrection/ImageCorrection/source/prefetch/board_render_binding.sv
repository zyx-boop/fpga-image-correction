// Board-only descriptor, target ACK and rearm binding. No synthetic Owner/Lease.
module board_render_binding #(parameter PIPELINED_MODE=0,USE_ACK_CERT=0)(
 input wire clk,rst_n,start,abort_req,rearm_req,publication_confirmed,source_lease_active,
 input wire completed_descriptor_accepted,
 input wire [1:0] source_bank,target_bank,
 input wire [31:0] source_fid,source_epoch,job_seq,next_job,next_epoch,
 input wire modules_open_ready,context_open,core_release_req,core_job_done,core_aborted,core_quiet,
 input wire writer_done,writer_quiet,input wire [17:0] writer_words,input wire writer_ack_count_complete,
 input wire [21:0] output_pixels,input wire [27:0] next_write_addr,input wire eof_seen,
 output reg descriptor_valid,output reg [1:0] descriptor_bank,held_target_bank,
 output reg [31:0] descriptor_fid,descriptor_job,descriptor_epoch,
 output wire writer_reset_n,job_done_ready,
  output reg done,rearm_ack,error,output reg [7:0] error_code
);
 localparam IDLE=0,OPEN=1,ACTIVE=2,ACK_CORE=3,RESET_WRITER=4,READY_WRITER=5;
 reg [2:0] state;reg [1:0] reset_ticks;reg aborted;
 reg [31:0] held_source_epoch;
 reg source_bank_ok;
 always_comb begin
  source_bank_ok=0;
  case(source_bank) 0:source_bank_ok=1;3:source_bank_ok=PIPELINED_MODE;default:source_bank_ok=0;endcase
 end
 // The scheduler may rearm only after an immutable completed target descriptor
 // owns this job. All original full-frame/quiet/Lease certificates still apply.
 wire completion_owned=PIPELINED_MODE?completed_descriptor_accepted:publication_confirmed;
 wire [27:0] target_end=(held_target_bank==1?28'hfd200:28'h1fa400)+28'hfd200;
 wire context_changed=source_bank!=descriptor_bank||target_bank!=held_target_bank||
   source_fid!=descriptor_fid||job_seq!=descriptor_job||source_epoch!=held_source_epoch;
 wire words_complete=USE_ACK_CERT?writer_ack_count_complete:(writer_words==129600);
 wire certificate=core_release_req&&core_quiet&&writer_quiet&&writer_done&&
   words_complete&&output_pixels==2073600&&eof_seen&&next_write_addr==target_end&&
   !core_aborted&&!aborted&&!abort_req;
 assign writer_reset_n=rst_n&&state!=RESET_WRITER;
 assign job_done_ready=state==ACK_CORE&&!error;
 task fail(input [7:0] code);begin error<=1;if(!error)error_code<=code;end endtask
 always @(posedge clk)begin
  if(!rst_n)begin
   state<=IDLE;descriptor_valid<=0;descriptor_bank<=0;held_target_bank<=1;
   descriptor_fid<=0;descriptor_job<=0;descriptor_epoch<=0;held_source_epoch<=0;
   done<=0;rearm_ack<=0;error<=0;error_code<=0;reset_ticks<=0;aborted<=0;
  end else begin
   rearm_ack<=0;
   if(abort_req&&state!=IDLE)aborted<=1;
   if(state!=IDLE&&context_changed)fail(8'hc2);
   if(start&&state!=IDLE)fail(8'hc3);
   if(!error)case(state)
    IDLE:if(start)begin
     if(abort_req||!source_lease_active||!core_quiet||!writer_quiet||!modules_open_ready||
        !source_bank_ok||target_bank==0||target_bank>2||job_seq!=next_job||
        source_epoch+32'hffff!=next_epoch)fail(8'hc1);
     else begin
      descriptor_valid<=1;descriptor_bank<=source_bank;held_target_bank<=target_bank;
      descriptor_fid<=source_fid;descriptor_job<=job_seq;descriptor_epoch<=next_epoch;
      held_source_epoch<=source_epoch;done<=0;aborted<=0;state<=OPEN;
     end
    end
    OPEN:if(context_open)begin descriptor_valid<=0;state<=ACTIVE;end
    ACTIVE:begin
     if(certificate&&!context_changed)done<=1;
     if(rearm_req)begin
      if(!done||!completion_owned||source_lease_active||!core_job_done||!core_quiet||!writer_quiet||aborted||abort_req)
       fail(8'hc4);
      else state<=ACK_CORE;
     end
    end
    ACK_CORE:begin
     if(!core_job_done||source_lease_active||!core_quiet||!writer_quiet)fail(8'hc5);
     else begin reset_ticks<=0;state<=RESET_WRITER;end
    end
    RESET_WRITER:begin
     if(source_lease_active||!core_quiet)fail(8'hc6);
     if(reset_ticks==1)state<=READY_WRITER;else reset_ticks<=reset_ticks+1'b1;
    end
    READY_WRITER:if(modules_open_ready&&writer_quiet&&!core_job_done)begin rearm_ack<=1;state<=IDLE;end
    default:fail(8'hcf);
   endcase
  end
 end
endmodule
