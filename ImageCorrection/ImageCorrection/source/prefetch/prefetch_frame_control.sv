// New controller, not a loop around the former single-frame renderer FSM.
module prefetch_frame_control(
 input wire clk,rst,run_enable,app_abort,external_fault,
 input wire descriptor_valid,output wire descriptor_ready,
 input wire [1:0] descriptor_bank,input wire [31:0] descriptor_fid,descriptor_job,descriptor_epoch,
 output wire source_request,output reg [31:0] next_job,next_epoch,
 input wire modules_open_ready,caller_quiet,reader_closed,adapter_quiet,prefetch_quiet,window_idle,window_rows_done,lease_active,
 input wire launch_take,reader_take,output_take,output_eof,
 output wire context_open,context_active,stop_new_rows,caller_close,caller_abort,reader_close,reader_abort,window_close,lease_release_req,
 output wire job_done_valid,input wire job_done_ready,output reg aborted,fault,output reg [7:0] error_code,
  output reg [1:0] ctx_bank,output reg [31:0] ctx_fid,ctx_job,ctx_epoch,launched_count,reader_input_count,retired_count
 );
 localparam [3:0] WAIT_SOURCE=0,RUN=1,SEAL=2,READER_CLOSE=3,WINDOW_CLOSE=4,LEASE_CLOSE=5,DONE=6,HALT=7;
 reg [3:0] state;reg normal_complete,eof_seen;
 assign descriptor_ready=state==WAIT_SOURCE&&modules_open_ready&&!fault;
 assign context_open=descriptor_valid&&descriptor_ready;
 assign source_request=state==WAIT_SOURCE&&run_enable&&!app_abort&&!fault;
 assign context_active=state!=WAIT_SOURCE&&state!=DONE;
 assign stop_new_rows=state!=RUN||app_abort||aborted||fault;
 assign caller_close=state==SEAL;
 assign caller_abort=app_abort||aborted||fault;
 assign reader_close=state==READER_CLOSE;
 assign reader_abort=app_abort||aborted||fault;
 assign window_close=state==WINDOW_CLOSE;
 assign lease_release_req=state==LEASE_CLOSE&&caller_quiet&&reader_closed&&adapter_quiet&&prefetch_quiet&&window_idle&&!fault;
 assign job_done_valid=state==DONE&&!fault;
 always @(posedge clk)begin
  if(rst)begin
   state<=WAIT_SOURCE;next_job<=1;next_epoch<=32'h10000;
   ctx_bank<=0;ctx_fid<=0;ctx_job<=0;ctx_epoch<=0;
   aborted<=0;fault<=0;error_code<=0;normal_complete<=0;eof_seen<=0;
   launched_count<=0;reader_input_count<=0;retired_count<=0;
  end else if(!fault)begin
   if(context_open)begin
    ctx_bank<=descriptor_bank;ctx_fid<=descriptor_fid;ctx_job<=descriptor_job;ctx_epoch<=descriptor_epoch;
    next_job<=next_job+1'b1;next_epoch<=next_epoch+1'b1;
    launched_count<=0;reader_input_count<=0;retired_count<=0;eof_seen<=0;normal_complete<=0;aborted<=app_abort;
    state<=app_abort?SEAL:RUN;
    if(descriptor_job!=next_job||descriptor_epoch!=next_epoch)begin fault<=1;error_code<=8'h20;state<=HALT;end
   end else begin
    if(launch_take)launched_count<=launched_count+1'b1;
    if(reader_take)reader_input_count<=reader_input_count+1'b1;
    if(output_take)begin retired_count<=retired_count+1'b1;if(output_eof)eof_seen<=1;end
    if(app_abort&&context_active)aborted<=1;
    case(state)
     RUN:if(app_abort||window_rows_done)begin normal_complete<=window_rows_done;state<=SEAL;end
     SEAL:if(caller_quiet)state<=READER_CLOSE;
     READER_CLOSE:if(reader_closed&&adapter_quiet&&prefetch_quiet)begin
      if(launched_count!=reader_input_count||launched_count!=retired_count||
         (!aborted&&(!normal_complete||!eof_seen||retired_count!=2073600)))begin fault<=1;error_code<=8'h21;state<=HALT;end
      else state<=WINDOW_CLOSE;
     end
     WINDOW_CLOSE:if(window_idle&&caller_quiet&&reader_closed&&adapter_quiet&&prefetch_quiet)state<=LEASE_CLOSE;
     LEASE_CLOSE:if(!lease_active)state<=DONE;
     DONE:if(job_done_ready)state<=WAIT_SOURCE;
     default:begin end
    endcase
   end
   if(external_fault)begin fault<=1;error_code<=8'h10;state<=HALT;end
  end
 end
endmodule
