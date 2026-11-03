// One real 4-beat DDR read in flight. No RREADY: reserve all four beats.
// Application stop cannot cancel an offered AR or an accepted RAM row.
module ddr4_row_prefetch #(parameter WATCHDOG=125000000,DUAL_SOURCE=0)(
 input wire clk,rst,row_req_valid,stop_new_rows,allow_new_ar,
 output wire row_req_ready,
 input wire [1:0] ctx_bank,input wire [31:0] ctx_fid,ctx_job,ctx_epoch,
 input wire [10:0] row_req_y,
 output wire row_done_valid,input wire row_done_ready,
 output wire [10:0] row_done_y,output wire [31:0] row_done_epoch,
 output wire busy,commit_pending,output reg fault,timeout,output reg [7:0] error_code,
 output wire arvalid,input wire arready,output reg [27:0] araddr,output wire [3:0] arlen,
 input wire rvalid,rlast,input wire [255:0] rdata,
 output wire wr_begin,input wire wr_begin_ready,
 output wire [10:0] wr_y,output wire [31:0] wr_epoch,
 output wire wr_valid,input wire wr_ready,output wire [6:0] wr_word,output wire [255:0] wr_data,
  output reg [31:0] ar_count,r_count,burst_count,word_count,rows_count
 );
 localparam [3:0] IDLE=0,BEGIN_ROW=1,PREP=2,OFFER=3,RECEIVE=4,COMMIT=5,DONE=6,QUARANTINE=7;
 reg [3:0] state;
 reg [1:0] bank,commit_index;
 reg [2:0] received;
 reg [4:0] burst;
 reg [10:0] y;
 reg [31:0] fid,job,epoch,stall_count;
 reg [27:0] next_addr;
 reg [255:0] beats[0:3] /* synthesis syn_ramstyle="registers" */;
 wire [27:0] bank_base=DUAL_SOURCE?(ctx_bank==3?28'h2f7600:28'd0):
  (ctx_bank==0?28'h0:ctx_bank==1?28'hfd200:28'h1fa400);
 reg source_bank_ok;
 always_comb begin
  source_bank_ok=0;
  if(DUAL_SOURCE)case(ctx_bank) 0,3:source_bank_ok=1;default:source_bank_ok=0;endcase
  else case(ctx_bank) 0,1,2:source_bank_ok=1;default:source_bank_ok=0;endcase
 end
 assign row_req_ready=state==IDLE&&!stop_new_rows&&!fault&&!rvalid;
 assign wr_begin=state==BEGIN_ROW&&!fault;
 assign wr_y=y;assign wr_epoch=epoch;
 assign wr_valid=state==COMMIT&&!fault;
 assign wr_word={burst,2'b00}+commit_index;
 // D58: explicit same-cycle read of four real storage slots, not a
 // look-ahead output FF whose control cone includes the current DDR RVALID.
 // Unknown index retains the original unpacked-array X semantics.
 reg [255:0] commit_data;
 always_comb begin
  case(commit_index)
   2'd0:commit_data=beats[0];
   2'd1:commit_data=beats[1];
   2'd2:commit_data=beats[2];
   2'd3:commit_data=beats[3];
   default:commit_data={256{1'bx}};
  endcase
 end
 reg [255:0] commit_head;
 reg [255:0] visible_commit;
 always_comb begin
  visible_commit=commit_head;
`ifndef SYNTHESIS
  // Invalid-cycle values and unknown simulation indices remain observational.
  // The real valid/accepted data always exercises the explicit head cache.
  if(wr_valid!==1'b1 || (^commit_index)===1'bx)visible_commit=commit_data;
`endif
 end
 assign wr_data=visible_commit;
 assign arvalid=state==OFFER; // MUST remain offered even if timeout/stop asserts.
 assign arlen=4'd3;
 assign row_done_valid=state==DONE&&!fault;
 assign row_done_y=y;assign row_done_epoch=epoch;
 assign busy=state!=IDLE;
 assign commit_pending=state!=IDLE||fault;
 wire ar_hs=arvalid&&arready;
 wire begin_hs=wr_begin&&wr_begin_ready;
 wire write_hs=wr_valid&&wr_ready;
 wire expecting_r=state==RECEIVE||ar_hs;
 wire [2:0] beat_index=(state==RECEIVE)?received:3'd0;
 wire bad_r=rvalid&&(!expecting_r||beat_index>3||rlast!=(beat_index==3));
 wire progress=ar_hs||begin_hs||write_hs||(rvalid&&expecting_r&&!bad_r);
 wire wait_active=state!=IDLE&&state!=DONE&&state!=QUARANTINE;
 wire watchdog_fire=wait_active&&!progress&&(stall_count>=WATCHDOG-1);
 // By the fourth real return, beat0 has been stable for at least two clocks.
 // Observe it while receiving; never place the late RVALID/fault cone on
 // a wide payload CE. Actual RAM Word commitment remains write_hs only.
 always @(posedge clk)begin
  if(rst)commit_head<=0;
  else if(state==RECEIVE)commit_head<=beats[0];
  else if(write_hs&&commit_index!=3)begin
   case(commit_index)
    0:commit_head<=beats[1];1:commit_head<=beats[2];2:commit_head<=beats[3];
    default:commit_head<={256{1'bx}};
   endcase
  end
 end
 always @(posedge clk)begin
  if(rst)begin
   state<=IDLE;bank<=0;commit_index<=0;received<=0;burst<=0;y<=0;fid<=0;job<=0;epoch<=0;
   next_addr<=0;araddr<=0;stall_count<=0;fault<=0;timeout<=0;error_code<=0;
   ar_count<=0;r_count<=0;burst_count<=0;word_count<=0;rows_count<=0;
  end else begin
   if(!wait_active||progress)stall_count<=0;
   else if(!timeout)stall_count<=stall_count+1'b1;
   if(rvalid)r_count<=r_count+1'b1;
   case(state)
    IDLE:if(row_req_valid&&row_req_ready)begin
     if(!source_bank_ok||row_req_y>=1080)begin state<=QUARANTINE;fault<=1;error_code<=8'h01;end
     else begin
      bank<=ctx_bank;y<=row_req_y;fid<=ctx_fid;job<=ctx_job;epoch<=ctx_epoch;
      next_addr<=bank_base+({17'b0,row_req_y}<<10)-({17'b0,row_req_y}<<6);
      burst<=0;state<=BEGIN_ROW;
     end
    end
    BEGIN_ROW:begin
     if(begin_hs)state<=PREP; // Acceptance wins a simultaneous application stop.
     else if(stop_new_rows)state<=IDLE; // RAM row never began: safe to cancel.
    end
    PREP:if(allow_new_ar&&!fault)begin araddr<=next_addr;received<=0;state<=OFFER;end
    OFFER:if(ar_hs)begin
     ar_count<=ar_count+1'b1;received<=0;state<=RECEIVE;
     if(rvalid)begin beats[0]<=rdata;received<=1;end
    end
    RECEIVE:if(rvalid&&beat_index<4)begin
     beats[beat_index[1:0]]<=rdata;
     if(beat_index==3)begin
      if(fault||bad_r||watchdog_fire)state<=QUARANTINE;
      else begin burst_count<=burst_count+1'b1;commit_index<=0;state<=COMMIT;end
     end else received<=received+1'b1;
    end
    COMMIT:if(write_hs)begin
     word_count<=word_count+1'b1;
     if(commit_index==3)begin
      if(burst==29)begin rows_count<=rows_count+1'b1;state<=DONE;end
      else begin burst<=burst+1'b1;next_addr<=next_addr+28'd32;state<=PREP;end
     end else commit_index<=commit_index+1'b1;
    end
    DONE:if(row_done_ready)state<=IDLE;
    default:state<=QUARANTINE;
   endcase
   if(bad_r||watchdog_fire)begin
    fault<=1;
    if(!fault)error_code<=bad_r?8'h02:8'h03;
    if(watchdog_fire)timeout<=1;
    // Never withdraw an offered AR, or pretend an accepted read has drained.
    if(state!=OFFER&&state!=RECEIVE)state<=QUARANTINE;
   end
  end
 end
`ifndef SYNTHESIS
 initial if(WATCHDOG<1)$fatal(1,"WATCHDOG must be positive");
`endif
endmodule
