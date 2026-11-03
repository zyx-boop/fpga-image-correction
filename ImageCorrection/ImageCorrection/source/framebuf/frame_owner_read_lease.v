// Standalone lease/drain observer. Client owns/holds AR until handshake.
// Reset must coordinate reader, Cache and DDR; local flush uses abort/drain.
module frame_owner_read_lease #(parameter SERVICE_TIMEOUT_CYCLES=125000000)(
 input wire clk,rst_n,frame_start,frame_finish,frame_good,vblank,snapshot_hold,
 output wire writer_active,display_valid,pending_valid,dropped_frame,
 output wire [1:0] writer_bank,display_bank,pending_bank,
 input wire lease_req_valid,lease_release_req,abort_req,
 output wire lease_req_ready,lease_grant,allow_new_ar,owner_vblank,
 output reg lease_active,closing,fault,timeout,protocol_fault,rd_inflight,
 output reg [1:0] lease_bank,
 output reg [15:0] lease_epoch,
 output reg [2:0] beat_count,
 input wire arvalid,arready,rvalid,rlast,
 input wire [27:0] araddr,
 input wire [3:0] arlen,
 input wire return_commit_pending,client_quiescent
);
 assign lease_req_ready=!lease_active && display_valid && !lease_release_req && !abort_req && !protocol_fault;
 assign lease_grant=lease_req_valid && lease_req_ready;
 assign owner_vblank=vblank && !snapshot_hold && !(lease_active || lease_grant);
 assign allow_new_ar=lease_active && !closing && !fault && !lease_release_req && !abort_req;
 reg ar_waiting;
 reg [27:0] saved_araddr;
 reg [3:0] saved_arlen;
 reg [31:0] stall_count;
 wire ar_hs=arvalid&&arready;
 wire [2:0] current_beat=rd_inflight ? beat_count : 3'd0;
 wire [27:0] bank_base=lease_bank==0?28'h0:(lease_bank==1?28'hfd200:28'h1fa400);
 wire addr_bad=arvalid && (araddr[2:0]!=0 || {1'b0,araddr}<{1'b0,bank_base} ||
                          ({1'b0,araddr}+29'd32)>({1'b0,bank_base}+29'hfd200));
 wire protocol_bad=(arvalid && (!lease_active || arlen!=3 || addr_bad ||
                    (closing&&!ar_waiting))) ||
                   (ar_waiting && (!arvalid || araddr!=saved_araddr || arlen!=saved_arlen)) ||
                   (ar_hs && rd_inflight) ||
                   (rvalid && (!(rd_inflight||ar_hs) || current_beat>3 ||
                               rlast!=(current_beat==3)));
 wire waiting=arvalid||rd_inflight||return_commit_pending||(closing&&!client_quiescent);
 wire forward_event=ar_hs||(rvalid&&!protocol_bad);
 wire drain_ok=closing&&!arvalid&&!ar_waiting&&!rd_inflight&&!rvalid&&
                !return_commit_pending&&client_quiescent&&!protocol_fault&&!protocol_bad;
 generate if(SERVICE_TIMEOUT_CYCLES<1) begin:bad_timeout
  initial $fatal(1,"watchdog cycles must be positive, not a realtime deadline");
 end endgenerate
 frame_owner owner(.ddr_clk(clk),.rst_n(rst_n),.frame_start(frame_start),
  .frame_finish(frame_finish),.frame_good(frame_good),.vblank(owner_vblank),
  .writer_active(writer_active),.writer_bank(writer_bank),.display_valid(display_valid),
  .display_bank(display_bank),.pending_valid(pending_valid),.pending_bank(pending_bank),
  .dropped_frame(dropped_frame));
 always @(posedge clk) begin
  if(!rst_n)begin
   lease_active<=0;closing<=0;lease_bank<=0;lease_epoch<=0;
   fault<=0;timeout<=0;protocol_fault<=0;rd_inflight<=0;beat_count<=0;
   ar_waiting<=0;saved_araddr<=0;saved_arlen<=0;stall_count<=0;
  end else begin
   if(lease_grant)begin
    lease_active<=1;closing<=0;lease_bank<=display_bank;lease_epoch<=lease_epoch+1'b1;
    fault<=0;timeout<=0;stall_count<=0;
   end else if(lease_active)begin
    if(lease_release_req||abort_req)closing<=1;
    if(drain_ok)begin
     lease_active<=0;closing<=0;
    end
   end
   if(arvalid&&!arready&&!ar_waiting)begin
    ar_waiting<=1;saved_araddr<=araddr;saved_arlen<=arlen;
   end
   if(ar_hs)begin
    ar_waiting<=0;
    if(!rd_inflight)begin rd_inflight<=1;beat_count<=0;end
   end
   if(rvalid && (rd_inflight||ar_hs))begin
    if(current_beat<4)beat_count<=current_beat+1'b1;
    if(current_beat==3 && rlast)begin rd_inflight<=0;beat_count<=0;end
   end
   if(protocol_bad)begin protocol_fault<=1;fault<=1;if(lease_active)closing<=1;end
   if(lease_active && waiting && !forward_event)begin
    if(stall_count>=SERVICE_TIMEOUT_CYCLES-1)begin
     timeout<=1;fault<=1;closing<=1;
    end else stall_count<=stall_count+1'b1;
   end else stall_count<=0;
  end
 end
endmodule
