// Pure source lease: no raw frame_owner and no display-return observation.
// The client retains all offered transactions; faults never force a release.
module render_read_lease_guard #(parameter WATCHDOG_CYCLES=125000000,DUAL_SOURCE=0)(
 input wire clk,rst_n,grant_req,close_req,abort_req,
 input wire [1:0] grant_bank,input wire [31:0] grant_epoch,
 input wire arvalid,arready,input wire [27:0] araddr,input wire [3:0] arlen,
 input wire rvalid,rlast,return_pending,client_quiet,
 output reg active,closing,close_ack,error,output reg [7:0] error_code,
 output reg [1:0] bank,output reg [31:0] epoch,
 output wire allow_new_ar,output reg inflight
);
 reg waiting;reg [27:0] held_addr;reg [3:0] held_len;
 reg [2:0] beat;reg [31:0] stalls;
 wire take=arvalid&&arready;
 reg grant_bank_ok;reg [27:0] source_base;
 always_comb begin
  grant_bank_ok=0;source_base=0;
  case(grant_bank) 0:grant_bank_ok=1;3:grant_bank_ok=DUAL_SOURCE;default:grant_bank_ok=0;endcase
  case(bank) 3:source_base=28'h2f7600;default:source_base=0;endcase
 end
 wire [2:0] current_beat=inflight?beat:3'd0;
 wire protocol_bad=(arvalid&&(!active||arlen!=3||araddr[4:0]!=0||
                           {1'b0,araddr}<{1'b0,source_base}||
                           {1'b0,araddr}+29'd32>{1'b0,source_base}+29'hfd200||
                           (closing&&!waiting))) ||
   (waiting&&(!arvalid||araddr!=held_addr||arlen!=held_len)) || (take&&inflight) ||
   (rvalid&&(!(inflight||take)||current_beat>3||rlast!=(current_beat==3)));
 wire drained=closing&&!arvalid&&!waiting&&!inflight&&!rvalid&&!return_pending&&client_quiet;
 assign allow_new_ar=active&&!closing&&!close_req&&!abort_req&&!error;
 task fault(input [7:0] code);begin error<=1;if(!error)error_code<=code;closing<=1;end endtask
 always @(posedge clk)begin
  if(!rst_n)begin
   active<=0;closing<=0;close_ack<=0;error<=0;error_code<=0;bank<=0;epoch<=0;
   waiting<=0;held_addr<=0;held_len<=0;beat<=0;inflight<=0;stalls<=0;
  end else begin
   close_ack<=0;
   if(grant_req&&!active&&!error)begin
    if(!grant_bank_ok||arvalid||rvalid||return_pending||!client_quiet)fault(8'h61);
    else begin active<=1;closing<=0;bank<=grant_bank;epoch<=grant_epoch;stalls<=0;end
   end
   if(active&&(close_req||abort_req))closing<=1;
   if(active&&drained&&!error&&!protocol_bad)begin active<=0;closing<=0;close_ack<=1;end
   if(arvalid&&!arready&&!waiting)begin waiting<=1;held_addr<=araddr;held_len<=arlen;end
   if(take)begin waiting<=0;inflight<=1;beat<=0;end
   if(rvalid&&(inflight||take))begin
    if(current_beat==3&&rlast)begin inflight<=0;beat<=0;end
    else beat<=current_beat+1'b1;
   end
   if(protocol_bad)fault(8'h62);
   if(active&&(arvalid||inflight||return_pending||(closing&&!client_quiet))&&!take&&!rvalid)begin
    if(stalls>=WATCHDOG_CYCLES-1)fault(8'h63);else stalls<=stalls+1'b1;
   end else stalls<=0;
  end
 end
endmodule
