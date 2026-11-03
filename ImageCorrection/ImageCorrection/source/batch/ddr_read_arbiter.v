// Native Pango read port: no RREADY, one globally accepted burst at a time.
// Both clients keep their AR offer until their routed handshake; closing a
// lease cannot cancel an already offered request, even before our selection.
module ddr_read_arbiter #(parameter DUAL_SOURCE=0)(
 input wire clk,rst_n,source_lease_active,input wire [31:0] source_epoch,
 input wire [1:0] source_bank,
 input wire s_arvalid,output wire s_arready,input wire [27:0] s_araddr,input wire [3:0] s_arlen,
 output wire [255:0] s_rdata,output wire s_rvalid,s_rlast,
 input wire display_lease_active,input wire [1:0] display_bank,input wire [15:0] display_epoch,
 input wire [9:0] fifo_used,
 input wire d_arvalid,output wire d_arready,input wire [27:0] d_araddr,input wire [3:0] d_arlen,
 output wire [255:0] d_rdata,output wire d_rvalid,d_rlast,
 output wire arvalid,input wire arready,output wire [27:0] araddr,output wire [3:0] arlen,
 input wire [255:0] rdata,input wire rvalid,rlast,
 output reg error,output reg [7:0] error_code,output wire quiet,
 output reg [31:0] source_wait_max,display_wait_max,completed_bursts
);
 localparam IDLE=0,OFFER=1,RETURN=2;
 reg [1:0] state;reg selected_display,urgent,display_gap_waited;
 reg [27:0] held_addr;reg [3:0] held_len;
 reg [31:0] held_source_epoch;reg [15:0] held_display_epoch;reg [1:0] held_display_bank,held_source_bank;
 reg [4:0] beat;reg [31:0] swait,dwait;
 wire s_candidate=s_arvalid&&source_lease_active;
 wire d_candidate=d_arvalid&&display_lease_active;
 wire display_priority=(fifo_used<384)&&(urgent||fifo_used<=128);
 wire pick_display=d_candidate&&(!s_candidate||display_priority);
 assign arvalid=state==OFFER;
 assign araddr=held_addr;
 assign arlen=held_len;
 assign s_arready=state==OFFER&&!selected_display&&arready;
 assign d_arready=state==OFFER&&selected_display&&arready;
 wire take=arvalid&&arready;
 wire return_context=state==RETURN||take;
 assign s_rvalid=rvalid&&return_context&&!selected_display;
 assign d_rvalid=rvalid&&return_context&&selected_display;
 assign s_rdata=rdata;assign d_rdata=rdata;
 assign s_rlast=rlast&&s_rvalid;assign d_rlast=rlast&&d_rvalid;
 assign quiet=state==IDLE&&!rvalid&&!arvalid;
 wire [4:0] current_beat=state==RETURN?beat:5'd0;
 wire [27:0] display_base=display_bank==1?28'hfd200:28'h1fa400;
 reg source_bank_ok;reg [27:0] source_base;
 always_comb begin
  source_bank_ok=1;source_base=0;
  if(DUAL_SOURCE)case(source_bank)
   0:source_base=0;3:source_base=28'h2f7600;default:source_bank_ok=0;
  endcase
 end
 wire source_bad=s_arvalid&&(!source_lease_active||!source_bank_ok||s_arlen!=3||s_araddr[4:0]!=0||
  {1'b0,s_araddr}<{1'b0,source_base}||{1'b0,s_araddr}+29'd32>{1'b0,source_base}+29'hfd200);
 wire display_bad=d_arvalid&&(!display_lease_active||display_bank==0||display_bank>2||d_arlen!=15||
   d_araddr[2:0]!=0||{1'b0,d_araddr}<{1'b0,display_base}||
   {1'b0,d_araddr}+29'd128>{1'b0,display_base}+29'hfd200);
 wire changed_offer=state==OFFER&&(selected_display?
  (!d_arvalid||d_araddr!=held_addr||d_arlen!=held_len):
  (!s_arvalid||s_araddr!=held_addr||s_arlen!=held_len));
 wire changed_context=state!=IDLE&&(selected_display?
   (!display_lease_active||display_bank!=held_display_bank||display_epoch!=held_display_epoch):
   (!source_lease_active||source_epoch!=held_source_epoch||(DUAL_SOURCE&&source_bank!=held_source_bank)));
 wire bad_return=rvalid&&(!return_context||current_beat>{1'b0,held_len}||
                        rlast!=(current_beat=={1'b0,held_len}));
 task fault(input [7:0] code);begin error<=1;if(!error)error_code<=code;end endtask
 always @(posedge clk)begin
  if(!rst_n)begin
   state<=IDLE;selected_display<=0;urgent<=1;display_gap_waited<=0;held_addr<=0;held_len<=0;
   held_source_epoch<=0;held_source_bank<=0;held_display_epoch<=0;held_display_bank<=0;beat<=0;
   swait<=0;dwait<=0;source_wait_max<=0;display_wait_max<=0;completed_bursts<=0;error<=0;error_code<=0;
  end else begin
   if(fifo_used<=128)urgent<=1;else if(fifo_used>=384)urgent<=0;
   if(s_arvalid&&!s_arready)begin if(swait!=32'hffffffff)swait<=swait+1'b1;end
   else begin if(swait>source_wait_max)source_wait_max<=swait;swait<=0;end
   if(d_arvalid&&!d_arready)begin if(dwait!=32'hffffffff)dwait<=dwait+1'b1;end
   else begin if(dwait>display_wait_max)display_wait_max<=dwait;dwait<=0;end
   if(state==IDLE&&!error&&!source_bad&&!display_bad&&!bad_return&&(s_candidate||d_candidate))begin
    // The live reader has a WAIT_ROOM clock between RLAST and its next AR.
    // Without this bounded grace, a queued source steals every inter-burst
    // gap even at low display water level. One clock admits the real next AR;
    // if no AR follows (frame tail), source service resumes, never starvation.
    if(!d_candidate&&s_candidate&&display_lease_active&&display_priority&&!display_gap_waited)
     display_gap_waited<=1;
    else begin
     display_gap_waited<=0;
     selected_display<=pick_display;held_addr<=pick_display?d_araddr:s_araddr;
     held_len<=pick_display?d_arlen:s_arlen;held_source_epoch<=source_epoch;
     held_source_bank<=source_bank;
     held_display_epoch<=display_epoch;held_display_bank<=display_bank;state<=OFFER;
    end
   end
   else display_gap_waited<=0;
   if(take)begin state<=RETURN;beat<=0;end
   if(rvalid&&return_context)begin
    // Protocol failure never becomes a successful completion/drain event.
    if(!bad_return)begin
     if(rlast)begin state<=IDLE;beat<=0;completed_bursts<=completed_bursts+1'b1;end
     else begin state<=RETURN;beat<=current_beat+1'b1;end
    end
   end
   if(source_bad||display_bad)fault(8'h91);
   if(changed_offer||changed_context)fault(8'h92);
   if(bad_return)fault(8'h93);
  end
 end
endmodule
