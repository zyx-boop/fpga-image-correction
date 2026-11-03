// Camera uses up to four buffered Words; Target remains a single Word.
// The external AW Owner stays locked through the real WLAST completion.
module ddr_write_arbiter(
 input wire clk,rst_n,abort_req,input wire [9:0] camera_fifo_used,
 input wire c_active,input wire [1:0] c_bank,
 input wire c_awvalid,output wire c_awready,input wire [27:0] c_awaddr,input wire [3:0] c_awlen,
 input wire [255:0] c_wdata,input wire [31:0] c_wstrb,output wire c_wready,c_wlast,
 input wire t_active,input wire [1:0] t_bank,
 input wire t_awvalid,output wire t_awready,input wire [27:0] t_awaddr,input wire [3:0] t_awlen,
 input wire [255:0] t_wdata,input wire [31:0] t_wstrb,output wire t_wready,t_wlast,
 output wire awvalid,input wire awready,output wire [27:0] awaddr,output wire [3:0] awlen,
 output wire [255:0] wdata,output wire [31:0] wstrb,input wire wready,wlast,
 output wire quiet,output reg error,output reg [7:0] error_code
);
 localparam IDLE=0,OFFER=1,DATA=2,ACK=3;
 reg [1:0] state,camera_run;reg owner,last_owner,urgent,stopped;
 reg [3:0] held_len;
 reg [4:0] beats_left;
 reg [27:0] held_addr;reg [1:0] held_bank;
 reg c_bank_ok,t_bank_ok;reg [27:0] c_base,t_base;
 always_comb begin
  c_bank_ok=1;c_base=0;
  case(c_bank) 0:c_base=0;3:c_base=28'h2f7600;default:begin c_bank_ok=0;c_base=0;end endcase
  t_bank_ok=1;t_base=0;
  case(t_bank) 1:t_base=28'hfd200;2:t_base=28'h1fa400;default:begin t_bank_ok=0;t_base=0;end endcase
 end
 wire [8:0] c_span=({5'd0,c_awlen}+9'd1)<<3;
 wire c_bad=c_awvalid&&(!c_active||!c_bank_ok||c_awlen>3||c_awaddr[2:0]!=0||
  {1'b0,c_awaddr}<{1'b0,c_base}||{1'b0,c_awaddr}+c_span>{1'b0,c_base}+29'hfd200||
  {1'b0,c_awaddr[9:0]}+c_span>11'd1024);
 wire t_bad=t_awvalid&&(!t_active||!t_bank_ok||t_awlen!=0||t_awaddr[2:0]!=0||
  {1'b0,t_awaddr}<{1'b0,t_base}||{1'b0,t_awaddr}+29'd8>{1'b0,t_base}+29'hfd200);
 wire pick_camera=c_awvalid&&(!t_awvalid||(urgent?camera_run<3:last_owner));
 wire selected_valid=owner?t_awvalid:c_awvalid;
 wire [27:0] selected_addr=owner?t_awaddr:c_awaddr;
 wire [3:0] selected_len=owner?t_awlen:c_awlen;
 wire [1:0] selected_bank=owner?t_bank:c_bank;
 wire selected_active=owner?t_active:c_active;
 wire changed_offer=state==OFFER&&(!selected_valid||selected_addr!=held_addr||selected_len!=held_len);
 wire changed_owner=state!=IDLE&&(!selected_active||selected_bank!=held_bank);
 assign awvalid=state==OFFER;assign awaddr=held_addr;assign awlen=held_len;
 assign wdata=owner?t_wdata:c_wdata;assign wstrb=owner?t_wstrb:c_wstrb;
 assign c_awready=state==OFFER&&!owner&&awready;
 assign t_awready=state==OFFER&&owner&&awready;
 wire aw_take=awvalid&&awready;
 wire data_phase=state==DATA||aw_take;
 assign c_wready=data_phase&&!owner&&wready;
 assign t_wready=data_phase&&owner&&wready;
 // Route actual completion even when invalid, so client and arbiter both see it.
 assign c_wlast=state!=IDLE&&!owner&&wlast;
 assign t_wlast=state!=IDLE&&owner&&wlast;
 wire last_data=data_phase&&wready&&beats_left==1;
 wire ack_legal=state==ACK||last_data;
 wire bad_ack=wlast&&!ack_legal;
 // Unselected client offers remain held on a fatal stop; never pretend quiet.
 assign quiet=state==IDLE&&!c_awvalid&&!t_awvalid&&!wlast;
 task fault(input[7:0]code);begin error<=1;stopped<=1;if(!error)error_code<=code;end endtask
 always @(posedge clk)begin
  if(!rst_n)begin state<=IDLE;owner<=0;last_owner<=1;urgent<=0;stopped<=0;camera_run<=0;held_addr<=0;held_bank<=0;held_len<=0;beats_left<=0;error<=0;error_code<=0;end
  else begin
   if(camera_fifo_used>=384)urgent<=1;else if(camera_fifo_used<=128)urgent<=0;
   if(abort_req)stopped<=1;
   if(state==IDLE&&!stopped&&!abort_req&&!error&&!c_bad&&!t_bad&&!bad_ack&&(c_awvalid||t_awvalid))begin
    owner<=!pick_camera;held_addr<=pick_camera?c_awaddr:t_awaddr;held_bank<=pick_camera?c_bank:t_bank;
    held_len<=pick_camera?c_awlen:t_awlen;beats_left<={1'b0,(pick_camera?c_awlen:t_awlen)}+5'd1;state<=OFFER;
   end
   if(aw_take)state<=last_data?ACK:DATA;
   if(data_phase&&wready)begin beats_left<=beats_left-1'b1;if(last_data)state<=ACK;end
   if(wlast&&ack_legal)begin
    state<=IDLE;last_owner<=owner;
    if(owner)camera_run<=0;else if(camera_run<3)camera_run<=camera_run+1'b1;
   end
   if(c_bad||t_bad)fault(8'hd1);
   if(changed_offer||changed_owner)fault(8'hd2);
   if(bad_ack)fault(8'hd3);
   if(state==ACK&&wready)fault(8'hd4);
  end
 end
`ifndef SYNTHESIS
 reg [255:0] offered_data;reg [31:0] offered_strb;reg stalled;
 always @(posedge clk)begin
  if(!rst_n)begin offered_data<=0;offered_strb<=0;stalled<=0;end
  else begin
   if(stalled&&(wdata!==offered_data||wstrb!==offered_strb))$fatal(1,"D170_DDR_WRITE_STALLED_PAYLOAD_CHANGED");
   stalled<=(state==OFFER||state==DATA)&&!(data_phase&&wready);
   offered_data<=wdata;offered_strb<=wstrb;
  end
 end
`endif
endmodule
