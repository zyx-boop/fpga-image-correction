// B3: accept into P0, issue its registered Bank permits next edge.
// Original B1 RAM/response stages retained; Reader samples response at E0+3.
module row_ring_compact #(parameter integer ROWS=128)(
 input wire clk,reset_n,wr_begin,wr_valid,rd_valid,
 input wire [10:0] wr_y,rd_y,rd_x,input wire [31:0] wr_epoch,rd_epoch,
 input wire [6:0] wr_word,input wire [255:0] wr_data,
 output wire wr_begin_ready,wr_ready,rd_ready,
 output reg rsp_valid,rsp_hit,error,output wire [15:0] rsp_data,
  output reg [10:0] rsp_x,rsp_y,output reg [31:0] rsp_epoch
 );
 localparam integer BANKS=(ROWS*120+1023)/1024;
 function [6:0] slot(input [10:0] y);begin
   if(ROWS==128)slot=y[6:0];else slot=y%ROWS;
 end endfunction
 function [13:0] base(input [6:0] s);begin base=({7'b0,s}<<7)-({7'b0,s}<<3);end endfunction
 wire [6:0] ws=slot(wr_y),rs=slot(rd_y);
 reg active;reg [6:0] active_slot,next_word,pending_slot,response_slot;
 reg [13:0] active_base;
 // One-hot bank for the current accepted write position. Predecode at row
 // begin, then advance only on an accepted word crossing a 1024-word bank.
 reg [BANKS-1:0] write_bank /* synthesis syn_preserve=1 */;
 wire [13:0] begin_base=base(ws);
 always @(posedge clk)begin
   if(!reset_n)write_bank<=0;
   else if(wr_begin&&wr_begin_ready)write_bank<=({{(BANKS-1){1'b0}},1'b1} << begin_base[13:10]);
   else if(take_wr&&next_word!=119&&wa[9:0]==10'd1023)write_bank<=write_bank << 1;
 end
 reg [ROWS-1:0] row_valid;
 reg [42:0] tags[0:ROWS-1] /* synthesis syn_ramstyle="block_ram" */;
 reg [42:0] tag_q;
 reg pending,valid_q,bounds_q;reg [3:0] bank_q;
 reg [10:0] x_q,y_q;reg [31:0] epoch_q;
 reg p0_valid,p0_row_valid,p0_bounds;
 reg [10:0] p0_x,p0_y;reg [31:0] p0_epoch;
 reg [13:0] p0_address;
 reg [BANKS-1:0] p0_bank_enable;
 wire [6:0] p0_slot=slot(p0_y);
 assign wr_begin_ready=!active&&!(p0_valid&&p0_slot==ws)&&!(pending&&pending_slot==ws)&&!(rsp_valid&&response_slot==ws)&&wr_y<1080&&!error;
 assign rd_ready=!(active&&active_slot==rs)&&!(wr_begin&&ws==rs)&&!error;
 assign wr_ready=active&&!error;
 wire take_rd=rd_valid&&rd_ready;
 wire take_wr=wr_valid&&wr_ready&&wr_word==next_word;
 wire [13:0] wa=active_base+next_word;
 wire [13:0] ra=base(rs)+rd_x[10:4];
 wire hit_q=valid_q&&bounds_q&&tag_q=={epoch_q,y_q};
 wire [BANKS*256-1:0] words_by_bank;
 wire [BANKS*16-1:0] bank_responses;
 genvar l,b,t;
 // Distinct registered Bank predicates, not copies of one global read CE.
 generate for(b=0;b<BANKS;b=b+1)begin:g_p0_bank
   always @(posedge clk)begin
     if(!reset_n)p0_bank_enable[b]<=0;
     else p0_bank_enable[b]<=take_rd&&ra[13:10]==b;
   end
 end endgenerate
 // Original memory dimensions/write template; read via OLD P0, never live input.
 generate for(l=0;l<16;l=l+1)begin:g_lane
   for(b=0;b<BANKS;b=b+1)begin:g_bank
     reg [15:0] memory[0:1023] /* synthesis syn_ramstyle="block_ram" */;
     reg [15:0] q;
     always @(posedge clk)begin
       if(take_wr&&write_bank[b])memory[wa[9:0]]<=wr_data[l*16+:16];
       if(p0_bank_enable[b])q<=memory[p0_address[9:0]];
     end
     assign words_by_bank[b*256+l*16+:16]=q;
   end
 end endgenerate
 // Stage 2 is distributed, not followed by another data register.
 // A pending miss clears all contributions. A bubble holds all contributions,
 // exactly preserving the former rsp_data's invalid-cycle hold behavior.
 generate for(b=0;b<BANKS;b=b+1)begin:g_response_bank
   wire [255:0] local_words=words_by_bank[b*256+:256];
   reg [15:0] response_q;
   always @(posedge clk)begin
     if(!reset_n)response_q<=0;
     else if(pending)
       response_q<=(hit_q&&bank_q==b)?local_words[x_q[3:0]*16+:16]:16'b0;
   end
   assign bank_responses[b*16+:16]=response_q;
 end endgenerate
 // Balanced combinational collection of registered, mutually exclusive banks.
 // 128 rows -> BANKS=15. Padding leaf 15 is a constant, not an extra RAM bank.
 localparam integer LEAVES=1<<$clog2(BANKS);
 wire [15:0] collected[1:2*LEAVES-1];
 generate for(t=0;t<LEAVES;t=t+1)begin:g_leaf
   if(t<BANKS)assign collected[LEAVES+t]=bank_responses[t*16+:16];
   else assign collected[LEAVES+t]=16'b0;
 end
 for(t=1;t<LEAVES;t=t+1)begin:g_or
   assign collected[t]=collected[2*t]|collected[2*t+1];
 end endgenerate
 assign rsp_data=collected[1];
 always @(posedge clk)begin
   if(wr_begin&&wr_begin_ready)tags[ws]<={wr_epoch,wr_y};
   if(p0_valid)tag_q<=tags[p0_slot];
 end
 always @(posedge clk)begin
   if(!reset_n)begin
     active<=0;active_slot<=0;active_base<=0;next_word<=0;row_valid<=0;error<=0;
     p0_valid<=0;p0_row_valid<=0;p0_bounds<=0;p0_x<=0;p0_y<=0;p0_epoch<=0;p0_address<=0;
     pending<=0;valid_q<=0;bounds_q<=0;bank_q<=0;pending_slot<=0;response_slot<=0;x_q<=0;y_q<=0;epoch_q<=0;
     rsp_valid<=0;rsp_hit<=0;rsp_x<=0;rsp_y<=0;rsp_epoch<=0;
   end else begin
     // Formal acceptance remains take_rd; no abort/close cancels this register.
     p0_valid<=take_rd;
     if(take_rd)begin
       p0_x<=rd_x;p0_y<=rd_y;p0_epoch<=rd_epoch;p0_address<=ra;
       p0_row_valid<=row_valid[rs];p0_bounds<=rd_x<1920&&rd_y<1080;
     end
     pending<=p0_valid;
     if(p0_valid)begin
       x_q<=p0_x;y_q<=p0_y;epoch_q<=p0_epoch;pending_slot<=p0_slot;
       bank_q<=p0_address[13:10];valid_q<=p0_row_valid;bounds_q<=p0_bounds;
     end
     rsp_valid<=pending;
     if(pending)begin
       rsp_x<=x_q;rsp_y<=y_q;rsp_epoch<=epoch_q;response_slot<=pending_slot;
       rsp_hit<=hit_q;
     end
     if(wr_begin&&wr_begin_ready)begin
       active<=1;active_slot<=ws;active_base<=base(ws);next_word<=0;row_valid[ws]<=0;
     end
     if(wr_valid&&wr_ready)begin
       if(wr_word!=next_word)error<=1;
       else if(next_word==119)begin row_valid[active_slot]<=1;active<=0;end
       else next_word<=next_word+1'b1;
     end
   end
 end
endmodule
