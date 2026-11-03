// Demand cache with frame-fixed row reuse or original vertical layout.
// Row number is the index, horizontal burst number is the tag. Unlike the
// old modulo-128 whole-row cache, a vertical output scan cannot alias rows.
// Four native 256-bit beats, unchanged source lease and arbiter protocol.
module dynamic_burst_cache #(parameter WATCHDOG=125000000, ROW_REUSE=0)(
 input wire clk,rst,frame_open,frame_close,cancel,ctx_active,allow_new_ar,
 input wire prefer_rows,
 input wire [1:0] ctx_bank,input wire [31:0] ctx_fid,ctx_epoch,
 input wire req_valid,output wire req_ready,input wire [10:0] req_x,req_y,
 input wire [31:0] req_fid,req_epoch,
 output reg rsp_valid,output reg [15:0] rsp_data,output reg [255:0] rsp_word,
 output wire quiet,initialized,output reg fault,
 output reg [31:0] hits,misses,
 output wire arvalid,input wire arready,output reg [27:0] araddr,output wire [3:0] arlen,
 input wire rvalid,rlast,input wire [255:0] rdata
);
 localparam IDLE=0,CLEAR=1,LOOK=2,CHECK=3,PREP=4,OFFER=5,RECEIVE=6,READ=7,RESP=8,HALT=9;
 reg [3:0] state;
 reg open;reg [10:0] clear_y,y;reg [10:0] x;
 reg [1:0] beat;reg [31:0] watchdog;
 reg [5:0] tags[0:2047] /* synthesis syn_ramstyle="block_ram" */;
 reg [255:0] words[0:8191] /* synthesis syn_ramstyle="block_ram" */;
 reg [5:0] tag_q;reg [255:0] data_q;
 // Same 2048 blocks and same 8192 payload words in both layouts.
 // Horizontal frames retain 32 blocks across each of 64 source rows.
 // Other poses retain the R19 one-block-per-source-row layout.
 // The mode is captured only on the first request after a frame clear.
 reg row_layout,layout_locked;
 wire [10:0] cache_index=row_layout?{y[5:0],x[10:6]}:y;
 wire [5:0] wanted_tag=row_layout?{1'b1,y[10:6]}:{1'b1,x[10:6]};
 // Four-word neighbourhood buffer: parity separates both bilinear rows and
 // the two words straddling x=15/16. Consecutive pixels usually need no BRAM
 // lookup. Full tags prevent false hits for arbitrary rotations.
 reg [3:0] near_valid;
 reg [17:0] near_tag[0:3];
 reg [255:0] near_data[0:3];
 wire [1:0] near_index={req_y[0],req_x[4]};
 wire near_hit=near_valid[near_index]&&near_tag[near_index]=={req_y,req_x[10:4]};
 wire [255:0] near_word=near_data[near_index];
 wire take=req_valid&&req_ready;
 wire ar_take=arvalid&&arready;
 wire receiving=state==RECEIVE||ar_take;
 wire [1:0] return_beat=state==RECEIVE?beat:2'd0;
 wire bad_return=rvalid&&(!receiving||rlast!=(return_beat==3));
 assign req_ready=open&&state==IDLE&&!fault&&!rvalid;
 assign arvalid=state==OFFER;
 assign arlen=3;
 assign initialized=open&&state!=CLEAR;
 assign quiet=state==IDLE&&!rsp_valid&&!rvalid&&!fault;
 // Unconditional read registers avoid wide late-control enables on payloads.
 always @(posedge clk)begin
   tag_q<=tags[cache_index];
   data_q<=words[{cache_index,x[5:4]}];
   if(state==CLEAR)tags[clear_y]<=0;
   else if(rvalid&&receiving&&!bad_return&&!fault&&return_beat==3)
     tags[cache_index]<=wanted_tag;
   if(rvalid&&receiving&&!bad_return&&!fault)
     words[{cache_index,return_beat}]<=rdata;
 end
 always @(posedge clk)begin
  if(rst)begin
   state<=IDLE;open<=0;row_layout<=0;layout_locked<=0;clear_y<=0;y<=0;x<=0;beat<=0;
   rsp_valid<=0;rsp_data<=0;rsp_word<=0;araddr<=0;fault<=0;watchdog<=0;hits<=0;misses<=0;near_valid<=0;
  end else begin
   rsp_valid<=0;
   if(state==IDLE||state==CLEAR||ar_take||rvalid)watchdog<=0;
   else if(watchdog<WATCHDOG)watchdog<=watchdog+1'b1;
   if(frame_open)begin
    if(!quiet||open)begin fault<=1;state<=HALT;end
    else begin open<=1;row_layout<=0;layout_locked<=0;clear_y<=0;state<=CLEAR;hits<=0;misses<=0;near_valid<=0;end
   end else if(frame_close)begin
    if(!quiet)begin fault<=1;state<=HALT;end
    else open<=0;
   end else case(state)
    CLEAR:if(clear_y==2047)state<=IDLE;else clear_y<=clear_y+1'b1;
    IDLE:if(take)begin
     x<=req_x;y<=req_y;state<=LOOK;
     if(!layout_locked)begin row_layout<=ROW_REUSE?prefer_rows:1'b0;layout_locked<=1;end
     if(!ctx_active||req_fid!=ctx_fid||req_epoch!=ctx_epoch||req_x>=1920||req_y>=1080||
        (ctx_bank!=0&&ctx_bank!=3))begin fault<=1;state<=HALT;end
     else if(cancel||near_hit)begin
      rsp_valid<=1;rsp_data<=cancel?16'd0:near_word[req_x[3:0]*16+:16];rsp_word<=cancel?256'd0:near_word;state<=IDLE;
      if(!cancel)hits<=hits+1'b1;
     end
    end
    LOOK:state<=CHECK;
    CHECK:begin
     if(cancel)begin rsp_valid<=1;rsp_data<=0;rsp_word<=0;state<=IDLE;end
     else if(tag_q==wanted_tag)begin hits<=hits+1'b1;state<=RESP;end
     else begin
      misses<=misses+1'b1;
      // DDR address unit is four bytes: row*960 + burst*32.
      araddr<=(ctx_bank==3?28'h2f7600:28'd0)+({17'b0,y}<<10)-({17'b0,y}<<6)+{18'd0,x[10:6],5'd0};
      state<=PREP;
     end
    end
    PREP:if(cancel)begin rsp_valid<=1;rsp_data<=0;rsp_word<=0;state<=IDLE;end
         else if(allow_new_ar)state<=OFFER;
    OFFER:if(ar_take)begin state<=RECEIVE;beat<=rvalid?1:0;end
    RECEIVE:if(rvalid)begin
      if(return_beat==3)state<=fault||bad_return?HALT:READ;
      else beat<=beat+1'b1;
    end
    READ:state<=RESP;
    RESP:begin
     rsp_valid<=1;rsp_data<=cancel?16'd0:data_q[x[3:0]*16+:16];rsp_word<=cancel?256'd0:data_q;state<=IDLE;
     if(!cancel)begin near_valid[{y[0],x[4]}]<=1;near_tag[{y[0],x[4]}]<={y,x[10:4]};near_data[{y[0],x[4]}]<=data_q;end
    end
    default:begin end
   endcase
   if(bad_return||(watchdog==WATCHDOG&&state!=IDLE&&state!=CLEAR))begin
    fault<=1;
    // An offered request is irrevocable. Preserve OFFER/RECEIVE until drain.
    if(state!=OFFER&&state!=RECEIVE)state<=HALT;
   end
  end
 end
endmodule
