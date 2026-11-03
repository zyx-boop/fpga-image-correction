// One128-byte block and one DDR request. No RREADY: always catch all four beats.
module source_pixel_reader #(parameter integer WATCHDOG_CYCLES=125000000)(
 input wire clk,rst_n,context_start,abort_req,allow_new_ar,
 input wire [1:0] source_bank,
 input wire request_valid,output wire request_ready,
 input wire signed [12:0] src_x,src_y,
 output wire pixel_valid,input wire pixel_ready,output reg [15:0] pixel,
 output reg [27:0] araddr,output wire [3:0] arlen,output wire arvalid,
 input wire arready,input wire [255:0] rdata,input wire rvalid,rlast,
 output wire quiet,output reg rd_inflight,output wire return_pending,
 output reg error,output reg [7:0] error_code,output reg [31:0] hit_count,miss_count
);
 localparam IDLE=0,ROW=1,ADDR=2,TAG=3,AR=4,RECV=5,COMMIT=6,WORD=7,LANE=8,OFFER=9;
 reg [3:0] state;
 reg opened,aborted,quarantine,tag_valid;
 reg [27:0] source_base,row_units,block_addr,tag_addr;
 reg [10:0] x,y;reg [2:0] beat_count;
 reg [255:0] block0,block1,block2,block3,selected_word;
 reg [31:0] stall;
 wire ar_hs=arvalid&&arready;
 wire take_r=rvalid&&(rd_inflight||ar_hs);
 wire [2:0] beat=rd_inflight?beat_count:3'd0;
 assign arvalid=(state==AR);
 assign arlen=4'd3;
 assign request_ready=opened&&state==IDLE&&!error&&!abort_req;
 assign pixel_valid=state==OFFER;
 assign return_pending=(state==COMMIT||state==WORD||state==LANE||state==OFFER);
 assign quiet=(state==IDLE)&&!rd_inflight&&!rvalid&&!quarantine;
 wire waiting=arvalid||rd_inflight;
 wire forward_event=ar_hs||take_r;
 task fault(input [7:0] code);
 begin error<=1;if(!error)error_code<=code;end endtask
 always @(posedge clk)begin
  if(!rst_n)begin
   state<=IDLE;opened<=0;aborted<=0;quarantine<=0;tag_valid<=0;
   source_base<=0;row_units<=0;block_addr<=0;tag_addr<=0;araddr<=0;
   x<=0;y<=0;beat_count<=0;rd_inflight<=0;pixel<=0;stall<=0;
   error<=0;error_code<=0;hit_count<=0;miss_count<=0;selected_word<=0;
  end else begin
   if(context_start)begin
    if(!quiet||opened)begin fault(8'h11);quarantine<=1;end
    else begin opened<=1;tag_valid<=0;
     source_base<=source_bank==0?28'h0:source_bank==1?28'hfd200:28'h1fa400;
     if(source_bank>2)begin fault(8'h12);quarantine<=1;end
    end
   end
   if(abort_req)begin aborted<=1;fault(8'h01);end
   if(waiting&&!forward_event)begin
    if(stall>=WATCHDOG_CYCLES-1)begin aborted<=1;fault(8'h13);end else stall<=stall+1'b1;
   end else stall<=0;
   case(state)
    IDLE:if(request_valid&&request_ready)begin
     if(src_x<0||src_x>=1920||src_y<0||src_y>=1080)begin pixel<=0;state<=OFFER;end
     else begin x<=src_x[10:0];y<=src_y[10:0];state<=ROW;end
    end
    ROW:begin row_units<=y*28'd960;state<=ADDR;end
    ADDR:begin block_addr<=source_base+row_units+{x[10:6],5'b0};state<=TAG;end
    TAG:if(tag_valid&&tag_addr==block_addr)begin hit_count<=hit_count+1'b1;state<=WORD;end
     else if(allow_new_ar)begin
      if(block_addr<source_base||block_addr+28'd32>source_base+28'hfd200)begin fault(8'h14);quarantine<=1;state<=IDLE;end
      else begin araddr<=block_addr;tag_valid<=0;miss_count<=miss_count+1'b1;state<=AR;end
     end
    AR:if(ar_hs)begin rd_inflight<=1;beat_count<=0;state<=RECV;end
    RECV:state<=RECV;
    COMMIT:if(!error&&!aborted&&!abort_req)begin tag_addr<=araddr;tag_valid<=1;state<=WORD;end else state<=IDLE;
    WORD:begin
     case(x[5:4])0:selected_word<=block0;1:selected_word<=block1;2:selected_word<=block2;3:selected_word<=block3;endcase
     state<=LANE;
    end
    LANE:begin pixel<=selected_word[x[3:0]*16+:16];state<=OFFER;end
    OFFER:if(pixel_ready)state<=IDLE;
    default:begin fault(8'h1f);quarantine<=1;state<=IDLE;end
   endcase
   if(take_r)begin
    if(beat>3||rlast!=(beat==3))begin fault(8'h15);quarantine<=1;end
    case(beat)0:block0<=rdata;1:block1<=rdata;2:block2<=rdata;3:block3<=rdata;default:begin end endcase
    if(beat_count<7)beat_count<=beat+1'b1;
    if(beat==3&&rlast)begin rd_inflight<=0;beat_count<=0;state<=COMMIT;end
   end else if(rvalid)begin fault(8'h16);quarantine<=1;end
   // Cancel only requests not yet offered. Offered pixel remains until take.
   if((aborted||abort_req||error) && state!=AR && state!=RECV && state!=OFFER)state<=IDLE;
  end
 end
endmodule
