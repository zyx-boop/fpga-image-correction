// IP WREADY accepts one beat, returned WLAST acknowledges its completion.
// D70: held_word is the offered transaction; shift may assemble one next Word
// while it drains. No extra DDR request is issued before the real prior ACK.
module corrected_frame_writer #(
 parameter integer FRAME_PIXELS=2073600,WATCHDOG_CYCLES=125000000,
 parameter integer H_MIRROR=0,FRAME_WIDTH=1920
)(
 input wire clk,rst_n,start,abort_req,input wire [1:0] target_bank,
 input wire pixel_valid,output wire pixel_ready,input wire [15:0] pixel,input wire pixel_eof,
 output wire [27:0] awaddr,output wire [3:0] awlen,output wire awvalid,input wire awready,
 output wire [255:0] wdata,output wire [31:0] wstrb,input wire wready,wlast,
 output wire quiet,output reg write_inflight,output wire done,
 output reg error,output reg [7:0] error_code,
 output reg [21:0] pixel_count,output reg [17:0] word_ack_count,output reg [27:0] next_addr,
 output reg ack_count_complete
);
 // Mirror only the completed image. Sequential logical progress is retained
 // for frame completion guards; physical writes reverse Words and RGB565 lanes.
 localparam integer ROW_WORDS=FRAME_WIDTH/16;
 reg [27:0] logical_awaddr,mirror_addr;
 reg [10:0] mirror_column;
 assign awaddr=H_MIRROR?mirror_addr:logical_awaddr;
 generate if(H_MIRROR)begin:g_mirror
  genvar k;
  for(k=0;k<16;k=k+1)begin:g_lane
   assign wdata[k*16+:16]=held_word[(15-k)*16+:16];
  end
 end else begin:g_direct
  assign wdata=held_word;
 end endgenerate
`ifndef SYNTHESIS
 initial if(H_MIRROR && (FRAME_WIDTH<16 || FRAME_WIDTH%16!=0 || FRAME_PIXELS%FRAME_WIDTH!=0))
  $fatal(1,"Mirror requires complete rows aligned to 16 pixels");
`endif
 localparam WORDS=FRAME_PIXELS/16,COLLECT=0,AW=1,DATA=2,ACK=3,FINISHED=4,STOPPED=5;
 reg [2:0] state;reg opened,aborted,eof_seen,assembly_full;
 reg [3:0] lane;reg [255:0] shift,held_word;reg [31:0] stall;
 assign pixel_ready=opened&&state<FINISHED&&!assembly_full&&pixel_count<FRAME_PIXELS&&!error&&!abort_req;
 assign awvalid=state==AW;assign awlen=0;assign wstrb=32'hffffffff;
 assign done=state==FINISHED&&!error;
 assign quiet=!write_inflight && ((state==COLLECT&&lane==0&&!assembly_full)||state==FINISHED||state==STOPPED);
 task fault(input [7:0] code);begin error<=1;if(!error)error_code<=code;end endtask
 task complete_word;
 begin
  write_inflight<=0;word_ack_count<=word_ack_count+1'b1;next_addr<=next_addr+28'd8;
  if(H_MIRROR)begin
   if(mirror_column==ROW_WORDS-1)begin
    mirror_column<=0;mirror_addr<=mirror_addr+(FRAME_WIDTH-8);
   end else begin mirror_column<=mirror_column+1'b1;mirror_addr<=mirror_addr-28'd8;end
  end
  // Exact shadow of the full genuine-ACK count, updated on that same edge.
  // No delayed completion certificate and no speculative ACK is introduced.
  ack_count_complete<=(WORDS>=0&&WORDS<262144)&&(word_ack_count==((WORDS-1)&18'h3ffff));
  if(aborted||abort_req||error)begin assembly_full<=0;lane<=0;state<=STOPPED;end
  else if(word_ack_count==WORDS-1)begin
   if(pixel_count==FRAME_PIXELS && eof_seen)state<=FINISHED;
   else begin fault(8'h22);state<=STOPPED;end
  end else if(assembly_full)begin
   // Reuse the existing assembly bank only after its complete Word is copied.
   assembly_full<=0;logical_awaddr<=next_addr+28'd8;state<=AW;
  end else state<=COLLECT;
 end endtask
 always @(posedge clk)begin
  if(!rst_n)begin
   state<=COLLECT;opened<=0;aborted<=0;eof_seen<=0;assembly_full<=0;lane<=0;
   shift<=0;stall<=0;logical_awaddr<=0;next_addr<=0;mirror_addr<=0;mirror_column<=0;
   pixel_count<=0;word_ack_count<=0;write_inflight<=0;error<=0;error_code<=0;
   ack_count_complete<=(WORDS==0);
  end else begin
   if(start)begin
    if(opened||!quiet||target_bank>2)begin fault(8'h21);end
    else begin
     opened<=1;next_addr<=target_bank==0?28'h0:target_bank==1?28'hfd200:28'h1fa400;
     mirror_addr<=(target_bank==0?28'h0:target_bank==1?28'hfd200:28'h1fa400)+(FRAME_WIDTH/2-8);
     mirror_column<=0;
    end
   end
   if(abort_req)begin aborted<=1;assembly_full<=0;lane<=0;fault(8'h01);end
   if((state==AW||state==DATA||state==ACK)&&
      !(awvalid&&awready)&&!wready&&!wlast)begin
    if(stall>=WATCHDOG_CYCLES-1)begin aborted<=1;fault(8'h23);end else stall<=stall+1'b1;
   end else stall<=0;
   // Collection is independent of AW/DATA/ACK. The offered Word/address stay
   // held until WLAST; a full unoffered assembly Word applies backpressure.
   if(!aborted&&!abort_req&&!error&&pixel_valid&&pixel_ready)begin
      if(pixel_eof!=(pixel_count==FRAME_PIXELS-1)||pixel_count>=FRAME_PIXELS)begin fault(8'h24);end
      if(pixel_eof)eof_seen<=1;
      pixel_count<=pixel_count+1'b1;shift<={pixel,shift[255:16]};
      if(lane==15)begin
       lane<=0;
       if(state==COLLECT)begin logical_awaddr<=next_addr;state<=AW;end
       else assembly_full<=1;
      end
      else lane<=lane+1'b1;
   end
   case(state)
    COLLECT:begin
     if(aborted||abort_req||error)begin assembly_full<=0;lane<=0;state<=STOPPED;end
     else if(assembly_full)begin assembly_full<=0;logical_awaddr<=next_addr;state<=AW;end
    end
    AW:if(awready)begin
     write_inflight<=1;
     if(wready)begin if(wlast)complete_word();else state<=ACK;end else state<=DATA;
    end
    DATA:if(wready)begin if(wlast)complete_word();else state<=ACK;end
    ACK:if(wlast)complete_word();
    FINISHED:state<=FINISHED;
    STOPPED:state<=STOPPED;
    default:begin fault(8'h2f);state<=STOPPED;end
   endcase
   if(wlast && !((state==ACK)||(state==DATA&&wready)||(state==AW&&awready&&wready)))begin
    fault(8'h25);aborted<=1;
   end
  end
 end
 // D138: payload registers observe unoffered data independently of the
 // global fault cone. Only the unchanged control FSM can publish a new AW.
 // An offered/inflight Word stays immutable until its genuine completion.
 wire payload_retire=wlast&&((state==AW&&awready&&wready)||
                            (state==DATA&&wready)||(state==ACK));
 always @(posedge clk)begin
  if(!rst_n)held_word<=0;
  else if(state==COLLECT)begin
   if(assembly_full)held_word<=shift;
   else held_word<={pixel,shift[255:16]};
  end else if(payload_retire&&assembly_full)held_word<=shift;
 end
endmodule
