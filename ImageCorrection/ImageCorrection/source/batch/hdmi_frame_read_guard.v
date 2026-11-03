// Bank stays fixed until real DDR/FIFO/two-Word consumer drain is certified.
module hdmi_frame_read_guard #(
 parameter integer FRAME_WIDTH=1920,FRAME_HEIGHT=1080,FIFO_ADDR_BITS=9,
 parameter integer BANK_ADDR_UNITS=FRAME_WIDTH*FRAME_HEIGHT/2,
 parameter integer WATCHDOG_CYCLES=125000000,
 parameter USE_LAUNCH_ENABLE=0
)(
 input wire clk,pixel_clk,rst_n,vblank,display_valid,launch_enable,input wire [1:0] requested_bank,
 input wire [FIFO_ADDR_BITS:0] fifo_used,input wire fifo_full,
 input wire consumer_quiet_pix,consumer_error_pix,
 input wire [21:0] consumer_pixels_pix,input wire [15:0] consumer_epoch_pix,
 output wire [27:0] araddr,output wire [3:0] arlen,output wire arvalid,input wire arready,
 input wire [255:0] rdata,input wire rvalid,rlast,
 output wire fifo_wr_en,output wire [255:0] fifo_wr_data,
 output reg epoch_quiet,error,output reg [7:0] error_code,
 output wire lease_active,output reg [15:0] display_epoch,
 output reg [1:0] reader_bank,output reg [17:0] words_received,output reg [31:0] defer_count,
 output wire reader_addr_error,output wire [15:0] reader_addr_error_count,
 output wire [27:0] reader_addr_actual,reader_addr_expected,output wire [3:0] reader_addr_code
);
 localparam PIXELS=FRAME_WIDTH*FRAME_HEIGHT,WORDS=PIXELS/16;
 wire reader_quiet,read_error;
 reg reader_start,launch_pending,have_frame;
 reg rd_inflight;reg [4:0] beat_index;reg [3:0] held_len;
 reg ar_waiting;reg [27:0] held_addr;
 wire ar_hs=arvalid&&arready;
 wire [4:0] current_beat=rd_inflight?beat_index:5'd0;
 wire [3:0] current_len=rd_inflight?held_len:arlen;
 assign lease_active=have_frame&&!epoch_quiet;
 reg request_toggle,snapshot_busy;
 (* ASYNC_REG="TRUE" *) reg [1:0] request_pix_sync;
 reg ack_pix;reg [39:0] held_snapshot_pix;
 (* ASYNC_REG="TRUE" *) reg [1:0] consumer_error_sync;
 reg [31:0] stall_cycles;
 reg [21:0] last_consumer_pixels;
 wire snapshot_progress=snapshot_busy&&ack_core_sync[2]==request_toggle&&
  held_snapshot_pix[37:22]==display_epoch&&held_snapshot_pix[21:0]!=last_consumer_pixels;
 wire watchdog_due=have_frame&&!epoch_quiet&&!error&&!ar_hs&&!rvalid&&!snapshot_progress&&
  stall_cycles>=WATCHDOG_CYCLES-1;
 wire stop_new_reads=error||consumer_error_sync[1]||watchdog_due||read_error||reader_addr_error;
 (* ASYNC_REG="TRUE" *) reg [2:0] ack_core_sync;
 (* ASYNC_REG="TRUE" *) reg [1:0] pixel_reset_sync;
 always @(posedge pixel_clk or negedge rst_n)begin
  if(!rst_n)pixel_reset_sync<=0;else pixel_reset_sync<={pixel_reset_sync[0],1'b1};
 end
 always @(posedge pixel_clk or negedge pixel_reset_sync[1])begin
  if(!pixel_reset_sync[1])begin request_pix_sync<=0;ack_pix<=0;held_snapshot_pix<=0;end
  else begin
   request_pix_sync<={request_pix_sync[0],request_toggle};
   if(request_pix_sync[1]!=ack_pix)begin
    held_snapshot_pix<={consumer_error_pix,consumer_quiet_pix,consumer_epoch_pix,consumer_pixels_pix};
    ack_pix<=request_pix_sync[1];
   end
  end
 end
 ddr_live_reader #(.FRAME_WIDTH(FRAME_WIDTH),.FRAME_HEIGHT(FRAME_HEIGHT),.FIFO_ADDR_BITS(FIFO_ADDR_BITS),
  .BANK_ADDR_UNITS(BANK_ADDR_UNITS),.STOP_ON_DISABLE(1)) u_reader(
 .clk(clk),.rst_n(rst_n),.vblank(reader_start),
 .display_valid((display_valid||(USE_LAUNCH_ENABLE&&have_frame&&!epoch_quiet))&&!stop_new_reads),.display_bank(reader_bank),
 .fifo_used(fifo_used),.fifo_full(fifo_full),.araddr(araddr),.arlen(arlen),.arvalid(arvalid),
 .arready(arready),.rdata(rdata),.rvalid(rvalid),.rlast(rlast),
 .fifo_wr_en(fifo_wr_en),.fifo_wr_data(fifo_wr_data),.read_error(read_error),
 .addr_error(reader_addr_error),.addr_error_count(reader_addr_error_count),
 .addr_error_actual(reader_addr_actual),.addr_error_expected(reader_addr_expected),
 .addr_error_code(reader_addr_code),.reader_quiet(reader_quiet));
 task fault(input [7:0] code);begin error<=1;if(!error)error_code<=code;epoch_quiet<=0;end endtask
 always @(posedge clk)begin
  if(!rst_n)begin
   epoch_quiet<=0;error<=0;error_code<=0;display_epoch<=0;reader_bank<=0;
   words_received<=0;defer_count<=0;reader_start<=0;launch_pending<=0;have_frame<=0;
   rd_inflight<=0;beat_index<=0;held_len<=0;ar_waiting<=0;held_addr<=0;
   request_toggle<=0;snapshot_busy<=0;ack_core_sync<=0;
   consumer_error_sync<=0;stall_cycles<=0;last_consumer_pixels<=0;
  end else begin
   reader_start<=0;ack_core_sync<={ack_core_sync[1:0],ack_pix};
   // Sticky single-bit underflow crosses independently: waiting for a full
   // DDR/FIFO drain must not hide a display failure or a permanent read stall.
   consumer_error_sync<={consumer_error_sync[0],consumer_error_pix};
   if(consumer_error_sync[1])fault(8'h41);
   if(!have_frame||epoch_quiet||error)stall_cycles<=0;
   else if(ar_hs||rvalid||snapshot_progress)
      stall_cycles<=0;
   else if(stall_cycles>=WATCHDOG_CYCLES-1)fault(8'h48);
   else stall_cycles<=stall_cycles+1'b1;
   if(!snapshot_busy&&!epoch_quiet&&!error&&!launch_pending&&!reader_start&&reader_quiet&&
      !arvalid&&!rd_inflight&&!rvalid&&!fifo_wr_en&&fifo_used==0&&(!have_frame||words_received==WORDS))begin
    request_toggle<=~request_toggle;snapshot_busy<=1;
   end
   if(snapshot_busy&&ack_core_sync[2]==request_toggle)begin
    snapshot_busy<=0;
    if(held_snapshot_pix[37:22]==display_epoch)last_consumer_pixels<=held_snapshot_pix[21:0];
    if(held_snapshot_pix[39])fault(8'h41);
    else if(held_snapshot_pix[38]&&held_snapshot_pix[37:22]==display_epoch&&
      held_snapshot_pix[21:0]==(have_frame?PIXELS:0)&&reader_quiet&&fifo_used==0&&!rd_inflight)
       epoch_quiet<=1;
   end
   if(vblank&&display_valid&&(!USE_LAUNCH_ENABLE||launch_enable)&&!error)begin
    if(epoch_quiet&&!snapshot_busy&&reader_quiet&&fifo_used==0&&!rd_inflight&&!arvalid)begin
     launch_pending<=1;epoch_quiet<=0;
    end else defer_count<=defer_count+1'b1;
   end
   // One clock after vblank: Bank manager's same-edge safe A/B update is visible.
   if(launch_pending&&!error)begin
    if(requested_bank>2||!reader_quiet||fifo_used!=0||rd_inflight)fault(8'h42);
    else begin reader_bank<=requested_bank;reader_start<=1;display_epoch<=display_epoch+1'b1;
     words_received<=0;have_frame<=1;stall_cycles<=0;last_consumer_pixels<=0;end
    launch_pending<=0;
   end
   if(ar_waiting&&(!arvalid||araddr!=held_addr))fault(8'h43);
   if(arvalid&&!arready&&!ar_waiting)begin ar_waiting<=1;held_addr<=araddr;end
   if(ar_hs)begin
    ar_waiting<=0;
    if(rd_inflight||!lease_active)fault(8'h44);
    rd_inflight<=1;beat_index<=0;held_len<=arlen;
   end
   if(rvalid)begin
    if(!(rd_inflight||ar_hs)||current_beat>current_len||rlast!=(current_beat==current_len))fault(8'h45);
    else begin
     words_received<=words_received+1'b1;
     if(words_received>=WORDS)fault(8'h46);
     if(rlast)begin rd_inflight<=0;beat_index<=0;end else beat_index<=current_beat+1'b1;
    end
   end
   if(read_error||reader_addr_error)fault(8'h47);
   if(error)epoch_quiet<=0;
  end
 end
endmodule
