// Read-only metrics. Descriptors and first-DE events use held toggle/ACK CDC.
// Timestamp is core observation of capture SOF to core observation of first DE;
// CDC reception adds a documented few-clock uncertainty, never raw VSYNC FPS.
module continuous_metrics #(parameter WINDOW_CYCLES=125000000)(
 input wire core_clk,pixel_clk,rst_n,pixel_rst_n,startup_ready,frame_start_pix,pixel_valid_pix,
 input wire [15:0] scan_epoch,input wire [31:0] display_job,display_fid,
 input wire [63:0] display_sof_cycle,input wire [31:0] input_frame,publish_count,
 output reg [31:0] displayed_count,first_display_job,first_display_fid,
 output reg [63:0] latency_cycles,output reg [31:0] input_fps,output_fps,display_fps,
  output wire error
);
 reg [63:0] core_cycles;reg [31:0] timer,last_input,last_publish,last_display;
 reg [15:0] scan_seen;reg request,descriptor_busy,descriptor_ack;
 reg [127:0] descriptor_hold,descriptor_pix,event_hold;
 reg event_req,event_ack,pixel_first_pending;
 reg [31:0] last_pixel_job;reg core_error,pixel_error;
 // Snapshot the original event-cycle subtraction in four independent limbs.
 // Reporting is delayed two clocks; the measured interval is unchanged.
 reg [16:0] delta0,delta1,delta2,delta3;
 reg [63:0] delta_value;
 reg delta_underflow;
 reg [1:0] event_valid_pipe;
 reg [63:0] event_identity0,event_identity1;
 wire borrow0=delta0[16];
 wire borrow1=delta1[16]||((delta1[15:0]==0)&&borrow0);
 wire borrow2=delta2[16]||((delta2[15:0]==0)&&borrow1);
 wire borrow3=delta3[16]||((delta3[15:0]==0)&&borrow2);
 always @(posedge core_clk)begin
  delta0<={1'b0,core_cycles[15:0]}-{1'b0,event_hold[15:0]};
  delta1<={1'b0,core_cycles[31:16]}-{1'b0,event_hold[31:16]};
  delta2<={1'b0,core_cycles[47:32]}-{1'b0,event_hold[47:32]};
  delta3<={1'b0,core_cycles[63:48]}-{1'b0,event_hold[63:48]};
  delta_value<={delta3[15:0]-{15'd0,borrow2},delta2[15:0]-{15'd0,borrow1},
                 delta1[15:0]-{15'd0,borrow0},delta0[15:0]};
  delta_underflow<=borrow3;
  event_identity0<=event_hold[127:64];event_identity1<=event_identity0;
 end
 (* ASYNC_REG="TRUE" *) reg [1:0] req_pix_sync,ack_core_sync,event_core_sync,event_ack_pix_sync,pixel_error_sync;
 assign error=core_error||pixel_error_sync[1];
 always @(posedge core_clk or negedge rst_n)begin
  if(!rst_n)begin
   core_cycles<=0;timer<=0;last_input<=0;last_publish<=0;last_display<=0;
   scan_seen<=0;request<=0;descriptor_busy<=0;descriptor_hold<=0;ack_core_sync<=0;
   event_core_sync<=0;event_ack<=0;pixel_error_sync<=0;core_error<=0;
   event_valid_pipe<=0;
   displayed_count<=0;first_display_job<=0;first_display_fid<=0;latency_cycles<=0;input_fps<=0;output_fps<=0;display_fps<=0;
  end else begin
   core_cycles<=core_cycles+1'b1;
   ack_core_sync<={ack_core_sync[0],descriptor_ack};event_core_sync<={event_core_sync[0],event_req};
   pixel_error_sync<={pixel_error_sync[0],pixel_error};
   event_valid_pipe<={event_valid_pipe[0],event_core_sync[1]!=event_ack};
   if(descriptor_busy&&ack_core_sync[1]==request)descriptor_busy<=0;
   if(startup_ready&&scan_epoch!=scan_seen)begin
    scan_seen<=scan_epoch;
    if(descriptor_busy)core_error<=1;
    else begin descriptor_hold<={display_job,display_fid,display_sof_cycle};request<=~request;descriptor_busy<=1;end
   end
   if(event_core_sync[1]!=event_ack)begin
    event_ack<=event_core_sync[1];
   end
   if(event_valid_pipe[1])begin
    if(event_identity1[63:32]!=first_display_job+1'b1||delta_underflow)core_error<=1;
    else begin displayed_count<=displayed_count+1'b1;first_display_job<=event_identity1[63:32];first_display_fid<=event_identity1[31:0];
     latency_cycles<=delta_value;end
   end
   if(!startup_ready)timer<=0;
   else if(timer==WINDOW_CYCLES-1)begin
    timer<=0;input_fps<=input_frame-last_input;output_fps<=publish_count-last_publish;display_fps<=displayed_count-last_display;
    last_input<=input_frame;last_publish<=publish_count;last_display<=displayed_count;
   end else timer<=timer+1'b1;
  end
 end
 always @(posedge pixel_clk or negedge pixel_rst_n)begin
  if(!pixel_rst_n)begin req_pix_sync<=0;descriptor_ack<=0;descriptor_pix<=0;event_hold<=0;
   event_req<=0;event_ack_pix_sync<=0;pixel_first_pending<=0;last_pixel_job<=0;pixel_error<=0;end
  else begin
   req_pix_sync<={req_pix_sync[0],request};event_ack_pix_sync<={event_ack_pix_sync[0],event_ack};
   if(req_pix_sync[1]!=descriptor_ack)begin descriptor_pix<=descriptor_hold;descriptor_ack<=req_pix_sync[1];end
   if(frame_start_pix)pixel_first_pending<=1;
   if(pixel_first_pending&&pixel_valid_pix)begin
    pixel_first_pending<=0;
    if(descriptor_pix[127:96]!=last_pixel_job)begin
     if(event_req!=event_ack_pix_sync[1]||descriptor_pix[127:96]==0)pixel_error<=1;
     else begin event_hold<=descriptor_pix;event_req<=~event_req;last_pixel_job<=descriptor_pix[127:96];end
    end
   end
  end
 end
endmodule
