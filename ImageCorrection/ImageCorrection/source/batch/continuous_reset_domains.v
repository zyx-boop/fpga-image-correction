// Cold-start RDC only. Never driven by abort, rearm, epoch, Bank or vblank.
// init_done qualifies boot once; a later loss faults without resetting state.
module continuous_reset_domains(
 input wire core_clk,cam_clk,pixel_clk,sys_clk,reset_request_n,init_done,
 input wire capture_sof,token_valid,token_eof,
 output wire core_rst_n,cam_rst_n,pixel_rst_n,sys_rst_n,
 output reg ready_core=0,output wire ready_cam,output reg stream_active=0,
 output wire accept_sof,accept_token,output reg boot_fault=0
);
 (* ASYNC_REG="TRUE" *) reg [1:0] system_core_sync=0;
 reg boot_seen=0;
 wire transport_request_n=reset_request_n&&boot_seen;
 (* ASYNC_REG="TRUE" *) reg [1:0] core_release=0,cam_release=0,pixel_release=0,sys_release=0;
 (* ASYNC_REG="TRUE" *) reg [1:0] cam_ready_core=0,pixel_ready_core=0,sys_ready_core=0,ready_cam_sync=0;
 reg tail_seen=0;
 assign core_rst_n=core_release[1];assign cam_rst_n=cam_release[1];
 assign pixel_rst_n=pixel_release[1];assign sys_rst_n=sys_release[1];
 assign ready_cam=ready_cam_sync[1];
 assign accept_sof=cam_rst_n&&ready_cam&&capture_sof&&
  (stream_active||(tail_seen&&!(token_valid&&token_eof)));
 assign accept_token=cam_rst_n&&stream_active&&token_valid;
 always @(posedge core_clk or negedge reset_request_n)begin
  if(!reset_request_n)begin system_core_sync<=0;boot_seen<=0;boot_fault<=0;end
  else begin
   system_core_sync<={system_core_sync[0],1'b1};
   if(system_core_sync[1])begin
    if(init_done)boot_seen<=1;
    if(boot_seen&&!init_done)boot_fault<=1;
   end
  end
 end
 always @(posedge core_clk or negedge transport_request_n)
  if(!transport_request_n)core_release<=0;else core_release<={core_release[0],1'b1};
 always @(posedge cam_clk or negedge transport_request_n)
  if(!transport_request_n)cam_release<=0;else cam_release<={cam_release[0],1'b1};
 always @(posedge pixel_clk or negedge transport_request_n)
  if(!transport_request_n)pixel_release<=0;else pixel_release<={pixel_release[0],1'b1};
 always @(posedge sys_clk or negedge transport_request_n)
  if(!transport_request_n)sys_release<=0;else sys_release<={sys_release[0],1'b1};
 always @(posedge core_clk or negedge core_rst_n)begin
  if(!core_rst_n)begin cam_ready_core<=0;pixel_ready_core<=0;sys_ready_core<=0;ready_core<=0;end
  else begin
   cam_ready_core<={cam_ready_core[0],cam_rst_n};pixel_ready_core<={pixel_ready_core[0],pixel_rst_n};
   sys_ready_core<={sys_ready_core[0],sys_rst_n};
   ready_core<=cam_ready_core[1]&&pixel_ready_core[1]&&sys_ready_core[1];
  end
 end
 always @(posedge cam_clk or negedge cam_rst_n)begin
  if(!cam_rst_n)begin ready_cam_sync<=0;tail_seen<=0;stream_active<=0;end
  else begin
   ready_cam_sync<={ready_cam_sync[0],ready_core};
   if(!stream_active&&ready_cam)begin
    if(token_valid&&token_eof)tail_seen<=1;
    else if(tail_seen&&capture_sof)stream_active<=1;
   end
  end
 end
endmodule
