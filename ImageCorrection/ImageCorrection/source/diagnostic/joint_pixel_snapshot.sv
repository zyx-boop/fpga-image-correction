// Observation only. One source frame, two RAW/RGB565 tiles, immutable until
// the next request. Both RAM write ports are PCLK; UART reads via a mailbox.
module joint_pixel_snapshot #(
 parameter ROI0_X=472,ROI1_X=952,ROI_Y=532
)(
 input wire pclk,p_rst_n,clk,rst_n,
 input wire snap_start,output reg snap_done,snap_busy,
 output reg[351:0] snapshot_meta,output wire[31:0] frame_id_sys,
 input wire rd_start,input wire[9:0]rd_addr,
 output reg rd_done,rd_busy,output reg[15:0]rd_data,
 input wire raw_valid,raw_sof,input wire[7:0]raw,input wire[11:0]raw_x,input wire[10:0]raw_y,
 input wire rgb_valid,rgb_sof,rgb_eof,input wire[15:0]rgb565,input wire[11:0]rgb_x,input wire[10:0]rgb_y,
 input wire frame_done,frame_good,
 input wire[12:0]gain_r,gain_g,gain_b,input wire[7:0]tone,
 input wire[15:0]stat_n,stat_h,input wire[23:0]stat_v
);
 (* syn_ramstyle="block_ram" *) reg[15:0] samples[0:1023];
 reg[31:0] frame_counter,frame_gray;
 wire[31:0] next_fid=frame_counter+32'd1;
 reg req_toggle,read_toggle,ack_p,read_ack_p;
 (* ASYNC_REG="TRUE" *)reg req1,req2,read1,read2;
 (* ASYNC_REG="TRUE" *)reg ack1,ack2,rack1,rack2;
 (* ASYNC_REG="TRUE" *)reg[31:0]gray1,gray2;
 function automatic[31:0] ungray(input[31:0]v);
 integer i;begin ungray[31]=v[31];for(i=30;i>=0;i=i-1)ungray[i]=ungray[i+1]^v[i];end endfunction
 assign frame_id_sys=ungray(gray2);
 reg[351:0] meta_p;reg[15:0]read_hold;
 reg[9:0]read_address_hold;
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin
 req_toggle<=0;read_toggle<=0;ack1<=0;ack2<=0;rack1<=0;rack2<=0;gray1<=0;gray2<=0;
 snap_done<=0;snap_busy<=0;snapshot_meta<=0;rd_done<=0;rd_busy<=0;rd_data<=0;read_address_hold<=0;
 end else begin
 ack1<=ack_p;ack2<=ack1;rack1<=read_ack_p;rack2<=rack1;gray1<=frame_gray;gray2<=gray1;
 snap_done<=0;rd_done<=0;
 if(snap_start&&!snap_busy&&!rd_busy)begin req_toggle<=~req_toggle;snap_busy<=1;end
 else if(snap_busy&&ack2==req_toggle)begin snapshot_meta<=meta_p;snap_done<=1;snap_busy<=0;end
 if(rd_start&&!rd_busy&&!snap_busy&&!snap_start)begin read_toggle<=~read_toggle;read_address_hold<=rd_addr;rd_busy<=1;end
 else if(rd_busy&&rack2==read_toggle)begin rd_data<=read_hold;rd_done<=1;rd_busy<=0;end
 end
 end
 localparam IDLE=0,WAIT_SOF=1,CAPTURE=2,HOLD=3;
 reg[1:0] state;reg seen_req,post_started,post_eof,bad_order;
 reg[8:0]r0,r1,c0,c1;
 wire raw_y_hit=raw_y>=ROI_Y&&raw_y<ROI_Y+16;
 wire rgb_y_hit=rgb_y>=ROI_Y&&rgb_y<ROI_Y+16;
 wire rh0=raw_y_hit&&raw_x>=ROI0_X&&raw_x<ROI0_X+16;
 wire rh1=raw_y_hit&&raw_x>=ROI1_X&&raw_x<ROI1_X+16;
 wire ch0=rgb_y_hit&&rgb_x>=ROI0_X&&rgb_x<ROI0_X+16;
 wire ch1=rgb_y_hit&&rgb_x>=ROI1_X&&rgb_x<ROI1_X+16;
 wire[11:0]raw_lx=raw_x-(rh1?ROI1_X:ROI0_X),rgb_lx=rgb_x-(ch1?ROI1_X:ROI0_X);
 wire[10:0]raw_ly=raw_y-ROI_Y,rgb_ly=rgb_y-ROI_Y;
 wire[7:0]raw_local={raw_ly[3:0],raw_lx[3:0]},rgb_local={rgb_ly[3:0],rgb_lx[3:0]};
 wire overlap=state==CAPTURE&&raw_valid&&raw_sof;
 wire raw_we=state==CAPTURE&&!overlap&&raw_valid&&(rh0||rh1);
 wire rgb_we=state==CAPTURE&&!overlap&&post_started&&rgb_valid&&(ch0||ch1);
 wire[9:0]raw_address={1'b0,rh1,raw_local},rgb_address={1'b1,ch1,rgb_local};
 reg read_go;reg[9:0]read_address_p;reg[15:0]ram_q;reg[1:0]rstate;
 // ADS Language Support 4.1.3.1 NORMAL WRITE: one port, one address.
 wire[9:0]port_a_address=raw_we?raw_address:read_address_p;
 always @(posedge pclk)begin
 if(raw_we||read_go)begin
 if(raw_we)samples[port_a_address]<={8'd0,raw};
 else ram_q<=samples[port_a_address];
 end
 end
 always @(posedge pclk)if(rgb_we)samples[rgb_address]<=rgb565;
 wire raw_ok=r0==256&&r1==256,rgb_ok=c0==256&&c1==256;
 wire eof_now=rgb_valid&&rgb_eof&&post_started;
 wire[31:0]flags={26'd0,(post_eof||eof_now),1'b0,bad_order,rgb_ok,raw_ok,frame_good};
 wire[15:0]raw_total={7'd0,r0}+{7'd0,r1},rgb_total={7'd0,c0}+{7'd0,c1};
 always @(posedge pclk or negedge p_rst_n)begin
 if(!p_rst_n)begin
 frame_counter<=0;frame_gray<=0;req1<=0;req2<=0;read1<=0;read2<=0;
 state<=IDLE;seen_req<=0;ack_p<=0;read_ack_p<=0;meta_p<=0;read_hold<=0;
 post_started<=0;post_eof<=0;bad_order<=0;r0<=0;r1<=0;c0<=0;c1<=0;
 read_go<=0;read_address_p<=0;rstate<=0;
 end else begin
 req1<=req_toggle;req2<=req1;read1<=read_toggle;read2<=read1;read_go<=0;
 if(raw_valid&&raw_sof)begin frame_counter<=next_fid;frame_gray<=(next_fid>>1)^next_fid;end
 if((state==IDLE||state==HOLD)&&req2!=seen_req)begin
 seen_req<=req2;state<=WAIT_SOF;meta_p<=0;
 end else if(state==WAIT_SOF&&raw_valid&&raw_sof)begin
 state<=CAPTURE;meta_p[31:0]<=next_fid;
 r0<=0;r1<=0;c0<=0;c1<=0;bad_order<=0;post_started<=0;post_eof<=0;
 end else if(state==CAPTURE)begin
 if(overlap)begin
 state<=HOLD;meta_p[63:32]<=(flags|32'h10)&32'hfffffffe;
 meta_p[319:288]<={raw_total,rgb_total};ack_p<=seen_req;
 end else begin
 if(rgb_valid&&rgb_sof)begin
 post_started<=1;meta_p[95:64]<={19'd0,gain_r};meta_p[127:96]<={19'd0,gain_g};
 meta_p[159:128]<={19'd0,gain_b};meta_p[191:160]<={24'd0,tone};
 end
 if(eof_now)post_eof<=1;
 if(raw_we)begin
 if(rh1)begin if(r1>=256||{1'b0,raw_local}!=r1)bad_order<=1;if(r1<511)r1<=r1+1'b1;end
 else begin if(r0>=256||{1'b0,raw_local}!=r0)bad_order<=1;if(r0<511)r0<=r0+1'b1;end
 end
 if(rgb_we)begin
 if(ch1)begin if(c1>=256||{1'b0,rgb_local}!=c1)bad_order<=1;if(c1<511)c1<=c1+1'b1;end
 else begin if(c0>=256||{1'b0,rgb_local}!=c0)bad_order<=1;if(c0<511)c0<=c0+1'b1;end
 end
 if(frame_done&&(post_eof||eof_now))begin
 state<=HOLD;meta_p[63:32]<=flags;meta_p[223:192]<={16'd0,stat_n};
 meta_p[255:224]<={16'd0,stat_h};meta_p[287:256]<={8'd0,stat_v};
 meta_p[319:288]<={raw_total,rgb_total};meta_p[351:320]<=0;ack_p<=seen_req;
 end
 end
 end
 case(rstate)
 0:if(state==HOLD&&req2==seen_req&&read2!=read_ack_p)begin read_address_p<=read_address_hold;read_go<=1;rstate<=1;end
 1:rstate<=2;
 2:begin read_hold<=ram_q;read_ack_p<=read2;rstate<=0;end
 default:rstate<=0;
 endcase
 end
 end
endmodule
