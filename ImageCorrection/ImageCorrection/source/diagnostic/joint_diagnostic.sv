// Diagnostic scheduler. Outputs drive only UART and post-init SCCB reads.
// Camera/video/DDR controls are not inputs to any feedback path from here.
module joint_diagnostic #(
 parameter SYS_HZ=27000000,BAUD_DIV=234,WIDTH=1920,HEIGHT=1080,
 parameter ROI0_X=472,ROI1_X=952,ROI_Y=532
)(
 input wire serial_available,output wire serial_busy,
 input wire clk,rst_n,pclk,p_rst_n,enabled,input wire[31:0]sensor_info,
 input wire bus_busy,bus_done,bus_error,input wire[3:0]bus_code,input wire[7:0]bus_data,
 output wire bus_start,output wire[15:0]bus_addr,output wire tx,
 input wire raw_valid,raw_sof,input wire[7:0]raw,input wire[11:0]raw_x,input wire[10:0]raw_y,
 input wire rgb_valid,rgb_sof,rgb_eof,input wire[15:0]rgb565,input wire[11:0]rgb_x,input wire[10:0]rgb_y,
 input wire frame_done,frame_good,
 input wire[12:0]gain_r,gain_g,gain_b,input wire[7:0]tone,
 input wire[15:0]stat_n,stat_h,input wire[23:0]stat_v
);
 reg ev_start,snap_start,packet_start;
 wire ev_busy,ev_done,ev_timeout,snap_done,snap_busy,packet_busy,packet_done;
 wire[111:0]ev_values;
 reg ev_rd_start;reg[6:0]ev_rd_index;wire ev_rd_done,ev_equal;wire[7:0]ev_rd_data;wire[103:0]ev_mask;wire[31:0]ev_error,ev0_start,ev0_end,ev1_start,ev1_end,source_fid;
 wire[351:0]snapshot_meta;reg[351:0]held_meta;
 reg rd_start;reg[9:0]rd_addr;wire rd_done,rd_busy;wire[15:0]rd_data;
 joint_pixel_snapshot #(.ROI0_X(ROI0_X),.ROI1_X(ROI1_X),.ROI_Y(ROI_Y))pixels(
 .pclk(pclk),.p_rst_n(p_rst_n),.clk(clk),.rst_n(rst_n),.snap_start(snap_start),.snap_done(snap_done),.snap_busy(snap_busy),.snapshot_meta(snapshot_meta),.frame_id_sys(source_fid),
 .rd_start(rd_start),.rd_addr(rd_addr),.rd_done(rd_done),.rd_busy(rd_busy),.rd_data(rd_data),
 .raw_valid(raw_valid),.raw_sof(raw_sof),.raw(raw),.raw_x(raw_x),.raw_y(raw_y),.rgb_valid(rgb_valid),.rgb_sof(rgb_sof),.rgb_eof(rgb_eof),.rgb565(rgb565),.rgb_x(rgb_x),.rgb_y(rgb_y),
 .frame_done(frame_done),.frame_good(frame_good),.gain_r(gain_r),.gain_g(gain_g),.gain_b(gain_b),.tone(tone),.stat_n(stat_n),.stat_h(stat_h),.stat_v(stat_v));
 joint_exposure_read #(.TIMEOUT_TICKS(SYS_HZ/5))exposure(
 .clk(clk),.rst_n(rst_n),.enabled(enabled),.start(ev_start),.frame_id(source_fid),
 .bus_busy(bus_busy),.bus_done(bus_done),.bus_error(bus_error),.bus_code(bus_code),.bus_data(bus_data),.bus_start(bus_start),.bus_addr(bus_addr),
 .busy(ev_busy),.done(ev_done),.timed_out(ev_timeout),.values(ev_values),.rd_start(ev_rd_start),.rd_index(ev_rd_index),.rd_done(ev_rd_done),.rd_data(ev_rd_data),.scans_equal(ev_equal),.valid_mask(ev_mask),.first_error(ev_error),.scan0_start(ev0_start),.scan0_end(ev0_end),.scan1_start(ev1_start),.scan1_end(ev1_end));
 assign serial_busy=(state==4)||packet_start||packet_busy;
 reg[2:0]state;reg[31:0]ticks,timer,sequence_id,event_start,event_end;
 reg snap_fault,boot_fault,fatal_diag,have_pixels;
 reg provider_fault;reg[31:0]read_timer;
 always @(posedge clk)begin
 if(!rst_n)begin
 state<=0;ticks<=0;timer<=0;sequence_id<=0;event_start<=0;event_end<=0;
 ev_start<=0;snap_start<=0;packet_start<=0;held_meta<=0;
 snap_fault<=0;boot_fault<=0;fatal_diag<=0;have_pixels<=0;
 end else begin
 ticks<=ticks+1'b1;ev_start<=0;snap_start<=0;packet_start<=0;
 case(state)
 0:begin
 if(enabled&&!bus_busy)begin
 ev_start<=1;sequence_id<=sequence_id+1'b1;event_start<=ticks;
 timer<=0;snap_fault<=0;boot_fault<=0;state<=1;
 end else if(timer==SYS_HZ*3-1)begin
 boot_fault<=1;fatal_diag<=1;held_meta<=0;have_pixels<=0;
 sequence_id<=sequence_id+1'b1;event_start<=ticks;event_end<=ticks;state<=3;
 end else timer<=timer+1'b1;
 end
 1:if(ev_done)begin
 if(ev_timeout)begin held_meta<=0;have_pixels<=0;fatal_diag<=1;event_end<=ticks;state<=3;end
 else begin snap_start<=1;timer<=0;state<=2;end
 end
 2:begin
 if(snap_done)begin
 held_meta<=snapshot_meta;have_pixels<=snapshot_meta[63:32]==32'h27;
 event_end<=ticks;state<=3;
 end else if(timer==SYS_HZ*2-1)begin
 held_meta<=0;have_pixels<=0;snap_fault<=1;fatal_diag<=1;event_end<=ticks;state<=3;
 end else timer<=timer+1'b1;
 end
 3:if(serial_available)begin packet_start<=1;state<=4;end
 4:if(packet_done)begin timer<=0;state<=5;end
 5:if(timer==SYS_HZ-1)begin timer<=0;state<=fatal_diag?3:0;end else timer<=timer+1'b1;
 default:state<=0;
 endcase
 // A partial packet with a formerly-valid header must never complete with
 // replacement zeros and a valid CRC. Abort only the diagnostic UART, then
 // leave a full idle pause before repeating explicitly-invalid packets.
 if(provider_fault)begin
 fatal_diag<=1;snap_fault<=1;have_pixels<=0;held_meta<=0;
 event_end<=ticks;packet_start<=0;timer<=0;state<=5;
 end
 end
 end
 wire all_ev=&ev_mask;
 wire[15:0]sys_flags={9'd0,ev_timeout,boot_fault,snap_fault,
 (ev_mask[103:52]=={52{1'b1}}&&ev1_start==ev1_end),(ev_mask[51:0]=={52{1'b1}}&&ev0_start==ev0_end),
 (all_ev&&ev_equal),all_ev};
 function automatic[31:0]header(input[5:0]i);
 begin case(i)
 0:header=32'hd1600140;1:header=sequence_id;2:header={sys_flags,held_meta[47:32]};3:header=held_meta[31:0];
 4:header=ev0_start;5:header=ev0_end;6:header=ev1_start;7:header=ev1_end;
 8:header={12'd0,ev_values[3:0],ev_values[15:8],ev_values[23:16]};
 9:header={12'd0,ev_values[59:56],ev_values[71:64],ev_values[79:72]};
 10:header={ev_values[31:24],ev_values[55:48],ev_values[39:32],ev_values[47:40]};
 11:header={ev_values[87:80],ev_values[111:104],ev_values[95:88],ev_values[103:96]};
 12:header={18'd0,ev_mask[58:52],ev_mask[6:0]};13:header=ev_error;
 14:header=held_meta[95:64];15:header=held_meta[127:96];16:header=held_meta[159:128];17:header=held_meta[191:160];
 18:header=held_meta[223:192];19:header=held_meta[255:224];20:header=held_meta[287:256];21:header=held_meta[319:288];
 22:header=0;23:header=(ROI0_X<<16)|ROI_Y;24:header=(ROI1_X<<16)|ROI_Y;
 25:header=32'h00100010;26:header=(WIDTH<<16)|HEIGHT;27:header=event_start;28:header=event_end;
 29:header=32'h00010001;30:header=sensor_info;31:header=32'h04004001;
 32:header=ev_mask[31:0];33:header={12'd0,ev_mask[51:32]};
 34:header=ev_mask[83:52];35:header={12'd0,ev_mask[103:84]};
 62:header=32'h34340201;63:header=32'h56e838cd;
 default:begin
 header=0; // Bulk H24..3D provided by synchronous observation RAM below.
 end
 endcase end endfunction
 wire word_req;wire[9:0]word_index;reg word_ready;reg[31:0]word_data;
 reg[2:0]provider;reg[15:0]high_word;reg[1:0]ev_byte;reg[23:0]ev_word;
 wire[9:0]pair_address=(word_index-10'd64)<<1;
 always @(posedge clk)begin
 if(!rst_n)begin rd_start<=0;rd_addr<=0;word_ready<=0;word_data<=0;provider<=0;high_word<=0;provider_fault<=0;read_timer<=0;ev_rd_start<=0;ev_rd_index<=0;ev_byte<=0;ev_word<=0;end
 else begin
 ev_rd_start<=0;rd_start<=0;word_ready<=0;provider_fault<=0;
 if(provider==0)read_timer<=0;else read_timer<=read_timer+1'b1;
 case(provider)
 0:if(word_req)begin
 if(word_index>=36&&word_index<=61)begin
 ev_rd_index<=(word_index-36)<<2;ev_rd_start<=1;ev_byte<=0;ev_word<=0;provider<=3;
 end else if(word_index<64)begin word_data<=header(word_index[5:0]);word_ready<=1;end
 else if(!have_pixels)begin word_data<=0;word_ready<=1;end
 else begin rd_addr<=pair_address;rd_start<=1;provider<=1;end
 end
 1:if(rd_done)begin high_word<=rd_data;rd_addr<=rd_addr+1'b1;rd_start<=1;provider<=2;end
 2:if(rd_done)begin word_data<={high_word,rd_data};word_ready<=1;provider<=0;end
 3:if(ev_rd_done)begin
 if(ev_byte==3)begin word_data<={ev_word,ev_rd_data};word_ready<=1;provider<=0;end
 else begin ev_word<={ev_word[15:0],ev_rd_data};ev_byte<=ev_byte+1'b1;ev_rd_index<=ev_rd_index+1'b1;ev_rd_start<=1;end
 end
 default:provider<=0;
 endcase
 if(provider!=0&&!rd_done&&!ev_rd_done&&read_timer==SYS_HZ/5-1)begin
 provider<=0;provider_fault<=1;read_timer<=0;rd_start<=0;ev_rd_start<=0;word_ready<=0;
 end
 end
 end
 joint_diag_uart #(.BAUD_DIV(BAUD_DIV))serial(.clk(clk),.rst_n(rst_n&&!provider_fault),.start(packet_start),.busy(packet_busy),.done(packet_done),.tx(tx),.word_req(word_req),.word_index(word_index),.word_ready(word_ready),.word_data(word_data));
endmodule
