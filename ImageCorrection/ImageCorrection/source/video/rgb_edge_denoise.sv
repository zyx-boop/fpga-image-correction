// Symmetric 3x3 range-gated binomial filter, before WB/Gamma.
// Rejected neighbours are replaced by the center, keeping denominator 16.
// All channels share the range decision; never mix unrelated CFA colours.
// Two rotating RGB rows. Read old row at input acceptance; write that pixel
// one clock later. Ordered x addresses never collide, even with no HBLANK.
// Banked LUTRAM with three registered read stages, no frame store/new CDC.
// Needs >= WIDTH+16 idle clocks after EOF, provided by sensor VBLANK.
module rgb_edge_denoise #(parameter WIDTH=1920,HEIGHT=1080)(
 input wire clk,rst_n,in_valid,input wire [23:0] in_rgb,
 input wire [11:0] in_x,input wire [10:0] in_y,input wire in_sof,in_eol,in_eof,
 output wire out_valid,output wire [23:0] out_rgb,
 output wire [11:0] out_x,output wire [10:0] out_y,
 output wire out_sof,out_eol,out_eof
);
 wire [23:0] r0,r1;
 reg [23:0] bottom,write_data;
 reg bank,last_bank,bank_q,write_select,write_pending;
 reg [11:0] write_addr;
 reg [1:0] replay_delay;
 reg replay_active,vq,lq;reg [11:0] xq,replay_x;reg [10:0] yq;
 wire write_bank=in_sof?1'b0:bank;
 wire replay_read=!in_valid&&(replay_active||replay_delay==1);
 wire [11:0] read_x=in_valid?in_x:replay_x;
 denoise_line_store #(.WIDTH(WIDTH)) line0(.clk(clk),.rst_n(rst_n),.read_en(in_valid||replay_read),
  .write_en(write_pending&&!write_select),.read_addr(read_x),.write_addr(write_addr),.write_data(write_data),.read_data(r0));
 denoise_line_store #(.WIDTH(WIDTH)) line1(.clk(clk),.rst_n(rst_n),.read_en(in_valid||replay_read),
  .write_en(write_pending&&write_select),.read_addr(read_x),.write_addr(write_addr),.write_data(write_data),.read_data(r1));
 always @(posedge clk)begin
  if(!rst_n)begin write_pending<=0;write_addr<=0;write_data<=0;write_select<=0;end
  else begin
   write_pending<=in_valid;
   if(in_valid)begin write_addr<=in_x;write_data<=in_rgb;write_select<=write_bank;end
  end
 end
 always @(posedge clk)begin
  if(!rst_n)begin bank<=0;last_bank<=0;bank_q<=0;bottom<=0;replay_delay<=0;replay_active<=0;vq<=0;lq<=0;xq<=0;yq<=0;replay_x<=0;end
  else begin
   vq<=0;lq<=0;if(replay_delay!=0)replay_delay<=replay_delay-1'b1;
   if(in_valid)begin
    vq<=1;lq<=in_eol;xq<=in_x;yq<=in_y;bottom<=in_rgb;bank_q<=write_bank;
    if(in_sof)begin bank<=0;replay_active<=0;replay_delay<=0;end
    if(in_eol)bank<=~write_bank;
    if(in_eof)begin last_bank<=write_bank;replay_delay<=2;replay_x<=0;end
   end else if(replay_read)begin
    vq<=1;lq<=replay_x==WIDTH-1;xq<=replay_x;yq<=HEIGHT;bottom<=0;
    bank_q<=~last_bank;
    replay_active<=replay_x!=WIDTH-1;replay_x<=replay_x+1'b1;
   end
  end
 end
 reg [1:0] vpipe,lpipe,bkpipe;
 reg [11:0] xp1,xp2;reg [10:0] yp1,yp2;reg [23:0] bp1,bp2;
 always @(posedge clk)begin
  if(!rst_n)begin vpipe<=0;lpipe<=0;bkpipe<=0;xp1<=0;xp2<=0;yp1<=0;yp2<=0;bp1<=0;bp2<=0;end
  else begin
   vpipe<={vpipe[0],vq};lpipe<={lpipe[0],lq};bkpipe<={bkpipe[0],bank_q};
   xp1<=xq;xp2<=xp1;yp1<=yq;yp2<=yp1;bp1<=bottom;bp2<=bp1;
  end
 end
 wire [23:0] middle=bkpipe[1]?r0:r1;
 wire [23:0] older=bkpipe[1]?r1:r0;
 wire [23:0] top_sample=yp2>=2?older:24'd0;
 wire [23:0] mid_sample=yp2>=1?middle:24'd0;
 reg [23:0] t2,t1,m2,m1,b2,b1,flush_pixel;
 reg flush_right,wv,wb;reg [10:0] flush_y,wy;reg [11:0] wx;
 reg [23:0] window[0:8];
 always @(posedge clk)begin
  if(!rst_n)begin t2<=0;t1<=0;m2<=0;m1<=0;b2<=0;b1<=0;flush_right<=0;flush_pixel<=0;flush_y<=0;wv<=0;wb<=0;wx<=0;wy<=0;end
  else begin
   wv<=0;
   if(flush_right)begin wv<=1;wb<=1;wx<=WIDTH-1;wy<=flush_y;window[4]<=flush_pixel;flush_right<=0;end
   if(vpipe[1])begin
    if(xp2==0)begin t2<=0;t1<=top_sample;m2<=0;m1<=mid_sample;b2<=0;b1<=bp2;end
    else begin
     t2<=t1;t1<=top_sample;m2<=m1;m1<=mid_sample;b2<=b1;b1<=bp2;
     if(yp2!=0)begin
      wv<=1;wx<=xp2-1'b1;wy<=yp2-1'b1;
      wb<=xp2==1||yp2==1||yp2==HEIGHT;
      window[0]<=t2;window[1]<=t1;window[2]<=top_sample;
      window[3]<=m2;window[4]<=m1;window[5]<=mid_sample;
      window[6]<=b2;window[7]<=b1;window[8]<=bp2;
     end
    end
    if(lpipe[1]&&yp2!=0)begin flush_right<=1;flush_y<=yp2-1'b1;flush_pixel<=mid_sample;end
   end
  end
 end
 wire [7:0] peak_rg=window[4][23:16]>window[4][15:8]?window[4][23:16]:window[4][15:8];
 wire [7:0] peak=peak_rg>window[4][7:0]?peak_rg:window[4][7:0];
 wire [7:0] threshold=peak<64?8'd24:peak<128?8'd16:8'd8;
 reg [5:0] valid;
 reg [22:0] meta[0:5];reg [23:0] center[0:5];reg [5:0] border;
 reg [23:0] a[0:8],b[0:8],chosen[0:8];
 reg [7:0] dr[0:8],dg[0:8],db[0:8];reg [7:0] threshold_a;reg [8:0] accept;
 reg [9:0] row_r[0:2],row_g[0:2],row_b[0:2];
 reg [11:0] sum_r,sum_g,sum_b;reg [23:0] result;
 function [7:0] diff(input [7:0] x,y);diff=x>y?x-y:y-x;endfunction
 integer i,j;
 always @(posedge clk)begin
  if(!rst_n)begin valid<=0;border<=0;result<=0;for(i=0;i<6;i=i+1)begin meta[i]<=0;center[i]<=0;end end
  else begin
   valid<={valid[4:0],wv};border<={border[4:0],wb};meta[0]<={wy,wx};center[0]<=window[4];
   for(i=1;i<6;i=i+1)begin meta[i]<=meta[i-1];center[i]<=center[i-1];end
   if(wv)begin
    threshold_a<=threshold;
    for(i=0;i<9;i=i+1)begin a[i]<=window[i];dr[i]<=diff(window[i][23:16],window[4][23:16]);dg[i]<=diff(window[i][15:8],window[4][15:8]);db[i]<=diff(window[i][7:0],window[4][7:0]);end
   end
   if(valid[0])for(i=0;i<9;i=i+1)begin b[i]<=a[i];accept[i]<=dr[i]<=threshold_a&&dg[i]<=threshold_a&&db[i]<=threshold_a;end
   if(valid[1])for(i=0;i<9;i=i+1)chosen[i]<=accept[i]?b[i]:center[1];
   if(valid[2])for(j=0;j<3;j=j+1)begin
    row_r[j]<={2'b0,chosen[3*j][23:16]}+{1'b0,chosen[3*j+1][23:16],1'b0}+{2'b0,chosen[3*j+2][23:16]};
    row_g[j]<={2'b0,chosen[3*j][15:8]}+{1'b0,chosen[3*j+1][15:8],1'b0}+{2'b0,chosen[3*j+2][15:8]};
    row_b[j]<={2'b0,chosen[3*j][7:0]}+{1'b0,chosen[3*j+1][7:0],1'b0}+{2'b0,chosen[3*j+2][7:0]};
   end
   if(valid[3])begin sum_r<={2'b0,row_r[0]}+{1'b0,row_r[1],1'b0}+{2'b0,row_r[2]}+12'd8;
    sum_g<={2'b0,row_g[0]}+{1'b0,row_g[1],1'b0}+{2'b0,row_g[2]}+12'd8;
    sum_b<={2'b0,row_b[0]}+{1'b0,row_b[1],1'b0}+{2'b0,row_b[2]}+12'd8;end
   if(valid[4])result<=border[4]?center[4]:{sum_r[11:4],sum_g[11:4],sum_b[11:4]};
  end
 end
 assign out_valid=valid[5];assign out_rgb=result;assign {out_y,out_x}=meta[5];
 assign out_sof=out_valid&&out_x==0&&out_y==0;
 assign out_eol=out_valid&&out_x==WIDTH-1;
 assign out_eof=out_eol&&out_y==HEIGHT-1;
`ifndef SYNTHESIS
 initial if(WIDTH<2||HEIGHT<2)$fatal(1,"denoise dimensions must be >=2");
 always @(posedge clk)if(rst_n&&in_valid)begin
  if(in_x>=WIDTH||in_y>=HEIGHT)$fatal(1,"denoise input range");
  if(in_sof!=(in_x==0&&in_y==0)||in_eol!=(in_x==WIDTH-1)||in_eof!=(in_x==WIDTH-1&&in_y==HEIGHT-1))$fatal(1,"denoise input markers");
 end
 always @(posedge clk)if(rst_n&&write_pending&&(in_valid||replay_read)&&write_addr==read_x)
  $fatal(1,"denoise illegal same-address read/write; input ordering/blanking violated");
`endif
endmodule
