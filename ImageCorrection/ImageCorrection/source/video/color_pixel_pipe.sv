// WB -> bilinear multiscale illumination gain -> pixel safeguards -> RGB565.
// One token/clock, six-edge latency. Statistics remain at the WB boundary.
module color_pixel_pipe #(
 parameter TONE_FILE="data/color_tone_gain.hex", // retained API, unused
 parameter PROTECT_FILE="data/msag_protect.hex"
)(
 input wire clk,rst_n,in_valid,input wire [23:0] in_rgb,
 input wire [11:0] in_x,input wire [10:0] in_y,input wire in_sof,in_eol,in_eof,
 input wire [12:0] gain_r,gain_g,gain_b,input wire [7:0] tone_pos,
 input wire [11:0] map00,map10,map01,map11,input wire [4:0] map_fx,map_fy,
 output wire out_valid,output reg [23:0] out_rgb,output wire [15:0] out565,
 output wire [11:0] out_x,output wire [10:0] out_y,output wire out_sof,out_eol,out_eof,
 output wire stat_valid,output wire [23:0] stat_raw,stat_wb,
 output wire [11:0] stat_x,output wire [10:0] stat_y,output wire stat_sof,stat_eof
);
 reg [6:0] vp;
 reg [25:0] m0,m1,m2,m3,m4,m5,m6;
 reg [20:0] wr0,wg0,wb0;
 reg [23:0] raw0,raw1,b1,b2,b3,b4;
 reg [16:0] hx1,hx2;
 reg [4:0] fy1;
 wire [22:0] vy=hx1*(6'd32-{1'b0,fy1})+hx2*{1'b0,fy1};
 reg [11:0] map_gain2;
 (* syn_ramstyle="block_ram" *) reg [32:0] protection[0:255];
 reg [32:0] limits2;
 initial $readmemh(PROTECT_FILE,protection);
 wire [7:0] peakrg=b1[23:16]>b1[15:8]?b1[23:16]:b1[15:8];
 wire [7:0] peak=peakrg>b1[7:0]?peakrg:b1[7:0];
 wire [11:0] nearcap=limits2[32:21],clipcap=limits2[11:0];
 wire [8:0] highlight=limits2[20:12];
 wire [11:0] bounded=map_gain2>nearcap?nearcap:map_gain2;
 reg [20:0] blended3;
 reg [11:0] clip3,gain4;
 reg [19:0] pr5,pg5,pb5;
 wire [12:0] final_gain=13'd512+(blended3>>8);
 function automatic [7:0] wb_round(input [20:0] product);
  reg [21:0] s;begin s={1'b0,product}+2048;wb_round=(s[21:12]>255)?255:s[19:12];end
 endfunction
 function automatic [7:0] tone_round(input [19:0] product);
  reg [20:0] s;begin s={1'b0,product}+256;tone_round=(s[20:9]>255)?255:s[16:9];end
 endfunction
 always @(posedge clk)if(vp[1])limits2<=protection[peak];
 always @(posedge clk)begin
  if(!rst_n)begin
   vp<=0;m0<=0;m1<=0;m2<=0;m3<=0;m4<=0;m5<=0;m6<=0;
   wr0<=0;wg0<=0;wb0<=0;raw0<=0;raw1<=0;
   b1<=0;b2<=0;b3<=0;b4<=0;hx1<=0;hx2<=0;fy1<=0;
   map_gain2<=512;blended3<=0;clip3<=512;gain4<=512;
   pr5<=0;pg5<=0;pb5<=0;out_rgb<=0;
  end else begin
   vp<={vp[5:0],in_valid};
   m0<={in_sof,in_eol,in_eof,in_x,in_y};m1<=m0;m2<=m1;m3<=m2;m4<=m3;m5<=m4;m6<=m5;
   if(in_valid)begin
    wr0<=in_rgb[23:16]*gain_r;wg0<=in_rgb[15:8]*gain_g;wb0<=in_rgb[7:0]*gain_b;raw0<=in_rgb;
   end
   if(vp[0])begin
    b1<={wb_round(wr0),wb_round(wg0),wb_round(wb0)};raw1<=raw0;
    hx1<=map00*(6'd32-{1'b0,map_fx})+map10*{1'b0,map_fx};
    hx2<=map01*(6'd32-{1'b0,map_fx})+map11*{1'b0,map_fx};fy1<=map_fy;
   end
   if(vp[1])begin map_gain2<=(vy+512)>>10;b2<=b1;end
   if(vp[2])begin blended3<=(bounded-12'd512)*highlight;clip3<=clipcap;b3<=b2;end
   if(vp[3])begin gain4<=final_gain>clip3?clip3:final_gain[11:0];b4<=b3;end
   if(vp[4])begin pr5<=b4[23:16]*gain4;pg5<=b4[15:8]*gain4;pb5<=b4[7:0]*gain4;end
   if(vp[5])out_rgb<={tone_round(pr5),tone_round(pg5),tone_round(pb5)};
  end
 end
 assign out_valid=vp[6];assign out565={out_rgb[23:19],out_rgb[15:10],out_rgb[7:3]};
 assign {out_sof,out_eol,out_eof,out_x,out_y}=m6;
 assign stat_valid=vp[1];assign stat_raw=raw1;assign stat_wb=b1;
 wire unused_eol;assign {stat_sof,unused_eol,stat_eof,stat_x,stat_y}=m1;
endmodule
