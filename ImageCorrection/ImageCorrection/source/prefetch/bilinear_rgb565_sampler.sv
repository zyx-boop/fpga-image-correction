// R19: two-cycle neighbourhood lookup on four parity-separated cached words.
// Serial fallback remains the default for existing scalar-service callers.
module bilinear_rgb565_sampler #(parameter WORD_CACHE=0)(
 input wire clk,rst,in_valid,output wire in_ready,
 input wire [10:0] in_x,in_y,input wire [15:0] in_fx,in_fy,
 input wire [31:0] in_fid,in_epoch,
 output wire tap_valid,input wire tap_ready,
 output wire [10:0] tap_x,tap_y,output wire [31:0] tap_fid,tap_epoch,
 input wire tap_rsp_valid,input wire [15:0] tap_rsp_data,input wire [255:0] tap_rsp_word,
 output wire rsp_valid,output wire [15:0] rsp_data,output wire busy,fault
);
 generate if(WORD_CACHE)begin:g_words
  bilinear_word_sampler impl(.clk(clk),.rst(rst),.in_valid(in_valid),.in_ready(in_ready),.in_x(in_x),.in_y(in_y),.in_fx(in_fx),.in_fy(in_fy),.in_fid(in_fid),.in_epoch(in_epoch),.tap_valid(tap_valid),.tap_ready(tap_ready),.tap_x(tap_x),.tap_y(tap_y),.tap_fid(tap_fid),.tap_epoch(tap_epoch),.tap_rsp_valid(tap_rsp_valid),.tap_rsp_data(tap_rsp_data),.tap_rsp_word(tap_rsp_word),.rsp_valid(rsp_valid),.rsp_data(rsp_data),.busy(busy),.fault(fault));
 end else begin:g_serial
  bilinear_rgb565_sampler_serial impl(.clk(clk),.rst(rst),.in_valid(in_valid),.in_ready(in_ready),
   .in_x(in_x),.in_y(in_y),.in_fx(in_fx),.in_fy(in_fy),.in_fid(in_fid),.in_epoch(in_epoch),
   .tap_valid(tap_valid),.tap_ready(tap_ready),.tap_x(tap_x),.tap_y(tap_y),.tap_fid(tap_fid),.tap_epoch(tap_epoch),
   .tap_rsp_valid(tap_rsp_valid),.tap_rsp_data(tap_rsp_data),.rsp_valid(rsp_valid),.rsp_data(rsp_data),.busy(busy),.fault(fault));
 end endgenerate
endmodule

module bilinear_word_sampler(
 input wire clk,rst,in_valid,output wire in_ready,
 input wire [10:0] in_x,in_y,input wire [15:0] in_fx,in_fy,
 input wire [31:0] in_fid,in_epoch,
 output wire tap_valid,input wire tap_ready,
 output wire [10:0] tap_x,tap_y,output wire [31:0] tap_fid,tap_epoch,
 input wire tap_rsp_valid,input wire [15:0] tap_rsp_data,input wire [255:0] tap_rsp_word,
 output wire rsp_valid,output wire [15:0] rsp_data,output wire busy,output reg fault
);
 localparam IDLE=0,CHECK=1,OFFER=2,WAIT_WORD=3;
 reg [1:0] state;
 reg [10:0] x0,x1,y0,y1,mx,my;
 reg [15:0] fx,fy,ax,ay,p00,p10,p01,p11;
 reg [31:0] fid,epoch;
 reg context_valid;
 reg [3:0] word_valid;
 reg [17:0] word_tag[0:3];
 reg [255:0] word_data[0:3];
 reg fire;reg [5:0] valid_pipe;
 wire [1:0] b00={y0[0],x0[4]},b10={y0[0],x1[4]},b01={y1[0],x0[4]},b11={y1[0],x1[4]};
 wire h00=word_valid[b00]&&word_tag[b00]=={y0,x0[10:4]};
 wire h10=word_valid[b10]&&word_tag[b10]=={y0,x1[10:4]};
 wire h01=word_valid[b01]&&word_tag[b01]=={y1,x0[10:4]};
 wire h11=word_valid[b11]&&word_tag[b11]=={y1,x1[10:4]};
 wire [255:0] w00=word_data[b00],w10=word_data[b10],w01=word_data[b01],w11=word_data[b11];
 assign in_ready=state==IDLE&&!fault;
 assign tap_valid=state==OFFER&&!fault;
 assign tap_x=mx;assign tap_y=my;assign tap_fid=fid;assign tap_epoch=epoch;
 assign rsp_valid=valid_pipe[5];
 assign busy=state!=IDLE||fire||(|valid_pipe);
 always @(posedge clk)begin
  if(rst)begin
   state<=IDLE;fault<=0;context_valid<=0;word_valid<=0;fire<=0;valid_pipe<=0;
   x0<=0;x1<=0;y0<=0;y1<=0;mx<=0;my<=0;fid<=0;epoch<=0;
   fx<=0;fy<=0;ax<=0;ay<=0;p00<=0;p10<=0;p01<=0;p11<=0;
  end else begin
   fire<=0;valid_pipe<={valid_pipe[4:0],fire};
   case(state)
    IDLE:if(in_valid&&in_ready)begin
     if(!context_valid||fid!=in_fid||epoch!=in_epoch)word_valid<=0;
     context_valid<=1;fid<=in_fid;epoch<=in_epoch;
     x0<=in_x;x1<=in_x==1919?in_x:in_x+1'b1;
     y0<=in_y;y1<=in_y==1079?in_y:in_y+1'b1;
     fx<=in_fx;fy<=in_fy;state<=CHECK;
     if(in_x>=1920||in_y>=1080)fault<=1;
    end
    CHECK:if(h00&&h10&&h01&&h11)begin
     p00<=w00[x0[3:0]*16+:16];p10<=w10[x1[3:0]*16+:16];
     p01<=w01[x0[3:0]*16+:16];p11<=w11[x1[3:0]*16+:16];
     ax<=fx;ay<=fy;fire<=1;state<=IDLE;
    end else begin
     mx<=!h00?x0:!h10?x1:!h01?x0:x1;
     my<=!h00||!h10?y0:y1;state<=OFFER;
    end
    OFFER:if(tap_ready)state<=WAIT_WORD;
    WAIT_WORD:if(tap_rsp_valid)begin
     word_valid[{my[0],mx[4]}]<=1;
     word_tag[{my[0],mx[4]}]<={my,mx[10:4]};
     word_data[{my[0],mx[4]}]<=tap_rsp_word;
     state<=CHECK;
    end
   endcase
   // Existing cache service guarantees >=1-cycle response, never same-cycle.
   if(tap_rsp_valid&&state!=WAIT_WORD)fault<=1;
  end
 end
 bilinear_channel #(.BITS(5)) red(.clk(clk),.v({valid_pipe[4:0],fire}),
  .p00(p00[15:11]),.p10(p10[15:11]),.p01(p01[15:11]),.p11(p11[15:11]),.fx(ax),.fy(ay),.q(rsp_data[15:11]));
 bilinear_channel #(.BITS(6)) green(.clk(clk),.v({valid_pipe[4:0],fire}),
  .p00(p00[10:5]),.p10(p10[10:5]),.p01(p01[10:5]),.p11(p11[10:5]),.fx(ax),.fy(ay),.q(rsp_data[10:5]));
 bilinear_channel #(.BITS(5)) blue(.clk(clk),.v({valid_pipe[4:0],fire}),
  .p00(p00[4:0]),.p10(p10[4:0]),.p01(p01[4:0]),.p11(p11[4:0]),.fx(ax),.fy(ay),.q(rsp_data[4:0]));
endmodule

// Four ordered reads through the existing checked row-cache service.
// One input token reserves four responses. No new clock domain or bulk RAM.
// The one-cycle group handoff deliberately isolates upstream ready from
// tag/adapter/tap_ready, which was a critical path in the NN baseline.
module bilinear_rgb565_sampler_serial(
 input wire clk,rst,in_valid,output wire in_ready,
 input wire [10:0] in_x,in_y,input wire [15:0] in_fx,in_fy,
 input wire [31:0] in_fid,in_epoch,
 output wire tap_valid,input wire tap_ready,
 output wire [10:0] tap_x,tap_y,output wire [31:0] tap_fid,tap_epoch,
 input wire tap_rsp_valid,input wire [15:0] tap_rsp_data,
 output wire rsp_valid,output wire [15:0] rsp_data,output wire busy,output reg fault
);
 localparam integer GROUPS=8;
 reg issuing;reg [1:0] issue_phase,return_phase;
 reg [10:0] x0,x1,y0,y1;reg [31:0] fid,epoch;
 reg [2:0] weight_wr,weight_rd;
 reg [3:0] groups;
 reg [5:0] outstanding;
 reg [31:0] weights[0:GROUPS-1] /* synthesis syn_ramstyle="registers" */;
 reg [15:0] p00,p10,p01;
 reg [5:0] valid_pipe;
 assign in_ready=!issuing&&(groups<GROUPS)&&!fault;
 wire take_input=in_valid&&in_ready;
 assign tap_valid=issuing;
 assign tap_x=issue_phase[0]?x1:x0;
 assign tap_y=issue_phase[1]?y1:y0;
 assign tap_fid=fid;assign tap_epoch=epoch;
 wire take_tap=tap_valid&&tap_ready;
 wire take_return=tap_rsp_valid&&(outstanding!=0);
 wire group_return=take_return&&(return_phase==3);
 assign rsp_valid=valid_pipe[5];
 assign busy=issuing||(groups!=0)||(outstanding!=0)||(|valid_pipe);
 wire [31:0] current_weights=weights[weight_rd];
 always @(posedge clk)begin
  if(rst)begin
   issuing<=0;issue_phase<=0;return_phase<=0;x0<=0;x1<=0;y0<=0;y1<=0;fid<=0;epoch<=0;
   weight_wr<=0;weight_rd<=0;groups<=0;outstanding<=0;fault<=0;
   p00<=0;p10<=0;p01<=0;valid_pipe<=0;
  end else begin
   valid_pipe<={valid_pipe[4:0],group_return};
   if(take_input)begin
    issuing<=1;issue_phase<=0;x0<=in_x;y0<=in_y;
    x1<=(in_x==1919)?in_x:in_x+1'b1;y1<=(in_y==1079)?in_y:in_y+1'b1;
    fid<=in_fid;epoch<=in_epoch;
    weights[weight_wr]<={in_fx,in_fy};weight_wr<=weight_wr+1'b1;
    if(in_x>=1920||in_y>=1080)fault<=1;
   end
   if(take_tap)begin
    issue_phase<=issue_phase+1'b1;
    if(issue_phase==3)issuing<=0;
   end
   if(take_return)begin
    return_phase<=return_phase+1'b1;
    case(return_phase)
     0:p00<=tap_rsp_data;
     1:p10<=tap_rsp_data;
     2:p01<=tap_rsp_data;
     3:weight_rd<=weight_rd+1'b1;
    endcase
   end
   if(tap_rsp_valid&&!take_return)fault<=1;
   case({take_input,group_return})
    2'b10:groups<=groups+1'b1;
    2'b01:groups<=groups-1'b1;
    default:groups<=groups;
   endcase
   case({take_tap,take_return})
    2'b10:outstanding<=outstanding+1'b1;
    2'b01:outstanding<=outstanding-1'b1;
    default:outstanding<=outstanding;
   endcase
  end
 end
 // Independent channel arithmetic, never interpolate the packed RGB word.
 bilinear_channel #(.BITS(5)) red(.clk(clk),.v({valid_pipe[4:0],group_return}),
  .p00(p00[15:11]),.p10(p10[15:11]),.p01(p01[15:11]),.p11(tap_rsp_data[15:11]),
  .fx(current_weights[31:16]),.fy(current_weights[15:0]),.q(rsp_data[15:11]));
 bilinear_channel #(.BITS(6)) green(.clk(clk),.v({valid_pipe[4:0],group_return}),
  .p00(p00[10:5]),.p10(p10[10:5]),.p01(p01[10:5]),.p11(tap_rsp_data[10:5]),
  .fx(current_weights[31:16]),.fy(current_weights[15:0]),.q(rsp_data[10:5]));
 bilinear_channel #(.BITS(5)) blue(.clk(clk),.v({valid_pipe[4:0],group_return}),
  .p00(p00[4:0]),.p10(p10[4:0]),.p01(p01[4:0]),.p11(tap_rsp_data[4:0]),
  .fx(current_weights[31:16]),.fy(current_weights[15:0]),.q(rsp_data[4:0]));
`ifndef SYNTHESIS
 always @(posedge clk)if(!rst)begin
  if(groups>GROUPS||outstanding>4*GROUPS)$fatal(1,"Bilinear credit overflow");
  if(group_return&&groups==0)$fatal(1,"Bilinear group underflow");
  if(outstanding>4*groups)$fatal(1,"Bilinear orphan responses");
 end
`endif
endmodule

// H = (p0 << 16) + (p1-p0)*fx, V = (H0 << 16)+(H1-H0)*fy.
// No intermediate rounding. Final round-to-nearest, half upward, Q32.
// Signed intermediates include explicit positive sign bits on all operands.
module bilinear_channel #(parameter integer BITS=6)(
 input wire clk,input wire [5:0] v,
 input wire [BITS-1:0] p00,p10,p01,p11,input wire [15:0] fx,fy,
 output reg [BITS-1:0] q
);
 localparam integer HW=BITS+16;
 reg [BITS-1:0] a00,a10,a01,a11;reg [15:0] ax,ay;
 reg signed [BITS+17:0] bx0,bx1;
 reg [HW-1:0] b0,b1;reg [15:0] by;
 reg [HW-1:0] h0,h1;reg [15:0] cy;
 reg signed [HW:0] delta;reg [HW+15:0] dbase;reg [15:0] dy;
 reg signed [HW+17:0] product;reg [HW+15:0] ebase;
 wire signed [HW+17:0] rounded=$signed({2'b0,ebase})+product+(40'sd2147483648);
 always @(posedge clk)begin
  if(v[0])begin a00<=p00;a10<=p10;a01<=p01;a11<=p11;ax<=fx;ay<=fy;end
  if(v[1])begin
   bx0<=($signed({1'b0,a10})-$signed({1'b0,a00}))*$signed({1'b0,ax});
   bx1<=($signed({1'b0,a11})-$signed({1'b0,a01}))*$signed({1'b0,ax});
   b0<={a00,16'b0};b1<={a01,16'b0};by<=ay;
  end
  if(v[2])begin h0<=$signed({2'b0,b0})+bx0;h1<=$signed({2'b0,b1})+bx1;cy<=by;end
  if(v[3])begin delta<=$signed({1'b0,h1})-$signed({1'b0,h0});dbase<={h0,16'b0};dy<=cy;end
  if(v[4])begin product<=delta*$signed({1'b0,dy});ebase<=dbase;end
  if(v[5])q<=rounded[32+:BITS];
 end
endmodule
