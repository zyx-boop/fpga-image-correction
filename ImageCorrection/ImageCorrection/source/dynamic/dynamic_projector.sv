// Throughput-one inverse projection, followed by the existing F24 lens model.
// Frame-stable Q24 rotation matrix. Caller credits reserve every pipeline token.
module dynamic_projector(
 input wire clk,rst,in_valid,identity,input wire [287:0] matrix,
 input wire [10:0] in_u,in_v,
 output wire in_ready,output reg out_valid,
 output reg signed [27:0] out_u_q,out_v_q,output reg invalid
);
 `include "model_a_f24.vh"
 assign in_ready=1'b1;
 reg [11:0] valid;
 reg [23:0] meta[0:11]; // exact identity bypass, integer raster and invalid bit
 reg signed [27:0] du,dv;
 reg signed [60:0] px,py;
 reg signed [31:0] nx,ny;
 wire signed [63:0] rx0,rx1,ry0,ry1,rz0,rz1;
 reg signed [65:0] sumx,sumy,sumz,carryx,carryy,carryz;
 reg signed [31:0] rayx,rayy,rayz;
 wire signed [63:0] fx,fy;
 reg [31:0] z8,z9,z10,divisor;
 reg [63:0] ax,ay;
 reg [1:0] signs;
 reg [25:0] div_meta[0:64];
 wire div_valid_x,div_valid_y;
 wire [63:0] qx,qy;
 wire [23:0] final_meta=div_meta[64][23:0];
 wire signed [64:0] uq=(div_meta[64][25]?-$signed({1'b0,qx}):$signed({1'b0,qx}))+65'sd62496001;
 wire signed [64:0] vq=(div_meta[64][24]?-$signed({1'b0,qy}):$signed({1'b0,qy}))+65'sd24341361;
 // Exact nearest/ties-away from an arithmetic floor: positive half increments,
 // negative half does not. This avoids abs -> round -> negate carry chains.
 // Even every signed28 du/dv satisfies |du*4513186/2^24| <= 36105488,
 // strictly inside signed27. Saturation is mathematically unreachable here.
 wire normx_inc=px[23]&&(!px[60]||(|px[22:0]));
 wire normy_inc=py[23]&&(!py[60]||(|py[22:0]));
 wire signed [26:0] normx=$signed(px[50:24])+$signed({26'd0,normx_inc});
 wire signed [26:0] normy=$signed(py[50:24])+$signed({26'd0,normy_inc});
 wire normx_sat=1'b0,normy_sat=1'b0;
 unsigned_divide64_32 dx(.clk(clk),.rst(rst),.start(valid[11]),.numerator(ax),.denominator(divisor),.valid(div_valid_x),.quotient(qx));
 unsigned_divide64_32 dy(.clk(clk),.rst(rst),.start(valid[11]),.numerator(ay),.denominator(divisor),.valid(div_valid_y),.quotient(qy));
 dynamic_mul32 mx0(.clk(clk),.a(matrix[0*32+:32]),.b(nx),.product(rx0));
 dynamic_mul32 mx1(.clk(clk),.a(matrix[1*32+:32]),.b(ny),.product(rx1));
 dynamic_mul32 my0(.clk(clk),.a(matrix[3*32+:32]),.b(nx),.product(ry0));
 dynamic_mul32 my1(.clk(clk),.a(matrix[4*32+:32]),.b(ny),.product(ry1));
 dynamic_mul32 mz0(.clk(clk),.a(matrix[6*32+:32]),.b(nx),.product(rz0));
 dynamic_mul32 mz1(.clk(clk),.a(matrix[7*32+:32]),.b(ny),.product(rz1));
 dynamic_mul32 scale_x(.clk(clk),.a(rayx),.b({{4{MODEL_A_F24_K_OUT_FX[27]}},MODEL_A_F24_K_OUT_FX}),.product(fx));
 dynamic_mul32 scale_y(.clk(clk),.a(rayy),.b({{4{MODEL_A_F24_K_OUT_FY[27]}},MODEL_A_F24_K_OUT_FY}),.product(fy));
 wire [65:0] tx0={{2{rx0[63]}},rx0},tx1={{2{rx1[63]}},rx1},tx2=$signed(matrix[2*32+:32])*66'sd16777216;
 wire [65:0] ty0={{2{ry0[63]}},ry0},ty1={{2{ry1[63]}},ry1},ty2=$signed(matrix[5*32+:32])*66'sd16777216;
 wire [65:0] tz0={{2{rz0[63]}},rz0},tz1={{2{rz1[63]}},rz1},tz2=$signed(matrix[8*32+:32])*66'sd16777216;
 integer i;
 always @(posedge clk)begin
  // Payloads run unconditionally; only valid bits reset or control admission.
  du<=$signed({1'b0,in_u,16'd0})-MODEL_A_F24_K_OUT_CX;
  dv<=$signed({1'b0,in_v,16'd0})-MODEL_A_F24_K_OUT_CY;
  px<=du*MODEL_A_F24_K_OUT_INV_FX;py<=dv*MODEL_A_F24_K_OUT_INV_FY;
  nx<=normx;ny<=normy;
  sumx<=tx0^tx1^tx2;carryx<=((tx0&tx1)|(tx0&tx2)|(tx1&tx2))<<1;
  sumy<=ty0^ty1^ty2;carryy<=((ty0&ty1)|(ty0&ty2)|(ty1&ty2))<<1;
  sumz<=tz0^tz1^tz2;carryz<=((tz0&tz1)|(tz0&tz2)|(tz1&tz2))<<1;
  rayx<=($signed(sumx)+$signed(carryx))>>>24;
  rayy<=($signed(sumy)+$signed(carryy))>>>24;
  rayz<=($signed(sumz)+$signed(carryz))>>>24;
  z8<=rayz;z9<=z8;z10<=z9;
  ax<=fx<0?-fx:fx;ay<=fy<0?-fy:fy;divisor<=z10[31]||z10==0?32'd1:z10;
  signs<={fx<0,fy<0};
  meta[0]<={identity,in_u,in_v,1'b0};
  for(i=1;i<12;i=i+1)meta[i]<=meta[i-1];
  meta[2][0]<=meta[1][0]||normx_sat||normy_sat;
  meta[8][0]<=meta[7][0]||rayz<=0;
  div_meta[0]<={signs,meta[11]};
  for(i=1;i<65;i=i+1)div_meta[i]<=div_meta[i-1];
  if(rst)begin valid<=0;out_valid<=0;invalid<=0;out_u_q<=0;out_v_q<=0;end
  else begin
   valid<={valid[10:0],in_valid};out_valid<=div_valid_x&&div_valid_y;
   if(div_valid_x&&div_valid_y)begin
    if(final_meta[23])begin
     out_u_q<={1'b0,final_meta[22:12],16'd0};out_v_q<={1'b0,final_meta[11:1],16'd0};invalid<=0;
    end else if(final_meta[0]||uq< -65'sd134217728||uq>65'sd134217727||vq< -65'sd134217728||vq>65'sd134217727)begin
     invalid<=1;out_u_q<=0;out_v_q<=0;
    end else begin invalid<=0;out_u_q<=uq[27:0];out_v_q<=vq[27:0];end
   end
  end
 end
endmodule

// Fully pipelined restoring divide, one quotient bit per stage.
// A 34-bit difference supplies BOTH the borrow test and candidate remainder;
// there is no separate compare chain followed by a dependent subtraction.
// Every stage has fixed local control: no high-fanout global datapath CE.
module unsigned_divide64_32(
 input wire clk,rst,start,input wire [63:0] numerator,input wire [31:0] denominator,
 output wire valid,output wire [63:0] quotient
);
 reg [63:0] q[0:64];reg [31:0] d[0:64],r[0:64];reg [64:0] v;
 assign valid=v[64];assign quotient=q[64];
 always @(posedge clk)begin
  q[0]<=numerator;d[0]<=denominator;r[0]<=0;
  if(rst)v<=0;else v<={v[63:0],start};
 end
 genvar s;
 generate for(s=0;s<64;s=s+1)begin:g_stage
  wire [32:0] shifted={r[s],q[s][63]};
  wire [33:0] difference={1'b0,shifted}-{2'b00,d[s]};
  wire subtract_ok=!difference[33];
  always @(posedge clk)begin
   q[s+1]<={q[s][62:0],subtract_ok};
   r[s+1]<=subtract_ok?difference[31:0]:shifted[31:0];
   d[s+1]<=d[s];
  end
 end endgenerate
endmodule
