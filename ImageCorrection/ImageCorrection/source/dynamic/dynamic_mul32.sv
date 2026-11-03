// Exact signed 32x32 multiply, three register boundaries. Four small products
// fit native multipliers; a carry-save stage precedes the final carry chain.
// This prevents a multi-APM cascade and wide product sum in a single clock.
module dynamic_mul32(input wire clk,input wire signed [31:0] a,b,output reg signed [63:0] product);
 wire signed [16:0] al=$signed({1'b0,a[15:0]}),bl=$signed({1'b0,b[15:0]});
 wire signed [15:0] ah=$signed(a[31:16]),bh=$signed(b[31:16]);
 reg [31:0] ll;reg signed [32:0] lh,hl;reg signed [31:0] hh;
 wire [63:0] v0={hh,ll};
 wire [63:0] v1=$signed({{31{lh[32]}},lh})<<16;
 wire [63:0] v2=$signed({{31{hl[32]}},hl})<<16;
 reg [63:0] cs,cc;
 always @(posedge clk)begin
  ll<=al*bl;lh<=al*bh;hl<=ah*bl;hh<=ah*bh;
  cs<=v0^v1^v2;cc<=((v0&v1)|(v0&v2)|(v1&v2))<<1;
  product<=cs+cc;
 end
endmodule
