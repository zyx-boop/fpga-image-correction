// Shared bounded unsigned division for once-per-frame color adaptation.
// No combinational variable division is synthesized into the pixel path.
module color_ratio_divider(
 input wire clk,rst_n,start,input wire [35:0] numerator,input wire [23:0] denominator,
 output reg busy,done,divide_by_zero,output reg [35:0] quotient
);
 reg [35:0] work;
 reg [24:0] remainder;
 reg [23:0] divisor;
 reg [5:0] left;
 wire [24:0] shifted={remainder[23:0],work[35]};
 wire ge=shifted>={1'b0,divisor};
 wire [35:0] next_work={work[34:0],ge};
 always @(posedge clk)begin
  if(!rst_n)begin busy<=0;done<=0;divide_by_zero<=0;quotient<=0;work<=0;remainder<=0;divisor<=0;left<=0;end
  else begin
   done<=0;
   if(start&&!busy)begin
    divide_by_zero<=denominator==0;
    if(denominator==0)begin quotient<=0;done<=1;end
    else begin work<=numerator;divisor<=denominator;remainder<=0;left<=36;busy<=1;end
   end else if(busy)begin
    remainder<=ge?shifted-{1'b0,divisor}:shifted;work<=next_work;left<=left-1'b1;
    if(left==1)begin quotient<=next_work;busy<=0;done<=1;end
   end
  end
 end
endmodule
