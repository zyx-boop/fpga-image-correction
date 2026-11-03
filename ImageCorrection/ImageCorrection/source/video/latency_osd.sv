// 13 cells x 16 pixels. 5x7 glyphs at 2x scale, upper right, 16px margin.
// One output register delays RGB and HS/VS/DE together by one pixel clock.
module latency_osd(
 input wire clk,rst_n,input wire [11:0] x,input wire [10:0] y,
 input wire hs,vs,de,input wire [23:0] rgb,input wire [19:0] digits,
 input wire valid,overflow,
 output reg out_hs,out_vs,out_de,output reg [23:0] out_rgb
);
 wire box=(x>=1696&&x<1904&&y>=16&&y<40);
 wire [11:0] dx=x-12'd1696;
 wire [10:0] dy=y-11'd16;
 wire [3:0] cell_index=dx[7:4];
 wire [2:0] column=dx[3:1];
 wire [3:0] row=dy[4:1];
 reg [7:0] ch;
 reg [34:0] glyph;
 always @* begin
  ch=8'd32;
  case(cell_index)
   0:ch="L";1:ch="A";2:ch="T";
   4:ch=!valid?"-":overflow?"O":8'd48+{4'd0,digits[19:16]};
   5:ch=!valid?"-":overflow?"V":8'd48+{4'd0,digits[15:12]};
   6:ch=!valid?"-":overflow?"E":8'd48+{4'd0,digits[11:8]};
   7:ch=!valid?"-":overflow?"R":8'd48+{4'd0,digits[7:4]};
   8:ch=overflow&&valid?" ":".";
   9:ch=!valid?"-":overflow?" ":8'd48+{4'd0,digits[3:0]};
   11:ch="m";12:ch="s";
   default:ch=" ";
  endcase
  case(ch)
   "0":glyph=35'b01110_10001_10011_10101_11001_10001_01110;
   "1":glyph=35'b00100_01100_00100_00100_00100_00100_01110;
   "2":glyph=35'b01110_10001_00001_00010_00100_01000_11111;
   "3":glyph=35'b11110_00001_00001_01110_00001_00001_11110;
   "4":glyph=35'b00010_00110_01010_10010_11111_00010_00010;
   "5":glyph=35'b11111_10000_10000_11110_00001_00001_11110;
   "6":glyph=35'b01110_10000_10000_11110_10001_10001_01110;
   "7":glyph=35'b11111_00001_00010_00100_01000_01000_01000;
   "8":glyph=35'b01110_10001_10001_01110_10001_10001_01110;
   "9":glyph=35'b01110_10001_10001_01111_00001_00001_01110;
   "L":glyph=35'b10000_10000_10000_10000_10000_10000_11111;
   "A":glyph=35'b01110_10001_10001_11111_10001_10001_10001;
   "T":glyph=35'b11111_00100_00100_00100_00100_00100_00100;
   "m":glyph=35'b00000_00000_11010_10101_10101_10101_10101;
   "s":glyph=35'b00000_00000_01111_10000_01110_00001_11110;
   "O":glyph=35'b01110_10001_10001_10001_10001_10001_01110;
   "V":glyph=35'b10001_10001_10001_10001_10001_01010_00100;
   "E":glyph=35'b11111_10000_10000_11110_10000_10000_11111;
   "R":glyph=35'b11110_10001_10001_11110_10100_10010_10001;
   "-":glyph=35'b00000_00000_00000_11111_00000_00000_00000;
   ".":glyph=35'b00000_00000_00000_00000_00000_00110_00110;
   default:glyph=0;
  endcase
 end
 wire ink=column>=1&&column<=5&&row>=2&&row<=8&&glyph[(8-row)*5+(5-column)];
 always @(posedge clk)begin
  if(!rst_n)begin out_hs<=1;out_vs<=1;out_de<=0;out_rgb<=0;end
  else begin
   out_hs<=hs;out_vs<=vs;out_de<=de;
   out_rgb<=!de?24'd0:box?(ink?24'hffffff:24'h000000):rgb;
  end
 end
endmodule
