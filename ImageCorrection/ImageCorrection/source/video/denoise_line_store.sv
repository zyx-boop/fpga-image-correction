// Three registered read stages: 64-deep bank -> group of 8 -> group select.
// LUTRAM leaves scarce block RAM sites to the unchanged DDR row cache.
module denoise_line_store #(parameter WIDTH=1920)(
 input wire clk,rst_n,read_en,write_en,
 input wire [11:0] read_addr,write_addr,input wire [23:0] write_data,
 output reg [23:0] read_data
);
 localparam BANKS=(WIDTH+63)/64,GROUPS=(BANKS+7)/8;
 wire [23:0] bank_read[0:GROUPS*8-1];
 reg [23:0] group_read[0:GROUPS-1];
 reg v1,v2;reg [2:0] low1,group1,group2;
 always @(posedge clk)begin
  if(!rst_n)begin v1<=0;v2<=0;low1<=0;group1<=0;group2<=0;read_data<=0;end
  else begin
   v1<=read_en;v2<=v1;
   if(read_en)begin low1<=read_addr[8:6];group1<=read_addr[11:9];end
   if(v1)group2<=group1;
   if(v2)read_data<=group_read[group2];
  end
 end
 genvar b,g,k;
 generate for(b=0;b<GROUPS*8;b=b+1)begin: banks
  if(b<BANKS)begin: used
   wire [23:0] mem_read;
   wire bank_write=rst_n&&write_en&&write_addr[11:6]==b;
   for(k=0;k<24;k=k+1)begin: bits
    GTP_RAM64X1DP #(.INIT(64'd0)) memory(.WCLK(clk),.WE(bank_write),
     .WADDR(write_addr[5:0]),.RADDR(read_addr[5:0]),.DI(write_data[k]),.DO(mem_read[k]));
   end
   reg [23:0] q;
   always @(posedge clk)begin
    // Always sample: a shared dynamic CE on 1440 native RAM output flops
    // forms a long dedicated CE cascade on this device. v1 below controls
    // consumption, so sampling during idle/reset cannot publish a pixel.
    q<=mem_read;
   end
   assign bank_read[b]=q;
  end else begin: unused_bank
   assign bank_read[b]=24'd0;
  end
 end
 for(g=0;g<GROUPS;g=g+1)begin: groups
  always @(posedge clk)if(rst_n&&v1)group_read[g]<=bank_read[g*8+low1];
 end endgenerate
endmodule
