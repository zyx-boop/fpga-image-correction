// B3: full identity follows acceptance E0, RAM issue E1, response sampled E3.
// A miss is a fatal/quarantined outstanding reference, never a black pixel.
module reader_ram_adapter(
 input wire clk,rst,ctx_active,input wire [31:0] ctx_fid,ctx_epoch,
 input wire row_ready,input wire [10:0] row_tag_y,window_lo,window_hi,input wire [31:0] row_tag_epoch,
 input wire mem_valid,input wire [10:0] mem_x,mem_y,input wire [31:0] mem_fid,mem_epoch,
 output wire mem_ready,mem_rsp_valid,output wire [15:0] mem_rsp_data,
 output wire ram_rd_valid,input wire ram_rd_ready,output wire [10:0] ram_rd_x,ram_rd_y,output wire [31:0] ram_rd_epoch,
 input wire ram_rsp_valid,ram_rsp_hit,input wire [15:0] ram_rsp_data,input wire [10:0] ram_rsp_x,ram_rsp_y,input wire [31:0] ram_rsp_epoch,
  output wire quiet,output reg fault,output reg [7:0] error_code,output reg [31:0] ram_refs,request_count,response_count
 );
 reg v0,v1,v2;reg [10:0] x0,y0,x1,y1,x2,y2;reg [31:0] epoch0,epoch1,epoch2,fid0,fid1,fid2;
 wire identity_ok=ctx_active&&mem_fid==ctx_fid&&mem_epoch==ctx_epoch;
 wire tag_ok=row_tag_y==mem_y&&row_tag_epoch==ctx_epoch;
 wire protected_row=mem_y>=window_lo&&mem_y<=window_hi;
 wire eligible=identity_ok&&row_ready&&tag_ok&&protected_row&&mem_x<1920&&mem_y<1080&&!fault;
 assign mem_ready=eligible&&ram_rd_ready;
 assign ram_rd_valid=mem_valid&&eligible;
 assign ram_rd_x=mem_x;assign ram_rd_y=mem_y;assign ram_rd_epoch=mem_epoch;
 wire take=mem_valid&&mem_ready;
 // D58: preserve bounded combinational byte compares, not registered identity.
 // Every bit is checked on the original return cycle; no guard is removed.
 (* syn_keep=1 *) wire [3:0] return_fid_eq8;
 (* syn_keep=1 *) wire [3:0] return_context_epoch_eq8;
 (* syn_keep=1 *) wire [3:0] return_response_epoch_eq8;
 genvar return_identity_byte;
 generate for(return_identity_byte=0;return_identity_byte<4;return_identity_byte=return_identity_byte+1)begin:g_return_identity
  assign return_fid_eq8[return_identity_byte]=fid2[return_identity_byte*8+:8]==ctx_fid[return_identity_byte*8+:8];
  assign return_context_epoch_eq8[return_identity_byte]=epoch2[return_identity_byte*8+:8]==ctx_epoch[return_identity_byte*8+:8];
  assign return_response_epoch_eq8[return_identity_byte]=ram_rsp_epoch[return_identity_byte*8+:8]==epoch2[return_identity_byte*8+:8];
 end endgenerate
 wire matching=ctx_active&&(&return_fid_eq8)&&(&return_context_epoch_eq8)&&ram_rsp_hit&&ram_rsp_x==x2&&ram_rsp_y==y2&&(&return_response_epoch_eq8);
 wire good_return=ram_rsp_valid&&v2&&matching&&!fault;
 // Preserve ordinary-case X/Z default and explicit modulo-2^32 increments.
 reg [31:0] ram_refs_delta;
 always_comb begin
  case({take,good_return})
   2'b10:ram_refs_delta=32'd1;
   2'b01:ram_refs_delta=32'hffffffff;
   default:ram_refs_delta=32'd0;
  endcase
 end
 assign mem_rsp_valid=good_return;assign mem_rsp_data=ram_rsp_data;
 assign quiet=ram_refs==0&&!v0&&!v1&&!v2&&!ram_rsp_valid&&!fault;
 always @(posedge clk)begin
  if(rst)begin
   v0<=0;v1<=0;v2<=0;x0<=0;y0<=0;x1<=0;y1<=0;x2<=0;y2<=0;epoch0<=0;epoch1<=0;epoch2<=0;fid0<=0;fid1<=0;fid2<=0;
   fault<=0;error_code<=0;ram_refs<=0;request_count<=0;response_count<=0;
  end else begin
   v0<=take;v1<=v0;v2<=v1;
   if(take)begin x0<=mem_x;y0<=mem_y;epoch0<=mem_epoch;fid0<=mem_fid;request_count<=request_count+1'b1;end
   if(v0)begin x1<=x0;y1<=y0;epoch1<=epoch0;fid1<=fid0;end
   if(v1)begin x2<=x1;y2<=y1;epoch2<=epoch1;fid2<=fid1;end
   if(good_return)response_count<=response_count+1'b1;
   ram_refs<=ram_refs+ram_refs_delta;
   if(!fault)begin
    if(mem_valid&&(!identity_ok||mem_x>=1920||mem_y>=1080))begin fault<=1;error_code<=1;end
    else if(mem_valid&&row_ready&&(!tag_ok||!protected_row))begin fault<=1;error_code<=2;end
    else if(ram_rsp_valid!=v2||(ram_rsp_valid&&!matching))begin fault<=1;error_code<=3;end
    else if(good_return&&ram_refs==0)begin fault<=1;error_code<=4;end
   end
  end
 end
endmodule
