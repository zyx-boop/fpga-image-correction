// Serialized F24 Brown5, real source reads and acknowledged full-frame writes.
module frame_render #(parameter integer OUTPUT_WIDTH=1920,OUTPUT_HEIGHT=1080)(
 input wire clk,rst_n,start,abort_req,input wire [1:0] source_bank,target_bank,
 input wire [15:0] source_frame_id,source_epoch,input wire allow_new_ar,
 output wire [27:0] araddr,output wire [3:0] arlen,output wire arvalid,input wire arready,
 input wire [255:0] rdata,input wire rvalid,rlast,
 output wire [27:0] awaddr,output wire [3:0] awlen,output wire awvalid,input wire awready,
 output wire [255:0] wdata,output wire [31:0] wstrb,input wire wready,wlast,
 output reg done,error,output reg [7:0] error_code,
 output wire reader_quiet,writer_quiet,return_pending,read_inflight,write_inflight,
 output reg [21:0] pixel_count,output wire [17:0] word_ack_count,
 output wire [27:0] next_write_addr,output reg [5:0] token_count,
 output reg eof_seen,output reg [63:0] render_cycles,
 output wire [31:0] cache_hits,cache_misses,
 output reg [31:0] coord_inputs,coord_outputs,output reg [3:0] stage
);
 localparam PIXELS=OUTPUT_WIDTH*OUTPUT_HEIGHT,WORDS=PIXELS/16;
 localparam IDLE=0,LAUNCH=1,COORD=2,READ_REQUEST=3,PIXEL=4,DRAIN=5,COMPLETE=6,ABORT_DRAIN=7,FAILED=8;
 reg [10:0] u,v;reg [15:0] fid,epoch;reg [1:0] src,dst;reg aborted,completion_recorded;
 reg signed[12:0] held_x,held_y;
 wire ci_valid=stage==LAUNCH&&!aborted&&!abort_req&&!error;
 wire ci_ready,co_valid,co_oob,co_sat,co_input_oob;
 wire signed[12:0] co_x,co_y;wire[10:0] co_u,co_v;wire[15:0] co_fid;
 wire co_sof,co_eol,co_eof;
 wire last=(u==OUTPUT_WIDTH-1 && v==OUTPUT_HEIGHT-1);
 wire r_req=stage==READ_REQUEST&&!aborted&&!abort_req&&!error;
 wire r_ready,r_pixel_valid;wire[15:0] r_pixel;
 wire w_pixel_ready,w_done,r_error,w_error;wire[7:0] r_code,w_code;
 wire [21:0] w_pixels;
 wire w_pixel_valid=stage==PIXEL&&r_pixel_valid&&!aborted&&!abort_req&&!error;
 wire r_pixel_ready=(stage==PIXEL&&w_pixel_ready)||aborted||abort_req||error;
 wire accepted_pixel=w_pixel_valid&&w_pixel_ready;
 wire ci_take=ci_valid&&ci_ready;
 wire bad_metadata=co_u!=u || co_v!=v || co_fid!=fid || co_sof!=(u==0&&v==0) ||
   co_eol!=(u==OUTPUT_WIDTH-1) || co_eof!=last || co_sat || co_input_oob;
 wire close_ok=token_count==0 && reader_quiet && writer_quiet && !return_pending &&
  !arvalid&&!rvalid&&!awvalid&&!read_inflight&&!write_inflight;
 model_a_coordinate_map #(.NORMALIZED_FRAC(24)) u_f24(
 .clk(clk),.rst(!rst_n),.in_valid(ci_valid),.in_ready(ci_ready),.out_ready(1'b1),.out_valid(co_valid),
 .in_u(u),.in_v(v),.in_frame_id(fid),.in_sof(u==0&&v==0),.in_eol(u==OUTPUT_WIDTH-1),.in_eof(last),
 .src_x_q(),.src_y_q(),.src_x_nn(co_x),.src_y_nn(co_y),.source_oob(co_oob),
 .input_oob(co_input_oob),.saturation_seen(co_sat),.out_u(co_u),.out_v(co_v),
 .out_frame_id(co_fid),.out_sof(co_sof),.out_eol(co_eol),.out_eof(co_eof));
 source_pixel_reader u_source(.clk(clk),.rst_n(rst_n),.context_start(start),.source_bank(source_bank),
 .abort_req(aborted||abort_req||error),.allow_new_ar(allow_new_ar),
 .request_valid(r_req),.request_ready(r_ready),.src_x(held_x),.src_y(held_y),
 .pixel_valid(r_pixel_valid),.pixel_ready(r_pixel_ready),.pixel(r_pixel),
 .araddr(araddr),.arlen(arlen),.arvalid(arvalid),.arready(arready),
 .rdata(rdata),.rvalid(rvalid),.rlast(rlast),.quiet(reader_quiet),
 .rd_inflight(read_inflight),.return_pending(return_pending),.error(r_error),.error_code(r_code),
 .hit_count(cache_hits),.miss_count(cache_misses));
 corrected_frame_writer #(.FRAME_PIXELS(PIXELS)) u_target(
 .clk(clk),.rst_n(rst_n),.start(start),.target_bank(target_bank),.abort_req(aborted||abort_req||error),
 .pixel_valid(w_pixel_valid),.pixel_ready(w_pixel_ready),.pixel(r_pixel),.pixel_eof(last),
 .awaddr(awaddr),.awlen(awlen),.awvalid(awvalid),.awready(awready),
 .wdata(wdata),.wstrb(wstrb),.wready(wready),.wlast(wlast),.quiet(writer_quiet),
 .write_inflight(write_inflight),.done(w_done),.error(w_error),.error_code(w_code),
 .pixel_count(w_pixels),.word_ack_count(word_ack_count),.next_addr(next_write_addr));
 always @(posedge clk)begin
  if(!rst_n)begin
   stage<=IDLE;u<=0;v<=0;fid<=0;epoch<=0;src<=0;dst<=0;aborted<=0;completion_recorded<=0;
   held_x<=0;held_y<=0;done<=0;error<=0;error_code<=0;
   pixel_count<=0;token_count<=0;eof_seen<=0;render_cycles<=0;coord_inputs<=0;coord_outputs<=0;
  end else begin
   if(!completion_recorded&&stage!=IDLE&&stage!=COMPLETE&&stage!=FAILED)render_cycles<=render_cycles+1'b1;
   if(ci_take)begin coord_inputs<=coord_inputs+1'b1;token_count<=token_count+1'b1;end
   if(co_valid)begin
    coord_outputs<=coord_outputs+1'b1;
    if(token_count!=0)token_count<=token_count-1'b1;
    else begin error<=1;error_code<=8'h31;aborted<=1;end
   end
   if(abort_req||r_error||w_error)begin
    aborted<=1;error<=1;
    if(!error)error_code<=r_error?r_code:w_error?w_code:8'h01;
   end
   case(stage)
    IDLE:if(start)begin
     fid<=source_frame_id;epoch<=source_epoch;src<=source_bank;dst<=target_bank;
     if(source_bank==target_bank||source_bank>2||target_bank>2)begin error<=1;error_code<=8'h32;stage<=ABORT_DRAIN;end
     else stage<=LAUNCH;
    end
    LAUNCH:if(ci_take)stage<=COORD;
    COORD:if(co_valid)begin
     if(bad_metadata)begin error<=1;error_code<=8'h33;aborted<=1;stage<=ABORT_DRAIN;end
     else begin held_x<=co_x;held_y<=co_y;stage<=READ_REQUEST;end
    end
    READ_REQUEST:if(r_req&&r_ready)stage<=PIXEL;
    PIXEL:if(accepted_pixel)begin
     pixel_count<=pixel_count+1'b1;
     if(last)begin eof_seen<=1;stage<=DRAIN;end
     else begin
      if(u==OUTPUT_WIDTH-1)begin u<=0;v<=v+1'b1;end else u<=u+1'b1;
      stage<=LAUNCH;
     end
    end
    DRAIN:if(close_ok&&w_done&&pixel_count==PIXELS&&w_pixels==PIXELS&&word_ack_count==WORDS&&
                 eof_seen&&coord_inputs==PIXELS&&coord_outputs==PIXELS&&!error)begin
     done<=1;stage<=COMPLETE;completion_recorded<=1;
    end
    COMPLETE:stage<=COMPLETE;
    ABORT_DRAIN:if(close_ok)stage<=FAILED;
    FAILED:stage<=FAILED;
    default:begin error<=1;error_code<=8'h3f;stage<=ABORT_DRAIN;end
   endcase
   if((aborted||abort_req||r_error||w_error||error) && stage!=IDLE && stage!=FAILED && stage!=ABORT_DRAIN)begin
    done<=0;stage<=ABORT_DRAIN;
   end
   if(stage!=IDLE&&(source_frame_id!=fid||source_epoch!=epoch||source_bank!=src||target_bank!=dst))begin
    error<=1;error_code<=8'h34;aborted<=1;done<=0;stage<=ABORT_DRAIN;
   end
  end
 end
endmodule
