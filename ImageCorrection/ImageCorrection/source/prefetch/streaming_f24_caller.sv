// Independent caller around the frozen F24 core. One credit reserves one
// result slot from admission until final consumption. All close/abort work
// drains, including a token accepted on the control request edge.
module streaming_f24_caller #(parameter integer DEPTH=64)(
 input wire clk,rst,ctx_valid,
 input wire [80:0] ctx_pose,
 output wire ctx_ready,
 input wire [31:0] ctx_source_frame_id,ctx_job,ctx_epoch,ctx_first_index,
 input wire in_valid,
 output wire in_ready,
 input wire close_req,abort_req,
 output reg active,closing,
 output wire out_valid,
 input wire out_ready,
 output wire [31:0] out_source_frame_id,out_job,out_epoch,out_pixel_index,
 output wire out_sof,out_eol,out_eof,
 output wire signed [27:0] src_x_q,src_y_q,
 output wire signed [12:0] src_x_nn,src_y_nn,
 output wire source_oob,input_oob,saturation_seen,
 output reg cache_row_mode,
 output reg [31:0] reserved_count,result_count
);
 localparam integer TOTAL=1920*1080;
 localparam integer PW=(DEPTH<=2)?1:$clog2(DEPTH);
 reg [31:0] source_frame_id,job,epoch,index;
 reg [10:0] raster_x,raster_y;
 reg [PW-1:0] tag_write=0,result_write=0,read_ptr=0;
 reg [130:0] tags[0:DEPTH-1];
 reg [84:0] results[0:DEPTH-1];
 wire core_in_ready,core_valid;
 wire signed [27:0] core_x,core_y;
 wire signed [12:0] core_nx,core_ny;
 wire core_oob,core_input_oob,core_sat;
 wire take_output=out_valid&&out_ready;
 // Allow same-edge head reclaim; both counters use one net update.
 assign in_ready=active&&!closing&&rotation_ready&&!rotation_busy&&project_ready&&core_in_ready&&((reserved_count<DEPTH)||take_output);
 wire take_input=in_valid&&in_ready;
 assign ctx_ready=!active&&(reserved_count==0);
 assign out_valid=(result_count!=0);
 assign {out_source_frame_id,out_job,out_epoch,out_pixel_index,out_sof,out_eol,out_eof}=tags[read_ptr];
 assign {src_x_q,src_y_q,src_x_nn,src_y_nn,source_oob,input_oob,saturation_seen}=results[read_ptr];
 wire rotation_ready,rotation_busy,reference_valid,rotation_identity;
 wire [287:0] rotation_matrix;
 pose_rotation_matrix pose_matrix(.clk(clk),.rst(rst),.start(ctx_valid&&ctx_ready),
  .pose_fresh(ctx_pose[80]),.roll(ctx_pose[47:32]),.pitch(ctx_pose[31:16]),.yaw(ctx_pose[15:0]),
  .ready(rotation_ready),.busy(rotation_busy),.reference_valid(reference_valid),.identity(rotation_identity),.matrix(rotation_matrix));
 // Classification affects cache placement only, never coordinates or weights.
 // Conservative near-horizontal relative transforms (including 180 degrees)
 // retain whole row spans as demand-filled 64-pixel blocks. Other transforms
 // use the proven R19 vertical layout. Matrix is already paired to this frame.
 function automatic near_zero(input signed [31:0] a);
  near_zero=(a>=-32'sd4194304)&&(a<=32'sd4194304);
 endfunction
 function automatic strong_diagonal(input signed [31:0] a);
  strong_diagonal=(a>=32'sd14680064)||(a<=-32'sd14680064);
 endfunction
 wire rows_safe=near_zero(rotation_matrix[32+:32])&&near_zero(rotation_matrix[64+:32])&&
  near_zero(rotation_matrix[96+:32])&&near_zero(rotation_matrix[160+:32])&&
  near_zero(rotation_matrix[192+:32])&&near_zero(rotation_matrix[224+:32])&&
  strong_diagonal(rotation_matrix[0+:32])&&strong_diagonal(rotation_matrix[128+:32])&&
  ($signed(rotation_matrix[256+:32])>=32'sd14680064);
 always @(posedge clk)begin
  if(rst)cache_row_mode<=0;
  else if(rotation_ready&&!rotation_busy)cache_row_mode<=rows_safe;
 end
 wire project_ready,project_valid,project_invalid;
 wire signed [27:0] project_u,project_v;
 dynamic_projector projector(.clk(clk),.rst(rst),.in_valid(take_input),.in_ready(project_ready),
  .identity(rotation_identity),.matrix(rotation_matrix),.in_u(raster_x),.in_v(raster_y),
  .out_valid(project_valid),.out_u_q(project_u),.out_v_q(project_v),.invalid(project_invalid));
 wire [10:0] in_u=raster_x;
 wire [10:0] in_v=raster_y;
 // Reserved storage makes out_ready permanently safe: F24 never freezes.
 model_a_coordinate_map #(.NORMALIZED_FRAC(24),.WIDTH(1920),.HEIGHT(1080),.FRACTIONAL_INPUT(1)) f24(
   .clk(clk),.rst(rst),.in_valid(project_valid),.in_ready(core_in_ready),
   .out_ready(1'b1),.out_valid(core_valid),.in_u(in_u),.in_v(in_v),.in_u_q(project_u),.in_v_q(project_v),.in_invalid(project_invalid),
   .in_frame_id(source_frame_id[15:0]),.in_sof(index==0),
   .in_eol(raster_x==1919),.in_eof(index==TOTAL-1),
   .src_x_q(core_x),.src_y_q(core_y),.src_x_nn(core_nx),.src_y_nn(core_ny),
   .source_oob(core_oob),.input_oob(core_input_oob),.saturation_seen(core_sat),
   .out_u(),.out_v(),.out_frame_id(),.out_sof(),.out_eol(),.out_eof());
 function automatic [PW-1:0] advance_ptr(input [PW-1:0] p);
   advance_ptr=(p==DEPTH-1)?0:p+1'b1;
 endfunction
 always @(posedge clk) begin
   if(rst) begin
     active<=0;closing<=0;reserved_count<=0;result_count<=0;
     tag_write<=0;result_write<=0;read_ptr<=0;
     source_frame_id<=0;job<=0;epoch<=0;index<=0;
     raster_x<=0;raster_y<=0;
   end else begin
     if(ctx_valid&&ctx_ready) begin
       active<=1;closing<=0;
       source_frame_id<=ctx_source_frame_id;job<=ctx_job;epoch<=ctx_epoch;
       index<=ctx_first_index;
       // Context-only initialization preserves arbitrary legal first_index.
       // The integrated full-frame caller supplies constant zero here.
       raster_x<=ctx_first_index%1920;raster_y<=ctx_first_index/1920;
     end
     if(active&&(close_req||abort_req)) closing<=1;
     if(take_input) begin
       tags[tag_write]<={source_frame_id,job,epoch,index,index==0,raster_x==1919,index==TOTAL-1};
       tag_write<=advance_ptr(tag_write);index<=index+1'b1;
       if(raster_x==1919)begin raster_x<=0;raster_y<=raster_y+1'b1;end
       else raster_x<=raster_x+1'b1;
       if(index==TOTAL-1) closing<=1;
     end
     if(core_valid) begin
       results[result_write]<={(core_input_oob||core_sat)?-28'sd65536:core_x,
          (core_input_oob||core_sat)?-28'sd65536:core_y,
          core_nx,core_ny,core_oob||core_input_oob||core_sat,1'b0,1'b0};
       result_write<=advance_ptr(result_write);
     end
     if(take_output) read_ptr<=advance_ptr(read_ptr);
     case({take_input,take_output})
       2'b10:reserved_count<=reserved_count+1'b1;
       2'b01:reserved_count<=reserved_count-1'b1;
       default:reserved_count<=reserved_count;
     endcase
     case({core_valid,take_output})
       2'b10:result_count<=result_count+1'b1;
       2'b01:result_count<=result_count-1'b1;
       default:result_count<=result_count;
     endcase
     if(active&&closing&&((reserved_count==0)||((reserved_count==1)&&take_output))) begin
       active<=0;closing<=0;
     end
   end
 end
`ifndef SYNTHESIS
 initial if(DEPTH<2) $fatal(1,"DEPTH must be >=2");
 always @(posedge clk) if(!rst) begin
   if(reserved_count>DEPTH || result_count>reserved_count) $fatal(1,"credit invariant");
   if(core_valid && result_count==DEPTH && !take_output) $fatal(1,"unreserved result");
   if(ctx_valid&&ctx_ready&&ctx_first_index>=TOTAL) $fatal(1,"context index outside 1920x1080 frame");
 end
`endif
endmodule

