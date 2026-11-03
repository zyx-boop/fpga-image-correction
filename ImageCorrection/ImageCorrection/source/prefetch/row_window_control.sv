// Model-A-specific schedule only. F24 still computes every source coordinate.
// Source rows are protected for a whole output row, not merely individual reads.
module row_window_control #(parameter BOUNDS_FILE="data/row_bounds.hex")(
 input wire clk,rst,frame_open,frame_close,stop_new_rows,input wire [31:0] frame_epoch,
 input wire launch_take,output_take,pipeline_quiet,prefetch_quiet,
 output wire launch_enable,window_ready,idle,rows_done,stopped,output reg fault,
 output reg [10:0] window_lo,window_hi,output_y,
 output wire row_req_valid,input wire row_req_ready,output wire [10:0] row_req_y,
 input wire row_done_valid,output wire row_done_ready,input wire [10:0] row_done_y,input wire [31:0] row_done_epoch,
  input wire [10:0] lookup_y,output wire lookup_ready,output wire [10:0] lookup_tag_y,output wire [31:0] lookup_epoch
 );
 localparam [3:0] IDLE=0,ROM_WAIT=1,BOUNDS=2,SCAN=3,FILL=4,RUN=5,STOP=6,FINISHED=7,HALT=8;
 reg [3:0] state;reg [10:0] cursor,launched,retired,fill_y;reg [31:0] epoch;
 reg fill_pending;reg [127:0] valid_rows;
 reg [10:0] tags[0:127] /* synthesis syn_ramstyle="lut_ram" */;
 reg [21:0] bounds[0:1079] /* synthesis syn_ramstyle="block_ram" */;
 reg [21:0] bound_q;
 // D146: ordering is checked separately, so bounded 11-bit hi-lo>126
 // equals the original 32-bit-promoted hi-lo+1>=128 for every legal range.
 wire [10:0] bounds_span=bound_q[10:0]-bound_q[21:11];
 initial $readmemh(BOUNDS_FILE,bounds);
 always @(posedge clk)bound_q<=bounds[output_y];
 // G8: the selected Tag and all safety decisions remain SHARED, as in C1.
 wire [10:0] selected_tag=tags[cursor[6:0]];
 wire selected_valid=valid_rows[cursor[6:0]];
 wire cached=selected_valid&&selected_tag==cursor;
 wire selected_protected=selected_valid&&selected_tag>=window_lo&&selected_tag<=window_hi;
 assign lookup_ready=valid_rows[lookup_y[6:0]];
 assign lookup_tag_y=tags[lookup_y[6:0]];assign lookup_epoch=epoch;
 assign idle=state==IDLE&&!fault;
 assign rows_done=state==FINISHED&&!fault;
 assign stopped=state==STOP&&prefetch_quiet&&!fill_pending;
 assign window_ready=state==RUN&&!stop_new_rows&&!fault;
 assign launch_enable=window_ready&&launched<1920;
 assign row_req_valid=state==SCAN&&!cached&&!stop_new_rows&&!fault;
 assign row_req_y=cursor;
 assign row_done_ready=(state==FILL||state==STOP)&&fill_pending&&!fault;
 wire safe_clear=row_req_valid&&row_req_ready&&pipeline_quiet&&!selected_protected;
 wire safe_commit=row_done_valid&&row_done_ready&&row_done_y==fill_y&&row_done_epoch==epoch;
 wire [15:0] clear_mask=16'h0001<<cursor[3:0];
 wire [15:0] commit_mask=16'h0001<<fill_y[3:0];
 genvar group_index;
 generate for(group_index=0;group_index<8;group_index=group_index+1)begin:g_bitmap_group
  wire clear_here=safe_clear&&cursor[6:4]==group_index;
  wire commit_here=safe_commit&&fill_y[6:4]==group_index;
  wire update_here=clear_here||commit_here;
  wire [15:0] next_bits=(valid_rows[group_index*16+:16]&~(clear_here?clear_mask:16'h0000))|
                        (commit_here?commit_mask:16'h0000);
  always @(posedge clk)begin
   if(rst)valid_rows[group_index*16+:16]<=0;
   else if(!fault)begin
    if(frame_open)begin
     if(state==IDLE&&pipeline_quiet&&prefetch_quiet)valid_rows[group_index*16+:16]<=0;
    end else if(!frame_close)begin
     if(update_here)valid_rows[group_index*16+:16]<=next_bits;
    end
   end
  end
 end endgenerate
 always @(posedge clk)begin
  if(rst)begin
   state<=IDLE;fault<=0;window_lo<=0;window_hi<=0;output_y<=0;cursor<=0;epoch<=0;
   launched<=0;retired<=0;fill_pending<=0;fill_y<=0;
  end else if(!fault)begin
   if(frame_open)begin
    if(state!=IDLE||!pipeline_quiet||!prefetch_quiet)begin fault<=1;state<=HALT;end
    else begin state<=ROM_WAIT;epoch<=frame_epoch;output_y<=0;launched<=0;retired<=0;fill_pending<=0;end
   end else if(frame_close)begin
    if(!pipeline_quiet||!prefetch_quiet||fill_pending||row_done_valid)begin fault<=1;state<=HALT;end
    else state<=IDLE;
   end else begin
    if(launch_take)launched<=launched+1'b1;
    if(output_take)retired<=retired+1'b1;
    case(state)
     ROM_WAIT:state<=BOUNDS;
     BOUNDS:begin
      window_lo<=bound_q[21:11];window_hi<=bound_q[10:0];cursor<=bound_q[21:11];state<=SCAN;
      if(bound_q[21:11]>bound_q[10:0]||bound_q[10:0]>=11'd1080||bounds_span>11'd126)begin fault<=1;state<=HALT;end
     end
     SCAN:begin
      if(cached)begin if(cursor==window_hi)state<=RUN;else cursor<=cursor+1'b1;end
      else if(row_req_valid&&row_req_ready)begin
       if(!pipeline_quiet||selected_protected)begin fault<=1;state<=HALT;end
       else begin fill_pending<=1;fill_y<=cursor;state<=FILL;end
      end
     end
     RUN:if(retired==1920&&pipeline_quiet&&prefetch_quiet)begin
      if(launched!=1920)begin fault<=1;state<=HALT;end
      else if(output_y==1079)state<=FINISHED;
      else begin output_y<=output_y+1'b1;launched<=0;retired<=0;state<=ROM_WAIT;end
     end
     STOP:if(prefetch_quiet&&!row_done_valid)fill_pending<=0; // Unbegun accepted request may cancel.
     default:begin end
    endcase
    if(row_done_valid&&row_done_ready)begin
     if(row_done_y!=fill_y||row_done_epoch!=epoch)begin fault<=1;state<=HALT;end
     else begin tags[fill_y[6:0]]<=fill_y;fill_pending<=0;if(state==FILL)state<=SCAN;end
    end
    if(stop_new_rows&&state!=IDLE&&state!=HALT)state<=STOP;
    if((launch_take&&!launch_enable)||(output_take&&(retired>=1920||(retired>=launched&&!launch_take))))begin fault<=1;state<=HALT;end
   end
  end
 end
endmodule
