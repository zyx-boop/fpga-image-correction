// Independent NN pixel reader. Ordered, >=1-cycle row-pixel service; no DDR/Owner.
// abort marks the context; upstream must stop issuance, drain F24 here, then close.
module nn_stream_reader #(parameter integer DEPTH=32, parameter integer BILINEAR=0)(
 input wire clk,rst,ctx_valid,
 input wire [31:0] ctx_fid,ctx_job,ctx_epoch,ctx_first_index,
 output wire ctx_ready,output reg active,closing,aborted,error,
 output wire [31:0] reserved_count,output wire [31:0] pending_count,
 input wire close_req,abort_req,in_valid,output wire in_ready,
 input wire signed [12:0] in_x,in_y,
 input wire signed [27:0] in_x_q,in_y_q,
 output wire [15:0] mem_fx,mem_fy,
 input wire [31:0] in_fid,in_job,in_epoch,in_index,
 input wire in_sof,in_eol,in_eof,in_oor,
 output wire mem_valid,input wire mem_ready,
 output wire [10:0] mem_x,mem_y,output wire [31:0] mem_fid,mem_epoch,
 input wire mem_rsp_valid,input wire [15:0] mem_rsp_data,
 output wire out_valid,input wire out_ready,
 output wire [31:0] out_fid,out_job,out_epoch,out_index,
 output wire out_sof,out_eol,out_eof,out_oor,output wire [15:0] out_pixel);
 localparam integer PW=$clog2(DEPTH);
 localparam integer CW=BILINEAR?58:26;
 // Q16 arithmetic floor, with all sixteen fractional bits retained.
 wire [CW-1:0] input_coordinates=BILINEAR?
   {in_x_q[15:0],in_y_q[15:0],in_x_q[27],in_x_q[27:16],in_y_q[27],in_y_q[27:16]}:
   {in_x,in_y};
 reg [31:0] fid,job,epoch,next_index;
 reg [PW-1:0] wr_ptr,issue_ptr,out_ptr,request_wr,request_rd;
 // Bounded by pending <= reserved <= DEPTH; external identity stays 32 bit.
 localparam integer INFLIGHT_W=$clog2(DEPTH+1);
 reg [INFLIGHT_W-1:0] to_issue,reserved_internal;
 assign reserved_count={{(32-INFLIGHT_W){1'b0}},reserved_internal};
 reg [INFLIGHT_W-1:0] inflight;
 reg [DEPTH-1:0] ready_bits;
 // Deliberately small register queues, not another bulk DRM allocation.
 reg [35:0] metadata[0:DEPTH-1] /* synthesis syn_ramstyle="registers" */;
 reg [CW-1:0] coordinates[0:DEPTH-1] /* synthesis syn_ramstyle="registers" */;
 reg [15:0] pixels[0:DEPTH-1] /* synthesis syn_ramstyle="registers" */;
 reg [PW-1:0] return_slots[0:DEPTH-1] /* synthesis syn_ramstyle="registers" */;
 wire take_out=out_valid&&out_ready;
 // D145: decode each slot once and reduce parallel masked terms. Keep this
 // combinational: a returned pixel remains visible on the same write edge.
 (* syn_keep=1 *) wire [DEPTH-1:0] output_select,issue_select;
 wire [DEPTH-1:0] output_ready_terms,issue_oor_terms;
 genvar select_slot;
 generate for(select_slot=0;select_slot<DEPTH;select_slot=select_slot+1) begin:g_slot_select
   assign output_select[select_slot]=(out_ptr==select_slot);
   assign issue_select[select_slot]=(issue_ptr==select_slot);
   assign output_ready_terms[select_slot]=output_select[select_slot]&&ready_bits[select_slot];
   assign issue_oor_terms[select_slot]=issue_select[select_slot]&&metadata[select_slot][0];
 end endgenerate
 // D146: retain the current issue head, with write-through for the first
 // accepted item and for consuming the sole item while accepting its successor.
 // This is a redundant cache of the queue head, not an extra request stage.
 reg [CW-1:0] issue_head_coordinates;
 reg issue_head_oor;
 wire [PW-1:0] next_issue_ptr=bump(issue_ptr);
 (* syn_keep=1 *) wire [DEPTH-1:0] next_issue_select;
 reg [CW-1:0] next_issue_coordinates;
 reg next_issue_oor;
 integer next_slot;
 generate for(select_slot=0;select_slot<DEPTH;select_slot=select_slot+1) begin:g_next_issue_select
   assign next_issue_select[select_slot]=issue_select[(select_slot==0)?DEPTH-1:select_slot-1];
 end endgenerate
 always_comb begin
   next_issue_coordinates={CW{1'b0}};next_issue_oor=1'b0;
   for(next_slot=0;next_slot<DEPTH;next_slot=next_slot+1)
     if(next_issue_select[next_slot]) begin
       next_issue_coordinates=next_issue_coordinates|coordinates[next_slot];
       next_issue_oor=next_issue_oor|metadata[next_slot][0];
     end
 end
 reg output_ready,selected_issue_oor;
 always_comb begin
   output_ready=|output_ready_terms;
   selected_issue_oor=|issue_oor_terms;
`ifndef SYNTHESIS
   // Preserve the baseline's four-state indexed-read semantics for a corrupt
   // or unknown pointer; no X optimism in diagnostic/equivalence simulation.
   if((^out_ptr)===1'bx || out_ptr>=DEPTH) output_ready=ready_bits[out_ptr];
   if((^issue_ptr)===1'bx || issue_ptr>=DEPTH) selected_issue_oor=metadata[issue_ptr][0];
`endif
 end
 // Cache exactly ready_bits[out_ptr] AFTER each original bitmap update.
 // This removes the current 32-slot ready selection from the acceptance
 // control chain; it adds no output cycle and never delays a returned pixel.
 (* syn_preserve=1 *) reg output_ready_head;
 reg visible_output_ready;
 always_comb begin
   visible_output_ready=output_ready_head;
`ifndef SYNTHESIS
   if((^out_ptr)===1'bx || out_ptr>=DEPTH)visible_output_ready=output_ready;
`endif
 end
 assign out_valid=active && reserved_count!=0 && visible_output_ready;
 // Preserve the same-cycle register FIFO read, including a returned pixel
 // becoming ready on its write edge. An addressable array read was mapped
 // to a look-ahead output FF without write-through, exposing the old slot.
 reg [35:0] output_metadata;
 reg [15:0] output_pixel;
 integer output_slot;
 always_comb begin
   output_metadata=36'd0;
   output_pixel=16'd0;
   for(output_slot=0;output_slot<DEPTH;output_slot=output_slot+1)
     if(output_select[output_slot]) begin
       output_metadata=output_metadata|metadata[output_slot];
       output_pixel=output_pixel|pixels[output_slot];
     end
 end
 assign {out_index,out_sof,out_eol,out_eof,out_oor}=output_metadata;
 assign out_pixel=out_oor?16'b0:output_pixel;
 assign out_fid=fid;assign out_job=job;assign out_epoch=epoch;
 assign in_ready=active&&!closing&&!error&&((reserved_count<DEPTH)||take_out);
 wire take_in=in_valid&&in_ready;
 // Outside the closed image rectangle is black; edge neighbours clamp.
 // map_oor describes rounded NN coordinates and is deliberately not used here.
 wire input_oor=BILINEAR?(in_x_q<0||in_x_q>28'sd125763584||in_y_q<0||in_y_q>28'sd70713344):
   (in_oor||in_x<0||in_x>=1920||in_y<0||in_y>=1080);
 reg visible_issue_oor;
 always_comb begin
   visible_issue_oor=issue_head_oor;
`ifndef SYNTHESIS
   if((^issue_ptr)===1'bx || issue_ptr>=DEPTH) visible_issue_oor=selected_issue_oor;
`endif
 end
 wire issue_oor=(to_issue!=0)&&visible_issue_oor;
 assign mem_valid=active&&(to_issue!=0)&&!issue_oor;
 // Constant-slot combinational selection is intentional. The addressable
 // memory read template was mapped to a registered look-ahead read without
 // write-through when an empty queue receives its first new coordinate.
 // Keep the original same-cycle FIFO read semantics; no extra stage/credit.
 reg [CW-1:0] issue_coordinates;
 integer coordinate_slot;
 always_comb begin
   issue_coordinates={CW{1'b0}};
   for(coordinate_slot=0;coordinate_slot<DEPTH;coordinate_slot=coordinate_slot+1)
     if(issue_select[coordinate_slot]) issue_coordinates=issue_coordinates|coordinates[coordinate_slot];
 end
 reg [CW-1:0] visible_issue_coordinates;
 always_comb begin
   visible_issue_coordinates=issue_head_coordinates;
`ifndef SYNTHESIS
   // Invalid-cycle coordinates are observational only; preserve their old
   // four-state values for reference checking without a live issue mux in HW.
   if(to_issue==0 || (^to_issue)===1'bx)visible_issue_coordinates=issue_coordinates;
   if((^issue_ptr)===1'bx || issue_ptr>=DEPTH)visible_issue_coordinates=issue_coordinates;
`endif
 end
 assign mem_x=visible_issue_coordinates[23:13];
 assign mem_y=visible_issue_coordinates[10:0];
 generate if(BILINEAR)begin:g_fraction
   assign {mem_fx,mem_fy}=visible_issue_coordinates[57:26];
 end else begin:g_no_fraction
   assign mem_fx=0;assign mem_fy=0;
 end endgenerate
 assign mem_fid=fid;assign mem_epoch=epoch;
 wire take_mem=mem_valid&&mem_ready;
 wire take_issue=issue_oor||take_mem;
 wire take_return=mem_rsp_valid&&(inflight!=0);
 assign pending_count=to_issue+inflight;
 assign ctx_ready=!active&&(reserved_count==0)&&(to_issue==0)&&(inflight==0);
 function [PW-1:0] bump(input [PW-1:0] p);bump=(p==DEPTH-1)?0:p+1'b1;endfunction
 wire [PW-1:0] ready_head_slot=take_out?bump(out_ptr):out_ptr;
 reg ready_head_next;
 always_comb begin
   ready_head_next=ready_bits[ready_head_slot];
   // Same priority as the four nonblocking writes in the bitmap block.
   if(take_in&&wr_ptr==ready_head_slot)ready_head_next=1'b0;
   if(take_issue&&issue_oor&&issue_ptr==ready_head_slot)ready_head_next=1'b1;
   if(take_return&&return_slots[request_rd]==ready_head_slot)ready_head_next=1'b1;
   if(take_out&&out_ptr==ready_head_slot)ready_head_next=1'b0;
 end
 always @(posedge clk)begin
   if(rst)output_ready_head<=1'b0;
   else output_ready_head<=ready_head_next;
 end
 always @(posedge clk) begin
   if(rst) begin issue_head_coordinates<=0;issue_head_oor<=0;end
   else if(take_in&&((to_issue==0)||((to_issue==1)&&take_issue))) begin
     issue_head_coordinates<=input_coordinates;issue_head_oor<=input_oor;
   end else if(take_issue&&(to_issue>1)) begin
     // Explicit same-edge array write bypass, even though normal queue
     // ownership prevents overwriting an occupied successor.
     if(take_in&&(wr_ptr==next_issue_ptr)) begin
       issue_head_coordinates<=input_coordinates;issue_head_oor<=input_oor;
     end else begin
       issue_head_coordinates<=next_issue_coordinates;issue_head_oor<=next_issue_oor;
     end
   end
 end
 always @(posedge clk) begin
   if(rst) begin
     active<=0;closing<=0;aborted<=0;error<=0;reserved_internal<=0;
     fid<=0;job<=0;epoch<=0;next_index<=0;
     wr_ptr<=0;issue_ptr<=0;out_ptr<=0;request_wr<=0;request_rd<=0;
     to_issue<=0;inflight<=0;ready_bits<=0;
   end else begin
     // Sample only the unoccupied return-FIFO tail. Ownership and the tail
     // pointer still advance exclusively on the unchanged accepted request.
     // pending<=reserved<=DEPTH implies take_mem cannot occur when full.
     if(inflight<DEPTH)return_slots[request_wr]<=issue_ptr;
     if(ctx_valid&&ctx_ready) begin
       active<=1;closing<=0;aborted<=0;fid<=ctx_fid;job<=ctx_job;epoch<=ctx_epoch;next_index<=ctx_first_index;
     end
     if(active&&abort_req) aborted<=1;
     if(active&&close_req) closing<=1;
     if(take_in) begin
       if(in_fid!=fid||in_job!=job||in_epoch!=epoch||in_index!=next_index) error<=1;
       metadata[wr_ptr]<={in_index,in_sof,in_eol,in_eof,input_oor};
       coordinates[wr_ptr]<=input_coordinates;
       ready_bits[wr_ptr]<=0;wr_ptr<=bump(wr_ptr);next_index<=next_index+1'b1;
       if(in_eof) closing<=1;
     end
     if(take_issue) begin
       issue_ptr<=bump(issue_ptr);
       if(issue_oor) ready_bits[issue_ptr]<=1;
     end
     if(take_mem) begin
       request_wr<=bump(request_wr);
     end
     if(take_return) begin
       pixels[return_slots[request_rd]]<=mem_rsp_data;
       ready_bits[return_slots[request_rd]]<=1;request_rd<=bump(request_rd);
     end
     if(mem_rsp_valid&&!take_return) error<=1;
     if(take_out) begin ready_bits[out_ptr]<=0;out_ptr<=bump(out_ptr);end
     case({take_in,take_out})
       2'b10:reserved_internal<=reserved_internal+1'b1;
       2'b01:reserved_internal<=reserved_internal-1'b1;
       default:reserved_internal<=reserved_internal;
     endcase
     case({take_in,take_issue})
       2'b10:to_issue<=to_issue+1'b1;
       2'b01:to_issue<=to_issue-1'b1;
       default:to_issue<=to_issue;
     endcase
     case({take_mem,take_return})
       2'b10:inflight<=inflight+1'b1;
       2'b01:inflight<=inflight-1'b1;
       default:inflight<=inflight;
     endcase
     if(active&&closing&&((reserved_count==0)||(reserved_count==1&&take_out))&& !error) begin
       active<=0;closing<=0;
     end
   end
 end
`ifndef SYNTHESIS
 initial if(DEPTH<2) $fatal(1,"DEPTH must be >=2");
 always @(posedge clk) if(!rst) begin
   if(reserved_count>DEPTH||pending_count>reserved_count) $fatal(1,"Reader credit invariant");
   if(take_issue&&to_issue==0) $fatal(1,"Reader issue underflow");
   if(to_issue!=0&&((^issue_ptr)!==1'bx)) begin
     if(issue_head_coordinates!==coordinates[issue_ptr]||issue_head_oor!==metadata[issue_ptr][0])
       $fatal(1,"Reader issue-head cache invariant");
   end
 end
`endif
endmodule
