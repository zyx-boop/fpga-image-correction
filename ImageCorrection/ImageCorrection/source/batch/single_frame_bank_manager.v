// One capture and one post-render switch. Bank locks survive every local fault.
module single_frame_bank_manager #(
 parameter integer RENDER_PIXELS=2073600,RENDER_WORDS=129600,
 parameter ENABLE_PREVIEW=0,USE_EXPORT_HOLD=0
)(
 input wire clk,rst_n,key_pulse,vblank,export_hold,
 input wire camera_frame_start,camera_frame_finish,camera_frame_good,
 input wire [17:0] camera_end_words,
 input wire [27:0] camera_end_actual,camera_end_expected,
 input wire camera_end_good,camera_write_quiet,
 output wire camera_writer_active,
 output wire [1:0] camera_writer_bank,
 output wire owner_pending_valid,owner_display_valid,owner_dropped,
 output wire [1:0] owner_pending_bank,owner_display_bank,
 output reg render_start,render_bus_owner,
 output reg [1:0] source_bank,target_bank,
 output reg [15:0] source_frame_id,source_epoch,
 output reg source_frozen,
 output wire source_lease_active,allow_new_ar,lease_fault,
 input wire render_arvalid,render_arready,
 input wire [27:0] render_araddr,input wire [3:0] render_arlen,
 input wire render_rvalid,render_rlast,render_return_pending,
 input wire render_reader_quiet,render_writer_quiet,render_done,
 input wire [21:0] render_pixels,input wire [17:0] render_word_acks,
 input wire [27:0] render_next_addr,input wire [5:0] render_tokens,
 input wire render_eof_seen,render_error,display_epoch_quiet,
 output wire display_valid,display_launch_enable,
 output reg display_original,ab_used,bank_error,
 output wire [1:0] display_bank,output reg [3:0] stage,
 output reg [15:0] publish_count
);
 localparam WAIT_KEY=0,ARMED=1,SEAL=2,LEASE=3,RENDER=4,
            READY=5,SHOW_CORRECTED=6,SWITCH_PENDING=7,SHOW_ORIGINAL=8,ERROR_HELD=9;
 function [27:0] base(input [1:0] b);
 begin case(b)0:base=28'h0;1:base=28'hfd200;2:base=28'h1fa400;default:base=28'h0;endcase end
 endfunction
 wire complete_camera = camera_frame_good && camera_end_good && camera_end_words==18'd129600 &&
  camera_end_actual==camera_end_expected && camera_end_expected==base(camera_writer_bank)+28'hfd200;
 wire source_accept = camera_writer_active && camera_frame_finish && complete_camera && !owner_pending_valid;
 wire source_published=source_frozen && owner_display_valid && owner_display_bank==source_bank &&
                       !owner_pending_valid && !camera_writer_active;
 wire owner_hold=source_frozen && owner_display_valid && owner_display_bank==source_bank;
 wire lease_grant;wire [1:0] lease_bank;wire [15:0] lease_epoch;
 reg [15:0] camera_sequence;
 reg published_display_valid;
 reg [1:0] published_display_bank;
 wire preview_context=ENABLE_PREVIEW && stage<=SEAL;
 assign display_valid=preview_context?owner_display_valid:published_display_valid;
 assign display_bank=preview_context?owner_display_bank:published_display_bank;
 // Stop new preview epochs on the key edge. Existing epochs drain completely.
 assign display_launch_enable=!ENABLE_PREVIEW ||
  (stage==WAIT_KEY&&!key_pulse&&!source_frozen) ||
  stage==SHOW_CORRECTED || stage==SWITCH_PENDING || stage==SHOW_ORIGINAL;
 wire context_bad=render_bus_owner && (source_bank==target_bank || source_bank>2 || target_bank>2 ||
  (source_lease_active && lease_bank!=source_bank) || camera_writer_active);
 wire publish_ok=render_done && render_pixels==RENDER_PIXELS && render_word_acks==RENDER_WORDS &&
  render_next_addr==base(target_bank)+RENDER_WORDS*8 && render_tokens==0 && render_eof_seen &&
  render_reader_quiet && render_writer_quiet && !render_arvalid && !render_rvalid &&
  !render_return_pending && !render_error && !lease_fault && !context_bad;
 frame_owner_read_lease u_lease(
 .clk(clk),.rst_n(rst_n),.frame_start(camera_frame_start&&!source_frozen&&stage!=ERROR_HELD&&!render_error),
 .frame_finish(camera_frame_finish),.frame_good(complete_camera),
 .vblank(vblank&&(!ENABLE_PREVIEW||display_epoch_quiet)),.snapshot_hold(owner_hold),
 .writer_active(camera_writer_active),.writer_bank(camera_writer_bank),
 .display_valid(owner_display_valid),.display_bank(owner_display_bank),
 .pending_valid(owner_pending_valid),.pending_bank(owner_pending_bank),.dropped_frame(owner_dropped),
 .lease_req_valid(stage==LEASE),.lease_release_req(1'b0),.abort_req(render_error),
 .lease_req_ready(),.lease_grant(lease_grant),.allow_new_ar(allow_new_ar),.owner_vblank(),
 .lease_active(source_lease_active),.closing(),.fault(lease_fault),.timeout(),.protocol_fault(),
 .rd_inflight(),.lease_bank(lease_bank),.lease_epoch(lease_epoch),.beat_count(),
 .arvalid(render_arvalid),.arready(render_arready),.araddr(render_araddr),.arlen(render_arlen),
 .rvalid(render_rvalid),.rlast(render_rlast),.return_commit_pending(render_return_pending),
 .client_quiescent(render_reader_quiet && render_writer_quiet && render_tokens==0));
 always @(posedge clk)begin
  if(!rst_n)begin
   stage<=WAIT_KEY;source_frozen<=0;source_bank<=0;target_bank<=1;
   source_frame_id<=0;source_epoch<=0;camera_sequence<=0;
   render_start<=0;render_bus_owner<=0;published_display_valid<=0;published_display_bank<=0;
   display_original<=0;ab_used<=0;bank_error<=0;publish_count<=0;
  end else begin
   render_start<=0;
   if(camera_frame_start&&!source_frozen)camera_sequence<=camera_sequence+1'b1;
   if(context_bad)bank_error<=1;
   if(stage<ERROR_HELD && (render_error || (stage>=LEASE&&(context_bad||lease_fault))))begin
    stage<=ERROR_HELD;published_display_valid<=0;source_frozen<=1;
   end else case(stage)
    WAIT_KEY,ARMED:begin
     if(key_pulse)stage<=ARMED;
     if((stage==ARMED||key_pulse)&&source_accept)begin
      source_frozen<=1;source_bank<=camera_writer_bank;
      source_frame_id<=camera_sequence;stage<=SEAL;
     end
    end
    SEAL:if(source_published&&camera_write_quiet&&(!ENABLE_PREVIEW||display_epoch_quiet))begin
     target_bank<=source_bank==0?2'd1:2'd0;
     render_bus_owner<=1;stage<=LEASE;
    end
    LEASE:if(source_lease_active&&lease_bank==source_bank)begin
     source_epoch<=lease_epoch;render_start<=1;stage<=RENDER;
    end
    RENDER:if(publish_ok)stage<=READY;
    READY:if(vblank&&display_epoch_quiet&&publish_ok)begin
     render_bus_owner<=0;published_display_bank<=target_bank;published_display_valid<=1;
     publish_count<=publish_count+1'b1;stage<=SHOW_CORRECTED;
    end
    SHOW_CORRECTED:if(key_pulse&&!ab_used&&(!USE_EXPORT_HOLD||!export_hold))stage<=SWITCH_PENDING;
    SWITCH_PENDING:if(vblank&&display_epoch_quiet&&(!USE_EXPORT_HOLD||!export_hold))begin
     published_display_bank<=source_bank;display_original<=1;ab_used<=1;stage<=SHOW_ORIGINAL;
    end
    SHOW_ORIGINAL:stage<=SHOW_ORIGINAL;
    ERROR_HELD:begin
     // No new Bank owner or display. Existing port ownership stays to drain.
     published_display_valid<=0;
    end
    default:begin bank_error<=1;published_display_valid<=0;stage<=ERROR_HELD;end
   endcase
  end
 end
endmodule
