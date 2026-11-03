// Atomic bundled-data mailbox. Source holds the payload until acknowledgement.
// Only request/acknowledge toggles pass through synchronizer chains.
module pose_mailbox #(parameter WIDTH=81)(
 input wire source_clk,dest_clk,reset_n,
 input wire [WIDTH-1:0] source_data,
 output reg [WIDTH-1:0] dest_data
);
 reg [WIDTH-1:0] held;
 reg req,ack;
 (* ASYNC_REG="TRUE" *) reg [1:0] ack_sync,req_sync;
 (* ASYNC_REG="TRUE" *) reg [1:0] source_reset,dest_reset;
 always @(posedge source_clk or negedge reset_n)
  if(!reset_n)source_reset<=0;else source_reset<={source_reset[0],1'b1};
 always @(posedge dest_clk or negedge reset_n)
  if(!reset_n)dest_reset<=0;else dest_reset<={dest_reset[0],1'b1};
 always @(posedge source_clk or negedge reset_n)begin
  if(!reset_n)begin held<=0;req<=0;ack_sync<=0;end
  else begin
   ack_sync<={ack_sync[0],ack};
   if(source_reset[1]&&ack_sync[1]==req&&source_data!=held)begin held<=source_data;req<=~req;end
  end
 end
 always @(posedge dest_clk or negedge reset_n)begin
  if(!reset_n)begin dest_data<=0;ack<=0;req_sync<=0;end
  else begin
   req_sync<={req_sync[0],req};
   if(dest_reset[1]&&req_sync[1]!=ack)begin dest_data<=held;ack<=req_sync[1];end
  end
 end
endmodule
