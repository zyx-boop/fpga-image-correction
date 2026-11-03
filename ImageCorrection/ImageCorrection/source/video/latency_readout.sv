// 125 MHz core observation of capture SOF to first corrected display pixel.
// Convert to nearest 0.1 ms without a combinational divide. Latest complete
// sample crosses as a held mailbox, then commits only at a display boundary.
module latency_readout(
 input wire core_clk,core_rst_n,pixel_clk,pixel_rst_n,frame_start,
 input wire [63:0] latency_cycles,input wire [31:0] displayed_count,
 input wire measurement_error,
 output reg [19:0] digits,output reg valid,overflow
);
 reg [31:0] seen,dividend;
 reg error_seen,request,ack;
 (* ASYNC_REG="TRUE" *) reg [1:0] ack_sync,req_sync;
 reg [2:0] state;
 reg [5:0] bits_left;
 reg [13:0] remainder;
 wire [14:0] trial={remainder,dividend[31]};
 wire subtract=trial>=15'd12500;
 reg [16:0] quotient;
 reg [19:0] bcd;
 reg [19:0] adjusted;
 reg [21:0] mailbox,pending;
 reg pending_valid,sample_valid;
 integer k;
 always @* begin
  adjusted=bcd;
  for(k=0;k<5;k=k+1)
   if(bcd[k*4+:4]>=5)adjusted[k*4+:4]=bcd[k*4+:4]+4'd3;
 end
 always @(posedge core_clk or negedge core_rst_n)begin
  if(!core_rst_n)begin seen<=0;error_seen<=0;request<=0;ack_sync<=0;state<=0;
   dividend<=0;bits_left<=0;remainder<=0;quotient<=0;bcd<=0;mailbox<=0;sample_valid<=0;end
  else begin
   ack_sync<={ack_sync[0],ack};
   case(state)
    0:if(ack_sync[1]==request&&(displayed_count!=seen||measurement_error!=error_seen))begin
     seen<=displayed_count;error_seen<=measurement_error;
     sample_valid<=!measurement_error&&displayed_count!=0;
     if(measurement_error||displayed_count==0)begin mailbox<=0;state<=3;end
     else if(|latency_cycles[63:31]||latency_cycles[30:0]>=31'd1249993750)begin
      mailbox<={2'b11,20'd0};state<=3;
     end else begin
      dividend<={1'b0,latency_cycles[30:0]}+32'd6250;
      remainder<=0;quotient<=0;bits_left<=32;state<=1;
     end
    end
    1:begin
     remainder<=subtract?trial-15'd12500:trial;
     dividend<={dividend[30:0],1'b0};quotient<={quotient[15:0],subtract};
     bits_left<=bits_left-1'b1;
     if(bits_left==1)begin state<=2;bcd<=0;bits_left<=17;end
    end
    2:begin
     bcd<={adjusted[18:0],quotient[16]};quotient<={quotient[15:0],1'b0};bits_left<=bits_left-1'b1;
     if(bits_left==1)begin mailbox<={sample_valid,1'b0,adjusted[18:0],quotient[16]};state<=3;end
    end
    3:begin request<=~request;state<=0;end
    default:state<=0;
   endcase
  end
 end
 always @(posedge pixel_clk or negedge pixel_rst_n)begin
  if(!pixel_rst_n)begin req_sync<=0;ack<=0;pending<=0;pending_valid<=0;digits<=0;valid<=0;overflow<=0;end
  else begin
   req_sync<={req_sync[0],request};
   if(frame_start&&pending_valid)begin {valid,overflow,digits}<=pending;pending_valid<=0;end
   if(req_sync[1]!=ack)begin pending<=mailbox;pending_valid<=1;ack<=req_sync[1];end
  end
 end
endmodule
