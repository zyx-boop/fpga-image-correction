// Complete data is buffered before a native DDR request; EOF waits for real ACK.
module camera_burst_writer #(
 parameter FRAME_WIDTH=1920,FRAME_HEIGHT=1080,DUAL_SOURCE=1
)(
 input wire clk,rst_n,init_done,fifo_empty,input wire [258:0] fifo_rd_data,
 output wire fifo_rd_en,token_consume_en,output wire [258:0] token_consume_data,
 input wire writer_active,input wire [1:0] writer_bank,
 output reg frame_start,frame_finish,frame_good,
 output reg [27:0] awaddr,output wire [3:0] awlen,output wire awvalid,
 input wire awready,output wire [255:0] wdata,output wire [31:0] wstrb,
 input wire wready,wlast,output reg error,addr_error,
 output reg [15:0] addr_error_count,output reg [27:0] addr_error_actual,addr_error_expected,
 output reg [3:0] addr_error_code,output reg frame_end_pulse,
 output reg [17:0] frame_end_word_count,
 output reg [27:0] frame_end_addr_actual,frame_end_addr_expected,
 output reg frame_end_token_good
);
 localparam FRAME_WORDS=FRAME_WIDTH*FRAME_HEIGHT/16;
 localparam FRAME_ADDR_UNITS=FRAME_WORDS*8;
 localparam WAIT_INIT=0,GET_TOKEN=1,DECODE=2,WAIT_BANK=3,SEND_AW=4,SEND_W=5,WAIT_ACK=6,END_DELAY=7;
 reg [2:0] state,buffer_count;
 reg [1:0] bank_wait,send_index;
 reg [17:0] word_count;
 reg dropping;
 reg [255:0] buffer0,buffer1,buffer2,buffer3;
 reg [258:0] lookahead_token;
 reg lookahead_valid,fetch_pending;
 wire is_end=lookahead_token[258:257]==1||lookahead_token[258:257]==2;
 assign fifo_rd_en=init_done&&!fifo_empty&&!lookahead_valid&&!fetch_pending;
 // Preserve the original one-clock consume -> DECODE identity relationship.
 // An end token stays unconsumed while the partial buffered burst drains.
 assign token_consume_en=init_done&&lookahead_valid&&state==GET_TOKEN&&!(is_end&&buffer_count!=0);
 assign token_consume_data=lookahead_token;
 always @(posedge clk)begin
  if(!rst_n)begin lookahead_token<=0;lookahead_valid<=0;fetch_pending<=0;end
  else begin
   fetch_pending<=fifo_rd_en;
   if(fetch_pending)begin lookahead_token<=fifo_rd_data;lookahead_valid<=1;end
   if(token_consume_en)lookahead_valid<=0;
  end
 end
 function [27:0] bank_base(input [1:0] bank);
  case(bank)0:bank_base=0;1:bank_base=FRAME_ADDR_UNITS;2:bank_base=2*FRAME_ADDR_UNITS;
   3:bank_base=DUAL_SOURCE?3*FRAME_ADDR_UNITS:0;endcase
 endfunction
 wire [27:0] expected_word_addr=bank_base(writer_bank)+{word_count,3'b000};
 wire [27:0] expected_end_addr=bank_base(writer_bank)+FRAME_ADDR_UNITS;
 wire source_bank_ok=writer_bank==0||(DUAL_SOURCE&&writer_bank==3);
 assign awvalid=state==SEND_AW;
 assign awlen={1'b0,buffer_count}-4'd1;
 assign wstrb=32'hffffffff;
 assign wdata=send_index==0?buffer0:send_index==1?buffer1:send_index==2?buffer2:buffer3;
 wire aw_take=awvalid&&awready;
 wire data_take=(state==SEND_W||aw_take)&&wready;
 wire last_data=data_take&&({1'b0,send_index}+3'd1==buffer_count);
 wire ack_legal=(state==WAIT_ACK)||last_data;
 task record_addr_error(input [3:0] code);
  begin addr_error<=1;if(addr_error_count!=16'hffff)addr_error_count<=addr_error_count+1'b1;
   addr_error_actual<=expected_word_addr;addr_error_expected<=expected_end_addr;addr_error_code<=code;end
 endtask
 always @(posedge clk)begin
  if(!rst_n)begin
   state<=WAIT_INIT;buffer_count<=0;send_index<=0;bank_wait<=0;word_count<=0;dropping<=0;
   buffer0<=0;buffer1<=0;buffer2<=0;buffer3<=0;awaddr<=0;
   frame_start<=0;frame_finish<=0;frame_good<=0;error<=0;addr_error<=0;addr_error_count<=0;
   addr_error_actual<=0;addr_error_expected<=0;addr_error_code<=0;
   frame_end_pulse<=0;frame_end_word_count<=0;frame_end_addr_actual<=0;frame_end_addr_expected<=0;frame_end_token_good<=0;
  end else begin
   frame_start<=0;frame_finish<=0;frame_good<=0;frame_end_pulse<=0;
   if(wlast&&!ack_legal)error<=1;
   case(state)
    WAIT_INIT:if(init_done)state<=GET_TOKEN;
    GET_TOKEN:begin
     if(init_done&&lookahead_valid&&is_end&&buffer_count!=0)begin
      awaddr<=expected_word_addr;send_index<=0;state<=SEND_AW;
     end else if(token_consume_en)state<=DECODE;
    end
    DECODE:case(token_consume_data[258:257])
     2'b00:begin
      if(token_consume_data[256])begin
       word_count<=0;dropping<=0;
       if(writer_active||buffer_count!=0)begin error<=1;dropping<=1;state<=GET_TOKEN;end
       else begin
        buffer0<=token_consume_data[255:0];buffer_count<=1;frame_start<=1;bank_wait<=0;state<=WAIT_BANK;
       end
      end else if(writer_active&&!dropping&&word_count+buffer_count<FRAME_WORDS)begin
       case(buffer_count)
        0:buffer0<=token_consume_data[255:0];1:buffer1<=token_consume_data[255:0];
        2:buffer2<=token_consume_data[255:0];3:buffer3<=token_consume_data[255:0];
        default:error<=1;
       endcase
       buffer_count<=buffer_count+1'b1;
       if(buffer_count==3)begin awaddr<=expected_word_addr;send_index<=0;state<=SEND_AW;end
       else state<=GET_TOKEN;
      end else begin
       if(writer_active&&!dropping)record_addr_error(4'd2);
       dropping<=1;state<=GET_TOKEN;
      end
     end
     2'b01,2'b10:begin
      if(writer_active)begin
       frame_end_pulse<=1;frame_end_word_count<=word_count;
       frame_end_addr_actual<=expected_word_addr;frame_end_addr_expected<=expected_end_addr;
       frame_end_token_good<=token_consume_data[258:257]==1&&!dropping&&word_count==FRAME_WORDS;
       frame_finish<=1;frame_good<=token_consume_data[258:257]==1&&!dropping&&word_count==FRAME_WORDS;
       if(!dropping&&word_count!=FRAME_WORDS)record_addr_error(4'd4);
      end
      word_count<=0;dropping<=0;state<=END_DELAY;
     end
     default:begin error<=1;dropping<=1;state<=GET_TOKEN;end
    endcase
    WAIT_BANK:begin
     if(writer_active)begin
      if(!source_bank_ok)begin error<=1;dropping<=1;buffer_count<=0;end
      state<=GET_TOKEN;
     end else if(bank_wait==2)begin dropping<=1;buffer_count<=0;state<=GET_TOKEN;end
     else bank_wait<=bank_wait+1'b1;
    end
    SEND_AW:if(aw_take)state<=data_take?(last_data?WAIT_ACK:SEND_W):SEND_W;
    SEND_W:if(last_data)state<=WAIT_ACK;
    WAIT_ACK:begin end
    END_DELAY:state<=GET_TOKEN;
    default:begin error<=1;state<=WAIT_INIT;end
   endcase
   if(data_take&&!last_data)send_index<=send_index+1'b1;
   if(wlast&&ack_legal)begin
    word_count<=word_count+buffer_count;buffer_count<=0;send_index<=0;state<=GET_TOKEN;
   end
  end
 end
endmodule
