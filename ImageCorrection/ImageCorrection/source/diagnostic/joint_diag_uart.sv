// Frozen-word consumer. Serial bytes, tags, and CRC all use the same latched
// word. The provider is requested once per word and may take arbitrary time.
module joint_diag_uart #(parameter BAUD_DIV=234)(
 input wire clk,rst_n,start,output reg busy,done,output wire tx,
 output reg word_req,output reg[9:0]word_index,
 input wire word_ready,input wire[31:0]word_data
);
 localparam IDLE=0,SEND=1,WAIT_BUSY=2,WAIT_IDLE=3,GET_WORD=4,CRC=5;
 reg[2:0]state;reg[1:0]phase;reg[4:0]pos;reg[1:0]crc_pos;
 reg[31:0]held_word,crc_word;reg[15:0]crc;
 reg tx_start;reg[7:0]tx_data,character;wire tx_busy;
 wire[9:0]memory_index=word_index-10'd64;
 function automatic[7:0]hex(input[3:0]n);begin hex=n<10?8'h30+n:8'h41+n-10;end endfunction
 function automatic[15:0]crc_byte(input[15:0]old,input[7:0]data);
 reg[15:0]c;integer i;begin c=old^{data,8'd0};for(i=0;i<8;i=i+1)c=c[15]?(c<<1)^16'h1021:c<<1;crc_byte=c;end endfunction
 always @*begin
 character=8'h3f;
 if(pos<5)case(pos)0:character="J";1:character="1";2:character="6";3:character="0";default:character=" ";endcase
 else if(phase==0)case(pos)5:character="B";6:character="E";7:character="G";8:character="I";9:character="N";10:character=13;11:character=10;default:character=63;endcase
 else if(phase==2)case(pos)5:character="C";6:character="R";7:character="C";8:character="=";9:character=hex(crc[15:12]);10:character=hex(crc[11:8]);11:character=hex(crc[7:4]);12:character=hex(crc[3:0]);13:character=13;14:character=10;default:character=63;endcase
 else if(word_index<64)begin
 case(pos)5:character="H";6:character=hex({2'd0,word_index[5:4]});7:character=hex(word_index[3:0]);8:character="=";17:character=13;18:character=10;
 default:if(pos>=9&&pos<=16)character=hex(held_word>>((16-pos)*4));endcase
 end else begin
 case(pos)5:character="M";6:character=hex({2'd0,memory_index[9:8]});7:character=hex(memory_index[7:4]);8:character=hex(memory_index[3:0]);9:character="=";18:character=13;19:character=10;
 default:if(pos>=10&&pos<=17)character=hex(held_word>>((17-pos)*4));endcase
 end
 end
 uart_tx #(.CLKS_PER_BIT(BAUD_DIV))u_tx(.clk(clk),.rst_n(rst_n),.start(tx_start),.data(tx_data),.tx(tx),.busy(tx_busy));
 always @(posedge clk)begin
 if(!rst_n)begin
 state<=IDLE;phase<=0;pos<=0;crc_pos<=0;held_word<=0;crc_word<=0;crc<=16'hffff;
 tx_start<=0;tx_data<=0;busy<=0;done<=0;word_req<=0;word_index<=0;
 end else begin
 tx_start<=0;done<=0;word_req<=0;
 case(state)
 IDLE:if(start)begin busy<=1;phase<=0;pos<=0;crc<=16'hffff;word_index<=0;state<=SEND;end
 SEND:if(!tx_busy)begin tx_data<=character;tx_start<=1;state<=WAIT_BUSY;end
 WAIT_BUSY:if(tx_busy)state<=WAIT_IDLE;
 WAIT_IDLE:if(!tx_busy)begin
 if((phase==0&&pos==11)||(phase==1&&word_index<64&&pos==18)||(phase==1&&word_index>=64&&pos==19)||(phase==2&&pos==14))begin
 pos<=0;
 if(phase==0)begin word_index<=0;word_req<=1;state<=GET_WORD;end
 else if(phase==2)begin busy<=0;done<=1;state<=IDLE;end
 else if(word_index==575)begin phase<=2;state<=SEND;end
 else begin word_index<=word_index+1'b1;word_req<=1;state<=GET_WORD;end
 end else begin pos<=pos+1'b1;state<=SEND;end
 end
 GET_WORD:if(word_ready)begin held_word<=word_data;crc_word<=word_data;crc_pos<=0;state<=CRC;end
 CRC:begin
 crc<=crc_byte(crc,crc_word[31:24]);crc_word<=crc_word<<8;
 if(crc_pos==3)begin phase<=1;pos<=0;state<=SEND;end else crc_pos<=crc_pos+1'b1;
 end
 default:state<=IDLE;
 endcase
 end
 end
endmodule
