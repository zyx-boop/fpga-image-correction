// Post-initialization read requests only. There is deliberately no write-data
// or write-enable port. D160: 52 read-only registers per scan; 2 scans. Values are observed bytes, not a sensor atomic latch.
module joint_exposure_read #(parameter TIMEOUT_TICKS=5400000)(
 input wire clk,rst_n,enabled,start,input wire[31:0]frame_id,
 input wire bus_busy,bus_done,bus_error,input wire[3:0]bus_code,input wire[7:0]bus_data,
 output reg bus_start,output wire[15:0]bus_addr,
 output reg busy,done,timed_out,output reg[111:0]values,output reg[103:0]valid_mask,
 input wire rd_start,input wire[6:0]rd_index,output reg rd_done,output reg[7:0]rd_data,output reg scans_equal,
 output reg[31:0]first_error,scan0_start,scan0_end,scan1_start,scan1_end
);
 reg[1:0]state;reg pass;reg[5:0]item;reg[31:0]timer;
 function automatic[15:0]address(input[5:0]i);
 begin case(i)
 6'd0:address=16'h3500;
 6'd1:address=16'h3501;
 6'd2:address=16'h3502;
 6'd3:address=16'h3503;
 6'd4:address=16'h350a;
 6'd5:address=16'h350b;
 6'd6:address=16'h56a1;
 6'd7:address=16'h5691;
 6'd8:address=16'h5692;
 6'd9:address=16'h5693;
 6'd10:address=16'h5694;
 6'd11:address=16'h5695;
 6'd12:address=16'h5696;
 6'd13:address=16'h5697;
 6'd14:address=16'h5698;
 6'd15:address=16'h5699;
 6'd16:address=16'h569a;
 6'd17:address=16'h569b;
 6'd18:address=16'h569c;
 6'd19:address=16'h569d;
 6'd20:address=16'h569e;
 6'd21:address=16'h569f;
 6'd22:address=16'h56a0;
 6'd23:address=16'h5688;
 6'd24:address=16'h5689;
 6'd25:address=16'h568a;
 6'd26:address=16'h568b;
 6'd27:address=16'h568c;
 6'd28:address=16'h568d;
 6'd29:address=16'h568e;
 6'd30:address=16'h568f;
 6'd31:address=16'h3a0f;
 6'd32:address=16'h3a10;
 6'd33:address=16'h3a1b;
 6'd34:address=16'h3a1e;
 6'd35:address=16'h3a11;
 6'd36:address=16'h3a1f;
 6'd37:address=16'h3a00;
 6'd38:address=16'h3a01;
 6'd39:address=16'h3a05;
 6'd40:address=16'h3a18;
 6'd41:address=16'h3a19;
 6'd42:address=16'h501d;
 6'd43:address=16'h5680;
 6'd44:address=16'h5681;
 6'd45:address=16'h5682;
 6'd46:address=16'h5683;
 6'd47:address=16'h5684;
 6'd48:address=16'h5685;
 6'd49:address=16'h5686;
 6'd50:address=16'h5687;
 6'd51:address=16'h5690;
 default:address=16'h3500;endcase end endfunction
 wire[6:0]index=(pass?7'd52:7'd0)+{1'b0,item};
 assign bus_addr=address(item);
 // Bulk observation storage is one synchronous read/write RAM. No reset on
 // its array or output; valid_mask is the authority for initialized bytes.
 (* syn_ramstyle="block_ram" *) reg[7:0]observations[0:127];
 wire ram_write=rst_n&&state==2&&bus_done&&!bus_error;
 wire ram_read=rst_n&&((busy&&state==1)||(!busy&&rd_start));
 wire[6:0]ram_address=busy?{1'b0,item}:rd_index;
 reg[7:0]ram_q;reg rd_pending;reg[6:0]rd_held;
 always @(posedge clk)if(ram_write)observations[index]<=bus_data;
 always @(posedge clk)if(ram_read)ram_q<=observations[ram_address];
 always @(posedge clk)begin
 if(!rst_n)begin rd_pending<=0;rd_done<=0;rd_data<=0;rd_held<=0;end
 else begin
 rd_pending<=!busy&&rd_start;rd_done<=rd_pending;
 if(!busy&&rd_start)rd_held<=rd_index;
 if(rd_pending)rd_data<=(rd_held<104&&valid_mask[rd_held])?ram_q:8'd0;
 end
 end
 always @(posedge clk)begin
 if(!rst_n)begin
 state<=0;pass<=0;item<=0;timer<=0;bus_start<=0;busy<=0;done<=0;
 values<=0;valid_mask<=0;first_error<=0;timed_out<=0;scans_equal<=1;scan0_start<=0;scan0_end<=0;scan1_start<=0;scan1_end<=0;
 end else begin
 bus_start<=0;done<=0;
 case(state)
 0:if(start&&enabled&&!bus_busy)begin
 busy<=1;values<=0;valid_mask<=0;first_error<=0;timed_out<=0;scans_equal<=1;pass<=0;item<=0;timer<=0;
 scan0_start<=0;scan0_end<=0;scan1_start<=0;scan1_end<=0;state<=1;
 end
 1:begin
 if(!bus_busy)begin
 bus_start<=1;timer<=0;state<=2;
 if(item==0)begin if(pass)scan1_start<=frame_id;else scan0_start<=frame_id;end
 end else if(timer==TIMEOUT_TICKS-1)begin
 if(first_error==0)first_error<={bus_addr,{1'b0,index},4'hf,4'd0};
 if(pass)scan1_end<=frame_id;else scan0_end<=frame_id;
 timed_out<=1;busy<=0;done<=1;state<=0;
 end else timer<=timer+1'b1;
 end
 2:begin
 if(bus_done)begin
 if(!bus_error)begin
 if(item<7)begin if(pass)values[(item+7)*8+:8]<=bus_data;else values[item*8+:8]<=bus_data;end
 valid_mask[index]<=1;
 if(pass)scans_equal<=scans_equal&(ram_q==bus_data);
 end
 else if(first_error==0)first_error<={bus_addr,{1'b0,index},bus_code,4'd0};
 timer<=0;
 if(item==51)begin
 if(pass)begin scan1_end<=frame_id;busy<=0;done<=1;state<=0;end
 else begin scan0_end<=frame_id;pass<=1;item<=0;state<=1;end
 end else begin item<=item+1'b1;state<=1;end
 end else if(timer==TIMEOUT_TICKS-1)begin
 if(first_error==0)first_error<={bus_addr,{1'b0,index},4'hf,4'd0};
 if(pass)scan1_end<=frame_id;else scan0_end<=frame_id;
 timed_out<=1;busy<=0;done<=1;state<=0;
 end else timer<=timer+1'b1;
 end
 default:state<=0;
 endcase
 end
 end
endmodule
