// Passive write monitor. Faults never cancel an offered AW or an accepted Word.
module camera_write_guard #(parameter integer WATCHDOG_CYCLES=125000000, BURST_MODE=0)(
 input wire clk,rst_n,render_owner,writer_active,awvalid,awready,wready,write_ack,
 input wire [3:0] awlen,
 output reg pending,error,output wire quiet,output reg [7:0] error_code
);
 reg [4:0] remaining;reg[31:0] wait_cycles;
 wire aw_take=awvalid&&awready&&!render_owner;
 wire [4:0] available=aw_take?(BURST_MODE?{1'b0,awlen}+5'd1:5'd1):remaining;
 wire data_take=wready&&!render_owner&&(pending||aw_take)&&available!=0;
 wire final_data=data_take&&available==1;
 wire ack_take=write_ack&&!render_owner;
 assign quiet=!pending&&!awvalid;
 task fault(input[7:0] code);begin error<=1;if(!error)error_code<=code;end endtask
 always@(posedge clk)begin
  if(!rst_n)begin pending<=0;remaining<=0;wait_cycles<=0;error<=0;error_code<=0;end
  else begin
   if(render_owner)begin
    wait_cycles<=0;
    if(pending||awvalid||writer_active)fault(8'h51);
   end else begin
    if(aw_take)begin if(pending||(BURST_MODE&&awlen>3))fault(8'h51);pending<=1;remaining<=available;end
    if(data_take)remaining<=available-1'b1;
    if(wready&&pending&&remaining==0)fault(8'h51);
    if(ack_take)begin
     if(!(pending||aw_take)||!(final_data||(pending&&remaining==0)))fault(8'h51);
     else begin pending<=0;remaining<=0;end
    end
    if(error||(!pending&&!awvalid&&!writer_active)||aw_take||data_take||ack_take)wait_cycles<=0;
    else if(wait_cycles>=WATCHDOG_CYCLES-1)fault(8'h52);
    else wait_cycles<=wait_cycles+1'b1;
   end
  end
 end
endmodule
