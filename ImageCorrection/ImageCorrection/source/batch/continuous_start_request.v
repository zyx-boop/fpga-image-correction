// One debounced Key1 request survives the DDR initialization reset interval.
// System reset only clears it. Manager treats this level as sticky start, not
// as a job restart/mode key; its existing error-held behavior is unchanged.
module continuous_start_request(input wire clk,system_reset_n,key_pulse,output reg start_request);
 always @(posedge clk)begin
  if(!system_reset_n)start_request<=0;
  else if(key_pulse)start_request<=1;
 end
endmodule
