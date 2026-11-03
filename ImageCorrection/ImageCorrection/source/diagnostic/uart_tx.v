module uart_tx #(parameter integer CLKS_PER_BIT=234) (
    input wire clk,rst_n,start,
    input wire [7:0] data,
    output wire tx,
    output reg busy
);
    reg [9:0] shifter;
    reg [15:0] clocks;
    reg [3:0] remaining;
    assign tx=busy ? shifter[0] : 1'b1;
    always @(posedge clk) begin
        if(!rst_n) begin
            shifter<=10'h3ff;clocks<=0;remaining<=0;busy<=0;
        end else if(!busy) begin
            if(start) begin
                shifter<={1'b1,data,1'b0};
                clocks<=0;remaining<=10;busy<=1;
            end
        end else if(clocks==CLKS_PER_BIT-1) begin
            clocks<=0;shifter<={1'b1,shifter[9:1]};
            if(remaining==1) busy<=0;
            else remaining<=remaining-4'd1;
        end else clocks<=clocks+16'd1;
    end
endmodule
