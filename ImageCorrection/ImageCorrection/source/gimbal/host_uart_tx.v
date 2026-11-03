`timescale 1ns/1ps
module r17_host_uart_tx #(
    parameter integer CLOCK_HZ = 27000000,
    parameter integer BAUD = 115200
)(
    input wire clk, input wire rst_n, input wire start, input wire [7:0] data,
    output wire tx, output reg busy, output reg done
);
    localparam integer TICKS = CLOCK_HZ / BAUD;
    reg [9:0] shift;
    reg [15:0] count;
    reg [3:0] bit_index;
    assign tx = busy ? shift[0] : 1'b1;
    always @(posedge clk) begin
        if (!rst_n) begin
            busy <= 0; done <= 0; shift <= 10'h3ff;
            count <= 0; bit_index <= 0;
        end else begin
            done <= 0;
            if (!busy) begin
                if (start) begin
                    shift <= {1'b1, data, 1'b0}; busy <= 1;
                    count <= 0; bit_index <= 0;
                end
            end else if (count == TICKS-1) begin
                count <= 0;
                shift <= {1'b1, shift[9:1]};
                if (bit_index == 9) begin busy <= 0; done <= 1; end
                else bit_index <= bit_index + 1'b1;
            end else count <= count + 1'b1;
        end
    end
endmodule
