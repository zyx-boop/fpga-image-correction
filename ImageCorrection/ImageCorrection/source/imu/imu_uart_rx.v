`timescale 1ns / 1ps
// Project-authored UART, 8N1. Inputs are synchronized and stop bits checked.
module r15_imu_uart_rx #(
    parameter integer CLOCK_HZ = 27000000,
    parameter integer BAUD = 115200
)(
    input wire clk, input wire rst_n, input wire rx,
    output reg [7:0] data, output reg valid, output wire rx_level
);
    localparam integer TICKS = CLOCK_HZ / BAUD;
    (* ASYNC_REG = "TRUE" *) reg rx_meta, rx_sync;
    assign rx_level=rx_sync;
    reg [1:0] state;
    reg [15:0] count;
    reg [2:0] bit_index;
    reg [7:0] shift;
    always @(posedge clk) begin
        if (!rst_n) begin rx_meta <= 1; rx_sync <= 1; end
        else begin rx_meta <= rx; rx_sync <= rx_meta; end
    end
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= 0; count <= 0; bit_index <= 0;
            shift <= 0; data <= 0; valid <= 0;
        end else begin
            valid <= 0;
            case (state)
                0: if (!rx_sync) begin state <= 1; count <= 0; end
                1: if (count == (TICKS-1)/2) begin
                    count <= 0;
                    if (!rx_sync) begin state <= 2; bit_index <= 0; end
                    else state <= 0;
                end else count <= count + 1'b1;
                2: if (count == TICKS-1) begin
                    count <= 0; shift[bit_index] <= rx_sync;
                    if (bit_index == 7) state <= 3;
                    else bit_index <= bit_index + 1'b1;
                end else count <= count + 1'b1;
                3: if (count == TICKS-1) begin
                    state <= 0; count <= 0;
                    if (rx_sync) begin data <= shift; valid <= 1; end
                end else count <= count + 1'b1;
            endcase
        end
    end
endmodule

