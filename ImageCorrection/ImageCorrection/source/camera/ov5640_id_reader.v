// Read chip ID once after power_ready. Press KEY0 to retry.
module ov5640_id_reader #(
    parameter integer CLK_HZ = 27000000
) (
    input wire clk,
    input wire rst_n,
    input wire power_ready,
    output reg start,
    output reg [15:0] reg_addr,
    input wire done,
    input wire bus_error,
    input wire [3:0] bus_error_code,
    input wire [7:0] rd_data,
    output reg id_ok,
    output reg id_error,
    output reg [15:0] chip_id,
    output reg [3:0] diagnostic_code,
    output reg failed_low_register
);
    localparam [2:0] WAIT_POWER=0, START_HIGH=1, WAIT_HIGH=2,
        START_LOW=3, WAIT_LOW=4, CHECK_ID=5, FINISHED=6;
    localparam integer TIMEOUT_CYCLES = CLK_HZ / 10; // 100 ms per register
    reg [2:0] state;
    reg [31:0] timeout_count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= WAIT_POWER;
            start <= 0;
            reg_addr <= 16'h300a;
            id_ok <= 0;
            id_error <= 0;
            chip_id <= 0;
            diagnostic_code <= 0;
            failed_low_register <= 0;
            timeout_count <= 0;
        end else begin
            start <= 0;
            case (state)
                WAIT_POWER: if (power_ready) state <= START_HIGH;
                START_HIGH: begin
                    reg_addr <= 16'h300a;
                    start <= 1;
                    timeout_count <= 0;
                    state <= WAIT_HIGH;
                end
                WAIT_HIGH: begin
                    if (done) begin
                        chip_id[15:8] <= rd_data;
                        if (bus_error) begin
                            id_error <= 1;
                            diagnostic_code <= bus_error_code;
                            state <= FINISHED;
                        end
                        else state <= START_LOW;
                    end else if (timeout_count == TIMEOUT_CYCLES - 1) begin
                        id_error <= 1;
                        diagnostic_code <= 4'd7;
                        state <= FINISHED;
                    end else timeout_count <= timeout_count + 1'b1;
                end
                START_LOW: begin
                    failed_low_register <= 1;
                    reg_addr <= 16'h300b;
                    start <= 1;
                    timeout_count <= 0;
                    state <= WAIT_LOW;
                end
                WAIT_LOW: begin
                    if (done) begin
                        chip_id[7:0] <= rd_data;
                        if (bus_error) begin
                            id_error <= 1;
                            diagnostic_code <= bus_error_code;
                            state <= FINISHED;
                        end
                        else state <= CHECK_ID;
                    end else if (timeout_count == TIMEOUT_CYCLES - 1) begin
                        id_error <= 1;
                        diagnostic_code <= 4'd7;
                        state <= FINISHED;
                    end else timeout_count <= timeout_count + 1'b1;
                end
                CHECK_ID: begin
                    id_ok <= (chip_id == 16'h5640);
                    id_error <= (chip_id != 16'h5640);
                    diagnostic_code <= (chip_id == 16'h5640) ? 4'd0 : 4'd8;
                    failed_low_register <= 0;
                    state <= FINISHED;
                end
                FINISHED: ;
                default: begin
                    id_error <= 1;
                    diagnostic_code <= 4'd9;
                    state <= FINISHED;
                end
            endcase
        end
    end
endmodule
