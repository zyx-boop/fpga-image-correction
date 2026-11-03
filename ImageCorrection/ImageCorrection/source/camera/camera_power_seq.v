// Camera power sequence, all counters run from the 27 MHz board clock.
module camera_power_seq #(
    parameter integer CLK_HZ = 27000000
) (
    input wire clk,
    input wire rst_n,
    output reg cam_pwdn,
    output reg cam_rst_n,
    output reg ready,
    output wire [1:0] debug_state,
    output wire [31:0] debug_count
);
    localparam integer WAIT_POWER = CLK_HZ / 1000 * 100;
    localparam integer WAIT_PWDN  = CLK_HZ / 1000 * 10;
    localparam integer WAIT_RESET = CLK_HZ / 1000 * 100;
    reg [1:0] state;
    reg [31:0] count;
    assign debug_state = state;
    assign debug_count = count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= 0;
            count <= 0;
            cam_pwdn <= 1;
            cam_rst_n <= 0;
            ready <= 0;
        end else begin
            case (state)
                0: if (count == WAIT_POWER - 1) begin
                    count <= 0;
                    cam_pwdn <= 0;
                    state <= 1;
                end else count <= count + 1'b1;
                1: if (count == WAIT_PWDN - 1) begin
                    count <= 0;
                    cam_rst_n <= 1;
                    state <= 2;
                end else count <= count + 1'b1;
                2: if (count == WAIT_RESET - 1) begin
                    count <= 0;
                    ready <= 1;
                    state <= 3;
                end else count <= count + 1'b1;
                3: ;
                default: begin
                    state <= 0;
                    count <= 0;
                    cam_pwdn <= 1;
                    cam_rst_n <= 0;
                    ready <= 0;
                end
            endcase
        end
    end
endmodule
