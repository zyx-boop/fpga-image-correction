// Sequential DDR3 frame reader. Reserves room for a whole unbackpressured
// 16-word read burst before presenting AR to the DDR controller.
module ddr_pattern_reader #(
    parameter FRAME_WIDTH = 1920,
    parameter FRAME_HEIGHT = 1080,
    parameter FIFO_ADDR_BITS = 9
) (
    input wire clk,
    input wire rst_n,
    input wire frame_written,
    input wire [FIFO_ADDR_BITS:0] fifo_used,
    input wire fifo_full,
    output reg [27:0] araddr,
    output wire [3:0] arlen,
    output wire arvalid,
    input wire arready,
    input wire [255:0] rdata,
    input wire rvalid,
    input wire rlast,
    output wire fifo_wr_en,
    output wire [255:0] fifo_wr_data,
    output reg read_error
);
    localparam integer FRAME_WORDS = (FRAME_WIDTH / 16) * FRAME_HEIGHT;
    localparam integer FRAME_ADDR_UNITS = FRAME_WORDS * 8;
    localparam integer FIFO_WORDS = (1 << FIFO_ADDR_BITS);
    localparam [1:0] IDLE=0, REQUEST=1, RECEIVE=2, FAILED=3;
    reg [1:0] state;
    reg [4:0] beat_index;

    assign arlen = 4'd15;
    assign arvalid = (state == REQUEST);
    assign fifo_wr_en = (state == RECEIVE) && rvalid && !fifo_full;
    assign fifo_wr_data = rdata;

    always @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
            araddr <= 28'd0;
            beat_index <= 5'd0;
            read_error <= 1'b0;
        end else begin
            case (state)
                IDLE: if (frame_written && fifo_used <= FIFO_WORDS-16)
                    state <= REQUEST;
                REQUEST: if (arready) begin
                    beat_index <= 5'd0;
                    state <= RECEIVE;
                end
                RECEIVE: if (rvalid) begin
                    if (fifo_full || (rlast != (beat_index == 5'd15))) begin
                        read_error <= 1'b1;
                        state <= FAILED;
                    end else if (rlast) begin
                        if (araddr == FRAME_ADDR_UNITS-128)
                            araddr <= 28'd0;
                        else
                            araddr <= araddr + 28'd128;
                        state <= IDLE;
                    end else
                        beat_index <= beat_index + 5'd1;
                end
                FAILED: state <= FAILED;
                default: state <= IDLE;
            endcase
        end
    end
endmodule
