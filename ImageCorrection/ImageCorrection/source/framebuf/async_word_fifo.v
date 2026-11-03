// Dual-clock, one-clock-read-latency FIFO for complete DDR user words.
// Depth must be a power of two and ADDR_BITS >= 2.
module async_word_fifo #(
    parameter WIDTH = 256,
    parameter ADDR_BITS = 9,
    parameter SPLIT_RESET = 0
) (
    input wire wr_clk,
    input wire rd_clk,
    input wire rst_n,
    input wire wr_en,
    input wire [WIDTH-1:0] wr_data,
    output reg wr_full,
    output wire [ADDR_BITS:0] wr_used,
    // Sticky diagnostics in each local clock domain.  These record an
    // attempted write while full or read while empty; they do not alter the
    // FIFO handshake or data path.
    output reg wr_overflow,
    input wire rd_en,
    output reg [WIDTH-1:0] rd_data,
    output reg rd_empty,
    output reg rd_underflow,
    input wire wr_rst_n,
    input wire rd_rst_n,
    output wire [ADDR_BITS:0] rd_used
);
    localparam DEPTH = (1 << ADDR_BITS);
    // Optional local, synchronously released resets supplied by one shared
    // coordinator. Default preserves legacy named-port instances unchanged.
    wire write_reset_n = SPLIT_RESET ? wr_rst_n : rst_n;
    wire read_reset_n = SPLIT_RESET ? rd_rst_n : rst_n;
    reg [WIDTH-1:0] memory [0:DEPTH-1];
    reg [ADDR_BITS:0] wr_bin, wr_gray;
    reg [ADDR_BITS:0] rd_bin, rd_gray;
    (* ASYNC_REG = "TRUE" *) reg [ADDR_BITS:0] rd_gray_w1, rd_gray_w2;
    (* ASYNC_REG = "TRUE" *) reg [ADDR_BITS:0] wr_gray_r1, wr_gray_r2;

    wire wr_take = wr_en && !wr_full;
    wire rd_take = rd_en && !rd_empty;
    wire [ADDR_BITS:0] wr_bin_next = wr_bin + wr_take;
    wire [ADDR_BITS:0] rd_bin_next = rd_bin + rd_take;
    wire [ADDR_BITS:0] wr_gray_next = (wr_bin_next >> 1) ^ wr_bin_next;
    wire [ADDR_BITS:0] rd_gray_next = (rd_bin_next >> 1) ^ rd_bin_next;
    wire [ADDR_BITS:0] wr_full_cmp = {
        ~rd_gray_w2[ADDR_BITS:ADDR_BITS-1], rd_gray_w2[ADDR_BITS-2:0]
    };

    function [ADDR_BITS:0] gray_to_bin;
        input [ADDR_BITS:0] gray;
        integer bit_idx;
        begin
            gray_to_bin[ADDR_BITS] = gray[ADDR_BITS];
            for (bit_idx = ADDR_BITS-1; bit_idx >= 0; bit_idx = bit_idx-1)
                gray_to_bin[bit_idx] = gray_to_bin[bit_idx+1] ^ gray[bit_idx];
        end
    endfunction
    assign wr_used = wr_bin - gray_to_bin(rd_gray_w2);
    // Read-domain observation of already synchronized Gray write position.
    // No FIFO handshake, storage or response timing depends on this output.
    assign rd_used = gray_to_bin(wr_gray_r2) - rd_bin;

    always @(posedge wr_clk or negedge write_reset_n) begin
        if (!write_reset_n) begin
            wr_bin <= 0;
            wr_gray <= 0;
            rd_gray_w1 <= 0;
            rd_gray_w2 <= 0;
            wr_full <= 1'b0;
            wr_overflow <= 1'b0;
        end else begin
            rd_gray_w1 <= rd_gray;
            rd_gray_w2 <= rd_gray_w1;
            if (wr_take) begin
                memory[wr_bin[ADDR_BITS-1:0]] <= wr_data;
                wr_bin <= wr_bin_next;
                wr_gray <= wr_gray_next;
            end
            if (wr_en && wr_full) wr_overflow <= 1'b1;
            wr_full <= (wr_gray_next == wr_full_cmp);
        end
    end

    always @(posedge rd_clk or negedge read_reset_n) begin
        if (!read_reset_n) begin
            rd_bin <= 0;
            rd_gray <= 0;
            wr_gray_r1 <= 0;
            wr_gray_r2 <= 0;
            rd_empty <= 1'b1;
            rd_data <= 0;
            rd_underflow <= 1'b0;
        end else begin
            wr_gray_r1 <= wr_gray;
            wr_gray_r2 <= wr_gray_r1;
            if (rd_take) begin
                rd_data <= memory[rd_bin[ADDR_BITS-1:0]];
                rd_bin <= rd_bin_next;
                rd_gray <= rd_gray_next;
            end
            if (rd_en && rd_empty) rd_underflow <= 1'b1;
            rd_empty <= (rd_gray_next == wr_gray_r2);
        end
    end
endmodule
