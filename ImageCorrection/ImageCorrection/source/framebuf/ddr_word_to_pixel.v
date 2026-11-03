// Pixel-clock consumer of 256-bit DDR words. FIFO read data is registered
// one cycle after a read request; two local words hide that latency.
module ddr_word_to_pixel (
    input wire clk,
    input wire rst_n,
    input wire de,
    input wire frame_start,
    input wire fifo_empty,
    input wire [255:0] fifo_rd_data,
    output wire fifo_rd_en,
    output wire ready,
    output reg [15:0] rgb565,
    output reg pixel_valid,
    output reg underflow,
    output wire consumer_quiet,
    output reg [21:0] consumed_pixels,
    output reg [15:0] consumed_epoch
);
    reg [255:0] current_word, next_word;
    reg current_valid, next_valid;
    reg fetch_pending, fetch_to_next;
    reg [3:0] lane;
    reg active;

    assign fifo_rd_en = rst_n && !fetch_pending && !fifo_empty &&
                        (!current_valid || !next_valid);
    assign ready = current_valid && next_valid;
    assign consumer_quiet = !current_valid && !next_valid && !fetch_pending &&
                            fifo_empty && lane==0 && !pixel_valid;

    always @(posedge clk) begin
        if (!rst_n) begin
            current_word <= 256'd0;
            next_word <= 256'd0;
            current_valid <= 1'b0;
            next_valid <= 1'b0;
            fetch_pending <= 1'b0;
            fetch_to_next <= 1'b0;
            lane <= 4'd0;
            active <= 1'b0;
            rgb565 <= 16'd0;
            pixel_valid <= 1'b0;
            underflow <= 1'b0;
            consumed_pixels <= 0;
            consumed_epoch <= 0;
        end else begin
            pixel_valid <= 1'b0;
            if (frame_start && ready)begin
                active <= 1'b1;
                consumed_epoch <= consumed_epoch+1'b1;
            end

            if (fifo_rd_en) begin
                fetch_pending <= 1'b1;
                fetch_to_next <= current_valid;
            end else if (fetch_pending) begin
                fetch_pending <= 1'b0;
                if (fetch_to_next) begin
                    next_word <= fifo_rd_data;
                    next_valid <= 1'b1;
                end else begin
                    current_word <= fifo_rd_data;
                    current_valid <= 1'b1;
                end
            end

            if (de && (active || (frame_start && ready))) begin
                consumed_pixels <= (frame_start && ready) ? 22'd1 : consumed_pixels+1'b1;
                pixel_valid <= 1'b1;
                if (current_valid) begin
                    rgb565 <= current_word[lane*16 +: 16];
                    if (lane == 4'd15) begin
                        lane <= 4'd0;
                        current_word <= next_word;
                        current_valid <= next_valid;
                        next_valid <= 1'b0;
                    end else
                        lane <= lane + 4'd1;
                end else begin
                    rgb565 <= 16'hF81F;
                    underflow <= 1'b1;
                end
            end
        end
    end
endmodule
