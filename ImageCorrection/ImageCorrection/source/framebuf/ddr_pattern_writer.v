// Writes one complete RGB565 diagnostic frame to DDR3 after calibration.
// DDR addresses count 32-bit units; a 256-bit beat advances the address by 8.
module ddr_pattern_writer #(
    parameter FRAME_WIDTH = 1920,
    parameter FRAME_HEIGHT = 1080
) (
    input wire clk,
    input wire rst_n,
    input wire init_done,
    output reg [27:0] awaddr,
    output wire [3:0] awlen,
    output wire awvalid,
    input wire awready,
    output wire [255:0] wdata,
    output wire [31:0] wstrb,
    input wire wready,
    input wire wlast,
    output reg frame_written
);
    localparam integer WORDS_PER_LINE = FRAME_WIDTH / 16;
    localparam integer BAR_WORDS = FRAME_WIDTH / 128;
    localparam integer FRAME_WORDS = WORDS_PER_LINE * FRAME_HEIGHT;
    localparam [2:0] WAIT_INIT=0, SEND_AW=1, SEND_DATA=2,
                     WAIT_ACK=3, COMPLETE=4;
    reg [2:0] state;
    reg [17:0] written_words;
    reg [4:0] burst_words;
    reg [4:0] burst_position;
    reg [15:0] line_word;
    reg [15:0] bar_word;
    reg [2:0] bar_index;
    reg [15:0] rgb565;

    assign awvalid = (state == SEND_AW);
    assign awlen = burst_words[3:0] - 4'd1;
    assign wstrb = 32'hFFFFFFFF;
    assign wdata = {16{rgb565}};

    always @* begin
        case (bar_index)
            3'd0: rgb565 = 16'hF800; // red
            3'd1: rgb565 = 16'h07E0; // green
            3'd2: rgb565 = 16'h001F; // blue
            3'd3: rgb565 = 16'hFFFF; // white
            3'd4: rgb565 = 16'h07FF; // cyan
            3'd5: rgb565 = 16'hF81F; // magenta
            3'd6: rgb565 = 16'hFFE0; // yellow
            default: rgb565 = 16'h0000;
        endcase
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            state <= WAIT_INIT;
            awaddr <= 28'd0;
            written_words <= 18'd0;
            burst_words <= (FRAME_WORDS >= 16) ? 5'd16 : FRAME_WORDS;
            burst_position <= 5'd0;
            line_word <= 16'd0;
            bar_word <= 16'd0;
            bar_index <= 3'd0;
            frame_written <= 1'b0;
        end else begin
            case (state)
                WAIT_INIT: if (init_done) state <= SEND_AW;
                SEND_AW: if (awready) begin
                    burst_position <= 5'd0;
                    state <= SEND_DATA;
                end
                SEND_DATA: if (wready) begin
                    written_words <= written_words + 18'd1;
                    burst_position <= burst_position + 5'd1;
                    if (line_word == WORDS_PER_LINE-1) begin
                        line_word <= 16'd0;
                        bar_word <= 16'd0;
                        bar_index <= 3'd0;
                    end else begin
                        line_word <= line_word + 16'd1;
                        if (bar_word == BAR_WORDS-1) begin
                            bar_word <= 16'd0;
                            bar_index <= bar_index + 3'd1;
                        end else
                            bar_word <= bar_word + 16'd1;
                    end
                    if (burst_position == burst_words-1) begin
                        if (wlast) begin
                            if (written_words == FRAME_WORDS-1) begin
                                frame_written <= 1'b1;
                                state <= COMPLETE;
                            end else begin
                                awaddr <= awaddr + {burst_words, 3'b000};
                                burst_words <= (FRAME_WORDS-written_words-1 >= 16) ?
                                               5'd16 : FRAME_WORDS-written_words-1;
                                state <= SEND_AW;
                            end
                        end else
                            state <= WAIT_ACK;
                    end
                end
                WAIT_ACK: if (wlast) begin
                    if (written_words == FRAME_WORDS) begin
                        frame_written <= 1'b1;
                        state <= COMPLETE;
                    end else begin
                        awaddr <= awaddr + {burst_words, 3'b000};
                        burst_words <= (FRAME_WORDS-written_words >= 16) ?
                                       5'd16 : FRAME_WORDS-written_words;
                        state <= SEND_AW;
                    end
                end
                COMPLETE: state <= COMPLETE;
                default: state <= WAIT_INIT;
            endcase
        end
    end
endmodule
