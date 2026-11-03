// DDR core-clock domain. Each 256-bit word is one acknowledged DDR write.
// A frame is offered to frame_owner only after its end token is validated
// and every write response (wlast) has arrived. Address units are 32 bits.
// RGB565 is two bytes/pixel; one 256-bit beat therefore contains 16 pixels
// and advances the generated DDR address by eight 32-bit address units.
module ddr_live_writer #(
    parameter integer FRAME_WIDTH=1920,
    parameter integer FRAME_HEIGHT=1080,
    parameter integer DEBUG_FORCE_COMMIT=0,
    parameter integer DUAL_SOURCE=0
) (
    input wire clk,rst_n,init_done,
    input wire fifo_empty,
    input wire [258:0] fifo_rd_data,
    output wire fifo_rd_en,
    input wire writer_active,
    input wire [1:0] writer_bank,
    output reg frame_start,frame_finish,frame_good,
    output reg [27:0] awaddr,
    output wire [3:0] awlen,
    output wire awvalid,
    input wire awready,
    output wire [255:0] wdata,
    output wire [31:0] wstrb,
    input wire wready,wlast,
    output reg error,
    output reg addr_error,
    output reg [15:0] addr_error_count,
    output reg [27:0] addr_error_actual,
    output reg [27:0] addr_error_expected,
    output reg [3:0] addr_error_code,
    // Snapshot only at the real end-token. These fields are diagnostic
    // evidence for frame-end validation, not live address monitors.
    output reg frame_end_pulse,
    output reg [17:0] frame_end_word_count,
    output reg [27:0] frame_end_addr_actual,
    output reg [27:0] frame_end_addr_expected,
    output reg frame_end_token_good
);
    localparam integer RGB565_BYTES_PER_PIXEL=2;
    localparam integer DDR_BEAT_BYTES=32;
    localparam integer DDR_ADDR_UNIT_BYTES=4;
    localparam integer FRAME_BYTES=FRAME_WIDTH*FRAME_HEIGHT*RGB565_BYTES_PER_PIXEL;
    localparam integer FRAME_WORDS=FRAME_BYTES/DDR_BEAT_BYTES;
    localparam integer FRAME_ADDR_UNITS=FRAME_BYTES/DDR_ADDR_UNIT_BYTES;
    localparam integer BEAT_ADDR_UNITS=DDR_BEAT_BYTES/DDR_ADDR_UNIT_BYTES;
    localparam [2:0] WAIT_INIT=0,GET_TOKEN=1,DECODE=2,
                     WAIT_BANK=3,SEND_AW=4,SEND_W=5,
                     WAIT_ACK=6,END_DELAY=7;
    reg [2:0] state;
    reg [1:0] bank_wait;
    reg [17:0] word_count;
    reg [255:0] held_word;
    reg dropping;
    function [27:0] bank_base;
        input [1:0] bank;
        begin
            case (bank)
                2'd0: bank_base=28'd0;
                2'd1: bank_base=FRAME_ADDR_UNITS;
                2'd2: bank_base=2*FRAME_ADDR_UNITS;
                2'd3: bank_base=DUAL_SOURCE?3*FRAME_ADDR_UNITS:28'd0;
                default: bank_base=28'd0;
            endcase
        end
    endfunction
    wire [27:0] expected_word_addr = bank_base(writer_bank) +
                                     {word_count,3'b000};
    wire [27:0] expected_end_addr = bank_base(writer_bank) + FRAME_ADDR_UNITS;
    reg source_bank_ok;
    always_comb begin
        source_bank_ok=0;
        case(writer_bank) 0,3:source_bank_ok=1;default:source_bank_ok=0;endcase
    end

    // Codes: 1=SOF while writer busy (frame ownership, not an address error),
    // 2=data beyond FRAME_WORDS, 4=end count/address mismatch.  AW requests
    // are intentionally not classified as frame-end errors.
    // Code 1 is deliberately not reported through addr_error; it belongs to
    // frame_owner/drop diagnostics and was previously a false DDR_ADDR_ERR.
    task automatic record_addr_error;
        input [3:0] code;
        input [27:0] actual_addr;
        input [27:0] expected_addr;
        begin
            addr_error<=1'b1;
            if (addr_error_count != 16'hffff)
                addr_error_count<=addr_error_count+16'd1;
            addr_error_actual<=actual_addr;
            addr_error_expected<=expected_addr;
            addr_error_code<=code;
        end
    endtask
    assign fifo_rd_en=(state==GET_TOKEN) && init_done && !fifo_empty;
    assign awvalid=(state==SEND_AW);
    assign awlen=4'd0;
    assign wdata=held_word;
    assign wstrb=32'hffffffff;
    always @(posedge clk) begin
        if (!rst_n) begin
            state<=WAIT_INIT; bank_wait<=0; word_count<=0;
            held_word<=0; dropping<=0; awaddr<=0; addr_error<=1'b0;
            addr_error_count<=16'd0; addr_error_actual<=28'd0;
            addr_error_expected<=28'd0; addr_error_code<=4'd0;
            frame_start<=0;frame_finish<=0;frame_good<=0;error<=0;
            frame_end_pulse<=0; frame_end_word_count<=0;
            frame_end_addr_actual<=0; frame_end_addr_expected<=0;
            frame_end_token_good<=0;
        end else begin
            frame_start<=0; frame_finish<=0; frame_good<=0;
            frame_end_pulse<=0;
            case (state)
                WAIT_INIT: if (init_done) state<=GET_TOKEN;
                GET_TOKEN: if (fifo_rd_en) state<=DECODE;
                DECODE: begin
                    case (fifo_rd_data[258:257])
                        2'b00: begin
                            held_word<=fifo_rd_data[255:0];
                            if (fifo_rd_data[256]) begin
                                word_count<=0;
                                dropping<=0;
                                if (writer_active) begin
                                    // This is a frame-owner collision.  Do not
                                    // mislabel it as a DDR address violation.
                                    error<=1; dropping<=1; state<=GET_TOKEN;
                                end else begin
                                    frame_start<=1;
                                    bank_wait<=0;
                                    state<=WAIT_BANK;
                                end
                            end else if (writer_active && !dropping &&
                                         word_count<FRAME_WORDS) begin
                                awaddr<=expected_word_addr;
                                state<=SEND_AW;
                            end else begin
                                if (writer_active && !dropping)
                                    record_addr_error(4'd2,expected_word_addr,
                                                      expected_end_addr);
                                dropping<=1;
                                state<=GET_TOKEN;
                            end
                        end
                        2'b01,2'b10: begin
                            if (writer_active) begin
                                // This is the only point at which the
                                // generated DDR end address is meaningful:
                                // word_count is the number of acknowledged
                                // 256-bit writes in this frame.
                                frame_end_pulse<=1'b1;
                                frame_end_word_count<=word_count;
                                frame_end_addr_actual<=expected_word_addr;
                                frame_end_addr_expected<=expected_end_addr;
                                frame_end_token_good<=
                                    (fifo_rd_data[258:257]==2'b01) &&
                                    !dropping &&
                                    (DEBUG_FORCE_COMMIT || word_count==FRAME_WORDS);
                                frame_finish<=1;
                                frame_good<=(fifo_rd_data[258:257]==2'b01) &&
                                    !dropping && (DEBUG_FORCE_COMMIT || word_count==FRAME_WORDS);
                                if (!DEBUG_FORCE_COMMIT && !dropping && word_count!=FRAME_WORDS)
                                    record_addr_error(4'd4,expected_word_addr,
                                                      expected_end_addr);
                            end
                            word_count<=0;
                            dropping<=0;
                            state<=END_DELAY;
                        end
                        default: begin
                            error<=1;
                            dropping<=1;
                            state<=GET_TOKEN;
                        end
                    endcase
                end
                WAIT_BANK: begin
                    if (writer_active) begin
                        if(DUAL_SOURCE&&!source_bank_ok)begin
                            error<=1;dropping<=1;state<=GET_TOKEN;
                        end else begin awaddr<=bank_base(writer_bank);state<=SEND_AW;end
                    end else if (bank_wait==2'd2) begin
                        dropping<=1;
                        state<=GET_TOKEN;
                    end else bank_wait<=bank_wait+2'd1;
                end
                SEND_AW: if (awready) begin
                    // Do not classify a live AW address as a frame-end
                    // failure. The end-token snapshot above is the sole
                    // frame-size/address check; this avoids false DDR_ADDR
                    // errors while a frame is still being written.
                    state<=SEND_W;
                end
                SEND_W: if (wready) begin
                    if (wlast) begin
                        word_count<=word_count+18'd1;
                        state<=GET_TOKEN;
                    end else state<=WAIT_ACK;
                end
                WAIT_ACK: if (wlast) begin
                    word_count<=word_count+18'd1;
                    state<=GET_TOKEN;
                end
                END_DELAY: state<=GET_TOKEN;
                default: begin error<=1; state<=WAIT_INIT; end
            endcase
        end
    end
endmodule
