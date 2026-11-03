// One display frame per vertical blank. Stop at the exact frame boundary,
// leaving no old-bank words in the output FIFO before the next bank switch.
module ddr_live_reader #(
    parameter integer FRAME_WIDTH=1920,
    parameter integer FRAME_HEIGHT=1080,
    parameter integer FIFO_ADDR_BITS=9,
    parameter integer BANK_ADDR_UNITS=FRAME_WIDTH*FRAME_HEIGHT/2,
    parameter integer STOP_ON_DISABLE=0
) (
    input wire clk,rst_n,vblank,display_valid,
    input wire [1:0] display_bank,
    input wire [FIFO_ADDR_BITS:0] fifo_used,
    input wire fifo_full,
    output reg [27:0] araddr,
    output wire [3:0] arlen,
    output wire arvalid,
    input wire arready,
    input wire [255:0] rdata,
    input wire rvalid,rlast,
    output wire fifo_wr_en,
    output wire [255:0] fifo_wr_data,
    output reg read_error,
    output reg addr_error,
    output reg [15:0] addr_error_count,
    output reg [27:0] addr_error_actual,
    output reg [27:0] addr_error_expected,
    output reg [3:0] addr_error_code,
    output wire reader_quiet
);
    localparam integer RGB565_BYTES_PER_PIXEL=2;
    localparam integer DDR_BEAT_BYTES=32;
    localparam integer DDR_ADDR_UNIT_BYTES=4;
    localparam integer FRAME_BYTES=FRAME_WIDTH*FRAME_HEIGHT*RGB565_BYTES_PER_PIXEL;
    localparam integer FRAME_WORDS=FRAME_BYTES/DDR_BEAT_BYTES;
    localparam integer FRAME_ADDR_UNITS=FRAME_BYTES/DDR_ADDR_UNIT_BYTES;
    localparam integer FIFO_WORDS=(1<<FIFO_ADDR_BITS);
    localparam [2:0] IDLE=0,WAIT_OWNER=1,WAIT_ROOM=2,
                     REQUEST=3,RECEIVE=4,FAILED=5;
    reg [2:0] state;
    reg [17:0] words_issued;
    reg [4:0] burst_words,beat_index;
    function [27:0] bank_base;
        input [1:0] bank;
        begin
            case (bank)
                2'd0: bank_base=28'd0;
                2'd1: bank_base=BANK_ADDR_UNITS;
                2'd2: bank_base=2*BANK_ADDR_UNITS;
                default: bank_base=28'd0;
            endcase
        end
    endfunction
    wire [27:0] expected_read_addr = bank_base(display_bank) +
                                     {words_issued,3'b000};
    wire [27:0] expected_read_end = bank_base(display_bank) + FRAME_ADDR_UNITS;

    // Codes: 8=read burst address mismatch, 9=read burst exceeds frame,
    // A=read frame did not issue exactly FRAME_WORDS.  Writer diagnostics use
    // codes 2..4, so the source is visible in the code without another UART
    // field.
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
    assign arvalid=(state==REQUEST);
    assign reader_quiet=(state==IDLE) && !rvalid;
    assign arlen=burst_words[3:0]-4'd1;
    assign fifo_wr_en=((state==RECEIVE)||(state==REQUEST&&arready)) && rvalid && !fifo_full;
    assign fifo_wr_data=rdata;
    always @(posedge clk) begin
        if (!rst_n) begin
            state<=IDLE; araddr<=0; words_issued<=0;
            burst_words<=0; beat_index<=0; read_error<=0;
            addr_error<=0; addr_error_count<=16'd0;
            addr_error_actual<=28'd0; addr_error_expected<=28'd0;
            addr_error_code<=4'd0;
        end else if (vblank && state!=IDLE) begin
            read_error<=1;
            state<=FAILED;
        end else begin
            case (state)
                IDLE: if (vblank) state<=WAIT_OWNER;
                WAIT_OWNER: begin
                    if (display_valid && fifo_used==0) begin
                        araddr<=bank_base(display_bank);
                        words_issued<=0;
                        burst_words<=(FRAME_WORDS>=16) ? 5'd16 : FRAME_WORDS;
                        state<=WAIT_ROOM;
                    end else if (display_valid) begin
                        read_error<=1;
                        state<=FAILED;
                    end else state<=IDLE;
                end
                WAIT_ROOM: begin
                    // Cancel only an unoffered next burst. REQUEST/RECEIVE
                    // must still preserve and drain the existing transaction.
                    if(STOP_ON_DISABLE && !display_valid)state<=IDLE;
                    else if (fifo_used<=FIFO_WORDS-burst_words && !fifo_full)state<=REQUEST;
                end
                REQUEST: if (arready) begin
                    if (araddr != expected_read_addr)
                        record_addr_error(4'h8,araddr,expected_read_addr);
                    if (burst_words==0 || words_issued+burst_words>FRAME_WORDS)
                        record_addr_error(4'h9,araddr,expected_read_end);
                    beat_index<=rvalid?5'd1:5'd0;
                    state<=RECEIVE;
                    if(rvalid && (fifo_full || rlast!=(burst_words==1)))begin
                        read_error<=1;state<=FAILED;
                    end else if(rvalid && rlast)begin
                        words_issued<=words_issued+burst_words;
                        if(words_issued+burst_words==FRAME_WORDS)state<=IDLE;
                        else begin
                            araddr<=araddr+{burst_words,3'b000};
                            burst_words<=((FRAME_WORDS-words_issued-burst_words)>=16)?5'd16:
                                FRAME_WORDS-words_issued-burst_words;
                            state<=WAIT_ROOM;
                        end
                    end
                end
                RECEIVE: if (rvalid) begin
                    if (fifo_full || (rlast != (beat_index==burst_words-1))) begin
                        read_error<=1;
                        state<=FAILED;
                    end else if (rlast) begin
                        words_issued<=words_issued+burst_words;
                        if (words_issued+burst_words==FRAME_WORDS) begin
                            state<=IDLE;
                        end else begin
                            araddr<=araddr+{burst_words,3'b000};
                            burst_words<=((FRAME_WORDS-words_issued-burst_words)>=16) ?
                                5'd16 : FRAME_WORDS-words_issued-burst_words;
                            state<=WAIT_ROOM;
                        end
                    end else beat_index<=beat_index+5'd1;
                end
                FAILED: state<=FAILED;
                default: begin read_error<=1; state<=FAILED; end
            endcase
        end
    end
endmodule
