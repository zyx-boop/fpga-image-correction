// Tap an unchanged, repeatedly-read frozen DDR display frame. Never backpressure
// the normal video reader. If the export FIFO fills, take the next desired word
// on the next repeat of the same frame. No additional DDR read port is required.
module key_frame_snapshot #(
    parameter integer WIDTH=1920, HEIGHT=1080, CLOCK_HZ=27000000,
    parameter integer FIFO_ADDR_BITS=8,
    parameter integer DEBOUNCE_CYCLES=CLOCK_HZ/50,
    parameter integer PREAMBLE_CYCLES=CLOCK_HZ/50,
    parameter integer ACK_TIMEOUT_CYCLES=CLOCK_HZ*10,
    // 180 seconds exceeds signed 32-bit cycles; use seconds plus subsecond ticks.
    parameter integer TRANSFER_TIMEOUT_CYCLES=0,
    parameter integer TRANSFER_TIMEOUT_SECONDS=180,
    parameter integer RECOVERY_CYCLES=CLOCK_HZ/2
) (
    input wire sys_clk,core_clk,rst_n,key_n,uart_rx,display_valid,core_vblank,
    input wire video_word_valid,
    input wire [255:0] video_word,
    output reg hold_display,
    output wire uart_owned,
    output wire uart_tx
);
    localparam integer FRAME_BYTES=WIDTH*HEIGHT*2, FRAME_WORDS=FRAME_BYTES/32;
    localparam [3:0] IDLE=0,PREAMBLE=1,ANNOUNCE=2,ANNOUNCE_DRAIN=3,
        WAIT_ACK=4,HEADER=5,GET_WORD=6,LOAD_WORD=7,PAYLOAD=8,FOOTER=9,
        FINAL_DRAIN=10,RECOVER=11;
    reg [3:0] state;
    reg [31:0] timer, transfer_ticks;
    reg [15:0] transfer_seconds;
    reg [4:0] byte_index;
    reg [31:0] payload_index,crc;
    reg [255:0] word_shift;
    reg request_toggle,release_toggle,ack_first;
    // Registered ownership and baud selection: no multi-bit decoder glitches.
    reg export_busy,fast_mode;
    (* ASYNC_REG="TRUE" *) reg [1:0] key_sync,display_sync;
    reg key_stable;
    reg [31:0] debounce_count;
    wire press_event=key_stable && !key_sync[1] && debounce_count==DEBOUNCE_CYCLES-1;
    assign uart_owned=export_busy;
    always @(posedge sys_clk or negedge rst_n) begin
        if(!rst_n) begin key_sync<=2'b11; display_sync<=0; key_stable<=1; debounce_count<=0; end
        else begin
            key_sync<={key_sync[0],key_n}; display_sync<={display_sync[0],display_valid};
            if(key_sync[1]==key_stable) debounce_count<=0;
            else if(debounce_count==DEBOUNCE_CYCLES-1) begin key_stable<=key_sync[1]; debounce_count<=0; end
            else debounce_count<=debounce_count+1'b1;
        end
    end

    (* ASYNC_REG="TRUE" *) reg [2:0] request_sync,release_sync;
    reg request_seen,release_seen,scanning;
    reg [31:0] read_index,wanted_word;
    wire fifo_full,fifo_empty,fifo_overflow,fifo_underflow;
    wire [255:0] fifo_data;
    wire [FIFO_ADDR_BITS:0] fifo_used;
    // Reset FIFO between captures, including aborted/partially filled captures.
    // Requests traverse 3 stages before capture, then await a full vblank scan.
    wire fifo_reset_n=rst_n && uart_owned;
    wire fifo_write=hold_display && scanning && video_word_valid &&
        read_index==wanted_word && wanted_word<FRAME_WORDS && !fifo_full && !core_vblank;
    wire fifo_read=(state==GET_WORD) && !fifo_empty;
    snapshot_word_fifo #(.ADDR_BITS(FIFO_ADDR_BITS)) u_snapshot_fifo (
        .wr_clk(core_clk),.rd_clk(sys_clk),.rst_n(fifo_reset_n),
        .wr_en(fifo_write),.wr_data(video_word),.wr_full(fifo_full),
        .wr_used(fifo_used),.wr_overflow(fifo_overflow),
        .rd_en(fifo_read),.rd_data(fifo_data),.rd_empty(fifo_empty),.rd_underflow(fifo_underflow));
    always @(posedge core_clk or negedge rst_n) begin
        if(!rst_n) begin
            request_sync<=0;release_sync<=0;request_seen<=0;release_seen<=0;
            hold_display<=0;scanning<=0;read_index<=0;wanted_word<=0;
        end else begin
            request_sync<={request_sync[1:0],request_toggle};
            release_sync<={release_sync[1:0],release_toggle};
            if(core_vblank) begin read_index<=0; if(hold_display) scanning<=1; end
            else if(video_word_valid) read_index<=read_index+1'b1;
            if(fifo_write) wanted_word<=wanted_word+1'b1;
            if(request_sync[2]!=request_seen) begin
                request_seen<=request_sync[2];hold_display<=1;scanning<=0;wanted_word<=0;
            end
            if(release_sync[2]!=release_seen) begin
                release_seen<=release_sync[2];hold_display<=0;scanning<=0;
            end
        end
    end

    function [7:0] announce_byte;
        input [4:0] n;
        reg [143:0] text_line;
        begin text_line={"SNAP_FAST=921600",8'h0d,8'h0a};announce_byte=text_line[(17-n)*8+:8]; end
    endfunction
    function [7:0] header_byte;
        input [4:0] n;
        reg [63:0] magic;
        begin
            magic="PDSIMG01";
            if(n<8) header_byte=magic[(7-n)*8+:8];
            else case(n)
                8:header_byte=WIDTH&255; 9:header_byte=(WIDTH>>8)&255;
                10:header_byte=HEIGHT&255; 11:header_byte=(HEIGHT>>8)&255;
                12:header_byte=1;13,14,15:header_byte=0;
                default:header_byte=(FRAME_BYTES>>((n-16)*8))&255;
            endcase
        end
    endfunction
    function [31:0] crc_byte;
        input [31:0] old_crc;
        input [7:0] b;
        reg [31:0] v;
        integer k;
        begin
            v=old_crc ^ {24'b0,b};
            for(k=0;k<8;k=k+1) v=v[0] ? (v>>1)^32'hedb88320 : (v>>1);
            crc_byte=v;
        end
    endfunction
    wire slow_busy,slow_tx,fast_busy,fast_tx,rx_valid;
    wire [7:0] rx_data;
    wire slow_start=state==ANNOUNCE && !slow_busy;
    wire fast_start=(state==HEADER || state==PAYLOAD || state==FOOTER) && !fast_busy;
    wire [7:0] fast_data=state==HEADER ? header_byte(byte_index) :
        state==FOOTER ? ((~crc)>>(byte_index*8)) : word_shift[7:0];
    uart_tx #(.CLKS_PER_BIT(234)) u_slow_tx(
        .clk(sys_clk),.rst_n(rst_n),.start(slow_start),.data(announce_byte(byte_index)),.tx(slow_tx),.busy(slow_busy));
    snapshot_uart_tx #(.CLOCK_HZ(CLOCK_HZ)) u_fast_tx(
        .clk(sys_clk),.rst_n(rst_n),.start(fast_start),.data(fast_data),.tx(fast_tx),.busy(fast_busy));
    snapshot_uart_rx #(.CLOCK_HZ(CLOCK_HZ)) u_fast_rx(
        .clk(sys_clk),.rst_n(rst_n),.rx(uart_rx),.data(rx_data),.valid(rx_valid));
    assign uart_tx=fast_mode ? fast_tx : slow_tx;
    wire transfer_expired=(TRANSFER_TIMEOUT_CYCLES!=0) ?
        (transfer_ticks>=TRANSFER_TIMEOUT_CYCLES) : (transfer_seconds>=TRANSFER_TIMEOUT_SECONDS);
    always @(posedge sys_clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;timer<=0;transfer_ticks<=0;transfer_seconds<=0;
            byte_index<=0;payload_index<=0;crc<=32'hffffffff;word_shift<=0;
            request_toggle<=0;release_toggle<=0;ack_first<=0;
            export_busy<=0;fast_mode<=0;
        end else begin
            if(state>=HEADER && state<=FINAL_DRAIN) begin
                if(TRANSFER_TIMEOUT_CYCLES!=0) transfer_ticks<=transfer_ticks+1'b1;
                else if(transfer_ticks==CLOCK_HZ-1) begin transfer_ticks<=0;transfer_seconds<=transfer_seconds+1'b1; end
                else transfer_ticks<=transfer_ticks+1'b1;
            end
            case(state)
                IDLE: if(press_event && display_sync[1]) begin
                    request_toggle<=~request_toggle;state<=PREAMBLE;timer<=0;
                    export_busy<=1;
                    byte_index<=0;payload_index<=0;crc<=32'hffffffff;ack_first<=0;
                    transfer_ticks<=0;transfer_seconds<=0;
                end
                PREAMBLE: if(timer==PREAMBLE_CYCLES-1) begin state<=ANNOUNCE;timer<=0;end else timer<=timer+1'b1;
                ANNOUNCE: if(slow_start) begin
                    if(byte_index==17) begin state<=ANNOUNCE_DRAIN;byte_index<=0;end
                    else byte_index<=byte_index+1'b1;
                end
                ANNOUNCE_DRAIN: if(!slow_busy) begin state<=WAIT_ACK;timer<=0;fast_mode<=1;end
                WAIT_ACK: begin
                    timer<=timer+1'b1;
                    if(timer==ACK_TIMEOUT_CYCLES-1) begin state<=RECOVER;timer<=0;release_toggle<=~release_toggle;end
                    else if(rx_valid) begin
                        if(ack_first && rx_data==8'h5a) begin state<=HEADER;byte_index<=0;end
                        ack_first<=rx_data==8'ha5;
                    end
                end
                HEADER: if(fast_start) begin
                    if(byte_index==19) begin state<=GET_WORD;byte_index<=0;end else byte_index<=byte_index+1'b1;
                end
                GET_WORD: if(fifo_read) state<=LOAD_WORD;
                LOAD_WORD: begin word_shift<=fifo_data;byte_index<=0;state<=PAYLOAD;end
                PAYLOAD: if(fast_start) begin
                    crc<=crc_byte(crc,word_shift[7:0]);word_shift<=word_shift>>8;
                    payload_index<=payload_index+1'b1;
                    if(payload_index==FRAME_BYTES-1) begin state<=FOOTER;byte_index<=0;end
                    else if(byte_index==31) begin state<=GET_WORD;byte_index<=0;end
                    else byte_index<=byte_index+1'b1;
                end
                FOOTER: if(fast_start) begin
                    if(byte_index==3) state<=FINAL_DRAIN;else byte_index<=byte_index+1'b1;
                end
                FINAL_DRAIN: if(!fast_busy) begin state<=RECOVER;timer<=0;release_toggle<=~release_toggle;end
                RECOVER: if(!fast_busy && !slow_busy) begin
                    if(timer==RECOVERY_CYCLES-1) begin state<=IDLE;timer<=0;export_busy<=0;fast_mode<=0;end else timer<=timer+1'b1;
                end
                default: state<=IDLE;
            endcase
            if(state>=HEADER && state<=FINAL_DRAIN && transfer_expired) begin
                state<=RECOVER;timer<=0;release_toggle<=~release_toggle;
            end
        end
    end
endmodule
