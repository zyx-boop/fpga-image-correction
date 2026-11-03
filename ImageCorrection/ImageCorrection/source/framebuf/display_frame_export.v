// Tap an unchanged, repeatedly-read frozen DDR display frame. Never backpressure
// the normal video reader. If the export FIFO fills, take the next desired word
// on the next repeat of the same frame. No additional DDR read port is required.
module display_frame_export #(
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
    input wire sys_clk,core_clk,rst_n,uart_rx,display_valid,scan_start,
    input wire corrected,original,core_error,
    input wire [1:0] display_bank,reader_bank,
    input wire [15:0] frame_id,source_epoch,
    input wire video_word_valid,
    input wire [255:0] video_word,
    output reg hold_display,
    output wire block_switch,
    output wire uart_owned,
    output wire uart_tx
);
    localparam integer FRAME_BYTES=WIDTH*HEIGHT*2, FRAME_WORDS=FRAME_BYTES/32;
    localparam [3:0] IDLE=0,PREAMBLE=1,ANNOUNCE=2,ANNOUNCE_DRAIN=3,
        WAIT_ACK=4,HEADER=5,GET_WORD=6,LOAD_WORD=7,PAYLOAD=8,FOOTER=9,
        FINAL_DRAIN=10,RECOVER=11,LOCK_WAIT=12;
    reg [3:0] state;
    reg [31:0] timer, transfer_ticks;
    reg [15:0] transfer_seconds;
    reg [4:0] byte_index;
    reg [31:0] payload_index,crc;
    reg [255:0] word_shift;
    reg request_toggle,release_toggle,ack_first,abort_first;
    // Registered ownership and baud selection: no multi-bit decoder glitches.
    reg export_busy,fast_mode;
    wire cmd_valid; wire [7:0] cmd_data;
    reg cmd_prefix,requested_kind;
    reg [31:0] request_number;
    wire command_request=state==IDLE&&cmd_valid&&cmd_prefix&&(cmd_data==8'h43||cmd_data==8'h4f);
    snapshot_uart_rx #(.CLOCK_HZ(CLOCK_HZ),.BAUD(115200)) u_command_rx(
      .clk(sys_clk),.rst_n(rst_n),.rx(uart_rx),.data(cmd_data),.valid(cmd_valid));
    assign uart_owned=export_busy;
    always @(posedge sys_clk or negedge rst_n)
      if(!rst_n)cmd_prefix<=0;
      else if(state!=IDLE)cmd_prefix<=0;
      else if(cmd_valid)cmd_prefix<=cmd_data==8'h45;

    (* ASYNC_REG="TRUE" *) reg [2:0] request_sync,release_sync;
    reg request_seen,release_seen,scanning;
    reg lock_accepted,tap_fault;
    reg [15:0] held_fid,held_epoch,packet_fid,packet_epoch;
    reg [1:0] held_bank,packet_bank;
    reg held_kind,packet_kind;
    (* ASYNC_REG="TRUE" *) reg [2:0] lock_ack_sync;
    (* ASYNC_REG="TRUE" *) reg [2:0] release_ack_sync;
    (* ASYNC_REG="TRUE" *) reg [1:0] tap_fault_sync;
    wire request_edge=request_sync[2]!=request_seen;
    assign block_switch=hold_display||request_edge;
    reg [31:0] read_index,wanted_word;
    wire fifo_full,fifo_empty,fifo_overflow,fifo_underflow;
    wire [255:0] fifo_data;
    wire [FIFO_ADDR_BITS:0] fifo_used;
    // Reset FIFO between captures, including aborted/partially filled captures.
    // Requests traverse 3 stages before capture, then await a full vblank scan.
    wire fifo_reset_n=rst_n && uart_owned;
    wire fifo_write=hold_display&&!tap_fault&&(scanning||scan_start)&&video_word_valid&&
        (scan_start?32'd0:read_index)==wanted_word&&wanted_word<FRAME_WORDS&&!fifo_full;
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
            lock_accepted<=0;tap_fault<=0;held_fid<=0;held_epoch<=0;held_bank<=0;held_kind<=0;
        end else begin
            request_sync<={request_sync[1:0],request_toggle};
            release_sync<={release_sync[1:0],release_toggle};
            if(scan_start) begin read_index<=video_word_valid?1:0; if(hold_display) scanning<=1;
              if(hold_display&&scanning&&read_index!=FRAME_WORDS)tap_fault<=1;
            end
            else if(video_word_valid) read_index<=read_index+1'b1;
            if(fifo_write) wanted_word<=wanted_word+1'b1;
            if(request_edge) begin
                request_seen<=request_sync[2];scanning<=0;wanted_word<=0;tap_fault<=0;
                lock_accepted<=display_valid&&!core_error&&display_bank<3&&
                  (requested_kind?corrected:original);
                hold_display<=display_valid&&!core_error&&display_bank<3&&
                  (requested_kind?corrected:original);
                held_fid<=frame_id;held_epoch<=source_epoch;held_bank<=display_bank;held_kind<=requested_kind;
            end
            if(hold_display&&(core_error||!display_valid||frame_id!=held_fid||source_epoch!=held_epoch||
               display_bank!=held_bank||(held_kind?!corrected:!original)||
               (video_word_valid&&scanning&&reader_bank!=held_bank)))tap_fault<=1;
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
            magic="PDSIMG02";
            if(n<8) header_byte=magic[(7-n)*8+:8];
            else case(n)
                8:header_byte=WIDTH&255; 9:header_byte=(WIDTH>>8)&255;
                10:header_byte=HEIGHT&255; 11:header_byte=(HEIGHT>>8)&255;
                12:header_byte=1;13:header_byte={7'd0,packet_kind};
                14:header_byte=packet_fid[7:0];15:header_byte=packet_fid[15:8];
                16,17,18,19:header_byte=(FRAME_BYTES>>((n-16)*8))&255;
                20:header_byte=packet_epoch[7:0];21:header_byte=packet_epoch[15:8];
                22:header_byte={6'd0,packet_bank};23:header_byte=0;
                default:header_byte=(request_number>>((n-24)*8))&255;
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
    snapshot_uart_tx #(.CLOCK_HZ(CLOCK_HZ),.BAUD(115200)) u_slow_tx(
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
            abort_first<=0;
            export_busy<=0;fast_mode<=0;
            lock_ack_sync<=0;release_ack_sync<=0;tap_fault_sync<=0;requested_kind<=0;request_number<=0;
            packet_fid<=0;packet_epoch<=0;packet_bank<=0;packet_kind<=0;
        end else begin
            lock_ack_sync<={lock_ack_sync[1:0],request_seen};
            release_ack_sync<={release_ack_sync[1:0],release_seen};
            tap_fault_sync<={tap_fault_sync[0],tap_fault|fifo_overflow};
            if(state<WAIT_ACK||state>FINAL_DRAIN)abort_first<=0;
            else if(rx_valid)abort_first<=rx_data==8'hde;
            if(state>=HEADER && state<=FINAL_DRAIN) begin
                if(TRANSFER_TIMEOUT_CYCLES!=0) transfer_ticks<=transfer_ticks+1'b1;
                else if(transfer_ticks==CLOCK_HZ-1) begin transfer_ticks<=0;transfer_seconds<=transfer_seconds+1'b1; end
                else transfer_ticks<=transfer_ticks+1'b1;
            end
            case(state)
                IDLE: if(command_request) begin
                    requested_kind<=cmd_data==8'h43;request_number<=request_number+1'b1;
                    request_toggle<=~request_toggle;state<=LOCK_WAIT;timer<=0;
                    export_busy<=1;
                    byte_index<=0;payload_index<=0;crc<=32'hffffffff;ack_first<=0;
                    transfer_ticks<=0;transfer_seconds<=0;
                end
                LOCK_WAIT: begin
                    timer<=timer+1'b1;
                    if(lock_ack_sync[2]==request_toggle)begin
                        timer<=0;
                        if(lock_accepted)begin
                            packet_fid<=held_fid;packet_epoch<=held_epoch;packet_bank<=held_bank;packet_kind<=held_kind;
                            state<=PREAMBLE;
                        end else begin state<=RECOVER;release_toggle<=~release_toggle;end
                    end else if(timer==ACK_TIMEOUT_CYCLES-1)begin
                        state<=RECOVER;timer<=0;release_toggle<=~release_toggle;
                    end
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
                    crc<=crc_byte(crc,header_byte(byte_index));
                    if(byte_index==27) begin state<=GET_WORD;byte_index<=0;end else byte_index<=byte_index+1'b1;
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
                // A timeout requests release; it does not fabricate core ACK.
                // Both mailbox operations must complete before a new toggle.
                RECOVER: if(!fast_busy && !slow_busy&&
                    release_ack_sync[2]==release_toggle&&lock_ack_sync[2]==request_toggle) begin
                    if(timer==RECOVERY_CYCLES-1) begin state<=IDLE;timer<=0;export_busy<=0;fast_mode<=0;end else timer<=timer+1'b1;
                end
                default: state<=IDLE;
            endcase
            if(state>=HEADER && state<=FINAL_DRAIN && transfer_expired) begin
                state<=RECOVER;timer<=0;release_toggle<=~release_toggle;
            end
            if(export_busy&&state!=IDLE&&state!=RECOVER&&state!=LOCK_WAIT&&(tap_fault_sync[1]||fifo_underflow))begin
                state<=RECOVER;timer<=0;release_toggle<=~release_toggle;
            end
            if(state>=WAIT_ACK&&state<=FINAL_DRAIN&&rx_valid&&abort_first&&rx_data==8'had)begin
                state<=RECOVER;timer<=0;release_toggle<=~release_toggle;
            end
        end
    end
endmodule
