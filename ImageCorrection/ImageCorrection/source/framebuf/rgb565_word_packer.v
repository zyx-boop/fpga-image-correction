// Camera-clock domain: pack sixteen RGB565 pixels into one DDR3 user word.
// FIFO token [258:257]: 00=data, 01=validated end, 10=discard end.
// Data token bit 256 marks the first word; pixel 0 occupies bits [15:0].
module rgb565_word_packer #(
    parameter integer WIDTH=1920,
    parameter integer HEIGHT=1080,
    parameter integer ROW_CHECK_ENABLE=0,
    parameter integer DEBUG_FORCE_COMMIT=0
) (
    input wire clk,rst_n,
    input wire [15:0] pixel,
    input wire valid,sof,eol,eof,
    input wire frame_done,frame_good,
    input wire fifo_full,
    output wire fifo_wr_en,
    output wire [258:0] fifo_wr_data,
    output reg overflow,
    output reg row_error,
    output reg fifo_full_stall
);
    localparam integer FRAME_PIXELS=WIDTH*HEIGHT;
    reg [255:0] shift;
    reg [21:0] pixel_count;
    reg saw_eof;
    reg [11:0] row_pixel_count;
    reg end_pending;
    reg end_pending_good;
    // A capture front-end may assert frame_done on the same camera clock as
    // the final valid pixel.  Keep the end event for the following cycle so
    // that the final 16-pixel data token is formed and accepted first.
    reg frame_done_pending;
    reg frame_good_pending;
    reg token_valid;
    reg [258:0] token_data;
    wire token_take=token_valid && !fifo_full;
    assign fifo_wr_en=token_take;
    assign fifo_wr_data=token_data;
    wire [21:0] pixel_index=sof ? 22'd0 : pixel_count;
    wire finish_event=frame_done || frame_done_pending;
    wire finish_good_event=frame_done_pending ? frame_good_pending : frame_good;
    always @(posedge clk) begin
        if (!rst_n) begin
            shift<=0; pixel_count<=0; saw_eof<=0; overflow<=0;
            row_pixel_count<=0; row_error<=0;
            fifo_full_stall<=0;
            end_pending<=0; end_pending_good<=0;
            frame_done_pending<=0; frame_good_pending<=0;
            token_valid<=0; token_data<=0;
        end else begin
            if (token_take) token_valid<=0;
            // Defer only the simultaneous case.  The normal frame_done path
            // remains one-cycle and is handled without changing its timing.
            if (frame_done && valid) begin
                frame_done_pending<=1'b1;
                frame_good_pending<=frame_good;
            end
            if (end_pending) begin
                if (!token_valid || token_take) begin
                    token_valid<=1;
                    token_data<={end_pending_good ? 2'b01 : 2'b10,1'b0,256'd0};
                    end_pending<=0;
                end
                if (valid) begin
                    overflow<=1;
                    if (fifo_full) fifo_full_stall<=1;
                end
            end else if (finish_event && !valid) begin
                if (!token_valid || token_take) begin
                    token_valid<=1;
                    token_data<={(DEBUG_FORCE_COMMIT || (finish_good_event && saw_eof &&
                        pixel_count==FRAME_PIXELS && !overflow &&
                        (!ROW_CHECK_ENABLE || !row_error) && !valid)) ?
                        2'b01 : 2'b10,1'b0,256'd0};
                end else begin
                    end_pending<=1;
                    if (fifo_full) fifo_full_stall<=1;
                    end_pending_good<=DEBUG_FORCE_COMMIT || (finish_good_event && saw_eof &&
                        pixel_count==FRAME_PIXELS && !overflow &&
                        (!ROW_CHECK_ENABLE || !row_error) && !valid);
                end
                frame_done_pending<=1'b0;
            end else if (valid) begin
                if (sof) begin
                    overflow<=0;
                    saw_eof<=0;
                    row_pixel_count<=0;
                    row_error<=0;
                end
                if (ROW_CHECK_ENABLE) begin
                    if (!sof && ((row_pixel_count == WIDTH-1) != eol))
                        row_error<=1;
                    if (eol) row_pixel_count<=0;
                    else if (row_pixel_count < WIDTH-1)
                        row_pixel_count<=row_pixel_count+12'd1;
                    else row_error<=1;
                end
                pixel_count<=pixel_index+22'd1;
                shift<={pixel,shift[255:16]};
                if (eof) saw_eof<=1;
                if (pixel_index[3:0]==4'd15) begin
                    if (!token_valid || token_take) begin
                        token_valid<=1;
                        token_data<={2'b00,(pixel_index==22'd15),
                            pixel,shift[255:16]};
                    end else begin
                        overflow<=1;
                        if (fifo_full) fifo_full_stall<=1;
                    end
                end
            end
        end
    end
endmodule
