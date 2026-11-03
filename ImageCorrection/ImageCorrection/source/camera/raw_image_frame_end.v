// Camera clock only: join capture verdict and processed-pixel completion.
// Pixel contents never determine validity. Bad frames are discarded without
// waiting for an EOF that might not exist; good frames wait for BOTH events.
module raw_image_frame_end(
    input wire clk,rst_n,in_sof,capture_done,capture_good,pixel_eof,
    output reg out_done,out_good
);
    reg verdict_pending,eof_seen;
    always @(posedge clk) begin
        if(!rst_n) begin
            verdict_pending<=0;eof_seen<=0;out_done<=0;out_good<=0;
        end else begin
            out_done<=0;out_good<=0;
            if(in_sof) begin verdict_pending<=0;eof_seen<=0;end
            if(pixel_eof) eof_seen<=1;
            if(capture_done && !capture_good) begin
                out_done<=1;out_good<=0;verdict_pending<=0;eof_seen<=0;
            end else if((capture_done && capture_good && (eof_seen || pixel_eof)) ||
                        (verdict_pending && pixel_eof)) begin
                out_done<=1;out_good<=1;verdict_pending<=0;eof_seen<=0;
            end else if(capture_done && capture_good) verdict_pending<=1;
        end
    end
endmodule
