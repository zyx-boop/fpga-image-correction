// Three independent DDR frame regions. Only a complete, validated and fully
// acknowledged frame may become pending; the display switches only at vblank.
// All inputs are single-cycle pulses synchronous to ddr_clk. If pulses collide,
// finish has priority over vblank, and vblank over start; delaying a switch by
// one display frame is safe, while ever reading a writer-owned bank is not.
module frame_owner (
    input  wire       ddr_clk,
    input  wire       rst_n,
    input  wire       frame_start,
    input  wire       frame_finish,
    input  wire       frame_good,
    input  wire       vblank,
    output reg        writer_active,
    output reg [1:0]  writer_bank,
    output reg        display_valid,
    output reg [1:0]  display_bank,
    output reg        pending_valid,
    output reg [1:0]  pending_bank,
    output reg        dropped_frame
);
    wire bank0_busy = (display_valid && display_bank == 2'd0) ||
                      (pending_valid && pending_bank == 2'd0);
    wire bank1_busy = (display_valid && display_bank == 2'd1) ||
                      (pending_valid && pending_bank == 2'd1);
    wire bank2_busy = (display_valid && display_bank == 2'd2) ||
                      (pending_valid && pending_bank == 2'd2);

    always @(posedge ddr_clk) begin
        if (!rst_n) begin
            writer_active <= 1'b0;
            writer_bank <= 2'd0;
            display_valid <= 1'b0;
            display_bank <= 2'd0;
            pending_valid <= 1'b0;
            pending_bank <= 2'd0;
            dropped_frame <= 1'b0;
        end else begin
            dropped_frame <= 1'b0;
            if (frame_finish) begin
                if (writer_active && frame_good && !pending_valid) begin
                    pending_valid <= 1'b1;
                    pending_bank <= writer_bank;
                end else if (writer_active) begin
                    dropped_frame <= 1'b1;
                end
                writer_active <= 1'b0;
            end else begin
                if (vblank && pending_valid) begin
                    display_valid <= 1'b1;
                    display_bank <= pending_bank;
                    pending_valid <= 1'b0;
                end
                if (frame_start) begin
                    if (writer_active) begin
                        dropped_frame <= 1'b1;
                    end else if (!bank0_busy) begin
                        writer_active <= 1'b1;
                        writer_bank <= 2'd0;
                    end else if (!bank1_busy) begin
                        writer_active <= 1'b1;
                        writer_bank <= 2'd1;
                    end else if (!bank2_busy) begin
                        writer_active <= 1'b1;
                        writer_bank <= 2'd2;
                    end else begin
                        dropped_frame <= 1'b1;
                    end
                end
            end
        end
    end
endmodule
