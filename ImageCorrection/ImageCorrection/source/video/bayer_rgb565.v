// Optional causal RAW prefilter followed by the legacy Bayer core.
// BLACK levels default to zero until calibrated; do not infer offsets from
// frozen UART tiles. Gain-after-demosaic is opt-in to preserve old tests.
module bayer_rgb565 #(
    parameter integer WIDTH=1920, HEIGHT=1080, CFA_PHASE=0,
    parameter integer RAW_GAIN_SHIFT=0, RAW_GAIN_MULT=0,
    parameter integer LOWLIGHT_CHROMA_ENABLE=0,
    parameter integer LOWLIGHT_LUMA_THRESHOLD=96,
    parameter integer LOWLIGHT_CHROMA_THRESHOLD=20,
    parameter integer BLACK_R=0, BLACK_GR=0, BLACK_GB=0, BLACK_B=0,
    parameter integer RAW_DENOISE_ENABLE=0,
    parameter integer RAW_DENOISE_THRESHOLD=8,
    parameter integer RAW_SMOOTH_RANGE=8,
    parameter integer GAIN_AFTER_DEMOSAIC=0
)(
    input wire pclk,rst_n,
    input wire [7:0] raw,
    input wire raw_valid,raw_sof,raw_eol,raw_eof,
    input wire [11:0] raw_x,
    input wire [10:0] raw_y,
    output wire [15:0] rgb565,
    output wire valid,sof,eol,eof,
    output wire [11:0] x,
    output wire [10:0] y
);
    wire [7:0] clean_raw;
    wire clean_valid,clean_sof,clean_eol,clean_eof;
    wire [11:0] clean_x;
    wire [10:0] clean_y;
    generate if (RAW_DENOISE_ENABLE || BLACK_R || BLACK_GR || BLACK_GB || BLACK_B) begin : g_raw_filter
        raw_cfa_prefilter #(.WIDTH(WIDTH),.CFA_PHASE(CFA_PHASE),
            .BLACK_R(BLACK_R),.BLACK_GR(BLACK_GR),.BLACK_GB(BLACK_GB),.BLACK_B(BLACK_B),
            .DENOISE_ENABLE(RAW_DENOISE_ENABLE),.OUTLIER_THRESHOLD(RAW_DENOISE_THRESHOLD),
            .SMOOTH_RANGE(RAW_SMOOTH_RANGE)) u_filter (
            .clk(pclk),.rst_n(rst_n),.raw(raw),.in_valid(raw_valid),
            .in_sof(raw_sof),.in_eol(raw_eol),.in_eof(raw_eof),.in_x(raw_x),.in_y(raw_y),
            .clean(clean_raw),.out_valid(clean_valid),.out_sof(clean_sof),
            .out_eol(clean_eol),.out_eof(clean_eof),.out_x(clean_x),.out_y(clean_y));
    end else begin : g_raw_bypass
        assign clean_raw=raw;
        assign {clean_valid,clean_sof,clean_eol,clean_eof}={raw_valid,raw_sof,raw_eol,raw_eof};
        assign clean_x=raw_x;
        assign clean_y=raw_y;
    end endgenerate
    bayer_rgb565_core #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.CFA_PHASE(CFA_PHASE),
        .RAW_GAIN_SHIFT(RAW_GAIN_SHIFT),.RAW_GAIN_MULT(RAW_GAIN_MULT),
        .GAIN_AFTER_DEMOSAIC(GAIN_AFTER_DEMOSAIC),
        .LOWLIGHT_CHROMA_ENABLE(LOWLIGHT_CHROMA_ENABLE),
        .LOWLIGHT_LUMA_THRESHOLD(LOWLIGHT_LUMA_THRESHOLD),
        .LOWLIGHT_CHROMA_THRESHOLD(LOWLIGHT_CHROMA_THRESHOLD)) u_core (
        .pclk(pclk),.rst_n(rst_n),.raw(clean_raw),.raw_valid(clean_valid),
        .raw_sof(clean_sof),.raw_eol(clean_eol),.raw_eof(clean_eof),
        .raw_x(clean_x),.raw_y(clean_y),.rgb565(rgb565),.valid(valid),
        .sof(sof),.eol(eol),.eof(eof),.x(x),.y(y));
endmodule

// Same-colour causal neighbours: (x-2,y) and (x,y-2). Two parity row
// memories keep UNFILTERED corrected samples: no recursive smoothing.
// Read-before-write synchronous RAM: register the old row before replacing
// it. No RAM reset or asynchronous read. Border two rows/columns bypass.
module raw_cfa_prefilter #(
    parameter integer WIDTH=1920,CFA_PHASE=0,
    parameter integer BLACK_R=0,BLACK_GR=0,BLACK_GB=0,BLACK_B=0,
    parameter integer DENOISE_ENABLE=0,OUTLIER_THRESHOLD=8,SMOOTH_RANGE=8
)(
    input wire clk,rst_n,
    input wire [7:0] raw,
    input wire in_valid,in_sof,in_eol,in_eof,
    input wire [11:0] in_x,
    input wire [10:0] in_y,
    output reg [7:0] clean,
    output reg out_valid,out_sof,out_eol,out_eof,
    output reg [11:0] out_x,
    output reg [10:0] out_y
);
    (* syn_ramstyle = "block_ram" *) reg [7:0] even_row[0:WIDTH-1];
    (* syn_ramstyle = "block_ram" *) reg [7:0] odd_row[0:WIDTH-1];
    reg [7:0] horizontal1,horizontal2;
    reg [7:0] center_q,horizontal_q,vertical_q;
    reg v_q,s_q,l_q,e_q,interior_q;
    reg [11:0] x_q;
    reg [10:0] y_q;
    // Convert physical parity to canonical RGGB parity; green next to R
    // on the R row is Gr, green next to B on the B row is Gb.
    wire [1:0] phase={in_y[0],in_x[0]} ^ (CFA_PHASE ? 2'b11 : 2'b00);
    wire [7:0] black = phase==0 ? BLACK_R : phase==1 ? BLACK_GR :
                       phase==2 ? BLACK_GB : BLACK_B;
    wire [7:0] corrected = raw > black ? raw-black : 8'd0;
    // Isolate inferred RAM processes from reset logic.
    always @(posedge clk) begin
        if (rst_n && in_valid) begin
            if (!in_y[0]) begin
                vertical_q <= even_row[in_x];
                even_row[in_x] <= corrected;
            end else begin
                vertical_q <= odd_row[in_x];
                odd_row[in_x] <= corrected;
            end
        end
    end
    always @(posedge clk) begin
        if (!rst_n) begin
            horizontal1<=0;horizontal2<=0;center_q<=0;horizontal_q<=0;
            v_q<=0;s_q<=0;l_q<=0;e_q<=0;interior_q<=0;x_q<=0;y_q<=0;
        end else begin
            v_q<=in_valid;s_q<=in_valid && in_sof;
            l_q<=in_valid && in_eol;e_q<=in_valid && in_eof;
            if (in_valid) begin
                center_q<=corrected;horizontal_q<=horizontal2;
                interior_q<=(in_x>=2 && in_y>=2);
                x_q<=in_x;y_q<=in_y;
                horizontal1<=corrected;
                horizontal2<=(in_x==0) ? 8'd0 : horizontal1;
            end
        end
    end
    function [7:0] absdiff;
        input [7:0] a,b;
        begin absdiff=a>=b ? a-b : b-a;end
    endfunction
    wire [7:0] dh=absdiff(center_q,horizontal_q);
    wire [7:0] dv=absdiff(center_q,vertical_q);
    wire [7:0] dn=absdiff(horizontal_q,vertical_q);
    wire outlier=((center_q>horizontal_q && center_q>vertical_q) ||
                  (center_q<horizontal_q && center_q<vertical_q)) &&
                 dh>=OUTLIER_THRESHOLD && dv>=OUTLIER_THRESHOLD && dn<=SMOOTH_RANGE;
    wire flat=(dh<=SMOOTH_RANGE && dv<=SMOOTH_RANGE && dn<=SMOOTH_RANGE);
    wire [8:0] neighbour_sum={1'b0,horizontal_q}+{1'b0,vertical_q}+9'd1;
    wire [9:0] soft_sum=({2'b00,center_q}<<1)+{2'b00,horizontal_q}+
                       {2'b00,vertical_q}+10'd2;
    always @(posedge clk) begin
        if (!rst_n) begin
            clean<=0;out_valid<=0;out_sof<=0;out_eol<=0;out_eof<=0;out_x<=0;out_y<=0;
        end else begin
            out_valid<=v_q;out_sof<=s_q;out_eol<=l_q;out_eof<=e_q;
            if (v_q) begin
                out_x<=x_q;out_y<=y_q;
                if (DENOISE_ENABLE && interior_q && outlier)
                    clean<=neighbour_sum[8:1];
                else if (DENOISE_ENABLE && interior_q && flat)
                    clean<=soft_sum[9:2];
                else clean<=center_q;
            end
        end
    end
endmodule

// RAW8 3x3 bilinear demosaic. Output is delayed one row and one column.
// Three rotating line RAMs store raw rows; the last row is replayed during
// vertical blank with its lower neighbor replicated. CFA_PHASE=0 interprets
// the first 2x2 site as RGGB; CFA_PHASE=1 swaps R/B for BGGR.
module bayer_rgb565_core #(
    parameter integer WIDTH = 1920,
    parameter integer HEIGHT = 1080,
    parameter integer CFA_PHASE = 0,
    // RAW8 from this sensor profile is visibly compressed toward the dark
    // end. Keep the module neutral by default; the board top selects one
    // saturating gain after demosaic. RAW_GAIN_MULT=0 selects the
    // legacy power-of-two shift path; a constant multiplier such as 3 gives
    // a middle ground between the 2x and 4x board experiments.
    parameter integer RAW_GAIN_SHIFT = 0,
    parameter integer RAW_GAIN_MULT = 0,
    // Optional post-demosaic low-light chroma suppression.  This is not a
    // CFA-neighbour average: it only attenuates large RGB chroma excursions
    // when the estimated luminance is low, preserving bright colour texture.
    parameter integer LOWLIGHT_CHROMA_ENABLE = 0,
    parameter integer LOWLIGHT_LUMA_THRESHOLD = 96,
    parameter integer LOWLIGHT_CHROMA_THRESHOLD = 20,
    parameter integer GAIN_AFTER_DEMOSAIC = 0
) (
    input wire pclk, rst_n,
    input wire [7:0] raw,
    input wire raw_valid, raw_sof, raw_eol, raw_eof,
    input wire [11:0] raw_x,
    input wire [10:0] raw_y,
    output reg [15:0] rgb565,
    output reg valid, sof, eol, eof,
    output reg [11:0] x,
    output reg [10:0] y
);
    (* syn_ramstyle = "block_ram" *) reg [7:0] line0 [0:WIDTH-1];
    (* syn_ramstyle = "block_ram" *) reg [7:0] line1 [0:WIDTH-1];
    (* syn_ramstyle = "block_ram" *) reg [7:0] line2 [0:WIDTH-1];
    reg [1:0] write_bank, last_bank;
    wire [1:0] effective_bank = raw_sof ? 2'd0 : write_bank;
    reg [1:0] replay_delay;
    reg replay_active;
    reg [11:0] replay_x;
    wire [9:0] raw_gain_ext = RAW_GAIN_MULT != 0 ?
                               ({2'b00,raw} * RAW_GAIN_MULT) :
                               ({2'b00,raw} << RAW_GAIN_SHIFT);
    wire [7:0] raw_scaled = GAIN_AFTER_DEMOSAIC ? raw :
                            ((|raw_gain_ext[9:8]) ? 8'hff : raw_gain_ext[7:0]);

    // Stage 1: registered RAM reads and current camera byte. The two read
    // banks are disjoint from the write bank except at row-zero replication.
    reg [7:0] top_q, mid_q, bot_q;
    reg [11:0] x_q;
    reg [10:0] y_q;
    reg stage_valid, stage_eol;
    always @(posedge pclk) begin
        if (!rst_n) begin
            write_bank <= 2'd0; last_bank <= 2'd0;
            replay_delay <= 2'd0; replay_active <= 1'b0;
            replay_x <= 12'd0; stage_valid <= 1'b0;
            stage_eol <= 1'b0; x_q <= 12'd0; y_q <= 11'd0;
            top_q <= 8'd0; mid_q <= 8'd0; bot_q <= 8'd0;
        end else begin
            stage_valid <= 1'b0;
            if (replay_delay != 0) replay_delay <= replay_delay - 2'd1;
            if (raw_valid) begin
                stage_valid <= 1'b1;
                stage_eol <= raw_eol;
                x_q <= raw_x; y_q <= raw_y; bot_q <= raw_scaled;
                if (raw_sof) write_bank <= 2'd0;
                case (effective_bank)
                    2'd0: begin
                        line0[raw_x] <= raw_scaled;
                        mid_q <= line2[raw_x];
                        top_q <= line1[raw_x];
                    end
                    2'd1: begin
                        line1[raw_x] <= raw_scaled;
                        mid_q <= line0[raw_x];
                        top_q <= (raw_y == 1) ? line0[raw_x] : line2[raw_x];
                    end
                    default: begin
                        line2[raw_x] <= raw_scaled;
                        mid_q <= line1[raw_x];
                        top_q <= line0[raw_x];
                    end
                endcase
                if (raw_eol)
                    write_bank <= (effective_bank == 2'd2) ? 2'd0 : effective_bank + 2'd1;
                if (raw_eof) begin
                    last_bank <= effective_bank;
                    replay_x <= 12'd0;
                    replay_delay <= 2'd2;
                end
            end else if (replay_active || replay_delay == 2'd1) begin
                stage_valid <= 1'b1;
                stage_eol <= (replay_x == WIDTH-1);
                x_q <= replay_x; y_q <= HEIGHT;
                case (last_bank)
                    2'd0: begin
                        mid_q <= line0[replay_x]; bot_q <= line0[replay_x];
                        top_q <= (HEIGHT == 1) ? line0[replay_x] : line2[replay_x];
                    end
                    2'd1: begin
                        mid_q <= line1[replay_x]; bot_q <= line1[replay_x];
                        top_q <= line0[replay_x];
                    end
                    default: begin
                        mid_q <= line2[replay_x]; bot_q <= line2[replay_x];
                        top_q <= line1[replay_x];
                    end
                endcase
                replay_active <= (replay_x != WIDTH-1);
                replay_x <= replay_x + 12'd1;
            end
        end
    end

    // Stage 2: 3x3 shift window; output center (x-1,y-1). At line end,
    // one blank clock emits x=WIDTH-1 with its right neighbor replicated.
    reg [7:0] t2,t1,m2,m1,b2,b1;
    reg flush_right;
    reg [10:0] flush_y;
    wire [11:0] center_x = flush_right ? WIDTH-1 : x_q-12'd1;
    wire [10:0] center_y = flush_right ? flush_y-11'd1 : y_q-11'd1;
    wire [7:0] tr = flush_right ? t1 : top_q;
    wire [7:0] mr = flush_right ? m1 : mid_q;
    wire [7:0] br = flush_right ? b1 : bot_q;
    wire [9:0] cross_sum = {2'b00,t1}+{2'b00,m2}+{2'b00,mr}+{2'b00,b1};
    wire [9:0] diag_sum  = {2'b00,t2}+{2'b00,tr}+{2'b00,b2}+{2'b00,br};
    wire [8:0] horizontal_sum = {1'b0,m2}+{1'b0,mr};
    wire [8:0] vertical_sum = {1'b0,t1}+{1'b0,b1};
    // A normal demosaic result is only valid when all three rows and all
    // three columns are real samples.  Keep the pixel/line/frame token
    // cadence unchanged for the downstream packer, but force incomplete
    // border windows to black instead of treating replicated samples as a
    // normal RGB estimate.
    // x_q/y_q are already range-limited by the capture/replay generators;
    // testing bits above bit zero is the minimal form of >= 2 here.
    wire window_valid = (|x_q[11:1]) && (|y_q[10:1]);
    reg [7:0] red8,green8,blue8;
    // Preserve the measured channel at each CFA site. At R/B sites the
    // diagonals are the OPPOSITE colour; at green sites direct horizontal
    // and vertical neighbours are R/B. Mixing them into the center is not
    // same-colour denoising. A true same-CFA filter needs offsets of two.
    always @* begin
        case ({center_y[0],center_x[0]})
            2'b00: begin
                red8=m1; green8=cross_sum[9:2]; blue8=diag_sum[9:2];
            end
            2'b01: begin
                red8=horizontal_sum[8:1]; green8=m1;
                blue8=vertical_sum[8:1];
            end
            2'b10: begin
                red8=vertical_sum[8:1]; green8=m1;
                blue8=horizontal_sum[8:1];
            end
            default: begin
                red8=diag_sum[9:2]; green8=cross_sum[9:2]; blue8=m1;
            end
        endcase
    end
    wire [7:0] base_red = CFA_PHASE==0 ? red8 : blue8;
    wire [7:0] base_blue = CFA_PHASE==0 ? blue8 : red8;
    wire [9:0] red_gain_ext = RAW_GAIN_MULT != 0 ?
                              ({2'b00,base_red} * RAW_GAIN_MULT) :
                              ({2'b00,base_red} << RAW_GAIN_SHIFT);
    wire [9:0] green_gain_ext = RAW_GAIN_MULT != 0 ?
                                ({2'b00,green8} * RAW_GAIN_MULT) :
                                ({2'b00,green8} << RAW_GAIN_SHIFT);
    wire [9:0] blue_gain_ext = RAW_GAIN_MULT != 0 ?
                               ({2'b00,base_blue} * RAW_GAIN_MULT) :
                               ({2'b00,base_blue} << RAW_GAIN_SHIFT);
    wire [7:0] gained_red = !GAIN_AFTER_DEMOSAIC ? base_red :
                           ((|red_gain_ext[9:8]) ? 8'hff : red_gain_ext[7:0]);
    wire [7:0] gained_green = !GAIN_AFTER_DEMOSAIC ? green8 :
                             ((|green_gain_ext[9:8]) ? 8'hff : green_gain_ext[7:0]);
    wire [7:0] gained_blue = !GAIN_AFTER_DEMOSAIC ? base_blue :
                            ((|blue_gain_ext[9:8]) ? 8'hff : blue_gain_ext[7:0]);
    wire [9:0] luma_sum = {2'b00,gained_red} +
                           ({2'b00,gained_green} << 1) + {2'b00,gained_blue};
    wire [7:0] luma_est = luma_sum[9:2];
    wire [7:0] rgb_max = (gained_red >= gained_green) ?
                         ((gained_red >= gained_blue) ? gained_red : gained_blue) :
                         ((gained_green >= gained_blue) ? gained_green : gained_blue);
    wire [7:0] rgb_min = (gained_red <= gained_green) ?
                         ((gained_red <= gained_blue) ? gained_red : gained_blue) :
                         ((gained_green <= gained_blue) ? gained_green : gained_blue);
    wire [8:0] chroma_range = {1'b0,rgb_max} - {1'b0,rgb_min};
    wire lowlight_chroma = (LOWLIGHT_CHROMA_ENABLE != 0) &&
                           (luma_est <= LOWLIGHT_LUMA_THRESHOLD) &&
                           (chroma_range >= LOWLIGHT_CHROMA_THRESHOLD);
    // Retain 75% of the R/B excursion from green.  The 10-bit sums avoid
    // wraparound at 255; the result is naturally bounded to 8 bits.
    wire [9:0] red_blend_sum = ({2'b00,gained_red} * 10'd3) +
                               {2'b00,gained_green};
    wire [9:0] blue_blend_sum = ({2'b00,gained_blue} * 10'd3) +
                                {2'b00,gained_green};
    wire [7:0] output_red = lowlight_chroma ? red_blend_sum[9:2] : gained_red;
    wire [7:0] output_blue = lowlight_chroma ? blue_blend_sum[9:2] : gained_blue;
    wire [15:0] mixed_rgb={output_red[7:3],gained_green[7:2],output_blue[7:3]};
    always @(posedge pclk) begin
        if (!rst_n) begin
            t2<=0; t1<=0; m2<=0; m1<=0; b2<=0; b1<=0;
            flush_right<=0; flush_y<=0;
            rgb565<=0; valid<=0; sof<=0; eol<=0; eof<=0;
            x<=0; y<=0;
        end else begin
            valid<=0; sof<=0; eol<=0; eof<=0;
            if (flush_right) begin
                valid<=1; eol<=1;
                eof<=(flush_y==HEIGHT);
                x<=WIDTH-1; y<=flush_y-11'd1;
                rgb565<=16'd0;
                flush_right<=0;
            end
            if (stage_valid) begin
                if (x_q==0) begin
                    t2<=top_q; t1<=top_q;
                    m2<=mid_q; m1<=mid_q;
                    b2<=bot_q; b1<=bot_q;
                end else begin
                    t2<=t1; t1<=top_q;
                    m2<=m1; m1<=mid_q;
                    b2<=b1; b1<=bot_q;
                    if (y_q!=0) begin
                        valid<=1;
                        sof<=(x_q==1 && y_q==1);
                        x<=x_q-12'd1; y<=y_q-11'd1;
                        rgb565<=window_valid ? mixed_rgb : 16'd0;
                    end
                end
                if (stage_eol && y_q!=0) begin
                    flush_right<=1;
                    flush_y<=y_q;
                end
            end
        end
    end
endmodule
