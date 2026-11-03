// Neutral RAW8 3x3 bilinear demosaic, camera PCLK domain only.
// CFA_PHASE: 0=RGGB, 1=BGGR, 2=GRBG, 3=GBRG at input coordinate (0,0).
// No gain, denoise, white balance, gamma, or data-dependent pixel rejection.
// Output retains WIDTH*HEIGHT pixels; the one-pixel perimeter is black.
// Interior center (x-1,y-1) waits for the next RAW row/column. One blank
// clock flushes the right border; the bottom border is replayed in VBLANK.
// Required input blanking: >=1 clock after EOL and >=WIDTH+8 after EOF.
module bayer_bilinear_rgb888 #(
    parameter integer WIDTH=1920, HEIGHT=1080, CFA_PHASE=1
)(
    input wire pclk,rst_n,
    input wire [7:0] raw,
    input wire raw_valid,raw_sof,raw_eol,raw_eof,
    input wire [11:0] raw_x,
    input wire [10:0] raw_y,
    output reg [23:0] rgb888,
    output wire [15:0] rgb565,
    output reg valid,sof,eol,eof,line_start,
    output reg [11:0] x,
    output reg [10:0] y
);
    // Read-before-write single-clock RAM. No RAM reset, no combinational
    // RAM read: all three RAM outputs and their metadata share one stage.
    (* syn_ramstyle="block_ram" *) reg [7:0] line0[0:WIDTH-1];
    (* syn_ramstyle="block_ram" *) reg [7:0] line1[0:WIDTH-1];
    (* syn_ramstyle="block_ram" *) reg [7:0] line2[0:WIDTH-1];
    reg [7:0] row0_q,row1_q,row2_q;
    reg [1:0] write_bank;
    wire [1:0] effective_bank=raw_sof ? 2'd0 : write_bank;
    always @(posedge pclk) begin
        if(rst_n && raw_valid) begin
            row0_q<=line0[raw_x];
            row1_q<=line1[raw_x];
            row2_q<=line2[raw_x];
            case(effective_bank)
                0:line0[raw_x]<=raw;
                1:line1[raw_x]<=raw;
                2:line2[raw_x]<=raw;
            endcase
        end
    end

    // Stage 1: RAW column and metadata registered with the RAM reads.
    reg v_q,l_q;
    reg [7:0] bottom_q;
    reg [1:0] bank_q,replay_delay;
    reg replay_active;
    reg [11:0] x_q,replay_x;
    reg [10:0] y_q;
    always @(posedge pclk) begin
        if(!rst_n) begin
            write_bank<=0;v_q<=0;l_q<=0;bottom_q<=0;bank_q<=0;
            x_q<=0;y_q<=0;replay_delay<=0;replay_active<=0;replay_x<=0;
        end else begin
            v_q<=0;l_q<=0;
            if(replay_delay!=0) replay_delay<=replay_delay-2'd1;
            if(raw_valid) begin
                v_q<=1;l_q<=raw_eol;bottom_q<=raw;bank_q<=effective_bank;
                x_q<=raw_x;y_q<=raw_y;
                if(raw_sof) begin
                    write_bank<=0;replay_active<=0;replay_delay<=0;
                end
                if(raw_eol) write_bank<=effective_bank==2 ? 2'd0 : effective_bank+2'd1;
                if(raw_eof) begin replay_delay<=2;replay_x<=0;end
            end else if(replay_active || replay_delay==1) begin
                // The complete bottom row is border-black. No line RAM
                // read is needed and no unknown RAM value enters its window.
                v_q<=1;l_q<=(replay_x==WIDTH-1);bottom_q<=0;
                x_q<=replay_x;y_q<=HEIGHT;
                replay_active<=(replay_x!=WIDTH-1);
                replay_x<=replay_x+12'd1;
            end
        end
    end
    wire [7:0] previous_row=bank_q==0 ? row2_q : bank_q==1 ? row0_q : row1_q;
    wire [7:0] older_row=bank_q==0 ? row1_q : bank_q==1 ? row2_q : row0_q;
    // Rows unavailable in this frame are explicitly zeroed. Only centers
    // with y>=1 and y<HEIGHT-1 are interpolated, after both prior rows exist.
    wire [7:0] top_sample=(y_q>=2 && y_q<HEIGHT) ? older_row : 8'd0;
    wire [7:0] mid_sample=(y_q>=1 && y_q<HEIGHT) ? previous_row : 8'd0;
    reg [7:0] t2,t1,m2,m1,b2,b1;
    reg flush_right;
    reg [10:0] flush_y;
    localparam [1:0] CFA_XOR=CFA_PHASE==0 ? 2'b00 : CFA_PHASE==1 ? 2'b11 :
                               CFA_PHASE==2 ? 2'b01 : 2'b10;

    // Stage 2: center-aligned window sums and tokens. Shift only on valid,
    // restart columns at x=0. The black borders still carry valid tokens.
    reg win_valid,win_border;
    reg [11:0] win_x;
    reg [10:0] win_y;
    reg [1:0] win_phase;
    reg [7:0] center;
    reg [9:0] cross_sum,diag_sum;
    reg [8:0] horizontal_sum,vertical_sum;
    wire [11:0] cx=x_q-12'd1;
    wire [10:0] cy=y_q-11'd1;
    always @(posedge pclk) begin
        if(!rst_n) begin
            t2<=0;t1<=0;m2<=0;m1<=0;b2<=0;b1<=0;
            flush_right<=0;flush_y<=0;win_valid<=0;win_border<=1;
            win_x<=0;win_y<=0;win_phase<=0;center<=0;
            cross_sum<=0;diag_sum<=0;horizontal_sum<=0;vertical_sum<=0;
        end else begin
            win_valid<=0;
            if(flush_right) begin
                win_valid<=1;win_border<=1;
                win_x<=WIDTH-1;win_y<=flush_y-11'd1;
                flush_right<=0;
            end
            if(v_q) begin
                if(x_q==0) begin
                    t2<=0;t1<=top_sample;m2<=0;m1<=mid_sample;
                    b2<=0;b1<=bottom_q;
                end else begin
                    t2<=t1;t1<=top_sample;m2<=m1;m1<=mid_sample;
                    b2<=b1;b1<=bottom_q;
                    if(y_q!=0) begin
                        win_valid<=1;win_x<=cx;win_y<=cy;
                        win_phase<={cy[0],cx[0]} ^ CFA_XOR;
                        win_border<=(cx==0 || cy==0 || cy==HEIGHT-1);
                        // Widths cover 2*255=510 and 4*255=1020.
                        // Incomplete windows do not participate in arithmetic.
                        if(cx!=0 && cy!=0 && cy<HEIGHT-1) begin
                            center<=m1;
                            cross_sum<={2'b0,t1}+{2'b0,m2}+{2'b0,mid_sample}+{2'b0,b1};
                            diag_sum<={2'b0,t2}+{2'b0,top_sample}+{2'b0,b2}+{2'b0,bottom_q};
                            horizontal_sum<={1'b0,m2}+{1'b0,mid_sample};
                            vertical_sum<={1'b0,t1}+{1'b0,b1};
                        end else begin
                            center<=0;cross_sum<=0;diag_sum<=0;
                            horizontal_sum<=0;vertical_sum<=0;
                        end
                    end
                end
                if(l_q && y_q!=0) begin flush_right<=1;flush_y<=y_q;end
            end
        end
    end

    // Stage 3: RGB888, coordinates and all markers registered together.
    reg [23:0] interpolated;
    always @* begin
        case(win_phase)
            0:interpolated={center,cross_sum[9:2],diag_sum[9:2]};
            1:interpolated={horizontal_sum[8:1],center,vertical_sum[8:1]};
            2:interpolated={vertical_sum[8:1],center,horizontal_sum[8:1]};
            3:interpolated={diag_sum[9:2],cross_sum[9:2],center};
        endcase
    end
    always @(posedge pclk) begin
        if(!rst_n) begin
            rgb888<=0;valid<=0;sof<=0;eol<=0;eof<=0;line_start<=0;x<=0;y<=0;
        end else begin
            valid<=win_valid;
            sof<=win_valid && win_x==0 && win_y==0;
            line_start<=win_valid && win_x==0;
            eol<=win_valid && win_x==WIDTH-1;
            eof<=win_valid && win_x==WIDTH-1 && win_y==HEIGHT-1;
            if(win_valid) begin
                x<=win_x;y<=win_y;rgb888<=win_border ? 24'd0 : interpolated;
            end
        end
    end
    // No additional latency or gain: direct R[7:3],G[7:2],B[7:3] truncation.
    assign rgb565={rgb888[23:19],rgb888[15:10],rgb888[7:3]};
endmodule
