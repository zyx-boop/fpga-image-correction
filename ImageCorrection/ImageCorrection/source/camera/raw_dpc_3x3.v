// Centered RAW8 3x3 isolated-outlier correction. No gain or smoothing.
// RAW neighbours mix CFA colours: require ALL eight neighbours to agree
// within NEIGHBOR_RANGE and the center to lie THRESHOLD beyond their extrema.
// This intentionally leaves uncertain edges/colour detail untouched.
// Three unfiltered rotating row RAMs; synchronous read. All borders bypass.
// Output has one row + one column latency; last row replays during blanking.
module raw_dpc_3x3 #(
    parameter integer WIDTH=1920, HEIGHT=1080,
    parameter integer THRESHOLD=24, NEIGHBOR_RANGE=12
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
    (* syn_ramstyle = "block_ram" *) reg [7:0] line0[0:WIDTH-1];
    (* syn_ramstyle = "block_ram" *) reg [7:0] line1[0:WIDTH-1];
    (* syn_ramstyle = "block_ram" *) reg [7:0] line2[0:WIDTH-1];
    reg [1:0] write_bank,last_bank,replay_delay;
    reg replay_active;
    reg [11:0] replay_x;
    wire [1:0] effective_bank=in_sof ? 2'd0 : write_bank;
    wire replay_read=!in_valid && (replay_active || replay_delay==1);
    reg [7:0] top_q,mid_q,bot_q;
    reg [11:0] x_q;
    reg [10:0] y_q;
    reg stage_valid,stage_eol;
    // Isolate RAM process from reset so synthesis can infer block RAM.
    always @(posedge clk) begin
        if (rst_n && in_valid) begin
            case(effective_bank)
                0: begin
                    line0[in_x]<=raw;
                    mid_q<=line2[in_x]; top_q<=line1[in_x];
                end
                1: begin
                    line1[in_x]<=raw;
                    mid_q<=line0[in_x];
                    top_q<=(in_y==1) ? line0[in_x] : line2[in_x];
                end
                default: begin
                    line2[in_x]<=raw;
                    mid_q<=line1[in_x]; top_q<=line0[in_x];
                end
            endcase
            bot_q<=raw;
        end else if(rst_n && replay_read) begin
            case(last_bank)
                0: begin
                    mid_q<=line0[replay_x]; bot_q<=line0[replay_x];
                    top_q<=(HEIGHT==1) ? line0[replay_x] : line2[replay_x];
                end
                1: begin
                    mid_q<=line1[replay_x]; bot_q<=line1[replay_x];
                    top_q<=line0[replay_x];
                end
                default: begin
                    mid_q<=line2[replay_x]; bot_q<=line2[replay_x];
                    top_q<=line1[replay_x];
                end
            endcase
        end
    end
    always @(posedge clk) begin
        if(!rst_n) begin
            write_bank<=0;last_bank<=0;replay_delay<=0;replay_active<=0;
            replay_x<=0;stage_valid<=0;stage_eol<=0;x_q<=0;y_q<=0;
        end else begin
            stage_valid<=0;
            if(replay_delay!=0) replay_delay<=replay_delay-1'b1;
            if(in_valid) begin
                stage_valid<=1;stage_eol<=in_eol;x_q<=in_x;y_q<=in_y;
                if(in_sof) begin write_bank<=0;replay_active<=0;replay_delay<=0;end
                if(in_eol) write_bank<=effective_bank==2 ? 0 : effective_bank+1'b1;
                if(in_eof) begin
                    last_bank<=effective_bank;replay_x<=0;replay_delay<=2;
                end
            end else if(replay_read) begin
                stage_valid<=1;stage_eol<=(replay_x==WIDTH-1);
                x_q<=replay_x;y_q<=HEIGHT;
                replay_active<=(replay_x!=WIDTH-1);replay_x<=replay_x+1'b1;
            end
        end
    end
    reg [7:0] t2,t1,m2,m1,b2,b1;
    reg flush_right;
    reg [10:0] flush_y;
    wire [11:0] center_x=x_q-1'b1;
    wire [10:0] center_y=y_q-1'b1;
    wire interior=(x_q>=2 && x_q<WIDTH && y_q>=2 && y_q<HEIGHT);
    function [7:0] min2;
        input [7:0] a,b;
        begin min2=a<b ? a : b;end
    endfunction
    function [7:0] max2;
        input [7:0] a,b;
        begin max2=a>b ? a : b;end
    endfunction
    // m1 is the center; exclude it from all neighbour operations.
    wire [7:0] lo=min2(min2(min2(t2,t1),min2(top_q,m2)),
                       min2(min2(mid_q,b2),min2(b1,bot_q)));
    wire [7:0] hi=max2(max2(max2(t2,t1),max2(top_q,m2)),
                       max2(max2(mid_q,b2),max2(b1,bot_q)));
    wire [10:0] sum8={3'd0,t2}+{3'd0,t1}+{3'd0,top_q}+{3'd0,m2}+
                     {3'd0,mid_q}+{3'd0,b2}+{3'd0,b1}+{3'd0,bot_q};
    wire [10:0] rounded_sum=sum8+11'd4;
    wire [7:0] mean8=rounded_sum[10:3];
    wire [8:0] range8={1'b0,hi}-{1'b0,lo};
    wire [8:0] high_diff={1'b0,m1}-{1'b0,hi};
    wire [8:0] low_diff={1'b0,lo}-{1'b0,m1};
    wire isolated=((m1>hi && high_diff>THRESHOLD) ||
                   (m1<lo && low_diff>THRESHOLD));
    wire replace_pixel=interior && range8<=NEIGHBOR_RANGE && isolated;
    always @(posedge clk) begin
        if(!rst_n) begin
            t2<=0;t1<=0;m2<=0;m1<=0;b2<=0;b1<=0;
            flush_right<=0;flush_y<=0;clean<=0;
            out_valid<=0;out_sof<=0;out_eol<=0;out_eof<=0;out_x<=0;out_y<=0;
        end else begin
            out_valid<=0;out_sof<=0;out_eol<=0;out_eof<=0;
            if(flush_right) begin
                clean<=m1;out_valid<=1;out_eol<=1;
                out_eof<=(flush_y==HEIGHT);out_x<=WIDTH-1;out_y<=flush_y-1'b1;
                // WIDTH=1 has no center output in the normal window path.
                out_sof<=(WIDTH==1 && flush_y==1);
                flush_right<=0;
            end
            if(stage_valid) begin
                if(x_q==0) begin
                    t2<=top_q;t1<=top_q;m2<=mid_q;m1<=mid_q;b2<=bot_q;b1<=bot_q;
                end else begin
                    t2<=t1;t1<=top_q;m2<=m1;m1<=mid_q;b2<=b1;b1<=bot_q;
                    if(y_q!=0) begin
                        clean<=replace_pixel ? mean8 : m1;
                        out_valid<=1;out_sof<=(x_q==1 && y_q==1);
                        out_x<=center_x;out_y<=center_y;
                    end
                end
                if(stage_eol && y_q!=0) begin flush_right<=1;flush_y<=y_q;end
            end
        end
    end
endmodule
