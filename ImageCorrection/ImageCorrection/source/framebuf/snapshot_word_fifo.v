// Snapshot-specific synchronous-read Gray-pointer FIFO. Each domain has
// independent async-assert/sync-release reset; original video FIFO is untouched.
module snapshot_word_fifo #(parameter ADDR_BITS=8) (
    input wire wr_clk,rd_clk,rst_n,wr_en,rd_en,
    input wire [255:0] wr_data,
    output reg wr_full,wr_overflow,rd_empty,rd_underflow,
    output wire [ADDR_BITS:0] wr_used,
    output reg [255:0] rd_data
);
    (* ASYNC_REG="TRUE" *) reg [1:0] wr_reset,rd_reset;
    always @(posedge wr_clk or negedge rst_n)
        if(!rst_n) wr_reset<=0;else wr_reset<={wr_reset[0],1'b1};
    always @(posedge rd_clk or negedge rst_n)
        if(!rst_n) rd_reset<=0;else rd_reset<={rd_reset[0],1'b1};
    reg [255:0] memory[0:(1<<ADDR_BITS)-1];
    reg [ADDR_BITS:0] wr_bin,wr_gray,rd_bin,rd_gray;
    (* ASYNC_REG="TRUE" *) reg [ADDR_BITS:0] rd_w1,rd_w2,wr_r1,wr_r2;
    wire wr_take=wr_en && !wr_full,rd_take=rd_en && !rd_empty;
    wire [ADDR_BITS:0] wb_next=wr_bin+wr_take,rb_next=rd_bin+rd_take;
    wire [ADDR_BITS:0] wg_next=(wb_next>>1)^wb_next,rg_next=(rb_next>>1)^rb_next;
    function [ADDR_BITS:0] bin_from_gray;
        input [ADDR_BITS:0] gray;
        integer i;
        begin
            bin_from_gray[ADDR_BITS]=gray[ADDR_BITS];
            for(i=ADDR_BITS-1;i>=0;i=i-1) bin_from_gray[i]=bin_from_gray[i+1]^gray[i];
        end
    endfunction
    assign wr_used=wr_bin-bin_from_gray(rd_w2);
    always @(posedge wr_clk or negedge wr_reset[1]) begin
        if(!wr_reset[1]) begin wr_bin<=0;wr_gray<=0;rd_w1<=0;rd_w2<=0;wr_full<=0;wr_overflow<=0;end
        else begin
            rd_w1<=rd_gray;rd_w2<=rd_w1;
            if(wr_take) begin memory[wr_bin[ADDR_BITS-1:0]]<=wr_data;wr_bin<=wb_next;wr_gray<=wg_next;end
            if(wr_en && wr_full) wr_overflow<=1;
            wr_full<=wg_next=={~rd_w2[ADDR_BITS:ADDR_BITS-1],rd_w2[ADDR_BITS-2:0]};
        end
    end
    always @(posedge rd_clk or negedge rd_reset[1]) begin
        if(!rd_reset[1]) begin rd_bin<=0;rd_gray<=0;wr_r1<=0;wr_r2<=0;rd_empty<=1;rd_underflow<=0;rd_data<=0;end
        else begin
            wr_r1<=wr_gray;wr_r2<=wr_r1;
            if(rd_take) begin rd_data<=memory[rd_bin[ADDR_BITS-1:0]];rd_bin<=rb_next;rd_gray<=rg_next;end
            if(rd_en && rd_empty) rd_underflow<=1;
            rd_empty<=rg_next==wr_r2;
        end
    end
endmodule
