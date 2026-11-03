`timescale 1ns / 1ps
// Receive-only compatibility with WIT/JY61 11-byte 55 53 Euler packets.
// Sliding window recovers from inserted/dropped bytes and embedded 0x55 data.
module r15_jy61_decoder #(
    parameter integer CLOCK_HZ=27000000,
    parameter integer BAUD=115200
)(
    input wire clk, rst_n, rx,
    output reg sample_valid,
    output reg [15:0] roll, pitch, yaw,
    output reg [31:0] byte_count,
    output reg [15:0] bad_count,
    output reg confirmed, output wire rx_level
);
    wire [7:0] data;
    wire valid;
    r15_imu_uart_rx #(.CLOCK_HZ(CLOCK_HZ),.BAUD(BAUD)) u_rx
        (.clk(clk),.rst_n(rst_n),.rx(rx),.data(data),.valid(valid),.rx_level(rx_level));
    reg [7:0] window [0:9];
    reg [3:0] filled;
    reg [31:0] gap, confirmation_age;
    reg seen_one;
    reg [7:0] sum;
    integer k,j;
    always @* begin
        sum=0;
        for(j=0;j<10;j=j+1) sum=sum+window[j];
    end
    wire known_header = window[0]==8'h55 &&
        (window[1]==8'h51 || window[1]==8'h52 || window[1]==8'h53);
    always @(posedge clk) begin
        if(!rst_n) begin
            filled<=0; gap<=0; confirmation_age<=0; seen_one<=0; confirmed<=0;
            roll<=0; pitch<=0; yaw<=0; sample_valid<=0; byte_count<=0; bad_count<=0;
            for(k=0;k<10;k=k+1) window[k]<=0;
        end else begin
            sample_valid<=0;
            if(confirmation_age<CLOCK_HZ/2) confirmation_age<=confirmation_age+1'b1;
            else begin seen_one<=0; confirmed<=0; end
            if(valid) begin
                byte_count<=byte_count+1'b1;
                gap<=0;
                for(k=0;k<9;k=k+1) window[k]<=window[k+1];
                window[9]<=data;
                if(filled<10) filled<=filled+1'b1;
                else if(known_header) begin
                    if(sum!=data) begin
                        if(bad_count!=16'hffff) bad_count<=bad_count+1'b1;
                    end else if(window[1]==8'h53) begin
                        roll<={window[3],window[2]};
                        pitch<={window[5],window[4]};
                        yaw<={window[7],window[6]};
                        sample_valid<=1;
                        confirmation_age<=0; seen_one<=1;
                        // Two checksum-correct angle packets qualify a baud rate.
                        if(seen_one && confirmation_age<CLOCK_HZ/2) confirmed<=1;
                    end
                end
            end else if(gap<CLOCK_HZ/50) gap<=gap+1'b1;
            else filled<=0;
        end
    end
endmodule

module r15_jy61_uart #(
    parameter integer CLOCK_HZ=27000000,
    parameter integer FAST_BAUD=115200,
    parameter integer SLOW_BAUD=9600,
    parameter integer STALE_MS=500
)(
    input wire clk, rst_n, rx,
    output reg [15:0] roll, pitch, yaw,
    output reg has_sample,
    output wire fresh,
    output reg [15:0] age_ms,
    output reg [31:0] sample_count,
    output wire [31:0] byte_count,
    output wire [15:0] bad_count,
    output reg [7:0] baud_code,
    output wire rx_level,
    output reg [31:0] edge_count, min_edge_ticks,
    output reg [15:0] idle_ms
);
    wire fast_valid,slow_valid,fast_confirmed,slow_confirmed;
    wire [15:0] fast_roll,fast_pitch,fast_yaw,slow_roll,slow_pitch,slow_yaw;
    wire [31:0] fast_bytes,slow_bytes;
    wire [15:0] fast_bad,slow_bad;
    r15_jy61_decoder #(.CLOCK_HZ(CLOCK_HZ),.BAUD(FAST_BAUD)) u_fast
        (.clk(clk),.rst_n(rst_n),.rx(rx),.sample_valid(fast_valid),
         .roll(fast_roll),.pitch(fast_pitch),.yaw(fast_yaw),
         .byte_count(fast_bytes),.bad_count(fast_bad),.confirmed(fast_confirmed),.rx_level(rx_level));
    r15_jy61_decoder #(.CLOCK_HZ(CLOCK_HZ),.BAUD(SLOW_BAUD)) u_slow
        (.clk(clk),.rst_n(rst_n),.rx(rx),.sample_valid(slow_valid),
         .roll(slow_roll),.pitch(slow_pitch),.yaw(slow_yaw),
         .byte_count(slow_bytes),.bad_count(slow_bad),.confirmed(slow_confirmed));
    assign byte_count=baud_code==1 ? fast_bytes : baud_code==2 ? slow_bytes : fast_bytes+slow_bytes;
    assign bad_count=baud_code==2 ? slow_bad : fast_bad;
    assign fresh=has_sample && age_ms<STALE_MS;
    reg [31:0] ms_ticks;
    reg previous_level;
    reg [31:0] edge_ticks, idle_ticks;
    // Observe the existing two-stage synchronized input without another CDC path.
    // This distinguishes quiet/stuck lines from data at an unsupported baud rate.
    always @(posedge clk) begin
        if(!rst_n) begin
            previous_level<=1; edge_ticks<=0; idle_ticks<=0;
            edge_count<=0; min_edge_ticks<=32'hffffffff; idle_ms<=0;
        end else begin
            previous_level<=rx_level;
            if(rx_level!=previous_level) begin
                edge_count<=edge_count+1'b1;
                if(edge_count!=0 && edge_ticks!=32'hffffffff && edge_ticks<min_edge_ticks)
                    min_edge_ticks<=edge_ticks+1'b1;
                edge_ticks<=0; idle_ticks<=0; idle_ms<=0;
            end else begin
                if(edge_ticks!=32'hffffffff) edge_ticks<=edge_ticks+1'b1;
                if(idle_ticks==CLOCK_HZ/1000-1) begin
                    idle_ticks<=0;
                    if(idle_ms!=16'hffff) idle_ms<=idle_ms+1'b1;
                end else idle_ticks<=idle_ticks+1'b1;
            end
        end
    end
    wire choose_fast=fast_valid && fast_confirmed && (baud_code!=2 || !fresh);
    wire choose_slow=slow_valid && slow_confirmed && (baud_code!=1 || !fresh) && !choose_fast;
    always @(posedge clk) begin
        if(!rst_n) begin
            roll<=0; pitch<=0; yaw<=0; has_sample<=0; age_ms<=16'hffff;
            sample_count<=0; baud_code<=0; ms_ticks<=0;
        end else begin
            if(ms_ticks==CLOCK_HZ/1000-1) begin
                ms_ticks<=0;
                if(age_ms!=16'hffff) age_ms<=age_ms+1'b1;
            end else ms_ticks<=ms_ticks+1'b1;
            if(choose_fast || choose_slow) begin
                roll<=choose_fast ? fast_roll : slow_roll;
                pitch<=choose_fast ? fast_pitch : slow_pitch;
                yaw<=choose_fast ? fast_yaw : slow_yaw;
                baud_code<=choose_fast ? 1 : 2;
                has_sample<=1; age_ms<=0; ms_ticks<=0;
                sample_count<=sample_count+1'b1;
            end
        end
    end
endmodule
