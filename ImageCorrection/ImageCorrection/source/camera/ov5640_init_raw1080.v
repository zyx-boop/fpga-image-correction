// SCCB register writer with ACK/timeout handling and final critical readback.
module ov5640_init_raw1080 #(
    parameter integer CLK_HZ = 27000000,
    parameter integer TEST_COLORBAR = 0,
    parameter integer LOW_PCLK_DIAG = 0
) (
    input wire clk, rst_n, enable,
    input wire scl_idle_in, sda_idle_in,
    output reg start,
    output wire read_en,
    output wire [15:0] reg_addr,
    output wire [7:0] wr_data,
    input wire bus_done, bus_error,
    input wire [7:0] rd_data,
    output reg configured, failed,
    output reg [8:0] failed_index,
    output reg [15:0] failed_addr,
    output reg [7:0] expected_data, observed_data,
    output reg [7:0] pre_r4740, pre_r4741,
    output reg pre_read_ok, pre_scl_idle, pre_sda_idle
);
    localparam IDLE=4'd0, SEND=4'd1, WAIT_BUS=4'd2, GAP=4'd3,
               VERIFY_SEND=4'd4, VERIFY_WAIT=4'd5,
               FINISHED=4'd6, FAILED=4'd7,
               PRE_READ_SEND=4'd8, PRE_READ_WAIT=4'd9;
    reg [3:0] state;
    reg [8:0] index;
    reg [4:0] verify_index;
    reg verification_done;
    reg pre_read_index, pre_first_ok;
    reg [31:0] timer;
    wire [23:0] rom_word;
    wire last;
    ov5640_cfg_raw1080 #(.TEST_COLORBAR(TEST_COLORBAR),
                         .LOW_PCLK_DIAG(LOW_PCLK_DIAG)) u_rom(
        .index(index), .word(rom_word), .last(last));

    reg [23:0] check_word;
    always @* begin
        case (verify_index)
            0: check_word=24'h380807;
            1: check_word=24'h380980;
            2: check_word=24'h380a04;
            3: check_word=24'h380b38;
            4: check_word=24'h303511;
            5: check_word=24'h4300f8;
            6: check_word=24'h501f03;
            7: check_word=24'h382100;
            8: check_word=24'h382402;
            9: check_word=24'h300842;
            10: check_word=24'h380e04;
            11: check_word=24'h380f60;
            12: check_word=24'h30021c;
            13: check_word=24'h3006c3;
            14: check_word=24'h460c22;
            15: check_word=24'h503d80;
            default: check_word=24'h000000;
        endcase
    end
    assign read_en = (state == VERIFY_SEND || state == VERIFY_WAIT ||
                      state == PRE_READ_SEND || state == PRE_READ_WAIT);
    assign reg_addr = (state == PRE_READ_SEND || state == PRE_READ_WAIT) ?
                      (pre_read_index ? 16'h4741 : 16'h4740) :
                      (read_en ? check_word[23:8] : rom_word[23:8]);
    assign wr_data = rom_word[7:0];

    always @(posedge clk) begin
        if (!rst_n) begin
            state<=IDLE; index<=0; verify_index<=0; verification_done<=0; timer<=0;
            pre_read_index<=0; pre_first_ok<=0;
            start<=0; configured<=0; failed<=0; failed_index<=0;
            failed_addr<=0; expected_data<=0; observed_data<=0;
            pre_r4740<=0; pre_r4741<=0; pre_read_ok<=0;
            pre_scl_idle<=0; pre_sda_idle<=0;
        end else begin
            start<=0;
            case (state)
                IDLE: if (enable) state<=SEND;
                SEND: begin
                    if (last && !verification_done) begin
                        verify_index<=0;
                        state<=VERIFY_SEND;
                    end else begin
                        start<=1; timer<=0; state<=WAIT_BUS;
                    end
                end
                WAIT_BUS: begin
                    if (bus_done) begin
                        timer<=0;
                        if (bus_error) begin
                            failed<=1; failed_index<=index;
                            failed_addr<=rom_word[23:8];
                            expected_data<=rom_word[7:0];
                            state<=FAILED;
                        end else if (last) begin configured<=1; state<=FINISHED; end
                        else state<=GAP;
                    end else if (timer == CLK_HZ/10-1) begin
                        failed<=1; failed_index<=index;
                        failed_addr<=rom_word[23:8];
                        expected_data<=rom_word[7:0];
                        state<=FAILED;
                    end else timer<=timer+1'b1;
                end
                GAP: begin
                    if (timer == ((rom_word == 24'h300882) ? CLK_HZ/50-1 : CLK_HZ/1000-1)) begin
                        timer<=0;
                        index<=index+1'b1; state<=SEND;
                    end else timer<=timer+1'b1;
                end
                VERIFY_SEND: begin start<=1; timer<=0; state<=VERIFY_WAIT; end
                VERIFY_WAIT: begin
                    if (bus_done) begin
                        timer<=0;
                        if (bus_error || rd_data != check_word[7:0]) begin
                            failed<=1; failed_index<={4'd0,verify_index};
                            failed_addr<=check_word[23:8];
                            expected_data<=check_word[7:0];
                            observed_data<=rd_data;
                            state<=FAILED;
                        end else if (verify_index == (TEST_COLORBAR ? 5'd15 : 5'd14)) begin
                            // Keep the original verification result and
                            // stream-on sequence, but insert two diagnostic
                            // reads before the final 0x3008=0x02 write. These
                            // reads never compare against an expected value.
                            pre_read_index<=0;
                            pre_first_ok<=1;
                            pre_read_ok<=0;
                            pre_scl_idle<=scl_idle_in;
                            pre_sda_idle<=sda_idle_in;
                            state<=PRE_READ_SEND;
                        end else begin
                            verify_index<=verify_index+5'd1;
                            state<=VERIFY_SEND;
                        end
                    end else if (timer == CLK_HZ/10-1) begin
                        failed<=1; failed_index<={4'd0,verify_index};
                        failed_addr<=check_word[23:8];
                        expected_data<=check_word[7:0];
                        state<=FAILED;
                    end else timer<=timer+1'b1;
                end
                PRE_READ_SEND: begin
                    start<=1; timer<=0; state<=PRE_READ_WAIT;
                end
                PRE_READ_WAIT: begin
                    if (bus_done) begin
                        timer<=0;
                        if (!pre_read_index) begin
                            pre_r4740<=rd_data;
                            pre_first_ok<=!bus_error;
                            pre_read_index<=1;
                            state<=PRE_READ_SEND;
                        end else begin
                            pre_r4741<=rd_data;
                            pre_read_ok<=pre_first_ok && !bus_error;
                            verification_done<=1;
                            state<=SEND;
                        end
                    end else if (timer == CLK_HZ/10-1) begin
                        timer<=0;
                        if (!pre_read_index) begin
                            pre_first_ok<=0;
                            pre_read_index<=1;
                            state<=PRE_READ_SEND;
                        end else begin
                            pre_read_ok<=0;
                            verification_done<=1;
                            state<=SEND;
                        end
                    end else timer<=timer+1'b1;
                end
                FINISHED: ;
                FAILED: ;
                default: state<=FAILED;
            endcase
        end
    end
endmodule
