// Single OV5640 register transaction: 7-bit device address, 16-bit register,
// 8-bit data. Read uses STOP + START between the address and read phases.
// top converts SCL to open-drain and feeds the physical pad into scl_in.
// 10 kHz diagnosis with weak pad pull-ups; this is not clock-stretch support.
module sccb_master #(
    parameter integer CLK_HZ = 27000000,
    parameter integer SCL_HZ = 10000,
    parameter [6:0] DEVICE_ADDR = 7'h3c
) (
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire read_en,
    input wire [15:0] reg_addr,
    input wire [7:0] wr_data,
    output reg [7:0] rd_data,
    output reg busy,
    output reg done,
    output reg error,
    output reg [3:0] error_code,
    output reg scl,
    input wire scl_in,
    inout wire sda,
    input wire probe_enable,
    output wire [11:0] probe_windows,
    output wire [3:0] probe_flags,
    output wire probe_first_valid, probe_result_valid,
    output wire [3:0] probe_pattern, probe_position,
    output wire probe_expected,
    output wire [3:0] probe_result_code,
    output wire [7:0] probe_result_data
);
    localparam integer QUARTER = (CLK_HZ + 4*SCL_HZ - 1)/(4*SCL_HZ);
    localparam [4:0] IDLE=0, START_A=1, START_B=2, START_C=3,
        TX0=4, TX1=5, TX2=6, TX3=7,
        ACK0=8, ACK1=9, ACK2=10, ACK3=11,
        RX0=12, RX1=13, RX2=14, RX3=15,
        NACK0=16, NACK1=17, NACK2=18, NACK3=19,
        STOP_A=20, STOP_B=21, STOP_C=22, STOP_D=23;
    reg [4:0] state;
    reg [31:0] divider;
    reg sda_low;
    reg sda_meta, sda_sync;
    reg scl_meta, scl_sync;
    reg [7:0] tx_byte, rx_byte, data_latched;
    reg [15:0] address_latched;
    reg [2:0] bit_index;
    reg [1:0] byte_index;
    reg read_latched, read_phase, restart_pending, ack_failed;
    reg sample_early, sample_middle, late_scl_bad;
    wire majority_bit = (sample_early & sample_middle) |
                        (sample_early & sda_sync) | (sample_middle & sda_sync);
    wire [7:0] probe_expected_byte = 8'h05; // stress reads 3808=05 only

    assign sda = sda_low ? 1'b0 : 1'bz;

    // ACK/data majority uses +0.5Q, +Q, +1.5Q after SCL release.
    // The passive probe observes the same samples and preserves anomaly counts.
    sccb_window_probe window_probe(.clk(clk),.rst_n(rst_n),.enable(probe_enable),
        .early_tick((state==ACK2 || state==RX2) && divider==QUARTER/2-1),
        .middle_tick((state==ACK2 || state==RX2) && divider==QUARTER-1),
        .late_tick((state==ACK3 || state==RX3) && divider==QUARTER/2-1),
        .ack_window(state==ACK3),.sda_sample(sda_sync),.scl_sample(scl_sync),
        .position(state==ACK3 ? (read_phase ? 4'd4 : {2'b00,byte_index}) : {1'b0,bit_index}),
        .expected_bit(state==ACK3 ? 1'b0 : probe_expected_byte[bit_index]),
        .transaction_done(done),.transaction_error(error),
        .transaction_code(error_code),.transaction_data(rd_data),
        .unstable_windows(probe_windows),.flags(probe_flags),
        .first_valid(probe_first_valid),.result_valid(probe_result_valid),
        .first_pattern(probe_pattern),.first_position(probe_position),
        .first_expected(probe_expected),.first_result_code(probe_result_code),
        .first_result_data(probe_result_data));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sda_meta <= 1;
            sda_sync <= 1;
            scl_meta <= 1;
            scl_sync <= 1;
        end else begin
            sda_meta <= sda;
            sda_sync <= sda_meta;
            scl_meta <= scl_in;
            scl_sync <= scl_meta;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            divider <= 0;
            scl <= 1;
            sda_low <= 0;
            busy <= 0;
            done <= 0;
            error <= 0;
            error_code <= 0;
            rd_data <= 0;
            tx_byte <= 0;
            rx_byte <= 0;
            data_latched <= 0;
            address_latched <= 0;
            bit_index <= 7;
            byte_index <= 0;
            read_latched <= 0;
            read_phase <= 0;
            restart_pending <= 0;
            ack_failed <= 0;
            sample_early <= 1;
            sample_middle <= 1;
            late_scl_bad <= 0;
        end else begin
            done <= 0;
            if ((state==ACK2 || state==RX2) && divider==QUARTER/2-1)
                sample_early <= sda_sync;
            if ((state==ACK2 || state==RX2) && divider==QUARTER-1)
                sample_middle <= sda_sync;
            // Commit before the +2Q falling edge. RX bit0 is therefore stored
            // before RX3 copies rx_byte into rd_data; no stale-last-bit hazard.
            if ((state==ACK3 || state==RX3) && divider==QUARTER/2-1) begin
                late_scl_bad <= !scl_sync;
                if (state==ACK3) ack_failed <= majority_bit;
                else rx_byte[bit_index] <= majority_bit;
            end
            if (state == IDLE) begin
                divider <= 0;
                scl <= 1;
                sda_low <= 0;
                if (start) begin
                    busy <= 1;
                    error <= 0;
                    error_code <= 0;
                    rd_data <= 0;
                    rx_byte <= 0;
                    address_latched <= reg_addr;
                    data_latched <= wr_data;
                    read_latched <= read_en;
                    read_phase <= 0;
                    restart_pending <= 0;
                    byte_index <= 0;
                    bit_index <= 7;
                    tx_byte <= {DEVICE_ADDR, 1'b0};
                    state <= START_A;
                end
            end else if (divider == QUARTER - 1) begin
                divider <= 0;
                case (state)
                    START_A: begin scl <= 1; sda_low <= 0; state <= START_B; end
                    START_B: begin
                        if (!scl_sync) begin
                            error <= 1;
                            error_code <= 4'd10; // released SCL did not rise
                            restart_pending <= 0;
                            state <= STOP_A;
                        end else if (!sda_sync) begin
                            error <= 1;
                            error_code <= 4'd1;
                            restart_pending <= 0;
                            state <= STOP_A;
                        end else begin
                            sda_low <= 1;
                            state <= START_C;
                        end
                    end
                    START_C: begin
                        scl <= 0;
                        if (sda_sync) begin
                            error <= 1;
                            error_code <= 4'd11; // driven SDA did not fall
                            state <= STOP_A;
                        end else state <= TX0;
                    end
                    TX0: begin
                        if (scl_sync) begin
                            error <= 1;
                            error_code <= 4'd12; // driven SCL did not fall
                            state <= STOP_A;
                        end else begin
                            sda_low <= !tx_byte[bit_index];
                            state <= TX1;
                        end
                    end
                    TX1: begin scl <= 1; state <= TX2; end
                    TX2: begin
                        if (!scl_sync) begin
                            error <= 1;
                            error_code <= 4'd10;
                            state <= STOP_A;
                        end else if (sda_sync != tx_byte[bit_index]) begin
                            error <= 1;
                            error_code <= 4'd13; // transmitted SDA bit mismatch
                            state <= STOP_A;
                        end else state <= TX3;
                    end
                    TX3: begin
                        scl <= 0;
                        if (bit_index == 0) state <= ACK0;
                        else begin bit_index <= bit_index - 1'b1; state <= TX0; end
                    end
                    ACK0: begin sda_low <= 0; state <= ACK1; end
                    ACK1: begin scl <= 1; late_scl_bad <= 0; state <= ACK2; end
                    ACK2: begin
                        if (!scl_sync) begin
                            error <= 1;
                            error_code <= 4'd10;
                            state <= STOP_A;
                        end else state <= ACK3;
                    end
                    ACK3: begin
                        scl <= 0;
                        bit_index <= 7;
                        if (late_scl_bad) begin
                            error <= 1;
                            error_code <= 4'd10;
                            restart_pending <= 0;
                            state <= STOP_A;
                        end else if (ack_failed) begin
                            error <= 1;
                            if (read_phase) error_code <= 4'd5;
                            else case (byte_index)
                                0: error_code <= 4'd2; // device write address 0x78
                                1: error_code <= 4'd3; // register address high byte
                                2: error_code <= 4'd4; // register address low byte
                                3: error_code <= 4'd6; // write data (not used by ID test)
                            endcase
                            restart_pending <= 0;
                            state <= STOP_A;
                        end else if (read_phase) begin
                            rx_byte <= 0;
                            state <= RX0;
                        end else begin
                            case (byte_index)
                                0: begin
                                    tx_byte <= address_latched[15:8];
                                    byte_index <= 1;
                                    state <= TX0;
                                end
                                1: begin
                                    tx_byte <= address_latched[7:0];
                                    byte_index <= 2;
                                    state <= TX0;
                                end
                                2: if (read_latched) begin
                                    restart_pending <= 1;
                                    state <= STOP_A;
                                end else begin
                                    tx_byte <= data_latched;
                                    byte_index <= 3;
                                    state <= TX0;
                                end
                                3: state <= STOP_A;
                            endcase
                        end
                    end
                    RX0: begin sda_low <= 0; state <= RX1; end
                    RX1: begin scl <= 1; late_scl_bad <= 0; state <= RX2; end
                    RX2: begin
                        if (!scl_sync) begin
                            error <= 1;
                            error_code <= 4'd10;
                            state <= STOP_A;
                        end else state <= RX3;
                    end
                    RX3: begin
                        scl <= 0;
                        if (late_scl_bad) begin
                            error <= 1;
                            error_code <= 4'd10;
                            state <= STOP_A;
                        end else if (bit_index == 0) begin
                            rd_data <= rx_byte;
                            state <= NACK0;
                        end else begin bit_index <= bit_index - 1'b1; state <= RX0; end
                    end
                    // End a one-byte read by leaving SDA high on the ninth clock.
                    NACK0: begin sda_low <= 0; state <= NACK1; end
                    NACK1: begin scl <= 1; state <= NACK2; end
                    NACK2: state <= NACK3;
                    NACK3: begin scl <= 0; state <= STOP_A; end
                    STOP_A: begin scl <= 0; sda_low <= 1; state <= STOP_B; end
                    STOP_B: begin scl <= 1; state <= STOP_C; end
                    STOP_C: begin sda_low <= 0; state <= STOP_D; end
                    STOP_D: begin
                        if (restart_pending) begin
                            restart_pending <= 0;
                            read_phase <= 1;
                            tx_byte <= {DEVICE_ADDR, 1'b1};
                            bit_index <= 7;
                            state <= START_A;
                        end else begin
                            busy <= 0;
                            done <= 1;
                            state <= IDLE;
                        end
                    end
                    default: begin
                        scl <= 1;
                        sda_low <= 0;
                        busy <= 0;
                        done <= 1;
                        error <= 1;
                        error_code <= 4'd9;
                        state <= IDLE;
                    end
                endcase
            end else divider <= divider + 1'b1;
        end
    end
endmodule
