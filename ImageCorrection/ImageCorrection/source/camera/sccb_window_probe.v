// Passive digital observation only: no feedback into bus control or data decisions.
// Count bit windows with unequal SDA samples, not failed transactions.
module sccb_window_probe(
    input wire clk, rst_n, enable,
    input wire early_tick, middle_tick, late_tick, ack_window,
    input wire sda_sample, scl_sample,
    input wire [3:0] position,
    input wire expected_bit,
    input wire transaction_done, transaction_error,
    input wire [3:0] transaction_code,
    input wire [7:0] transaction_data,
    output reg [11:0] unstable_windows,
    output reg [3:0] flags,
    output reg first_valid, result_valid,
    output reg [3:0] first_pattern, first_position,
    output reg first_expected,
    output reg [3:0] first_result_code,
    output reg [7:0] first_result_data
);
    reg early_data, middle_data, have_early, have_middle;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            early_data<=0; middle_data<=0; have_early<=0; have_middle<=0;
            unstable_windows<=0; flags<=0;
            first_valid<=0; result_valid<=0; first_pattern<=0;
            first_position<=0; first_expected<=0;
            first_result_code<=0; first_result_data<=0;
        end else if(enable) begin
            // done is asserted only at final STOP, not the inter-phase STOP.
            // Freeze the outcome of the transaction containing the first event.
            if(first_valid && !result_valid && transaction_done) begin
                result_valid<=1;
                first_result_data<=transaction_data;
                first_result_code<=transaction_error ? transaction_code :
                                   (transaction_data==8'h05 ? 4'd0 : 4'd14);
            end
            if(early_tick) begin
                early_data<=sda_sample; have_early<=1; have_middle<=0;
                if(!scl_sample) flags[2]<=1;
            end
            if(middle_tick && have_early) begin
                middle_data<=sda_sample; have_middle<=1;
                if(!scl_sample) flags[2]<=1;
            end
            if(late_tick && have_early && have_middle) begin
                have_early<=0; have_middle<=0;
                if(!scl_sample) flags[2]<=1;
                if(early_data!=middle_data || middle_data!=sda_sample) begin
                    if(!first_valid) begin
                        first_valid<=1;
                        first_pattern<={ack_window,early_data,middle_data,sda_sample};
                        first_position<=position;
                        first_expected<=expected_bit;
                    end
                    if(unstable_windows==12'hfff) flags[3]<=1;
                    else unstable_windows<=unstable_windows+1'b1;
                    if(ack_window) flags[0]<=1;
                    else flags[1]<=1;
                end
            end
        end else begin
            // Freeze results when stress ends; don't include initialization.
            have_early<=0; have_middle<=0;
        end
    end
endmodule
