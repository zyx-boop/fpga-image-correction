// Full signed value, nearest/ties-away, then saturation. No wrap-before-clip.
module fixed_round_sat #(
    parameter IN_WIDTH=61, OUT_WIDTH=28, SHIFT=24
) (
    input wire signed [IN_WIDTH-1:0] value,
    output reg signed [OUT_WIDTH-1:0] result,
    output reg sat
);
    localparam signed [IN_WIDTH:0] LIMIT_LOW = -( {{IN_WIDTH{1'b0}},1'b1} << (OUT_WIDTH-1));
    localparam signed [IN_WIDTH:0] LIMIT_HIGH = ( {{IN_WIDTH{1'b0}},1'b1} << (OUT_WIDTH-1)) - 1;
    localparam [IN_WIDTH:0] HALF = (SHIFT>0) ? ({{IN_WIDTH{1'b0}},1'b1} << (SHIFT-1)) : 0;
    reg signed [IN_WIDTH:0] extended_value, rounded_value;
    // floor(v/2^SHIFT) plus a single increment, exactly nearest/ties-away.
    // For negative ties floor already points away from zero. No abs/negate
    // carry chains are required. Keep the full-width saturation below.
    wire increment;
    generate if(SHIFT==0) begin:g_no_shift
        assign increment=1'b0;
    end else if(SHIFT==1) begin:g_one_shift
        assign increment=value[0] && !value[IN_WIDTH-1];
    end else begin:g_shift
        assign increment=value[SHIFT-1] &&
                         (!value[IN_WIDTH-1] || (|value[SHIFT-2:0]));
    end endgenerate
    generate if (SHIFT<0 || SHIFT>IN_WIDTH || OUT_WIDTH<2 || OUT_WIDTH>IN_WIDTH) begin : illegal
        initial $fatal(1,"invalid fixed_round_sat parameters");
    end endgenerate
    always @* begin
        extended_value={value[IN_WIDTH-1],value};
        rounded_value=(extended_value >>> SHIFT) + $signed({{IN_WIDTH{1'b0}},increment});
        sat=1'b0;
        if (rounded_value<LIMIT_LOW) begin result=LIMIT_LOW[OUT_WIDTH-1:0]; sat=1'b1; end
        else if (rounded_value>LIMIT_HIGH) begin result=LIMIT_HIGH[OUT_WIDTH-1:0]; sat=1'b1; end
        else result=rounded_value[OUT_WIDTH-1:0];
    end
endmodule
