module hdmi_1080p30_timing (
    input  wire        clk,
    input  wire        rst_n,
    output wire        hs,
    output wire        vs,
    output wire        de,
    output wire [11:0] x,
    output wire [10:0] y,
    output wire        frame_start,
    output wire        vblank_start
);
    // CEA-861 1920x1080 progressive at 30 Hz: 74.25 MHz pixel clock.
    localparam integer H_ACTIVE = 1920;
    localparam integer H_FRONT  = 88;
    localparam integer H_SYNC   = 44;
    localparam integer H_BACK   = 148;
    localparam integer V_ACTIVE = 1080;
    localparam integer V_FRONT  = 4;
    localparam integer V_SYNC   = 5;
    localparam integer V_BACK   = 36;
    localparam integer H_TOTAL  = H_ACTIVE + H_FRONT + H_SYNC + H_BACK;
    localparam integer V_TOTAL  = V_ACTIVE + V_FRONT + V_SYNC + V_BACK;
    localparam integer H_START  = H_SYNC + H_BACK;
    localparam integer V_START  = V_SYNC + V_BACK;

    reg [11:0] h_count;
    reg [10:0] v_count;

    // Reset is sampled in the pixel domain; do not release an asynchronous
    // reset directly from the unrelated 10 MHz configuration clock.
    always @(posedge clk) begin
        if (!rst_n) begin
            h_count <= 12'd0;
            v_count <= 11'd0;
        end else if (h_count == H_TOTAL - 1) begin
            h_count <= 12'd0;
            if (v_count == V_TOTAL - 1)
                v_count <= 11'd0;
            else
                v_count <= v_count + 11'd1;
        end else begin
            h_count <= h_count + 12'd1;
        end
    end

    assign hs = (h_count < H_SYNC);
    assign vs = (v_count < V_SYNC);
    assign de = (h_count >= H_START) && (h_count < H_START + H_ACTIVE) &&
                (v_count >= V_START) && (v_count < V_START + V_ACTIVE);
    assign x = de ? h_count - H_START : 12'd0;
    assign y = de ? v_count - V_START : 11'd0;
    assign frame_start = (h_count == H_START) && (v_count == V_START);
    assign vblank_start = (h_count == 12'd0) && (v_count == V_START + V_ACTIVE);
endmodule
