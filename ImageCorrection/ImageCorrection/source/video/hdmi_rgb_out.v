module hdmi_rgb_out (
    input  wire        de,
    input  wire [11:0] x,
    input  wire [15:0] rgb565,
    input  wire        pixel_valid,
    output wire [7:0]  r,
    output wire [7:0]  g,
    output wire [7:0]  b
);
    reg [23:0] color_bar;
    always @* begin
        if      (x < 12'd240)  color_bar = 24'hFFFFFF;
        else if (x < 12'd480)  color_bar = 24'hFFFF00;
        else if (x < 12'd720)  color_bar = 24'h00FFFF;
        else if (x < 12'd960)  color_bar = 24'h00FF00;
        else if (x < 12'd1200) color_bar = 24'hFF00FF;
        else if (x < 12'd1440) color_bar = 24'hFF0000;
        else if (x < 12'd1680) color_bar = 24'h0000FF;
        else                   color_bar = 24'h000000;
    end
    wire [23:0] expanded = {rgb565[15:11], rgb565[15:13],
                            rgb565[10:5], rgb565[10:9],
                            rgb565[4:0], rgb565[4:2]};
    wire [23:0] selected = pixel_valid ? expanded : color_bar;
    assign {r, g, b} = de ? selected : 24'h000000;
endmodule
