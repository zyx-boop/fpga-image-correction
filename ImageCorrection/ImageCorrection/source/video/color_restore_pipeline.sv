// Preserve AWB/statistics and all downstream complete-frame protections.
module color_restore_pipeline #(
 parameter WIDTH=1920,HEIGHT=1080,SAMPLE_LOG2=4,MIN_NEUTRAL=128,
 parameter TONE_FILE="data/color_tone_gain.hex"
)(
 input wire clk,rst_n,in_valid,input wire [23:0] in_rgb,
 input wire [11:0] in_x,input wire [10:0] in_y,input wire in_sof,in_eol,in_eof,
 input wire frame_done,frame_good,
 output wire out_valid,output wire [23:0] out_rgb,output wire [15:0] out565,
 output wire [11:0] out_x,output wire [10:0] out_y,output wire out_sof,out_eol,out_eof,
 output wire [12:0] debug_r,debug_g,debug_b,output wire [7:0] debug_tone,
 output wire [15:0] debug_n,debug_h,output wire [23:0] debug_v
);
 wire [12:0] gain_r,gain_g,gain_b;wire [7:0] unused_global_tone;
 wire sv,ss,se;wire [23:0] sr,sw;wire [11:0] sx;wire [10:0] sy;
 wire [11:0] q00,q10,q01,q11;wire [4:0] fx,fy;
 wire map_ready;wire [3:0] gamma_bin,map_state;
 assign debug_r=gain_r;assign debug_g=gain_g;assign debug_b=gain_b;
 // Different algorithm: this field now exposes active Gamma bin, not D175 tone.
 assign debug_tone={4'b0,gamma_bin};
 color_adapt_control #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.SAMPLE_LOG2(SAMPLE_LOG2),.MIN_NEUTRAL(MIN_NEUTRAL)) adapt(
  .clk(clk),.rst_n(rst_n),.apply_sof(in_valid&&in_sof),.stat_valid(sv),.stat_raw(sr),.stat_wb(sw),
  .stat_x(sx),.stat_y(sy),.stat_sof(ss),.stat_eof(se),.frame_done(frame_done),.frame_good(frame_good),
  .gain_r(gain_r),.gain_g(gain_g),.gain_b(gain_b),.tone_pos(unused_global_tone),
  .debug_n(debug_n),.debug_h(debug_h),.debug_v(debug_v));
 msag_map #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) illumination(
  .clk(clk),.rst_n(rst_n),.apply_sof(in_valid&&in_sof),
  .stat_valid(sv),.stat_rgb(sw),.stat_x(sx),.stat_y(sy),.stat_sof(ss),.stat_eof(se),
  .frame_done(frame_done),.frame_good(frame_good),
  .pixel_valid(in_valid),.pixel_x(in_x),.pixel_y(in_y),
  .q00(q00),.q10(q10),.q01(q01),.q11(q11),.frac_x(fx),.frac_y(fy),
  .map_ready(map_ready),.active_gamma(gamma_bin),.debug_state(map_state));
 color_pixel_pipe #(.TONE_FILE(TONE_FILE)) pixels(
  .clk(clk),.rst_n(rst_n),.in_valid(in_valid),.in_rgb(in_rgb),.in_x(in_x),.in_y(in_y),.in_sof(in_sof),.in_eol(in_eol),.in_eof(in_eof),
  .gain_r(gain_r),.gain_g(gain_g),.gain_b(gain_b),.tone_pos(unused_global_tone),
  .map00(q00),.map10(q10),.map01(q01),.map11(q11),.map_fx(fx),.map_fy(fy),
  .out_valid(out_valid),.out_rgb(out_rgb),.out565(out565),.out_x(out_x),.out_y(out_y),.out_sof(out_sof),.out_eol(out_eol),.out_eof(out_eof),
  .stat_valid(sv),.stat_raw(sr),.stat_wb(sw),.stat_x(sx),.stat_y(sy),.stat_sof(ss),.stat_eof(se));
endmodule
