// Frame-statistics observer. Never controls valid, capture, publication or DDR.
// Pending parameters become visible only at an input SOF, including its pixel.
module color_adapt_control #(
 parameter WIDTH=1920,HEIGHT=1080,SAMPLE_LOG2=4,MIN_NEUTRAL=128
)(
 input wire clk,rst_n,apply_sof,stat_valid,
 input wire [23:0] stat_raw,stat_wb,
 input wire [11:0] stat_x,input wire [10:0] stat_y,
 input wire stat_sof,stat_eof,frame_done,frame_good,
 output wire [12:0] gain_r,gain_g,gain_b,output wire [7:0] tone_pos,
 output wire[15:0]debug_n,debug_h,output wire[23:0]debug_v
);
 localparam integer SAMPLE_STEP=(1<<SAMPLE_LOG2);
 localparam integer SAMPLE_OFFSET=SAMPLE_STEP/2;
 reg [12:0] active_r,active_g,active_b,pending_r,pending_g,pending_b;
 reg [7:0] active_tone,pending_tone;
 reg [2:0] active_zone,pending_zone;
 reg pending_valid;
 wire use_pending=apply_sof&&pending_valid;
 assign gain_r=use_pending?pending_r:active_r;
 assign gain_g=use_pending?pending_g:active_g;
 assign gain_b=use_pending?pending_b:active_b;
 assign tone_pos=use_pending?pending_tone:active_tone;

 reg running,overflow,eof_pending,snap_overflow;
 reg [15:0] samples,neutrals,highlights,snap_samples,snap_neutrals,snap_high;
 reg [23:0] sum_v,sum_r,sum_g,sum_b,snap_v,snap_r,snap_g,snap_b;
 reg [3:0] quadrants,snap_quadrants;
 assign debug_n=snap_samples;assign debug_h=snap_high;assign debug_v=snap_v;
 function automatic [7:0] seed_scale(input [7:0] v,input [12:0] g);
  reg [21:0] z;begin z=v*g+22'd2048;seed_scale=z[19:12];end
 endfunction
 wire [7:0] seed_r=stat_raw[23:16];
 wire [7:0] seed_g=seed_scale(stat_raw[15:8],13'd2775);
 wire [7:0] seed_b=seed_scale(stat_raw[7:0],13'd3260);
 wire [7:0] seed_hi_rg=seed_r>seed_g?seed_r:seed_g;
 wire [7:0] seed_lo_rg=seed_r<seed_g?seed_r:seed_g;
 wire [7:0] seed_hi=seed_hi_rg>seed_b?seed_hi_rg:seed_b;
 wire [7:0] seed_lo=seed_lo_rg<seed_b?seed_lo_rg:seed_b;
 wire [9:0] chroma4={2'b0,seed_hi-seed_lo}<<2;
 // Seed attenuation must not disguise censored raw channels as neutral.
 wire raw_unclipped=stat_raw[23:16]<250&&stat_raw[15:8]<250&&stat_raw[7:0]<250;
 wire neutral=raw_unclipped&&seed_hi>=32&&seed_hi<=224&&chroma4<={2'b0,seed_hi};
 wire [7:0] wb_hi_rg=stat_wb[23:16]>stat_wb[15:8]?stat_wb[23:16]:stat_wb[15:8];
 wire [7:0] wb_hi=wb_hi_rg>stat_wb[7:0]?wb_hi_rg:stat_wb[7:0];
 // One observation-only boundary cuts seed classification from accumulation.
 // The pixel stream and existing capture/frame certificate are not delayed.
 reg obs_valid,obs_sof,obs_eof,obs_neutral;
 reg [11:0] obs_x;reg [10:0] obs_y;
 reg [23:0] obs_raw;reg [7:0] obs_hi;
 always @(posedge clk)begin
  if(!rst_n)begin obs_valid<=0;obs_sof<=0;obs_eof<=0;obs_neutral<=0;obs_x<=0;obs_y<=0;obs_raw<=0;obs_hi<=0;end
  else begin
   obs_valid<=stat_valid;
   if(stat_valid)begin obs_sof<=stat_sof;obs_eof<=stat_eof;obs_neutral<=neutral;obs_x<=stat_x;obs_y<=stat_y;obs_raw<=stat_raw;obs_hi<=wb_hi;end
  end
 end
 wire sample=obs_valid&&(running||obs_sof)&&obs_x>0&&obs_x<WIDTH-1&&obs_y>0&&obs_y<HEIGHT-1
  &&((obs_x&(SAMPLE_STEP-1))==SAMPLE_OFFSET)&&((obs_y&(SAMPLE_STEP-1))==SAMPLE_OFFSET);
 wire [1:0] quadrant={obs_y>=HEIGHT/2,obs_x>=WIDTH/2};
 wire [15:0] base_n=obs_sof?16'd0:samples,base_c=obs_sof?16'd0:neutrals,base_h=obs_sof?16'd0:highlights;
 wire [23:0] base_v=obs_sof?24'd0:sum_v,base_r=obs_sof?24'd0:sum_r,base_g=obs_sof?24'd0:sum_g,base_b=obs_sof?24'd0:sum_b;
 wire [16:0] next_n={1'b0,base_n}+sample;
 wire [16:0] next_c={1'b0,base_c}+(sample&&obs_neutral);
 wire [16:0] next_h={1'b0,base_h}+(sample&&obs_hi>=240);
 // Meter the visible non-saturated region. Bright windows/screens must not
 // force an otherwise dark subject onto a globally darker curve.
 wire [24:0] next_v={1'b0,base_v}+(sample&&obs_hi<240?{17'b0,obs_hi}:25'd0);
 wire [24:0] next_r={1'b0,base_r}+(sample&&obs_neutral?{17'b0,obs_raw[23:16]}:25'd0);
 wire [24:0] next_g={1'b0,base_g}+(sample&&obs_neutral?{17'b0,obs_raw[15:8]}:25'd0);
 wire [24:0] next_b={1'b0,base_b}+(sample&&obs_neutral?{17'b0,obs_raw[7:0]}:25'd0);
 wire [3:0] next_q=(obs_sof?4'd0:quadrants)|(sample&&obs_neutral?(4'b0001<<quadrant):4'd0);
 // Reachable-state invariant: subcounts<=N, sums<=255*N. Before N
 // exceeds 65535 no 24-bit sum or subcount can overflow. N overflow
 // is sticky until SOF, so this is exactly the original overflow guard.
 wire next_overflow=(obs_sof?1'b0:overflow)|next_n[16];

 wire [3:0] quadrant_count={3'd0,snap_quadrants[0]}+{3'd0,snap_quadrants[1]}+{3'd0,snap_quadrants[2]}+{3'd0,snap_quadrants[3]};
 wire confident=snap_neutrals>=MIN_NEUTRAL&&({4'd0,snap_neutrals}<<4)>={4'd0,snap_samples}&&quadrant_count>=2&&snap_r!=0&&snap_g!=0&&snap_b!=0;
 wire [23:0] min_rg=snap_r<snap_g?snap_r:snap_g;
 wire [23:0] min_sum=min_rg<snap_b?min_rg:snap_b;
 function automatic [7:0] boundary(input [2:0] z);
  begin case(z)1:boundary=48;2:boundary=72;3:boundary=96;4:boundary=144;5:boundary=176;default:boundary=208;endcase end
 endfunction
 reg [2:0] chosen_zone;
 reg [7:0] chosen_tone;
 reg [23:0] lower_limit,upper_limit;
 // Reuse the existing counters; no new frame counter or image storage.
 wire [15:0] tone_samples=snap_samples-snap_high;
 wire tone_confident=snap_high<=snap_samples&&tone_samples!=0
  &&({4'd0,tone_samples}<<4)>={4'd0,snap_samples};
 always @* begin
  lower_limit=tone_samples*(boundary(active_zone-1'b1)-8'd4);
  upper_limit=tone_samples*(boundary(active_zone)+8'd4);
  chosen_zone=active_zone;
  chosen_tone=active_tone;
  if(tone_confident)begin
   if(active_zone>1&&snap_v<lower_limit)chosen_zone=active_zone-1'b1;
   else if(active_zone<7&&snap_v>upper_limit)chosen_zone=active_zone+1'b1;
   if(active_tone<{1'b0,chosen_zone,4'b0})chosen_tone=active_tone+1'b1;
   else if(active_tone>{1'b0,chosen_zone,4'b0})chosen_tone=active_tone-1'b1;
  end
 end

 reg div_start;
 reg [35:0] numerator;
 reg [23:0] denominator;
 wire div_busy,div_done,div_zero;
 wire [35:0] quotient;
 reg [1:0] state;
 color_ratio_divider divider(.clk(clk),.rst_n(rst_n),.start(div_start),.numerator(numerator),.denominator(denominator),
  .busy(div_busy),.done(div_done),.divide_by_zero(div_zero),.quotient(quotient));
 function automatic [12:0] bounded_step(input [35:0] q,input [12:0] old,input [12:0] lo,input [12:0] hi);
  reg [12:0] target;begin
   target=q<lo?lo:(q>hi?hi:q[12:0]);
   if(target>old+13'd8)bounded_step=old+13'd8;
   else if(target+13'd8<old)bounded_step=old-13'd8;
   else bounded_step=target;
  end
 endfunction
 // Freeze division operands against the following frame's statistics.
 reg [23:0] calc_g,calc_b;
 always @(posedge clk)begin
  if(!rst_n)begin
   active_r<=4096;active_g<=2775;active_b<=3260;active_tone<=64;active_zone<=4;
   pending_r<=4096;pending_g<=2775;pending_b<=3260;pending_tone<=64;pending_zone<=4;pending_valid<=0;
   running<=0;overflow<=0;eof_pending<=0;snap_overflow<=0;
   samples<=0;neutrals<=0;highlights<=0;sum_v<=0;sum_r<=0;sum_g<=0;sum_b<=0;quadrants<=0;
   snap_samples<=0;snap_neutrals<=0;snap_high<=0;snap_v<=0;snap_r<=0;snap_g<=0;snap_b<=0;snap_quadrants<=0;
   state<=0;div_start<=0;numerator<=0;denominator<=0;calc_g<=0;calc_b<=0;
  end else begin
   div_start<=0;
   if(use_pending)begin
    active_r<=pending_r;active_g<=pending_g;active_b<=pending_b;active_tone<=pending_tone;active_zone<=pending_zone;pending_valid<=0;
   end
   if(obs_valid&&(running||obs_sof))begin
    samples<=next_n[15:0];neutrals<=next_c[15:0];highlights<=next_h[15:0];sum_v<=next_v[23:0];
    sum_r<=next_r[23:0];sum_g<=next_g[23:0];sum_b<=next_b[23:0];quadrants<=next_q;overflow<=next_overflow;
    if(obs_sof)begin running<=1;eof_pending<=0;end
    if(obs_eof)begin
     running<=0;eof_pending<=1;snap_samples<=next_n[15:0];snap_neutrals<=next_c[15:0];snap_high<=next_h[15:0];
     snap_v<=next_v[23:0];snap_r<=next_r[23:0];snap_g<=next_g[23:0];snap_b<=next_b[23:0];snap_quadrants<=next_q;snap_overflow<=next_overflow;
    end
   end
   if(frame_done)begin
    eof_pending<=0;
    if(frame_good&&eof_pending&&!snap_overflow&&snap_samples!=0&&state==0&&!pending_valid&&!apply_sof)begin
     pending_r<=active_r;pending_g<=active_g;pending_b<=active_b;pending_tone<=chosen_tone;pending_zone<=chosen_zone;
     if(confident)begin numerator<={min_sum,12'b0};denominator<=snap_r;calc_g<=snap_g;calc_b<=snap_b;div_start<=1;state<=1;end
     else pending_valid<=1;
    end
   end
   if(div_done&&state!=0)begin
    if(div_zero)state<=0;
    else case(state)
     1:begin pending_r<=bounded_step(quotient,active_r,13'd3072,13'd4096);denominator<=calc_g;div_start<=1;state<=2;end
     2:begin pending_g<=bounded_step(quotient,active_g,13'd2220,13'd3330);denominator<=calc_b;div_start<=1;state<=3;end
     3:begin pending_b<=bounded_step(quotient,active_b,13'd2608,13'd3912);pending_valid<=1;state<=0;end
    endcase
   end
  end
 end
endmodule
