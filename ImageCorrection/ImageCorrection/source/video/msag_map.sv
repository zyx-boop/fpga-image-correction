// Observation-only illumination map builder. No pixel backpressure or CDC.
// Only a complete, ordered, certified good frame may launch map computation.
// A finished map becomes readable at SOF; a map being built is never displayed.
module msag_map #(
 parameter WIDTH=1920,HEIGHT=1080,
 parameter GAMMA_FILE="data/msag_gamma_q9.hex"
)(
 input wire clk,rst_n,apply_sof,
 input wire stat_valid,input wire [23:0] stat_rgb,
 input wire [11:0] stat_x,input wire [10:0] stat_y,
 input wire stat_sof,stat_eof,frame_done,frame_good,
 input wire pixel_valid,input wire [11:0] pixel_x,input wire [10:0] pixel_y,
 output reg [11:0] q00,q10,q01,q11,
 output reg [4:0] frac_x,frac_y,
 output wire map_ready,output reg [3:0] active_gamma,
 output wire [3:0] debug_state
);
 localparam MW=(WIDTH+15)/16,MH=(HEIGHT+15)/16,COUNT=MW*MH;
 localparam PW=(MW+1)/2,PH=(MH+1)/2,PD=PW*PH;
 localparam TOTAL=WIDTH*HEIGHT,ADDR_BITS=$clog2(COUNT);
 localparam IDLE=0,CAPTURE=1,WAIT_GOOD=2,GAMMA=3,
            TAP_REQUEST=4,TAP_SUM=5,LUT_REQUEST=6,LUT_SUM=7,STORE=8,PENDING=9,
            TAP_COORDS=10,TAP_ADDRESS=11;
 reg [3:0] state;
 assign debug_state=state;
 reg active_bank,ready,pending;
 reg [3:0] gamma_bin;
 wire switch_now=apply_sof&&pending;
 wire read_bank=switch_now?~active_bank:active_bank;
 assign map_ready=ready||switch_now;
 (* syn_ramstyle="block_ram" *) reg [7:0] raw_map[0:COUNT-1];
 // Four parity banks give four neighbors per cycle without four full copies.
 (* syn_ramstyle="block_ram" *) reg [11:0] g00[0:2*PD-1];
 (* syn_ramstyle="block_ram" *) reg [11:0] g10[0:2*PD-1];
 (* syn_ramstyle="block_ram" *) reg [11:0] g01[0:2*PD-1];
 (* syn_ramstyle="block_ram" *) reg [11:0] g11[0:2*PD-1];
 (* syn_ramstyle="block_ram" *) reg [11:0] gamma_lut[0:4095];
 (* syn_ramstyle="distributed_ram" *) reg [15:0] column_sum[0:MW-1];
 initial $readmemh(GAMMA_FILE,gamma_lut);
 reg [11:0] expected_x;reg [10:0] expected_y;
 reg [21:0] seen,dark_count;
 reg malformed;
 reg [12:0] row_sum;
 reg [15:0] column_q;
 wire [9:0] rgb_sum={2'b0,stat_rgb[23:16]}+{2'b0,stat_rgb[15:8]}+{2'b0,stat_rgb[7:0]}+10'd1;
 wire [21:0] intensity_product=rgb_sum*12'd2731;
 // Register the observation-only intensity before row/block accumulation.
 // All statistics metadata follows that same register; display tokens and
 // their latency are untouched. The certified output EOF is still later.
 reg [7:0] intensity;
 reg obs_valid,obs_sof,obs_eof;
 reg [11:0] obs_x;reg [10:0] obs_y;
 always @(posedge clk)begin
  if(!rst_n)begin intensity<=0;obs_valid<=0;obs_sof<=0;obs_eof<=0;obs_x<=0;obs_y<=0;end
  else begin
   obs_valid<=stat_valid;
   if(stat_valid)begin
    intensity<=intensity_product[20:13];obs_sof<=stat_sof;obs_eof<=stat_eof;obs_x<=stat_x;obs_y<=stat_y;
   end
  end
 end
 wire collecting=(state==CAPTURE)||(state==IDLE&&obs_sof&&!pending);
 wire [12:0] horizontal=(obs_x[3:0]==0?13'd0:row_sum)+intensity;
 wire [16:0] block_sum={1'b0,(obs_y[3:0]==0?16'd0:column_q)}+horizontal;
 wire block_end_x=(obs_x[3:0]==15)||(obs_x==WIDTH-1);
 wire block_end_y=(obs_y[3:0]==15)||(obs_y==HEIGHT-1);
 // Supported product widths are divisible by 16; height may end with 8 rows.
 wire [7:0] block_mean=obs_y>=HEIGHT-(HEIGHT%16)&&HEIGHT%16==8?
                         (block_sum+17'd64)>>7:(block_sum+17'd128)>>8;
 integer bx,by,ex,ox,ey,oy,base;
 reg [11:0] r00,r10,r01,r11;
 reg parity_x,parity_y,read_valid,read_has_map;
 // Read addresses and phase come exclusively from token coordinates.
 always @* begin
  bx=pixel_x<8?0:(pixel_x-8)>>4;
  by=pixel_y<8?0:(pixel_y-8)>>4;
  if(bx>=MW)bx=MW-1;if(by>=MH)by=MH-1;
  ex=(bx+1)>>1;ox=bx>>1;ey=(by+1)>>1;oy=by>>1;
  if(ex>=PW)ex=PW-1;if(ey>=PH)ey=PH-1;
  base=read_bank?PD:0;
 end
 // Single synchronous read port per parity RAM. The engine only writes the
 // opposite bank, using each RAM's independent write port.
 always @(posedge clk)begin
  if(pixel_valid)begin
   r00<=g00[base+ey*PW+ex];r10<=g10[base+ey*PW+ox];
   r01<=g01[base+oy*PW+ex];r11<=g11[base+oy*PW+ox];
   parity_x<=bx[0];parity_y<=by[0];
   frac_x<=pixel_x<8||bx==MW-1?0:((pixel_x<<1)-15)&31;
   frac_y<=pixel_y<8||by==MH-1?0:((pixel_y<<1)-15)&31;
  end
  read_valid<=pixel_valid;read_has_map<=map_ready;
 end
 // Reorder the fixed parity ports into the four logical neighboring samples.
 always @* begin
  if(!read_has_map)begin q00=512;q10=512;q01=512;q11=512;end
  else case({parity_y,parity_x})
   0:begin q00=r00;q10=r10;q01=r01;q11=r11;end
   1:begin q00=r10;q10=r00;q01=r11;q11=r01;end
   2:begin q00=r01;q10=r11;q01=r00;q11=r10;end
   3:begin q00=r11;q10=r01;q01=r10;q11=r00;end
  endcase
 end

 reg [6:0] map_x,map_y;
 reg [1:0] scale;
 reg [2:0] tap_x,tap_y;
 reg [17:0] filter_sum;
 reg [7:0] raw_q;
 reg [11:0] lut_q;
 reg [13:0] fusion;
 reg [11:0] write_gain;
 reg [3:0] certified_gamma;
 integer wa;
 reg signed [8:0] candidate_x,candidate_y;
 reg [6:0] clamped_x,clamped_y;
 reg [ADDR_BITS-1:0] raw_address;
 wire [ADDR_BITS-1:0] row_index=clamped_y;
 wire [ADDR_BITS-1:0] row_address;
 generate if(MW==120)begin : g_width120
  // 120*y = 128*y - 8*y. This stage contains only a narrow
  // constant row-offset calculation, independent of scale/clamping.
  assign row_address=(row_index<<7)-(row_index<<3);
 end else begin : g_other_width
  assign row_address=row_index*MW;
 end endgenerate
 reg [5:0] weight;
 function automatic signed [8:0] tap_offset(input [2:0] tap,input [1:0] s);
  reg signed [8:0] delta;
  begin
   delta=$signed({1'b0,tap})-9'sd2;
   case(s)0:tap_offset=delta;1:tap_offset=delta<<<2;default:tap_offset=delta<<<3;endcase
  end
 endfunction
 function automatic [2:0] k(input [2:0] i);
  begin case(i)0,4:k=1;1,3:k=4;default:k=6;endcase end
 endfunction
 always @* begin
  candidate_x=$signed({2'b0,map_x})+tap_offset(tap_x,scale);
  candidate_y=$signed({2'b0,map_y})+tap_offset(tap_y,scale);
  weight=k(tap_x)*k(tap_y);
  wa=(active_bank?0:PD)+(map_y>>1)*PW+(map_x>>1);
 end
 always @(posedge clk)begin
  if(state==TAP_REQUEST)raw_q<=raw_map[raw_address];
  if(state==LUT_REQUEST)lut_q<=gamma_lut[{gamma_bin,8'b0}+((filter_sum+128)>>8)];
  if(state==STORE)case({map_y[0],map_x[0]})
   0:g00[wa]<=write_gain;1:g10[wa]<=write_gain;
   2:g01[wa]<=write_gain;3:g11[wa]<=write_gain;
  endcase
  if(obs_valid&&collecting&&obs_x[3:0]==0)column_q<=column_sum[obs_x>>4];
  if(obs_valid&&collecting&&block_end_x)begin
   if(block_end_y)raw_map[(obs_y>>4)*MW+(obs_x>>4)]<=block_mean;
   else column_sum[obs_x>>4]<=block_sum[15:0];
  end
 end
 reg [17:0] weighted_sample;
 wire [17:0] sample_wide={10'b0,raw_q};
 always @* begin
  case(weight)
   1:weighted_sample=sample_wide;
   4:weighted_sample=sample_wide<<2;
   6:weighted_sample=(sample_wide<<2)+(sample_wide<<1);
   16:weighted_sample=sample_wide<<4;
   24:weighted_sample=(sample_wide<<4)+(sample_wide<<3);
   36:weighted_sample=(sample_wide<<5)+(sample_wide<<2);
   default:weighted_sample=0;
  endcase
 end
 wire [17:0] next_filter=filter_sum+weighted_sample;
 wire [13:0] next_fusion=fusion+(scale==2?{1'b0,lut_q,1'b0}:{2'b0,lut_q});
 always @(posedge clk)begin
  if(!rst_n)begin
   state<=IDLE;active_bank<=0;ready<=0;pending<=0;active_gamma<=0;
   gamma_bin<=0;certified_gamma<=0;expected_x<=0;expected_y<=0;
   seen<=0;dark_count<=0;malformed<=0;row_sum<=0;
   map_x<=0;map_y<=0;scale<=0;tap_x<=0;tap_y<=0;
   clamped_x<=0;clamped_y<=0;raw_address<=0;
   filter_sum<=0;fusion<=0;write_gain<=512;
  end else begin
   if(switch_now)begin active_bank<=~active_bank;ready<=1;pending<=0;active_gamma<=gamma_bin;state<=IDLE;end
   if(obs_valid&&collecting)begin
    row_sum<=horizontal;
    if(state==IDLE)begin
     seen<=1;dark_count<=intensity<64;malformed<=obs_x!=0||obs_y!=0;
     expected_x<=WIDTH==1?0:1;expected_y<=0;state<=CAPTURE;
    end else begin
     seen<=seen+1'b1;dark_count<=dark_count+(intensity<64);
     if(obs_x!=expected_x||obs_y!=expected_y||obs_sof)malformed<=1;
     if(expected_x==WIDTH-1)begin expected_x<=0;expected_y<=expected_y+1'b1;end
     else expected_x<=expected_x+1'b1;
    end
    if(obs_eof)begin
     if(state==CAPTURE&&seen==TOTAL-1&&!malformed&&obs_x==WIDTH-1&&obs_y==HEIGHT-1)state<=WAIT_GOOD;
     else state<=IDLE;
    end
   end
   if(state==CAPTURE&&frame_done)state<=IDLE; // aborted/truncated source
   if(state==WAIT_GOOD)begin
    if(obs_valid&&obs_sof)state<=IDLE; // stale certificate cannot cross SOF
    else if(frame_done)begin
     if(frame_good)begin certified_gamma<=0;state<=GAMMA;end
     else state<=IDLE;
    end
   end
   if(state==GAMMA)begin
    // Floor(16*dark/TOTAL), no pixel-rate divider. Fifteen comparisons.
    if(certified_gamma<15&&({4'b0,dark_count}<<4)>=(certified_gamma+1)*TOTAL)
     certified_gamma<=certified_gamma+1'b1;
    else begin gamma_bin<=certified_gamma;map_x<=0;map_y<=0;scale<=0;tap_x<=0;tap_y<=0;filter_sum<=0;fusion<=0;state<=TAP_COORDS;end
   end
   if(state==TAP_COORDS)begin
    clamped_x<=candidate_x<0?0:(candidate_x>=MW?MW-1:candidate_x[6:0]);
    clamped_y<=candidate_y<0?0:(candidate_y>=MH?MH-1:candidate_y[6:0]);
    state<=TAP_ADDRESS;
   end
   if(state==TAP_ADDRESS)begin raw_address<=row_address+clamped_x;state<=TAP_REQUEST;end
   if(state==TAP_REQUEST)state<=TAP_SUM;
   if(state==TAP_SUM)begin
    filter_sum<=next_filter;
    if(tap_x==4&&tap_y==4)state<=LUT_REQUEST;
    else begin
     if(tap_x==4)begin tap_x<=0;tap_y<=tap_y+1'b1;end else tap_x<=tap_x+1'b1;
     state<=TAP_COORDS;
    end
   end
   if(state==LUT_REQUEST)state<=LUT_SUM;
   if(state==LUT_SUM)begin
    fusion<=next_fusion;
    if(scale==2)begin write_gain<=(next_fusion+2)>>2;state<=STORE;end
    else begin scale<=scale+1'b1;filter_sum<=0;tap_x<=0;tap_y<=0;state<=TAP_COORDS;end
   end
   if(state==STORE)begin
    if(map_x==MW-1&&map_y==MH-1)begin pending<=1;state<=PENDING;end
    else begin
     if(map_x==MW-1)begin map_x<=0;map_y<=map_y+1'b1;end else map_x<=map_x+1'b1;
     scale<=0;filter_sum<=0;fusion<=0;tap_x<=0;tap_y<=0;state<=TAP_COORDS;
    end
   end
  end
 end
endmodule
