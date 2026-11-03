// Independent Model A. Complete products and explicit rounding/saturation.
// Input edge t -> earliest output handshake t+23; global CE freezes all tokens.
// One explicit extra R6 multiplication stage; token metadata follows the same CE.
module model_a_coordinate_map #(
 parameter NORMALIZED_FRAC=24, WIDTH=1920, HEIGHT=1080, FRACTIONAL_INPUT=0
)(
 input wire clk,rst,in_valid,out_ready,
 output wire in_ready,out_valid,
 input wire [10:0] in_u,in_v,
 input wire signed [27:0] in_u_q,in_v_q,input wire in_invalid,
 input wire [15:0] in_frame_id,
 input wire in_sof,in_eol,in_eof,
 output wire signed [27:0] src_x_q,src_y_q,
 output reg signed [12:0] src_x_nn,src_y_nn,
 output reg source_oob,
 output wire input_oob,saturation_seen,
 output wire [10:0] out_u,out_v,
 output wire [15:0] out_frame_id,
 output wire out_sof,out_eol,out_eof
);
 localparam NW=NORMALIZED_FRAC+3, MW=NORMALIZED_FRAC+4, CW=NORMALIZED_FRAC+2;
 `include "model_a_f24.vh"
 `include "model_a_f20.vh"
 localparam signed [(28)-1:0] KI_FX = (NORMALIZED_FRAC==24) ? MODEL_A_F24_K_IN_FX : MODEL_A_F20_K_IN_FX;
 localparam signed [(28)-1:0] KI_FY = (NORMALIZED_FRAC==24) ? MODEL_A_F24_K_IN_FY : MODEL_A_F20_K_IN_FY;
 localparam signed [(28)-1:0] KI_CX = (NORMALIZED_FRAC==24) ? MODEL_A_F24_K_IN_CX : MODEL_A_F20_K_IN_CX;
 localparam signed [(28)-1:0] KI_CY = (NORMALIZED_FRAC==24) ? MODEL_A_F24_K_IN_CY : MODEL_A_F20_K_IN_CY;
 localparam signed [(28)-1:0] KO_CX = (NORMALIZED_FRAC==24) ? MODEL_A_F24_K_OUT_CX : MODEL_A_F20_K_OUT_CX;
 localparam signed [(28)-1:0] KO_CY = (NORMALIZED_FRAC==24) ? MODEL_A_F24_K_OUT_CY : MODEL_A_F20_K_OUT_CY;
 localparam signed [(33)-1:0] KO_INV_FX = (NORMALIZED_FRAC==24) ? MODEL_A_F24_K_OUT_INV_FX : MODEL_A_F20_K_OUT_INV_FX;
 localparam signed [(33)-1:0] KO_INV_FY = (NORMALIZED_FRAC==24) ? MODEL_A_F24_K_OUT_INV_FY : MODEL_A_F20_K_OUT_INV_FY;
 localparam signed [(CW)-1:0] K1 = (NORMALIZED_FRAC==24) ? MODEL_A_F24_K1 : MODEL_A_F20_K1;
 localparam signed [(CW)-1:0] K2 = (NORMALIZED_FRAC==24) ? MODEL_A_F24_K2 : MODEL_A_F20_K2;
 localparam signed [(CW)-1:0] K3 = (NORMALIZED_FRAC==24) ? MODEL_A_F24_K3 : MODEL_A_F20_K3;
 localparam signed [(CW)-1:0] P1 = (NORMALIZED_FRAC==24) ? MODEL_A_F24_P1 : MODEL_A_F20_P1;
 localparam signed [(CW)-1:0] P2 = (NORMALIZED_FRAC==24) ? MODEL_A_F24_P2 : MODEL_A_F20_P2;
 localparam signed [MW-1:0] ONE_Q = 28'sd1 <<< NORMALIZED_FRAC;
 generate if(NORMALIZED_FRAC!=24 && NORMALIZED_FRAC!=20) begin : illegal_format
   initial $fatal(1,"NORMALIZED_FRAC must be20 or24");
 end endgenerate
 localparam PIPELINE_LATENCY=23;
 reg [22:0] valid_pipe=0,sat_pipe=0;
 reg [41:0] metadata[0:22];
 wire advance=!out_valid || out_ready;
 assign in_ready=advance;
 assign out_valid=valid_pipe[22];
 assign saturation_seen=sat_pipe[22];
 assign input_oob=metadata[22][0];
 assign out_eof=metadata[22][1];
 assign out_eol=metadata[22][2];
 assign out_sof=metadata[22][3];
 assign out_v=metadata[22][14:4];
 assign out_u=metadata[22][25:15];
 assign out_frame_id=metadata[22][41:26];
 integer m;
 reg signed [(28)-1:0] du[1:1];
 wire signed [(28)-1:0] du_rounded;
 wire du_sat;
 wire signed [(29)-1:0] du_sum = (FRACTIONAL_INPUT?$signed(in_u_q):$signed({1'b0,in_u,16'b0})) - $signed({KO_CX[27],KO_CX});
 fixed_round_sat #(.IN_WIDTH(29),.OUT_WIDTH(28),.SHIFT(0)) round_du
   (.value(du_sum),.result(du_rounded),.sat(du_sat));
 reg signed [(28)-1:0] dv[1:1];
 wire signed [(28)-1:0] dv_rounded;
 wire dv_sat;
 wire signed [(29)-1:0] dv_sum = (FRACTIONAL_INPUT?$signed(in_v_q):$signed({1'b0,in_v,16'b0})) - $signed({KO_CY[27],KO_CY});
 fixed_round_sat #(.IN_WIDTH(29),.OUT_WIDTH(28),.SHIFT(0)) round_dv
   (.value(dv_sum),.result(dv_rounded),.sat(dv_sat));
 reg signed [(NW)-1:0] x[3:14];
 wire signed [(NW)-1:0] x_rounded;
 wire x_sat;
 reg signed [(61)-1:0] x_product;
 (* syn_preserve=1, syn_keep=1 *) reg signed [60:0] x_product_isolated;
 (* syn_preserve=1, syn_keep=1 *) reg signed [NW-1:0] x_round_isolated;
 fixed_round_sat #(.IN_WIDTH(61),.OUT_WIDTH(NW),.SHIFT(48-NORMALIZED_FRAC)) round_x
   (.value(x_product_isolated),.result(x_rounded),.sat(x_sat));
 reg signed [(NW)-1:0] y[3:14];
 wire signed [(NW)-1:0] y_rounded;
 wire y_sat;
 reg signed [(61)-1:0] y_product;
 (* syn_preserve=1, syn_keep=1 *) reg signed [60:0] y_product_isolated;
 (* syn_preserve=1, syn_keep=1 *) reg signed [NW-1:0] y_round_isolated;
 fixed_round_sat #(.IN_WIDTH(61),.OUT_WIDTH(NW),.SHIFT(48-NORMALIZED_FRAC)) round_y
   (.value(y_product_isolated),.result(y_rounded),.sat(y_sat));
 reg signed [(MW)-1:0] xx[5:6];
 wire signed [(MW)-1:0] xx_rounded;
 wire xx_sat;
 reg signed [(2*NW)-1:0] xx_product;
 fixed_round_sat #(.IN_WIDTH(2*NW),.OUT_WIDTH(MW),.SHIFT(NORMALIZED_FRAC)) round_xx
   (.value(xx_product),.result(xx_rounded),.sat(xx_sat));
 reg signed [(MW)-1:0] yy[5:6];
 wire signed [(MW)-1:0] yy_rounded;
 wire yy_sat;
 reg signed [(2*NW)-1:0] yy_product;
 fixed_round_sat #(.IN_WIDTH(2*NW),.OUT_WIDTH(MW),.SHIFT(NORMALIZED_FRAC)) round_yy
   (.value(yy_product),.result(yy_rounded),.sat(yy_sat));
 reg signed [(MW)-1:0] xy[5:6];
 wire signed [(MW)-1:0] xy_rounded;
 wire xy_sat;
 reg signed [(2*NW)-1:0] xy_product;

 // Exact signed XY retiming across the existing x[3]/y[3] boundary.
 // Low limbs are unsigned; high limbs retain the sign. Full precision only.
 localparam XYL=NW/2, XYH=NW-XYL, XYP=2*NW;
 wire signed [XYL:0] xy_xlo=$signed({1'b0,x_round_isolated[XYL-1:0]});
 wire signed [XYL:0] xy_ylo=$signed({1'b0,y_round_isolated[XYL-1:0]});
 wire signed [XYH-1:0] xy_xhi=$signed(x_round_isolated[NW-1:XYL]);
 wire signed [XYH-1:0] xy_yhi=$signed(y_round_isolated[NW-1:XYL]);
 (* syn_preserve=1 *) reg [2*XYL-1:0] xy_ll;
 (* syn_preserve=1 *) reg signed [NW:0] xy_lh,xy_hl;
 (* syn_preserve=1 *) reg signed [2*XYH-1:0] xy_hh;
 // LL and shifted HH do not overlap, so concatenation is their exact sum.
 wire [XYP-1:0] xy_a={xy_hh,xy_ll};
 wire [XYP-1:0] xy_b=$signed({{(XYP-NW-1){xy_lh[NW]}},xy_lh}) << XYL;
 wire [XYP-1:0] xy_c=$signed({{(XYP-NW-1){xy_hl[NW]}},xy_hl}) << XYL;
 wire [XYP-1:0] xy_sum=xy_a ^ xy_b ^ xy_c;
 wire [XYP-1:0] xy_carry=((xy_a & xy_b)|(xy_a & xy_c)|(xy_b & xy_c)) << 1;

 fixed_round_sat #(.IN_WIDTH(2*NW),.OUT_WIDTH(MW),.SHIFT(NORMALIZED_FRAC)) round_xy
   (.value(xy_product),.result(xy_rounded),.sat(xy_sat));
 reg signed [(MW)-1:0] r2[6:8];
 wire signed [(MW)-1:0] r2_rounded;
 wire r2_sat;
 wire signed [(MW+1)-1:0] r2_sum = $signed({{((MW+1)-(MW)){xx[5][(MW)-1]}},xx[5]}) + $signed({{((MW+1)-(MW)){yy[5][(MW)-1]}},yy[5]});
 fixed_round_sat #(.IN_WIDTH(MW+1),.OUT_WIDTH(MW),.SHIFT(0)) round_r2
   (.value(r2_sum),.result(r2_rounded),.sat(r2_sat));
 (* syn_preserve=1 *) reg signed [(MW)-1:0] r4[8:8];
 wire signed [(MW)-1:0] r4_rounded;
 wire r4_sat;
 reg signed [(2*MW)-1:0] r4_product;
 fixed_round_sat #(.IN_WIDTH(2*MW),.OUT_WIDTH(MW),.SHIFT(NORMALIZED_FRAC)) round_r4
   (.value(r4_product),.result(r4_rounded),.sat(r4_sat));
 reg signed [(MW)-1:0] p1xy[8:16];
 wire signed [(MW)-1:0] p1xy_rounded;
 wire p1xy_sat;
 reg signed [(CW+MW)-1:0] p1xy_product;
 fixed_round_sat #(.IN_WIDTH(CW+MW),.OUT_WIDTH(MW),.SHIFT(NORMALIZED_FRAC)) round_p1xy
   (.value(p1xy_product),.result(p1xy_rounded),.sat(p1xy_sat));
 reg signed [(MW)-1:0] p2xy[8:16];
 wire signed [(MW)-1:0] p2xy_rounded;
 wire p2xy_sat;
 reg signed [(CW+MW)-1:0] p2xy_product;
 fixed_round_sat #(.IN_WIDTH(CW+MW),.OUT_WIDTH(MW),.SHIFT(NORMALIZED_FRAC)) round_p2xy
   (.value(p2xy_product),.result(p2xy_rounded),.sat(p2xy_sat));
 reg signed [(MW)-1:0] tx[7:7];
 wire signed [(MW)-1:0] tx_rounded;
 wire tx_sat;
 wire signed [(MW+2)-1:0] tx_sum = $signed({{((MW+2)-(MW)){r2[6][(MW)-1]}},r2[6]}) + ($signed({{((MW+2)-(MW)){xx[6][(MW)-1]}},xx[6]}) <<< 1);
 fixed_round_sat #(.IN_WIDTH(MW+2),.OUT_WIDTH(MW),.SHIFT(0)) round_tx
   (.value(tx_sum),.result(tx_rounded),.sat(tx_sat));
 reg signed [(MW)-1:0] ty[7:7];
 wire signed [(MW)-1:0] ty_rounded;
 wire ty_sat;
 wire signed [(MW+2)-1:0] ty_sum = $signed({{((MW+2)-(MW)){r2[6][(MW)-1]}},r2[6]}) + ($signed({{((MW+2)-(MW)){yy[6][(MW)-1]}},yy[6]}) <<< 1);
 fixed_round_sat #(.IN_WIDTH(MW+2),.OUT_WIDTH(MW),.SHIFT(0)) round_ty
   (.value(ty_sum),.result(ty_rounded),.sat(ty_sat));
 reg signed [(MW)-1:0] p2tx[9:16];
 wire signed [(MW)-1:0] p2tx_rounded;
 wire p2tx_sat;
 reg signed [(CW+MW)-1:0] p2tx_product;
 fixed_round_sat #(.IN_WIDTH(CW+MW),.OUT_WIDTH(MW),.SHIFT(NORMALIZED_FRAC)) round_p2tx
   (.value(p2tx_product),.result(p2tx_rounded),.sat(p2tx_sat));
 reg signed [(MW)-1:0] p1ty[9:16];
 wire signed [(MW)-1:0] p1ty_rounded;
 wire p1ty_sat;
 reg signed [(CW+MW)-1:0] p1ty_product;
 fixed_round_sat #(.IN_WIDTH(CW+MW),.OUT_WIDTH(MW),.SHIFT(NORMALIZED_FRAC)) round_p1ty
   (.value(p1ty_product),.result(p1ty_rounded),.sat(p1ty_sat));
 reg signed [(MW)-1:0] r6[11:11];
 wire signed [(MW)-1:0] r6_rounded;
 wire r6_sat;
 reg signed [(2*MW)-1:0] r6_product;
 // Exact R4*R2: rounded operands are registered at S8, parallel small
 // products at S9, carry-save reduction plus final sum at S10, round S11.
 // The extra stage separates rounding from multiplication. Parallel radial,
 // tangential, coordinate and token paths are delayed by the same one cycle.
 // All payload registers and metadata share advance/reset.
 localparam R6L=MW/2, R6H=MW-R6L, R6P=2*MW;
 wire signed [R6L:0] r6_alo=$signed({1'b0,r4[8][R6L-1:0]});
 wire signed [R6L:0] r6_blo=$signed({1'b0,r2[8][R6L-1:0]});
 wire signed [R6H-1:0] r6_ahi=$signed(r4[8][MW-1:R6L]);
 wire signed [R6H-1:0] r6_bhi=$signed(r2[8][MW-1:R6L]);
 (* syn_preserve=1 *) reg [2*R6L-1:0] r6_ll;
 (* syn_preserve=1 *) reg signed [MW:0] r6_lh,r6_hl;
 (* syn_preserve=1 *) reg signed [2*R6H-1:0] r6_hh;
 wire [R6P-1:0] r6_a={r6_hh,r6_ll};
 wire [R6P-1:0] r6_b=$signed({{(R6P-MW-1){r6_lh[MW]}},r6_lh}) << R6L;
 wire [R6P-1:0] r6_c=$signed({{(R6P-MW-1){r6_hl[MW]}},r6_hl}) << R6L;
 wire [R6P-1:0] r6_sum=r6_a ^ r6_b ^ r6_c;
 wire [R6P-1:0] r6_carry=((r6_a&r6_b)|(r6_a&r6_c)|(r6_b&r6_c)) << 1;
 fixed_round_sat #(.IN_WIDTH(2*MW),.OUT_WIDTH(MW),.SHIFT(NORMALIZED_FRAC)) round_r6
   (.value(r6_product),.result(r6_rounded),.sat(r6_sat));
 reg signed [(MW)-1:0] k1term[10:13];
 wire signed [(MW)-1:0] k1term_rounded;
 wire k1term_sat;
 reg signed [(CW+MW)-1:0] k1term_product;
 fixed_round_sat #(.IN_WIDTH(CW+MW),.OUT_WIDTH(MW),.SHIFT(NORMALIZED_FRAC)) round_k1term
   (.value(k1term_product),.result(k1term_rounded),.sat(k1term_sat));
 reg signed [(MW)-1:0] k2term[10:13];
 wire signed [(MW)-1:0] k2term_rounded;
 wire k2term_sat;
 reg signed [(CW+MW)-1:0] k2term_product;
 fixed_round_sat #(.IN_WIDTH(CW+MW),.OUT_WIDTH(MW),.SHIFT(NORMALIZED_FRAC)) round_k2term
   (.value(k2term_product),.result(k2term_rounded),.sat(k2term_sat));
 reg signed [(MW)-1:0] k3term[13:13];
 wire signed [(MW)-1:0] k3term_rounded;
 wire k3term_sat;
 reg signed [(CW+MW)-1:0] k3term_product;
 fixed_round_sat #(.IN_WIDTH(CW+MW),.OUT_WIDTH(MW),.SHIFT(NORMALIZED_FRAC)) round_k3term
   (.value(k3term_product),.result(k3term_rounded),.sat(k3term_sat));
 reg signed [(MW)-1:0] radial[14:14];
 wire signed [(MW)-1:0] radial_rounded;
 wire radial_sat;
 wire signed [(MW+2)-1:0] radial_sum = $signed({{2{ONE_Q[MW-1]}},ONE_Q}) + $signed({{((MW+2)-(MW)){k1term[13][(MW)-1]}},k1term[13]}) + $signed({{((MW+2)-(MW)){k2term[13][(MW)-1]}},k2term[13]}) + $signed({{((MW+2)-(MW)){k3term[13][(MW)-1]}},k3term[13]});
 fixed_round_sat #(.IN_WIDTH(MW+2),.OUT_WIDTH(MW),.SHIFT(0)) round_radial
   (.value(radial_sum),.result(radial_rounded),.sat(radial_sat));
 reg signed [(MW)-1:0] xr[16:16];
 wire signed [(MW)-1:0] xr_rounded;
 wire xr_sat;
 reg signed [(NW+MW)-1:0] xr_product;
 fixed_round_sat #(.IN_WIDTH(NW+MW),.OUT_WIDTH(MW),.SHIFT(NORMALIZED_FRAC)) round_xr
   (.value(xr_product),.result(xr_rounded),.sat(xr_sat));
 reg signed [(MW)-1:0] yr[16:16];
 wire signed [(MW)-1:0] yr_rounded;
 wire yr_sat;
 reg signed [(NW+MW)-1:0] yr_product;
 fixed_round_sat #(.IN_WIDTH(NW+MW),.OUT_WIDTH(MW),.SHIFT(NORMALIZED_FRAC)) round_yr
   (.value(yr_product),.result(yr_rounded),.sat(yr_sat));
 reg signed [(NW)-1:0] xd[17:17];
 wire signed [(NW)-1:0] xd_rounded;
 wire xd_sat;
 wire signed [(MW+2)-1:0] xd_sum = $signed({{((MW+2)-(MW)){xr[16][(MW)-1]}},xr[16]}) + ($signed({{((MW+2)-(MW)){p1xy[16][(MW)-1]}},p1xy[16]}) <<< 1) + $signed({{((MW+2)-(MW)){p2tx[16][(MW)-1]}},p2tx[16]});
 fixed_round_sat #(.IN_WIDTH(MW+2),.OUT_WIDTH(NW),.SHIFT(0)) round_xd
   (.value(xd_sum),.result(xd_rounded),.sat(xd_sat));
 reg signed [(NW)-1:0] yd[17:17];
 wire signed [(NW)-1:0] yd_rounded;
 wire yd_sat;
 wire signed [(MW+2)-1:0] yd_sum = $signed({{((MW+2)-(MW)){yr[16][(MW)-1]}},yr[16]}) + $signed({{((MW+2)-(MW)){p1ty[16][(MW)-1]}},p1ty[16]}) + ($signed({{((MW+2)-(MW)){p2xy[16][(MW)-1]}},p2xy[16]}) <<< 1);
 fixed_round_sat #(.IN_WIDTH(MW+2),.OUT_WIDTH(NW),.SHIFT(0)) round_yd
   (.value(yd_sum),.result(yd_rounded),.sat(yd_sat));
 reg signed [(28)-1:0] fxd[19:19];
 wire signed [(28)-1:0] fxd_rounded;
 wire fxd_sat;
 reg signed [(28+NW)-1:0] fxd_product;
 fixed_round_sat #(.IN_WIDTH(28+NW),.OUT_WIDTH(28),.SHIFT(NORMALIZED_FRAC)) round_fxd
   (.value(fxd_product),.result(fxd_rounded),.sat(fxd_sat));
 reg signed [(28)-1:0] fyd[19:19];
 wire signed [(28)-1:0] fyd_rounded;
 wire fyd_sat;
 reg signed [(28+NW)-1:0] fyd_product;
 fixed_round_sat #(.IN_WIDTH(28+NW),.OUT_WIDTH(28),.SHIFT(NORMALIZED_FRAC)) round_fyd
   (.value(fyd_product),.result(fyd_rounded),.sat(fyd_sat));
 reg signed [(28)-1:0] sx[20:21];
 wire signed [(28)-1:0] sx_rounded;
 wire sx_sat;
 wire signed [(29)-1:0] sx_sum = $signed({{((29)-(28)){fxd[19][(28)-1]}},fxd[19]}) + $signed({KI_CX[27],KI_CX});
 fixed_round_sat #(.IN_WIDTH(29),.OUT_WIDTH(28),.SHIFT(0)) round_sx
   (.value(sx_sum),.result(sx_rounded),.sat(sx_sat));
 reg signed [(28)-1:0] sy[20:21];
 wire signed [(28)-1:0] sy_rounded;
 wire sy_sat;
 wire signed [(29)-1:0] sy_sum = $signed({{((29)-(28)){fyd[19][(28)-1]}},fyd[19]}) + $signed({KI_CY[27],KI_CY});
 fixed_round_sat #(.IN_WIDTH(29),.OUT_WIDTH(28),.SHIFT(0)) round_sy
   (.value(sy_sum),.result(sy_rounded),.sat(sy_sat));
 assign src_x_q=sx[21];
 assign src_y_q=sy[21];
 wire signed [28:0] nnx_value=($signed({sx[20][27],sx[20]})+29'sd32768) >>> 16;
 wire signed [28:0] nny_value=($signed({sy[20][27],sy[20]})+29'sd32768) >>> 16;
 always @(posedge clk) begin
   if(rst) begin
     valid_pipe<=0; sat_pipe<=0; src_x_nn<=0; src_y_nn<=0; source_oob<=0;
     for(m=0;m<PIPELINE_LATENCY;m=m+1) metadata[m]<=0;
     x_product_isolated<=0;y_product_isolated<=0;
     x_round_isolated<=0;y_round_isolated<=0;
     du[1]<=0;
     dv[1]<=0;
     x[3]<=0;
     x[4]<=0;
     x[5]<=0;
     x[6]<=0;
     x[7]<=0;
     x[8]<=0;
     x[9]<=0;
     x[10]<=0;
     x[11]<=0;
     x[12]<=0;
     x[13]<=0;
     x[14]<=0;
     x_product<=0;
     y[3]<=0;
     y[4]<=0;
     y[5]<=0;
     y[6]<=0;
     y[7]<=0;
     y[8]<=0;
     y[9]<=0;
     y[10]<=0;
     y[11]<=0;
     y[12]<=0;
     y[13]<=0;
     y[14]<=0;
     y_product<=0;
     xx[5]<=0;
     xx[6]<=0;
     xx_product<=0;
     yy[5]<=0;
     yy[6]<=0;
     yy_product<=0;
     xy[5]<=0;
     xy[6]<=0;
     xy_product<=0;
     xy_ll<=0;xy_lh<=0;xy_hl<=0;xy_hh<=0;
     r2[6]<=0;
     r2[7]<=0;
     r2[8]<=0;
     r4[8]<=0;
     r4_product<=0;
     p1xy[8]<=0;
     p1xy[9]<=0;
     p1xy[10]<=0;
     p1xy[11]<=0;
     p1xy[12]<=0;
     p1xy[13]<=0;
     p1xy[14]<=0;
     p1xy[15]<=0;
     p1xy[16]<=0;
     p1xy_product<=0;
     p2xy[8]<=0;
     p2xy[9]<=0;
     p2xy[10]<=0;
     p2xy[11]<=0;
     p2xy[12]<=0;
     p2xy[13]<=0;
     p2xy[14]<=0;
     p2xy[15]<=0;
     p2xy[16]<=0;
     p2xy_product<=0;
     tx[7]<=0;
     ty[7]<=0;
     p2tx[9]<=0;
     p2tx[10]<=0;
     p2tx[11]<=0;
     p2tx[12]<=0;
     p2tx[13]<=0;
     p2tx[14]<=0;
     p2tx[15]<=0;
     p2tx[16]<=0;
     p2tx_product<=0;
     p1ty[9]<=0;
     p1ty[10]<=0;
     p1ty[11]<=0;
     p1ty[12]<=0;
     p1ty[13]<=0;
     p1ty[14]<=0;
     p1ty[15]<=0;
     p1ty[16]<=0;
     p1ty_product<=0;
     r6[11]<=0;
     r6_product<=0;
     r6_ll<=0;r6_lh<=0;r6_hl<=0;r6_hh<=0;
     k1term[10]<=0;
     k1term[11]<=0;
     k1term[12]<=0;
     k1term[13]<=0;
     k1term_product<=0;
     k2term[10]<=0;
     k2term[11]<=0;
     k2term[12]<=0;
     k2term[13]<=0;
     k2term_product<=0;
     k3term[13]<=0;
     k3term_product<=0;
     radial[14]<=0;
     xr[16]<=0;
     xr_product<=0;
     yr[16]<=0;
     yr_product<=0;
     xd[17]<=0;
     yd[17]<=0;
     fxd[19]<=0;
     fxd_product<=0;
     fyd[19]<=0;
     fyd_product<=0;
     sx[20]<=0;
     sx[21]<=0;
     sy[20]<=0;
     sy[21]<=0;
   end else if(advance) begin
     valid_pipe <= {valid_pipe[21:0],in_valid};
     metadata[0] <= {in_frame_id,in_u,in_v,in_sof,in_eol,in_eof,(FRACTIONAL_INPUT?in_invalid:((in_u>=WIDTH)||(in_v>=HEIGHT)))};
     for(m=1;m<PIPELINE_LATENCY;m=m+1) metadata[m]<=metadata[m-1];
     sat_pipe[0]<=in_valid ? (du_sat | dv_sat) : 1'b0;
     sat_pipe[1]<=valid_pipe[0] ? (sat_pipe[0] | 1'b0) : 1'b0;
     sat_pipe[2]<=valid_pipe[1] ? sat_pipe[1] : 1'b0;
     sat_pipe[3]<=valid_pipe[2] ? (sat_pipe[2] | x_sat | y_sat) : 1'b0;
     sat_pipe[4]<=valid_pipe[3] ? sat_pipe[3] : 1'b0;
     sat_pipe[5]<=valid_pipe[4] ? sat_pipe[4] : 1'b0;
     sat_pipe[6]<=valid_pipe[5] ? (sat_pipe[5] | xx_sat | yy_sat | xy_sat) : 1'b0;
     sat_pipe[7]<=valid_pipe[6] ? (sat_pipe[6] | r2_sat) : 1'b0;
     sat_pipe[8]<=valid_pipe[7] ? (sat_pipe[7] | tx_sat | ty_sat) : 1'b0;
     sat_pipe[9]<=valid_pipe[8] ? (sat_pipe[8] | r4_sat | p1xy_sat | p2xy_sat) : 1'b0;
     sat_pipe[10]<=valid_pipe[9] ? (sat_pipe[9] | p2tx_sat | p1ty_sat) : 1'b0;
     sat_pipe[11]<=valid_pipe[10] ? (sat_pipe[10] | k1term_sat | k2term_sat) : 1'b0;
     sat_pipe[12]<=valid_pipe[11] ? (sat_pipe[11] | r6_sat) : 1'b0;
     sat_pipe[13]<=valid_pipe[12] ? sat_pipe[12] : 1'b0;
     sat_pipe[14]<=valid_pipe[13] ? (sat_pipe[13] | k3term_sat) : 1'b0;
     sat_pipe[15]<=valid_pipe[14] ? (sat_pipe[14] | radial_sat) : 1'b0;
     sat_pipe[16]<=valid_pipe[15] ? sat_pipe[15] : 1'b0;
     sat_pipe[17]<=valid_pipe[16] ? (sat_pipe[16] | xr_sat | yr_sat) : 1'b0;
     sat_pipe[18]<=valid_pipe[17] ? (sat_pipe[17] | xd_sat | yd_sat) : 1'b0;
     sat_pipe[19]<=valid_pipe[18] ? sat_pipe[18] : 1'b0;
     sat_pipe[20]<=valid_pipe[19] ? (sat_pipe[19] | fxd_sat | fyd_sat) : 1'b0;
     sat_pipe[21]<=valid_pipe[20] ? (sat_pipe[20] | sx_sat | sy_sat) : 1'b0;
     sat_pipe[22]<=valid_pipe[21] ? sat_pipe[21] : 1'b0;
     du[1]<=du_rounded; // rounded/saturated S1
     dv[1]<=dv_rounded; // rounded/saturated S1
     x_product<=du[1] * KO_INV_FX; // complete product at S2
     x_product_isolated<=x_product; // new registered product isolation
     x_round_isolated<=x_rounded; // exact old round/saturation, registered
     x[3]<=x_round_isolated; // logical oldS3 now atS5
     x[4]<=x[3];
     x[5]<=x[4];
     x[6]<=x[5];
     x[7]<=x[6];
     x[8]<=x[7];
     x[9]<=x[8];
     x[10]<=x[9];
     x[11]<=x[10];
     x[12]<=x[11];
     x[13]<=x[12];
     x[14]<=x[13];
     y_product<=dv[1] * KO_INV_FY; // complete product at S2
     y_product_isolated<=y_product;
     y_round_isolated<=y_rounded;
     y[3]<=y_round_isolated; // symmetricY, same new latency
     y[4]<=y[3];
     y[5]<=y[4];
     y[6]<=y[5];
     y[7]<=y[6];
     y[8]<=y[7];
     y[9]<=y[8];
     y[10]<=y[9];
     y[11]<=y[10];
     y[12]<=y[11];
     y[13]<=y[12];
     y[14]<=y[13];
     xx_product<=x[3] * x[3]; // complete product at S4
     xx[5]<=xx_rounded; // rounded/saturated S5
     xx[6]<=xx[5];
     yy_product<=y[3] * y[3]; // complete product at S4
     yy[5]<=yy_rounded; // rounded/saturated S5
     yy[6]<=yy[5];
     xy_ll<=xy_xlo * xy_ylo;
     xy_lh<=xy_xlo * xy_yhi;
     xy_hl<=xy_xhi * xy_ylo;
     xy_hh<=xy_xhi * xy_yhi;
     xy_product<=xy_sum + xy_carry; // same complete product and cycle as S4
     xy[5]<=xy_rounded; // rounded/saturated S5
     xy[6]<=xy[5];
     r2[6]<=r2_rounded; // rounded/saturated S6
     r2[7]<=r2[6];
     r2[8]<=r2[7];
     r4_product<=r2[6] * r2[6]; // complete product at S7
     r4[8]<=r4_rounded; // rounded/saturated S8
     p1xy_product<=P1 * xy[6]; // complete product at S7
     p1xy[8]<=p1xy_rounded; // rounded/saturated S8
     p1xy[9]<=p1xy[8];
     p1xy[10]<=p1xy[9];
     p1xy[11]<=p1xy[10];
     p1xy[12]<=p1xy[11];
     p1xy[13]<=p1xy[12];
     p1xy[14]<=p1xy[13];
     p1xy[15]<=p1xy[14];
     p1xy[16]<=p1xy[15];
     p2xy_product<=P2 * xy[6]; // complete product at S7
     p2xy[8]<=p2xy_rounded; // rounded/saturated S8
     p2xy[9]<=p2xy[8];
     p2xy[10]<=p2xy[9];
     p2xy[11]<=p2xy[10];
     p2xy[12]<=p2xy[11];
     p2xy[13]<=p2xy[12];
     p2xy[14]<=p2xy[13];
     p2xy[15]<=p2xy[14];
     p2xy[16]<=p2xy[15];
     tx[7]<=tx_rounded; // rounded/saturated S7
     ty[7]<=ty_rounded; // rounded/saturated S7
     p2tx_product<=P2 * tx[7]; // complete product at S8
     p2tx[9]<=p2tx_rounded; // rounded/saturated S9
     p2tx[10]<=p2tx[9];
     p2tx[11]<=p2tx[10];
     p2tx[12]<=p2tx[11];
     p2tx[13]<=p2tx[12];
     p2tx[14]<=p2tx[13];
     p2tx[15]<=p2tx[14];
     p2tx[16]<=p2tx[15];
     p1ty_product<=P1 * ty[7]; // complete product at S8
     p1ty[9]<=p1ty_rounded; // rounded/saturated S9
     p1ty[10]<=p1ty[9];
     p1ty[11]<=p1ty[10];
     p1ty[12]<=p1ty[11];
     p1ty[13]<=p1ty[12];
     p1ty[14]<=p1ty[13];
     p1ty[15]<=p1ty[14];
     p1ty[16]<=p1ty[15];
     r6_ll<=r6_alo*r6_blo;
     r6_lh<=r6_alo*r6_bhi;
     r6_hl<=r6_ahi*r6_blo;
     r6_hh<=r6_ahi*r6_bhi;
     r6_product<=r6_sum+r6_carry; // complete product at S10
     r6[11]<=r6_rounded; // rounded/saturated S10
     k1term_product<=K1 * r2[8]; // complete product at S9
     k1term[10]<=k1term_rounded; // rounded/saturated S10
     k1term[11]<=k1term[10];
     k1term[12]<=k1term[11];
     k1term[13]<=k1term[12];
     k2term_product<=K2 * r4[8]; // complete product at S9
     k2term[10]<=k2term_rounded; // rounded/saturated S10
     k2term[11]<=k2term[10];
     k2term[12]<=k2term[11];
     k2term[13]<=k2term[12];
     k3term_product<=K3 * r6[11]; // complete product at S11
     k3term[13]<=k3term_rounded; // rounded/saturated S12
     radial[14]<=radial_rounded; // rounded/saturated S13
     xr_product<=x[14] * radial[14]; // complete product at S14
     xr[16]<=xr_rounded; // rounded/saturated S15
     yr_product<=y[14] * radial[14]; // complete product at S14
     yr[16]<=yr_rounded; // rounded/saturated S15
     xd[17]<=xd_rounded; // rounded/saturated S16
     yd[17]<=yd_rounded; // rounded/saturated S16
     fxd_product<=KI_FX * xd[17]; // complete product at S17
     fxd[19]<=fxd_rounded; // rounded/saturated S18
     fyd_product<=KI_FY * yd[17]; // complete product at S17
     fyd[19]<=fyd_rounded; // rounded/saturated S18
     sx[20]<=sx_rounded; // rounded/saturated S19
     sx[21]<=sx[20];
     sy[20]<=sy_rounded; // rounded/saturated S19
     sy[21]<=sy[20];
     src_x_nn<=nnx_value[12:0]; src_y_nn<=nny_value[12:0];
     source_oob<=(nnx_value<0)||(nnx_value>=WIDTH)||(nny_value<0)||(nny_value>=HEIGHT);
   end
 end
endmodule
