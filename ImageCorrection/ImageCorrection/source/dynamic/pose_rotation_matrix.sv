// R_source_from_reference = R_current^T R_reference, WIT Euler Z-Y-X.
// Six continuous-angle rotations; no angle bins or subtracted Euler shortcut.
// Camera axis permutation/signs describe the rigid IMU-to-camera mounting.
module pose_rotation_matrix #(
 parameter X_AXIS=0,Y_AXIS=1,Z_AXIS=2,X_SIGN=1,Y_SIGN=-1,Z_SIGN=1
)(
 input wire clk,rst,start,pose_fresh,
 input wire [15:0] roll,pitch,yaw,
 output wire ready,output reg busy,reference_valid,identity,
 output wire [287:0] matrix
);
 localparam IDLE=0,ANGLE=1,TRIG=2,SELECT=3,MULT=4,ADD=5,COMMIT=6,FINISH=7,WAIT_PRODUCT=8;
 reg [3:0] state;
 reg [15:0] ref_roll,ref_pitch,ref_yaw,cur_roll,cur_pitch,cur_yaw;
 reg signed [31:0] m[0:8];
 reg signed [31:0] result[0:8];
 reg [2:0] rotation;reg [1:0] col;
 reg [3:0] ia,ib;
 reg signed [31:0] a,b,s,c;
 wire signed [63:0] ac,bs,as_,bc;reg wait_product;
 dynamic_mul32 mul_ac(.clk(clk),.a(a),.b(c),.product(ac));
 dynamic_mul32 mul_bs(.clk(clk),.a(b),.b(s),.product(bs));
 dynamic_mul32 mul_as(.clk(clk),.a(a),.b(s),.product(as_));
 dynamic_mul32 mul_bc(.clk(clk),.a(b),.b(c),.product(bc));
 reg signed [64:0] sum_a,sum_b;
 reg [15:0] angle;
 reg trig_start;wire trig_ready,trig_valid;
 wire signed [31:0] trig_c,trig_s;
 wire [15:0] selected_angle=rotation==0?ref_roll:rotation==1?ref_pitch:rotation==2?ref_yaw:
                                  rotation==3?-cur_yaw:rotation==4?-cur_pitch:-cur_roll;
 wire [1:0] axis=(rotation==0||rotation==5)?0:(rotation==1||rotation==4)?1:2;
 wire [1:0] row_a=axis==0?1:axis==1?2:0;
 wire [1:0] row_b=axis==0?2:axis==1?0:1;
 assign ready=state==IDLE;
 genvar n;
 generate for(n=0;n<9;n=n+1)begin:g_output
  assign matrix[n*32+:32]=result[n];
 end endgenerate
 function integer camera_axis(input integer i);
  camera_axis=i==0?X_AXIS:i==1?Y_AXIS:Z_AXIS;
 endfunction
 function integer camera_sign(input integer i);
  camera_sign=i==0?X_SIGN:i==1?Y_SIGN:Z_SIGN;
 endfunction
 cordic_sincos trig(.clk(clk),.rst(rst),.start(trig_start),.angle(angle),
  .ready(trig_ready),.valid(trig_valid),.cosine(trig_c),.sine(trig_s));
 integer k;
 always @(posedge clk)begin
  if(rst)begin
   state<=IDLE;busy<=0;reference_valid<=0;identity<=1;
   ref_roll<=0;ref_pitch<=0;ref_yaw<=0;cur_roll<=0;cur_pitch<=0;cur_yaw<=0;
   rotation<=0;col<=0;ia<=0;ib<=0;angle<=0;trig_start<=0;
   a<=0;b<=0;s<=0;c<=0;wait_product<=0;sum_a<=0;sum_b<=0;
   for(k=0;k<9;k=k+1)begin m[k]<=(k%4==0)?32'sd16777216:0;result[k]<=(k%4==0)?32'sd16777216:0;end
  end else begin
   trig_start<=0;
   case(state)
    IDLE:if(start)begin
     // A stale frame freezes the last transform. Never silently re-zero IMU.
     if(pose_fresh)begin
      cur_roll<=roll;cur_pitch<=pitch;cur_yaw<=yaw;
      if(!reference_valid)begin ref_roll<=roll;ref_pitch<=pitch;ref_yaw<=yaw;reference_valid<=1;identity<=1;end
      else if(roll==ref_roll&&pitch==ref_pitch&&yaw==ref_yaw)begin
       identity<=1;
       for(k=0;k<9;k=k+1)result[k]<=(k%4==0)?32'sd16777216:0;
      end else begin
       identity<=0;busy<=1;state<=ANGLE;rotation<=0;col<=0;
       for(k=0;k<9;k=k+1)m[k]<=(k%4==0)?32'sd16777216:0;
      end
     end
    end
    ANGLE:if(trig_ready)begin angle<=selected_angle;trig_start<=1;state<=TRIG;end
    TRIG:if(trig_valid)begin c<=trig_c;s<=trig_s;col<=0;state<=SELECT;end
    SELECT:begin
     ia<=row_a*3+col;ib<=row_b*3+col;
     a<=m[row_a*3+col];b<=m[row_b*3+col];state<=MULT;
    end
    MULT:begin wait_product<=0;state<=WAIT_PRODUCT;end
    WAIT_PRODUCT:if(wait_product)state<=ADD;else wait_product<=1;
    ADD:begin sum_a<=$signed(ac)-$signed(bs)+65'sd536870912;sum_b<=$signed(as_)+$signed(bc)+65'sd536870912;state<=COMMIT;end
    COMMIT:begin
     m[ia]<=sum_a>>>30;m[ib]<=sum_b>>>30;
     if(col==2)begin
      if(rotation==5)state<=FINISH;
      else begin rotation<=rotation+1'b1;state<=ANGLE;end
     end else begin col<=col+1'b1;state<=SELECT;end
    end
    FINISH:begin
     for(k=0;k<9;k=k+1)
      result[k]<=camera_sign(k/3)*camera_sign(k%3)*m[camera_axis(k/3)*3+camera_axis(k%3)];
     busy<=0;state<=IDLE;
    end
    default:state<=IDLE;
   endcase
  end
 end
endmodule

// Binary-angle CORDIC. Input is signed raw WIT angle (180deg = 32768).
// Q30 output, twenty iterations plus exact cardinal-angle cases.
module cordic_sincos(
 input wire clk,rst,start,input wire [15:0] angle,
 output wire ready,output reg valid,output reg signed [31:0] cosine,sine
);
 reg busy;reg [4:0] step;
 reg signed [31:0] x,y,z;
 assign ready=!busy;
 function [31:0] atan_step(input [4:0] i);
  case(i)
   0:atan_step=32'd536870912;1:atan_step=32'd316933406;2:atan_step=32'd167458907;
   3:atan_step=32'd85004756;4:atan_step=32'd42667331;5:atan_step=32'd21354465;
   6:atan_step=32'd10679838;7:atan_step=32'd5340245;8:atan_step=32'd2670163;
   9:atan_step=32'd1335087;10:atan_step=32'd667544;11:atan_step=32'd333772;
   12:atan_step=32'd166886;13:atan_step=32'd83443;14:atan_step=32'd41722;
   15:atan_step=32'd20861;16:atan_step=32'd10430;17:atan_step=32'd5215;
   18:atan_step=32'd2608;default:atan_step=32'd1304;
  endcase
 endfunction
 wire signed [31:0] nx=z<0?x+(y>>>step):x-(y>>>step);
 wire signed [31:0] ny=z<0?y-(x>>>step):y+(x>>>step);
 always @(posedge clk)begin
  if(rst)begin busy<=0;valid<=0;step<=0;x<=0;y<=0;z<=0;cosine<=0;sine<=0;end
  else begin
   valid<=0;
   if(start&&!busy)begin
    if(angle[13:0]==0)begin
     cosine<=angle==0?32'sd1073741824:angle==16'h8000 ? -32'sd1073741824:0;
     sine<=angle==16'h4000 ? 32'sd1073741824:angle==16'hc000 ? -32'sd1073741824:0;valid<=1;
    end else begin
     busy<=1;step<=0;y<=0;
     if($signed(angle)>16384||$signed(angle)<-16384)begin x<=-32'sd652032874;z<={angle^16'h8000,16'd0};end
     else begin x<=32'sd652032874;z<={angle,16'd0};end
    end
   end else if(busy)begin
    x<=nx;y<=ny;z<=z<0?z+$signed(atan_step(step)):z-$signed(atan_step(step));
    if(step==19)begin cosine<=nx;sine<=ny;valid<=1;busy<=0;end
    else step<=step+1'b1;
   end
  end
 end
endmodule
