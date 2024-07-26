/* Generated by Yosys 0.33 (git sha1 2584903a060) */

module \$paramod\register_bank\BITS=s32'00000000000000000000000000100000 (d, reset, enable, q, clk);
  wire _000_;
  wire _001_;
  wire _002_;
  wire _003_;
  wire _004_;
  wire _005_;
  wire _006_;
  wire _007_;
  wire _008_;
  wire _009_;
  wire _010_;
  wire _011_;
  wire _012_;
  wire _013_;
  wire _014_;
  wire _015_;
  wire _016_;
  wire _017_;
  wire _018_;
  wire _019_;
  wire _020_;
  wire _021_;
  wire _022_;
  wire _023_;
  wire _024_;
  wire _025_;
  wire _026_;
  wire _027_;
  wire _028_;
  wire _029_;
  wire _030_;
  wire _031_;
  wire _032_;
  wire _033_;
  wire _034_;
  wire _035_;
  wire _036_;
  wire _037_;
  wire _038_;
  wire _039_;
  wire _040_;
  wire _041_;
  wire _042_;
  wire _043_;
  wire _044_;
  wire _045_;
  wire _046_;
  wire _047_;
  wire _048_;
  wire _049_;
  wire _050_;
  wire _051_;
  wire _052_;
  wire _053_;
  wire _054_;
  wire _055_;
  wire _056_;
  wire _057_;
  wire _058_;
  wire _059_;
  wire _060_;
  wire _061_;
  wire _062_;
  wire _063_;
  input clk;
  wire clk;
  input [31:0] d;
  wire [31:0] d;
  input enable;
  wire enable;
  output [31:0] q;
  wire [31:0] q;
  input reset;
  wire reset;
  sky130_fd_sc_hd__clkinv_1 _064_ (
    .A(reset),
    .Y(_000_)
  );
  sky130_fd_sc_hd__mux2_1 _065_ (
    .A0(q[31]),
    .A1(d[31]),
    .S(enable),
    .X(_063_)
  );
  sky130_fd_sc_hd__mux2_1 _066_ (
    .A0(q[30]),
    .A1(d[30]),
    .S(enable),
    .X(_062_)
  );
  sky130_fd_sc_hd__mux2_1 _067_ (
    .A0(q[29]),
    .A1(d[29]),
    .S(enable),
    .X(_061_)
  );
  sky130_fd_sc_hd__mux2_1 _068_ (
    .A0(q[28]),
    .A1(d[28]),
    .S(enable),
    .X(_060_)
  );
  sky130_fd_sc_hd__mux2_1 _069_ (
    .A0(q[27]),
    .A1(d[27]),
    .S(enable),
    .X(_059_)
  );
  sky130_fd_sc_hd__mux2_1 _070_ (
    .A0(q[26]),
    .A1(d[26]),
    .S(enable),
    .X(_058_)
  );
  sky130_fd_sc_hd__mux2_1 _071_ (
    .A0(q[25]),
    .A1(d[25]),
    .S(enable),
    .X(_057_)
  );
  sky130_fd_sc_hd__mux2_1 _072_ (
    .A0(q[24]),
    .A1(d[24]),
    .S(enable),
    .X(_056_)
  );
  sky130_fd_sc_hd__mux2_1 _073_ (
    .A0(q[23]),
    .A1(d[23]),
    .S(enable),
    .X(_055_)
  );
  sky130_fd_sc_hd__mux2_1 _074_ (
    .A0(q[22]),
    .A1(d[22]),
    .S(enable),
    .X(_054_)
  );
  sky130_fd_sc_hd__mux2_1 _075_ (
    .A0(q[21]),
    .A1(d[21]),
    .S(enable),
    .X(_053_)
  );
  sky130_fd_sc_hd__mux2_1 _076_ (
    .A0(q[20]),
    .A1(d[20]),
    .S(enable),
    .X(_052_)
  );
  sky130_fd_sc_hd__mux2_1 _077_ (
    .A0(q[19]),
    .A1(d[19]),
    .S(enable),
    .X(_051_)
  );
  sky130_fd_sc_hd__mux2_1 _078_ (
    .A0(q[18]),
    .A1(d[18]),
    .S(enable),
    .X(_050_)
  );
  sky130_fd_sc_hd__mux2_1 _079_ (
    .A0(q[17]),
    .A1(d[17]),
    .S(enable),
    .X(_049_)
  );
  sky130_fd_sc_hd__mux2_1 _080_ (
    .A0(q[16]),
    .A1(d[16]),
    .S(enable),
    .X(_048_)
  );
  sky130_fd_sc_hd__mux2_1 _081_ (
    .A0(q[15]),
    .A1(d[15]),
    .S(enable),
    .X(_047_)
  );
  sky130_fd_sc_hd__mux2_1 _082_ (
    .A0(q[14]),
    .A1(d[14]),
    .S(enable),
    .X(_046_)
  );
  sky130_fd_sc_hd__mux2_1 _083_ (
    .A0(q[13]),
    .A1(d[13]),
    .S(enable),
    .X(_045_)
  );
  sky130_fd_sc_hd__mux2_1 _084_ (
    .A0(q[12]),
    .A1(d[12]),
    .S(enable),
    .X(_044_)
  );
  sky130_fd_sc_hd__mux2_1 _085_ (
    .A0(q[11]),
    .A1(d[11]),
    .S(enable),
    .X(_043_)
  );
  sky130_fd_sc_hd__mux2_1 _086_ (
    .A0(q[10]),
    .A1(d[10]),
    .S(enable),
    .X(_042_)
  );
  sky130_fd_sc_hd__mux2_1 _087_ (
    .A0(q[9]),
    .A1(d[9]),
    .S(enable),
    .X(_041_)
  );
  sky130_fd_sc_hd__mux2_1 _088_ (
    .A0(q[8]),
    .A1(d[8]),
    .S(enable),
    .X(_040_)
  );
  sky130_fd_sc_hd__mux2_1 _089_ (
    .A0(q[7]),
    .A1(d[7]),
    .S(enable),
    .X(_039_)
  );
  sky130_fd_sc_hd__mux2_1 _090_ (
    .A0(q[6]),
    .A1(d[6]),
    .S(enable),
    .X(_038_)
  );
  sky130_fd_sc_hd__mux2_1 _091_ (
    .A0(q[5]),
    .A1(d[5]),
    .S(enable),
    .X(_037_)
  );
  sky130_fd_sc_hd__mux2_1 _092_ (
    .A0(q[4]),
    .A1(d[4]),
    .S(enable),
    .X(_036_)
  );
  sky130_fd_sc_hd__mux2_1 _093_ (
    .A0(q[3]),
    .A1(d[3]),
    .S(enable),
    .X(_035_)
  );
  sky130_fd_sc_hd__mux2_1 _094_ (
    .A0(q[2]),
    .A1(d[2]),
    .S(enable),
    .X(_034_)
  );
  sky130_fd_sc_hd__mux2_1 _095_ (
    .A0(q[1]),
    .A1(d[1]),
    .S(enable),
    .X(_033_)
  );
  sky130_fd_sc_hd__mux2_1 _096_ (
    .A0(q[0]),
    .A1(d[0]),
    .S(enable),
    .X(_032_)
  );
  sky130_fd_sc_hd__clkinv_1 _097_ (
    .A(reset),
    .Y(_001_)
  );
  sky130_fd_sc_hd__clkinv_1 _098_ (
    .A(reset),
    .Y(_002_)
  );
  sky130_fd_sc_hd__clkinv_1 _099_ (
    .A(reset),
    .Y(_003_)
  );
  sky130_fd_sc_hd__clkinv_1 _100_ (
    .A(reset),
    .Y(_004_)
  );
  sky130_fd_sc_hd__clkinv_1 _101_ (
    .A(reset),
    .Y(_005_)
  );
  sky130_fd_sc_hd__clkinv_1 _102_ (
    .A(reset),
    .Y(_006_)
  );
  sky130_fd_sc_hd__clkinv_1 _103_ (
    .A(reset),
    .Y(_007_)
  );
  sky130_fd_sc_hd__clkinv_1 _104_ (
    .A(reset),
    .Y(_008_)
  );
  sky130_fd_sc_hd__clkinv_1 _105_ (
    .A(reset),
    .Y(_009_)
  );
  sky130_fd_sc_hd__clkinv_1 _106_ (
    .A(reset),
    .Y(_010_)
  );
  sky130_fd_sc_hd__clkinv_1 _107_ (
    .A(reset),
    .Y(_011_)
  );
  sky130_fd_sc_hd__clkinv_1 _108_ (
    .A(reset),
    .Y(_012_)
  );
  sky130_fd_sc_hd__clkinv_1 _109_ (
    .A(reset),
    .Y(_013_)
  );
  sky130_fd_sc_hd__clkinv_1 _110_ (
    .A(reset),
    .Y(_014_)
  );
  sky130_fd_sc_hd__clkinv_1 _111_ (
    .A(reset),
    .Y(_015_)
  );
  sky130_fd_sc_hd__clkinv_1 _112_ (
    .A(reset),
    .Y(_016_)
  );
  sky130_fd_sc_hd__clkinv_1 _113_ (
    .A(reset),
    .Y(_017_)
  );
  sky130_fd_sc_hd__clkinv_1 _114_ (
    .A(reset),
    .Y(_018_)
  );
  sky130_fd_sc_hd__clkinv_1 _115_ (
    .A(reset),
    .Y(_019_)
  );
  sky130_fd_sc_hd__clkinv_1 _116_ (
    .A(reset),
    .Y(_020_)
  );
  sky130_fd_sc_hd__clkinv_1 _117_ (
    .A(reset),
    .Y(_021_)
  );
  sky130_fd_sc_hd__clkinv_1 _118_ (
    .A(reset),
    .Y(_022_)
  );
  sky130_fd_sc_hd__clkinv_1 _119_ (
    .A(reset),
    .Y(_023_)
  );
  sky130_fd_sc_hd__clkinv_1 _120_ (
    .A(reset),
    .Y(_024_)
  );
  sky130_fd_sc_hd__clkinv_1 _121_ (
    .A(reset),
    .Y(_025_)
  );
  sky130_fd_sc_hd__clkinv_1 _122_ (
    .A(reset),
    .Y(_026_)
  );
  sky130_fd_sc_hd__clkinv_1 _123_ (
    .A(reset),
    .Y(_027_)
  );
  sky130_fd_sc_hd__clkinv_1 _124_ (
    .A(reset),
    .Y(_028_)
  );
  sky130_fd_sc_hd__clkinv_1 _125_ (
    .A(reset),
    .Y(_029_)
  );
  sky130_fd_sc_hd__clkinv_1 _126_ (
    .A(reset),
    .Y(_030_)
  );
  sky130_fd_sc_hd__clkinv_1 _127_ (
    .A(reset),
    .Y(_031_)
  );
  sky130_fd_sc_hd__dfrtp_1 _128_ (
    .CLK(clk),
    .D(_032_),
    .Q(q[0]),
    .RESET_B(_000_)
  );
  sky130_fd_sc_hd__dfrtp_1 _129_ (
    .CLK(clk),
    .D(_033_),
    .Q(q[1]),
    .RESET_B(_001_)
  );
  sky130_fd_sc_hd__dfrtp_1 _130_ (
    .CLK(clk),
    .D(_034_),
    .Q(q[2]),
    .RESET_B(_002_)
  );
  sky130_fd_sc_hd__dfrtp_1 _131_ (
    .CLK(clk),
    .D(_035_),
    .Q(q[3]),
    .RESET_B(_003_)
  );
  sky130_fd_sc_hd__dfrtp_1 _132_ (
    .CLK(clk),
    .D(_036_),
    .Q(q[4]),
    .RESET_B(_004_)
  );
  sky130_fd_sc_hd__dfrtp_1 _133_ (
    .CLK(clk),
    .D(_037_),
    .Q(q[5]),
    .RESET_B(_005_)
  );
  sky130_fd_sc_hd__dfrtp_1 _134_ (
    .CLK(clk),
    .D(_038_),
    .Q(q[6]),
    .RESET_B(_006_)
  );
  sky130_fd_sc_hd__dfrtp_1 _135_ (
    .CLK(clk),
    .D(_039_),
    .Q(q[7]),
    .RESET_B(_007_)
  );
  sky130_fd_sc_hd__dfrtp_1 _136_ (
    .CLK(clk),
    .D(_040_),
    .Q(q[8]),
    .RESET_B(_008_)
  );
  sky130_fd_sc_hd__dfrtp_1 _137_ (
    .CLK(clk),
    .D(_041_),
    .Q(q[9]),
    .RESET_B(_009_)
  );
  sky130_fd_sc_hd__dfrtp_1 _138_ (
    .CLK(clk),
    .D(_042_),
    .Q(q[10]),
    .RESET_B(_010_)
  );
  sky130_fd_sc_hd__dfrtp_1 _139_ (
    .CLK(clk),
    .D(_043_),
    .Q(q[11]),
    .RESET_B(_011_)
  );
  sky130_fd_sc_hd__dfrtp_1 _140_ (
    .CLK(clk),
    .D(_044_),
    .Q(q[12]),
    .RESET_B(_012_)
  );
  sky130_fd_sc_hd__dfrtp_1 _141_ (
    .CLK(clk),
    .D(_045_),
    .Q(q[13]),
    .RESET_B(_013_)
  );
  sky130_fd_sc_hd__dfrtp_1 _142_ (
    .CLK(clk),
    .D(_046_),
    .Q(q[14]),
    .RESET_B(_014_)
  );
  sky130_fd_sc_hd__dfrtp_1 _143_ (
    .CLK(clk),
    .D(_047_),
    .Q(q[15]),
    .RESET_B(_015_)
  );
  sky130_fd_sc_hd__dfrtp_1 _144_ (
    .CLK(clk),
    .D(_048_),
    .Q(q[16]),
    .RESET_B(_016_)
  );
  sky130_fd_sc_hd__dfrtp_1 _145_ (
    .CLK(clk),
    .D(_049_),
    .Q(q[17]),
    .RESET_B(_017_)
  );
  sky130_fd_sc_hd__dfrtp_1 _146_ (
    .CLK(clk),
    .D(_050_),
    .Q(q[18]),
    .RESET_B(_018_)
  );
  sky130_fd_sc_hd__dfrtp_1 _147_ (
    .CLK(clk),
    .D(_051_),
    .Q(q[19]),
    .RESET_B(_019_)
  );
  sky130_fd_sc_hd__dfrtp_1 _148_ (
    .CLK(clk),
    .D(_052_),
    .Q(q[20]),
    .RESET_B(_020_)
  );
  sky130_fd_sc_hd__dfrtp_1 _149_ (
    .CLK(clk),
    .D(_053_),
    .Q(q[21]),
    .RESET_B(_021_)
  );
  sky130_fd_sc_hd__dfrtp_1 _150_ (
    .CLK(clk),
    .D(_054_),
    .Q(q[22]),
    .RESET_B(_022_)
  );
  sky130_fd_sc_hd__dfrtp_1 _151_ (
    .CLK(clk),
    .D(_055_),
    .Q(q[23]),
    .RESET_B(_023_)
  );
  sky130_fd_sc_hd__dfrtp_1 _152_ (
    .CLK(clk),
    .D(_056_),
    .Q(q[24]),
    .RESET_B(_024_)
  );
  sky130_fd_sc_hd__dfrtp_1 _153_ (
    .CLK(clk),
    .D(_057_),
    .Q(q[25]),
    .RESET_B(_025_)
  );
  sky130_fd_sc_hd__dfrtp_1 _154_ (
    .CLK(clk),
    .D(_058_),
    .Q(q[26]),
    .RESET_B(_026_)
  );
  sky130_fd_sc_hd__dfrtp_1 _155_ (
    .CLK(clk),
    .D(_059_),
    .Q(q[27]),
    .RESET_B(_027_)
  );
  sky130_fd_sc_hd__dfrtp_1 _156_ (
    .CLK(clk),
    .D(_060_),
    .Q(q[28]),
    .RESET_B(_028_)
  );
  sky130_fd_sc_hd__dfrtp_1 _157_ (
    .CLK(clk),
    .D(_061_),
    .Q(q[29]),
    .RESET_B(_029_)
  );
  sky130_fd_sc_hd__dfrtp_1 _158_ (
    .CLK(clk),
    .D(_062_),
    .Q(q[30]),
    .RESET_B(_030_)
  );
  sky130_fd_sc_hd__dfrtp_1 _159_ (
    .CLK(clk),
    .D(_063_),
    .Q(q[31]),
    .RESET_B(_031_)
  );
endmodule

module \$paramod\register_bank\BITS=s32'00000000000000000000000001000000 (d, reset, enable, q, clk);
  wire _000_;
  wire _001_;
  wire _002_;
  wire _003_;
  wire _004_;
  wire _005_;
  wire _006_;
  wire _007_;
  wire _008_;
  wire _009_;
  wire _010_;
  wire _011_;
  wire _012_;
  wire _013_;
  wire _014_;
  wire _015_;
  wire _016_;
  wire _017_;
  wire _018_;
  wire _019_;
  wire _020_;
  wire _021_;
  wire _022_;
  wire _023_;
  wire _024_;
  wire _025_;
  wire _026_;
  wire _027_;
  wire _028_;
  wire _029_;
  wire _030_;
  wire _031_;
  wire _032_;
  wire _033_;
  wire _034_;
  wire _035_;
  wire _036_;
  wire _037_;
  wire _038_;
  wire _039_;
  wire _040_;
  wire _041_;
  wire _042_;
  wire _043_;
  wire _044_;
  wire _045_;
  wire _046_;
  wire _047_;
  wire _048_;
  wire _049_;
  wire _050_;
  wire _051_;
  wire _052_;
  wire _053_;
  wire _054_;
  wire _055_;
  wire _056_;
  wire _057_;
  wire _058_;
  wire _059_;
  wire _060_;
  wire _061_;
  wire _062_;
  wire _063_;
  wire _064_;
  wire _065_;
  wire _066_;
  wire _067_;
  wire _068_;
  wire _069_;
  wire _070_;
  wire _071_;
  wire _072_;
  wire _073_;
  wire _074_;
  wire _075_;
  wire _076_;
  wire _077_;
  wire _078_;
  wire _079_;
  wire _080_;
  wire _081_;
  wire _082_;
  wire _083_;
  wire _084_;
  wire _085_;
  wire _086_;
  wire _087_;
  wire _088_;
  wire _089_;
  wire _090_;
  wire _091_;
  wire _092_;
  wire _093_;
  wire _094_;
  wire _095_;
  wire _096_;
  wire _097_;
  wire _098_;
  wire _099_;
  wire _100_;
  wire _101_;
  wire _102_;
  wire _103_;
  wire _104_;
  wire _105_;
  wire _106_;
  wire _107_;
  wire _108_;
  wire _109_;
  wire _110_;
  wire _111_;
  wire _112_;
  wire _113_;
  wire _114_;
  wire _115_;
  wire _116_;
  wire _117_;
  wire _118_;
  wire _119_;
  wire _120_;
  wire _121_;
  wire _122_;
  wire _123_;
  wire _124_;
  wire _125_;
  wire _126_;
  wire _127_;
  input clk;
  wire clk;
  input [63:0] d;
  wire [63:0] d;
  input enable;
  wire enable;
  output [63:0] q;
  wire [63:0] q;
  input reset;
  wire reset;
  sky130_fd_sc_hd__clkinv_1 _128_ (
    .A(reset),
    .Y(_000_)
  );
  sky130_fd_sc_hd__mux2_1 _129_ (
    .A0(q[10]),
    .A1(d[10]),
    .S(enable),
    .X(_127_)
  );
  sky130_fd_sc_hd__mux2_1 _130_ (
    .A0(q[9]),
    .A1(d[9]),
    .S(enable),
    .X(_126_)
  );
  sky130_fd_sc_hd__mux2_1 _131_ (
    .A0(q[8]),
    .A1(d[8]),
    .S(enable),
    .X(_125_)
  );
  sky130_fd_sc_hd__mux2_1 _132_ (
    .A0(q[7]),
    .A1(d[7]),
    .S(enable),
    .X(_124_)
  );
  sky130_fd_sc_hd__mux2_1 _133_ (
    .A0(q[6]),
    .A1(d[6]),
    .S(enable),
    .X(_123_)
  );
  sky130_fd_sc_hd__mux2_1 _134_ (
    .A0(q[5]),
    .A1(d[5]),
    .S(enable),
    .X(_122_)
  );
  sky130_fd_sc_hd__mux2_1 _135_ (
    .A0(q[4]),
    .A1(d[4]),
    .S(enable),
    .X(_121_)
  );
  sky130_fd_sc_hd__mux2_1 _136_ (
    .A0(q[3]),
    .A1(d[3]),
    .S(enable),
    .X(_120_)
  );
  sky130_fd_sc_hd__mux2_1 _137_ (
    .A0(q[2]),
    .A1(d[2]),
    .S(enable),
    .X(_119_)
  );
  sky130_fd_sc_hd__mux2_1 _138_ (
    .A0(q[1]),
    .A1(d[1]),
    .S(enable),
    .X(_118_)
  );
  sky130_fd_sc_hd__mux2_1 _139_ (
    .A0(q[0]),
    .A1(d[0]),
    .S(enable),
    .X(_117_)
  );
  sky130_fd_sc_hd__mux2_1 _140_ (
    .A0(q[63]),
    .A1(d[63]),
    .S(enable),
    .X(_116_)
  );
  sky130_fd_sc_hd__mux2_1 _141_ (
    .A0(q[62]),
    .A1(d[62]),
    .S(enable),
    .X(_115_)
  );
  sky130_fd_sc_hd__mux2_1 _142_ (
    .A0(q[61]),
    .A1(d[61]),
    .S(enable),
    .X(_114_)
  );
  sky130_fd_sc_hd__mux2_1 _143_ (
    .A0(q[60]),
    .A1(d[60]),
    .S(enable),
    .X(_113_)
  );
  sky130_fd_sc_hd__mux2_1 _144_ (
    .A0(q[59]),
    .A1(d[59]),
    .S(enable),
    .X(_112_)
  );
  sky130_fd_sc_hd__mux2_1 _145_ (
    .A0(q[58]),
    .A1(d[58]),
    .S(enable),
    .X(_111_)
  );
  sky130_fd_sc_hd__mux2_1 _146_ (
    .A0(q[57]),
    .A1(d[57]),
    .S(enable),
    .X(_110_)
  );
  sky130_fd_sc_hd__mux2_1 _147_ (
    .A0(q[56]),
    .A1(d[56]),
    .S(enable),
    .X(_109_)
  );
  sky130_fd_sc_hd__mux2_1 _148_ (
    .A0(q[55]),
    .A1(d[55]),
    .S(enable),
    .X(_108_)
  );
  sky130_fd_sc_hd__mux2_1 _149_ (
    .A0(q[54]),
    .A1(d[54]),
    .S(enable),
    .X(_107_)
  );
  sky130_fd_sc_hd__mux2_1 _150_ (
    .A0(q[53]),
    .A1(d[53]),
    .S(enable),
    .X(_106_)
  );
  sky130_fd_sc_hd__mux2_1 _151_ (
    .A0(q[52]),
    .A1(d[52]),
    .S(enable),
    .X(_105_)
  );
  sky130_fd_sc_hd__mux2_1 _152_ (
    .A0(q[51]),
    .A1(d[51]),
    .S(enable),
    .X(_104_)
  );
  sky130_fd_sc_hd__mux2_1 _153_ (
    .A0(q[50]),
    .A1(d[50]),
    .S(enable),
    .X(_103_)
  );
  sky130_fd_sc_hd__mux2_1 _154_ (
    .A0(q[49]),
    .A1(d[49]),
    .S(enable),
    .X(_102_)
  );
  sky130_fd_sc_hd__mux2_1 _155_ (
    .A0(q[48]),
    .A1(d[48]),
    .S(enable),
    .X(_101_)
  );
  sky130_fd_sc_hd__mux2_1 _156_ (
    .A0(q[47]),
    .A1(d[47]),
    .S(enable),
    .X(_100_)
  );
  sky130_fd_sc_hd__mux2_1 _157_ (
    .A0(q[46]),
    .A1(d[46]),
    .S(enable),
    .X(_099_)
  );
  sky130_fd_sc_hd__mux2_1 _158_ (
    .A0(q[45]),
    .A1(d[45]),
    .S(enable),
    .X(_098_)
  );
  sky130_fd_sc_hd__mux2_1 _159_ (
    .A0(q[44]),
    .A1(d[44]),
    .S(enable),
    .X(_097_)
  );
  sky130_fd_sc_hd__mux2_1 _160_ (
    .A0(q[43]),
    .A1(d[43]),
    .S(enable),
    .X(_096_)
  );
  sky130_fd_sc_hd__mux2_1 _161_ (
    .A0(q[42]),
    .A1(d[42]),
    .S(enable),
    .X(_095_)
  );
  sky130_fd_sc_hd__mux2_1 _162_ (
    .A0(q[41]),
    .A1(d[41]),
    .S(enable),
    .X(_094_)
  );
  sky130_fd_sc_hd__mux2_1 _163_ (
    .A0(q[40]),
    .A1(d[40]),
    .S(enable),
    .X(_093_)
  );
  sky130_fd_sc_hd__mux2_1 _164_ (
    .A0(q[39]),
    .A1(d[39]),
    .S(enable),
    .X(_092_)
  );
  sky130_fd_sc_hd__mux2_1 _165_ (
    .A0(q[38]),
    .A1(d[38]),
    .S(enable),
    .X(_091_)
  );
  sky130_fd_sc_hd__mux2_1 _166_ (
    .A0(q[37]),
    .A1(d[37]),
    .S(enable),
    .X(_090_)
  );
  sky130_fd_sc_hd__mux2_1 _167_ (
    .A0(q[36]),
    .A1(d[36]),
    .S(enable),
    .X(_089_)
  );
  sky130_fd_sc_hd__mux2_1 _168_ (
    .A0(q[35]),
    .A1(d[35]),
    .S(enable),
    .X(_088_)
  );
  sky130_fd_sc_hd__mux2_1 _169_ (
    .A0(q[34]),
    .A1(d[34]),
    .S(enable),
    .X(_087_)
  );
  sky130_fd_sc_hd__mux2_1 _170_ (
    .A0(q[33]),
    .A1(d[33]),
    .S(enable),
    .X(_086_)
  );
  sky130_fd_sc_hd__mux2_1 _171_ (
    .A0(q[32]),
    .A1(d[32]),
    .S(enable),
    .X(_085_)
  );
  sky130_fd_sc_hd__mux2_1 _172_ (
    .A0(q[31]),
    .A1(d[31]),
    .S(enable),
    .X(_084_)
  );
  sky130_fd_sc_hd__mux2_1 _173_ (
    .A0(q[30]),
    .A1(d[30]),
    .S(enable),
    .X(_083_)
  );
  sky130_fd_sc_hd__mux2_1 _174_ (
    .A0(q[29]),
    .A1(d[29]),
    .S(enable),
    .X(_082_)
  );
  sky130_fd_sc_hd__mux2_1 _175_ (
    .A0(q[28]),
    .A1(d[28]),
    .S(enable),
    .X(_081_)
  );
  sky130_fd_sc_hd__mux2_1 _176_ (
    .A0(q[27]),
    .A1(d[27]),
    .S(enable),
    .X(_080_)
  );
  sky130_fd_sc_hd__mux2_1 _177_ (
    .A0(q[26]),
    .A1(d[26]),
    .S(enable),
    .X(_079_)
  );
  sky130_fd_sc_hd__mux2_1 _178_ (
    .A0(q[25]),
    .A1(d[25]),
    .S(enable),
    .X(_078_)
  );
  sky130_fd_sc_hd__mux2_1 _179_ (
    .A0(q[24]),
    .A1(d[24]),
    .S(enable),
    .X(_077_)
  );
  sky130_fd_sc_hd__mux2_1 _180_ (
    .A0(q[23]),
    .A1(d[23]),
    .S(enable),
    .X(_076_)
  );
  sky130_fd_sc_hd__mux2_1 _181_ (
    .A0(q[22]),
    .A1(d[22]),
    .S(enable),
    .X(_075_)
  );
  sky130_fd_sc_hd__mux2_1 _182_ (
    .A0(q[21]),
    .A1(d[21]),
    .S(enable),
    .X(_074_)
  );
  sky130_fd_sc_hd__mux2_1 _183_ (
    .A0(q[20]),
    .A1(d[20]),
    .S(enable),
    .X(_073_)
  );
  sky130_fd_sc_hd__mux2_1 _184_ (
    .A0(q[19]),
    .A1(d[19]),
    .S(enable),
    .X(_072_)
  );
  sky130_fd_sc_hd__mux2_1 _185_ (
    .A0(q[18]),
    .A1(d[18]),
    .S(enable),
    .X(_071_)
  );
  sky130_fd_sc_hd__mux2_1 _186_ (
    .A0(q[17]),
    .A1(d[17]),
    .S(enable),
    .X(_070_)
  );
  sky130_fd_sc_hd__mux2_1 _187_ (
    .A0(q[16]),
    .A1(d[16]),
    .S(enable),
    .X(_069_)
  );
  sky130_fd_sc_hd__mux2_1 _188_ (
    .A0(q[15]),
    .A1(d[15]),
    .S(enable),
    .X(_068_)
  );
  sky130_fd_sc_hd__mux2_1 _189_ (
    .A0(q[14]),
    .A1(d[14]),
    .S(enable),
    .X(_067_)
  );
  sky130_fd_sc_hd__mux2_1 _190_ (
    .A0(q[13]),
    .A1(d[13]),
    .S(enable),
    .X(_066_)
  );
  sky130_fd_sc_hd__mux2_1 _191_ (
    .A0(q[12]),
    .A1(d[12]),
    .S(enable),
    .X(_065_)
  );
  sky130_fd_sc_hd__mux2_1 _192_ (
    .A0(q[11]),
    .A1(d[11]),
    .S(enable),
    .X(_064_)
  );
  sky130_fd_sc_hd__clkinv_1 _193_ (
    .A(reset),
    .Y(_001_)
  );
  sky130_fd_sc_hd__clkinv_1 _194_ (
    .A(reset),
    .Y(_002_)
  );
  sky130_fd_sc_hd__clkinv_1 _195_ (
    .A(reset),
    .Y(_003_)
  );
  sky130_fd_sc_hd__clkinv_1 _196_ (
    .A(reset),
    .Y(_004_)
  );
  sky130_fd_sc_hd__clkinv_1 _197_ (
    .A(reset),
    .Y(_005_)
  );
  sky130_fd_sc_hd__clkinv_1 _198_ (
    .A(reset),
    .Y(_006_)
  );
  sky130_fd_sc_hd__clkinv_1 _199_ (
    .A(reset),
    .Y(_007_)
  );
  sky130_fd_sc_hd__clkinv_1 _200_ (
    .A(reset),
    .Y(_008_)
  );
  sky130_fd_sc_hd__clkinv_1 _201_ (
    .A(reset),
    .Y(_009_)
  );
  sky130_fd_sc_hd__clkinv_1 _202_ (
    .A(reset),
    .Y(_010_)
  );
  sky130_fd_sc_hd__clkinv_1 _203_ (
    .A(reset),
    .Y(_011_)
  );
  sky130_fd_sc_hd__clkinv_1 _204_ (
    .A(reset),
    .Y(_012_)
  );
  sky130_fd_sc_hd__clkinv_1 _205_ (
    .A(reset),
    .Y(_013_)
  );
  sky130_fd_sc_hd__clkinv_1 _206_ (
    .A(reset),
    .Y(_014_)
  );
  sky130_fd_sc_hd__clkinv_1 _207_ (
    .A(reset),
    .Y(_015_)
  );
  sky130_fd_sc_hd__clkinv_1 _208_ (
    .A(reset),
    .Y(_016_)
  );
  sky130_fd_sc_hd__clkinv_1 _209_ (
    .A(reset),
    .Y(_017_)
  );
  sky130_fd_sc_hd__clkinv_1 _210_ (
    .A(reset),
    .Y(_018_)
  );
  sky130_fd_sc_hd__clkinv_1 _211_ (
    .A(reset),
    .Y(_019_)
  );
  sky130_fd_sc_hd__clkinv_1 _212_ (
    .A(reset),
    .Y(_020_)
  );
  sky130_fd_sc_hd__clkinv_1 _213_ (
    .A(reset),
    .Y(_021_)
  );
  sky130_fd_sc_hd__clkinv_1 _214_ (
    .A(reset),
    .Y(_022_)
  );
  sky130_fd_sc_hd__clkinv_1 _215_ (
    .A(reset),
    .Y(_023_)
  );
  sky130_fd_sc_hd__clkinv_1 _216_ (
    .A(reset),
    .Y(_024_)
  );
  sky130_fd_sc_hd__clkinv_1 _217_ (
    .A(reset),
    .Y(_025_)
  );
  sky130_fd_sc_hd__clkinv_1 _218_ (
    .A(reset),
    .Y(_026_)
  );
  sky130_fd_sc_hd__clkinv_1 _219_ (
    .A(reset),
    .Y(_027_)
  );
  sky130_fd_sc_hd__clkinv_1 _220_ (
    .A(reset),
    .Y(_028_)
  );
  sky130_fd_sc_hd__clkinv_1 _221_ (
    .A(reset),
    .Y(_029_)
  );
  sky130_fd_sc_hd__clkinv_1 _222_ (
    .A(reset),
    .Y(_030_)
  );
  sky130_fd_sc_hd__clkinv_1 _223_ (
    .A(reset),
    .Y(_031_)
  );
  sky130_fd_sc_hd__clkinv_1 _224_ (
    .A(reset),
    .Y(_032_)
  );
  sky130_fd_sc_hd__clkinv_1 _225_ (
    .A(reset),
    .Y(_033_)
  );
  sky130_fd_sc_hd__clkinv_1 _226_ (
    .A(reset),
    .Y(_034_)
  );
  sky130_fd_sc_hd__clkinv_1 _227_ (
    .A(reset),
    .Y(_035_)
  );
  sky130_fd_sc_hd__clkinv_1 _228_ (
    .A(reset),
    .Y(_036_)
  );
  sky130_fd_sc_hd__clkinv_1 _229_ (
    .A(reset),
    .Y(_037_)
  );
  sky130_fd_sc_hd__clkinv_1 _230_ (
    .A(reset),
    .Y(_038_)
  );
  sky130_fd_sc_hd__clkinv_1 _231_ (
    .A(reset),
    .Y(_039_)
  );
  sky130_fd_sc_hd__clkinv_1 _232_ (
    .A(reset),
    .Y(_040_)
  );
  sky130_fd_sc_hd__clkinv_1 _233_ (
    .A(reset),
    .Y(_041_)
  );
  sky130_fd_sc_hd__clkinv_1 _234_ (
    .A(reset),
    .Y(_042_)
  );
  sky130_fd_sc_hd__clkinv_1 _235_ (
    .A(reset),
    .Y(_043_)
  );
  sky130_fd_sc_hd__clkinv_1 _236_ (
    .A(reset),
    .Y(_044_)
  );
  sky130_fd_sc_hd__clkinv_1 _237_ (
    .A(reset),
    .Y(_045_)
  );
  sky130_fd_sc_hd__clkinv_1 _238_ (
    .A(reset),
    .Y(_046_)
  );
  sky130_fd_sc_hd__clkinv_1 _239_ (
    .A(reset),
    .Y(_047_)
  );
  sky130_fd_sc_hd__clkinv_1 _240_ (
    .A(reset),
    .Y(_048_)
  );
  sky130_fd_sc_hd__clkinv_1 _241_ (
    .A(reset),
    .Y(_049_)
  );
  sky130_fd_sc_hd__clkinv_1 _242_ (
    .A(reset),
    .Y(_050_)
  );
  sky130_fd_sc_hd__clkinv_1 _243_ (
    .A(reset),
    .Y(_051_)
  );
  sky130_fd_sc_hd__clkinv_1 _244_ (
    .A(reset),
    .Y(_052_)
  );
  sky130_fd_sc_hd__clkinv_1 _245_ (
    .A(reset),
    .Y(_053_)
  );
  sky130_fd_sc_hd__clkinv_1 _246_ (
    .A(reset),
    .Y(_054_)
  );
  sky130_fd_sc_hd__clkinv_1 _247_ (
    .A(reset),
    .Y(_055_)
  );
  sky130_fd_sc_hd__clkinv_1 _248_ (
    .A(reset),
    .Y(_056_)
  );
  sky130_fd_sc_hd__clkinv_1 _249_ (
    .A(reset),
    .Y(_057_)
  );
  sky130_fd_sc_hd__clkinv_1 _250_ (
    .A(reset),
    .Y(_058_)
  );
  sky130_fd_sc_hd__clkinv_1 _251_ (
    .A(reset),
    .Y(_059_)
  );
  sky130_fd_sc_hd__clkinv_1 _252_ (
    .A(reset),
    .Y(_060_)
  );
  sky130_fd_sc_hd__clkinv_1 _253_ (
    .A(reset),
    .Y(_061_)
  );
  sky130_fd_sc_hd__clkinv_1 _254_ (
    .A(reset),
    .Y(_062_)
  );
  sky130_fd_sc_hd__clkinv_1 _255_ (
    .A(reset),
    .Y(_063_)
  );
  sky130_fd_sc_hd__dfrtp_1 _256_ (
    .CLK(clk),
    .D(_064_),
    .Q(q[11]),
    .RESET_B(_000_)
  );
  sky130_fd_sc_hd__dfrtp_1 _257_ (
    .CLK(clk),
    .D(_065_),
    .Q(q[12]),
    .RESET_B(_001_)
  );
  sky130_fd_sc_hd__dfrtp_1 _258_ (
    .CLK(clk),
    .D(_066_),
    .Q(q[13]),
    .RESET_B(_002_)
  );
  sky130_fd_sc_hd__dfrtp_1 _259_ (
    .CLK(clk),
    .D(_067_),
    .Q(q[14]),
    .RESET_B(_003_)
  );
  sky130_fd_sc_hd__dfrtp_1 _260_ (
    .CLK(clk),
    .D(_068_),
    .Q(q[15]),
    .RESET_B(_004_)
  );
  sky130_fd_sc_hd__dfrtp_1 _261_ (
    .CLK(clk),
    .D(_069_),
    .Q(q[16]),
    .RESET_B(_005_)
  );
  sky130_fd_sc_hd__dfrtp_1 _262_ (
    .CLK(clk),
    .D(_070_),
    .Q(q[17]),
    .RESET_B(_006_)
  );
  sky130_fd_sc_hd__dfrtp_1 _263_ (
    .CLK(clk),
    .D(_071_),
    .Q(q[18]),
    .RESET_B(_007_)
  );
  sky130_fd_sc_hd__dfrtp_1 _264_ (
    .CLK(clk),
    .D(_072_),
    .Q(q[19]),
    .RESET_B(_008_)
  );
  sky130_fd_sc_hd__dfrtp_1 _265_ (
    .CLK(clk),
    .D(_073_),
    .Q(q[20]),
    .RESET_B(_009_)
  );
  sky130_fd_sc_hd__dfrtp_1 _266_ (
    .CLK(clk),
    .D(_074_),
    .Q(q[21]),
    .RESET_B(_010_)
  );
  sky130_fd_sc_hd__dfrtp_1 _267_ (
    .CLK(clk),
    .D(_075_),
    .Q(q[22]),
    .RESET_B(_011_)
  );
  sky130_fd_sc_hd__dfrtp_1 _268_ (
    .CLK(clk),
    .D(_076_),
    .Q(q[23]),
    .RESET_B(_012_)
  );
  sky130_fd_sc_hd__dfrtp_1 _269_ (
    .CLK(clk),
    .D(_077_),
    .Q(q[24]),
    .RESET_B(_013_)
  );
  sky130_fd_sc_hd__dfrtp_1 _270_ (
    .CLK(clk),
    .D(_078_),
    .Q(q[25]),
    .RESET_B(_014_)
  );
  sky130_fd_sc_hd__dfrtp_1 _271_ (
    .CLK(clk),
    .D(_079_),
    .Q(q[26]),
    .RESET_B(_015_)
  );
  sky130_fd_sc_hd__dfrtp_1 _272_ (
    .CLK(clk),
    .D(_080_),
    .Q(q[27]),
    .RESET_B(_016_)
  );
  sky130_fd_sc_hd__dfrtp_1 _273_ (
    .CLK(clk),
    .D(_081_),
    .Q(q[28]),
    .RESET_B(_017_)
  );
  sky130_fd_sc_hd__dfrtp_1 _274_ (
    .CLK(clk),
    .D(_082_),
    .Q(q[29]),
    .RESET_B(_018_)
  );
  sky130_fd_sc_hd__dfrtp_1 _275_ (
    .CLK(clk),
    .D(_083_),
    .Q(q[30]),
    .RESET_B(_019_)
  );
  sky130_fd_sc_hd__dfrtp_1 _276_ (
    .CLK(clk),
    .D(_084_),
    .Q(q[31]),
    .RESET_B(_020_)
  );
  sky130_fd_sc_hd__dfrtp_1 _277_ (
    .CLK(clk),
    .D(_085_),
    .Q(q[32]),
    .RESET_B(_021_)
  );
  sky130_fd_sc_hd__dfrtp_1 _278_ (
    .CLK(clk),
    .D(_086_),
    .Q(q[33]),
    .RESET_B(_022_)
  );
  sky130_fd_sc_hd__dfrtp_1 _279_ (
    .CLK(clk),
    .D(_087_),
    .Q(q[34]),
    .RESET_B(_023_)
  );
  sky130_fd_sc_hd__dfrtp_1 _280_ (
    .CLK(clk),
    .D(_088_),
    .Q(q[35]),
    .RESET_B(_024_)
  );
  sky130_fd_sc_hd__dfrtp_1 _281_ (
    .CLK(clk),
    .D(_089_),
    .Q(q[36]),
    .RESET_B(_025_)
  );
  sky130_fd_sc_hd__dfrtp_1 _282_ (
    .CLK(clk),
    .D(_090_),
    .Q(q[37]),
    .RESET_B(_026_)
  );
  sky130_fd_sc_hd__dfrtp_1 _283_ (
    .CLK(clk),
    .D(_091_),
    .Q(q[38]),
    .RESET_B(_027_)
  );
  sky130_fd_sc_hd__dfrtp_1 _284_ (
    .CLK(clk),
    .D(_092_),
    .Q(q[39]),
    .RESET_B(_028_)
  );
  sky130_fd_sc_hd__dfrtp_1 _285_ (
    .CLK(clk),
    .D(_093_),
    .Q(q[40]),
    .RESET_B(_029_)
  );
  sky130_fd_sc_hd__dfrtp_1 _286_ (
    .CLK(clk),
    .D(_094_),
    .Q(q[41]),
    .RESET_B(_030_)
  );
  sky130_fd_sc_hd__dfrtp_1 _287_ (
    .CLK(clk),
    .D(_095_),
    .Q(q[42]),
    .RESET_B(_031_)
  );
  sky130_fd_sc_hd__dfrtp_1 _288_ (
    .CLK(clk),
    .D(_096_),
    .Q(q[43]),
    .RESET_B(_032_)
  );
  sky130_fd_sc_hd__dfrtp_1 _289_ (
    .CLK(clk),
    .D(_097_),
    .Q(q[44]),
    .RESET_B(_033_)
  );
  sky130_fd_sc_hd__dfrtp_1 _290_ (
    .CLK(clk),
    .D(_098_),
    .Q(q[45]),
    .RESET_B(_034_)
  );
  sky130_fd_sc_hd__dfrtp_1 _291_ (
    .CLK(clk),
    .D(_099_),
    .Q(q[46]),
    .RESET_B(_035_)
  );
  sky130_fd_sc_hd__dfrtp_1 _292_ (
    .CLK(clk),
    .D(_100_),
    .Q(q[47]),
    .RESET_B(_036_)
  );
  sky130_fd_sc_hd__dfrtp_1 _293_ (
    .CLK(clk),
    .D(_101_),
    .Q(q[48]),
    .RESET_B(_037_)
  );
  sky130_fd_sc_hd__dfrtp_1 _294_ (
    .CLK(clk),
    .D(_102_),
    .Q(q[49]),
    .RESET_B(_038_)
  );
  sky130_fd_sc_hd__dfrtp_1 _295_ (
    .CLK(clk),
    .D(_103_),
    .Q(q[50]),
    .RESET_B(_039_)
  );
  sky130_fd_sc_hd__dfrtp_1 _296_ (
    .CLK(clk),
    .D(_104_),
    .Q(q[51]),
    .RESET_B(_040_)
  );
  sky130_fd_sc_hd__dfrtp_1 _297_ (
    .CLK(clk),
    .D(_105_),
    .Q(q[52]),
    .RESET_B(_041_)
  );
  sky130_fd_sc_hd__dfrtp_1 _298_ (
    .CLK(clk),
    .D(_106_),
    .Q(q[53]),
    .RESET_B(_042_)
  );
  sky130_fd_sc_hd__dfrtp_1 _299_ (
    .CLK(clk),
    .D(_107_),
    .Q(q[54]),
    .RESET_B(_043_)
  );
  sky130_fd_sc_hd__dfrtp_1 _300_ (
    .CLK(clk),
    .D(_108_),
    .Q(q[55]),
    .RESET_B(_044_)
  );
  sky130_fd_sc_hd__dfrtp_1 _301_ (
    .CLK(clk),
    .D(_109_),
    .Q(q[56]),
    .RESET_B(_045_)
  );
  sky130_fd_sc_hd__dfrtp_1 _302_ (
    .CLK(clk),
    .D(_110_),
    .Q(q[57]),
    .RESET_B(_046_)
  );
  sky130_fd_sc_hd__dfrtp_1 _303_ (
    .CLK(clk),
    .D(_111_),
    .Q(q[58]),
    .RESET_B(_047_)
  );
  sky130_fd_sc_hd__dfrtp_1 _304_ (
    .CLK(clk),
    .D(_112_),
    .Q(q[59]),
    .RESET_B(_048_)
  );
  sky130_fd_sc_hd__dfrtp_1 _305_ (
    .CLK(clk),
    .D(_113_),
    .Q(q[60]),
    .RESET_B(_049_)
  );
  sky130_fd_sc_hd__dfrtp_1 _306_ (
    .CLK(clk),
    .D(_114_),
    .Q(q[61]),
    .RESET_B(_050_)
  );
  sky130_fd_sc_hd__dfrtp_1 _307_ (
    .CLK(clk),
    .D(_115_),
    .Q(q[62]),
    .RESET_B(_051_)
  );
  sky130_fd_sc_hd__dfrtp_1 _308_ (
    .CLK(clk),
    .D(_116_),
    .Q(q[63]),
    .RESET_B(_052_)
  );
  sky130_fd_sc_hd__dfrtp_1 _309_ (
    .CLK(clk),
    .D(_117_),
    .Q(q[0]),
    .RESET_B(_053_)
  );
  sky130_fd_sc_hd__dfrtp_1 _310_ (
    .CLK(clk),
    .D(_118_),
    .Q(q[1]),
    .RESET_B(_054_)
  );
  sky130_fd_sc_hd__dfrtp_1 _311_ (
    .CLK(clk),
    .D(_119_),
    .Q(q[2]),
    .RESET_B(_055_)
  );
  sky130_fd_sc_hd__dfrtp_1 _312_ (
    .CLK(clk),
    .D(_120_),
    .Q(q[3]),
    .RESET_B(_056_)
  );
  sky130_fd_sc_hd__dfrtp_1 _313_ (
    .CLK(clk),
    .D(_121_),
    .Q(q[4]),
    .RESET_B(_057_)
  );
  sky130_fd_sc_hd__dfrtp_1 _314_ (
    .CLK(clk),
    .D(_122_),
    .Q(q[5]),
    .RESET_B(_058_)
  );
  sky130_fd_sc_hd__dfrtp_1 _315_ (
    .CLK(clk),
    .D(_123_),
    .Q(q[6]),
    .RESET_B(_059_)
  );
  sky130_fd_sc_hd__dfrtp_1 _316_ (
    .CLK(clk),
    .D(_124_),
    .Q(q[7]),
    .RESET_B(_060_)
  );
  sky130_fd_sc_hd__dfrtp_1 _317_ (
    .CLK(clk),
    .D(_125_),
    .Q(q[8]),
    .RESET_B(_061_)
  );
  sky130_fd_sc_hd__dfrtp_1 _318_ (
    .CLK(clk),
    .D(_126_),
    .Q(q[9]),
    .RESET_B(_062_)
  );
  sky130_fd_sc_hd__dfrtp_1 _319_ (
    .CLK(clk),
    .D(_127_),
    .Q(q[10]),
    .RESET_B(_063_)
  );
endmodule

module top(inputs_32, outputs_32, inputs_64, outputs_64, enable, reset, clk1, clk2);
  input clk1;
  wire clk1;
  input clk2;
  wire clk2;
  input enable;
  wire enable;
  input [31:0] inputs_32;
  wire [31:0] inputs_32;
  input [63:0] inputs_64;
  wire [63:0] inputs_64;
  output [31:0] outputs_32;
  wire [31:0] outputs_32;
  output [63:0] outputs_64;
  wire [63:0] outputs_64;
  input reset;
  wire reset;
  \$paramod\register_bank\BITS=s32'00000000000000000000000000100000  bank1 (
    .clk(clk1),
    .d(inputs_32),
    .enable(enable),
    .q(outputs_32),
    .reset(reset)
  );
  \$paramod\register_bank\BITS=s32'00000000000000000000000001000000  bank2 (
    .clk(clk2),
    .d(inputs_64),
    .enable(enable),
    .q(outputs_64),
    .reset(reset)
  );
endmodule