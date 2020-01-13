`timescale 1ps/1ps 


module pmos4_standard(
    inout  wire B,
    inout  wire D,
    inout  wire G,
    inout  wire S
);

endmodule


module cv_bus_term(
    input  wire VDD,
    input  wire VSS,
    input  wire [1:0] in,
    output wire out
);

pmos4_standard X4 (
    .B( VDD ),
    .D( out ),
    .G( in[1] ),
    .S( VSS )
);

pmos4_standard XP (
    .B( VDD ),
    .D( out ),
    .G( in[0] ),
    .S( VSS )
);

endmodule


module TEST__w_sup(
    input  wire VDD,
    input  wire VSS,
    input  wire [1:0] in0,
    input  wire [1:0] in1,
    output wire [1:0] out
);

cv_bus_term XINST_1 (
    .VDD( VDD ),
    .VSS( VSS ),
    .in( in1[1:0] ),
    .out( out[1] )
);

cv_bus_term XINST_0 (
    .VDD( VDD ),
    .VSS( VSS ),
    .in( in0[1:0] ),
    .out( out[0] )
);

endmodule


module TEST(
    input  wire [1:0] in0,
    input  wire [1:0] in1,
    output wire [1:0] out
);

wire VDD_val;
wire VSS_val;

assign VDD_val = 1'b1;
assign VSS_val = 1'b0;

TEST__w_sup XDUT (
    .VDD( VDD_val ),
    .VSS( VSS_val ),
    .in0( in0 ),
    .in1( in1 ),
    .out( out )
);

endmodule
