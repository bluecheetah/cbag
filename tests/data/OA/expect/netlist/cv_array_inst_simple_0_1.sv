`timescale 1ps/1ps 


module pmos4_standard(
    inout  wire B,
    inout  wire D,
    inout  wire G,
    inout  wire S
);

endmodule


module TEST__w_sup(
    input  wire VDD,
    input  wire VSS,
    input  wire [1:0] in,
    output wire out
);

pmos4_standard XP2_1 (
    .B( VSS ),
    .D( out ),
    .G( in[1] ),
    .S( VSS )
);

pmos4_standard XP2_0 (
    .B( VSS ),
    .D( out ),
    .G( in[0] ),
    .S( VSS )
);

pmos4_standard XP_1 (
    .B( VDD ),
    .D( out ),
    .G( in[1] ),
    .S( VDD )
);

pmos4_standard XP_0 (
    .B( VDD ),
    .D( out ),
    .G( in[0] ),
    .S( VDD )
);

endmodule


module TEST(
    input  wire [1:0] in,
    output wire out
);

wire VDD_val;
wire VSS_val;

assign VDD_val = 1'b1;
assign VSS_val = 1'b0;

TEST__w_sup XDUT (
    .VDD( VDD_val ),
    .VSS( VSS_val ),
    .in( in ),
    .out( out )
);

endmodule
