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


module TEST(
    input  wire VDD,
    input  wire VSS,
    input  wire [1:0] in,
    output wire out
);

cv_bus_term XINST (
    .VDD( VDD ),
    .VSS( VSS ),
    .in( in[1:0] ),
    .out( out )
);

endmodule
