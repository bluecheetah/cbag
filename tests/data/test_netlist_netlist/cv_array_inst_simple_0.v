

module nmos4_standard(
    inout  wire B,
    inout  wire D,
    inout  wire G,
    inout  wire S
);

endmodule


module pmos4_standard(
    inout  wire B,
    inout  wire D,
    inout  wire G,
    inout  wire S
);

endmodule


module TEST(
    input  wire VDD,
    input  wire VSS,
    input  wire [1:0] in,
    output wire out
);

nmos4_standard XN_1 (
    .B( VSS ),
    .D( out ),
    .G( in[1] ),
    .S( VSS )
);

nmos4_standard XN_0 (
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
