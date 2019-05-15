

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
    input  wire in,
    output wire out
);

nmos4_standard XN (
    .B( VSS ),
    .D( out ),
    .G( in ),
    .S( VSS )
);

pmos4_standard XP (
    .B( VDD ),
    .D( out ),
    .G( in ),
    .S( VDD )
);

endmodule
