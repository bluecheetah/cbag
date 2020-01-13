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
    input  wire in,
    output wire out
);

pmos4_standard XP (
    .B( VDD ),
    .D( out ),
    .G( in ),
    .S( VSS )
);

endmodule


module TEST(
    input  wire in,
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
