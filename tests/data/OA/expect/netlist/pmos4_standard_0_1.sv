`timescale 1ps/1ps 


module TEST__w_sup(
    inout  wire B,
    inout  wire D,
    inout  wire G,
    inout  wire S
);

endmodule


module TEST(
    inout  wire B,
    inout  wire D,
    inout  wire G,
    inout  wire S
);



TEST__w_sup XDUT (
    .B( B ),
    .D( D ),
    .G( G ),
    .S( S )
);

endmodule
