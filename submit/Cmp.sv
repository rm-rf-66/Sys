`include"core_struct.vh"
module Cmp (
    input CorePack::data_t a,
    input CorePack::data_t b,
    input CorePack::cmp_op_enum cmp_op,
    output cmp_res
);

    import CorePack::*;

logic result;
always_comb begin
    case (cmp_op)
        CMP_NO: result = 1'b0;
        CMP_EQ: result = (a == b);
        CMP_NE: result = (a != b);
        CMP_LT: result = ($signed(a) < $signed(b));
        CMP_GE: result = ($signed(a) >= $signed(b));
        CMP_LTU: result = (a < b);
        CMP_GEU: result = (a >= b);
        default: result = 1'b0;
    endcase 
end

assign cmp_res = result;

endmodule