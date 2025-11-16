`include "core_struct.vh"

module Immgen(
    input CorePack::imm_op_enum immgen_op,
    input logic [24:0] inst,
    output CorePack::data_t imm
);
    import CorePack::*;

always_comb begin 
    case (immgen_op)
        I_IMM: begin 
            imm = {{52{inst[24]}}, {inst[24:13]}};
        end
        S_IMM: begin
            imm = {{52{inst[24]}}, {inst[24:18]}, {inst[4:0]}};
        end
        B_IMM: begin 
            imm = {{52{inst[24]}}, {inst[0]}, {inst[23:18]}, {inst[4:1]}, 1'b0};
        end
        U_IMM: begin 
            imm = {{32{inst[24]}}, {inst[24:5]}, 12'b0};
        end
        UJ_IMM: begin
            imm = {{44{inst[24]}}, {inst[12:5]}, {inst[13]}, {inst[23:14]}, 1'b0};
        end
        IMM0: begin
            imm = 64'b0;
        end
        default: begin 
            imm = 64'b0;
        end
    endcase
end


endmodule
