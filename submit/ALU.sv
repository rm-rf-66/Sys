`include "core_struct.vh"
module ALU (
  input  CorePack::data_t a,
  input  CorePack::data_t b,
  input  CorePack::alu_op_enum  alu_op,
  output CorePack::data_t res
);

  import CorePack::*;

data_t result;
data_t add_res, sub_res, sll_res, slt_res, srl_res, sra_res;
logic [31:0] addw_res, subw_res, sllw_res, srlw_res, sraw_res;
data_t num_sh;

assign num_sh = (alu_op inside {ALU_SLLW, ALU_SRLW, ALU_SRAW}) ?  {59'b0, b[4:0]} : {58'b0, b[5:0]};              

assign add_res = a + b;
assign sub_res = a - b;

assign addw_res = a[31:0] + b[31:0];
assign subw_res = a[31:0] - b[31:0];

assign sll_res = a << num_sh;
assign srl_res = a >> num_sh;
assign sra_res = $signed(a) >>> num_sh;

assign sllw_res = a[31:0] << num_sh[4:0];
assign srlw_res = a[31:0] >> num_sh[4:0];
assign sraw_res = $signed(a[31:0]) >>> num_sh[4:0];

always_comb begin
    case (alu_op)
        ALU_ADD: result = add_res;
        ALU_SUB: result = sub_res;
        ALU_AND: result = a & b;
        ALU_OR: result = a | b;
        ALU_XOR: result = a ^ b;
        ALU_SLT: result = ($signed(a) < $signed(b)) ? 1 : 0;
        ALU_SLTU: result = (a < b) ? 1 : 0;
        ALU_SLL: result = sll_res;
        ALU_SRL: result = srl_res;
        ALU_SRA: result = sra_res;
        ALU_ADDW: result = {{32{addw_res[31]}}, addw_res}; 
        ALU_SUBW: result = {{32{subw_res[31]}}, subw_res}; 
        ALU_SLLW: result = {{32{sllw_res[31]}}, sllw_res}; 
        ALU_SRLW: result = {{32{srlw_res[31]}}, srlw_res}; 
        ALU_SRAW: result = {{32{sraw_res[31]}}, sraw_res}; 
        default: result = '0; 
    endcase
end

assign res = result;


endmodule
