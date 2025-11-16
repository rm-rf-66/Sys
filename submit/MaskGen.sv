`include "core_struct.vh"

module MaskGen(
    input CorePack::mem_op_enum mem_op,
    input CorePack::addr_t dmem_waddr,
    output CorePack::mask_t dmem_wmask
);

  import CorePack::*;
logic [2:0] offset;
assign offset = dmem_waddr[2:0];
always_comb begin 
    case (mem_op)
        MEM_B: begin
          dmem_wmask = 8'b00000001 << offset;
        end
        MEM_H: begin
          dmem_wmask = 8'b00000011 << offset;
        end
        MEM_W: begin
          dmem_wmask = 8'b00001111 << offset;
        end
        MEM_D: begin
          dmem_wmask = 8'b11111111;
        end
        default: dmem_wmask = 8'b00000000;
    endcase
end

endmodule
