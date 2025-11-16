`include "core_struct.vh"

module DataPkg(
    input CorePack::mem_op_enum mem_op,
    input CorePack::data_t reg_data,
    input CorePack::addr_t dmem_waddr,
    output CorePack::data_t dmem_wdata
);

  import CorePack::*;

logic [2:0] offset;
assign offset = dmem_waddr[2:0];

always_comb begin
    case (mem_op)
        MEM_B: begin 
            dmem_wdata = 64'b0;
            dmem_wdata[offset*8 +: 8] = reg_data[7:0]; 
        end
        MEM_H: begin 
            dmem_wdata = 64'b0;
            dmem_wdata[offset*8 +: 16] = reg_data[15:0]; 
        end
        MEM_W: begin 
            dmem_wdata = 64'b0;
            dmem_wdata[offset*8 +: 32] = reg_data[31:0]; 
        end
        MEM_D: begin 
            dmem_wdata = reg_data; 
        end
        default: begin 
            dmem_wdata = 64'b0;
        end

    endcase
end

endmodule
