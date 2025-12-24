`include "core_struct.vh"

module DataTrunc (
    input CorePack::data_t dmem_rdata,
    input CorePack::mem_op_enum mem_op,
    input CorePack::addr_t dmem_raddr,
    output CorePack::data_t read_data
);

  import CorePack::*;

logic [2:0] offset;
assign offset = dmem_raddr[2:0];
data_t trunc_data;
always_comb begin
    case (mem_op)
        MEM_B: begin
            trunc_data = {56'b0, dmem_rdata[offset*8 +: 8]};
            read_data = {{56{trunc_data[7]}}, trunc_data[7:0]};
        end
        MEM_UB: begin
            trunc_data = {56'b0, dmem_rdata[offset*8 +: 8]};
            read_data = {56'b0, dmem_rdata[offset*8 +: 8]};
        end
        MEM_H: begin
            trunc_data = {48'b0, dmem_rdata[offset*8 +: 16]};
            read_data = {{48{trunc_data[15]}}, trunc_data[15:0]};
        end
        MEM_UH: begin
            trunc_data = {48'b0, dmem_rdata[offset*8 +: 16]};
            read_data = {48'b0, dmem_rdata[offset*8 +: 16]};
        end
        MEM_W: begin
            trunc_data = {32'b0, dmem_rdata[offset*8 +: 32]};
            read_data = {{32{trunc_data[31]}}, trunc_data[31:0]};
        end
        MEM_UW: begin
            trunc_data = {32'b0, dmem_rdata[offset*8 +: 32]};
            read_data = {32'b0, dmem_rdata[offset*8 +: 32]};
        end
        MEM_D: begin
            trunc_data = dmem_rdata;
            read_data = dmem_rdata;
        end
        MEM_NO: begin
            trunc_data = 64'b0;
            read_data = 64'b0;
        end
    endcase
end

endmodule
