`include "core_struct.vh"
module controller (
    input CorePack::inst_t inst,
    output we_reg,
    output we_mem,
    output re_mem,
    output npc_sel,
    output CorePack::imm_op_enum immgen_op,
    output CorePack::alu_op_enum alu_op,
    output CorePack::cmp_op_enum cmp_op,
    output CorePack::alu_asel_op_enum alu_asel,
    output CorePack::alu_bsel_op_enum alu_bsel,
    output CorePack::wb_sel_op_enum wb_sel,
    output CorePack::mem_op_enum mem_op
    // output ControllerPack::ControllerSignals ctrl_signals
);

    import CorePack::*;
    import ControllerPack::*;


wire [6:0] opcode;
wire [2:0] funct3;
wire [6:0] funct7;


assign opcode = inst[6:0];
assign funct3 = inst[14:12];
assign funct7 = inst[31:25];

wire LOAD_inst   = opcode == 7'b0000011;
wire IMM_inst    = opcode == 7'b0010011;
wire AUIPC_inst  = opcode == 7'b0010111;
wire IMMW_inst   = opcode == 7'b0011011;
wire STORE_inst  = opcode == 7'b0100011;
wire REG_inst    = opcode == 7'b0110011;
wire LUI_inst    = opcode == 7'b0110111;
wire REGW_inst   = opcode == 7'b0111011;
wire BRANCH_inst = opcode == 7'b1100011;
wire JALR_inst   = opcode == 7'b1100111;
wire JAL_inst    = opcode == 7'b1101111;

wire reg_wen = IMM_inst | LOAD_inst | IMMW_inst | REG_inst | REGW_inst | JAL_inst | JALR_inst | AUIPC_inst | LUI_inst;
wire mem_wen = STORE_inst;
wire mem_ren = LOAD_inst;
wire sel_npc = JAL_inst | JALR_inst ;

assign we_reg = reg_wen;
assign we_mem = mem_wen;
assign re_mem = mem_ren;
assign npc_sel = sel_npc;

always_comb begin
    if (IMM_inst | IMMW_inst | LOAD_inst | JALR_inst) begin
        immgen_op = I_IMM;     
    end else if (STORE_inst) begin
        immgen_op = S_IMM;     
    end else if (BRANCH_inst) begin
        immgen_op = B_IMM;     
    end else if (LUI_inst | AUIPC_inst) begin
        immgen_op = U_IMM;     
    end else if (JAL_inst) begin
        immgen_op = UJ_IMM;    
    end else begin
        immgen_op = IMM0;      
    end
end

always_comb begin
    alu_op = ALU_ADD;
    if (REG_inst | REGW_inst) begin
        case (funct3)
            3'b000: begin
                if (REGW_inst) begin
                    alu_op = (funct7[5] ? ALU_SUBW : ALU_ADDW);
                end else begin
                    alu_op = (funct7[5] ? ALU_SUB : ALU_ADD);
                end
            end
            3'b001: alu_op = REGW_inst ? ALU_SLLW : ALU_SLL;  
            3'b010: alu_op = ALU_SLT;  
            3'b011: alu_op = ALU_SLTU; 
            3'b100: alu_op = ALU_XOR;  
            3'b101: begin
                if (REGW_inst) begin
                    alu_op = (funct7[5] ? ALU_SRAW : ALU_SRLW);
                end else begin
                    alu_op = (funct7[5] ? ALU_SRA : ALU_SRL);
                end
            end
            3'b110: alu_op = ALU_OR;   
            3'b111: alu_op = ALU_AND;  
        endcase
    end else if (IMM_inst | IMMW_inst) begin
        case (funct3)
            3'b000: alu_op = IMMW_inst ? ALU_ADDW : ALU_ADD;  
            3'b001: alu_op = IMMW_inst ? ALU_SLLW : ALU_SLL;  
            3'b010: alu_op = ALU_SLT;  
            3'b011: alu_op = ALU_SLTU; 
            3'b100: alu_op = ALU_XOR;  
            3'b101: begin
                if (IMMW_inst) begin
                    alu_op = (funct7[5] ? ALU_SRAW : ALU_SRLW);
                end else begin
                    alu_op = (funct7[5] ? ALU_SRA : ALU_SRL);
                end
            end
            3'b110: alu_op = ALU_OR;  
            3'b111: alu_op = ALU_AND;  
        endcase
    end else if (BRANCH_inst | JAL_inst | JALR_inst) begin
        alu_op = ALU_ADD; 
    end 
end

always_comb begin
    if (BRANCH_inst) begin
        case (funct3)
            3'b000: cmp_op = CMP_EQ;  
            3'b001: cmp_op = CMP_NE;   
            3'b100: cmp_op = CMP_LT;   
            3'b101: cmp_op = CMP_GE;   
            3'b110: cmp_op = CMP_LTU;  
            3'b111: cmp_op = CMP_GEU;  
            default: cmp_op = CMP_NO;
        endcase
    end else begin
        cmp_op = CMP_NO;
    end
end

always_comb begin
    if (AUIPC_inst | BRANCH_inst | JAL_inst) begin
        alu_asel = ASEL_PC;    
    end else if (LUI_inst) begin
        alu_asel = ASEL0;  
    end else begin
        alu_asel = ASEL_REG;   
    end
end

always_comb begin
    if (IMM_inst | IMMW_inst | LOAD_inst | STORE_inst | LUI_inst | AUIPC_inst | BRANCH_inst | JAL_inst | JALR_inst) begin
        alu_bsel = BSEL_IMM;  
    end else begin
        alu_bsel = BSEL_REG;  
    end
end

always_comb begin
    if (LOAD_inst) begin
        wb_sel = WB_SEL_MEM;      
    end else if (JAL_inst | JALR_inst) begin
        wb_sel = WB_SEL_PC;       
    end else if (IMM_inst | AUIPC_inst | IMMW_inst | STORE_inst | REG_inst | LUI_inst | REGW_inst | BRANCH_inst)begin
        wb_sel = WB_SEL_ALU;      
    end else begin
        wb_sel = WB_SEL0;
    end
end

always_comb begin
    if (LOAD_inst) begin
        case (funct3)
            3'b000: mem_op = MEM_B;   
            3'b001: mem_op = MEM_H;   
            3'b010: mem_op = MEM_W;   
            3'b011: mem_op = MEM_D;   
            3'b100: mem_op = MEM_UB;  
            3'b101: mem_op = MEM_UH;  
            3'b110: mem_op = MEM_UW;  
            default: mem_op = MEM_NO;
        endcase
    end else if (STORE_inst) begin
        case (funct3)
            3'b000: mem_op = MEM_B;  
            3'b001: mem_op = MEM_H;   
            3'b010: mem_op = MEM_W;   
            3'b011: mem_op = MEM_D;   
            default: mem_op = MEM_NO;
        endcase
    end else begin
        mem_op = MEM_NO;
    end
end


endmodule
