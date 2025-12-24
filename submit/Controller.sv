`include "core_struct.vh"
module controller (
    input CorePack::inst_t inst,
    output ControllerPack::ControllerSignals ctrl_signals
);

    import CorePack::*;
    import ControllerPack::*;
    import CsrPack::*;


wire [6:0] opcode;
wire [2:0] funct3;
wire [6:0] funct7;
wire [11:0] csr_addr_imm;


assign opcode = inst[6:0];
assign funct3 = inst[14:12];
assign funct7 = inst[31:25];
assign csr_addr_imm = inst[31:20];

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
wire SYSTEM_inst = opcode == 7'b1110011;

wire csr_inst = SYSTEM_inst && (funct3 != 3'b000);
wire reg_wen = IMM_inst | LOAD_inst | IMMW_inst | REG_inst | REGW_inst | JAL_inst | JALR_inst | AUIPC_inst | LUI_inst | csr_inst;
wire mem_wen = STORE_inst;
wire mem_ren = LOAD_inst;
wire sel_npc = JAL_inst | JALR_inst ;

always_comb begin
    // Default values
    ctrl_signals.we_reg = reg_wen;
    ctrl_signals.we_mem = mem_wen;
    ctrl_signals.re_mem = mem_ren;
    ctrl_signals.npc_sel = sel_npc;
    
    ctrl_signals.rd = inst[11:7];
    ctrl_signals.rs1 = inst[19:15];
    ctrl_signals.rs2 = inst[24:20];

    // ImmGen Op
    if (IMM_inst | IMMW_inst | LOAD_inst | JALR_inst) begin
        ctrl_signals.immgen_op = I_IMM;     
    end else if (STORE_inst) begin
        ctrl_signals.immgen_op = S_IMM;     
    end else if (BRANCH_inst) begin
        ctrl_signals.immgen_op = B_IMM;     
    end else if (LUI_inst | AUIPC_inst) begin
        ctrl_signals.immgen_op = U_IMM;     
    end else if (JAL_inst) begin
        ctrl_signals.immgen_op = UJ_IMM;    
    end else if (SYSTEM_inst) begin
        ctrl_signals.immgen_op = CSR_IMM;
    end else begin
        ctrl_signals.immgen_op = IMM0;      
    end

    // ALU Op
    ctrl_signals.alu_op = ALU_ADD;
    if (REG_inst | REGW_inst) begin
        case (funct3)
            3'b000: begin
                if (REGW_inst) begin
                    ctrl_signals.alu_op = (funct7[5] ? ALU_SUBW : ALU_ADDW);
                end else begin
                    ctrl_signals.alu_op = (funct7[5] ? ALU_SUB : ALU_ADD);
                end
            end
            3'b001: ctrl_signals.alu_op = REGW_inst ? ALU_SLLW : ALU_SLL;  
            3'b010: ctrl_signals.alu_op = ALU_SLT;  
            3'b011: ctrl_signals.alu_op = ALU_SLTU; 
            3'b100: ctrl_signals.alu_op = ALU_XOR;  
            3'b101: begin
                if (REGW_inst) begin
                    ctrl_signals.alu_op = (funct7[5] ? ALU_SRAW : ALU_SRLW);
                end else begin
                    ctrl_signals.alu_op = (funct7[5] ? ALU_SRA : ALU_SRL);
                end
            end
            3'b110: ctrl_signals.alu_op = ALU_OR;   
            3'b111: ctrl_signals.alu_op = ALU_AND;  
        endcase
    end else if (IMM_inst | IMMW_inst) begin
        case (funct3)
            3'b000: ctrl_signals.alu_op = IMMW_inst ? ALU_ADDW : ALU_ADD;  
            3'b001: ctrl_signals.alu_op = IMMW_inst ? ALU_SLLW : ALU_SLL;  
            3'b010: ctrl_signals.alu_op = ALU_SLT;  
            3'b011: ctrl_signals.alu_op = ALU_SLTU; 
            3'b100: ctrl_signals.alu_op = ALU_XOR;  
            3'b101: begin
                if (IMMW_inst) begin
                    ctrl_signals.alu_op = (funct7[5] ? ALU_SRAW : ALU_SRLW);
                end else begin
                    ctrl_signals.alu_op = (funct7[5] ? ALU_SRA : ALU_SRL);
                end
            end
            3'b110: ctrl_signals.alu_op = ALU_OR;  
            3'b111: ctrl_signals.alu_op = ALU_AND;  
        endcase
    end else if (BRANCH_inst | JAL_inst | JALR_inst | SYSTEM_inst) begin
        ctrl_signals.alu_op = ALU_ADD; 
    end 

    // CMP Op
    if (BRANCH_inst) begin
        case (funct3)
            3'b000: ctrl_signals.cmp_op = CMP_EQ;  
            3'b001: ctrl_signals.cmp_op = CMP_NE;   
            3'b100: ctrl_signals.cmp_op = CMP_LT;   
            3'b101: ctrl_signals.cmp_op = CMP_GE;   
            3'b110: ctrl_signals.cmp_op = CMP_LTU;  
            3'b111: ctrl_signals.cmp_op = CMP_GEU;  
            default: ctrl_signals.cmp_op = CMP_NO;
        endcase
    end else begin
        ctrl_signals.cmp_op = CMP_NO;
    end

    // ALU A Sel
    if (AUIPC_inst | BRANCH_inst | JAL_inst) begin
        ctrl_signals.alu_a_sel = ASEL_PC;    
    end else if (LUI_inst) begin
        ctrl_signals.alu_a_sel = ASEL0;  
    end else if (SYSTEM_inst) begin
        ctrl_signals.alu_a_sel = ASEL_CSR;
    end else begin
        ctrl_signals.alu_a_sel = ASEL_REG;   
    end

    // ALU B Sel
    if (IMM_inst | IMMW_inst | LOAD_inst | STORE_inst | LUI_inst | AUIPC_inst | BRANCH_inst | JAL_inst | JALR_inst) begin
        ctrl_signals.alu_b_sel = BSEL_IMM;  
    end else if (SYSTEM_inst) begin
        ctrl_signals.alu_b_sel = BSEL0;
    end else begin
        ctrl_signals.alu_b_sel = BSEL_REG;  
    end

    // WB Sel
    if (LOAD_inst) begin
        ctrl_signals.wb_sel = WB_SEL_MEM;      
    end else if (JAL_inst | JALR_inst) begin
        ctrl_signals.wb_sel = WB_SEL_PC;       
    end else if (IMM_inst | AUIPC_inst | IMMW_inst | STORE_inst | REG_inst | LUI_inst | REGW_inst | BRANCH_inst | SYSTEM_inst)begin
        ctrl_signals.wb_sel = WB_SEL_ALU;      
    end else begin
        ctrl_signals.wb_sel = WB_SEL0;
    end

    // Mem Op
    if (LOAD_inst) begin
        case (funct3)
            3'b000: ctrl_signals.mem_op = MEM_B;   
            3'b001: ctrl_signals.mem_op = MEM_H;   
            3'b010: ctrl_signals.mem_op = MEM_W;   
            3'b011: ctrl_signals.mem_op = MEM_D;   
            3'b100: ctrl_signals.mem_op = MEM_UB;  
            3'b101: ctrl_signals.mem_op = MEM_UH;  
            3'b110: ctrl_signals.mem_op = MEM_UW;  
            default: ctrl_signals.mem_op = MEM_NO;
        endcase
    end else if (STORE_inst) begin
        case (funct3)
            3'b000: ctrl_signals.mem_op = MEM_B;  
            3'b001: ctrl_signals.mem_op = MEM_H;   
            3'b010: ctrl_signals.mem_op = MEM_W;   
            3'b011: ctrl_signals.mem_op = MEM_D;   
            default: ctrl_signals.mem_op = MEM_NO;
        endcase
    end else begin
        ctrl_signals.mem_op = MEM_NO;
    end

    // CSR Logic
    ctrl_signals.csr_addr = csr_addr_imm;
    ctrl_signals.we_csr = 1'b0;
    ctrl_signals.csr_alu_op = CSR_ALU_ADD;
    ctrl_signals.csr_alu_asel = ASEL_CSR0;
    ctrl_signals.csr_alu_bsel = BSEL_CSR0;
    ctrl_signals.csr_ret = 2'b00;

    if (SYSTEM_inst) begin
        if (funct3 == 3'b000) begin
            if (csr_addr_imm == 12'h302) ctrl_signals.csr_ret = 2'b10; // mret
            else if (csr_addr_imm == 12'h102) ctrl_signals.csr_ret = 2'b01; // sret
        end else begin
            // CSR Instructions
            case (funct3[1:0])
                2'b01: begin // RW
                    ctrl_signals.we_csr = 1'b1;
                    ctrl_signals.csr_alu_op = CSR_ALU_ADD; 
                    ctrl_signals.csr_alu_asel = ASEL_CSR0;
                end
                2'b10: begin // RS
                    ctrl_signals.we_csr = 1'b1;//(ctrl_signals.rs1 != 0);
                    ctrl_signals.csr_alu_op = CSR_ALU_OR; 
                    ctrl_signals.csr_alu_asel = ASEL_CSRREG;
                end
                2'b11: begin // RC
                    ctrl_signals.we_csr = (ctrl_signals.rs1 != 0);
                    ctrl_signals.csr_alu_op = CSR_ALU_ANDNOT; 
                    ctrl_signals.csr_alu_asel = ASEL_CSRREG;
                end
                default: ;
            endcase
            
            if (funct3[2]) begin // Immediate
                ctrl_signals.csr_alu_bsel = BSEL_CSRIMM;
            end else begin
                ctrl_signals.csr_alu_bsel = BSEL_GPREG;
            end
        end
    end
end

endmodule

