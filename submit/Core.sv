`include "core_struct.vh"
module Core (
    input clk,
    input rst,
    input time_int,
    
    Mem_ift.Master imem_ift,
    Mem_ift.Master dmem_ift,

    output cosim_valid,
    output CorePack::CoreInfo cosim_core_info,
    output CsrPack::CSRPack cosim_csr_info,
    output cosim_interrupt,
    output cosim_switch_mode,
    output CorePack::data_t cosim_cause
);
    import CorePack::*;
    import PipelinePack::*;
    import CsrPack::*; // Removed to avoid conflict with CsrPack

typedef struct {
    data_t exe_alu_res;
    data_t mem_alu_res;
    data_t wb_wb_val;
    data_t wb_mem_rdata;
} FORWARDING;

typedef enum logic [2:0] {
    IDLE,
    IF1,
    IF2,
    WAITFOR1,
    WAITFOR2,
    MEM1,
    MEM2
} state_enum;

logic [63:0] pc, next_pc;   
logic [63:0] inst_64;
logic [31:0] id_inst;    
logic [4:0] id_rs1, id_rs2;  
logic [63:0] id_read_data_1, id_read_data_2;
logic [63:0] id_reg_data_1, id_reg_data_2;
logic [63:0] exe_alu_res;
logic [63:0] exe_final_res;
logic id_we_reg, id_we_mem, id_re_mem;
logic [4:0] id_rd; 
logic [63:0] wb_wb_val; 
logic exe_br_taken;
logic [63:0] exe_alu_a, exe_alu_b;              
alu_op_enum id_alu_op;
mem_op_enum id_mem_op;
cmp_op_enum id_cmp_op;    
wb_sel_op_enum id_wb_sel, ctrl_wb_sel;
logic [63:0] mem_dmem_rdata, exe_dmem_wdata, mem_data_trunc;
mask_t mem_dmem_wmask;
logic [63:0] id_imm;
imm_op_enum id_immgen_op;
logic id_npc_sel;
alu_asel_op_enum id_alu_asel;
alu_bsel_op_enum id_alu_bsel; 

logic [1:0] priv_mode;
logic switch_mode;
logic [63:0] pc_csr;
logic ctrl_we_reg;

logic [11:0] id_csr_addr;
logic [63:0] id_csr_read_val;
logic [63:0] id_csr_read_val_raw;
logic [63:0] time_csr_counter;
logic id_is_system;
logic id_is_csr;
logic id_is_mret, id_is_sret;
logic [2:0] id_funct3;

csr_alu_op_enmu id_csr_alu_op;
csr_alu_asel_op_enum id_csr_alu_asel;
csr_alu_bsel_op_enum id_csr_alu_bsel;
logic id_we_csr;
logic [63:0] exe_csr_alu_res;

ExceptPack id_except_out;
ExceptPack except_id_in;

logic except_happen_id;
logic trap_pending;
logic trap_req_id;
logic id_accept;

ExceptPack exe_mem_except, next_exe_mem_except;
ExceptPack mem_wb_except, next_mem_wb_except;

logic flush;
assign flush = id_exe.npc_sel | exe_br_taken | switch_mode;
assign cosim_switch_mode = cosim_csr_info.switch_mode[0];

IFID if_id, next_if_id;
IDEXE id_exe, next_id_exe;
EXEMEM exe_mem, next_exe_mem;
MEMWB mem_wb, next_mem_wb;
FORWARDING forwarding;

logic if_stall, mem_stall;

state_enum cur_state, next_state;

always_ff @(posedge clk) begin
    if (rst) begin
        cur_state <= IF1;
    end else if (switch_mode) begin
        cur_state <= IF1;
    end else begin
        cur_state <= next_state;
    end
end

always_comb begin
    if (trap_pending) begin
        case (cur_state)
            MEM1: begin
                if (exe_mem.re_mem ? dmem_ift.r_request_ready : dmem_ift.w_request_ready) begin
                    next_state = MEM2;
                end else begin
                    next_state = MEM1;
                end
            end
            MEM2: begin
                if (exe_mem.re_mem ? dmem_ift.r_reply_valid : dmem_ift.w_reply_valid) begin
                    next_state = IDLE;
                end else begin
                    next_state = MEM2;
                end
            end
            default: begin
                next_state = IDLE;
            end
        endcase
    end else begin
        case (cur_state)
        IDLE: begin
            next_state = IF1;
        end
        IF1: begin
            if (imem_ift.r_request_ready) begin
                next_state = IF2;
            end else if ((id_exe.re_mem & id_exe.valid) | (id_exe.we_mem & id_exe.valid)) begin
                next_state = WAITFOR1;
            end else begin
                next_state = IF1;
            end
        end
        IF2: begin
            if (imem_ift.r_reply_valid) begin
                next_state = IDLE;
            end else begin
                next_state = IF2;
            end
        end
        WAITFOR1: begin
            if (imem_ift.r_request_ready) begin
                next_state = WAITFOR2;
            end else begin
                next_state = WAITFOR1;
            end
        end
        WAITFOR2: begin
            if (imem_ift.r_reply_valid) begin
                next_state = MEM1;
            end else begin
                next_state = WAITFOR2;
            end 
        end
        MEM1: begin
            if (exe_mem.re_mem ? dmem_ift.r_request_ready : dmem_ift.w_request_ready) begin
                next_state = MEM2;
            end else begin
                next_state = MEM1;
            end
        end
        MEM2: begin
            if (exe_mem.re_mem ? dmem_ift.r_reply_valid : dmem_ift.w_reply_valid) begin
                next_state = IDLE;
            end else begin
                next_state = MEM2;
            end
        end
        default: begin
            next_state = IDLE;
        end
        endcase
    end
end

always_comb begin
    if_stall        = 1'b1; 
    mem_stall       = 1'b0; 
    imem_ift.r_request_valid = 1'b0; 
    imem_ift.r_reply_ready   = 1'b0; 
    dmem_ift.r_request_valid = 1'b0; 
    dmem_ift.r_reply_ready   = 1'b0; 
    dmem_ift.w_request_valid = 1'b0; 
    dmem_ift.w_reply_ready   = 1'b0;
        case (cur_state)
            IDLE: begin
                if_stall        = trap_pending ? 1'b1 : 1'b0;
            end
            IF1: begin
                if_stall        = 1'b1;   
                imem_ift.r_request_valid = trap_pending ? 1'b0 : 1'b1;
                imem_ift.r_reply_ready   = trap_pending ? 1'b0 : 1'b1;
            end
            IF2: begin
                imem_ift.r_reply_ready   = trap_pending ? 1'b0 : 1'b1;   
            end
            WAITFOR1: begin
                if_stall        = 1'b1;   
                mem_stall       = 1'b1;   
                imem_ift.r_request_valid = trap_pending ? 1'b0 : 1'b1;
                imem_ift.r_reply_ready   = trap_pending ? 1'b0 : 1'b1;
            end
            WAITFOR2: begin
                mem_stall       = 1'b1;   
                imem_ift.r_reply_ready   = trap_pending ? 1'b0 : 1'b1;   
            end
            MEM1: begin
                mem_stall       = 1'b1;
                dmem_ift.r_request_valid = exe_mem.re_mem ? 1'b1 : 1'b0;   
                dmem_ift.w_request_valid = exe_mem.we_mem ? 1'b1 : 1'b0;
            end
            MEM2: begin
                mem_stall       = 1'b1;   
                dmem_ift.r_reply_ready   = exe_mem.re_mem ? 1'b1 : 1'b0;   
                dmem_ift.w_reply_ready   = exe_mem.we_mem ? 1'b1 : 1'b0;
            end
            default: begin
                if_stall        = 1'b1;
                mem_stall       = 1'b0;
            end
        endcase
    end


assign forwarding.exe_alu_res = exe_final_res; // Modified
assign forwarding.mem_alu_res = exe_mem.alu_res;
assign forwarding.wb_wb_val = wb_wb_val;
assign forwarding.wb_mem_rdata = mem_wb.mem_rdata;

always_comb begin 
    if (if_id.valid)begin
        id_reg_data_1 = (id_exe.valid & (id_rs1 != '0) & (id_rs1 == id_exe.rd) & id_exe.we_reg) ? forwarding.exe_alu_res :
                        (exe_mem.valid & (id_rs1 != '0) & (id_rs1 == exe_mem.rd) & exe_mem.we_reg) ? forwarding.mem_alu_res :
                        (mem_wb.valid & (id_rs1 != '0) & (id_rs1 == mem_wb.rd) & mem_wb.we_reg) ? forwarding.wb_wb_val :
                        id_read_data_1;
        id_reg_data_2 = (id_exe.valid & (id_rs2 != '0) & (id_rs2 == id_exe.rd) & id_exe.we_reg) ? forwarding.exe_alu_res :
                        (exe_mem.valid & (id_rs2 != '0) & (id_rs2 == exe_mem.rd) & exe_mem.we_reg) ? forwarding.mem_alu_res :
                        (mem_wb.valid & (id_rs2 != '0) & (id_rs2 == mem_wb.rd) & mem_wb.we_reg) ? forwarding.wb_wb_val :
                        id_read_data_2;
    end else begin 
        id_reg_data_1 = '0;
        id_reg_data_2 = '0;
    end
end

always_ff @(posedge clk) begin
    if (rst) begin
        pc <= 64'h0;
    end else if (switch_mode) begin
        pc <= pc_csr;
    end else if (trap_pending) begin
        pc <= pc;
    end else if (id_exe.npc_sel | exe_br_taken) begin
        pc <= exe_alu_res;
    end else if (!if_stall & !mem_stall) begin
        pc <= pc + 4;
    end
end

//assign next_pc = (id_exe.valid) ? ((id_exe.npc_sel | exe_br_taken) ? exe_alu_res : (pc + 4)) : (pc + 4);

assign imem_ift.r_request_bits.raddr = {pc[63:3], 3'b0};
assign inst_64 = imem_ift.r_reply_bits.rdata;

assign id_inst = if_id.inst;

always_comb begin 
    next_if_id.pc   = pc;
    next_if_id.pc_4 = pc + 4;
    next_if_id.inst = pc[2] ? inst_64[63:32] : inst_64[31:0];
    if (flush) begin 
        next_if_id.valid = 1'b0;
    end else begin 
        next_if_id.valid = 1'b1;
    end
end

// IF/ID仅在IMEM回复握手达成时写入；分支优先清空
always_ff @(posedge clk) begin
    if (rst) begin
        if_id <= '{default: '0};
    end else if (flush) begin
        if_id <= '{default: '0};
    end else if (trap_req_id & if_id.valid & id_accept) begin
        if_id <= '{default: '0};
    end else if (imem_ift.r_reply_valid & imem_ift.r_reply_ready) begin
        if_id <= next_if_id;
    end
end

assign id_rd = id_inst[11:7];
assign id_rs1 = id_inst[19:15];
assign id_rs2 = id_inst[24:20];

ControllerPack::ControllerSignals ctrl_signals;

controller controller(
    .inst(id_inst),
    .ctrl_signals(ctrl_signals)
);

assign ctrl_we_reg = ctrl_signals.we_reg;
assign id_we_mem = ctrl_signals.we_mem;
assign id_re_mem = ctrl_signals.re_mem;
assign id_npc_sel = ctrl_signals.npc_sel;
assign id_immgen_op = ctrl_signals.immgen_op;
assign id_alu_op = ctrl_signals.alu_op;
assign id_cmp_op = ctrl_signals.cmp_op;
assign id_alu_asel = ctrl_signals.alu_a_sel;
assign id_alu_bsel = ctrl_signals.alu_b_sel;
assign ctrl_wb_sel = ctrl_signals.wb_sel;
assign id_mem_op = ctrl_signals.mem_op;

assign id_we_reg = ctrl_signals.we_reg;
assign id_wb_sel = ctrl_signals.wb_sel;

assign id_csr_addr = ctrl_signals.csr_addr;
assign id_we_csr = ctrl_signals.we_csr;
assign id_csr_alu_op = ctrl_signals.csr_alu_op;
assign id_csr_alu_asel = ctrl_signals.csr_alu_asel;
assign id_csr_alu_bsel = ctrl_signals.csr_alu_bsel;

assign id_is_mret = ctrl_signals.csr_ret[1];
assign id_is_sret = ctrl_signals.csr_ret[0];

assign id_is_system = (id_inst[6:0] == 7'b1110011);
assign id_funct3 = id_inst[14:12];
assign id_is_csr = id_is_system && (id_funct3 != 0);

assign trap_req_id = except_happen_id | except_id_in.except | id_is_mret | id_is_sret;
assign id_accept = !rst & !mem_stall & !flush & !if_stall;

always_ff @(posedge clk) begin
    if (rst) begin
        trap_pending <= 1'b0;
    end else if (switch_mode) begin
        trap_pending <= 1'b0;
    end else if (trap_req_id & if_id.valid & id_accept) begin
        trap_pending <= 1'b1;
    end
end

wire id_csr_illegal;
wire id_is_csrr_time;

// Provide a minimal `time` CSR (0xC01) for kernel timekeeping.
// We implement it as a simple cycle counter so S-mode `csrr time` does not trap.
always_ff @(posedge clk) begin
    if (rst) begin
        time_csr_counter <= 64'b0;
    end else begin
        time_csr_counter <= time_csr_counter + 64'd1;
    end
end

assign id_is_csrr_time = id_is_csr && (id_csr_addr == 12'hC01);
assign id_csr_illegal = 1'b0;

CSRModule csr_module (
    .clk(clk),
    .rst(rst),
    .csr_we_wb(mem_wb.we_csr & mem_wb.valid),
    .csr_addr_wb(mem_wb.csr_addr),
    .csr_val_wb(mem_wb.csr_alu_res),
    .csr_addr_id(id_csr_addr),
    .csr_val_id(id_csr_read_val_raw),
    .pc_ret(mem_wb.npc),
    .valid_wb(mem_wb.valid),
    .time_int(time_int),
    .csr_ret(mem_wb.csr_ret),
    .except_commit(mem_wb_except),
    .priv(priv_mode),
    .switch_mode(switch_mode),
    .pc_csr(pc_csr),
    .cosim_interrupt(cosim_interrupt),
    .cosim_cause(cosim_cause),
    .cosim_csr_info(cosim_csr_info)
);

// Override CSR read data for `time`.
always_comb begin
    id_csr_read_val = id_csr_read_val_raw;
    if (id_is_csrr_time) begin
        id_csr_read_val = time_csr_counter;
    end
end

always_comb begin
    except_id_in = '{default: '0};
    // No extra injected exceptions here; CSR/illegal detection is handled by IDExceptExamine.
end

IDExceptExamine id_except_examine(
    .clk(clk),
    .rst(rst),
    .stall(mem_stall),
    .flush(flush),
    .pc_id(if_id.pc),
    .priv(priv_mode),
    .inst_id(id_inst),
    .valid_id(if_id.valid),
    .except_id(except_id_in),
    .except_exe(id_except_out),
    .except_happen_id(except_happen_id)
);

RegFile reg_file(
  .clk(clk),
  .rst(rst),
    .we(mem_wb.we_reg & mem_wb.valid & ~mem_wb_except.except),
  .read_addr_1(id_rs1),
  .read_addr_2(id_rs2),
  .write_addr(mem_wb.rd),
  .write_data(wb_wb_val),
  .read_data_1(id_read_data_1),
  .read_data_2(id_read_data_2)
);

Immgen imm_gen(
    .immgen_op(id_immgen_op),
    .inst(id_inst[31:7]),
    .imm(id_imm)
);

logic [63:0] id_csr_val_forwarded;

always_comb begin
    // CSR Forwarding Logic
    id_csr_val_forwarded = id_csr_read_val;
    if (mem_wb.valid & mem_wb.we_csr & (mem_wb.csr_addr == id_csr_addr)) begin
        id_csr_val_forwarded = mem_wb.csr_alu_res;
    end
    if (exe_mem.valid & exe_mem.we_csr & (exe_mem.csr_addr == id_csr_addr)) begin
        id_csr_val_forwarded = exe_mem.csr_alu_res;
    end
    if (id_exe.valid & id_exe.we_csr & (id_exe.csr_addr == id_csr_addr)) begin
        id_csr_val_forwarded = exe_csr_alu_res;
    end

    next_id_exe.we_reg = id_we_reg;
    next_id_exe.we_mem = id_we_mem;
    next_id_exe.re_mem = id_re_mem;
    next_id_exe.npc_sel = id_npc_sel;
    next_id_exe.imm = id_imm;
    next_id_exe.alu_op = id_alu_op;
    next_id_exe.cmp_op = id_cmp_op;
    next_id_exe.alu_a_sel = id_alu_asel;
    next_id_exe.alu_b_sel = id_alu_bsel;
    next_id_exe.reg_data_1 = id_reg_data_1;
    next_id_exe.reg_data_2 = id_reg_data_2;
    next_id_exe.wb_sel = id_wb_sel;
    next_id_exe.mem_op = id_mem_op;

    next_id_exe.rd = id_rd;
    next_id_exe.rs1 = id_rs1;
    next_id_exe.rs2 = id_rs2;

    next_id_exe.pc = if_id.pc;
    next_id_exe.pc_4 = if_id.pc_4;
    next_id_exe.inst = id_inst;
    if (flush | if_stall) begin
        next_id_exe.valid = 1'b0;
    end else begin 
        next_id_exe.valid = if_id.valid;
    end
    
    // --- ID/EXE Pipeline Logic for New Signals ---
    next_id_exe.we_csr = id_we_csr;
    next_id_exe.csr_ret = {id_is_mret, id_is_sret};
    next_id_exe.csr_alu_op = id_csr_alu_op;
    next_id_exe.csr_alu_asel = id_csr_alu_asel;
    next_id_exe.csr_alu_bsel = id_csr_alu_bsel;
    next_id_exe.csr_val = id_csr_val_forwarded;
    next_id_exe.csr_addr = id_csr_addr;
end

// ID/EX：分支优先清空；mem_stall冻结；正常前进/插泡
always_ff @(posedge clk)begin
    if (rst) begin
        id_exe <= '{default: '0, alu_op: ALU_DEFAULT, cmp_op: CMP_NO, alu_a_sel: ASEL0, alu_b_sel: BSEL0, wb_sel: WB_SEL0, mem_op: MEM_NO, csr_alu_op: CSR_ALU_ADD, csr_alu_asel: ASEL_CSR0, csr_alu_bsel: BSEL_CSR0};
    end else if (flush) begin
        id_exe <= '{default: '0, alu_op: ALU_DEFAULT, cmp_op: CMP_NO, alu_a_sel: ASEL0, alu_b_sel: BSEL0, wb_sel: WB_SEL0, mem_op: MEM_NO, csr_alu_op: CSR_ALU_ADD, csr_alu_asel: ASEL_CSR0, csr_alu_bsel: BSEL_CSR0};
    end else if (!mem_stall) begin
        if (!next_id_exe.valid) begin
            id_exe <= '{default: '0, alu_op: ALU_DEFAULT, cmp_op: CMP_NO, alu_a_sel: ASEL0, alu_b_sel: BSEL0, wb_sel: WB_SEL0, mem_op: MEM_NO, csr_alu_op: CSR_ALU_ADD, csr_alu_asel: ASEL_CSR0, csr_alu_bsel: BSEL_CSR0};
        end else begin
            id_exe <= next_id_exe;
        end
    end
    // else: mem_stall 冻结
end

always_comb begin
    case (id_exe.alu_a_sel)
        ASEL0: begin
            exe_alu_a = '0;
        end
        ASEL_REG: begin
            exe_alu_a = id_exe.reg_data_1;
        end
        ASEL_PC: begin
            exe_alu_a = id_exe.pc;
        end
        default: begin
            exe_alu_a = 64'b0;
        end
    endcase
end

always_comb begin
    case (id_exe.alu_b_sel)
        BSEL0: begin
            exe_alu_b = '0;
        end
        BSEL_REG: begin
            exe_alu_b = id_exe.reg_data_2;
        end
        BSEL_IMM: begin
            exe_alu_b = id_exe.imm;
        end
        default: begin
            exe_alu_b = 64'b0;
        end
    endcase
end

ALU alu(
  .a(exe_alu_a),
  .b(exe_alu_b),
  .alu_op(id_exe.alu_op),
  .res(exe_alu_res)
);

// --- EXE Stage CSR ALU ---
logic [63:0] exe_csr_op_a, exe_csr_op_b;
// logic [63:0] exe_csr_alu_res; // Moved to top
logic exe_is_csr;

assign exe_is_csr = (id_exe.inst[6:0] == 7'b1110011) && (id_exe.inst[14:12] != 0);

always_comb begin
    case (id_exe.csr_alu_asel)
        ASEL_CSR0: exe_csr_op_a = '0;
        ASEL_CSRREG: exe_csr_op_a = id_exe.csr_val;
        default: exe_csr_op_a = '0;
    endcase

    case (id_exe.csr_alu_bsel)
        BSEL_CSR0: exe_csr_op_b = '0;
        BSEL_GPREG: exe_csr_op_b = id_exe.reg_data_1;
        BSEL_CSRIMM: exe_csr_op_b = {59'b0, id_exe.rs1}; // zext(rs1)
        default: exe_csr_op_b = '0;
    endcase

    case (id_exe.csr_alu_op)
        CSR_ALU_ADD: exe_csr_alu_res = exe_csr_op_a + exe_csr_op_b; 
        CSR_ALU_OR:  exe_csr_alu_res = exe_csr_op_a | exe_csr_op_b;
        CSR_ALU_ANDNOT: exe_csr_alu_res = exe_csr_op_a & (~exe_csr_op_b);
        default: exe_csr_alu_res = '0;
    endcase
end

assign exe_final_res = exe_is_csr ? id_exe.csr_val : exe_alu_res;

Cmp cmp(
    .a(id_exe.reg_data_1),
    .b(id_exe.reg_data_2),
    .cmp_op(id_exe.cmp_op),
    .cmp_res(exe_br_taken)
);

DataPkg data_pkg(
    .mem_op(id_exe.mem_op),
    .reg_data(id_exe.reg_data_2),
    .dmem_waddr(exe_alu_res),
    .dmem_wdata(exe_dmem_wdata)
);


MaskGen mask_gen(
    .mem_op(exe_mem.mem_op),
    .dmem_waddr(exe_mem.alu_res),
    .dmem_wmask(mem_dmem_wmask)
);


always_comb begin
    next_exe_mem.we_reg = id_exe.we_reg;
    next_exe_mem.we_mem = id_exe.we_mem;
    next_exe_mem.re_mem = id_exe.re_mem;
    next_exe_mem.br_taken = exe_br_taken;
    next_exe_mem.alu_res = exe_final_res; // Modified
    next_exe_mem.reg_data_1 = id_exe.reg_data_1;
    next_exe_mem.reg_data_2 = id_exe.reg_data_2;
    next_exe_mem.mem_wdata = exe_dmem_wdata;

    next_exe_mem.wb_sel = id_exe.wb_sel;
    next_exe_mem.mem_op = id_exe.mem_op;

    next_exe_mem.rd = id_exe.rd;
    next_exe_mem.rs1 = id_exe.rs1;
    next_exe_mem.rs2 = id_exe.rs2;

    next_exe_mem.pc = id_exe.pc;
    next_exe_mem.pc_4 = id_exe.pc_4;
    next_exe_mem.npc = (id_exe.npc_sel | exe_br_taken) ? exe_alu_res : id_exe.pc_4;
    next_exe_mem.inst = id_exe.inst;
    next_exe_mem.valid = id_exe.valid;

    // --- EXE/MEM Pipeline Logic for New Signals ---
    next_exe_mem.we_csr = id_exe.we_csr;
    next_exe_mem.csr_ret = id_exe.csr_ret;
    next_exe_mem.csr_alu_res = exe_csr_alu_res;
    next_exe_mem.csr_addr = id_exe.csr_addr;
    
    next_exe_mem_except = id_except_out;
end


always_ff @(posedge clk)begin
    if (rst) begin
        exe_mem <= '{default: '0, wb_sel: WB_SEL0, mem_op: MEM_NO};
        exe_mem_except <= '{default: '0};
    end else if (switch_mode) begin
        exe_mem <= '{default: '0, wb_sel: WB_SEL0, mem_op: MEM_NO};
        exe_mem_except <= '{default: '0};
    end else if (!mem_stall) begin 
        if (!next_exe_mem.valid) begin
            exe_mem <= '{default: '0, wb_sel: WB_SEL0, mem_op: MEM_NO};
            exe_mem_except <= '{default: '0};
        end else begin
            exe_mem <= next_exe_mem;
            exe_mem_except <= next_exe_mem_except;
        end
    end else if ( (exe_mem.re_mem ? (dmem_ift.r_reply_valid & dmem_ift.r_reply_ready)
                                   : (dmem_ift.w_reply_valid & dmem_ift.w_reply_ready)) ) begin
        exe_mem <= '{default: '0, wb_sel: WB_SEL0, mem_op: MEM_NO};
        exe_mem_except <= '{default: '0};
    end
end

assign dmem_ift.w_request_bits.waddr = exe_mem.alu_res;
assign dmem_ift.w_request_bits.wmask = mem_dmem_wmask;
assign dmem_ift.w_request_bits.wdata = exe_mem.mem_wdata;

assign dmem_ift.r_request_bits.raddr = exe_mem.alu_res;
assign mem_dmem_rdata = dmem_ift.r_reply_bits.rdata;

DataTrunc data_trunc(
    .dmem_rdata(mem_dmem_rdata),
    .mem_op(exe_mem.mem_op),
    .dmem_raddr(exe_mem.alu_res),
    .read_data(mem_data_trunc)
);

always_comb begin
    case (mem_wb.wb_sel)
        WB_SEL0: begin
            wb_wb_val = '0;
        end
        WB_SEL_ALU: begin
            wb_wb_val = mem_wb.alu_res;
        end
        WB_SEL_MEM: begin
            wb_wb_val = mem_wb.data_trunc;
        end
        WB_SEL_PC: begin
            wb_wb_val = mem_wb.pc_4;
        end
    endcase
end

always_comb begin
    next_mem_wb.we_reg = exe_mem.we_reg;
    next_mem_wb.re_mem = exe_mem.re_mem;
    next_mem_wb.br_taken = exe_mem.br_taken;
    next_mem_wb.alu_res = exe_mem.alu_res;
    next_mem_wb.data_trunc = mem_data_trunc;
    next_mem_wb.read_data_1 = exe_mem.reg_data_1;
    next_mem_wb.read_data_2 = exe_mem.reg_data_2;
    next_mem_wb.mem_wdata = exe_mem.mem_wdata;
    next_mem_wb.mem_rdata = mem_dmem_rdata;
    next_mem_wb.mem_addr = exe_mem.alu_res;
    next_mem_wb.wb_sel = exe_mem.wb_sel;

    next_mem_wb.rd = exe_mem.rd;
    next_mem_wb.rs1 = exe_mem.rs1;
    next_mem_wb.rs2 = exe_mem.rs2;

    next_mem_wb.pc = exe_mem.pc;
    next_mem_wb.pc_4 = exe_mem.pc_4;
    next_mem_wb.npc = exe_mem.npc;
    next_mem_wb.inst = exe_mem.inst;
    if (switch_mode) begin
        next_mem_wb.valid = 1'b0;
    end else if (mem_stall) begin
        if (exe_mem.re_mem ? (dmem_ift.r_reply_valid & dmem_ift.r_reply_ready)
                           : (dmem_ift.w_reply_valid & dmem_ift.w_reply_ready)) begin
            next_mem_wb.valid = exe_mem.valid;
        end else begin
            next_mem_wb.valid = 1'b0;
        end
    end else begin
        next_mem_wb.valid = exe_mem.valid;
    end

    // --- MEM/WB Pipeline Logic for New Signals ---
    next_mem_wb.we_csr = exe_mem.we_csr;
    next_mem_wb.csr_ret = exe_mem.csr_ret;
    next_mem_wb.csr_alu_res = exe_mem.csr_alu_res;
    next_mem_wb.csr_addr = exe_mem.csr_addr;
    
    next_mem_wb_except = exe_mem_except;
end

always_ff @(posedge clk)begin
    if (rst) begin
        mem_wb <= '{default: '0, wb_sel: WB_SEL0};
        mem_wb_except <= '{default: '0};
    end else if (!next_mem_wb.valid) begin
        mem_wb <= '{default: '0, wb_sel: WB_SEL0};
        mem_wb_except <= '{default: '0};
    end else begin
        mem_wb <= next_mem_wb;
        mem_wb_except <= next_mem_wb_except;
    end
    
end

    // cosim checks the architectural commit stream. In this lab framework, ECALLs are
    // expected to appear in the commit stream (with no GPR/mem side effects), while
    // other exception-causing instructions (e.g. illegal instruction) are not.
    wire wb_is_ecall;
    assign wb_is_ecall = mem_wb_except.except &&
                         ((mem_wb_except.ecause == U_CALL) ||
                          (mem_wb_except.ecause == S_CALL) ||
                          (mem_wb_except.ecause == H_CALL) ||
                          (mem_wb_except.ecause == M_CALL));
    wire commit_wb;
    assign commit_wb = mem_wb.valid & (~mem_wb_except.except | wb_is_ecall);
    assign cosim_valid = commit_wb;
    assign cosim_core_info.pc        = mem_wb.pc;
    assign cosim_core_info.inst      = {32'b0,mem_wb.inst};   
    assign cosim_core_info.rs1_id    = {59'b0,mem_wb.rs1};
    assign cosim_core_info.rs1_data  = mem_wb.read_data_1;
    assign cosim_core_info.rs2_id    = {59'b0,mem_wb.rs2};
    assign cosim_core_info.rs2_data  = mem_wb.read_data_2;
    assign cosim_core_info.alu       = mem_wb.alu_res;
    assign cosim_core_info.mem_addr  = mem_wb.mem_addr;
    assign cosim_core_info.mem_we    = {63'b0, ((mem_wb.inst[6:0] == 7'b0100011) & commit_wb & ~mem_wb_except.except)};
    assign cosim_core_info.mem_wdata = mem_wb.mem_wdata;
    assign cosim_core_info.mem_rdata = mem_wb.mem_rdata;
    assign cosim_core_info.rd_we     = {63'b0, (mem_wb.we_reg & commit_wb & ~mem_wb_except.except)};
    assign cosim_core_info.rd_id     = {59'b0, mem_wb.rd}; 
    assign cosim_core_info.rd_data   = wb_wb_val;
    assign cosim_core_info.br_taken  = {63'b0, mem_wb.br_taken};
    assign cosim_core_info.npc       = mem_wb.npc;



endmodule