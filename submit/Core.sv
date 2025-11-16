`include "core_struct.vh"
module Core (
    input clk,
    input rst,

    Mem_ift.Master imem_ift,
    Mem_ift.Master dmem_ift,

    output cosim_valid,
    output CorePack::CoreInfo cosim_core_info
);
    import CorePack::*;
    import PipelinePack::*;

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
logic id_we_reg, id_we_mem, id_re_mem;
logic [4:0] id_rd; 
logic [63:0] wb_wb_val; 
logic exe_br_taken;
logic [63:0] exe_alu_a, exe_alu_b;              
alu_op_enum id_alu_op;
mem_op_enum id_mem_op;
cmp_op_enum id_cmp_op;    
wb_sel_op_enum id_wb_sel;
logic [63:0] mem_dmem_rdata, exe_dmem_wdata, mem_data_trunc;
mask_t mem_dmem_wmask;
logic [63:0] id_imm;
imm_op_enum id_immgen_op;
logic id_npc_sel;
alu_asel_op_enum id_alu_asel;
alu_bsel_op_enum id_alu_bsel; 

logic flush;
assign flush = id_exe.npc_sel | exe_br_taken;

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
    end else begin
        cur_state <= next_state;
    end
end

always_comb begin
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
                if_stall        = 1'b0;
            end
            IF1: begin
                if_stall        = 1'b1;   
                imem_ift.r_request_valid = 1'b1;  
            end
            IF2: begin
                imem_ift.r_reply_ready   = 1'b1;   
            end
            WAITFOR1: begin
                if_stall        = 1'b1;   
                mem_stall       = 1'b1;   
                imem_ift.r_request_valid = 1'b1;   
            end
            WAITFOR2: begin
                mem_stall       = 1'b1;   
                imem_ift.r_reply_ready   = 1'b1;   
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


assign forwarding.exe_alu_res = exe_alu_res;
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
    end else if (flush) begin
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
    end else if (imem_ift.r_reply_valid & imem_ift.r_reply_ready) begin
        if_id <= next_if_id;
    end
end

assign id_rd = id_inst[11:7];
assign id_rs1 = id_inst[19:15];
assign id_rs2 = id_inst[24:20];

controller controller(
    .inst(id_inst),
    .we_reg(id_we_reg),
    .we_mem(id_we_mem),
    .re_mem(id_re_mem),
    .npc_sel(id_npc_sel),
    .immgen_op(id_immgen_op),
    .alu_op(id_alu_op),
    .cmp_op(id_cmp_op),
    .alu_asel(id_alu_asel),
    .alu_bsel(id_alu_bsel),
    .wb_sel(id_wb_sel),
    .mem_op(id_mem_op)
);

RegFile reg_file(
  .clk(clk),
  .rst(rst),
  .we(mem_wb.we_reg),
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

always_comb begin
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
    
end

// ID/EX：分支优先清空；mem_stall冻结；正常前进/插泡
always_ff @(posedge clk)begin
    if (rst) begin
        id_exe <= '{default: '0, alu_op: ALU_DEFAULT, cmp_op: CMP_NO, alu_a_sel: ASEL0, alu_b_sel: BSEL0, wb_sel: WB_SEL0, mem_op: MEM_NO};
    end else if (flush) begin
        id_exe <= '{default: '0, alu_op: ALU_DEFAULT, cmp_op: CMP_NO, alu_a_sel: ASEL0, alu_b_sel: BSEL0, wb_sel: WB_SEL0, mem_op: MEM_NO};
    end else if (!mem_stall) begin
        if (!next_id_exe.valid) begin
            id_exe <= '{default: '0, alu_op: ALU_DEFAULT, cmp_op: CMP_NO, alu_a_sel: ASEL0, alu_b_sel: BSEL0, wb_sel: WB_SEL0, mem_op: MEM_NO};
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
    next_exe_mem.alu_res = exe_alu_res;
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
end


always_ff @(posedge clk)begin
    if (rst) begin
        exe_mem <= '{default: '0, wb_sel: WB_SEL0, mem_op: MEM_NO};
    end else if (!mem_stall) begin 
        if (!next_exe_mem.valid) begin
            exe_mem <= '{default: '0, wb_sel: WB_SEL0, mem_op: MEM_NO};
        end else begin
            exe_mem <= next_exe_mem;
        end
    end else if ( (exe_mem.re_mem ? (dmem_ift.r_reply_valid & dmem_ift.r_reply_ready)
                                   : (dmem_ift.w_reply_valid & dmem_ift.w_reply_ready)) ) begin
        exe_mem <= '{default: '0, wb_sel: WB_SEL0, mem_op: MEM_NO};
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
    if (mem_stall) begin
        if (exe_mem.re_mem ? (dmem_ift.r_reply_valid & dmem_ift.r_reply_ready)
                           : (dmem_ift.w_reply_valid & dmem_ift.w_reply_ready)) begin
            next_mem_wb.valid = exe_mem.valid;
        end else begin
            next_mem_wb.valid = 1'b0;
        end
    end else begin
        next_mem_wb.valid = exe_mem.valid;
    end
end

always_ff @(posedge clk)begin
    if (rst) begin
        mem_wb <= '{default: '0, wb_sel: WB_SEL0};
    end else if (!next_mem_wb.valid) begin
        mem_wb <= '{default: '0, wb_sel: WB_SEL0};
    end else begin
        mem_wb <= next_mem_wb;
    end
    
end

    assign cosim_valid = mem_wb.valid;
    assign cosim_core_info.pc        = mem_wb.pc;
    assign cosim_core_info.inst      = {32'b0,mem_wb.inst};   
    assign cosim_core_info.rs1_id    = {59'b0,mem_wb.rs1};
    assign cosim_core_info.rs1_data  = mem_wb.read_data_1;
    assign cosim_core_info.rs2_id    = {59'b0,mem_wb.rs2};
    assign cosim_core_info.rs2_data  = mem_wb.read_data_2;
    assign cosim_core_info.alu       = mem_wb.alu_res;
    assign cosim_core_info.mem_addr  = mem_wb.mem_addr;
    assign cosim_core_info.mem_we    = {63'b0, (mem_wb.inst[6:0] == 7'b0100011)};
    assign cosim_core_info.mem_wdata = mem_wb.mem_wdata;
    assign cosim_core_info.mem_rdata = mem_wb.mem_rdata;
    assign cosim_core_info.rd_we     = {63'b0, mem_wb.we_reg};
    assign cosim_core_info.rd_id     = {59'b0, mem_wb.rd}; 
    assign cosim_core_info.rd_data   = wb_wb_val;
    assign cosim_core_info.br_taken  = {63'b0, mem_wb.br_taken};
    assign cosim_core_info.npc       = mem_wb.npc;

endmodule