`timescale 1us / 1us

`include "MipsDefinitions.sv"

`ifndef COMMON_H
`define COMMON_H

`define GET_REG_ADDR(RESULT, REG_ADDR_TYPE, INSTRUCTION) \
    case (REG_ADDR_TYPE) \
        REG_ADDR_TYPE_RS: RESULT = MipsReg'(INSTRUCTION.rs); \
        REG_ADDR_TYPE_RT: RESULT = MipsReg'(INSTRUCTION.rt); \
        REG_ADDR_TYPE_RD: RESULT = MipsReg'(INSTRUCTION.rd); \
        REG_ADDR_TYPE_RA: RESULT = RA; \
        REG_ADDR_TYPE_V0: RESULT = V0; \
        REG_ADDR_TYPE_A0: RESULT = A0; \
        default: RESULT = ZERO; \
    endcase

function PipelineStage get_data_ready_stage(RegWriteDataSource reg_write_data_source);
    case (reg_write_data_source)
        REG_WRITE_DATA_SOURCE_PC_PLUS_8: return PipelineStage_ID;
        REG_WRITE_DATA_SOURCE_ALU: return PipelineStage_EX;
        REG_WRITE_DATA_SOURCE_MDU: return PipelineStage_EX;
        REG_WRITE_DATA_SOURCE_MEM: return PipelineStage_MEM;
    endcase
endfunction

// 调试输出相关任务
task automatic print_pipeline_stage(
    int fd,
    string pipeline_name,
    logic [31:0] if_pc,
    logic [31:0] if_id,
    Instruction if_inst,
    logic if_valid,
    logic [31:0] id_pc,
    logic [31:0] id_id,
    Instruction id_inst,
    logic id_valid,
    logic [31:0] ex_pc,
    logic [31:0] ex_id,
    Instruction ex_inst,
    logic ex_valid,
    logic [31:0] mem_pc,
    logic [31:0] mem_id,
    Instruction mem_inst,
    logic mem_valid,
    logic [31:0] wb_pc,
    logic [31:0] wb_id,
    Instruction wb_inst,
    logic wb_valid
);
    $fdisplay(fd, "\n=== %s ===", pipeline_name);
    
    if (if_valid)
        $fdisplay(fd, "IF:  ID=%0d, PC=0x%h, Instruction=%s", if_id, if_pc, decode_instruction(if_inst.instruction));
    else
        $fdisplay(fd, "IF:  <无效>");
        
    if (id_valid)
        $fdisplay(fd, "ID:  ID=%0d, PC=0x%h, Instruction=%s", id_id, id_pc, decode_instruction(id_inst.instruction));
    else
        $fdisplay(fd, "ID:  <无效>");
        
    if (ex_valid)
        $fdisplay(fd, "EX:  ID=%0d, PC=0x%h, Instruction=%s", ex_id, ex_pc, decode_instruction(ex_inst.instruction));
    else
        $fdisplay(fd, "EX:  <无效>");
        
    if (mem_valid)
        $fdisplay(fd, "MEM: ID=%0d, PC=0x%h, Instruction=%s", mem_id, mem_pc, decode_instruction(mem_inst.instruction));
    else
        $fdisplay(fd, "MEM: <无效>");
        
    if (wb_valid)
        $fdisplay(fd, "WB:  ID=%0d, PC=0x%h, Instruction=%s", wb_id, wb_pc, decode_instruction(wb_inst.instruction));
    else
        $fdisplay(fd, "WB:  <无效>");
endtask

task automatic print_scheduling_info(
    int fd,
    logic inst0_valid,
    logic inst1_valid,
    logic can_dual_issue,
    logic has_raw_dependency,
    logic has_waw_dependency,
    logic has_war_dependency
);
    $fdisplay(fd, "\n=== Instruction Scheduling ===");
    if (inst0_valid && inst1_valid) begin
        if (can_dual_issue)
            $fdisplay(fd, "双发射：可以");
        else begin
            $fdisplay(fd, "双发射：不可以");
            if (has_raw_dependency)
                $fdisplay(fd, "  - RAW 依赖");
            if (has_waw_dependency)
                $fdisplay(fd, "  - WAW 依赖");
            if (has_war_dependency)
                $fdisplay(fd, "  - WAR 依赖");
        end
    end else begin
        $fdisplay(fd, "双发射：指令无效");
    end
endtask

task automatic print_hazard_info(
    int fd,
    string pipeline_name,
    logic hazard_rs_in_id,
    logic hazard_rt_in_id,
    logic hazard_rs_in_ex,
    logic hazard_rt_in_ex,
    logic stall_from_mdu,
    MipsReg rs_addr_id,
    MipsReg rt_addr_id,
    MipsReg rs_addr_ex,
    MipsReg rt_addr_ex
);
    $fdisplay(fd, "\n=== %s Hazard Status ===", pipeline_name);
    if (hazard_rs_in_id)
        $fdisplay(fd, "  ID stage stalled: waiting for RS(%s)", get_reg_name(rs_addr_id));
    if (hazard_rt_in_id)
        $fdisplay(fd, "  ID stage stalled: waiting for RT(%s)", get_reg_name(rt_addr_id));
    if (hazard_rs_in_ex)
        $fdisplay(fd, "  EX stage stalled: waiting for RS(%s)", get_reg_name(rs_addr_ex));
    if (hazard_rt_in_ex)
        $fdisplay(fd, "  EX stage stalled: waiting for RT(%s)", get_reg_name(rt_addr_ex));
    if (stall_from_mdu)
        $fdisplay(fd, "  EX stage stalled: waiting for MDU operation to complete");
endtask

task automatic print_forwarding_info(
    int fd,
    string pipeline_name,
    ForwardingType id_rs_type,
    ForwardingType id_rt_type,
    ForwardingType ex_rs_type,
    ForwardingType ex_rt_type,
    ForwardingType mem_rs_type,
    ForwardingType mem_rt_type,
    logic [31:0] id_rs_from,
    logic [31:0] id_rt_from,
    logic [31:0] ex_rs_from,
    logic [31:0] ex_rt_from,
    MipsReg id_rs_addr,
    MipsReg id_rt_addr,
    MipsReg ex_rs_addr,
    MipsReg ex_rt_addr,
    MipsReg mem_rs_addr,
    MipsReg mem_rt_addr,
    logic [31:0] id_rs_data,
    logic [31:0] id_rt_data,
    logic [31:0] ex_rs_data,
    logic [31:0] ex_rt_data,
    logic [31:0] mem_rs_data,
    logic [31:0] mem_rt_data,
    logic [31:0] id_rs_forwarded,
    logic [31:0] id_rt_forwarded,
    logic [31:0] ex_rs_forwarded,
    logic [31:0] ex_rt_forwarded,
    logic [31:0] mem_rs_forwarded,
    logic [31:0] mem_rt_forwarded
);
    $fdisplay(fd, "\n=== %s Forwarding Status ===", pipeline_name);
    if (id_rs_type != ForwardingType'(FORWARDING_TYPE_NONE))
        $fdisplay(fd, "  ID: RS(%s) forwarded(%s): 0x%h -> 0x%h from %0d", 
            get_reg_name(id_rs_addr), get_forwarding_type_name(id_rs_type), id_rs_data, id_rs_forwarded, id_rs_from);
    if (id_rt_type != ForwardingType'(FORWARDING_TYPE_NONE))
        $fdisplay(fd, "  ID: RT(%s) forwarded(%s): 0x%h -> 0x%h from %0d", 
            get_reg_name(id_rt_addr), get_forwarding_type_name(id_rt_type), id_rt_data, id_rt_forwarded, id_rt_from);
    if (ex_rs_type != ForwardingType'(FORWARDING_TYPE_NONE))
        $fdisplay(fd, "  EX: RS(%s) forwarded(%s): 0x%h -> 0x%h from %0d", 
            get_reg_name(ex_rs_addr), get_forwarding_type_name(ex_rs_type), ex_rs_data, ex_rs_forwarded);
    if (ex_rt_type != ForwardingType'(FORWARDING_TYPE_NONE))
        $fdisplay(fd, "  EX: RT(%s) forwarded(%s): 0x%h -> 0x%h from %0d", 
            get_reg_name(ex_rt_addr), get_forwarding_type_name(ex_rt_type), ex_rt_data, ex_rt_forwarded, ex_rt_from);
    if (mem_rs_type != ForwardingType'(FORWARDING_TYPE_NONE))
        $fdisplay(fd, "  MEM: RS(%s) forwarded(%s): 0x%h -> 0x%h", 
            get_reg_name(mem_rs_addr), get_forwarding_type_name(mem_rs_type), mem_rs_data, mem_rs_forwarded);
    if (mem_rt_type != ForwardingType'(FORWARDING_TYPE_NONE))
        $fdisplay(fd, "  MEM: RT(%s) forwarded(%s): 0x%h -> 0x%h", 
            get_reg_name(mem_rt_addr), get_forwarding_type_name(mem_rt_type), mem_rt_data, mem_rt_forwarded);
endtask

task automatic print_alu_info(
    int fd,
    string pipeline_name,
    AluOp alu_op,
    logic [31:0] operand1,
    logic [31:0] operand2,
    logic [31:0] result,
    logic overflow
);
    $fdisplay(fd, "\n=== %s ALU Operation ===", pipeline_name);
    case (alu_op)
        ALU_ADD:  $fdisplay(fd, "  Operation: ADD");
        ALU_ADDU: $fdisplay(fd, "  Operation: ADDU");
        ALU_SUB:  $fdisplay(fd, "  Operation: SUB");
        ALU_SUBU: $fdisplay(fd, "  Operation: SUBU");
        ALU_OR:   $fdisplay(fd, "  Operation: OR");
        ALU_AND:  $fdisplay(fd, "  Operation: AND");
        ALU_XOR:  $fdisplay(fd, "  Operation: XOR");
        ALU_NOR:  $fdisplay(fd, "  Operation: NOR");
        ALU_SLT:  $fdisplay(fd, "  Operation: SLT");
        ALU_SLTU: $fdisplay(fd, "  Operation: SLTU");
        ALU_SLL:  $fdisplay(fd, "  Operation: SLL");
        ALU_SRL:  $fdisplay(fd, "  Operation: SRL");
        ALU_SRA:  $fdisplay(fd, "  Operation: SRA");
        ALU_SLLV: $fdisplay(fd, "  Operation: SLLV");
        ALU_SRLV: $fdisplay(fd, "  Operation: SRLV");
        ALU_SRAV: $fdisplay(fd, "  Operation: SRAV");
        ALU_LUI:  $fdisplay(fd, "  Operation: LUI");
        default:  $fdisplay(fd, "  Operation: UNKNOWN");
    endcase
    $fdisplay(fd, "  Operand1: 0x%h", operand1);
    $fdisplay(fd, "  Operand2: 0x%h", operand2);
    $fdisplay(fd, "  Result: 0x%h", result);
    if (alu_op == ALU_ADD || alu_op == ALU_SUB) begin
        if (overflow)
            $fdisplay(fd, "  Status: INTEGER OVERFLOW DETECTED!");
        else
            $fdisplay(fd, "  Status: No overflow");
    end
endtask

task automatic print_mdu_info(
    int fd,
    string pipeline_name,
    mdu_operation_t mdu_op,
    logic [31:0] operand1,
    logic [31:0] operand2,
    logic [31:0] result,
    logic busy,
    int remaining_cycles
);
    $fdisplay(fd, "\n=== %s MDU Operation ===", pipeline_name);
    case (mdu_op)
        MDU_READ_HI:  $fdisplay(fd, "  Operation: READ_HI");
        MDU_READ_LO:  $fdisplay(fd, "  Operation: READ_LO");
        MDU_WRITE_HI: $fdisplay(fd, "  Operation: WRITE_HI");
        MDU_WRITE_LO: $fdisplay(fd, "  Operation: WRITE_LO");
        MDU_START_SIGNED_MUL:   $fdisplay(fd, "  Operation: START_SIGNED_MUL");
        MDU_START_UNSIGNED_MUL: $fdisplay(fd, "  Operation: START_UNSIGNED_MUL");
        MDU_START_SIGNED_DIV:   $fdisplay(fd, "  Operation: START_SIGNED_DIV");
        MDU_START_UNSIGNED_DIV: $fdisplay(fd, "  Operation: START_UNSIGNED_DIV");
        default:   $fdisplay(fd, "  Operation: UNKNOWN");
    endcase
    
    if (mdu_op == MDU_START_SIGNED_MUL || 
        mdu_op == MDU_START_UNSIGNED_MUL || 
        mdu_op == MDU_START_SIGNED_DIV || 
        mdu_op == MDU_START_UNSIGNED_DIV) begin
        $fdisplay(fd, "  Operand1: 0x%h", operand1);
        $fdisplay(fd, "  Operand2: 0x%h", operand2);
        $fdisplay(fd, "  Remaining Cycles: %0d", remaining_cycles);
    end else if (mdu_op == MDU_WRITE_HI || mdu_op == MDU_WRITE_LO) begin
        $fdisplay(fd, "  Data: 0x%h", operand1);
    end else if (mdu_op == MDU_READ_HI || mdu_op == MDU_READ_LO) begin
        $fdisplay(fd, "  Result: 0x%h", result);
    end
    
    if (busy)
        $fdisplay(fd, "  Status: BUSY");
endtask

`endif