`timescale 1us / 1us

`include "MipsDefinitions.sv"
`include "common.vh"

module PipelineCPU(
    input logic clk,
    input logic rst
);

    // 仿真开始时打开文件
    initial begin
        debug_log_fd = $fopen("pipeline_debug.log", "w");
        $fdisplay(debug_log_fd, "=== Pipeline CPU Debug Log ===");
    end

    // 仿真结束时关闭文件
    final begin
        $fflush(debug_log_fd);
        $fclose(debug_log_fd);
    end

    logic reg_write_enable;
    MipsReg rs_addr, rt_addr, reg_write_addr;
    logic [31:0] rs_data, rt_data, reg_write_data;

    // Stall信号定义
    logic stall_from_id;     // 在ID阶段执行时产生的stall
    logic stall_from_ex;     // 在EX阶段执行时产生的stall

    // Flush信号定义
    logic flush_if;          // IF阶段的flush信号
    logic flush_id;          // ID阶段的flush信号
    logic flush_ex;          // EX阶段的flush信号
    logic flush_mem;         // MEM阶段的flush信号

    // Flush控制逻辑
    always @(*) begin : flush_control
        // 默认值
        flush_if = 1'b0;
        flush_id = 1'b0;
        flush_ex = 1'b0;
        flush_mem = 1'b0;

        // ID阶段stall但EX阶段未stall时的flush（保持原有逻辑）
        if (stall_from_id && !stall_from_ex) begin
            flush_id = 1'b1;
        end
    end

    // 数据冒险检测
    logic hazard_rs_in_id, hazard_rt_in_id;    // ID阶段检测到的数据冒险
    logic hazard_rs_in_ex, hazard_rt_in_ex;    // EX阶段检测到的数据冒险
    
    // MDU忙检测
    logic mdu_busy;
    logic stall_from_mdu;
    always @(*) begin : check_mdu_stall
        stall_from_mdu = 1'b0;
        if (mdu_busy && (id_ex_reg.ctrl.mdu_start || id_ex_reg.ctrl.mdu_use)) begin
            stall_from_mdu = 1'b1;
        end
    end

    // 合并ID/EX阶段产生的所有stall原因
    assign stall_from_id = hazard_rs_in_id || hazard_rt_in_id;
    assign stall_from_ex = hazard_rs_in_ex || hazard_rt_in_ex || stall_from_mdu;

    // 添加分支指令状态跟踪
    logic prev_is_branch;
    logic prev_branch_taken;
    logic curr_is_branch;

    // 添加流水线寄存器和控制信号
    logic jump_enable;
    logic [31:0] pc_addr;
    logic [31:0] jump_address;
    Instruction instruction;
    IF_ID_Register if_id_reg;
    ID_EX_Register id_ex_reg;
    EX_MEM_Register ex_mem_reg;
    MEM_WB_Register mem_wb_reg;
    ControlSignals ctrl;
    logic [31:0] mem_read_result;

    // 添加转发相关信号
    ForwardingType ID_forwarding_rs_type, ID_forwarding_rt_type;
    logic [31:0] ID_forwarded_rs_data, ID_forwarded_rt_data;
    ForwardingType EX_forwarding_rs_type, EX_forwarding_rt_type;
    logic [31:0] EX_forwarded_rs_data, EX_forwarded_rt_data;

    // 在时序逻辑中更新分支状态
    always @(posedge clk) begin
        if (rst || (stall_from_id && !stall_from_ex)) begin
            prev_is_branch <= 0;
            prev_branch_taken <= 0;
        end else if (!stall_from_ex) begin
            prev_is_branch <= curr_is_branch;
            prev_branch_taken <= jump_enable;
        end
    end

    // 添加 ALU 结果线
    logic [31:0] alu_result;
    logic alu_overflow;

    InstructionFetchStage instruction_fetch_stage(
        .clk(clk),
        .rst(rst),
        .stall(stall_from_id || stall_from_ex),
        .flush(flush_if),
        .jump_enable(jump_enable),
        .jump_address(jump_address),
        .if_id_reg(if_id_reg),
        .pc(pc_addr),
        .instruction(instruction)
    );

    ForwardingUnit #(
        .PIPELINE_STAGE(PipelineStage_ID)
    ) ID_forwarding_rs_unit(
        .required_reg(rs_addr),
        .current_data(rs_data),
        .required_stage(ctrl.reg_read_rs_stage),
        .if_id_reg(if_id_reg),
        .id_ex_reg(id_ex_reg),
        .ex_mem_reg(ex_mem_reg),
        .mem_wb_reg(mem_wb_reg),
        .mem_read_in_mem(),
        .mem_read_result(),
        .forwarded_data(ID_forwarded_rs_data),
        .stall(hazard_rs_in_id),
        .forwarding_type(ID_forwarding_rs_type)
    );

    ForwardingUnit #(
        .PIPELINE_STAGE(PipelineStage_ID)
    ) ID_forwarding_rt_unit(
        .required_reg(rt_addr),
        .current_data(rt_data),
        .required_stage(ctrl.reg_read_rt_stage),
        .if_id_reg(if_id_reg),
        .id_ex_reg(id_ex_reg),
        .ex_mem_reg(ex_mem_reg),
        .mem_wb_reg(mem_wb_reg),
        .mem_read_in_mem(),
        .mem_read_result(),
        .forwarded_data(ID_forwarded_rt_data),
        .stall(hazard_rt_in_id),
        .forwarding_type(ID_forwarding_rt_type)
    );

    InstructionDecodeStage instruction_decode_stage(
        .clk(clk),
        .rst(rst),
        .stall(stall_from_ex),
        .flush(flush_id),
        .if_id_reg(if_id_reg),
        .forwarded_rs_data(ID_forwarded_rs_data),
        .forwarded_rt_data(ID_forwarded_rt_data),
        .id_ex_reg(id_ex_reg),
        .rs_addr(rs_addr),
        .rt_addr(rt_addr),
        .ctrl(ctrl),
        .jump_enable(jump_enable),
        .jump_addr(jump_address),
        .prev_is_branch(prev_is_branch),
        .prev_branch_taken(prev_branch_taken),
        .is_branch(curr_is_branch)
    );

    ForwardingUnit #(
        .PIPELINE_STAGE(PipelineStage_EX)
    ) EX_forwarding_rs_unit(
        .required_reg(id_ex_reg.rs_addr),
        .current_data(id_ex_reg.rs_data),
        .required_stage(id_ex_reg.ctrl.reg_read_rs_stage),
        .if_id_reg(if_id_reg),
        .id_ex_reg(id_ex_reg),
        .ex_mem_reg(ex_mem_reg),
        .mem_wb_reg(mem_wb_reg),
        .mem_read_in_mem(ex_mem_reg.ctrl.mem_read),
        .mem_read_result(mem_read_result),
        .forwarded_data(EX_forwarded_rs_data),
        .stall(hazard_rs_in_ex),
        .forwarding_type(EX_forwarding_rs_type)
    );

    ForwardingUnit #(
        .PIPELINE_STAGE(PipelineStage_EX)
    ) EX_forwarding_rt_unit(
        .required_reg(id_ex_reg.rt_addr),
        .current_data(id_ex_reg.rt_data),
        .required_stage(id_ex_reg.ctrl.reg_read_rt_stage),
        .if_id_reg(if_id_reg),
        .id_ex_reg(id_ex_reg),
        .ex_mem_reg(ex_mem_reg),
        .mem_wb_reg(mem_wb_reg),
        .mem_read_in_mem(ex_mem_reg.ctrl.mem_read),
        .mem_read_result(mem_read_result),
        .forwarded_data(EX_forwarded_rt_data),
        .stall(hazard_rt_in_ex),
        .forwarding_type(EX_forwarding_rt_type)
    );

    ExecutionStage execution_stage(
        .clk(clk),
        .rst(rst),
        .stall_rs(hazard_rs_in_ex),
        .stall_rt(hazard_rt_in_ex),
        .stall(stall_from_ex),
        .flush(flush_ex),
        .id_ex_reg(id_ex_reg),
        .forwarded_rs_data(EX_forwarded_rs_data),
        .forwarded_rt_data(EX_forwarded_rt_data),
        .ex_mem_reg(ex_mem_reg),
        .mdu_busy(mdu_busy),
        .alu_result_out(alu_result),      // 添加 ALU 结果输出
        .alu_overflow_out(alu_overflow)   // 添加溢出标志输出
    );

    logic [31:0] MEM_forwarded_rs_data, MEM_forwarded_rt_data;
    ForwardingType MEM_forwarding_rs_type, MEM_forwarding_rt_type;

    ForwardingUnit #(
        .PIPELINE_STAGE(PipelineStage_MEM)
    ) MEM_forwarding_rs_unit(
        .required_reg(ex_mem_reg.rs_addr),
        .current_data(ex_mem_reg.rs_data),
        .required_stage(ex_mem_reg.ctrl.reg_read_rs_stage),
        .if_id_reg(if_id_reg),
        .id_ex_reg(id_ex_reg),
        .ex_mem_reg(ex_mem_reg),
        .mem_wb_reg(mem_wb_reg),
        .mem_read_in_mem(),
        .mem_read_result(),
        .forwarded_data(MEM_forwarded_rs_data),
        .stall(),
        .forwarding_type(MEM_forwarding_rs_type)
    );

    ForwardingUnit #(
        .PIPELINE_STAGE(PipelineStage_MEM)
    ) MEM_forwarding_rt_unit(
        .required_reg(ex_mem_reg.rt_addr),
        .current_data(ex_mem_reg.rt_data),
        .required_stage(ex_mem_reg.ctrl.reg_read_rt_stage),
        .if_id_reg(if_id_reg),
        .id_ex_reg(id_ex_reg),
        .ex_mem_reg(ex_mem_reg),
        .mem_wb_reg(mem_wb_reg),
        .mem_read_in_mem(),
        .mem_read_result(),
        .forwarded_data(MEM_forwarded_rt_data),
        .stall(),
        .forwarding_type(MEM_forwarding_rt_type)
    );
    
    MemoryStage memory_stage(
        .clk(clk),
        .rst(rst),
        .flush(flush_mem),
        .stall(stall_from_ex),
        .ex_mem_reg(ex_mem_reg),
        .forwarded_rs_data(MEM_forwarded_rs_data),
        .forwarded_rt_data(MEM_forwarded_rt_data),
        .mem_wb_reg(mem_wb_reg),
        .mem_read_data(mem_read_result)
    );

    WriteBackStage write_back_stage(
        .clk(clk),
        .rst(rst),
        .flush(flush_mem),
        .stall(stall_from_ex),
        .mem_wb_reg(mem_wb_reg),
        .reg_write_enable(reg_write_enable),
        .reg_write_addr(reg_write_addr),
        .reg_write_data(reg_write_data)
    );

    GeneralPurposeRegisters reg_file(
        .clk(clk),
        .rst(rst),
        .pc(mem_wb_reg.pc),
        .rs_addr(rs_addr),
        .rt_addr(rt_addr),
        .reg_write(reg_write_enable),
        .write_addr(reg_write_addr),
        .write_data(reg_write_data),
        .rs_data(rs_data),
        .rt_data(rt_data)
    );

    // always 块中添加周期计数
    logic has_forwarded;
    logic [31:0] cycle_count;
    always @(posedge clk) begin
        if (rst)
            cycle_count <= 0;
        else
            cycle_count <= cycle_count + 1;
            
        // 基本周期信息
        $fdisplay(debug_log_fd, "\n========== Cycle %0d ==========", cycle_count);
        
        // 输出每个阶段的指令执行情况
        $fdisplay(debug_log_fd, "IF: PC=0x%h, Instruction=%s", 
            pc_addr,
            decode_instruction(instruction.instruction));
        
        $fdisplay(debug_log_fd, "ID: PC=0x%h, Instruction=%s", 
            if_id_reg.pc,
            decode_instruction(if_id_reg.instruction.instruction));
        
        $fdisplay(debug_log_fd, "EX: PC=0x%h, Instruction=%s", 
            id_ex_reg.pc,
            decode_instruction(id_ex_reg.instruction.instruction));
        
        $fdisplay(debug_log_fd, "MEM: PC=0x%h, Instruction=%s", 
            ex_mem_reg.pc,
            decode_instruction(ex_mem_reg.instruction.instruction));
        
        $fdisplay(debug_log_fd, "WB: PC=0x%h, Instruction=%s", 
            mem_wb_reg.pc,
            decode_instruction(mem_wb_reg.instruction.instruction));
        
        // 输出阻塞和数据转发信息
        if (stall_from_id || stall_from_ex) begin
            $fdisplay(debug_log_fd, "\nStall Status:");
            if (hazard_rs_in_id)
                $fdisplay(debug_log_fd, "  ID stage stalled: waiting for RS(%s)", 
                    get_reg_name(rs_addr));
            if (hazard_rt_in_id)
                $fdisplay(debug_log_fd, "  ID stage stalled: waiting for RT(%s)", 
                    get_reg_name(rt_addr));
            if (hazard_rs_in_ex)
                $fdisplay(debug_log_fd, "  EX stage stalled: waiting for RS(%s)", 
                    get_reg_name(id_ex_reg.rs_addr));
            if (hazard_rt_in_ex)
                $fdisplay(debug_log_fd, "  EX stage stalled: waiting for RT(%s)", 
                    get_reg_name(id_ex_reg.rt_addr));
            if (stall_from_mdu)
                $fdisplay(debug_log_fd, "  EX stage stalled: waiting for MDU operation to complete");
        end
        
        // 输出数据转发信息
        has_forwarded = (ID_forwarding_rs_type != ForwardingType'(FORWARDING_TYPE_NONE)) ||
            (ID_forwarding_rt_type != ForwardingType'(FORWARDING_TYPE_NONE)) ||
            (EX_forwarding_rs_type != ForwardingType'(FORWARDING_TYPE_NONE)) ||
            (EX_forwarding_rt_type != ForwardingType'(FORWARDING_TYPE_NONE)) ||
            (MEM_forwarding_rt_type != ForwardingType'(FORWARDING_TYPE_NONE));

        if (has_forwarded) begin
            $fdisplay(debug_log_fd, "\nForwarding Status:");
            if (ID_forwarding_rs_type != ForwardingType'(FORWARDING_TYPE_NONE))
                $fdisplay(debug_log_fd, "  ID: RS(%s) forwarded(%s): 0x%h -> 0x%h", 
                    get_reg_name(rs_addr), get_forwarding_type_name(ID_forwarding_rs_type), rs_data, ID_forwarded_rs_data);
            if (ID_forwarding_rt_type != ForwardingType'(FORWARDING_TYPE_NONE))
                $fdisplay(debug_log_fd, "  ID: RT(%s) forwarded(%s): 0x%h -> 0x%h", 
                    get_reg_name(rt_addr), get_forwarding_type_name(ID_forwarding_rt_type), rt_data, ID_forwarded_rt_data);
            if (EX_forwarding_rs_type != ForwardingType'(FORWARDING_TYPE_NONE))
                $fdisplay(debug_log_fd, "  EX: RS(%s) forwarded(%s): 0x%h -> 0x%h", 
                    get_reg_name(id_ex_reg.rs_addr), get_forwarding_type_name(EX_forwarding_rs_type), id_ex_reg.rs_data, EX_forwarded_rs_data);
            if (EX_forwarding_rt_type != ForwardingType'(FORWARDING_TYPE_NONE))
                $fdisplay(debug_log_fd, "  EX: RT(%s) forwarded(%s): 0x%h -> 0x%h", 
                    get_reg_name(id_ex_reg.rt_addr), get_forwarding_type_name(EX_forwarding_rt_type), id_ex_reg.rt_data, EX_forwarded_rt_data);
            if (MEM_forwarding_rt_type != ForwardingType'(FORWARDING_TYPE_NONE))
                $fdisplay(debug_log_fd, "  MEM: RT(%s) forwarded(%s): 0x%h -> 0x%h", 
                    get_reg_name(ex_mem_reg.rt_addr), get_forwarding_type_name(MEM_forwarding_rt_type), ex_mem_reg.rt_data, MEM_forwarded_rt_data);
        end    
            
        // 输出跳转信息
        if (jump_enable)
            $fdisplay(debug_log_fd, "\nJump: PC -> 0x%h", jump_address);
            
        // 输出寄存器写入信息
        if (reg_write_enable && reg_write_addr != ZERO) begin
            $fdisplay(debug_log_fd, "\nRegister Write:");
            $fdisplay(debug_log_fd, "  %s <= 0x%h", 
                get_reg_name(reg_write_addr), reg_write_data);
        end
        
        // 输出内存访问信息
        if (ex_mem_reg.ctrl.mem_read) begin
            $fdisplay(debug_log_fd, "\nMemory Read:");
            $fdisplay(debug_log_fd, "  Address: 0x%h => Data: 0x%h",
                ex_mem_reg.alu_result, mem_wb_reg.reg_write_data);
        end
        if (ex_mem_reg.ctrl.mem_write) begin
            $fdisplay(debug_log_fd, "\nMemory Write:");
            $fdisplay(debug_log_fd, "  Address: 0x%h <= Data: 0x%h",
                ex_mem_reg.alu_result, MEM_forwarded_rt_data);
        end
        
        // 输出 ALU 操作信息输出
        if (id_ex_reg.ctrl.alu_op != ALU_NOP) begin
            $fdisplay(debug_log_fd, "\nALU Operation:");
            case (id_ex_reg.ctrl.alu_op)
                ALU_ADD:  $fdisplay(debug_log_fd, "  Operation: ADD");
                ALU_ADDU: $fdisplay(debug_log_fd, "  Operation: ADDU");
                ALU_SUB:  $fdisplay(debug_log_fd, "  Operation: SUB");
                ALU_SUBU: $fdisplay(debug_log_fd, "  Operation: SUBU");
                ALU_OR:   $fdisplay(debug_log_fd, "  Operation: OR");
                ALU_AND:  $fdisplay(debug_log_fd, "  Operation: AND");
                ALU_XOR:  $fdisplay(debug_log_fd, "  Operation: XOR");
                ALU_NOR:  $fdisplay(debug_log_fd, "  Operation: NOR");
                ALU_SLT:  $fdisplay(debug_log_fd, "  Operation: SLT");
                ALU_SLTU: $fdisplay(debug_log_fd, "  Operation: SLTU");
                ALU_SLL:  $fdisplay(debug_log_fd, "  Operation: SLL");
                ALU_SRL:  $fdisplay(debug_log_fd, "  Operation: SRL");
                ALU_SRA:  $fdisplay(debug_log_fd, "  Operation: SRA");
                ALU_SLLV: $fdisplay(debug_log_fd, "  Operation: SLLV");
                ALU_SRLV: $fdisplay(debug_log_fd, "  Operation: SRLV");
                ALU_SRAV: $fdisplay(debug_log_fd, "  Operation: SRAV");
                ALU_LUI:  $fdisplay(debug_log_fd, "  Operation: LUI");
                default:  $fdisplay(debug_log_fd, "  Operation: UNKNOWN");
            endcase
            $fdisplay(debug_log_fd, "  Operand1: 0x%h", EX_forwarded_rs_data);
            $fdisplay(debug_log_fd, "  Operand2: 0x%h", EX_forwarded_rt_data);
            $fdisplay(debug_log_fd, "  Result: 0x%h", alu_result);
            // 添加溢出状态显示
            if (id_ex_reg.ctrl.alu_op == ALU_ADD || id_ex_reg.ctrl.alu_op == ALU_SUB) begin
                if (alu_overflow)
                    $fdisplay(debug_log_fd, "  Status: INTEGER OVERFLOW DETECTED!");
                else
                    $fdisplay(debug_log_fd, "  Status: No overflow");
            end
        end

        // 输出 MDU 操作信息输出
        if (id_ex_reg.ctrl.mdu_start || id_ex_reg.ctrl.mdu_use) begin
            $fdisplay(debug_log_fd, "\nMDU Operation:");
            case (id_ex_reg.ctrl.mdu_operation)
                MDU_READ_HI:  $fdisplay(debug_log_fd, "  Operation: READ_HI");
                MDU_READ_LO:  $fdisplay(debug_log_fd, "  Operation: READ_LO");
                MDU_WRITE_HI: $fdisplay(debug_log_fd, "  Operation: WRITE_HI");
                MDU_WRITE_LO: $fdisplay(debug_log_fd, "  Operation: WRITE_LO");
                MDU_START_SIGNED_MUL:   $fdisplay(debug_log_fd, "  Operation: START_SIGNED_MUL");
                MDU_START_UNSIGNED_MUL: $fdisplay(debug_log_fd, "  Operation: START_UNSIGNED_MUL");
                MDU_START_SIGNED_DIV:   $fdisplay(debug_log_fd, "  Operation: START_SIGNED_DIV");
                MDU_START_UNSIGNED_DIV: $fdisplay(debug_log_fd, "  Operation: START_UNSIGNED_DIV");
                default:   $fdisplay(debug_log_fd, "  Operation: UNKNOWN");
            endcase
            if (id_ex_reg.ctrl.mdu_operation == MDU_START_SIGNED_MUL || 
                id_ex_reg.ctrl.mdu_operation == MDU_START_UNSIGNED_MUL || 
                id_ex_reg.ctrl.mdu_operation == MDU_START_SIGNED_DIV || 
                id_ex_reg.ctrl.mdu_operation == MDU_START_UNSIGNED_DIV) begin
                $fdisplay(debug_log_fd, "  Operand1: 0x%h", EX_forwarded_rs_data);
                $fdisplay(debug_log_fd, "  Operand2: 0x%h", EX_forwarded_rt_data);
                $fdisplay(debug_log_fd, "  Remaining Cycles: %0d", execution_stage.mdu.remainingCycleCount);
            end else if (id_ex_reg.ctrl.mdu_operation == MDU_WRITE_HI || 
                        id_ex_reg.ctrl.mdu_operation == MDU_WRITE_LO) begin
                $fdisplay(debug_log_fd, "  Data: 0x%h", EX_forwarded_rs_data);
            end else if (id_ex_reg.ctrl.mdu_operation == MDU_READ_HI || 
                        id_ex_reg.ctrl.mdu_operation == MDU_READ_LO) begin
                $fdisplay(debug_log_fd, "  Result: 0x%h", ex_mem_reg.reg_write_data);
            end
            if (mdu_busy)
                $fdisplay(debug_log_fd, "  Status: BUSY");
        end
        
        // 输出 syscall 状态信息
        if (mem_wb_reg.ctrl.is_syscall && !stall_from_ex) begin
            $fdisplay(debug_log_fd, "\nSYSCALL Status:");
            case (mem_wb_reg.rs_data)  // $v0
                32'd1: $fdisplay(debug_log_fd, "  Syscall 1: print_int(%0d)", $signed(mem_wb_reg.rt_data));  // $a0
                32'd10: begin
                    $fdisplay(debug_log_fd, "  Syscall 10: exit");
                    $fflush(debug_log_fd);
                    $fclose(debug_log_fd);
                end
                default: $fdisplay(debug_log_fd, "  Unknown syscall: %0d", mem_wb_reg.rs_data);
            endcase
        end
        
        // 添加异常检测信息输出
        handle_pipeline_exception(PipelineStage_IF, if_id_reg.exception, if_id_reg.pc, if_id_reg.instruction, debug_log_fd);
        handle_pipeline_exception(PipelineStage_ID, id_ex_reg.exception, id_ex_reg.pc, id_ex_reg.instruction, debug_log_fd);
        handle_pipeline_exception(PipelineStage_EX, ex_mem_reg.exception, ex_mem_reg.pc, ex_mem_reg.instruction, debug_log_fd);
        handle_pipeline_exception(PipelineStage_WB, mem_wb_reg.exception, mem_wb_reg.pc, mem_wb_reg.instruction, debug_log_fd);
        
        // 在每个周期结束时添加分隔线
        $fdisplay(debug_log_fd, "\n----------------------------------------");
    end

    // 异常处理
    always @(posedge clk) begin : handle_exception
        if (!rst) begin
            // 按照流水线顺序检查异常，优先处理较早阶段的异常
            fatal_pipeline_exception(PipelineStage_IF, if_id_reg.exception, if_id_reg.pc, if_id_reg.instruction);
            fatal_pipeline_exception(PipelineStage_ID, id_ex_reg.exception, id_ex_reg.pc, id_ex_reg.instruction);
            fatal_pipeline_exception(PipelineStage_EX, ex_mem_reg.exception, ex_mem_reg.pc, ex_mem_reg.instruction);
            fatal_pipeline_exception(PipelineStage_WB, mem_wb_reg.exception, mem_wb_reg.pc, mem_wb_reg.instruction);
        end
    end

endmodule
