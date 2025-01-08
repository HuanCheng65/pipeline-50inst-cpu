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
    
    // 第二条指令的寄存器地址和数据
    MipsReg rs_addr1, rt_addr1;
    logic [31:0] rs_data1, rt_data1;

    // Stall信号定义
    logic stall_from_id0, stall_from_id1;     // ID阶段产生的stall
    logic stall_from_ex0, stall_from_ex1;     // EX阶段产生的stall

    // Flush信号定义
    logic flush_if0, flush_if1;          // IF阶段的flush信号
    logic flush_id0, flush_id1;          // ID阶段的flush信号
    logic flush_ex0, flush_ex1;          // EX阶段的flush信号
    logic flush_mem0, flush_mem1;        // MEM阶段的flush信号

    // Flush控制逻辑
    always @(*) begin : flush_control
        // 默认值
        flush_if0 = 1'b0;
        flush_if1 = 1'b0;
        flush_id0 = 1'b0;
        flush_id1 = 1'b0;
        flush_ex0 = 1'b0;
        flush_ex1 = 1'b0;
        flush_mem0 = 1'b0;
        flush_mem1 = 1'b0;

        // ID阶段stall但EX阶段未stall时的flush
        if (stall_from_id0 && !stall_from_ex0) begin
            flush_id0 = 1'b1;
        end

        if (stall_from_id1 && !stall_from_ex1) begin
            flush_id1 = 1'b1;
        end

        // 跳转发生时的flush
        if (jump_enable) begin
            flush_if1 = 1'b1;
            flush_id1 = 1'b1;
        end
    end

    // 数据冒险检测
    logic hazard_rs0_in_id, hazard_rt0_in_id;    // 第一条指令在 ID 阶段检测到的数据冒险
    logic hazard_rs0_in_ex, hazard_rt0_in_ex;    // 第一条指令在 EX 阶段检测到的数据冒险
    logic hazard_rs1_in_ex, hazard_rt1_in_ex;    // 第二条指令在 EX 阶段检测到的数据冒险
    
    // MDU忙检测
    logic mdu_busy;
    logic stall_from_mdu;
    always @(*) begin : check_mdu_stall
        stall_from_mdu = 1'b0;
        if (mdu_busy && (id_ex_reg.ctrl0.mdu_start || id_ex_reg.ctrl0.mdu_use)) begin
            stall_from_mdu = 1'b1;
        end
    end

    // 合并ID/EX阶段产生的所有stall原因
    assign stall_from_id0 = hazard_rs0_in_id || hazard_rt0_in_id;
    assign stall_from_id1 = 1'b0;  // 第二条指令在ID阶段不会产生stall，因为不会使用转发
    assign stall_from_ex0 = hazard_rs0_in_ex || hazard_rt0_in_ex || stall_from_mdu;
    assign stall_from_ex1 = hazard_rs1_in_ex || hazard_rt1_in_ex;

    // 添加分支指令状态跟踪
    logic prev_is_branch;
    logic prev_branch_taken;
    logic curr_is_branch;

    // 添加流水线寄存器和控制信号
    logic jump_enable;
    logic [31:0] pc_addr;
    logic [31:0] jump_address;
    logic [31:0] instr_id0, instr_id1;
    Instruction instruction0, instruction1;  // 当前正在取的指令
    IF_ID_Register if_id_reg;
    ID_EX_Register id_ex_reg;
    EX_MEM_Register ex_mem_reg;
    MEM_WB_Register mem_wb_reg;
    ControlSignals ctrl0, ctrl1;
    logic mem_read_in_mem;
    logic [31:0] mem_read_result;

    // 添加转发相关信号
    ForwardingType ID_forwarding_rs0_type, ID_forwarding_rt0_type;
    ForwardingType ID_forwarding_rs1_type, ID_forwarding_rt1_type;
    logic [31:0] ID_forwarded_rs0_from, ID_forwarded_rt0_from;
    logic [31:0] ID_forwarded_rs1_from, ID_forwarded_rt1_from;
    logic [31:0] ID_forwarded_rs0_data, ID_forwarded_rt0_data;
    logic [31:0] ID_forwarded_rs1_data, ID_forwarded_rt1_data;
    ForwardingType EX_forwarding_rs0_type, EX_forwarding_rt0_type;
    ForwardingType EX_forwarding_rs1_type, EX_forwarding_rt1_type;
    logic [31:0] EX_forwarded_rs0_from, EX_forwarded_rt0_from;
    logic [31:0] EX_forwarded_rs1_from, EX_forwarded_rt1_from;
    logic [31:0] EX_forwarded_rs0_data, EX_forwarded_rt0_data;
    logic [31:0] EX_forwarded_rs1_data, EX_forwarded_rt1_data;

    // 在时序逻辑中更新分支状态
    always @(posedge clk) begin
        if (rst || (stall_from_id0 && !stall_from_ex0)) begin
            prev_is_branch <= 0;
            prev_branch_taken <= 0;
        end else if (!stall_from_ex0) begin
            prev_is_branch <= curr_is_branch;
            prev_branch_taken <= jump_enable;
        end
    end

    // 添加 ALU 结果线
    logic [31:0] alu_result0, alu_result1;
    logic alu_overflow0, alu_overflow1;

    // 添加 ALU 操作数信号
    logic [31:0] alu_operand1_0, alu_operand2_0;
    logic [31:0] alu_operand1_1, alu_operand2_1;

    // 更新 IF 阶段连接
    InstructionFetchStage instruction_fetch_stage(
        .clk(clk),
        .rst(rst),
        .stall0(stall_from_id0 || stall_from_ex0),
        .stall1(stall_from_id1 || stall_from_ex1),
        .flush0(flush_if0),
        .flush1(flush_if1),
        .jump_enable(jump_enable),
        .jump_address(jump_address),
        .if_id_reg(if_id_reg),
        .pc(pc_addr),
        .instr_id0(instr_id0),
        .instr_id1(instr_id1),
        .instruction0(instruction0),
        .instruction1(instruction1),
        .has_raw_dependency(has_raw_dependency),
        .has_waw_dependency(has_waw_dependency),
        .has_war_dependency(has_war_dependency)
    );

    // 第一条指令的 ID 阶段转发
    ForwardingUnit #(
        .PIPELINE_STAGE(PipelineStage_ID)
    ) ID_forwarding_rs0_unit(
        .required_reg(rs_addr),
        .current_data(rs_data),
        .required_stage(ctrl0.reg_read_rs_stage),
        .if_id_reg(if_id_reg),
        .id_ex_reg(id_ex_reg),
        .ex_mem_reg(ex_mem_reg),
        .mem_wb_reg(mem_wb_reg),
        .mem_read_in_mem(mem_read_in_mem),
        .mem_read_result(mem_read_result),
        .forwarded_data(ID_forwarded_rs0_data),
        .stall(hazard_rs0_in_id),
        .forwarding_type(ID_forwarding_rs0_type),
        .forwarded_from(ID_forwarded_rs0_from)
    );

    ForwardingUnit #(
        .PIPELINE_STAGE(PipelineStage_ID)
    ) ID_forwarding_rt0_unit(
        .required_reg(rt_addr),
        .current_data(rt_data),
        .required_stage(ctrl0.reg_read_rt_stage),
        .if_id_reg(if_id_reg),
        .id_ex_reg(id_ex_reg),
        .ex_mem_reg(ex_mem_reg),
        .mem_wb_reg(mem_wb_reg),
        .mem_read_in_mem(mem_read_in_mem),
        .mem_read_result(mem_read_result),
        .forwarded_data(ID_forwarded_rt0_data),
        .stall(hazard_rt0_in_id),
        .forwarding_type(ID_forwarding_rt0_type),
        .forwarded_from(ID_forwarded_rt0_from)
    );

    ForwardingUnit #(
        .PIPELINE_STAGE(PipelineStage_ID)
    ) ID_forwarding_rs1_unit(
        .required_reg(rs_addr1),
        .current_data(rs_data1),
        .required_stage(ctrl1.reg_read_rs_stage),
        .if_id_reg(if_id_reg),
        .id_ex_reg(id_ex_reg),
        .ex_mem_reg(ex_mem_reg),
        .mem_wb_reg(mem_wb_reg),
        .mem_read_in_mem(mem_read_in_mem),
        .mem_read_result(mem_read_result),
        .forwarded_data(ID_forwarded_rs1_data),
        .stall(hazard_rs1_in_id),
        .forwarding_type(ID_forwarding_rs1_type),
        .forwarded_from(ID_forwarded_rs1_from)
    );

    ForwardingUnit #(
        .PIPELINE_STAGE(PipelineStage_ID)
    ) ID_forwarding_rt1_unit(
        .required_reg(rt_addr1),
        .current_data(rt_data1),
        .required_stage(ctrl1.reg_read_rt_stage),
        .if_id_reg(if_id_reg),
        .id_ex_reg(id_ex_reg),
        .ex_mem_reg(ex_mem_reg),
        .mem_wb_reg(mem_wb_reg),
        .mem_read_in_mem(mem_read_in_mem),
        .mem_read_result(mem_read_result),
        .forwarded_data(ID_forwarded_rt1_data),
        .stall(hazard_rt1_in_id),
        .forwarding_type(ID_forwarding_rt1_type),
        .forwarded_from(ID_forwarded_rt1_from)
    );

    // 第一条指令的 EX 阶段转发
    ForwardingUnit #(
        .PIPELINE_STAGE(PipelineStage_EX)
    ) EX_forwarding_rs0_unit(
        .required_reg(id_ex_reg.rs_addr0),
        .current_data(id_ex_reg.rs_data0),
        .required_stage(id_ex_reg.ctrl0.reg_read_rs_stage),
        .if_id_reg(if_id_reg),
        .id_ex_reg(id_ex_reg),
        .ex_mem_reg(ex_mem_reg),
        .mem_wb_reg(mem_wb_reg),
        .mem_read_in_mem(mem_read_in_mem),
        .mem_read_result(mem_read_result),
        .forwarded_data(EX_forwarded_rs0_data),
        .stall(hazard_rs0_in_ex),
        .forwarding_type(EX_forwarding_rs0_type),
        .forwarded_from(EX_forwarded_rs0_from)
    );

    ForwardingUnit #(
        .PIPELINE_STAGE(PipelineStage_EX)
    ) EX_forwarding_rt0_unit(
        .required_reg(id_ex_reg.rt_addr0),
        .current_data(id_ex_reg.rt_data0),
        .required_stage(id_ex_reg.ctrl0.reg_read_rt_stage),
        .if_id_reg(if_id_reg),
        .id_ex_reg(id_ex_reg),
        .ex_mem_reg(ex_mem_reg),
        .mem_wb_reg(mem_wb_reg),
        .mem_read_in_mem(mem_read_in_mem),
        .mem_read_result(mem_read_result),
        .forwarded_data(EX_forwarded_rt0_data),
        .stall(hazard_rt0_in_ex),
        .forwarding_type(EX_forwarding_rt0_type),
        .forwarded_from(EX_forwarded_rt0_from)
    );

    // 第二条指令的 EX 阶段转发
    ForwardingUnit #(
        .PIPELINE_STAGE(PipelineStage_EX)
    ) EX_forwarding_rs1_unit(
        .required_reg(id_ex_reg.rs_addr1),
        .current_data(id_ex_reg.rs_data1),
        .required_stage(id_ex_reg.ctrl1.reg_read_rs_stage),
        .if_id_reg(if_id_reg),
        .id_ex_reg(id_ex_reg),
        .ex_mem_reg(ex_mem_reg),
        .mem_wb_reg(mem_wb_reg),
        .mem_read_in_mem(mem_read_in_mem),
        .mem_read_result(mem_read_result),
        .forwarded_data(EX_forwarded_rs1_data),
        .stall(hazard_rs1_in_ex),
        .forwarding_type(EX_forwarding_rs1_type),
        .forwarded_from(EX_forwarded_rs1_from)
    );

    ForwardingUnit #(
        .PIPELINE_STAGE(PipelineStage_EX)
    ) EX_forwarding_rt1_unit(
        .required_reg(id_ex_reg.rt_addr1),
        .current_data(id_ex_reg.rt_data1),
        .required_stage(id_ex_reg.ctrl1.reg_read_rt_stage),
        .if_id_reg(if_id_reg),
        .id_ex_reg(id_ex_reg),
        .ex_mem_reg(ex_mem_reg),
        .mem_wb_reg(mem_wb_reg),
        .mem_read_in_mem(mem_read_in_mem),
        .mem_read_result(mem_read_result),
        .forwarded_data(EX_forwarded_rt1_data),
        .stall(hazard_rt1_in_ex),
        .forwarding_type(EX_forwarding_rt1_type),
        .forwarded_from(EX_forwarded_rt1_from)
    );

    // 更新 ID 阶段连接
    InstructionDecodeStage instruction_decode_stage(
        .clk(clk),
        .rst(rst),
        .stall0(stall_from_ex0),
        .stall1(stall_from_ex1),
        .flush0(flush_id0),
        .flush1(flush_id1),
        .if_id_reg(if_id_reg),
        .rs_data0(ID_forwarded_rs0_data),
        .rt_data0(ID_forwarded_rt0_data),
        .rs_data1(ID_forwarded_rs1_data),  // 第二条指令直接使用寄存器堆的数据
        .rt_data1(ID_forwarded_rt1_data),  // 第二条指令直接使用寄存器堆的数据
        .id_ex_reg(id_ex_reg),
        .rs_addr0(rs_addr),
        .rt_addr0(rt_addr),
        .ctrl0(ctrl0),
        .rs_addr1(rs_addr1),
        .rt_addr1(rt_addr1),
        .ctrl1(ctrl1),
        .jump_enable(jump_enable),
        .jump_addr(jump_address),
        .prev_is_branch(prev_is_branch),
        .prev_branch_taken(prev_branch_taken),
        .is_branch(curr_is_branch)
    );

    // 更新 EX 阶段连接
    ExecutionStage execution_stage(
        .clk(clk),
        .rst(rst),
        .stall0(stall_from_ex0),
        .stall1(stall_from_ex1),
        .flush0(flush_ex0),
        .flush1(flush_ex1),
        .id_ex_reg(id_ex_reg),
        .forwarded_rs_data0(EX_forwarded_rs0_data),
        .forwarded_rt_data0(EX_forwarded_rt0_data),
        .forwarded_rs_data1(EX_forwarded_rs1_data),
        .forwarded_rt_data1(EX_forwarded_rt1_data),
        .ex_mem_reg(ex_mem_reg),
        .mdu_busy(mdu_busy),
        .alu_result0_out(alu_result0),
        .alu_overflow0_out(alu_overflow0),
        .alu_result1_out(alu_result1),
        .alu_overflow1_out(alu_overflow1),
        .operand1_0_out(alu_operand1_0),
        .operand2_0_out(alu_operand2_0),
        .operand1_1_out(alu_operand1_1),
        .operand2_1_out(alu_operand2_1)
    );

    // MEM 阶段的转发单元
    logic [31:0] MEM_forwarded_rs_data0, MEM_forwarded_rt_data0;
    logic [31:0] MEM_forwarded_rs_data1, MEM_forwarded_rt_data1;
    ForwardingType MEM_forwarding_rs0_type, MEM_forwarding_rt0_type;
    ForwardingType MEM_forwarding_rs1_type, MEM_forwarding_rt1_type;

    // 第一条指令的 MEM 阶段转发
    ForwardingUnit #(
        .PIPELINE_STAGE(PipelineStage_MEM)
    ) MEM_forwarding_rs0_unit(
        .required_reg(ex_mem_reg.rs_addr0),
        .current_data(ex_mem_reg.rs_data0),
        .required_stage(ex_mem_reg.ctrl0.reg_read_rs_stage),
        .if_id_reg(if_id_reg),
        .id_ex_reg(id_ex_reg),
        .ex_mem_reg(ex_mem_reg),
        .mem_wb_reg(mem_wb_reg),
        .mem_read_in_mem(),
        .mem_read_result(),
        .forwarded_data(MEM_forwarded_rs_data0),
        .stall(),
        .forwarding_type(MEM_forwarding_rs0_type)
    );

    ForwardingUnit #(
        .PIPELINE_STAGE(PipelineStage_MEM)
    ) MEM_forwarding_rt0_unit(
        .required_reg(ex_mem_reg.rt_addr0),
        .current_data(ex_mem_reg.rt_data0),
        .required_stage(ex_mem_reg.ctrl0.reg_read_rt_stage),
        .if_id_reg(if_id_reg),
        .id_ex_reg(id_ex_reg),
        .ex_mem_reg(ex_mem_reg),
        .mem_wb_reg(mem_wb_reg),
        .mem_read_in_mem(),
        .mem_read_result(),
        .forwarded_data(MEM_forwarded_rt_data0),
        .stall(),
        .forwarding_type(MEM_forwarding_rt0_type)
    );

    // 第二条指令的 MEM 阶段转发
    ForwardingUnit #(
        .PIPELINE_STAGE(PipelineStage_MEM)
    ) MEM_forwarding_rs1_unit(
        .required_reg(ex_mem_reg.rs_addr1),
        .current_data(ex_mem_reg.rs_data1),
        .required_stage(ex_mem_reg.ctrl1.reg_read_rs_stage),
        .if_id_reg(if_id_reg),
        .id_ex_reg(id_ex_reg),
        .ex_mem_reg(ex_mem_reg),
        .mem_wb_reg(mem_wb_reg),
        .mem_read_in_mem(),
        .mem_read_result(),
        .forwarded_data(MEM_forwarded_rs_data1),
        .stall(),
        .forwarding_type(MEM_forwarding_rs1_type)
    );

    ForwardingUnit #(
        .PIPELINE_STAGE(PipelineStage_MEM)
    ) MEM_forwarding_rt1_unit(
        .required_reg(ex_mem_reg.rt_addr1),
        .current_data(ex_mem_reg.rt_data1),
        .required_stage(ex_mem_reg.ctrl1.reg_read_rt_stage),
        .if_id_reg(if_id_reg),
        .id_ex_reg(id_ex_reg),
        .ex_mem_reg(ex_mem_reg),
        .mem_wb_reg(mem_wb_reg),
        .mem_read_in_mem(),
        .mem_read_result(),
        .forwarded_data(MEM_forwarded_rt_data1),
        .stall(),
        .forwarding_type(MEM_forwarding_rt1_type)
    );
    
    MemoryStage memory_stage(
        .clk(clk),
        .rst(rst),
        .stall0(stall_from_ex0),
        .stall1(stall_from_ex1),
        .flush0(flush_mem0),
        .flush1(flush_mem1),
        .ex_mem_reg(ex_mem_reg),
        .forwarded_rs_data0(MEM_forwarded_rs_data0),
        .forwarded_rt_data0(MEM_forwarded_rt_data0),
        .forwarded_rs_data1(MEM_forwarded_rs_data1),
        .forwarded_rt_data1(MEM_forwarded_rt_data1),
        .mem_wb_reg(mem_wb_reg),
        .mem_read_in_mem(mem_read_in_mem),
        .mem_read_data(mem_read_result)
    );

    // 寄存器写回信号
    logic reg_write_enable0, reg_write_enable1;
    MipsReg reg_write_addr0, reg_write_addr1;
    logic [31:0] reg_write_data0, reg_write_data1;

    WriteBackStage write_back_stage(
        .clk(clk),
        .rst(rst),
        .stall0(stall_from_ex0),
        .stall1(stall_from_ex1),
        .flush0(flush_mem0),
        .flush1(flush_mem1),
        .mem_wb_reg(mem_wb_reg),
        .reg_write_enable0(reg_write_enable0),
        .reg_write_enable1(reg_write_enable1),
        .reg_write_addr0(reg_write_addr0),
        .reg_write_addr1(reg_write_addr1),
        .reg_write_data0(reg_write_data0),
        .reg_write_data1(reg_write_data1)
    );

    GeneralPurposeRegisters reg_file(
        .clk(clk),
        .rst(rst),
        .pc0(mem_wb_reg.pc0),
        .pc1(mem_wb_reg.pc1),
        // 第一条指令的读端口
        .rs_addr0(rs_addr),
        .rt_addr0(rt_addr),
        .rs_data0(rs_data),
        .rt_data0(rt_data),
        // 第二条指令的读端口
        .rs_addr1(rs_addr1),
        .rt_addr1(rt_addr1),
        .rs_data1(rs_data1),
        .rt_data1(rt_data1),
        // 写端口（支持双写）
        .reg_write0(reg_write_enable0),
        .write_addr0(reg_write_addr0),
        .write_data0(reg_write_data0),
        .reg_write1(reg_write_enable1),
        .write_addr1(reg_write_addr1),
        .write_data1(reg_write_data1)
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
        
        // Pipeline 0 (主流水线)
        print_pipeline_stage(debug_log_fd, "Pipeline 0",
            pc_addr, instr_id0, instruction0, 1'b1,
            if_id_reg.pc0, if_id_reg.instr_id0, if_id_reg.instruction0, if_id_reg.valid0,
            id_ex_reg.pc0, id_ex_reg.instr_id0, id_ex_reg.instruction0, id_ex_reg.valid0,
            ex_mem_reg.pc0, ex_mem_reg.instr_id0, ex_mem_reg.instruction0, ex_mem_reg.valid0,
            mem_wb_reg.pc0, mem_wb_reg.instr_id0, mem_wb_reg.instruction0, mem_wb_reg.valid0);

        // Pipeline 1 (第二条流水线)
        print_pipeline_stage(debug_log_fd, "Pipeline 1",
            pc_addr + 4, instr_id1, instruction1, 1'b1,
            if_id_reg.pc1, if_id_reg.instr_id1, if_id_reg.instruction1, if_id_reg.valid1,
            id_ex_reg.pc1, id_ex_reg.instr_id1, id_ex_reg.instruction1, id_ex_reg.valid1,
            ex_mem_reg.pc1, ex_mem_reg.instr_id1, ex_mem_reg.instruction1, ex_mem_reg.valid1,
            mem_wb_reg.pc1, mem_wb_reg.instr_id1, mem_wb_reg.instruction1, mem_wb_reg.valid1);

        // 指令调度信息
        print_scheduling_info(debug_log_fd,
            instruction_decode_stage.decode0.is_valid,
            instruction_decode_stage.decode1.is_valid,
            instruction_decode_stage.can_dual_issue,
            instruction_decode_stage.has_raw_dependency,
            instruction_decode_stage.has_waw_dependency,
            instruction_decode_stage.has_war_dependency);
        
        // Pipeline 0 的冒险信息
        if (stall_from_id0 || stall_from_ex0) begin
            print_hazard_info(debug_log_fd, "Pipeline 0",
                hazard_rs0_in_id, hazard_rt0_in_id,
                hazard_rs0_in_ex, hazard_rt0_in_ex,
                stall_from_mdu,
                rs_addr, rt_addr,
                id_ex_reg.rs_addr0, id_ex_reg.rt_addr0);
        end
        
        // Pipeline 0 的转发信息
        has_forwarded = (ID_forwarding_rs0_type != ForwardingType'(FORWARDING_TYPE_NONE)) ||
            (ID_forwarding_rt0_type != ForwardingType'(FORWARDING_TYPE_NONE)) ||
            (EX_forwarding_rs0_type != ForwardingType'(FORWARDING_TYPE_NONE)) ||
            (EX_forwarding_rt0_type != ForwardingType'(FORWARDING_TYPE_NONE)) ||
            (MEM_forwarding_rs0_type != ForwardingType'(FORWARDING_TYPE_NONE)) ||
            (MEM_forwarding_rt0_type != ForwardingType'(FORWARDING_TYPE_NONE));

        if (has_forwarded) begin
            print_forwarding_info(debug_log_fd, "Pipeline 0",
                ID_forwarding_rs0_type, ID_forwarding_rt0_type,
                EX_forwarding_rs0_type, EX_forwarding_rt0_type,
                MEM_forwarding_rs0_type, MEM_forwarding_rt0_type,
                ID_forwarded_rs0_from, ID_forwarded_rt0_from,
                EX_forwarded_rs0_from, EX_forwarded_rt0_from,
                rs_addr, rt_addr,
                id_ex_reg.rs_addr0, id_ex_reg.rt_addr0,
                ex_mem_reg.rs_addr0, ex_mem_reg.rt_addr0,
                rs_data, rt_data,
                id_ex_reg.rs_data0, id_ex_reg.rt_data0,
                ex_mem_reg.rs_data0, ex_mem_reg.rt_data0,
                ID_forwarded_rs0_data, ID_forwarded_rt0_data,
                EX_forwarded_rs0_data, EX_forwarded_rt0_data,
                MEM_forwarded_rs_data0, MEM_forwarded_rt_data0);
        end

        // Pipeline 1 的转发信息
        has_forwarded = (ID_forwarding_rs1_type != ForwardingType'(FORWARDING_TYPE_NONE)) ||
            (ID_forwarding_rt1_type != ForwardingType'(FORWARDING_TYPE_NONE)) ||
            (EX_forwarding_rs1_type != ForwardingType'(FORWARDING_TYPE_NONE)) ||
            (EX_forwarding_rt1_type != ForwardingType'(FORWARDING_TYPE_NONE)) ||
            (MEM_forwarding_rt1_type != ForwardingType'(FORWARDING_TYPE_NONE));

        if (has_forwarded && id_ex_reg.valid1) begin
            print_forwarding_info(debug_log_fd, "Pipeline 1",
                ID_forwarding_rs1_type, ID_forwarding_rt1_type,
                EX_forwarding_rs1_type, EX_forwarding_rt1_type,
                MEM_forwarding_rs1_type, MEM_forwarding_rt1_type,
                ID_forwarded_rs1_from, ID_forwarded_rt1_from,
                EX_forwarded_rs1_from, EX_forwarded_rt1_from,
                rs_addr1, rt_addr1,
                id_ex_reg.rs_addr1, id_ex_reg.rt_addr1,
                ex_mem_reg.rs_addr1, ex_mem_reg.rt_addr1,
                rs_data1, rt_data1,
                id_ex_reg.rs_data1, id_ex_reg.rt_data1,
                ex_mem_reg.rs_data1, ex_mem_reg.rt_data1,
                ID_forwarded_rs1_data, ID_forwarded_rt1_data,
                EX_forwarded_rs1_data, EX_forwarded_rt1_data,
                MEM_forwarded_rs_data1, MEM_forwarded_rt_data1);
        end
            
        // Pipeline 0 的跳转信息
        if (jump_enable)
            $fdisplay(debug_log_fd, "\n=== Pipeline 0 Jump ===\n  PC -> 0x%h", jump_address);
            
        // Pipeline 0/1 的寄存器写入信息
        if (reg_write_enable0 && reg_write_addr0 != ZERO) begin
            $fdisplay(debug_log_fd, "\n=== Pipeline 0 Register Write ===");
            $fdisplay(debug_log_fd, "  %s <= 0x%h", 
                get_reg_name(reg_write_addr0), reg_write_data0);
        end
        if (reg_write_enable1 && reg_write_addr1 != ZERO) begin
            $fdisplay(debug_log_fd, "\n=== Pipeline 1 Register Write ===");
            $fdisplay(debug_log_fd, "  %s <= 0x%h", 
                get_reg_name(reg_write_addr1), reg_write_data1);
        end
        
        // Pipeline 0 的内存访问信息
        if (ex_mem_reg.ctrl0.mem_read) begin
            $fdisplay(debug_log_fd, "\n=== Pipeline 0 Memory Read ===");
            $fdisplay(debug_log_fd, "  Address: 0x%h => Data: 0x%h",
                ex_mem_reg.alu_result0, mem_read_result);
        end
        if (ex_mem_reg.ctrl0.mem_write) begin
            $fdisplay(debug_log_fd, "\n=== Pipeline 0 Memory Write ===");
            $fdisplay(debug_log_fd, "  Address: 0x%h <= Data: 0x%h",
                ex_mem_reg.alu_result0, MEM_forwarded_rt_data0);
        end
        
        // Pipeline 1 的内存访问信息
        // if (ex_mem_reg.ctrl1.mem_read) begin
        //     $fdisplay(debug_log_fd, "\n=== Pipeline 1 Memory Read ===");
        //     $fdisplay(debug_log_fd, "  Address: 0x%h => Data: 0x%h",
        //         ex_mem_reg.alu_result1, mem_wb_reg.reg_write_data1);
        // end
        // if (ex_mem_reg.ctrl1.mem_write) begin
        //     $fdisplay(debug_log_fd, "\n=== Pipeline 1 Memory Write ===");
        //     $fdisplay(debug_log_fd, "  Address: 0x%h <= Data: 0x%h",
        //         ex_mem_reg.alu_result1, MEM_forwarded_rt_data0);
        // end
        
        // Pipeline 0 的 ALU 操作信息
        if (id_ex_reg.ctrl0.alu_op != ALU_NOP) begin
            print_alu_info(debug_log_fd, "Pipeline 0",
                id_ex_reg.ctrl0.alu_op,
                alu_operand1_0,
                alu_operand2_0,
                alu_result0,
                alu_overflow0);
            // 输出操作数来源信息
            $fdisplay(debug_log_fd, "  Operand Sources:");
            case (id_ex_reg.ctrl0.operand1_src)
                OPERAND_SOURCE_RS: $fdisplay(debug_log_fd, "    op1: rs (%s)", get_reg_name(id_ex_reg.rs_addr0));
                OPERAND_SOURCE_RT: $fdisplay(debug_log_fd, "    op1: rt (%s)", get_reg_name(id_ex_reg.rt_addr0));
                OPERAND_SOURCE_UNSIGNED_IMM16: $fdisplay(debug_log_fd, "    op1: unsigned_imm16 (0x%h)", id_ex_reg.instruction0.immediate[15:0]);
                OPERAND_SOURCE_SIGNED_IMM16: $fdisplay(debug_log_fd, "    op1: signed_imm16 (0x%h)", id_ex_reg.instruction0.immediate[15:0]);
                OPERAND_SOURCE_UNSIGNED_SHAMT: $fdisplay(debug_log_fd, "    op1: shamt (0x%h)", id_ex_reg.instruction0.shamt[4:0]);
                default: $fdisplay(debug_log_fd, "    op1: unknown");
            endcase
            case (id_ex_reg.ctrl0.operand2_src)
                OPERAND_SOURCE_RS: $fdisplay(debug_log_fd, "    op2: rs (%s)", get_reg_name(id_ex_reg.rs_addr0));
                OPERAND_SOURCE_RT: $fdisplay(debug_log_fd, "    op2: rt (%s)", get_reg_name(id_ex_reg.rt_addr0));
                OPERAND_SOURCE_UNSIGNED_IMM16: $fdisplay(debug_log_fd, "    op2: unsigned_imm16 (0x%h)", id_ex_reg.instruction0.immediate[15:0]);
                OPERAND_SOURCE_SIGNED_IMM16: $fdisplay(debug_log_fd, "    op2: signed_imm16 (0x%h)", id_ex_reg.instruction0.immediate[15:0]);
                OPERAND_SOURCE_UNSIGNED_SHAMT: $fdisplay(debug_log_fd, "    op2: shamt (0x%h)", id_ex_reg.instruction0.shamt[4:0]);
                default: $fdisplay(debug_log_fd, "    op2: unknown");
            endcase
        end

        // Pipeline 0 的 MDU 操作信息
        if (id_ex_reg.ctrl0.mdu_start || id_ex_reg.ctrl0.mdu_use) begin
            print_mdu_info(debug_log_fd, "Pipeline 0",
                id_ex_reg.ctrl0.mdu_operation,
                EX_forwarded_rs0_data,
                EX_forwarded_rt0_data,
                ex_mem_reg.reg_write_data0,
                mdu_busy,
                execution_stage.mdu.remainingCycleCount);
        end
        
        // Pipeline 0 的 syscall 状态信息
        if (mem_wb_reg.ctrl0.is_syscall && !stall_from_ex0) begin
            $fdisplay(debug_log_fd, "\n=== Pipeline 0 SYSCALL Status ===");
            case (mem_wb_reg.rs_data0)  // $v0
                32'd1: $fdisplay(debug_log_fd, "  Syscall 1: print_int(%0d)", $signed(mem_wb_reg.rt_data0));  // $a0
                32'd10: begin
                    $fdisplay(debug_log_fd, "  Syscall 10: exit");
                    $fflush(debug_log_fd);
                    $fclose(debug_log_fd);
                end
                default: $fdisplay(debug_log_fd, "  Unknown syscall: %0d", mem_wb_reg.rs_data0);
            endcase
        end
        
        // Pipeline 1 的 ALU 操作信息
        if (id_ex_reg.ctrl1.alu_op != ALU_NOP && id_ex_reg.valid1) begin
            print_alu_info(debug_log_fd, "Pipeline 1",
                id_ex_reg.ctrl1.alu_op,
                alu_operand1_1,
                alu_operand2_1,
                alu_result1,
                alu_overflow1);
            // 输出操作数来源信息
            $fdisplay(debug_log_fd, "  Operand Sources:");
            case (id_ex_reg.ctrl1.operand1_src)
                OPERAND_SOURCE_RS: $fdisplay(debug_log_fd, "    op1: rs (%s)", get_reg_name(id_ex_reg.rs_addr1));
                OPERAND_SOURCE_RT: $fdisplay(debug_log_fd, "    op1: rt (%s)", get_reg_name(id_ex_reg.rt_addr1));
                OPERAND_SOURCE_UNSIGNED_IMM16: $fdisplay(debug_log_fd, "    op1: unsigned_imm16 (0x%h)", id_ex_reg.instruction1.immediate[15:0]);
                OPERAND_SOURCE_SIGNED_IMM16: $fdisplay(debug_log_fd, "    op1: signed_imm16 (0x%h)", id_ex_reg.instruction1.immediate[15:0]);
                OPERAND_SOURCE_UNSIGNED_SHAMT: $fdisplay(debug_log_fd, "    op1: shamt (0x%h)", id_ex_reg.instruction1.shamt[4:0]);
                default: $fdisplay(debug_log_fd, "    op1: unknown");
            endcase
            case (id_ex_reg.ctrl1.operand2_src)
                OPERAND_SOURCE_RS: $fdisplay(debug_log_fd, "    op2: rs (%s)", get_reg_name(id_ex_reg.rs_addr1));
                OPERAND_SOURCE_RT: $fdisplay(debug_log_fd, "    op2: rt (%s)", get_reg_name(id_ex_reg.rt_addr1));
                OPERAND_SOURCE_UNSIGNED_IMM16: $fdisplay(debug_log_fd, "    op2: unsigned_imm16 (0x%h)", id_ex_reg.instruction1.immediate[15:0]);
                OPERAND_SOURCE_SIGNED_IMM16: $fdisplay(debug_log_fd, "    op2: signed_imm16 (0x%h)", id_ex_reg.instruction1.immediate[15:0]);
                OPERAND_SOURCE_UNSIGNED_SHAMT: $fdisplay(debug_log_fd, "    op2: shamt (0x%h)", id_ex_reg.instruction1.shamt[4:0]);
                default: $fdisplay(debug_log_fd, "    op2: unknown");
            endcase
        end

        // Pipeline 1 的 syscall 状态信息
        if (mem_wb_reg.ctrl1.is_syscall && !stall_from_ex0) begin
            $fdisplay(debug_log_fd, "\n=== Pipeline 1 SYSCALL Status ===");
            case (mem_wb_reg.rs_data1)  // $v0
                32'd1: $fdisplay(debug_log_fd, "  Syscall 1: print_int(%0d)", $signed(mem_wb_reg.rt_data1));  // $a0
                32'd10: begin
                    $fdisplay(debug_log_fd, "  Syscall 10: exit");
                    $fflush(debug_log_fd);
                    $fclose(debug_log_fd);
                end
                default: $fdisplay(debug_log_fd, "  Unknown syscall: %0d", mem_wb_reg.rs_data1);
            endcase
        end
        
        // 在每个周期结束时添加分隔线
        $fdisplay(debug_log_fd, "\n----------------------------------------");
    end

    // 异常处理
    always @(posedge clk) begin
        if (!rst) begin
            handle_pipeline_exception(PipelineStage_IF, if_id_reg.exception0, if_id_reg.pc0, if_id_reg.instruction0, debug_log_fd);
            handle_pipeline_exception(PipelineStage_ID, id_ex_reg.exception0, id_ex_reg.pc0, id_ex_reg.instruction0, debug_log_fd);
            handle_pipeline_exception(PipelineStage_EX, ex_mem_reg.exception0, ex_mem_reg.pc0, ex_mem_reg.instruction0, debug_log_fd);
            handle_pipeline_exception(PipelineStage_MEM, mem_wb_reg.exception0, mem_wb_reg.pc0, mem_wb_reg.instruction0, debug_log_fd);
        end
    end

    // 致命异常处理
    always @(posedge clk) begin
        if (!rst) begin
            fatal_pipeline_exception(PipelineStage_IF, if_id_reg.exception0, if_id_reg.pc0, if_id_reg.instruction0);
            fatal_pipeline_exception(PipelineStage_ID, id_ex_reg.exception0, id_ex_reg.pc0, id_ex_reg.instruction0);
            fatal_pipeline_exception(PipelineStage_EX, ex_mem_reg.exception0, ex_mem_reg.pc0, ex_mem_reg.instruction0);
            fatal_pipeline_exception(PipelineStage_MEM, mem_wb_reg.exception0, mem_wb_reg.pc0, mem_wb_reg.instruction0);
        end
    end

endmodule
