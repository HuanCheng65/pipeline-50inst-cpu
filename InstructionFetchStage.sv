`timescale 1us / 1us

`include "MipsDefinitions.sv"
`include "common.vh"

module InstructionFetchStage(
    input  logic clk,
    input  logic rst,
    input  logic stall0,          // 第一条指令的 stall 信号
    input  logic stall1,          // 第二条指令的 stall 信号
    input  logic flush0,          // 第一条指令的 flush 信号
    input  logic flush1,          // 第二条指令的 flush 信号
    input  logic jump_enable,     // 跳转使能
    input  logic [31:0] jump_address,  // 跳转地址
    output IF_ID_Register if_id_reg,
    output logic [31:0] pc,
    output logic [31:0] instr_id0,
    output logic [31:0] instr_id1,
    output Instruction instruction0,
    output Instruction instruction1,
    output logic has_raw_dependency,     // 是否有 RAW 依赖
    output logic has_waw_dependency,     // 是否有 WAW 依赖
    output logic has_war_dependency      // 是否有 WAR 依赖
);

    // 指令 ID 计数器
    logic [31:0] next_instr_id;

    assign instr_id0 = next_instr_id;
    assign instr_id1 = next_instr_id + 1;
    
    // 初始化和更新指令 ID 计数器
    always @(posedge clk) begin
        if (rst) begin
            next_instr_id <= 0;
        end else if (!stall0) begin
            if (jump_enable) begin
                next_instr_id <= next_instr_id + 1;
            end else if (can_dual_issue && !stall1) begin
                // 双发射时增加2
                next_instr_id <= next_instr_id + 2;
            end else begin
                // 单发射时增加1
                next_instr_id <= next_instr_id + 1;
            end
        end
    end

    // PC 寄存器
    always @(posedge clk) begin
        if (rst) begin
            pc <= 32'h0000_3000;
        end else if (!stall0) begin
            if (jump_enable)
                pc <= jump_address;
            else if (can_dual_issue && !stall1)  // 如果可以双发射且第二条指令不停顿
                pc <= pc + 8;  // 双发射时 PC+8
            else
                pc <= pc + 4;  // 单发射时 PC+4
        end
    end

    // 指令存储器
    InstructionMemory instruction_memory(
        .pc(pc),
        .instruction0(instruction0),
        .instruction1(instruction1)
    );

    // 部分解码单元
    PartialDecodeResult decode0, decode1;
    
    PartialDecoder decoder0(
        .instruction(instruction0),
        .decode_result(decode0)
    );

    PartialDecoder decoder1(
        .instruction(instruction1),
        .decode_result(decode1)
    );

    // 指令调度单元
    logic can_dual_issue;
    
    InstructionScheduler scheduler(
        .decode0(decode0),
        .decode1(decode1),
        .can_dual_issue(can_dual_issue),
        .has_raw_dependency(has_raw_dependency),
        .has_waw_dependency(has_waw_dependency),
        .has_war_dependency(has_war_dependency)
    );

    // 更新 IF/ID 寄存器
    always @(posedge clk) begin
        if (rst) begin
            if_id_reg <= '0;
        end else begin
            // 第一条指令
            if (!stall0) begin
                if (flush0) begin
                    if_id_reg.instruction0 <= '0;
                    if_id_reg.valid0 <= 1'b0;
                    if_id_reg.exception0 <= EXCEPTION_NONE;
                    if_id_reg.instr_id0 <= '0;
                end else begin
                    if_id_reg.pc0 <= pc;
                    if_id_reg.instruction0 <= instruction0;
                    if_id_reg.valid0 <= decode0.is_valid;
                    if_id_reg.exception0 <= EXCEPTION_NONE;
                    if_id_reg.instr_id0 <= next_instr_id;
                end
            end

            // 第二条指令
            if (!stall1) begin
                if (flush1) begin
                    if_id_reg.instruction1 <= '0;
                    if_id_reg.valid1 <= 1'b0;
                    if_id_reg.exception1 <= EXCEPTION_NONE;
                    if_id_reg.instr_id1 <= '0;
                end else begin
                    if_id_reg.pc1 <= pc + 4;
                    if_id_reg.instruction1 <= instruction1;
                    // 只有在可以双发射时第二条指令才有效
                    if_id_reg.valid1 <= decode1.is_valid && can_dual_issue;
                    if_id_reg.exception1 <= EXCEPTION_NONE;
                    if_id_reg.instr_id1 <= next_instr_id + 1;  // 第二条指令的ID为当前ID+1
                end
            end
        end
    end

endmodule 
