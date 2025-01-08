`timescale 1us / 1us

`include "MipsDefinitions.sv"
`include "common.vh"

module InstructionScheduler(
    input  PartialDecodeResult decode0,  // 第一条指令的部分解码结果
    input  PartialDecodeResult decode1,  // 第二条指令的部分解码结果
    output logic can_dual_issue,         // 是否可以双发射
    output logic has_raw_dependency,     // 是否有 RAW 依赖
    output logic has_waw_dependency,     // 是否有 WAW 依赖
    output logic has_war_dependency      // 是否有 WAR 依赖
);

    // 检查 RAW 依赖
    always @(*) begin
        has_raw_dependency = 0;
        has_waw_dependency = 0;
        has_war_dependency = 0;

        // 如果两条指令都有效
        if (decode0.is_valid && decode1.is_valid) begin
            // 检查 RAW 依赖
            // 第二条指令的源寄存器依赖于第一条指令的目标寄存器
            if (decode0.reg_write && decode0.write_reg_addr != ZERO) begin
                if (decode1.rs_addr == decode0.write_reg_addr ||
                    decode1.rt_addr == decode0.write_reg_addr) begin
                    has_raw_dependency = 1'b1;
                end
            end

            // 检查 WAW 依赖
            // 两条指令写入同一个寄存器
            if (decode0.reg_write && decode1.reg_write &&
                decode0.write_reg_addr == decode1.write_reg_addr &&
                decode0.write_reg_addr != ZERO) begin
                has_waw_dependency = 1'b1;
            end

            // 检查 WAR 依赖
            // 第二条指令写入的寄存器是第一条指令的源寄存器
            if (decode1.reg_write && decode1.write_reg_addr != ZERO) begin
                if (decode0.rs_addr == decode1.write_reg_addr ||
                    decode0.rt_addr == decode1.write_reg_addr) begin
                    has_war_dependency = 1'b1;
                end
            end
        end
    end

    // 判断是否可以双发射
    always @(*) begin
        can_dual_issue = decode0.is_valid && decode1.is_valid &&
                        !has_raw_dependency &&
                        !has_waw_dependency &&
                        !has_war_dependency &&
                        !decode0.is_branch &&  // 第一条不是跳转
                        !decode1.is_branch &&  // 第二条不是跳转
                        !decode0.is_mdu &&
                        !decode1.is_mdu &&
                        !decode0.is_mem_access &&
                        !decode1.is_mem_access; // 访存指令不允许双发射
    end

endmodule