`timescale 1us / 1us

`include "MipsDefinitions.sv"
`include "common.vh"

module MemoryStage(
    input  logic clk,
    input  logic rst,
    input  logic stall0,          // 第一条指令的 stall 信号
    input  logic stall1,          // 第二条指令的 stall 信号
    input  logic flush0,          // 第一条指令的 flush 信号
    input  logic flush1,          // 第二条指令的 flush 信号
    input  EX_MEM_Register ex_mem_reg,
    input  logic [31:0] forwarded_rs_data0,
    input  logic [31:0] forwarded_rt_data0,
    input  logic [31:0] forwarded_rs_data1,
    input  logic [31:0] forwarded_rt_data1,
    output MEM_WB_Register mem_wb_reg,
    output logic [31:0] mem_read_data
);

    // 由于调度器保证了同一周期最多只有一条访存指令
    // 所以我们只需要选择有效的那条访存指令即可
    logic mem_read, mem_write, mem_unsigned;
    logic [1:0] mem_width;
    logic [31:0] mem_addr, mem_write_data;
    logic is_inst1_mem;  // 是否是第二条指令在访存

    // 确定哪条指令在访存
    always @(*) begin : select_mem_instruction
        if (ex_mem_reg.valid1 && (ex_mem_reg.ctrl1.mem_read || ex_mem_reg.ctrl1.mem_write)) begin
            // 第二条指令是访存指令
            mem_addr = ex_mem_reg.alu_result1;
            mem_read = ex_mem_reg.ctrl1.mem_read;
            mem_write = ex_mem_reg.ctrl1.mem_write && !stall1;
            mem_width = ex_mem_reg.ctrl1.mem_width;
            mem_unsigned = ex_mem_reg.ctrl1.mem_unsigned;
            mem_write_data = forwarded_rt_data1;
            is_inst1_mem = 1'b1;
        end else begin
            // 第一条指令是访存指令（或没有访存指令）
            mem_addr = ex_mem_reg.alu_result0;
            mem_read = ex_mem_reg.ctrl0.mem_read;
            mem_write = ex_mem_reg.ctrl0.mem_write && !stall0;
            mem_width = ex_mem_reg.ctrl0.mem_width;
            mem_unsigned = ex_mem_reg.ctrl0.mem_unsigned;
            mem_write_data = forwarded_rt_data0;
            is_inst1_mem = 1'b0;
        end
    end

    DataMemory data_mem(
        .clk(clk),
        .rst(rst),
        .pc(is_inst1_mem ? ex_mem_reg.pc1 : ex_mem_reg.pc0),
        .addr(mem_addr),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .mem_width(mem_width),
        .mem_unsigned(mem_unsigned),
        .write_data(mem_write_data),
        .is_inst1(is_inst1_mem),
        .read_data(mem_read_data)
    );

    logic [31:0] reg_write_data0, reg_write_data1;

    // 第一条指令的写回数据选择
    always @(*) begin : update_reg_write_data0
        case (ex_mem_reg.ctrl0.reg_write_data_src)
            REG_WRITE_DATA_SOURCE_MEM: reg_write_data0 = is_inst1_mem ? ex_mem_reg.reg_write_data0 : mem_read_data;
            default: reg_write_data0 = ex_mem_reg.reg_write_data0;
        endcase
    end

    // 第二条指令的写回数据选择
    always @(*) begin : update_reg_write_data1
        case (ex_mem_reg.ctrl1.reg_write_data_src)
            REG_WRITE_DATA_SOURCE_MEM: reg_write_data1 = is_inst1_mem ? mem_read_data : ex_mem_reg.reg_write_data1;
            default: reg_write_data1 = ex_mem_reg.reg_write_data1;
        endcase
    end

    // 更新 MEM/WB 寄存器
    always @(posedge clk) begin
        if (rst) begin
            mem_wb_reg <= '0;
        end else begin
            // 第一条指令
            if (!stall0) begin
                if (flush0) begin
                    // 完全清空第一条指令的所有信号
                    mem_wb_reg.pc0 <= '0;
                    mem_wb_reg.instruction0 <= '0;
                    mem_wb_reg.ctrl0 <= '0;
                    mem_wb_reg.rs_data0 <= '0;
                    mem_wb_reg.rt_data0 <= '0;
                    mem_wb_reg.reg_write_addr0 <= ZERO;
                    mem_wb_reg.reg_write_data0 <= '0;
                    mem_wb_reg.valid0 <= 1'b0;
                    mem_wb_reg.exception0 <= EXCEPTION_NONE;
                    mem_wb_reg.instr_id0 <= '0;
                end else begin
                    mem_wb_reg.pc0 <= ex_mem_reg.pc0;
                    mem_wb_reg.instruction0 <= ex_mem_reg.instruction0;
                    mem_wb_reg.ctrl0 <= ex_mem_reg.ctrl0;
                    mem_wb_reg.rs_data0 <= forwarded_rs_data0;
                    mem_wb_reg.rt_data0 <= forwarded_rt_data0;
                    mem_wb_reg.reg_write_addr0 <= ex_mem_reg.reg_write_addr0;
                    mem_wb_reg.reg_write_data0 <= reg_write_data0;
                    mem_wb_reg.valid0 <= ex_mem_reg.valid0;
                    mem_wb_reg.exception0 <= ex_mem_reg.exception0;
                    mem_wb_reg.instr_id0 <= ex_mem_reg.instr_id0;
                end
            end

            // 第二条指令
            if (!stall1) begin
                if (flush1) begin
                    // 完全清空第二条指令的所有信号
                    mem_wb_reg.pc1 <= '0;
                    mem_wb_reg.instruction1 <= '0;
                    mem_wb_reg.ctrl1 <= '0;
                    mem_wb_reg.rs_data1 <= '0;
                    mem_wb_reg.rt_data1 <= '0;
                    mem_wb_reg.reg_write_addr1 <= ZERO;
                    mem_wb_reg.reg_write_data1 <= '0;
                    mem_wb_reg.valid1 <= 1'b0;
                    mem_wb_reg.exception1 <= EXCEPTION_NONE;
                    mem_wb_reg.instr_id1 <= '0;
                end else begin
                    mem_wb_reg.pc1 <= ex_mem_reg.pc1;
                    mem_wb_reg.instruction1 <= ex_mem_reg.instruction1;
                    mem_wb_reg.ctrl1 <= ex_mem_reg.ctrl1;
                    mem_wb_reg.rs_data1 <= forwarded_rs_data1;
                    mem_wb_reg.rt_data1 <= forwarded_rt_data1;
                    mem_wb_reg.reg_write_addr1 <= ex_mem_reg.reg_write_addr1;
                    mem_wb_reg.reg_write_data1 <= reg_write_data1;
                    mem_wb_reg.valid1 <= ex_mem_reg.valid1;
                    mem_wb_reg.exception1 <= ex_mem_reg.exception1;
                    mem_wb_reg.instr_id1 <= ex_mem_reg.instr_id1;
                end
            end
        end
    end

endmodule
