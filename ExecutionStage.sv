`timescale 1us / 1us

`include "MipsDefinitions.sv"
`include "common.vh"

module OperandMux(
    input  OperandSource source,
    input  logic [31:0] rs_data,
    input  logic [31:0] rt_data,
    input  Instruction instruction,
    output logic [31:0] operand
);

    always @(*) begin : get_operand
        case (source)
            OPERAND_SOURCE_RS: operand = rs_data;
            OPERAND_SOURCE_RT: operand = rt_data;
            OPERAND_SOURCE_UNSIGNED_IMM16: operand = {16'b0, instruction.immediate[15:0]};
            OPERAND_SOURCE_SIGNED_IMM16: operand = {{16{instruction.immediate[15]}}, instruction.immediate[15:0]};
            OPERAND_SOURCE_UNSIGNED_SHAMT: operand = {27'b0, instruction.shamt[4:0]};
            default: operand = 32'b0;
        endcase
    end

endmodule

module ExecutionStage(
    input  logic clk,
    input  logic rst,
    input  logic stall0,          // 第一条指令的 stall 信号
    input  logic stall1,          // 第二条指令的 stall 信号
    input  logic flush0,          // 第一条指令的 flush 信号
    input  logic flush1,          // 第二条指令的 flush 信号
    input  ID_EX_Register id_ex_reg,
    input  logic [31:0] forwarded_rs_data0,
    input  logic [31:0] forwarded_rt_data0,
    input  logic [31:0] forwarded_rs_data1,
    input  logic [31:0] forwarded_rt_data1,
    output EX_MEM_Register ex_mem_reg,
    output logic mdu_busy,
    output logic [31:0] alu_result0_out,
    output logic alu_overflow0_out,
    output logic [31:0] alu_result1_out,
    output logic alu_overflow1_out,
    output logic [31:0] operand1_0_out,
    output logic [31:0] operand2_0_out,
    output logic [31:0] operand1_1_out,
    output logic [31:0] operand2_1_out
);

    logic [31:0] operand1_0, operand1_1;
    logic [31:0] operand2_0, operand2_1;
    logic [31:0] alu_result_0, alu_result_1;
    logic zero_0, zero_1, overflow_0, overflow_1;

    // MDU signals
    logic [31:0] mdu_data_read;

    // 第一条指令的操作数选择
    OperandMux operand1_mux_0(
        .source(id_ex_reg.ctrl0.operand1_src),
        .rs_data(forwarded_rs_data0),
        .rt_data(forwarded_rt_data0),
        .instruction(id_ex_reg.instruction0),
        .operand(operand1_0)
    );

    OperandMux operand2_mux_0(
        .source(id_ex_reg.ctrl0.operand2_src),
        .rs_data(forwarded_rs_data0),
        .rt_data(forwarded_rt_data0),
        .instruction(id_ex_reg.instruction0),
        .operand(operand2_0)
    );

    // 第二条指令的操作数选择
    OperandMux operand1_mux_1(
        .source(id_ex_reg.ctrl1.operand1_src),
        .rs_data(forwarded_rs_data1),
        .rt_data(forwarded_rt_data1),
        .instruction(id_ex_reg.instruction1),
        .operand(operand1_1)
    );

    OperandMux operand2_mux_1(
        .source(id_ex_reg.ctrl1.operand2_src),
        .rs_data(forwarded_rs_data1),
        .rt_data(forwarded_rt_data1),
        .instruction(id_ex_reg.instruction1),
        .operand(operand2_1)
    );

    // 第一条指令的 ALU
    ArithmeticLogicUnit alu_0(
        .operand1(operand1_0),
        .operand2(operand2_0),
        .alu_op(id_ex_reg.ctrl0.alu_op),
        .result(alu_result_0),
        .zero(zero_0),
        .overflow(overflow_0)
    );

    // 第二条指令的 ALU
    ArithmeticLogicUnit alu_1(
        .operand1(operand1_1),
        .operand2(operand2_1),
        .alu_op(id_ex_reg.ctrl1.alu_op),
        .result(alu_result_1),
        .zero(zero_1),
        .overflow(overflow_1)
    );

    // 共享的 MDU（只处理第一条指令）
    MultiplicationDivisionUnit mdu(
        .reset(rst),
        .clock(clk),
        .operand1(operand1_0),
        .operand2(operand2_0),
        .operation(id_ex_reg.ctrl0.mdu_operation),
        .start(id_ex_reg.ctrl0.mdu_start && !stall0),
        .busy(mdu_busy),
        .dataRead(mdu_data_read)
    );

    logic [31:0] reg_write_data_0, reg_write_data_1;

    // 第一条指令的写回数据选择
    always @(*) begin : update_reg_write_data_0
        case (id_ex_reg.ctrl0.reg_write_data_src)
            REG_WRITE_DATA_SOURCE_ALU: reg_write_data_0 = alu_result_0;
            REG_WRITE_DATA_SOURCE_MDU: reg_write_data_0 = mdu_data_read;
            default: reg_write_data_0 = id_ex_reg.reg_write_data0;
        endcase
    end

    // 第二条指令的写回数据选择
    always @(*) begin : update_reg_write_data_1
        case (id_ex_reg.ctrl1.reg_write_data_src)
            REG_WRITE_DATA_SOURCE_ALU: reg_write_data_1 = alu_result_1;
            default: reg_write_data_1 = id_ex_reg.reg_write_data1;
        endcase
    end

    // 合并异常信号
    ExceptionType exception_0, exception_1;
    always @(*) begin : update_exception_0
        if (id_ex_reg.exception0 != EXCEPTION_NONE)
            exception_0 = id_ex_reg.exception0;
        else if (overflow_0 && (id_ex_reg.ctrl0.alu_op == ALU_ADD || id_ex_reg.ctrl0.alu_op == ALU_SUB))
            exception_0 = EXCEPTION_INTEGER_OVERFLOW;
        else
            exception_0 = EXCEPTION_NONE;
    end

    always @(*) begin : update_exception_1
        if (id_ex_reg.exception1 != EXCEPTION_NONE)
            exception_1 = id_ex_reg.exception1;
        else if (overflow_1 && (id_ex_reg.ctrl1.alu_op == ALU_ADD || id_ex_reg.ctrl1.alu_op == ALU_SUB))
            exception_1 = EXCEPTION_INTEGER_OVERFLOW;
        else
            exception_1 = EXCEPTION_NONE;
    end

    // 更新 EX/MEM 寄存器
    always @(posedge clk) begin
        if (rst) begin
            ex_mem_reg <= '0;
        end else begin
            // 第一条指令
            if (!stall0) begin
                if (flush0) begin
                    // 完全清空第一条指令的所有信号
                    ex_mem_reg.pc0 <= '0;
                    ex_mem_reg.instruction0 <= '0;
                    ex_mem_reg.ctrl0 <= '0;
                    ex_mem_reg.rs_data0 <= '0;
                    ex_mem_reg.rt_data0 <= '0;
                    ex_mem_reg.alu_result0 <= '0;
                    ex_mem_reg.reg_write_addr0 <= ZERO;
                    ex_mem_reg.reg_write_data0 <= '0;
                    ex_mem_reg.valid0 <= 1'b0;
                    ex_mem_reg.exception0 <= EXCEPTION_NONE;
                    ex_mem_reg.instr_id0 <= '0;
                end else begin
                    ex_mem_reg.pc0 <= id_ex_reg.pc0;
                    ex_mem_reg.instruction0 <= id_ex_reg.instruction0;
                    ex_mem_reg.ctrl0 <= id_ex_reg.ctrl0;
                    ex_mem_reg.rs_data0 <= forwarded_rs_data0;
                    ex_mem_reg.rt_data0 <= forwarded_rt_data0;
                    ex_mem_reg.alu_result0 <= alu_result_0;
                    ex_mem_reg.reg_write_addr0 <= id_ex_reg.reg_write_addr0;
                    ex_mem_reg.reg_write_data0 <= reg_write_data_0;
                    ex_mem_reg.valid0 <= id_ex_reg.valid0;
                    ex_mem_reg.exception0 <= id_ex_reg.exception0;
                    ex_mem_reg.instr_id0 <= id_ex_reg.instr_id0;
                end
            end

            // 第二条指令
            if (!stall1) begin
                if (flush1) begin
                    // 完全清空第二条指令的所有信号
                    ex_mem_reg.pc1 <= '0;
                    ex_mem_reg.instruction1 <= '0;
                    ex_mem_reg.ctrl1 <= '0;
                    ex_mem_reg.rs_data1 <= '0;
                    ex_mem_reg.rt_data1 <= '0;
                    ex_mem_reg.alu_result1 <= '0;
                    ex_mem_reg.reg_write_addr1 <= ZERO;
                    ex_mem_reg.reg_write_data1 <= '0;
                    ex_mem_reg.valid1 <= 1'b0;
                    ex_mem_reg.exception1 <= EXCEPTION_NONE;
                    ex_mem_reg.instr_id1 <= '0;
                end else begin
                    ex_mem_reg.pc1 <= id_ex_reg.pc1;
                    ex_mem_reg.instruction1 <= id_ex_reg.instruction1;
                    ex_mem_reg.ctrl1 <= id_ex_reg.ctrl1;
                    ex_mem_reg.rs_data1 <= forwarded_rs_data1;
                    ex_mem_reg.rt_data1 <= forwarded_rt_data1;
                    ex_mem_reg.alu_result1 <= alu_result_1;
                    ex_mem_reg.reg_write_addr1 <= id_ex_reg.reg_write_addr1;
                    ex_mem_reg.reg_write_data1 <= reg_write_data_1;
                    ex_mem_reg.valid1 <= id_ex_reg.valid1;
                    ex_mem_reg.exception1 <= id_ex_reg.exception1;
                    ex_mem_reg.instr_id1 <= id_ex_reg.instr_id1;
                end
            end
        end
    end

    assign alu_result0_out = alu_result_0;
    assign alu_overflow0_out = overflow_0;
    assign alu_result1_out = alu_result_1;
    assign alu_overflow1_out = overflow_1;

    // 添加操作数输出
    assign operand1_0_out = operand1_0;
    assign operand2_0_out = operand2_0;
    assign operand1_1_out = operand1_1;
    assign operand2_1_out = operand2_1;

endmodule