`timescale 1us / 1us

`include "MipsDefinitions.sv"
`include "common.vh"

module OperandMux(
    input  OperandSource source,
    input  logic [31:0] forwarded_rs_data,
    input  logic [31:0] forwarded_rt_data,
    input  ID_EX_Register id_ex_reg,
    output logic [31:0] operand
);

    always @(*) begin : get_operand
        case (source)
            OPERAND_SOURCE_RS: operand = forwarded_rs_data;
            OPERAND_SOURCE_RT: operand = forwarded_rt_data;
            OPERAND_SOURCE_UNSIGNED_IMM16: operand = {16'b0, id_ex_reg.instruction.immediate[15:0]};
            OPERAND_SOURCE_SIGNED_IMM16: operand = {{16{id_ex_reg.instruction.immediate[15]}}, id_ex_reg.instruction.immediate[15:0]};
            OPERAND_SOURCE_UNSIGNED_SHAMT: operand = {27'b0, id_ex_reg.instruction.shamt[4:0]};
            default: operand = 32'b0;
        endcase
    end

endmodule

module ExecutionStage(
    input  logic clk,
    input  logic rst,
    input  logic stall_rs,
    input  logic stall_rt,
    input  logic stall,
    input  logic flush,
    input  ID_EX_Register id_ex_reg,
    input  logic [31:0] forwarded_rs_data,
    input  logic [31:0] forwarded_rt_data,
    output EX_MEM_Register ex_mem_reg,
    output logic mdu_busy,
    output logic [31:0] alu_result_out,
    output logic alu_overflow_out
);

    logic [31:0] operand1;
    logic [31:0] operand2;
    logic [31:0] alu_result;
    logic zero, overflow;

    // MDU signals
    logic [31:0] mdu_data_read;

    OperandMux operand1_mux(
        .source(id_ex_reg.ctrl.operand1_src),
        .forwarded_rs_data(forwarded_rs_data),
        .forwarded_rt_data(forwarded_rt_data),
        .id_ex_reg(id_ex_reg),
        .operand(operand1)
    );

    OperandMux operand2_mux(
        .source(id_ex_reg.ctrl.operand2_src),
        .forwarded_rs_data(forwarded_rs_data),
        .forwarded_rt_data(forwarded_rt_data),
        .id_ex_reg(id_ex_reg),
        .operand(operand2)
    );

    ArithmeticLogicUnit alu(
        .operand1(operand1),
        .operand2(operand2),
        .alu_op(id_ex_reg.ctrl.alu_op),
        .result(alu_result),
        .zero(zero),
        .overflow(overflow)
    );

    MultiplicationDivisionUnit mdu(
        .reset(rst),
        .clock(clk),
        .operand1(operand1),
        .operand2(operand2),
        .operation(id_ex_reg.ctrl.mdu_operation),
        .start(id_ex_reg.ctrl.mdu_start && !stall),
        .busy(mdu_busy),
        .dataRead(mdu_data_read)
    );

    logic [31:0] reg_write_data;

    always @(*) begin : update_reg_write_data
        case (id_ex_reg.ctrl.reg_write_data_src)
            REG_WRITE_DATA_SOURCE_ALU: reg_write_data = alu_result;
            REG_WRITE_DATA_SOURCE_MDU: reg_write_data = mdu_data_read;
            default: reg_write_data = id_ex_reg.reg_write_data;
        endcase
    end

    // 合并异常信号
    ExceptionType exception;
    always @(*) begin
        // 优先传递已有的异常
        if (id_ex_reg.exception != EXCEPTION_NONE)
            exception = id_ex_reg.exception;
        // 其次检查是否发生整数溢出
        else if (overflow && (id_ex_reg.ctrl.alu_op == ALU_ADD || id_ex_reg.ctrl.alu_op == ALU_SUB))
            exception = EXCEPTION_INTEGER_OVERFLOW;
        else
            exception = EXCEPTION_NONE;
    end

    always @(posedge clk) begin : update_ex_mem_reg
        if (rst || flush) begin
            ex_mem_reg <= '0;
        end else if (stall) begin
            // do nothing
        end else begin
            ex_mem_reg.pc <= id_ex_reg.pc;
            ex_mem_reg.instruction <= id_ex_reg.instruction;
            ex_mem_reg.ctrl <= id_ex_reg.ctrl;
            ex_mem_reg.rs_addr <= id_ex_reg.rs_addr;
            ex_mem_reg.rt_addr <= id_ex_reg.rt_addr;
            ex_mem_reg.rs_data <= forwarded_rs_data;
            ex_mem_reg.rt_data <= forwarded_rt_data;
            ex_mem_reg.alu_result <= alu_result;
            ex_mem_reg.reg_write_addr <= id_ex_reg.reg_write_addr;
            ex_mem_reg.reg_write_data <= reg_write_data;
            ex_mem_reg.exception <= exception;  // 传递异常信号
            ex_mem_reg.valid <= id_ex_reg.valid;
        end
    end

    assign alu_result_out = alu_result;
    assign alu_overflow_out = overflow;

endmodule