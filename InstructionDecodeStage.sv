`timescale 1us / 1us

`include "MipsDefinitions.sv"
`include "common.vh"

module InstructionDecodeStage(
    input  logic clk,
    input  logic rst,
    input  logic stall0,          // 第一条指令的 stall 信号
    input  logic stall1,          // 第二条指令的 stall 信号
    input  logic flush0,          // 第一条指令的 flush 信号
    input  logic flush1,          // 第二条指令的 flush 信号
    input  IF_ID_Register if_id_reg,
    input  logic [31:0] rs_data0,  // 第一条指令的 rs 数据
    input  logic [31:0] rt_data0,  // 第一条指令的 rt 数据
    input  logic [31:0] rs_data1,  // 第二条指令的 rs 数据
    input  logic [31:0] rt_data1,  // 第二条指令的 rt 数据
    output ID_EX_Register id_ex_reg,
    output MipsReg rs_addr0,
    output MipsReg rt_addr0,
    output MipsReg rs_addr1,
    output MipsReg rt_addr1,
    output ControlSignals ctrl0,
    output ControlSignals ctrl1,
    output logic   jump_enable,
    output logic [31:0] jump_addr,
    input  logic prev_is_branch,
    input  logic prev_branch_taken,
    output logic is_branch
);

    // 第一条指令的信号
    logic [31:0] pc_addr0;
    Instruction instruction0;
    MipsReg reg_write_addr0;
    logic [31:0] sign_ext_imm0;
    logic [31:0] reg_write_data0;

    // 第二条指令的信号
    logic [31:0] pc_addr1;
    Instruction instruction1;
    MipsReg reg_write_addr1;
    logic [31:0] sign_ext_imm1;
    logic [31:0] reg_write_data1;

    assign pc_addr0 = if_id_reg.pc0;
    assign pc_addr1 = if_id_reg.pc1;
    assign instruction0 = if_id_reg.instruction0;
    assign instruction1 = if_id_reg.instruction1;

    // 第一条指令的控制单元
    ControllerUnit ctrl_unit0 (
        .instruction(instruction0),
        .ctrl(ctrl0)
    );

    // 第二条指令的控制单元
    ControllerUnit ctrl_unit1 (
        .instruction(instruction1),
        .ctrl(ctrl1)
    );

    // 第一条指令的寄存器地址解码
    always @(*) begin : get_reg_addr0
        `GET_REG_ADDR(rs_addr0, ctrl0.reg_read_rs_addr_type, instruction0);
        `GET_REG_ADDR(rt_addr0, ctrl0.reg_read_rt_addr_type, instruction0);
        `GET_REG_ADDR(reg_write_addr0, ctrl0.reg_write_addr_type, instruction0);
    end

    // 第二条指令的寄存器地址解码
    always @(*) begin : get_reg_addr1
        `GET_REG_ADDR(rs_addr1, ctrl1.reg_read_rs_addr_type, instruction1);
        `GET_REG_ADDR(rt_addr1, ctrl1.reg_read_rt_addr_type, instruction1);
        `GET_REG_ADDR(reg_write_addr1, ctrl1.reg_write_addr_type, instruction1);
    end

    // 第一条指令的立即数符号扩展
    always @(*) begin : get_sign_ext_imm0
        sign_ext_imm0 = {{16{instruction0.immediate[15]}}, instruction0.immediate};
    end

    // 第二条指令的立即数符号扩展
    always @(*) begin : get_sign_ext_imm1
        sign_ext_imm1 = {{16{instruction1.immediate[15]}}, instruction1.immediate};
    end

    logic jump_cond_result;

    // 暂时只使用第一条指令的跳转逻辑
    always @(*) begin : jump_cond
        case (ctrl0.jump_cond)
            JUMP_COND_TRUE: jump_cond_result = 1;
            JUMP_COND_FALSE: jump_cond_result = 0;
            JUMP_COND_EQ: jump_cond_result = (rs_data0 == rt_data0);
            JUMP_COND_NE: jump_cond_result = (rs_data0 != rt_data0);
            JUMP_COND_LEZ: jump_cond_result = ($signed(rs_data0) <= $signed(0));
            JUMP_COND_GTZ: jump_cond_result = ($signed(rs_data0) > $signed(0));
            JUMP_COND_GEZ: jump_cond_result = ($signed(rs_data0) >= $signed(0));
            JUMP_COND_LTZ: jump_cond_result = ($signed(rs_data0) < $signed(0));
            default: jump_cond_result = 0;
        endcase
    end

    // 暂时只使用第一条指令的分支判断
    always @(*) begin : check_branch
        is_branch = (ctrl0.jump_mode != JUMP_MODE_NEXT);
    end

    // 暂时只使用第一条指令的跳转使能
    always @(*) begin : check_jump_enable
        if (prev_is_branch) begin
            jump_enable = jump_cond_result && !prev_branch_taken;
        end else begin
            jump_enable = jump_cond_result;
        end
    end

    // 暂时只使用第一条指令的跳转地址
    always @(*) begin : get_jump_addr
        case (ctrl0.jump_mode)
            JUMP_MODE_REGISTER: jump_addr = rs_data0;
            JUMP_MODE_RELATIVE: jump_addr = pc_addr0 + 4 + (sign_ext_imm0 << 2);
            JUMP_MODE_ABSOLUTE: jump_addr = {pc_addr0[31:28], instruction0.address, 2'b00};
            default: jump_addr = pc_addr0 + 4;
        endcase
    end

    // 第一条指令的寄存器写回数据
    always @(*) begin : update_reg_write_data0
        case (ctrl0.reg_write_data_src)
            REG_WRITE_DATA_SOURCE_PC_PLUS_8: reg_write_data0 = pc_addr0 + 8;
            default: reg_write_data0 = 32'hxxxxxxxx;
        endcase
    end

    // 第二条指令的寄存器写回数据
    always @(*) begin : update_reg_write_data1
        case (ctrl1.reg_write_data_src)
            REG_WRITE_DATA_SOURCE_PC_PLUS_8: reg_write_data1 = pc_addr1 + 8;
            default: reg_write_data1 = 32'hxxxxxxxx;
        endcase
    end

    // 指令调度单元
    PartialDecodeResult decode0, decode1;
    
    PartialDecoder decoder0(
        .instruction(if_id_reg.instruction0),
        .decode_result(decode0)
    );

    PartialDecoder decoder1(
        .instruction(if_id_reg.instruction1),
        .decode_result(decode1)
    );

    InstructionScheduler scheduler(
        .decode0(decode0),
        .decode1(decode1),
        .can_dual_issue(can_dual_issue),
        .has_raw_dependency(has_raw_dependency),
        .has_waw_dependency(has_waw_dependency),
        .has_war_dependency(has_war_dependency)
    );

    // 更新 ID/EX 寄存器
    always @(posedge clk) begin
        if (rst) begin
            id_ex_reg <= '0;
        end else begin
            // 第一条指令
            if (!stall0) begin
                if (flush0) begin
                    // 完全清空第一条指令的所有信号
                    id_ex_reg.pc0 <= '0;
                    id_ex_reg.instruction0 <= '0;
                    id_ex_reg.ctrl0 <= '0;
                    id_ex_reg.rs_data0 <= '0;
                    id_ex_reg.rt_data0 <= '0;
                    id_ex_reg.sign_ext_imm0 <= '0;
                    id_ex_reg.rs_addr0 <= ZERO;
                    id_ex_reg.rt_addr0 <= ZERO;
                    id_ex_reg.reg_write_addr0 <= ZERO;
                    id_ex_reg.reg_write_data0 <= '0;
                    id_ex_reg.valid0 <= 1'b0;
                    id_ex_reg.exception0 <= EXCEPTION_NONE;
                    id_ex_reg.instr_id0 <= '0;
                end else begin
                    id_ex_reg.pc0 <= pc_addr0;
                    id_ex_reg.instruction0 <= instruction0;
                    id_ex_reg.ctrl0 <= ctrl0;
                    id_ex_reg.rs_data0 <= rs_data0;
                    id_ex_reg.rt_data0 <= rt_data0;
                    id_ex_reg.sign_ext_imm0 <= sign_ext_imm0;
                    id_ex_reg.rs_addr0 <= rs_addr0;
                    id_ex_reg.rt_addr0 <= rt_addr0;
                    id_ex_reg.reg_write_addr0 <= reg_write_addr0;
                    id_ex_reg.reg_write_data0 <= reg_write_data0;
                    id_ex_reg.valid0 <= if_id_reg.valid0;
                    id_ex_reg.exception0 <= if_id_reg.exception0;
                    id_ex_reg.instr_id0 <= if_id_reg.instr_id0;
                end
            end

            // 第二条指令
            if (!stall1) begin
                if (flush1) begin
                    // 完全清空第二条指令的所有信号
                    id_ex_reg.pc1 <= '0;
                    id_ex_reg.instruction1 <= '0;
                    id_ex_reg.ctrl1 <= '0;
                    id_ex_reg.rs_data1 <= '0;
                    id_ex_reg.rt_data1 <= '0;
                    id_ex_reg.sign_ext_imm1 <= '0;
                    id_ex_reg.rs_addr1 <= ZERO;
                    id_ex_reg.rt_addr1 <= ZERO;
                    id_ex_reg.reg_write_addr1 <= ZERO;
                    id_ex_reg.reg_write_data1 <= '0;
                    id_ex_reg.valid1 <= 1'b0;
                    id_ex_reg.exception1 <= EXCEPTION_NONE;
                    id_ex_reg.instr_id1 <= '0;
                end else begin
                    id_ex_reg.pc1 <= pc_addr1;
                    id_ex_reg.instruction1 <= instruction1;
                    id_ex_reg.ctrl1 <= ctrl1;
                    id_ex_reg.rs_data1 <= rs_data1;
                    id_ex_reg.rt_data1 <= rt_data1;
                    id_ex_reg.sign_ext_imm1 <= sign_ext_imm1;
                    id_ex_reg.rs_addr1 <= rs_addr1;
                    id_ex_reg.rt_addr1 <= rt_addr1;
                    id_ex_reg.reg_write_addr1 <= reg_write_addr1;
                    id_ex_reg.reg_write_data1 <= reg_write_data1;
                    // 如果第一条指令是跳转指令且条件成立，或者不能双发射，第二条指令无效
                    id_ex_reg.valid1 <= if_id_reg.valid1 && can_dual_issue && !jump_enable;
                    id_ex_reg.exception1 <= if_id_reg.exception1;
                    id_ex_reg.instr_id1 <= if_id_reg.instr_id1;
                end
            end
        end
    end

endmodule
