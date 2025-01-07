`timescale 1us / 1us

`include "MipsDefinitions.sv"
`include "common.vh"

module InstructionDecodeStage(
    input  logic clk,
    input  logic rst,
    input  logic stall,
    input  logic flush,
    input  IF_ID_Register if_id_reg,
    input  logic [31:0] forwarded_rs_data,
    input  logic [31:0] forwarded_rt_data,
    output ID_EX_Register id_ex_reg,
    output MipsReg rs_addr,
    output MipsReg rt_addr,
    output ControlSignals ctrl,
    output logic   jump_enable,
    output logic [31:0] jump_addr,
    input  logic prev_is_branch,
    input  logic prev_branch_taken,
    output logic is_branch
);

    logic [31:0] pc_addr;
    Instruction instruction;
    MipsReg reg_write_addr;
    logic [31:0] sign_ext_imm;
    logic [31:0] reg_write_data;

    assign pc_addr = if_id_reg.pc;
    assign instruction = if_id_reg.instruction;

    ControllerUnit ctrl_unit (
        .instruction(instruction),
        .ctrl(ctrl)
    );

    always @(*) begin : get_reg_addr
        `GET_REG_ADDR(rs_addr, ctrl.reg_read_rs_addr_type, instruction);
        `GET_REG_ADDR(rt_addr, ctrl.reg_read_rt_addr_type, instruction);
        `GET_REG_ADDR(reg_write_addr, ctrl.reg_write_addr_type, instruction);
    end

    always @(*) begin : get_sign_ext_imm
        sign_ext_imm = {{16{instruction.immediate[15]}}, instruction.immediate};
    end

    logic jump_cond_result;

    always @(*) begin : jump_cond
        case (ctrl.jump_cond)
            JUMP_COND_TRUE: jump_cond_result = 1;
            JUMP_COND_FALSE: jump_cond_result = 0;
            JUMP_COND_EQ: jump_cond_result = (forwarded_rs_data == forwarded_rt_data);
            JUMP_COND_NE: jump_cond_result = (forwarded_rs_data != forwarded_rt_data);
            JUMP_COND_LEZ: jump_cond_result = ($signed(forwarded_rs_data) <= $signed(0));
            JUMP_COND_GTZ: jump_cond_result = ($signed(forwarded_rs_data) > $signed(0));
            JUMP_COND_GEZ: jump_cond_result = ($signed(forwarded_rs_data) >= $signed(0));
            JUMP_COND_LTZ: jump_cond_result = ($signed(forwarded_rs_data) < $signed(0));
            default: jump_cond_result = 0;
        endcase
    end

    always @(*) begin : check_branch
        is_branch = (ctrl.jump_mode != JUMP_MODE_NEXT);
    end

    always @(*) begin : check_jump_enable
        if (prev_is_branch) begin
            jump_enable = jump_cond_result && !prev_branch_taken;
        end else begin
            jump_enable = jump_cond_result;
        end
    end

    always @(*) begin : get_jump_addr
        case (ctrl.jump_mode)
            JUMP_MODE_REGISTER: jump_addr = forwarded_rs_data;
            JUMP_MODE_RELATIVE: jump_addr = pc_addr + 4 + (sign_ext_imm << 2);
            JUMP_MODE_ABSOLUTE: jump_addr = {pc_addr[31:28], instruction.address, 2'b00};
            default: jump_addr = pc_addr + 4;
        endcase
    end

    always @(*) begin : update_reg_write_data
        case (ctrl.reg_write_data_src)
            REG_WRITE_DATA_SOURCE_PC_PLUS_8: reg_write_data = pc_addr + 8;
            default: reg_write_data = 32'hxxxxxxxx;
        endcase
    end

    always @(posedge clk) begin
        if (rst || flush) begin
            id_ex_reg <= '0;
        end else if (stall) begin
            // do nothing
        end else begin
            id_ex_reg.pc <= pc_addr;
            id_ex_reg.instruction <= instruction;
            id_ex_reg.ctrl <= ctrl;
            id_ex_reg.rs_data <= forwarded_rs_data;
            id_ex_reg.rt_data <= forwarded_rt_data;
            id_ex_reg.sign_ext_imm <= sign_ext_imm;
            id_ex_reg.rs_addr <= rs_addr;
            id_ex_reg.rt_addr <= rt_addr;
            id_ex_reg.reg_write_addr <= reg_write_addr;
            id_ex_reg.reg_write_data <= reg_write_data;
            id_ex_reg.valid <= if_id_reg.valid;
        end
    end

endmodule
