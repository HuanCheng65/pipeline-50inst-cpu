`timescale 1us / 1us

`include "MipsDefinitions.sv"
`include "common.vh"

module PartialDecoder(
    input  Instruction instruction,
    output PartialDecodeResult decode_result
);

    // 指令类型判断
    wire [5:0] opcode = instruction.opcode;
    wire [5:0] funct = instruction.funct;
    wire [4:0] rt = instruction.rt;

    always @(*) begin
        // 默认值
        decode_result = '0;

        // 检查是否是有效指令（不是 NOP）
        decode_result.is_valid = 1'b1;

        case (opcode)
            OP_RTYPE: begin
                case (funct)
                    // R 型算术和逻辑指令
                    FUNCT_ADD, FUNCT_ADDU, FUNCT_SUB, FUNCT_SUBU,
                    FUNCT_AND, FUNCT_OR, FUNCT_XOR, FUNCT_NOR,
                    FUNCT_SLT, FUNCT_SLTU: begin
                        decode_result.reg_write = 1'b1;
                        decode_result.rs_addr = instruction.rs;
                        decode_result.rt_addr = instruction.rt;
                        decode_result.write_reg_addr = instruction.rd;
                    end

                    // 移位指令（固定移位量）
                    FUNCT_SLL, FUNCT_SRL, FUNCT_SRA: begin
                        decode_result.reg_write = 1'b1;
                        decode_result.rt_addr = instruction.rt;
                        decode_result.write_reg_addr = instruction.rd;
                    end

                    // 移位指令（变量移位量）
                    FUNCT_SLLV, FUNCT_SRLV, FUNCT_SRAV: begin
                        decode_result.reg_write = 1'b1;
                        decode_result.rs_addr = instruction.rs;
                        decode_result.rt_addr = instruction.rt;
                        decode_result.write_reg_addr = instruction.rd;
                    end

                    // 跳转指令
                    FUNCT_JR: begin
                        decode_result.is_branch = 1'b1;
                        decode_result.rs_addr = instruction.rs;
                    end

                    FUNCT_JALR: begin
                        decode_result.is_branch = 1'b1;
                        decode_result.reg_write = 1'b1;
                        decode_result.rs_addr = instruction.rs;
                        decode_result.write_reg_addr = instruction.rd;
                    end

                    // 乘除法指令
                    FUNCT_MULT, FUNCT_MULTU, FUNCT_DIV, FUNCT_DIVU: begin
                        decode_result.is_mdu = 1'b1;
                        decode_result.rs_addr = instruction.rs;
                        decode_result.rt_addr = instruction.rt;
                    end

                    // HI/LO 寄存器访问指令
                    FUNCT_MFHI, FUNCT_MFLO: begin
                        decode_result.is_mdu = 1'b1;
                        decode_result.reg_write = 1'b1;
                        decode_result.write_reg_addr = instruction.rd;
                    end

                    FUNCT_MTHI, FUNCT_MTLO: begin
                        decode_result.is_mdu = 1'b1;
                        decode_result.rs_addr = instruction.rs;
                    end

                    // SYSCALL 指令
                    FUNCT_SYSCALL: begin
                        decode_result.rs_addr = V0;  // $v0
                        decode_result.rt_addr = A0;  // $a0
                    end
                endcase
            end

            // I 型算术和逻辑指令
            OP_ADDI, OP_ADDIU, OP_SLTI, OP_SLTIU,
            OP_ANDI, OP_ORI, OP_XORI: begin
                decode_result.reg_write = 1'b1;
                decode_result.rs_addr = instruction.rs;
                decode_result.write_reg_addr = instruction.rt;
            end

            // 加载立即数
            OP_LUI: begin
                decode_result.reg_write = 1'b1;
                decode_result.write_reg_addr = instruction.rt;
            end

            // 分支指令
            OP_BEQ, OP_BNE: begin
                decode_result.is_branch = 1'b1;
                decode_result.rs_addr = instruction.rs;
                decode_result.rt_addr = instruction.rt;
            end

            OP_BLEZ, OP_BGTZ: begin
                decode_result.is_branch = 1'b1;
                decode_result.rs_addr = instruction.rs;
            end

            OP_REGIMM: begin
                case (rt)
                    RT_BLTZ, RT_BGEZ: begin
                        decode_result.is_branch = 1'b1;
                        decode_result.rs_addr = instruction.rs;
                    end
                endcase
            end

            // 跳转指令
            OP_J: begin
                decode_result.is_branch = 1'b1;
            end

            OP_JAL: begin
                decode_result.is_branch = 1'b1;
                decode_result.reg_write = 1'b1;
                decode_result.write_reg_addr = RA;  // $ra
            end

            // 访存指令
            OP_LW, OP_LB, OP_LBU, OP_LH, OP_LHU: begin
                decode_result.is_mem_access = 1'b1;
                decode_result.reg_write = 1'b1;
                decode_result.rs_addr = instruction.rs;
                decode_result.write_reg_addr = instruction.rt;
            end

            OP_SW, OP_SB, OP_SH: begin
                decode_result.is_mem_access = 1'b1;
                decode_result.rs_addr = instruction.rs;
                decode_result.rt_addr = instruction.rt;
            end
        endcase
    end

endmodule 