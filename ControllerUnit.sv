`timescale 1us / 1us

`include "MipsDefinitions.sv"
`include "MultiplicationDivisionUnit.sv"

module ControllerUnit (
    input  Instruction instruction,
    output ControlSignals ctrl
);

    wire [5:0] funct = instruction.funct;    // Function code

    always @(*) begin
        ctrl = '0;
        ctrl.exception = EXCEPTION_NONE;  // 默认无异常

        casex (instruction.instruction)
            ADD, ADDU, SUB, SUBU, AND, OR, XOR, NOR, SLT, SLTU: begin
                ctrl.reg_read_rs_addr_type = REG_ADDR_TYPE_RS;
                ctrl.reg_read_rt_addr_type = REG_ADDR_TYPE_RT;
                ctrl.reg_read_rs_stage = PipelineStage_EX;
                ctrl.reg_read_rt_stage = PipelineStage_EX;
                ctrl.reg_write = 1;
                ctrl.reg_write_data_src = REG_WRITE_DATA_SOURCE_ALU;
                ctrl.reg_write_addr_type = REG_ADDR_TYPE_RD;
                ctrl.operand1_src = OPERAND_SOURCE_RS;
                ctrl.operand2_src = OPERAND_SOURCE_RT;

                case (funct)
                    FUNCT_ADD:  ctrl.alu_op = ALU_ADD;
                    FUNCT_ADDU: ctrl.alu_op = ALU_ADDU;
                    FUNCT_SUB:  ctrl.alu_op = ALU_SUB;
                    FUNCT_SUBU: ctrl.alu_op = ALU_SUBU;
                    FUNCT_AND:  ctrl.alu_op = ALU_AND;
                    FUNCT_OR:   ctrl.alu_op = ALU_OR;
                    FUNCT_XOR:  ctrl.alu_op = ALU_XOR;
                    FUNCT_NOR:  ctrl.alu_op = ALU_NOR;
                    FUNCT_SLT:  ctrl.alu_op = ALU_SLT;
                    FUNCT_SLTU: ctrl.alu_op = ALU_SLTU;
                endcase
            end
            SLL, SRL, SRA: begin
                // rt 寄存器是源操作数（要被移位的数）
                ctrl.reg_read_rt_addr_type = REG_ADDR_TYPE_RT;
                ctrl.reg_read_rt_stage = PipelineStage_EX;
                // rd 寄存器是目标寄存器
                ctrl.reg_write = 1;
                ctrl.reg_write_data_src = REG_WRITE_DATA_SOURCE_ALU;
                ctrl.reg_write_addr_type = REG_ADDR_TYPE_RD;
                // rt 作为第一个操作数（要被移位的数）
                ctrl.operand1_src = OPERAND_SOURCE_RT;
                // shamt 字段作为第二个操作数（移位量）
                ctrl.operand2_src = OPERAND_SOURCE_UNSIGNED_SHAMT;

                case (funct)
                    FUNCT_SLL: ctrl.alu_op = ALU_SLL;
                    FUNCT_SRL: ctrl.alu_op = ALU_SRL;
                    FUNCT_SRA: ctrl.alu_op = ALU_SRA;
                endcase
            end
            SLLV, SRLV, SRAV: begin
                // rt 寄存器是源操作数（要被移位的数）
                ctrl.reg_read_rt_addr_type = REG_ADDR_TYPE_RT;
                ctrl.reg_read_rt_stage = PipelineStage_EX;
                // rs 寄存器提供移位量
                ctrl.reg_read_rs_addr_type = REG_ADDR_TYPE_RS;
                ctrl.reg_read_rs_stage = PipelineStage_EX;
                // rd 寄存器是目标寄存器
                ctrl.reg_write = 1;
                ctrl.reg_write_data_src = REG_WRITE_DATA_SOURCE_ALU;
                ctrl.reg_write_addr_type = REG_ADDR_TYPE_RD;
                // rt 作为第一个操作数（要被移位的数）
                ctrl.operand1_src = OPERAND_SOURCE_RT;
                // rs 作为第二个操作数（移位量）
                ctrl.operand2_src = OPERAND_SOURCE_RS;

                case (funct)
                    FUNCT_SLLV: ctrl.alu_op = ALU_SLLV;
                    FUNCT_SRLV: ctrl.alu_op = ALU_SRLV;
                    FUNCT_SRAV: ctrl.alu_op = ALU_SRAV;
                endcase
            end
            ADDI, ADDIU, ANDI, ORI, XORI, SLTI, SLTIU: begin
                // 只需要从 rs 读取源操作数
                ctrl.reg_read_rs_addr_type = REG_ADDR_TYPE_RS;
                ctrl.reg_read_rs_stage = PipelineStage_EX;
                // rt 是目标寄存器
                ctrl.reg_write = 1;
                ctrl.reg_write_data_src = REG_WRITE_DATA_SOURCE_ALU;
                ctrl.reg_write_addr_type = REG_ADDR_TYPE_RT;
                // rs 作为第一个操作数
                ctrl.operand1_src = OPERAND_SOURCE_RS;

                // 根据指令类型选择立即数扩展方式
                casex (instruction.instruction)
                    ANDI, ORI, XORI: 
                        ctrl.operand2_src = OPERAND_SOURCE_UNSIGNED_IMM16;
                    default: // ADDI, ADDIU, SLTI, SLTIU
                        ctrl.operand2_src = OPERAND_SOURCE_SIGNED_IMM16;
                endcase

                // 根据指令类型选择 ALU 操作
                casex (instruction.instruction)
                    ADDI:  ctrl.alu_op = ALU_ADD;
                    ADDIU: ctrl.alu_op = ALU_ADDU;
                    ANDI:  ctrl.alu_op = ALU_AND;
                    ORI:   ctrl.alu_op = ALU_OR;
                    XORI:  ctrl.alu_op = ALU_XOR;
                    SLTI:  ctrl.alu_op = ALU_SLT;
                    SLTIU: ctrl.alu_op = ALU_SLTU;
                endcase
            end
            LUI: begin
                ctrl.reg_write = 1;
                ctrl.reg_write_data_src = REG_WRITE_DATA_SOURCE_ALU;
                ctrl.reg_write_addr_type = REG_ADDR_TYPE_RT;
                ctrl.operand2_src = OPERAND_SOURCE_UNSIGNED_IMM16;
                ctrl.alu_op = ALU_LUI;
            end
            BEQ: begin
                ctrl.reg_read_rs_addr_type = REG_ADDR_TYPE_RS;
                ctrl.reg_read_rt_addr_type = REG_ADDR_TYPE_RT;
                ctrl.reg_read_rs_stage = PipelineStage_ID;
                ctrl.reg_read_rt_stage = PipelineStage_ID;
                ctrl.jump_mode = JUMP_MODE_RELATIVE;
                ctrl.jump_cond = JUMP_COND_EQ;
            end
            BNE: begin
                ctrl.reg_read_rs_addr_type = REG_ADDR_TYPE_RS;
                ctrl.reg_read_rt_addr_type = REG_ADDR_TYPE_RT;
                ctrl.reg_read_rs_stage = PipelineStage_ID;
                ctrl.reg_read_rt_stage = PipelineStage_ID;
                ctrl.jump_mode = JUMP_MODE_RELATIVE;
                ctrl.jump_cond = JUMP_COND_NE;
            end
            BLEZ: begin
                ctrl.reg_read_rs_addr_type = REG_ADDR_TYPE_RS;
                ctrl.reg_read_rs_stage = PipelineStage_ID;
                ctrl.jump_mode = JUMP_MODE_RELATIVE;
                ctrl.jump_cond = JUMP_COND_LEZ;
            end
            BGTZ: begin
                ctrl.reg_read_rs_addr_type = REG_ADDR_TYPE_RS;
                ctrl.reg_read_rs_stage = PipelineStage_ID;
                ctrl.jump_mode = JUMP_MODE_RELATIVE;
                ctrl.jump_cond = JUMP_COND_GTZ;
            end
            BLTZ: begin
                ctrl.reg_read_rs_addr_type = REG_ADDR_TYPE_RS;
                ctrl.reg_read_rs_stage = PipelineStage_ID;
                ctrl.jump_mode = JUMP_MODE_RELATIVE;
                ctrl.jump_cond = JUMP_COND_LTZ;
            end
            BGEZ: begin
                ctrl.reg_read_rs_addr_type = REG_ADDR_TYPE_RS;
                ctrl.reg_read_rs_stage = PipelineStage_ID;
                ctrl.jump_mode = JUMP_MODE_RELATIVE;
                ctrl.jump_cond = JUMP_COND_GEZ;
            end
            JAL: begin
                ctrl.reg_write = 1;
                ctrl.reg_write_data_src = REG_WRITE_DATA_SOURCE_PC_PLUS_8;
                ctrl.reg_write_addr_type = REG_ADDR_TYPE_RA;
                ctrl.jump_mode = JUMP_MODE_ABSOLUTE;
                ctrl.jump_cond = JUMP_COND_TRUE;
            end
            JR: begin
                ctrl.reg_read_rs_addr_type = REG_ADDR_TYPE_RS;
                ctrl.reg_read_rs_stage = PipelineStage_ID;
                ctrl.jump_mode = JUMP_MODE_REGISTER;
                ctrl.jump_cond = JUMP_COND_TRUE;
            end
            JALR: begin
                ctrl.reg_read_rs_addr_type = REG_ADDR_TYPE_RS;
                ctrl.reg_read_rs_stage = PipelineStage_ID;
                ctrl.reg_write = 1;
                ctrl.reg_write_data_src = REG_WRITE_DATA_SOURCE_PC_PLUS_8;
                ctrl.reg_write_addr_type = REG_ADDR_TYPE_RD;
                ctrl.jump_mode = JUMP_MODE_REGISTER;
                ctrl.jump_cond = JUMP_COND_TRUE;
            end
            J: begin
                ctrl.jump_mode = JUMP_MODE_ABSOLUTE;
                ctrl.jump_cond = JUMP_COND_TRUE;
            end
            // 读内存指令
            LB, LBU, LH, LHU, LW: begin
                ctrl.reg_read_rs_addr_type = REG_ADDR_TYPE_RS;
                ctrl.reg_read_rs_stage = PipelineStage_EX;
                ctrl.reg_write = 1;
                ctrl.reg_write_data_src = REG_WRITE_DATA_SOURCE_MEM;
                ctrl.reg_write_addr_type = REG_ADDR_TYPE_RT;
                ctrl.mem_read = 1;
                ctrl.operand1_src = OPERAND_SOURCE_RS;
                ctrl.operand2_src = OPERAND_SOURCE_SIGNED_IMM16;
                ctrl.alu_op = ALU_ADD;

                // 根据指令类型设置访存宽度和符号扩展
                casex (instruction.instruction)
                    LB:  begin 
                        ctrl.mem_width = 2'b00;    // byte
                        ctrl.mem_unsigned = 1'b0;   // signed
                    end
                    LBU: begin 
                        ctrl.mem_width = 2'b00;    // byte
                        ctrl.mem_unsigned = 1'b1;   // unsigned
                    end
                    LH:  begin 
                        ctrl.mem_width = 2'b01;    // halfword
                        ctrl.mem_unsigned = 1'b0;   // signed
                    end
                    LHU: begin 
                        ctrl.mem_width = 2'b01;    // halfword
                        ctrl.mem_unsigned = 1'b1;   // unsigned
                    end
                    LW:  begin 
                        ctrl.mem_width = 2'b10;    // word
                        ctrl.mem_unsigned = 1'b0;   // signed
                    end
                endcase
            end

            // 写内存指令
            SB, SH, SW: begin
                ctrl.reg_read_rs_addr_type = REG_ADDR_TYPE_RS;
                ctrl.reg_read_rs_stage = PipelineStage_EX;
                ctrl.reg_read_rt_addr_type = REG_ADDR_TYPE_RT;
                ctrl.reg_read_rt_stage = PipelineStage_MEM;
                ctrl.mem_write = 1;
                ctrl.operand1_src = OPERAND_SOURCE_RS;
                ctrl.operand2_src = OPERAND_SOURCE_SIGNED_IMM16;
                ctrl.alu_op = ALU_ADD;

                // 根据指令类型设置访存宽度
                casex (instruction.instruction)
                    SB: ctrl.mem_width = 2'b00;    // byte
                    SH: ctrl.mem_width = 2'b01;    // halfword
                    SW: ctrl.mem_width = 2'b10;    // word
                endcase
            end
            SYSCALL: begin
                ctrl.reg_read_rs_addr_type = REG_ADDR_TYPE_V0;
                ctrl.reg_read_rs_stage = PipelineStage_EX;
                ctrl.reg_read_rt_addr_type = REG_ADDR_TYPE_A0;
                ctrl.reg_read_rt_stage = PipelineStage_EX;
                ctrl.is_syscall = 1'b1;
            end
            // 乘除法指令
            MULT, MULTU, DIV, DIVU: begin
                ctrl.reg_read_rs_addr_type = REG_ADDR_TYPE_RS;
                ctrl.reg_read_rt_addr_type = REG_ADDR_TYPE_RT;
                ctrl.reg_read_rs_stage = PipelineStage_EX;
                ctrl.reg_read_rt_stage = PipelineStage_EX;
                ctrl.operand1_src = OPERAND_SOURCE_RS;  // rs作为第一个操作数
                ctrl.operand2_src = OPERAND_SOURCE_RT;  // rt作为第二个操作数
                casex (instruction.instruction)
                    MULT:  ctrl.mdu_operation = MDU_START_SIGNED_MUL;
                    MULTU: ctrl.mdu_operation = MDU_START_UNSIGNED_MUL;
                    DIV:   ctrl.mdu_operation = MDU_START_SIGNED_DIV;
                    DIVU:  ctrl.mdu_operation = MDU_START_UNSIGNED_DIV;
                endcase
                ctrl.mdu_start = 1'b1;  // 启动MDU运算
                ctrl.mdu_use = 1'b1;    // 不使用MDU结果
            end
            MFHI: begin
                ctrl.reg_write = 1'b1;
                ctrl.reg_write_data_src = REG_WRITE_DATA_SOURCE_MDU;
                ctrl.reg_write_addr_type = REG_ADDR_TYPE_RD;
                ctrl.mdu_operation = MDU_READ_HI;
                ctrl.mdu_start = 1'b0;  // 不启动新的运算
                ctrl.mdu_use = 1'b1;    // 使用MDU结果
            end
            MFLO: begin
                ctrl.reg_write = 1'b1;
                ctrl.reg_write_data_src = REG_WRITE_DATA_SOURCE_MDU;
                ctrl.reg_write_addr_type = REG_ADDR_TYPE_RD;
                ctrl.mdu_operation = MDU_READ_LO;
                ctrl.mdu_start = 1'b0;  // 不启动新的运算
                ctrl.mdu_use = 1'b1;    // 使用MDU结果
            end
            MTHI: begin
                ctrl.reg_read_rs_addr_type = REG_ADDR_TYPE_RS;
                ctrl.reg_read_rs_stage = PipelineStage_EX;
                ctrl.operand1_src = OPERAND_SOURCE_RS;  // rs作为第一个操作数（要写入的值）
                ctrl.mdu_operation = MDU_WRITE_HI;
                ctrl.mdu_start = 1'b0;  // 不启动新的运算
                ctrl.mdu_use = 1'b1;    // 不使用MDU结果
            end
            MTLO: begin
                ctrl.reg_read_rs_addr_type = REG_ADDR_TYPE_RS;
                ctrl.reg_read_rs_stage = PipelineStage_EX;
                ctrl.operand1_src = OPERAND_SOURCE_RS;  // rs作为第一个操作数（要写入的值）
                ctrl.mdu_operation = MDU_WRITE_LO;
                ctrl.mdu_start = 1'b0;  // 不启动新的运算
                ctrl.mdu_use = 1'b1;    // 不使用MDU结果
            end
            NOP: begin
                // 对于全0指令（NOP/SLL $0,$0,0），不产生任何控制信号
            end
            default: begin
                // 未定义指令异常
                ctrl.exception = EXCEPTION_UNDEFINED_INSTRUCTION;
                // synthesis translate_off
                $display("Error: Unknown instruction: 0x%h", instruction.instruction);
                // synthesis translate_on
            end
        endcase
    end

endmodule 