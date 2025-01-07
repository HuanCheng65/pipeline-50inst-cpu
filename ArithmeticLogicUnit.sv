`timescale 1us / 1us

`include "MipsDefinitions.sv"

module ArithmeticLogicUnit (
    input  logic [31:0] operand1,     // First operand
    input  logic [31:0] operand2,     // Second operand
    input  AluOp        alu_op,       // ALU operation code
    output logic [31:0] result,       // Operation result
    output logic        zero,         // Zero flag (used for beq instruction)
    output logic        overflow      // Overflow flag
);

    always @(*) begin
        zero = 0;
        overflow = 0;
        result = 32'b0;

        case (alu_op)
            // 算术运算
            ALU_ADD: begin
                result = operand1 + operand2;
                // 检查溢出：当两个操作数符号相同，但结果符号不同时发生溢出
                overflow = (operand1[31] == operand2[31]) && (result[31] != operand1[31]);
            end
            ALU_ADDU: result = operand1 + operand2;                    // Unsigned addition
            ALU_SUB: begin
                result = operand1 - operand2;
                // 检查溢出：当被减数和减数符号不同，且结果与被减数符号不同时发生溢出
                overflow = (operand1[31] != operand2[31]) && (result[31] != operand1[31]);
            end
            ALU_SUBU: result = operand1 - operand2;                    // Unsigned subtraction
            
            // 逻辑运算
            ALU_AND:  result = operand1 & operand2;                    // AND operation
            ALU_OR:   result = operand1 | operand2;                    // OR operation
            ALU_XOR:  result = operand1 ^ operand2;                    // XOR operation
            ALU_NOR:  result = ~(operand1 | operand2);                 // NOR operation
            
            // 比较运算
            ALU_SLT:  result = ($signed(operand1) < $signed(operand2)) ? 32'd1 : 32'd0;  // Set less than (signed)
            ALU_SLTU: result = (operand1 < operand2) ? 32'd1 : 32'd0;  // Set less than (unsigned)
            
            // 移位运算 - 固定位数
            ALU_SLL:  result = operand1 << operand2[4:0];             // Shift left logical
            ALU_SRL:  result = operand1 >> operand2[4:0];             // Shift right logical
            ALU_SRA:  result = $signed(operand1) >>> operand2[4:0];    // Shift right arithmetic
            
            // 移位运算 - 变量位数
            ALU_SLLV: result = operand1 << operand2[4:0];             // Shift left logical variable
            ALU_SRLV: result = operand1 >> operand2[4:0];             // Shift right logical variable
            ALU_SRAV: result = $signed(operand1) >>> operand2[4:0];    // Shift right arithmetic variable
            
            // 其他
            ALU_LUI:  result = {operand2[15:0], 16'b0};               // Load upper immediate
            ALU_NOP:  result = operand1;                              // No operation
            default:  result = 32'b0;                                 // Default output 0
        endcase

        zero = (result == 32'b0);
    end

endmodule 