`timescale 1us / 1us

`include "MipsDefinitions.sv"

module InstructionMemory (
    input  logic [31:0] pc,          // Program counter value
    output Instruction instruction0,  // First instruction output
    output Instruction instruction1   // Second instruction output
);

    // 1024 32-bit words
    logic [31:0] memory [0:2047];
    
    // Calculate word address for both instructions
    logic [9:0] word_addr0, word_addr1;
    assign word_addr0 = pc[11:2];          // First instruction address
    assign word_addr1 = pc[11:2] + 10'd1;  // Second instruction address (PC + 4)
    
    // First instruction
    assign instruction0.instruction = InstructionCode'(memory[word_addr0]);
    assign instruction0.opcode = memory[word_addr0][31:26];
    assign instruction0.rs = MipsReg'(memory[word_addr0][25:21]);
    assign instruction0.rt = MipsReg'(memory[word_addr0][20:16]);
    assign instruction0.rd = MipsReg'(memory[word_addr0][15:11]);
    assign instruction0.immediate = memory[word_addr0][15:0];
    assign instruction0.address = memory[word_addr0][25:0];
    assign instruction0.shamt = memory[word_addr0][10:6];
    assign instruction0.funct = memory[word_addr0][5:0];

    // Second instruction
    assign instruction1.instruction = InstructionCode'(memory[word_addr1]);
    assign instruction1.opcode = memory[word_addr1][31:26];
    assign instruction1.rs = MipsReg'(memory[word_addr1][25:21]);
    assign instruction1.rt = MipsReg'(memory[word_addr1][20:16]);
    assign instruction1.rd = MipsReg'(memory[word_addr1][15:11]);
    assign instruction1.immediate = memory[word_addr1][15:0];
    assign instruction1.address = memory[word_addr1][25:0];
    assign instruction1.shamt = memory[word_addr1][10:6];
    assign instruction1.funct = memory[word_addr1][5:0];
    
    // Initialize instruction memory
    initial begin
        for (int i = 0; i < 2048; i = i + 1) begin
            memory[i] = 32'h0000_0000;
        end
        
        // Load instructions from file
        $readmemh("code.txt", memory);
    end

endmodule 