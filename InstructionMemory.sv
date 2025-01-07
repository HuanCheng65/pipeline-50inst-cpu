`timescale 1us / 1us

`include "MipsDefinitions.sv"

module InstructionMemory (
    input  logic [31:0] pc,          // Program counter value
    output Instruction instruction  // Output instruction
);

    // 1024 32-bit words
    logic [31:0] memory [0:2047];
    
    // Calculate word address (PC needs to be shifted right by 2 bits because PC is byte-addressed while memory is word-addressed)
    logic [9:0] word_addr;
    assign word_addr = pc[11:2];  // Use PC bits [11:2] as word address
    
    assign instruction.instruction = InstructionCode'(memory[word_addr]);
    assign instruction.opcode = memory[word_addr][31:26];
    assign instruction.rs = MipsReg'(memory[word_addr][25:21]);
    assign instruction.rt = MipsReg'(memory[word_addr][20:16]);
    assign instruction.rd = MipsReg'(memory[word_addr][15:11]);
    assign instruction.immediate = memory[word_addr][15:0];
    assign instruction.address = memory[word_addr][25:0];
    assign instruction.shamt = memory[word_addr][10:6];
    assign instruction.funct = memory[word_addr][5:0];
    
    // Initialize instruction memory
    initial begin
        for (int i = 0; i < 2048; i = i + 1) begin
            memory[i] = 32'h0000_0000;
        end
        
        // Load instructions from file
        // Note: Need to modify to correct file path when in use
        $readmemh("code.txt", memory);
    end

endmodule 