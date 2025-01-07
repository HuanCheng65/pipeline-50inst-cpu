`timescale 1us / 1us

`include "MipsDefinitions.sv"

module InstructionFetchStage (
    input  logic clk,
    input  logic rst,
    input  logic stall,
    input  logic flush,
    input  logic jump_enable,
    input  logic [31:0] jump_address,
    output IF_ID_Register if_id_reg,
    output logic [31:0] pc,
    output Instruction instruction
);
    
    logic [31:0] next_pc;

    always @(*) begin
        if (!stall) begin
            if (jump_enable)
                next_pc = jump_address;
            else
                next_pc = pc + 32'd4;
        end else
            next_pc = pc;
    end

    always @(posedge clk) begin
        if (rst) begin
            pc <= 32'h0000_3000;
        end else if (!stall) begin
            pc <= next_pc;
        end
    end

    InstructionMemory instructionMemory (
        .pc(pc),
        .instruction(instruction)
    );

    always @(posedge clk) begin
        if (rst) begin
            if_id_reg.pc <= 32'h0000_3000;
            if_id_reg.instruction <= '0;
        end else if (flush) begin
            if_id_reg.instruction <= '0;
        end else if (!stall) begin
            if_id_reg.pc <= pc;
            if_id_reg.instruction <= instruction;
            if_id_reg.valid <= 1'b1;
        end
    end

endmodule 
