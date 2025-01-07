`timescale 1us / 1us

`include "MipsDefinitions.sv"

module GeneralPurposeRegisters (
    input  logic        clk,
    input  logic        rst,
    input  logic [31:0] pc,
    input  MipsReg      rs_addr,
    input  MipsReg      rt_addr,
    input  logic        reg_write,
    input  MipsReg      write_addr,
    input  logic [31:0] write_data,
    output logic [31:0] rs_data,
    output logic [31:0] rt_data
);
    // Define 32 32-bit registers
    logic [31:0] registers [0:31];

    // Read data (combinational logic)
    always @(*) begin
        rs_data = (rs_addr == ZERO) ? 32'b0 : registers[rs_addr];
        rt_data = (rt_addr == ZERO) ? 32'b0 : registers[rt_addr];
    end
    
    // Write data (sequential logic)
    always @(posedge clk) begin
        if (rst) begin
            for (int i = 0; i < 32; i = i + 1) begin
                registers[i] <= 32'b0;
            end
        end else if (reg_write && write_addr != ZERO) begin
            registers[write_addr] <= write_data;
        end
    end

    always @(negedge clk) begin
        if (!rst && reg_write) begin
            $display("@%h: $%d <= %h", pc, write_addr, write_data);
        end
    end
    
endmodule 