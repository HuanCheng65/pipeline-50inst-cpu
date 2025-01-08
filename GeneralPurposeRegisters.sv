`timescale 1us / 1us

`include "MipsDefinitions.sv"

module GeneralPurposeRegisters (
    input  logic        clk,
    input  logic        rst,
    input  logic [31:0] pc0,
    input  logic [31:0] pc1,
    input  MipsReg      rs_addr0,
    input  MipsReg      rt_addr0,
    output logic [31:0] rs_data0,
    output logic [31:0] rt_data0,
    input  MipsReg      rs_addr1,
    input  MipsReg      rt_addr1,
    output logic [31:0] rs_data1,
    output logic [31:0] rt_data1,
    input  logic        reg_write0,
    input  MipsReg      write_addr0,
    input  logic [31:0] write_data0,
    input  logic        reg_write1,
    input  MipsReg      write_addr1,
    input  logic [31:0] write_data1
);
    // Define 32 32-bit registers
    logic [31:0] registers [0:31];

    // Read data (combinational logic)
    always @(*) begin
        rs_data0 = (rs_addr0 == ZERO) ? 32'b0 : registers[rs_addr0];
        rt_data0 = (rt_addr0 == ZERO) ? 32'b0 : registers[rt_addr0];
        rs_data1 = (rs_addr1 == ZERO) ? 32'b0 : registers[rs_addr1];
        rt_data1 = (rt_addr1 == ZERO) ? 32'b0 : registers[rt_addr1];
    end
    
    // Write data (sequential logic)
    always @(posedge clk) begin
        if (rst) begin
            for (int i = 0; i < 32; i = i + 1) begin
                registers[i] <= 32'b0;
            end
        end else begin
            if (reg_write0 && write_addr0 != ZERO) begin
                registers[write_addr0] <= write_data0;
            end
            if (reg_write1 && write_addr1 != ZERO) begin
                registers[write_addr1] <= write_data1;
            end
        end
    end

    // Debug output
    always @(negedge clk) begin
        if (!rst) begin
            if (reg_write0) begin
                $display("@%h: $%2d <= %h", pc0, write_addr0, write_data0);
            end
            if (reg_write1) begin
                $display("@%h: $%2d <= %h", pc1, write_addr1, write_data1);
            end
        end
    end
    
endmodule 