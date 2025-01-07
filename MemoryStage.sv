`timescale 1us / 1us

`include "MipsDefinitions.sv"
`include "common.vh"

module MemoryStage(
    input  logic clk,
    input  logic rst,
    input  logic flush,
    input  logic stall,
    input  EX_MEM_Register ex_mem_reg,
    input  logic [31:0] forwarded_rs_data,
    input  logic [31:0] forwarded_rt_data,
    output MEM_WB_Register mem_wb_reg,
    output logic [31:0] mem_read_data
);

    logic mem_read, mem_write, mem_unsigned;
    logic [1:0] mem_width;
    logic [31:0] mem_addr, mem_write_data;

    assign mem_addr = ex_mem_reg.alu_result;
    assign mem_read = ex_mem_reg.ctrl.mem_read;
    assign mem_write = ex_mem_reg.ctrl.mem_write && !stall;
    assign mem_width = ex_mem_reg.ctrl.mem_width;
    assign mem_unsigned = ex_mem_reg.ctrl.mem_unsigned;
    assign mem_write_data = forwarded_rt_data;

    DataMemory data_mem(
        .clk(clk),
        .rst(rst),
        .pc(ex_mem_reg.pc),
        .addr(mem_addr),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .mem_width(mem_width),
        .mem_unsigned(mem_unsigned),
        .write_data(mem_write_data),
        .read_data(mem_read_data)
    );

    logic [31:0] reg_write_data;

    always @(*) begin : update_reg_write_data
        case (ex_mem_reg.ctrl.reg_write_data_src)
            REG_WRITE_DATA_SOURCE_MEM: reg_write_data = mem_read_data;
            default: reg_write_data = ex_mem_reg.reg_write_data;
        endcase
    end

    always @(posedge clk) begin : update_mem_wb_reg
        if (rst || flush) begin
            mem_wb_reg <= '0;
        end else if (stall) begin
            mem_wb_reg <= mem_wb_reg;
        end else begin
            mem_wb_reg.pc <= ex_mem_reg.pc;
            mem_wb_reg.instruction <= ex_mem_reg.instruction;
            mem_wb_reg.ctrl <= ex_mem_reg.ctrl;
            mem_wb_reg.rs_addr <= ex_mem_reg.rs_addr;
            mem_wb_reg.rt_addr <= ex_mem_reg.rt_addr;
            mem_wb_reg.rs_data <= forwarded_rs_data;
            mem_wb_reg.rt_data <= forwarded_rt_data;
            mem_wb_reg.reg_write_addr <= ex_mem_reg.reg_write_addr;
            mem_wb_reg.reg_write_data <= reg_write_data;
            mem_wb_reg.exception <= ex_mem_reg.exception;
            mem_wb_reg.valid <= ex_mem_reg.valid;
        end
    end

endmodule
