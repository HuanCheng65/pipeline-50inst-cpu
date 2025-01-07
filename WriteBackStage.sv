`timescale 1us / 1us

`include "MipsDefinitions.sv"
`include "common.vh"

module WriteBackStage(
    input  logic clk,
    input  logic rst,
    input  logic flush,
    input  logic stall,
    input  MEM_WB_Register mem_wb_reg,
    output logic reg_write_enable,
    output MipsReg reg_write_addr,
    output logic [31:0] reg_write_data
);

    assign reg_write_enable = mem_wb_reg.ctrl.reg_write && mem_wb_reg.valid && !flush && !stall;
    assign reg_write_addr = mem_wb_reg.reg_write_addr;
    assign reg_write_data = mem_wb_reg.reg_write_data;

    // 处理 syscall
    always @(posedge clk) begin : handle_syscall
        if (mem_wb_reg.ctrl.is_syscall && !flush && !stall && mem_wb_reg.valid) begin
            case (mem_wb_reg.rs_data)  // $v0
                32'd1: begin // 打印整数
                    $display("%0d", $signed(mem_wb_reg.rt_data));  // $a0
                end
                32'd10: begin // 程序结束
                    $display("Program terminated by syscall");
                    $fflush();
                    $finish(2);  // 使用更强的终止方式
                end
                default: begin
                    $display("Unknown syscall: %0d", mem_wb_reg.rs_data);
                end
            endcase
        end
    end

endmodule

