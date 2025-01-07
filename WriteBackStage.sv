`timescale 1us / 1us

`include "MipsDefinitions.sv"
`include "common.vh"

module WriteBackStage(
    input  logic clk,
    input  logic rst,
    input  logic stall0,          // 第一条指令的 stall 信号
    input  logic stall1,          // 第二条指令的 stall 信号
    input  logic flush0,          // 第一条指令的 flush 信号
    input  logic flush1,          // 第二条指令的 flush 信号
    input  MEM_WB_Register mem_wb_reg,
    output logic reg_write_enable0,         // 第一条指令的写使能
    output logic reg_write_enable1,         // 第二条指令的写使能
    output MipsReg reg_write_addr0,         // 第一条指令的写地址
    output MipsReg reg_write_addr1,         // 第二条指令的写地址
    output logic [31:0] reg_write_data0,    // 第一条指令的写数据
    output logic [31:0] reg_write_data1     // 第二条指令的写数据
);

    // 第一条指令的写回控制
    assign reg_write_enable0 = mem_wb_reg.ctrl0.reg_write && mem_wb_reg.valid0 && !flush0 && !stall0;
    assign reg_write_addr0 = mem_wb_reg.reg_write_addr0;
    assign reg_write_data0 = mem_wb_reg.reg_write_data0;

    // 第二条指令的写回控制
    assign reg_write_enable1 = mem_wb_reg.ctrl1.reg_write && mem_wb_reg.valid1 && !flush1 && !stall1;
    assign reg_write_addr1 = mem_wb_reg.reg_write_addr1;
    assign reg_write_data1 = mem_wb_reg.reg_write_data1;

    // 处理 syscall
    always @(posedge clk) begin : handle_syscall
        // 第一条指令的 syscall
        if (mem_wb_reg.ctrl0.is_syscall && !flush0 && !stall0 && mem_wb_reg.valid0) begin
            case (mem_wb_reg.rs_data0)  // $v0
                32'd1: begin // 打印整数
                    $display("%0d", $signed(mem_wb_reg.rt_data0));  // $a0
                end
                32'd10: begin // 程序结束
                    $display("Program terminated by syscall");
                    $finish;
                end
                default: begin
                    $display("Unknown syscall: %0d", mem_wb_reg.rs_data0);
                end
            endcase
        end

        // 第二条指令的 syscall
        if (mem_wb_reg.ctrl1.is_syscall && !flush1 && !stall1 && mem_wb_reg.valid1) begin
            case (mem_wb_reg.rs_data1)  // $v0
                32'd1: begin // 打印整数
                    $display("%0d", $signed(mem_wb_reg.rt_data1));  // $a0
                end
                32'd10: begin // 程序结束
                    $display("Program terminated by syscall");
                    $finish;
                end
                default: begin
                    $display("Unknown syscall: %0d", mem_wb_reg.rs_data1);
                end
            endcase
        end
    end

endmodule

