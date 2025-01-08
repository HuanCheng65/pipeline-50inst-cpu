`timescale 1us / 1us

`include "MipsDefinitions.sv"
`include "common.vh"

module ForwardingUnit #(
    parameter PipelineStage PIPELINE_STAGE = PipelineStage_EX
) (
    input MipsReg required_reg,
    input logic [31:0] current_data,
    input PipelineStage required_stage,
    input IF_ID_Register if_id_reg,
    input ID_EX_Register id_ex_reg,
    input EX_MEM_Register ex_mem_reg,
    input MEM_WB_Register mem_wb_reg,
    input logic use_alu_result,
    input logic [31:0] alu_result0,
    input logic [31:0] alu_result1,
    input logic mem_read_in_mem,
    input logic [31:0] mem_read_result,
    output logic [31:0] forwarded_data,
    output logic stall,
    output ForwardingType forwarding_type, // for debug
    output logic [31:0] forwarded_from // From which pipeline
);
    PipelineStage data_ready_after_stage;
    always @(*) begin : forward_data
        forwarded_data = current_data;
        stall = 0;
        forwarding_type = FORWARDING_TYPE_NONE;

        if (required_stage != PipelineStage_NEVER &&
            required_reg != ZERO) begin
            // 检查 ID/EX 阶段的第一条指令
            if (PIPELINE_STAGE <= PipelineStage_ID && 
                id_ex_reg.ctrl0.reg_write && 
                id_ex_reg.valid0 &&
                id_ex_reg.reg_write_addr0 == required_reg) begin
                    data_ready_after_stage = get_data_ready_stage(id_ex_reg.ctrl0.reg_write_data_src);
                    if (use_alu_result && id_ex_reg.ctrl0.reg_write_data_src == REG_WRITE_DATA_SOURCE_ALU) begin
                        forwarded_data = alu_result0;
                        stall = 0;
                        forwarding_type = FORWARDING_TYPE_ID_EX;
                    end else begin
                        forwarded_data = id_ex_reg.reg_write_data0;
                        stall = (data_ready_after_stage > PipelineStage_ID) && (PIPELINE_STAGE == required_stage);
                        forwarding_type = FORWARDING_TYPE_ID_EX;
                    end
                    forwarded_from = 32'd0;
            end 
            // 检查 ID/EX 阶段的第二条指令
            else if (PIPELINE_STAGE <= PipelineStage_ID && 
                id_ex_reg.ctrl1.reg_write && 
                id_ex_reg.valid1 &&  // 第二条指令必须有效
                id_ex_reg.reg_write_addr1 == required_reg) begin
                    data_ready_after_stage = get_data_ready_stage(id_ex_reg.ctrl1.reg_write_data_src);
                    if (use_alu_result && id_ex_reg.ctrl1.reg_write_data_src == REG_WRITE_DATA_SOURCE_ALU) begin
                        forwarded_data = alu_result1;
                        stall = 0;
                        forwarding_type = FORWARDING_TYPE_ID_EX;
                    end else begin
                        forwarded_data = id_ex_reg.reg_write_data1;
                        stall = (data_ready_after_stage > PipelineStage_ID) && (PIPELINE_STAGE == required_stage);
                        forwarding_type = FORWARDING_TYPE_ID_EX;
                    end
                    forwarded_from = 32'd1;
            end 
            // 检查 EX/MEM 阶段的第一条指令
            else if (PIPELINE_STAGE <= PipelineStage_EX && 
                ex_mem_reg.ctrl0.reg_write && 
                ex_mem_reg.valid0 &&
                ex_mem_reg.reg_write_addr0 == required_reg) begin
                    if (mem_read_in_mem && ex_mem_reg.ctrl0.reg_write_data_src == REG_WRITE_DATA_SOURCE_MEM) begin
                        forwarded_data = mem_read_result;
                        data_ready_after_stage = PipelineStage_EX;
                        stall = 0;
                        forwarding_type = FORWARDING_TYPE_MEMORY;
                        forwarded_from = 32'd0;
                    end else begin
                        forwarded_data = ex_mem_reg.reg_write_data0;
                        data_ready_after_stage = get_data_ready_stage(ex_mem_reg.ctrl0.reg_write_data_src);
                        stall = (data_ready_after_stage > PipelineStage_EX) && (PIPELINE_STAGE == required_stage);
                        forwarding_type = FORWARDING_TYPE_EX_MEM;
                        forwarded_from = 32'd0;
                    end
            end 
            // 检查 EX/MEM 阶段的第二条指令
            else if (PIPELINE_STAGE <= PipelineStage_EX && 
                ex_mem_reg.ctrl1.reg_write && 
                ex_mem_reg.valid1 &&
                ex_mem_reg.reg_write_addr1 == required_reg) begin
                    if (mem_read_in_mem && ex_mem_reg.ctrl1.reg_write_data_src == REG_WRITE_DATA_SOURCE_MEM) begin
                        forwarded_data = mem_read_result;
                        data_ready_after_stage = PipelineStage_EX;
                        stall = 0;
                        forwarding_type = FORWARDING_TYPE_MEMORY;
                        forwarded_from = 32'd1;
                    end else begin
                        forwarded_data = ex_mem_reg.reg_write_data1;
                        data_ready_after_stage = get_data_ready_stage(ex_mem_reg.ctrl1.reg_write_data_src);
                        stall = (data_ready_after_stage > PipelineStage_EX) && (PIPELINE_STAGE == required_stage);
                        forwarding_type = FORWARDING_TYPE_EX_MEM;
                        forwarded_from = 32'd1;
                    end
            end 
            // 检查 MEM/WB 阶段的第一条指令
            else if (PIPELINE_STAGE <= PipelineStage_MEM && 
                mem_wb_reg.ctrl0.reg_write && 
                mem_wb_reg.valid0 &&
                mem_wb_reg.reg_write_addr0 == required_reg) begin
                    forwarded_data = mem_wb_reg.reg_write_data0;
                    data_ready_after_stage = get_data_ready_stage(mem_wb_reg.ctrl0.reg_write_data_src);
                    stall = (data_ready_after_stage > PipelineStage_MEM) && (PIPELINE_STAGE == required_stage);
                    forwarding_type = FORWARDING_TYPE_MEM_WB;
                    forwarded_from = 32'd0;
            end
            // 检查 MEM/WB 阶段的第二条指令
            else if (PIPELINE_STAGE <= PipelineStage_MEM && 
                mem_wb_reg.ctrl1.reg_write && 
                mem_wb_reg.valid1 &&
                mem_wb_reg.reg_write_addr1 == required_reg) begin
                    forwarded_data = mem_wb_reg.reg_write_data1;
                    data_ready_after_stage = get_data_ready_stage(mem_wb_reg.ctrl1.reg_write_data_src);
                    stall = (data_ready_after_stage > PipelineStage_MEM) && (PIPELINE_STAGE == required_stage);
                    forwarding_type = FORWARDING_TYPE_MEM_WB;
                    forwarded_from = 32'd1;
            end
        end
    end

endmodule
