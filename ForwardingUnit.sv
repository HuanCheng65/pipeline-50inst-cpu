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
    input logic mem_read_in_mem,
    input logic [31:0] mem_read_result,
    output logic [31:0] forwarded_data,
    output logic stall,
    output ForwardingType forwarding_type // for debug
);
    PipelineStage data_ready_after_stage;
    always @(*) begin : forward_data
        forwarded_data = current_data;
        stall = 0;
        forwarding_type = FORWARDING_TYPE_NONE;
        if (required_stage != PipelineStage_NEVER &&
            required_reg != ZERO) begin
            if (PIPELINE_STAGE <= PipelineStage_ID && 
                id_ex_reg.ctrl.reg_write && 
                id_ex_reg.reg_write_addr == required_reg) begin
                    forwarded_data = id_ex_reg.reg_write_data;
                    data_ready_after_stage = get_data_ready_stage(id_ex_reg.ctrl.reg_write_data_src);
                    stall = (data_ready_after_stage > PipelineStage_ID) && (PIPELINE_STAGE == required_stage);
                    forwarding_type = FORWARDING_TYPE_ID_EX;
            end else if (PIPELINE_STAGE <= PipelineStage_EX && 
                ex_mem_reg.ctrl.reg_write && 
                ex_mem_reg.reg_write_addr == required_reg) begin
                    if (mem_read_in_mem && ex_mem_reg.ctrl.reg_write_data_src == REG_WRITE_DATA_SOURCE_MEM) begin
                        forwarded_data = mem_read_result;
                        data_ready_after_stage = PipelineStage_EX;
                        stall = 0;
                        forwarding_type = FORWARDING_TYPE_MEMORY;
                    end else begin
                        forwarded_data = ex_mem_reg.reg_write_data;
                        data_ready_after_stage = get_data_ready_stage(ex_mem_reg.ctrl.reg_write_data_src);
                        stall = (data_ready_after_stage > PipelineStage_EX) && (PIPELINE_STAGE == required_stage);
                        forwarding_type = FORWARDING_TYPE_EX_MEM;
                    end
            end else if (PIPELINE_STAGE <= PipelineStage_MEM && 
                mem_wb_reg.ctrl.reg_write && 
                mem_wb_reg.reg_write_addr == required_reg) begin
                    forwarded_data = mem_wb_reg.reg_write_data;
                    data_ready_after_stage = get_data_ready_stage(mem_wb_reg.ctrl.reg_write_data_src);
                    stall = (data_ready_after_stage > PipelineStage_MEM) && (PIPELINE_STAGE == required_stage);
                    forwarding_type = FORWARDING_TYPE_MEM_WB;
            end
        end
    end

endmodule
