`timescale 1us / 1us

`include "MipsDefinitions.sv"

`ifndef COMMON_H
`define COMMON_H

`define GET_REG_ADDR(RESULT, REG_ADDR_TYPE, INSTRUCTION) \
    case (REG_ADDR_TYPE) \
        REG_ADDR_TYPE_RS: RESULT = MipsReg'(INSTRUCTION.rs); \
        REG_ADDR_TYPE_RT: RESULT = MipsReg'(INSTRUCTION.rt); \
        REG_ADDR_TYPE_RD: RESULT = MipsReg'(INSTRUCTION.rd); \
        REG_ADDR_TYPE_RA: RESULT = RA; \
        REG_ADDR_TYPE_V0: RESULT = V0; \
        REG_ADDR_TYPE_A0: RESULT = A0; \
        default: RESULT = ZERO; \
    endcase

function PipelineStage get_data_ready_stage(RegWriteDataSource reg_write_data_source);
    case (reg_write_data_source)
        REG_WRITE_DATA_SOURCE_PC_PLUS_8: return PipelineStage_ID;
        REG_WRITE_DATA_SOURCE_ALU: return PipelineStage_EX;
        REG_WRITE_DATA_SOURCE_MDU: return PipelineStage_EX;
        REG_WRITE_DATA_SOURCE_MEM: return PipelineStage_MEM;
    endcase
endfunction

`endif