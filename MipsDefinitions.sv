`timescale 1us / 1us

`include "MultiplicationDivisionUnit.sv"

`ifndef MIPS_DEFINITIONS_INCLUDE
`define MIPS_DEFINITIONS_INCLUDE
`define DEBUG_OUTPUT

// 声明一个全局文件描述符
int debug_log_fd;

`ifndef DEBUG_PRINT
    `ifdef DEBUG_OUTPUT
        // 简化的打印宏,直接使用全局文件描述符
        `define DEBUG_PRINT(msg, args) \
            $fdisplay(debug_log_fd, "[%0t] %s %s", $time, msg, args);
    `else
        `define DEBUG_PRINT(msg, args)
    `endif
`endif

// Register number definitions
typedef enum logic [4:0] {
    ZERO = 5'd0,    // $0  - Zero register
    AT   = 5'd1,    // $1  - Assembler temporary
    V0   = 5'd2,    // $2  - Function return value
    V1   = 5'd3,    // $3  - Function return value
    A0   = 5'd4,    // $4  - Function argument
    A1   = 5'd5,    // $5  - Function argument
    A2   = 5'd6,    // $6  - Function argument
    A3   = 5'd7,    // $7  - Function argument
    T0   = 5'd8,    // $8  - Temporary register
    T1   = 5'd9,    // $9  - Temporary register
    T2   = 5'd10,   // $10 - Temporary register
    T3   = 5'd11,   // $11 - Temporary register
    T4   = 5'd12,   // $12 - Temporary register
    T5   = 5'd13,   // $13 - Temporary register
    T6   = 5'd14,   // $14 - Temporary register
    T7   = 5'd15,   // $15 - Temporary register
    S0   = 5'd16,   // $16 - Saved register
    S1   = 5'd17,   // $17 - Saved register
    S2   = 5'd18,   // $18 - Saved register
    S3   = 5'd19,   // $19 - Saved register
    S4   = 5'd20,   // $20 - Saved register
    S5   = 5'd21,   // $21 - Saved register
    S6   = 5'd22,   // $22 - Saved register
    S7   = 5'd23,   // $23 - Saved register
    T8   = 5'd24,   // $24 - Temporary register
    T9   = 5'd25,   // $25 - Temporary register
    K0   = 5'd26,   // $26 - Kernel register
    K1   = 5'd27,   // $27 - Kernel register
    GP   = 5'd28,   // $28 - Global pointer
    SP   = 5'd29,   // $29 - Stack pointer
    FP   = 5'd30,   // $30 - Frame pointer
    RA   = 5'd31    // $31 - Return address
} MipsReg;

// ALU operation code definitions
typedef enum logic [5:0] {
    ALU_NOP  = 6'b000000,   // No operation
    ALU_ADD  = 6'b000001,   // Signed addition
    ALU_ADDU = 6'b000010,   // Unsigned addition
    ALU_SUB  = 6'b000011,   // Signed subtraction
    ALU_SUBU = 6'b000100,   // Unsigned subtraction
    ALU_OR   = 6'b000101,   // OR operation
    ALU_AND  = 6'b000110,   // AND operation
    ALU_XOR  = 6'b000111,   // XOR operation
    ALU_NOR  = 6'b001000,   // NOR operation
    ALU_SLT  = 6'b001001,   // Set less than (signed)
    ALU_SLTU = 6'b001010,   // Set less than unsigned
    ALU_SLL  = 6'b001011,   // Shift left logical
    ALU_SRL  = 6'b001100,   // Shift right logical
    ALU_SRA  = 6'b001101,   // Shift right arithmetic
    ALU_SLLV = 6'b001110,   // Shift left logical variable
    ALU_SRLV = 6'b001111,   // Shift right logical variable
    ALU_SRAV = 6'b010000,   // Shift right arithmetic variable
    ALU_LUI  = 6'b010001    // Load upper immediate
} AluOp;

// Operation code definitions
typedef enum logic [5:0] {
    OP_RTYPE   = 6'b000000, // R-type instruction
    OP_ADDI    = 6'b001000,
    OP_ADDIU   = 6'b001001,
    OP_SLTI    = 6'b001010,
    OP_SLTIU   = 6'b001011,
    OP_ANDI    = 6'b001100,
    OP_ORI     = 6'b001101,
    OP_XORI    = 6'b001110,
    OP_LUI     = 6'b001111,
    OP_LW      = 6'b100011,
    OP_SW      = 6'b101011,
    OP_BEQ     = 6'b000100,
    OP_BNE     = 6'b000101,  // 添加 BNE
    OP_BLEZ    = 6'b000110,  // 添加 BLEZ
    OP_BGTZ    = 6'b000111,  // 添加 BGTZ
    OP_REGIMM  = 6'b000001,  // 添加 REGIMM (用于 BLTZ/BGEZ)
    OP_JAL     = 6'b000011,
    OP_J       = 6'b000010
} OpCode;

// REGIMM 指令的 rt 字段定义
typedef enum logic [4:0] {
    RT_BLTZ = 5'b00000,  // BLTZ 的 rt 字段
    RT_BGEZ = 5'b00001   // BGEZ 的 rt 字段
} RegimmRT;

// R-type instruction function code definitions
typedef enum logic [5:0] {
    FUNCT_SLL     = 6'b000000,
    FUNCT_SRL     = 6'b000010,
    FUNCT_SRA     = 6'b000011,
    FUNCT_SLLV    = 6'b000100,
    FUNCT_SRLV    = 6'b000110,
    FUNCT_SRAV    = 6'b000111,
    FUNCT_JR      = 6'b001000,
    FUNCT_JALR    = 6'b001001,
    FUNCT_SYSCALL = 6'b001100,
    FUNCT_AND     = 6'b100100,
    FUNCT_OR      = 6'b100101,
    FUNCT_XOR     = 6'b100110,
    FUNCT_NOR     = 6'b100111,
    FUNCT_SLT     = 6'b101010,
    FUNCT_SLTU    = 6'b101011,
    FUNCT_ADD     = 6'b100000,
    FUNCT_ADDU    = 6'b100001,
    FUNCT_SUB     = 6'b100010,
    FUNCT_SUBU    = 6'b100011
} FunctCode;

typedef enum logic [31:0] {
    ADD     = 32'b000000_????????????????????_100000,
    ADDU    = 32'b000000_????????????????????_100001,
    SUB     = 32'b000000_????????????????????_100010,
    SUBU    = 32'b000000_????????????????????_100011,
    SLL     = 32'b000000_????????????????????_000000,
    SRL     = 32'b000000_????????????????????_000010,
    SRA     = 32'b000000_????????????????????_000011,
    SLLV    = 32'b000000_????????????????????_000100,
    SRLV    = 32'b000000_????????????????????_000110,
    SRAV    = 32'b000000_????????????????????_000111,
    AND     = 32'b000000_????????????????????_100100,
    OR      = 32'b000000_????????????????????_100101,
    XOR     = 32'b000000_????????????????????_100110,
    NOR     = 32'b000000_????????????????????_100111,
    SLT     = 32'b000000_????????????????????_101010,
    SLTU    = 32'b000000_????????????????????_101011,

    ADDI    = 32'b001000_????????????????????_??????,
    ADDIU   = 32'b001001_????????????????????_??????,
    ANDI    = 32'b001100_????????????????????_??????,
    ORI     = 32'b001101_????????????????????_??????,
    XORI    = 32'b001110_????????????????????_??????,
    SLTI    = 32'b001010_????????????????????_??????,
    SLTIU   = 32'b001011_????????????????????_??????,

    LUI     = 32'b001111_????????????????????_??????,

    MULT    = 32'b000000_????????????????????_011000,
    MULTU   = 32'b000000_????????????????????_011001,
    DIV     = 32'b000000_????????????????????_011010,
    DIVU    = 32'b000000_????????????????????_011011,
    MFHI    = 32'b000000_????????????????????_010000,
    MTHI    = 32'b000000_????????????????????_010001,
    MFLO    = 32'b000000_????????????????????_010010,
    MTLO    = 32'b000000_????????????????????_010011,

    BEQ     = 32'b000100_????????????????????_??????,
    BNE     = 32'b000101_????????????????????_??????,
    BLEZ    = 32'b000110_????????????????????_??????,
    BGTZ    = 32'b000111_????????????????????_??????,
    BGEZ    = 32'b000001_?????00001??????????_??????,
    BLTZ    = 32'b000001_?????00000??????????_??????,

    JR      = 32'b000000_????????????????????_001000,
    JALR    = 32'b000000_????????????????????_001001,
    J       = 32'b000010_????????????????????_??????,
    JAL     = 32'b000011_????????????????????_??????,

    LB      = 32'b100000_????????????????????_??????,
    LBU     = 32'b100100_????????????????????_??????,
    LH      = 32'b100001_????????????????????_??????,
    LHU     = 32'b100101_????????????????????_??????,
    LW      = 32'b100011_????????????????????_??????,
    SB      = 32'b101000_????????????????????_??????,
    SH      = 32'b101001_????????????????????_??????,
    SW      = 32'b101011_????????????????????_??????,

    SYSCALL = 32'b000000_????????????????????_001100,
    NOP     = 32'b000000_00000000000000000000000000
} InstructionCode;

typedef enum logic [1:0] {
    JUMP_MODE_NEXT     = 2'b00,
    JUMP_MODE_RELATIVE = 2'b01,
    JUMP_MODE_ABSOLUTE = 2'b10,
    JUMP_MODE_REGISTER = 2'b11
} JumpMode;

typedef enum logic [2:0] {
    JUMP_COND_FALSE = 3'b000,  // 修改为 3 位以支持更多条件
    JUMP_COND_TRUE  = 3'b001,
    JUMP_COND_EQ    = 3'b010,
    JUMP_COND_NE    = 3'b011,
    JUMP_COND_LEZ   = 3'b100,
    JUMP_COND_GTZ   = 3'b101,
    JUMP_COND_GEZ   = 3'b110,
    JUMP_COND_LTZ   = 3'b111
} JumpCondition;

typedef enum logic [2:0] {
    REG_ADDR_TYPE_ZERO = 3'b000,
    REG_ADDR_TYPE_RS   = 3'b001,
    REG_ADDR_TYPE_RT   = 3'b010,
    REG_ADDR_TYPE_RD   = 3'b011,
    REG_ADDR_TYPE_RA   = 3'b100,
    REG_ADDR_TYPE_V0   = 3'b101,
    REG_ADDR_TYPE_A0   = 3'b110
} RegAddrType;

typedef enum logic [1:0] {
    REG_WRITE_DATA_SOURCE_PC_PLUS_8,            // PC + 8 (For JAL)
    REG_WRITE_DATA_SOURCE_ALU,                  // ALU result
    REG_WRITE_DATA_SOURCE_MDU,                  // MDU result
    REG_WRITE_DATA_SOURCE_MEM                   // Memory data
} RegWriteDataSource;

typedef enum logic [2:0] {
    OPERAND_SOURCE_RS = 3'b000,
    OPERAND_SOURCE_RT,
    OPERAND_SOURCE_UNSIGNED_IMM16,
    OPERAND_SOURCE_SIGNED_IMM16,
    OPERAND_SOURCE_UNSIGNED_SHAMT
} OperandSource;

// Instruction
typedef struct packed {
    InstructionCode instruction;
    logic [5:0]     opcode;
    MipsReg         rs;
    MipsReg         rt;
    MipsReg         rd;
    logic [15:0]    immediate;
    logic [25:0]    address;
    logic [4:0]     shamt;
    logic [5:0]     funct;
} Instruction;

// Pipeline Stages
typedef enum logic [2:0] {
    PipelineStage_NEVER = 3'b000,
    PipelineStage_IF,
    PipelineStage_ID,
    PipelineStage_EX,
    PipelineStage_MEM,
    PipelineStage_WB
} PipelineStage;

// 异常类型定义
typedef enum logic [1:0] {
    EXCEPTION_NONE = 2'b00,
    EXCEPTION_UNDEFINED_INSTRUCTION = 2'b01,
    EXCEPTION_INTEGER_OVERFLOW = 2'b10
} ExceptionType;

// Control signals structure
typedef struct packed {
    logic                   reg_write;              // 是否写寄存器
    RegWriteDataSource      reg_write_data_src;    // 写寄存器的数据来源
    RegAddrType            reg_write_addr_type;    // 写寄存器的地址类型
    RegAddrType            reg_read_rs_addr_type;  // 读RS寄存器的地址类型
    RegAddrType            reg_read_rt_addr_type;  // 读RT寄存器的地址类型
    PipelineStage          reg_read_rs_stage;      // 读RS寄存器的流水线阶段
    PipelineStage          reg_read_rt_stage;      // 读RT寄存器的流水线阶段
    logic                   mem_read;               // 是否读内存
    logic                   mem_write;              // 是否写内存
    logic [1:0]            mem_width;              // 访存宽度
    logic                   mem_unsigned;           // 访存是否无符号扩展
    JumpMode               jump_mode;              // 跳转模式
    JumpCondition          jump_cond;              // 跳转条件
    OperandSource          operand1_src;           // 操作数1来源（用于ALU和MDU）
    OperandSource          operand2_src;           // 操作数2来源（用于ALU和MDU）
    AluOp                  alu_op;                 // ALU操作
    logic                   is_syscall;             // 是否是syscall指令
    mdu_operation_t        mdu_operation;          // MDU操作类型
    logic                   mdu_start;              // 是否启动MDU运算
    logic                   mdu_use;                // 是否使用MDU结果
    ExceptionType          exception;           // 异常类型
} ControlSignals;

// IF/ID Register
typedef struct packed {
    logic [31:0] pc;
    Instruction instruction;
    ExceptionType exception;           // 添加异常字段
    logic valid;
} IF_ID_Register;

// ID/EX Register
typedef struct packed {
    logic [31:0] pc;
    Instruction instruction;
    ControlSignals ctrl;
    MipsReg        rs_addr;
    MipsReg        rt_addr;
    logic [31:0]   rs_data;
    logic [31:0]   rt_data;
    logic [31:0]   sign_ext_imm;
    MipsReg        reg_write_addr;
    logic [31:0]   reg_write_data;
    ExceptionType  exception;          // 添加异常字段
    logic valid;
} ID_EX_Register;

// EX/MEM Register
typedef struct packed {
    logic [31:0] pc;
    Instruction instruction;
    ControlSignals ctrl;
    MipsReg        rs_addr;
    MipsReg        rt_addr;
    logic [31:0]   rs_data;
    logic [31:0]   rt_data;
    logic [31:0]   alu_result;
    MipsReg        reg_write_addr;
    logic [31:0]   reg_write_data;
    ExceptionType  exception;          // 添加异常字段
    logic valid;
} EX_MEM_Register;

// MEM/WB Register
typedef struct packed {
    logic [31:0] pc;
    Instruction instruction;
    ControlSignals ctrl;
    MipsReg        rs_addr;
    MipsReg        rt_addr;
    logic [31:0]   rs_data;
    logic [31:0]   rt_data;
    MipsReg        reg_write_addr;
    logic [31:0]   reg_write_data;
    ExceptionType  exception;          // 添加异常字段
    logic valid;
} MEM_WB_Register;

typedef enum logic [2:0] {
    FORWARDING_TYPE_NONE,
    FORWARDING_TYPE_ID_EX,
    FORWARDING_TYPE_EX_MEM,
    FORWARDING_TYPE_MEM_WB,
    FORWARDING_TYPE_MEMORY
} ForwardingType;

function automatic void init_reg_names();
    reg_names[0] = "$zero";
    reg_names[1] = "$at";
    reg_names[2] = "$v0";
    reg_names[3] = "$v1";
    reg_names[4] = "$a0";
    reg_names[5] = "$a1";
    reg_names[6] = "$a2";
    reg_names[7] = "$a3";
    reg_names[8] = "$t0";
    reg_names[9] = "$t1";
    reg_names[10] = "$t2";
    reg_names[11] = "$t3";
    reg_names[12] = "$t4";
    reg_names[13] = "$t5";
    reg_names[14] = "$t6";
    reg_names[15] = "$t7";
    reg_names[16] = "$s0";
    reg_names[17] = "$s1";
    reg_names[18] = "$s2";
    reg_names[19] = "$s3";
    reg_names[20] = "$s4";
    reg_names[21] = "$s5";
    reg_names[22] = "$s6";
    reg_names[23] = "$s7";
    reg_names[24] = "$t8";
    reg_names[25] = "$t9";
    reg_names[26] = "$k0";
    reg_names[27] = "$k1";
    reg_names[28] = "$gp";
    reg_names[29] = "$sp";
    reg_names[30] = "$fp";
    reg_names[31] = "$ra";
endfunction

// 声明寄存器名称数组
string reg_names[32];

function string get_forwarding_type_name(ForwardingType forwarding_type);
    case(forwarding_type)
        FORWARDING_TYPE_NONE: return "NONE";
        FORWARDING_TYPE_ID_EX: return "ID_EX";
        FORWARDING_TYPE_EX_MEM: return "EX_MEM";
        FORWARDING_TYPE_MEM_WB: return "MEM_WB";
        FORWARDING_TYPE_MEMORY: return "MEMORY";
    endcase
endfunction

function string decode_instruction(logic [31:0] instr);
    logic [5:0] op;
    logic [5:0] funct;
    logic [4:0] rs;
    logic [4:0] rt;
    logic [4:0] rd;
    logic [15:0] imm;
    logic [25:0] target;
    logic [4:0] shamt;
    
    op = instr[31:26];
    funct = instr[5:0];
    rs = instr[25:21];
    rt = instr[20:16];
    rd = instr[15:11];
    imm = instr[15:0];
    target = instr[25:0];
    shamt = instr[10:6];

    init_reg_names();

    casex (instr)
        ADD: return $sformatf("add %s, %s, %s", 
            reg_names[rd], reg_names[rs], reg_names[rt]);
        ADDU: return $sformatf("addu %s, %s, %s", 
            reg_names[rd], reg_names[rs], reg_names[rt]);
        SUB: return $sformatf("sub %s, %s, %s", 
            reg_names[rd], reg_names[rs], reg_names[rt]);
        SUBU: return $sformatf("subu %s, %s, %s", 
            reg_names[rd], reg_names[rs], reg_names[rt]);
        AND: return $sformatf("and %s, %s, %s", 
            reg_names[rd], reg_names[rs], reg_names[rt]);
        OR: return $sformatf("or %s, %s, %s", 
            reg_names[rd], reg_names[rs], reg_names[rt]);
        XOR: return $sformatf("xor %s, %s, %s", 
            reg_names[rd], reg_names[rs], reg_names[rt]);
        NOR: return $sformatf("nor %s, %s, %s", 
            reg_names[rd], reg_names[rs], reg_names[rt]);
        SLT: return $sformatf("slt %s, %s, %s", 
            reg_names[rd], reg_names[rs], reg_names[rt]);
        SLTU: return $sformatf("sltu %s, %s, %s", 
            reg_names[rd], reg_names[rs], reg_names[rt]);
        SLL: return $sformatf("sll %s, %s, %d", 
            reg_names[rd], reg_names[rt], shamt);
        SRL: return $sformatf("srl %s, %s, %d", 
            reg_names[rd], reg_names[rt], shamt);
        SRA: return $sformatf("sra %s, %s, %d", 
            reg_names[rd], reg_names[rt], shamt);
        SLLV: return $sformatf("sllv %s, %s, %s", 
            reg_names[rd], reg_names[rt], reg_names[rs]);
        SRLV: return $sformatf("srlv %s, %s, %s", 
            reg_names[rd], reg_names[rt], reg_names[rs]);
        SRAV: return $sformatf("srav %s, %s, %s", 
            reg_names[rd], reg_names[rt], reg_names[rs]);
        ORI: return $sformatf("ori %s, %s, 0x%h", 
            reg_names[rt], reg_names[rs], imm);
        ANDI: return $sformatf("andi %s, %s, 0x%h", 
            reg_names[rt], reg_names[rs], imm);
        LUI: return $sformatf("lui %s, 0x%h", 
            reg_names[rt], imm);
        LW: return $sformatf("lw %s, %0d(%s)", 
            reg_names[rt], $signed(imm), reg_names[rs]);
        SW: return $sformatf("sw %s, %0d(%s)", 
            reg_names[rt], $signed(imm), reg_names[rs]);
        BEQ: return $sformatf("beq %s, %s, %0d", 
            reg_names[rs], reg_names[rt], $signed(imm));
        BNE: return $sformatf("bne %s, %s, %0d", 
            reg_names[rs], reg_names[rt], $signed(imm));
        BLEZ: return $sformatf("blez %s, %0d", 
            reg_names[rs], $signed(imm));
        BGTZ: return $sformatf("bgtz %s, %0d", 
            reg_names[rs], $signed(imm));
        BLTZ: return $sformatf("bltz %s, %0d", 
            reg_names[rs], $signed(imm));
        BGEZ: return $sformatf("bgez %s, %0d", 
            reg_names[rs], $signed(imm));
        JAL: return $sformatf("jal 0x%h", 
            {4'b0, target, 2'b00});
        JR: return $sformatf("jr %s", 
            reg_names[rs]);
        JALR: return $sformatf("jalr %s, %s", 
            reg_names[rd], reg_names[rs]);
        J: return $sformatf("j 0x%h",
            {4'b0, target, 2'b00});
        SYSCALL: return "syscall";
        LB: return $sformatf("lb %s, %0d(%s)", 
            reg_names[rt], $signed(imm), reg_names[rs]);
        LBU: return $sformatf("lbu %s, %0d(%s)", 
            reg_names[rt], $signed(imm), reg_names[rs]);
        LH: return $sformatf("lh %s, %0d(%s)", 
            reg_names[rt], $signed(imm), reg_names[rs]);
        LHU: return $sformatf("lhu %s, %0d(%s)", 
            reg_names[rt], $signed(imm), reg_names[rs]);
        SB: return $sformatf("sb %s, %0d(%s)", 
            reg_names[rt], $signed(imm), reg_names[rs]);
        SH: return $sformatf("sh %s, %0d(%s)", 
            reg_names[rt], $signed(imm), reg_names[rs]);
        ADDI: return $sformatf("addi %s, %s, %0d", 
            reg_names[rt], reg_names[rs], $signed(imm));
        ADDIU: return $sformatf("addiu %s, %s, %0d", 
            reg_names[rt], reg_names[rs], $signed(imm));
        SLTI: return $sformatf("slti %s, %s, %0d", 
            reg_names[rt], reg_names[rs], $signed(imm));
        SLTIU: return $sformatf("sltiu %s, %s, %0d", 
            reg_names[rt], reg_names[rs], $signed(imm));
        ANDI: return $sformatf("andi %s, %s, 0x%h", 
            reg_names[rt], reg_names[rs], imm);
        ORI: return $sformatf("ori %s, %s, 0x%h", 
            reg_names[rt], reg_names[rs], imm);
        XORI: return $sformatf("xori %s, %s, 0x%h", 
            reg_names[rt], reg_names[rs], imm);
        MULT: return $sformatf("mult %s, %s", 
            reg_names[rs], reg_names[rt]);
        MULTU: return $sformatf("multu %s, %s", 
            reg_names[rs], reg_names[rt]);
        DIV: return $sformatf("div %s, %s", 
            reg_names[rs], reg_names[rt]);
        DIVU: return $sformatf("divu %s, %s", 
            reg_names[rs], reg_names[rt]);
        MFHI: return $sformatf("mfhi %s", 
            reg_names[rd]);
        MTHI: return $sformatf("mthi %s", 
            reg_names[rs]);
        MFLO: return $sformatf("mflo %s", 
            reg_names[rd]);
        MTLO: return $sformatf("mtlo %s", 
            reg_names[rs]);
        NOP: return "nop";
        default: return "unknown instruction";
    endcase
endfunction

function string get_reg_name(MipsReg reg_num);
    return reg_names[reg_num];
endfunction

function string get_pipeline_stage_name(PipelineStage stage);
    case(stage)
        PipelineStage_IF: return "IF";
        PipelineStage_ID: return "ID"; 
        PipelineStage_EX: return "EX";
        PipelineStage_MEM: return "MEM";
        PipelineStage_WB: return "WB";
        default: return "UNKNOWN";
    endcase
endfunction

// 添加异常处理相关的函数
function automatic void handle_pipeline_exception(
    input PipelineStage stage,
    input ExceptionType exception,
    input logic [31:0] pc,
    input Instruction instruction,
    input int fd
);
    if (exception != EXCEPTION_NONE) begin
        $fdisplay(fd, "\nException in %s stage:", get_pipeline_stage_name(stage));
        case (exception)
            EXCEPTION_UNDEFINED_INSTRUCTION: 
                $fdisplay(fd, "  Undefined instruction detected");
            EXCEPTION_INTEGER_OVERFLOW:
                $fdisplay(fd, "  Integer overflow detected");
            default: 
                $fdisplay(fd, "  Unknown exception");
        endcase
        if (stage == PipelineStage_WB) begin
            $fdisplay(fd, "  At PC=0x%h, Instruction=%s", 
                pc, decode_instruction(instruction.instruction));
        end
    end
endfunction

function automatic void fatal_pipeline_exception(
    input PipelineStage stage,
    input ExceptionType exception,
    input logic [31:0] pc,
    input Instruction instruction
);
    if (exception != EXCEPTION_NONE) begin
        case (exception)
            EXCEPTION_UNDEFINED_INSTRUCTION: begin
                $display("Error: Undefined instruction at PC=0x%h: %h", 
                    pc, instruction.instruction);
                $finish;
            end
            EXCEPTION_INTEGER_OVERFLOW: begin
                $display("Error: Integer overflow at PC=0x%h: %h", 
                    pc, instruction.instruction);
                $finish;
            end
            default: begin
                $display("Error: Unknown exception at PC=0x%h", pc);
                $finish;
            end
        endcase
    end
endfunction

`endif // MIPS_DEFINITIONS_INCLUDED