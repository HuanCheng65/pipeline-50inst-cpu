`timescale 1us / 1us

`include "MipsDefinitions.sv"

module DataMemory (
    input  logic        clk,          // Clock signal
    input  logic        rst,          // Reset signal
    input  logic [31:0] pc,           // PC for debug output
    input  logic [31:0] addr,         // Access address
    input  logic        mem_read,     // Read enable
    input  logic        mem_write,    // Write enable
    input  logic [1:0]  mem_width,    // 00-byte, 01-halfword, 10-word
    input  logic        mem_unsigned, // 1-unsigned, 0-signed
    input  logic [31:0] write_data,   // Data to write
    output logic [31:0] read_data     // Read data
);

    // 2048 32-bit words
    logic [31:0] memory [0:2047];
    
    logic [10:0] word_addr;
    logic [1:0]  byte_offset;
    assign word_addr = addr[12:2];    // Use address bits [12:2] as word address
    assign byte_offset = addr[1:0];   // Use address bits [1:0] as byte offset
    
    // 声明模块级变量
    logic [31:0] word_data, next_word_data;
    logic [31:0] old_word, next_old_word;
    
    // Read operation (combinational logic)
    always @(*) begin
        if (mem_read) begin
            word_data = memory[word_addr];
            next_word_data = memory[word_addr + 1];
            
            case (mem_width)
                2'b00: begin // Byte access
                    logic [7:0] byte_data;
                    case (byte_offset)
                        2'b00: byte_data = word_data[7:0];
                        2'b01: byte_data = word_data[15:8];
                        2'b10: byte_data = word_data[23:16];
                        2'b11: byte_data = word_data[31:24];
                    endcase
                    // 将8位扩展为32位
                    read_data = mem_unsigned ? {24'b0, byte_data} : {{24{byte_data[7]}}, byte_data};
                end
                2'b01: begin // Halfword access
                    logic [15:0] half_data;
                    case (byte_offset)
                        2'b00: half_data = word_data[15:0];
                        2'b01: half_data = word_data[23:8];
                        2'b10: half_data = word_data[31:16];
                        2'b11: half_data = {next_word_data[7:0], word_data[31:24]};
                    endcase
                    // 将16位扩展为32位
                    read_data = mem_unsigned ? {16'b0, half_data} : {{16{half_data[15]}}, half_data};
                end
                2'b10: begin // Word access
                    case (byte_offset)
                        2'b00: read_data = word_data;
                        2'b01: read_data = {next_word_data[7:0], word_data[31:8]};
                        2'b10: read_data = {next_word_data[15:0], word_data[31:16]};
                        2'b11: read_data = {next_word_data[23:0], word_data[31:24]};
                    endcase
                end
                default: read_data = 32'b0;
            endcase
        end else begin
            read_data = 32'b0;
        end
    end
    
    // Write operation preparation (combinational logic)
    logic [31:0] new_word, next_new_word;
    logic write_next_word;
    
    always @(*) begin
        new_word = memory[word_addr];
        next_new_word = memory[word_addr + 1];
        write_next_word = 1'b0;
        
        if (mem_write) begin
            old_word = memory[word_addr];
            next_old_word = memory[word_addr + 1];
            
            case (mem_width)
                2'b00: begin // Byte access
                    case (byte_offset)
                        2'b00: new_word = {old_word[31:8], write_data[7:0]};
                        2'b01: new_word = {old_word[31:16], write_data[7:0], old_word[7:0]};
                        2'b10: new_word = {old_word[31:24], write_data[7:0], old_word[15:0]};
                        2'b11: new_word = {write_data[7:0], old_word[23:0]};
                    endcase
                    write_next_word = 1'b0;
                end
                2'b01: begin // Halfword access
                    case (byte_offset)
                        2'b00: begin
                            new_word = {old_word[31:16], write_data[15:0]};
                            write_next_word = 1'b0;
                        end
                        2'b01: begin
                            new_word = {old_word[31:24], write_data[15:0], old_word[7:0]};
                            write_next_word = 1'b0;
                        end
                        2'b10: begin
                            new_word = {write_data[15:0], old_word[15:0]};
                            write_next_word = 1'b0;
                        end
                        2'b11: begin
                            new_word = {write_data[15:8], old_word[23:0]};
                            next_new_word = {next_old_word[31:8], write_data[7:0]};
                            write_next_word = 1'b1;
                        end
                    endcase
                end
                2'b10: begin // Word access
                    case (byte_offset)
                        2'b00: begin
                            new_word = write_data;
                            write_next_word = 1'b0;
                        end
                        2'b01: begin
                            new_word = {write_data[31:24], old_word[23:0]};
                            next_new_word = {next_old_word[31:8], write_data[23:0]};
                            write_next_word = 1'b1;
                        end
                        2'b10: begin
                            new_word = {write_data[31:16], old_word[15:0]};
                            next_new_word = {next_old_word[31:16], write_data[15:0]};
                            write_next_word = 1'b1;
                        end
                        2'b11: begin
                            new_word = {write_data[31:8], old_word[7:0]};
                            next_new_word = {next_old_word[31:24], write_data[7:0]};
                            write_next_word = 1'b1;
                        end
                    endcase
                end
            endcase
        end
    end
    
    // Write operation (sequential logic)
    always @(posedge clk) begin
        if (rst) begin
            for (int i = 0; i < 2048; i = i + 1) begin
                memory[i] <= 32'b0;
            end
        end else if (mem_write) begin
            memory[word_addr] <= new_word;
            if (write_next_word) begin
                memory[word_addr + 1] <= next_new_word;
            end
        end
    end

    // Output for memory writes
    always @(posedge clk) begin
        if (!rst && mem_write) begin
            $display("@%h: *%h <= %h", pc, {addr[31:2], 2'b00}, new_word);
            if (write_next_word) begin
                $display("@%h: *%h <= %h", pc, {addr[31:2], 2'b00} + 4, next_new_word);
            end
        end
    end
    
    // Debug information
    // synthesis translate_off
    final begin
        $display("\n=== Final Memory Dump ===");
        for (int i = 0; i < 2048; i++) begin
            if (memory[i] !== 32'b0) begin
                $display("memory[0x%h] = 0x%h", i << 2, memory[i]);
            end
        end
        $display("=====================\n");
    end
    // synthesis translate_on

endmodule 