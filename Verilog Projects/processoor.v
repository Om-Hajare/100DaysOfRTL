`include "Alu.v"
`include "Control_unit.v"
`include "Instruction_register.v"
`include "Memory_bank.v"
`include "Program_counter.v"
`include "Register_bank.v"

module Processor(
    input clk, 
    input reset,
    output [7:0] pc_out,
    input [7:0] instruction,
    output [7:0] alu_result,
    output [7:0] mem_data,
    output reg_write,
    output mem_write,
    output mem_read,
    output pc_write,
    output zero
);
    // Wires for internal connections
    wire [7:0] alu_result_internal;
    wire zero_internal;
    wire [7:0] reg_data1, reg_data2;
    wire [7:0] mem_read_data;         // Memory read data wire
    wire [7:0] mem_write_data;        // Memory write data wire
    wire [7:0] address;               // Address wire
    wire [2:0] ALU_op;

    // Instantiate components
    ProgramCounter PC(
        .clk(clk), 
        .reset(reset), 
        .pc_in(alu_result_internal),    // PC input is the ALU result (or branch address)
        .pc_write(pc_write), 
        .pc_out(pc_out)
    );

    MemoryBank mem_bank(
        .clk(clk),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .address(pc_out),               // Address is the PC value
        .write_data(mem_write_data),    // Memory write data
        .read_data(mem_read_data)       // Memory read data
    );

    InstructionRegister IR(
        .clk(clk), 
        .instruction_in(mem_read_data), // Read instruction from memory
        .ir_write(1'b1),                // Always write to instruction register
        .instruction_out(instruction)
    );

    RegisterBank RB(
        .clk(clk),
        .read_reg1(instruction[3:2]),   // Source register 1
        .read_reg2(instruction[1:0]),   // Source register 2
        .write_reg(instruction[3:2]),   // Destination register
        .write_data(alu_result),        // Data to write
        .reg_write(reg_write),          // Control signal
        .read_data1(reg_data1),         // Data from register 1
        .read_data2(reg_data2)          // Data from register 2
    );

    ALU alu(
        .A(reg_data1),                  // Input A is reg_data1
        .B(reg_data2),                  // Input B is reg_data2
        .ALU_op(ALU_op),                  // ALU operation
        .Result(alu_result_internal),   // ALU result
        .Zero(zero_internal)            // Zero flag
    );

    ControlUnit CU(
        .instruction(instruction[7:0]),      // Opcode from instruction
        .reg_write(reg_write),
        .mem_write(mem_write),
        .mem_read(mem_read),
        .pc_write(pc_write),
        .ALU_op(ALU_op)                   // ALU operation control
    );

    // Assign internal signals to outputs
    assign alu_result = alu_result_internal;
    assign zero = zero_internal;
    assign mem_data = mem_read_data;    // Assign memory data to output
    assign mem_write_data = reg_data2;  // Memory write data comes from reg_data2
    assign address = pc_out;            // Address comes from Program Counter

endmodule
