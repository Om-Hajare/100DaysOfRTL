
`include "processoor.v"  // Include your processor module

  // Include your processor module
`timescale 1ns / 1ps

module Processor_tb;
    // Inputs
    reg clk;
    reg reset;
    reg [7:0] instruction;

    // Outputs
    wire [7:0] pc_out;
    wire [7:0] alu_result;
    wire [7:0] mem_data;
    wire reg_write;
    wire mem_write;
    wire mem_read;
    wire pc_write;
    wire zero;

    // Instantiate the Processor
    Processor uut (
        .clk(clk), 
        .reset(reset), 
        .pc_out(pc_out), 
        .instruction(instruction), 
        .alu_result(alu_result), 
        .mem_data(mem_data), 
        .reg_write(reg_write), 
        .mem_write(mem_write), 
        .mem_read(mem_read), 
        .pc_write(pc_write), 
        .zero(zero)
    );

    // Clock generation
    always #5 clk = ~clk;

    initial begin
        // Initialize Inputs
        clk = 0;
        reset = 1;
        instruction = 8'b00000000; // NOP

        // Wait for global reset
        #10;
        reset = 0;

        // Initialize memory with some data
        // Directly writing to memory for initialization
        uut.mem_bank.memory[0] = 8'hA5; // Example data at address 0
        uut.mem_bank.memory[1] = 8'h5A; // Example data at address 1

        // Monitor signals
        $monitor("Time: %0t | PC: %h | Instruction: %h | ALU Result: %h | Mem Data: %h | Reg Write: %b | Mem Write: %b | Mem Read: %b | PC Write: %b | Zero: %b", 
                 $time, pc_out, instruction, alu_result, mem_data, reg_write, mem_write, mem_read, pc_write, zero);

        // Test ADD instruction
        // Assuming instruction format: [opcode(4 bits)][reg1(2 bits)][reg2(2 bits)]
        instruction = 8'b00000001; // ADD R0, R1
        #10;
        instruction = 8'b00000010; // ADD R0, R2
        #10;

        // Test STORE instruction
        // Assuming instruction format: [opcode(4 bits)][address(4 bits)]
        instruction = 8'b11100000; // STORE R0 to address 0
        #10;

        // Test LOAD instruction
        // Assuming instruction format: [opcode(4 bits)][address(4 bits)]
        instruction = 8'b11000000; // LOAD R0 from address 0
        #10;

        // Add more instructions as needed
        // ...

        // Finish simulation
        $finish;
    end
endmodule
