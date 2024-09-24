`include"8bit_processor.v"
module Processor_tb;

    // Inputs
    reg clk;
    reg reset;
    reg [7:0] instruction;  // External instruction input
    reg [3:0] irq;          // Interrupt request lines

    // Outputs
    wire [7:0] ALU_out;
    wire [7:0] psw_out;
    wire interrupt_ack;
    wire [7:0] mem_read_data;

    // Internal signals for debugging
    wire [7:0] reg_data1, reg_data2;  // To observe register values

    // Instantiate the Processor module
    Processor uut (
        .clk(clk),
        .reset(reset),
        .instruction(instruction),
        .irq(irq),
        .ALU_out(ALU_out),
        .psw_out(psw_out),
        .interrupt_ack(interrupt_ack),
        .mem_read_data(mem_read_data)
    );

    // Access internal signals for debugging purposes (assuming these signals are accessible)
    assign reg_data1 = uut.register_bank.read_data1;  // register r0
    assign reg_data2 = uut.register_bank.read_data2;  // register r1

    // Clock generation
    always #5 clk = ~clk;  // Generate a clock with 10-time unit period

    // Initialize memory for the test
    reg [7:0] memory [0:255];  // Memory block for testing

    initial begin
        // Initialize Inputs
        clk = 0;
        reset = 1;
        instruction = 8'b0;
        irq = 4'b0;
        
        // Initialize memory with values (manually filling some memory addresses)
        memory[0] = 8'h15;  // Example value 0x15 at memory address 0
        memory[1] = 8'h25;  // Example value 0x25 at memory address 1
        
        // Display initialized memory
        $display("Initial memory[0] = %h, memory[1] = %h", memory[0], memory[1]);

        // Apply reset
        #10;
        reset = 0;
        
        // Test Case 1: LOAD r0, mem[0]
        // LOAD r0, mem[0] -> Instruction format: 4'b1000 + r0 + memory address
        instruction = 8'b1000_0000;  // LOAD r0, mem_address 0
        #20;
        $display("After LOAD r0, Register r0 = %h (expected 15 from memory)", reg_data1);
        
        // Test Case 2: LOAD r1, mem[1]
        instruction = 8'b1000_0001;  // LOAD r1, mem_address 1
        #20;
        $display("After LOAD r1, Register r1 = %h (expected 25 from memory)", reg_data2);
        
        // Test Case 3: Perform ADD operation on r0 and r1
        // ADD r0, r1 -> Instruction format: 4'b0000 + r0 + r1
        instruction = 8'b0000_0001;  // ADD r0, r1
        #20;
        $display("After ADD, Register r0 = %h, Register r1 = %h, ALU_out = %h (expected 3A)", reg_data1, reg_data2, ALU_out);

        // End simulation
        $finish;
    end

endmodule


