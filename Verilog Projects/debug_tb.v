`include "debug.v"
module tb_Processor;

    reg clk;
    reg [7:0] instruction;
    reg [7:0] dataIn;
    wire [7:0] dataOut;

    // Instantiate the Processor
    Processor uut (
        .clk(clk),
        .instruction(instruction),
        .dataIn(dataIn),
        .dataOut(dataOut)
    );

    // Clock generation
    always #5 clk = ~clk;

    initial begin
        // Initialize signals
        clk = 0;
        instruction = 8'b00000000;
        dataIn = 8'b00000000;

        // Test Load Operation
        #10 instruction = 8'b00000000; // LOAD instruction with address 0
        dataIn = 8'h10; // Data to load into memory
        #10; // Wait for load

        // Check data loaded into register
        #10 instruction = 8'b00000001; // Read data from register
        #10;

        // Test Store Operation
        #10 instruction = 8'b00010000; // STORE instruction with address 1
        dataIn = 8'h20; // Data to store in memory
        #10;

        // Check data stored in memory
        #10 instruction = 8'b00000001; // Read data from memory
        #10;

        // Test ALU Operations
        #10 instruction = 8'b00100000; // ADD instruction
        dataIn = 8'h05; // Value to add
        #10;

        // Check ALU output
        #10 instruction = 8'b00110000; // SUB instruction
        dataIn = 8'h03; // Value to subtract
        #10;

        // Test other ALU operations
        #10 $finish;
    end

    initial begin
        $monitor("Time: %0t | Instruction: %b | Data In: %h | Data Out: %h", 
                 $time, instruction, dataIn, dataOut);
    end

endmodule


