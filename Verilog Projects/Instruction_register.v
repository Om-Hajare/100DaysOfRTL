module InstructionRegister(
    input clk,
    input [7:0] instruction_in,  // Instruction fetched from memory
    input ir_write,              // Write enable
    output reg [7:0] instruction_out // Current instruction
);
    always @(posedge clk) begin
        if (ir_write)
            instruction_out <= instruction_in;
    end
endmodule
