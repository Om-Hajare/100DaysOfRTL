module ProgramCounter(
    input clk,
    input reset,
    input [7:0] pc_in,   // Input to load specific address (for branches)
    input pc_write,      // Write enable
    output reg [7:0] pc_out // Current program counter
);
    always @(posedge clk or posedge reset) begin
        if (reset)
            pc_out <= 8'b0;  // Reset to zero
        else if (pc_write)
            pc_out <= pc_in; // Load new address
        else
            pc_out <= pc_out + 1; // Increment PC
    end
endmodule
