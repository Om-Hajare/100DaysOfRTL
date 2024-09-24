module ControlUnit(
    input [7:0] instruction,  // 8-bit opcode
    output reg reg_write, mem_write, mem_read, pc_write, // Control signals
    output reg [2:0] ALU_op // ALU operation control
);
    always @(*) begin
        // Default values for control signals
        reg_write = 0; mem_write = 0; mem_read = 0; pc_write = 0; 
        ALU_op = 3'b000;
        
        case(instruction[7:4])
            4'b0000: begin // ADD instruction
                reg_write = 1; 
                ALU_op = 3'b000; // ADD
            end
            4'b0001: begin // SUB instruction
                reg_write = 1; 
                ALU_op = 3'b001; // SUB
            end
            4'b0010: begin // AND instruction
                reg_write = 1; 
                ALU_op = 3'b010; // AND
            end
            4'b1100: begin // LOAD instruction
                mem_read = 1; 
                reg_write = 1;
            end
            4'b1110: begin // STORE instruction
                mem_write = 1;
            end
            // Add more instructions as needed
            default: begin
                // NOP or default state
            end
        endcase
    end
endmodule
