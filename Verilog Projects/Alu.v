module ALU(
    input [7:0] A, B,  // 8-bit inputs
    input [2:0] ALU_op, // Operation selector
    output reg [7:0] Result, // 8-bit result
    output Zero        // Zero flag
);
    always @(*) begin
        case(ALU_op)
            3'b000: Result = A + B;  // ADD
            3'b001: Result = A - B;  // SUB
            3'b010: Result = A & B;  // AND
            3'b011: Result = A | B;  // OR
            3'b100: Result = A ^ B;  // XOR
            3'b101: Result = ~A;     // NOT
            3'b110: Result = A << 1; // Shift Left
            3'b111: Result = A >> 1; // Shift Right
            default: Result = 8'b0;
        endcase
    end
    assign Zero = (Result == 8'b0); // Zero flag
endmodule
