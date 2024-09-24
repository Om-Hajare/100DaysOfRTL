module RegisterBank(
    input clk,
    input [1:0] read_reg1, read_reg2, write_reg, // 2-bit address for registers
    input [7:0] write_data,
    input reg_write,  // Write enable signal
    output [7:0] read_data1, read_data2
);
    reg [7:0] registers [7:0];  // 8 registers of 8 bits

    // Read
    assign read_data1 = registers[read_reg1];
    assign read_data2 = registers[read_reg2];

    // Write
    always @(posedge clk) begin
        if (reg_write)
            registers[write_reg] <= write_data;
    end
endmodule
