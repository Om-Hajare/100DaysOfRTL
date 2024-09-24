module MemoryBank(
    input clk,
    input [7:0] address,     // 8-bit address
    input [7:0] write_data,  // 8-bit write data
    input mem_write, mem_read, // Read/write enable
    output reg [7:0] read_data // 8-bit read data
);
    reg [7:0] memory [255:0];  // 256 memory locations, 8-bit each

    always @(posedge clk) begin
        if (mem_write)
            memory[address] <= write_data;
        if (mem_read)
            read_data <= memory[address];
    end
endmodule
