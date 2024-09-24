`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12.09.2024 13:51:20
// Design Name: 
// Module Name: processor
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


// ALU
module ALU(
    input wire clk,
    input [7:0] a, b,
    input [3:0] alu_opcode,
    input wire reset,
    output reg [7:0] ALU_out,
    output reg Carry_out,
    output reg Overflow_out
    
);

always @(*) begin
    case (alu_opcode)
        4'b0000: {Carry_out, ALU_out} = a + b;  // Add
        4'b0001: {Carry_out, ALU_out} = a - b;  // Subtract
        4'b0010: ALU_out = a & b; // AND
        4'b0011: ALU_out = a | b; // OR
        4'b0100: ALU_out = a ^ b; // XOR
        4'b0101: ALU_out = ~a;    // NOT
        4'b0110: ALU_out = a << 1; // Logical Left Shift
        4'b0111: ALU_out = a >> 1; // Logical Right Shift
        default: ALU_out = 8'd0;
    endcase

    // Overflow detection for addition and subtraction
    if (alu_opcode == 4'b0000 || alu_opcode == 4'b0001)
        Overflow_out = (~a[7] & ~b[7] & ALU_out[7]) | (a[7] & b[7] & ~ALU_out[7]);
    else
        Overflow_out = 1'b0;
end

endmodule


// PSW
module PSW(
    input wire reset,
    input wire clk,
    input wire sign_flag,
    input wire zero_flag,
    input wire carry_flag,
    input wire overflow_flag,
    input wire aux_carry_flag,
    input wire interrupt_enable,
    input wire set_interrupt,
    input wire clear_interrupt,
    output reg [7:0] psw
);

always @(posedge clk or posedge reset) begin
    if (reset) begin
        psw <= 8'd0;
    end else begin
        psw[0] <= sign_flag;
        psw[1] <= zero_flag;
        psw[3] <= aux_carry_flag;
        psw[4] <= carry_flag;
        psw[5] <= overflow_flag;

        // IE is set or cleared based on specific instructions
        if (set_interrupt) begin
            psw[7] <= 1'b1;
        end else if (clear_interrupt) begin
            psw[7] <= 1'b0;
        end
    end
end

endmodule


// Register Bank
// Register Bank - Corrected
module RegisterBank(
    input wire clk,
    input wire reset,
    input wire [1:0] read_reg1,
    input wire [1:0] read_reg2,
    input wire [1:0] write_reg,
    input wire [7:0] write_data,
    input wire write_enable,
    output reg [7:0] read_data1,
    output reg [7:0] read_data2
);

reg [7:0] R0, R1, R2, R3;

always @(posedge clk) begin
    if (write_enable) begin
        case (write_reg)
            2'b00: R0 <= write_data;
            2'b01: R1 <= write_data;
            2'b10: R2 <= write_data;
            2'b11: R3 <= write_data;
        endcase
    end
end

always @(*) begin
    case (read_reg1)
        2'b00: read_data1 = R0;
        2'b01: read_data1 = R1; 
        2'b10: read_data1 = R2;
        2'b11: read_data1 = R3;
    endcase

    case (read_reg2)
        2'b00: read_data2 = R0;
        2'b01: read_data2 = R1; 
        2'b10: read_data2 = R2;
        2'b11: read_data2 = R3; 
    endcase
end

endmodule



// Memory Bank
// Memory Bank - Potential Fix
module MemoryBank(
    input wire clk,
    input wire [7:0] address,      // Memory address to access
    input wire [7:0] write_data,   // Data to write
    input wire mem_write,          // Memory write enable
    input wire mem_read,           // Memory read enable
    output reg [7:0] read_data     // Data read from memory
);

reg [7:0] memory [255:0];         // 256-byte memory

always @(posedge clk) begin
    if (mem_write) begin
        memory[address] <= write_data;  // Write data to memory
    end
end

always @(posedge clk) begin
    if (mem_read) begin
        read_data <= memory[address];   // Read data from memory
    end else begin
        read_data <= 8'd0;
    end
end

endmodule




// Interrupt Controller
module InterruptController(
    input wire clk,
    input wire reset,
    input wire [3:0] irq, 
    input wire interrupt_enable,
    output reg interrupt_ack,
    output reg [7:0] vector_address
);

localparam VECTOR_IRQ0 = 8'h10;
localparam VECTOR_IRQ1 = 8'h20;
localparam VECTOR_IRQ2 = 8'h30;
localparam VECTOR_IRQ3 = 8'h40;

always @(posedge clk or posedge reset) begin
    if (reset) begin
        interrupt_ack <= 1'b0;
        vector_address <= 8'h00;
    end else begin
        if (interrupt_enable) begin
            if (irq[0]) begin
                interrupt_ack <= 1'b1;
                vector_address <= VECTOR_IRQ0;
            end else if (irq[1]) begin
                interrupt_ack <= 1'b1;
                vector_address <= VECTOR_IRQ1;
            end else if (irq[2]) begin
                interrupt_ack <= 1'b1;
                vector_address <= VECTOR_IRQ2;
            end else if (irq[3]) begin
                interrupt_ack <= 1'b1;
                vector_address <= VECTOR_IRQ3;
            end else begin
                interrupt_ack <= 1'b0;
                vector_address <= 8'h00;
            end
        end else begin
            interrupt_ack <= 1'b0;
        end
    end
end

endmodule


// Control Unit
// Control Unit - Add states to handle memory timing
// Control Unit - Add states to handle memory timing
module ControlUnit(
    input wire clk,
    input wire reset,
    input wire [7:0] instruction,
    input wire interrupt_ack,
    input wire [7:0] mem_read_data,  // Add memory read data input
    output reg [3:0] alu_opcode,
    output reg mem_read,
    output reg mem_write,
    output reg reg_write_enable,
    output reg [1:0] reg_read_addr1,
    output reg [1:0] reg_read_addr2,
    output reg [1:0] reg_write_addr,
    output reg set_interrupt,
    output reg clear_interrupt
);

    reg [1:0] state;  // state of control unit
    localparam FETCH = 2'b00, DECODE = 2'b01, EXECUTE = 2'b10, MEMORY_OP = 2'b11;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= FETCH;
            alu_opcode <= 4'b0000;
            mem_write <= 1'b0;
            mem_read <= 1'b0;
            reg_write_enable <= 1'b0;
            reg_read_addr1 <= 2'b00;
            reg_read_addr2 <= 2'b00;
            reg_write_addr <= 2'b00;
            set_interrupt <= 1'b0;
            clear_interrupt <= 1'b0;
        end else begin
            case (state)
                FETCH: begin
                    mem_read <= 1'b1;  // Set to read memory for the next instruction
                    //$display("FETCH state: Reading instruction");
                    state <= DECODE;
                end

                DECODE: begin
                    mem_read <= 1'b1;  // Stop reading memory after instruction is fetched
                    //$display("DECODE state: Decoding instruction %b", instruction);
                    case (instruction[7:4])
                        4'b0000: begin
                            // ADD r1, r2
                            alu_opcode <= 4'b0000;
                            reg_read_addr1 <= instruction[3:2];
                            reg_read_addr2 <= instruction[1:0];
                            reg_write_addr <= instruction[3:2];
                            reg_write_enable <= 1'b1;
                            state <= EXECUTE;
                        end
                        4'b0001: begin
                            // SUB r1, r2
                            alu_opcode <= 4'b0001;
                            reg_read_addr1 <= instruction[3:2];
                            reg_read_addr2 <= instruction[1:0];
                            reg_write_addr <= instruction[3:2];
                            reg_write_enable <= 1'b1;
                            state <= EXECUTE;
                        end
                        4'b1000: begin
                            // LOAD r1, addr
                            mem_read <= 1'b1;  // Enable memory read
                            reg_write_addr <= instruction[3:2];
                            reg_write_enable <= 1'b1;
                            state <= MEMORY_OP;
                        end
                        4'b1001: begin
                            // STORE r1, addr
                            mem_write <= 1'b1;  // Enable memory write
                            reg_read_addr1 <= instruction[3:2];
                            state <= MEMORY_OP;
                        end
                        default: state <= FETCH;
                    endcase
                end

                MEMORY_OP: begin
                    // Handle memory operations
                    if (instruction[7:4] == 4'b1000) begin
                        // For LOAD: Write the mem_read_data into the register
                        //$display("LOAD instruction: Writing mem_read_data to register");
                        reg_write_enable <= 1'b1;
                        reg_write_enable <= 1'b1;
                    end
                    mem_write <= 1'b0;
                    mem_read <= 1'b0;
                    state <= EXECUTE;
                end

                EXECUTE: begin
                    // Complete ALU operation or other tasks
                    //$display("EXECUTE state: Executing instruction");
                    reg_write_enable <= 1'b1;
                    state <= FETCH;
                end

                default: state <= FETCH;
            endcase
        end
    end

endmodule






//Processor module
// Processor module
module Processor(
    input wire clk,
    input wire reset,
    input wire [7:0] instruction, // External instruction input for simplicity
    input wire [3:0] irq,         // External interrupt request lines
    output wire [7:0] ALU_out,
    output wire [7:0] psw_out,
    output wire interrupt_ack,
    output wire [7:0] mem_read_data
);

    // Internal signals
    wire [3:0] alu_opcode;
    wire [7:0] reg_data1, reg_data2;
    wire [7:0] alu_result;
    wire carry_out, overflow_out;
    wire [7:0] vector_address;
    wire mem_read, mem_write, reg_write_enable;
    wire [1:0] reg_read_addr1, reg_read_addr2, reg_write_addr;
    wire set_interrupt, clear_interrupt;

    // Instantiate the ALU
    ALU alu(
        .a(reg_data1),
        .clk(clk),
        .reset(reset),
        .b(reg_data2),
        .alu_opcode(alu_opcode),
        .ALU_out(alu_result),
        .Carry_out(carry_out),
        .Overflow_out(overflow_out)
    );

    // Instantiate the Register Bank
    RegisterBank register_bank(
        .clk(clk),
        .reset(reset),
        .read_reg1(reg_read_addr1),
        .read_reg2(reg_read_addr2),
        .write_reg(reg_write_addr),
        .write_data(mem_read ? mem_read_data : alu_result),  // Write ALU result or memory data
        .write_enable(reg_write_enable),
        .read_data1(reg_data1),
        .read_data2(reg_data2)
    );

    // Instantiate the Memory Bank
    MemoryBank memory_bank(
        .clk(clk),
        .address(reg_data1),   // Assuming address is coming from reg_data1
        .write_data(reg_data2), // Assuming write data is from reg_data2
        .mem_write(mem_write),
        .mem_read(mem_read),
        .read_data(mem_read_data)
    );

    // Instantiate the PSW (Program Status Word)
    PSW psw(
        .reset(reset),
        .clk(clk),
        .sign_flag(alu_result[7]), // Most significant bit as the sign flag
        .zero_flag(alu_result == 8'b0), // Zero flag
        .carry_flag(carry_out),
        .overflow_flag(overflow_out),
        .aux_carry_flag(aux_carry_flag), 
        .interrupt_enable(set_interrupt), 
        .set_interrupt(set_interrupt),
        .clear_interrupt(clear_interrupt),
        .psw(psw_out)
    );

    // Instantiate the Interrupt Controller
    InterruptController interrupt_controller(
        .clk(clk),
        .reset(reset),
        .irq(irq),
        .interrupt_enable(psw_out[7]), // Interrupt Enable flag from PSW
        .interrupt_ack(interrupt_ack),
        .vector_address(vector_address)
    );

    // Instantiate the Control Unit
    ControlUnit control_unit(
        .clk(clk),
        .reset(reset),
        .instruction(instruction),
        .interrupt_ack(interrupt_ack),
        .mem_read_data(mem_read_data), // Pass mem_read_data to Control Unit
        .alu_opcode(alu_opcode),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .reg_write_enable(reg_write_enable),
        .reg_read_addr1(reg_read_addr1),
        .reg_read_addr2(reg_read_addr2),
        .reg_write_addr(reg_write_addr),
        .set_interrupt(set_interrupt),
        .clear_interrupt(clear_interrupt)
    );

    // Output connections
    assign ALU_out = alu_result;

    // Debugging statements
    // always @(posedge clk) begin
    //     // Memory Operation Debug
    //     if (mem_read || mem_write) begin
    //         $display("Memory Operation: Address=%h, Write_Data=%h, Mem_Read=%b, Mem_Write=%b, Read_Data=%h", reg_data1, reg_data2, mem_read, mem_write, mem_read_data);
    //     end
        
    //     // ALU Debug
    //     $display("ALU Operation: A=%h, B=%h, Opcode=%b, Result=%h", reg_data1, reg_data2, alu_opcode, alu_result);
        
    //     // Register Bank Debug
    //     $display("Register Bank: Reg1=%h, Reg2=%h, Write_Enable=%b, Write_Data=%h, Write_Reg=%b", reg_data1, reg_data2, reg_write_enable, alu_result, reg_write_addr);
    // end

endmodule

