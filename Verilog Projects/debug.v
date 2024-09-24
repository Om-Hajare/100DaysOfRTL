module ALU (
    input [7:0] A, B,
    input [2:0] ALU_Sel,
    output reg [7:0] ALU_Out,
    output reg Zero,
    output reg Carry
);
    always @(*) begin
        case (ALU_Sel)
            3'b000: {Carry, ALU_Out} = A + B; // ADD
            3'b001: {Carry, ALU_Out} = A - B; // SUB
            3'b010: ALU_Out = A & B; // AND
            3'b011: ALU_Out = A | B; // OR
            3'b100: ALU_Out = A ^ B; // XOR
            3'b101: ALU_Out = ~A;    // NOT
            default: ALU_Out = 8'b00000000;
        endcase
        Zero = (ALU_Out == 8'b00000000);
    end
endmodule
module PSW (
    input ZeroFlag,
    input CarryFlag,
    output reg [1:0] PSW
);
    always @(*) begin
        PSW = {CarryFlag, ZeroFlag};
    end
endmodule
module RegisterBank (
    input clk,
    input [2:0] regAddr,
    input [7:0] writeData,
    input writeEnable,
    output reg [7:0] readData
);
    reg [7:0] registers [7:0]; // 8 registers, 8-bit each

    always @(posedge clk) begin
        if (writeEnable) begin
            registers[regAddr] <= writeData;
        end
    end

    always @(*) begin
        readData = registers[regAddr];
    end
endmodule
module MemoryBank (
    input clk,
    input [7:0] addr,
    input [7:0] writeData,
    input memWrite,
    output reg [7:0] readData
);
    reg [7:0] memory [255:0]; // 256 bytes of memory

    always @(posedge clk) begin
        if (memWrite) begin
            memory[addr] <= writeData;
        end
    end

    always @(*) begin
        readData = memory[addr];
    end
endmodule
module InterruptController (
    input clk,
    input interrupt,
    output reg interruptSignal
);
    always @(posedge clk) begin
        interruptSignal = interrupt;
    end
endmodule
module ControlUnit (
    input [7:0] opcode,
    output reg [2:0] ALU_Sel,
    output reg memWrite,
    output reg load,
    output reg store
);

    always @(*) begin
        // Default values
        ALU_Sel = 3'b000;
        memWrite = 0;
        load = 0;
        store = 0;
        
        case (opcode[7:0])
            8'b00000000: begin // LOAD
                load = 1;
            end
            8'b00010000: begin // STORE
                store = 1;
            end
            8'b00100000: begin // ADD
                ALU_Sel = 3'b000;
            end
            8'b00110000: begin // SUB
                ALU_Sel = 3'b001;
            end
            // Add other ALU operations as needed
            default: begin
                // Handle unknown instructions
            end
        endcase
    end

endmodule

module Processor (
    input clk,
    input [7:0] instruction,
    input [7:0] dataIn,
    output reg [7:0] dataOut
);

    // Internal registers
    reg [7:0] regFile[0:7]; // Register file with 8 registers
    reg [7:0] mem[0:255];   // Memory with 256 locations
    reg [7:0] regData;      // Data read from registers
    reg [7:0] memAddr;      // Address for memory access
    reg [7:0] aluResult;    // Result from ALU

    // Control signals
    reg memWrite, loadEnable, storeEnable;
    wire [2:0] ALU_Sel;
    wire Zero, Carry;

    // ALU Instance
    ALU alu (
        .A(regData),
        .B(dataIn),
        .ALU_Sel(ALU_Sel),
        .ALU_Out(aluResult),
        .Zero(Zero),
        .Carry(Carry)
    );

    // Control Unit Instance
    ControlUnit controlUnit (
        .opcode(instruction),
        .ALU_Sel(ALU_Sel),
        .memWrite(memWrite),
        .load(loadEnable),
        .store(storeEnable)
    );

    // Update regData, regFile, and mem based on control signals
    always @(posedge clk) begin
        if (storeEnable) begin
            mem[memAddr] <= regData; // Store data from register to memory
        end
        if (loadEnable) begin
            regData <= mem[memAddr]; // Load data from memory to register
        end
        if (storeEnable) begin
            regFile[instruction[2:0]] <= aluResult; // Write ALU result to register
        end
        memAddr <= instruction[7:0]; // Update memory address
    end

    // DataOut assignment based on control signals
    always @(*) begin
        if (loadEnable) begin
            dataOut = regData; // Output data from register
        end else if (storeEnable) begin
            dataOut = aluResult; // Output data from ALU result
        end else begin
            dataOut = 8'bz; // High impedance if not loading or storing
        end
    end

endmodule

