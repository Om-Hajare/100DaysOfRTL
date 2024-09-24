`include "Traffic_light_control.v"

module Traffic_light_control_tb;
reg clk,rst;
wire [2:0]light_M1;
wire [2:0]light_S;
wire [2:0]light_MT;
wire [2:0]light_M2;
Traffic_light_control uut(.clk(clk) , .rst(rst) , .light_M1(light_M1) , .light_M2(light_M2)  ,.light_M3(light_M3),.light_M4(light_M4),.light_M5(light_M5),.light_M6(light_M6) );
initial
begin
    clk=1'b0;
    forever #(1) clk=~clk;
end
//    initial
//    $stop;//to add ps
initial
begin
    rst=0;
    #1;
    rst=1;
    #2;
    rst=0;
    #10;
    $finish;
    end

initial begin
    $dumpfile("Traffic_light_control.vcd");
    $dumpvars(1);
end



endmodule