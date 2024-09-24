module Traffic_light_control (
    input clk,rst,
    output reg [2:0]light_M1,
    output reg [2:0]light_M2,
    output reg [2:0]light_M3,
    output reg [2:0]light_M4,
    output reg [2:0]light_M5,
    output reg [2:0]light_M6

    
);
    parameter S1 = 0, S2 = 1, S3=2, S4=3,S5 = 4, S6=5, S7 =6 , S8 =7 ;
    reg [7:0]count;
    reg [7:0] ps;
    parameter T1 = 2, T2 = 2, T3 = 2, T4 = 2,  T5 = 2, T6 = 2, T7 = 2, T8 = 2 ;

    always @(posedge clk or posedge rst) begin
        if (rst==1)
        begin
            ps<=S1;
            count<=0;

        end
        else
        case (ps)
            S1:if (count<T1) begin
                ps<=S1;
                count<=count+1;

                
            end 
            else
            begin
                ps<=S2;
                count<=0;
            end
            S2: if (count<T2) begin
                ps<=S2;
                count<=count+1;


                
            end
            else begin
                ps<=S3;
                count<=0;
            end
            S3: if (count<T3) begin
                ps<=S3;
                count<=count+1;


                
            end
            else begin
                ps<=S4;
                count<=0;
            end
            S4: if (count<T4) begin
                ps<=S4;
                count<=count+1;


                
            end
            else begin
                ps<=S5;
                count<=0;
            end
            S5: if (count<T5) begin
                ps<=S5;
                count<=count+1;


                
            end
            else begin
                ps<=S6;
                count<=0;
            end
            S6: if (count<T6) begin
                ps<=S6;
                count<=count+1;


                
            end
            else begin
                ps<=S7;
                count<=0;
            end
            S5: if (count<T7) begin
                ps<=S7;
                count<=count+1;


                
            end
            else begin
                ps<=S8;
                count<=0;
            end
            S5: if (count<T8) begin
                ps<=S8;
                count<=count+1;


                
            end
            else begin
                ps<=S1;
                count<=0;
            end

            default: ps<=S1;
        endcase
        
    end
    always @(ps) begin
        case (ps)
        // light = RYG
            S1:begin
                light_M1<=3'b001;
                light_M2<=3'b001;
                light_M3<=3'b100;
                light_M4<=3'b001;
                light_M5<=3'b100;
                light_M6<=3'b100;


            end 
            S2:begin
                light_M1<=3'b001;
                light_M2<=3'b010;
                light_M3<=3'b100;
                light_M4<=3'b001;
                light_M5<=3'b100;
                light_M6<=3'b100;

            end 
            S3:begin
                light_M1<=3'b001;
                light_M2<=3'b100;
                light_M3<=3'b100;
                light_M4<=3'b001;
                light_M5<=3'b100;
                light_M6<=3'b001;

            end 
            S4:begin
                light_M1<=3'b001;
                light_M2<=3'b100;
                light_M3<=3'b100;
                light_M4<=3'b010;
                light_M5<=3'b100;
                light_M6<=3'b001;


            end 
            S5:begin
                light_M1<=3'b001;
                light_M2<=3'b100;
                light_M3<=3'b001;
                light_M4<=3'b100;
                light_M5<=3'b100;
                light_M6<=3'b001;
            end 
            S6:begin
                light_M1<=3'b010;
                light_M2<=3'b100;
                light_M3<=3'b010;
                light_M4<=3'b100;
                light_M5<=3'b100;
                light_M6<=3'b001;

            end
            S7:begin
                light_M1<=3'b100;
                light_M2<=3'b100;
                light_M3<=3'b100;
                light_M4<=3'b001;
                light_M5<=3'b001;
                light_M6<=3'b001;
            end
            S8:begin
                light_M1<=3'b100;
                light_M2<=3'b100;
                light_M3<=3'b100;
                light_M4<=3'b001;
                light_M5<=3'b010;
                light_M6<=3'b010;
            end
            default: begin
                light_M1<=3'b000;
                light_M2<=3'b000;
                light_M3<=3'b000;
                light_M4<=3'b000;
                light_M5<=3'b000;
                light_M6<=3'b000;
            end

        endcase
        
    end

        
    endmodule