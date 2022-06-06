`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 
// Students: HELIN ASLI AKSOY & HARUN CITFCI
// 
// Create Date: 08.04.2022 23:06:27
// Design Name: 
// Module Name: simulation_test
// Project Name: Homework 1
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


module and_gate_test();
    
    reg A;
    reg B;
    
    wire C;
    
    and_gate uut(A,B,C);
    
    initial begin
        A = 1'b0;   B = 1'b0;   #250;  
        A = 1'b0;   B = 1'b1;   #250;
        A = 1'b1;   B = 1'b0;   #250;  
        A = 1'b1;   B = 1'b1;   #250;
    end
endmodule

module or_gate_test();
    
    reg A;
    reg B;
    
    wire C;
    
    or_gate uut(A,B,C);
    
    initial begin
        A = 1'b0;   B = 1'b0;   #250;  
        A = 1'b0;   B = 1'b1;   #250;
        A = 1'b1;   B = 1'b0;   #250;  
        A = 1'b1;   B = 1'b1;   #250;
    end
endmodule

module not_gate_test();
    
    reg A;
    
    wire B;
    
    not_gate uut(A,B);
    
    initial begin
        A = 1'b0;   #250;  
        A = 1'b1;   #250;
       
    end
endmodule
module nand_gate_test();
    
    reg A;
    reg B;
    
    wire C;
    
    nand_gate uut(A,B,C);
    
    initial begin
        A = 1'b0;   B = 1'b0;   #250;  
        A = 1'b0;   B = 1'b1;   #250;
        A = 1'b1;   B = 1'b0;   #250;  
        A = 1'b1;   B = 1'b1;   #250;
    end
endmodule
module xor_gate_test();
    
    reg A;
    reg B;
    
    wire C;
    
    xor_gate uut(A,B,C);
    
    initial begin
        A = 1'b0;   B = 1'b0;   #250;  
        A = 1'b0;   B = 1'b1;   #250;
        A = 1'b1;   B = 1'b0;   #250;  
        A = 1'b1;   B = 1'b1;   #250;
    end
endmodule

module half_adder_test(); 
   reg A;
   reg B;
   wire C;
   wire S;
   half_adder uut(A,B,C,S);
   
   initial begin
       A = 0; B = 0;#250;
       A = 0; B = 1;#250;
       A = 1; B = 0;#250;
       A = 1; B = 1;#250;
   end
endmodule
module full_adder_test(); 
   reg A;
   reg B;
   reg Carry_in;
   wire Carry_out;
   wire S;
   full_adder uut(A,B,Carry_in,Carry_out,S);
   
   initial begin
       A = 1'b0; B = 1'b0; Carry_in = 1'b0;#125;
       A = 1'b0; B = 1'b0; Carry_in = 1'b1;#125;
       A = 1'b0; B = 1'b1; Carry_in = 1'b0;#125;
       A = 1'b0; B = 1'b1; Carry_in = 1'b1;#125;
       A = 1'b1; B = 1'b0; Carry_in = 1'b0;#125;
       A = 1'b1; B = 1'b0; Carry_in = 1'b1;#125;
       A = 1'b1; B = 1'b1; Carry_in = 1'b0;#125;
       A = 1'b1; B = 1'b1; Carry_in = 1'b1;#125;
       
   end
endmodule
module four_bit_full_adder_test(); 
   reg [3:0]A;
   reg [3:0]B;
   reg Cin;
   wire Cout;
   wire [3:0]S;
   four_bit_full_adder uut(A,B,Cin,Cout,S);
   
   initial begin
       A = 4'd8; B = 4'd1; Cin = 1'd0;#125;
       A = 4'd2; B = 4'd7; Cin = 1'd0;#125;
       A = 4'd4; B = 4'd5; Cin = 1'd0;#125;
       A = 4'd11; B = 4'd10; Cin = 1'd0;#125;
       A = 4'd14; B = 4'd5; Cin = 1'd0;#125;
       A = 4'd15; B = 4'd9; Cin = 1'd0;#125;
       A = 4'd6; B = 4'd3; Cin = 1'd0;#125;
       A = 4'd8; B = 4'd12; Cin = 1'd0;#125;
       
   end
endmodule
module eight_bit_full_adder_test(); 
   reg [7:0]A;
   reg [7:0]B;
   reg Cin;
   wire Cout;
   wire [7:0]S;
   eight_bit_full_adder uut(A,B,Cin,Cout,S);
   
   initial begin
       A = 8'd29; B = 8'd5; Cin = 1'd0;#125;
       A = 8'd51; B = 8'd92; Cin = 1'd0;#125;
       A = 8'd17; B = 8'd28; Cin = 1'd0;#125;
       A = 8'd191; B = 8'd2; Cin = 1'd0;#125;
       A = 8'd200; B = 8'd95; Cin = 1'd0;#125;
       A = 8'd49; B = 8'd25; Cin = 1'd0;#125;
       A = 8'd78; B = 8'd252; Cin = 1'd0;#125;
       A = 8'd43; B = 8'd59; Cin = 1'd0;#125;
   end
endmodule

module sixteen_bit_full_adder_subtractor_test(); 
   reg [15:0]A;
   reg [15:0]B_input;
   reg X;           //1 when subraction, 0 when summation
   wire Cout;
   wire [15:0]s;
   
   sixteen_bit_full_adder_subtractor uut(A,B_input,X,Cout,s);
   
   initial begin
       A = 16'd23; B_input = 16'd3; X = 1'b0;#125;
       A = 16'd21; B_input = 16'd75; X = 1'b0;#125;
       A = 16'd16800; B_input = 16'd16900; X = 1'b0;#125;
       A = 16'd69834; B_input = 16'd66500; X = 1'b0;#125;
       A = 16'd325; B_input = 16'd97; X = 1'b0;#125;
       A = 16'd44; B_input = 16'd190; X = 1'b0;#125;
       A = 16'd463; B_input = 16'd241; X = 1'b0;#125;
       A = 16'd86; B_input = 16'd572; X = 1'b0;#125;
       
   end
endmodule
module part11_test(); 
   reg [15:0]A;
   reg [15:0]B;
   wire Cout;
   wire [15:0]S;
   part11 uut(A,B,Cout,S);
   
   initial begin
       A = 16'd32; B = 16'd7; #125;
       A = 16'd21; B = 16'd85; #125;
       A = 16'd16; B = 16'd36; #125;
       A = 16'd256; B = 16'd5; #125;
       A = 16'd200; B = 16'd95;#125;
       A = 16'd45; B = 16'd153; #125;
       A = 16'd36; B = 16'd255; #125;
       A = 16'd25; B = 16'd56; #125;
   end
endmodule
