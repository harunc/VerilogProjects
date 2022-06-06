`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITU BBF
// Engineer: Kadir Ozlem
// 
// Create Date: 26/04/2022 10:50:21 PM
// Design Name: RAM Module
// Module Name: Memory
// Project Name: Computer Organization Project 1
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

//Please put it in your modules.v 
module Memory(
    input wire[7:0] address,
    input wire[7:0] data,
    input wire wr, //Read = 0, Write = 1
    input wire cs, //Chip is enable when cs = 0
    input wire clock,
    output reg[7:0] o // Output
);
    //Declaration o?f the RAM Area
    reg[7:0] RAM_DATA[0:255];
    //Read Ram data from the file
    initial $readmemh("RAM.mem", RAM_DATA);
    //Read the selected data from RAM
    always @(*) begin
        o = ~wr && ~cs ? RAM_DATA[address] : 8'hZ;
    end
    
    //Write the data to RAM
    always @(posedge clock) begin
        if (wr && ~cs) begin
            RAM_DATA[address] <= data; 
        end
    end
endmodule

//GATES//
module and_gate(
    input A,
    input B,
    output C
    );
    assign C = A & B;
endmodule

module or_gate(
    input A,
    input B,
    output C
    );
    assign C = A | B;
endmodule

module not_gate(
    input A,
    output C
    );
    assign C = ~A;
endmodule

module xor_gate(
    input A,
    input B,
    output C
    );
    assign C = (~A & B) + (A & ~B);
endmodule

module nand_gate(
    input A,
    input B,
    output C
    );
    assign C = ~(A & B);
endmodule

module nand_three_gate(
    input A,
    input B,
    input C,
    output E
    );
    wire x;
    and_three_gate one(A,B,C,x);
    assign E = ~x;
endmodule

module and_three_gate(
    input A,
    input B,
    input C,
    output E
    );
    wire x;
    and_gate one(A,B,x);
    and_gate sec(C,x,E);
endmodule

module or_gate_3input(
    input A,
    input B,
    input C,
    output D
    );
    assign D = A | B | C;
endmodule 

//Gates end

module multiplexer_8to1(
    output out,
    input i_0,
    input i_1,
    input i_2,
    input i_3,
    input i_4,
    input i_5,
    input i_6,
    input i_7,
    input s_0,
    input s_1,
    input s_2
    );
    assign out = ((((!s_0)&(!s_1)&(!s_2))&i_0)|
                 (((!s_0)&(!s_1)&(s_2))&i_1)|
                 (((!s_0)&(s_1)&(!s_2))&i_2)|
                 (((!s_0)&(s_1)&(s_2))&i_3)|
                 (((s_0)&(!s_1)&(!s_2))&i_4)|
                 (((s_0)&(!s_1)&(s_2))&i_5)|
                 (((s_0)&(s_1)&(!s_2))&i_6)|
                 (((s_0)&(s_1)&(s_2))&i_7));

endmodule

module multiplexer_4to1(out,i_0,i_1,i_2,i_3,s);
    input [15:0] i_0;
    input [15:0] i_1;
    input [15:0] i_2;
    input [15:0] i_3;
    input [1:0]s;
    output reg[15:0] out;
    always @(i_0,i_1,i_2,i_3,s)begin
    case(s)
    2'b00: out = i_0;
    2'b01: out = i_1;
    2'b10: out = i_2;
    2'b11: out = i_3;
    endcase
    end
endmodule
module eight_bit_multiplexer_4to1(out,i_0,i_1,i_2,i_3,s);
    input [7:0] i_0;
    input [7:0] i_1;
    input [7:0] i_2;
    input [7:0] i_3;
    input [1:0]s;
    output reg[7:0] out;
    always @(i_0,i_1,i_2,i_3,s)begin
    case(s)
    2'b00: out = i_0;
    2'b01: out = i_1;
    2'b10: out = i_2;
    2'b11: out = i_3;
    endcase
    end
endmodule


//3to1
module multiplexer_4to1_8bit(out,i_0,i_2,i_3,s);
    input [7:0] i_0;
    input [7:0] i_2;
    input [7:0] i_3;
    input [1:0]s;
    output reg[7:0] out;
    always @(i_0,i_2,i_3,s)begin
    case(s)
    2'b00: out = i_0;
    2'b01: out = i_0;
    2'b10: out = i_2;
    2'b11: out = i_3;
    endcase
    end
 
    

endmodule
module multiplexer_2to1_8bit(out,i_0,i_1,s);
    input [7:0] i_0;
    input [7:0] i_1;
    input s;
    output reg[7:0] out;
    always @(i_0,i_1,s)begin
    case(s)
    2'b0: out = i_0;
    2'b1: out = i_1;
    endcase
    end
    

endmodule
module decoder_3to8(
    input A,
    input B,
    input C,
    output D0,
    output D1,
    output D2,
    output D3,
    output D4,
    output D5,
    output D6,
    output D7
    );
    assign D0 = ~C & ~B & ~A;
    assign D1 = ~C & ~B & A;
    assign D2 = ~C & B & ~A;
    assign D3 = ~C & B & A;
    assign D4 = C & ~B & ~A;
    assign D5 = C & ~B & A;
    assign D6 = C & B & ~A;
    assign D7 = C & B & A;
endmodule


//ADDER-SUBTRACTOR//
module half_adder(
    input A,
    input B,
    output C,
    output S
    );
    
    xor_gate first_xor(A,B,S);
    and_gate first_and(A,B,C);
endmodule


module full_adder(
    input A,
    input B,
    input Carry_in,
    output Carry_out,
    output S
    );
    wire middle_s;
    wire first_c;
    wire second_c;
    
    half_adder first_half(A,B,first_c,middle_s);
    half_adder second_half(middle_s,Carry_in,second_c,S);
    
    or_gate first_or(first_c, second_c, Carry_out);
endmodule


module four_bit_full_adder(
    input [3:0] X,
    input [3:0] Y,
    input Cin,
    output Cout,
    output [3:0] s
    );
    wire co1;
    wire co2;
    wire co3;
    full_adder digit0 (X[0], Y[0], Cin, co1,s[0]);
    full_adder digit1 (X[1], Y[1], co1, co2,s[1]);
    full_adder digit2 (X[2], Y[2], co2, co3,s[2]);
    full_adder digit3 (X[3], Y[3], co3, Cout,s[3]);
endmodule


module eight_bit_full_adder(
    input [7:0] X,
    input [7:0] Y,
    input Cin,
    output Cout,
    output [7:0] s
    );
    wire co1;
    wire co2;
    wire co3;
    wire co4;
    wire co5;
    wire co6;
    wire co7;
    
    full_adder digit0 (X[0], Y[0], Cin, co1,s[0]);
    full_adder digit1 (X[1], Y[1], co1, co2,s[1]);
    full_adder digit2 (X[2], Y[2], co2, co3,s[2]);
    full_adder digit3 (X[3], Y[3], co3, co4,s[3]);
    full_adder digit4 (X[4], Y[4], co4, co5,s[4]);
    full_adder digit5 (X[5], Y[5], co5, co6,s[5]);
    full_adder digit6 (X[6], Y[6], co6, co7,s[6]);
    full_adder digit7 (X[7], Y[7], co7, Cout,s[7]);
endmodule

module A_sixteen_bit_full_adder(
    input [15:0] X,
    input [15:0] Y,
    input Cin,
    output Cout,
    output [15:0] s
    );
    wire co1;
    eight_bit_full_adder digit0 (X[7:0], Y[7:0], Cin, co1,s[7:0]);
    eight_bit_full_adder digit1 (X[15:8], Y[15:8], co1, Cout,s[15:8]);
endmodule

module A_sixteen_bit_full_adder_subtractor(
    input [15:0] A_input,
    input [15:0] B_input,
    input X,    //1 when subraction, 0 when summation
    output Cout,
    output [15:0] s 
    );
    wire [15:0] B_after_Xor ;
    xor_gate B_Xor_X(B_input[0], X, B_after_Xor[0]);
    xor_gate B_Xor_X1(B_input[1], X, B_after_Xor[1]);
    xor_gate B_Xor_X2(B_input[2], X, B_after_Xor[2]);
    xor_gate B_Xor_X3(B_input[3], X, B_after_Xor[3]);
    xor_gate B_Xor_X4(B_input[4], X, B_after_Xor[4]);
    xor_gate B_Xor_X5(B_input[5], X, B_after_Xor[5]);
    xor_gate B_Xor_X6(B_input[6], X, B_after_Xor[6]);
    xor_gate B_Xor_X7(B_input[7], X, B_after_Xor[7]);
    xor_gate B_Xor_X8(B_input[8], X, B_after_Xor[8]);
    xor_gate B_Xor_X9(B_input[9], X, B_after_Xor[9]);
    xor_gate B_Xor_X10(B_input[10], X, B_after_Xor[10]);
    xor_gate B_Xor_X11(B_input[11], X, B_after_Xor[11]);
    xor_gate B_Xor_X12(B_input[12], X, B_after_Xor[12]);
    xor_gate B_Xor_X13(B_input[13], X, B_after_Xor[13]);
    xor_gate B_Xor_X14(B_input[14], X, B_after_Xor[14]);
    xor_gate B_Xor_X15(B_input[15], X, B_after_Xor[15]);
    
    A_sixteen_bit_full_adder result(A_input, B_after_Xor, X, Cout, s);
endmodule

module A_eight_bit_full_adder_subtractor(
    input [7:0] A_input,
    input [7:0] B_input,
    input X,    //1 when subraction, 0 when summation
    output Cout,
    output [7:0] s 
    );
    wire [7:0] B_after_Xor ;
    xor_gate B_Xor_X(B_input[0], X, B_after_Xor[0]);
    xor_gate B_Xor_X1(B_input[1], X, B_after_Xor[1]);
    xor_gate B_Xor_X2(B_input[2], X, B_after_Xor[2]);
    xor_gate B_Xor_X3(B_input[3], X, B_after_Xor[3]);
    xor_gate B_Xor_X4(B_input[4], X, B_after_Xor[4]);
    xor_gate B_Xor_X5(B_input[5], X, B_after_Xor[5]);
    xor_gate B_Xor_X6(B_input[6], X, B_after_Xor[6]);
    xor_gate B_Xor_X7(B_input[7], X, B_after_Xor[7]);
    
    eight_bit_full_adder result(A_input, B_after_Xor, X, Cout, s);
endmodule

//Adder subtractors end

//D FLIP FLOP

module d_flip_flop(d,reset,en,clk,q);
    input d;
    input reset;
    input en;
    input clk;
    output reg q;

    always @ (posedge clk or posedge reset)
    
    if (reset)
    
    q <= 1'b0;
    
    else if (en)
    
    q <= d;
endmodule

module eight_bit_d_flip_flop(
    input [7:0] a_input,
    input reset,
    input enable,
    input clock,
    output [7:0] a_output
    );

    d_flip_flop D1(a_input[0],reset,enable,clock,a_output[0]);
    d_flip_flop D2(a_input[1],reset,enable,clock,a_output[1]);
    d_flip_flop D3(a_input[2],reset,enable,clock,a_output[2]);
    d_flip_flop D4(a_input[3],reset,enable,clock,a_output[3]);
    d_flip_flop D5(a_input[4],reset,enable,clock,a_output[4]);
    d_flip_flop D6(a_input[5],reset,enable,clock,a_output[5]);
    d_flip_flop D7(a_input[6],reset,enable,clock,a_output[6]);
    d_flip_flop D8(a_input[7],reset,enable,clock,a_output[7]);
endmodule

module sixteen_bit_d_flip_flop(
    input [15:0] a_input,
    input reset,
    input enable,
    input clock,
    output [15:0] a_output
    );

    d_flip_flop D1(a_input[0],reset,enable,clock,a_output[0]);
    d_flip_flop D2(a_input[1],reset,enable,clock,a_output[1]);
    d_flip_flop D3(a_input[2],reset,enable,clock,a_output[2]);
    d_flip_flop D4(a_input[3],reset,enable,clock,a_output[3]);
    d_flip_flop D5(a_input[4],reset,enable,clock,a_output[4]);
    d_flip_flop D6(a_input[5],reset,enable,clock,a_output[5]);
    d_flip_flop D7(a_input[6],reset,enable,clock,a_output[6]);
    d_flip_flop D8(a_input[7],reset,enable,clock,a_output[7]);
    d_flip_flop D9(a_input[8],reset,enable,clock,a_output[8]);
    d_flip_flop D10(a_input[9],reset,enable,clock,a_output[9]);
    d_flip_flop D11(a_input[10],reset,enable,clock,a_output[10]);
    d_flip_flop D12(a_input[11],reset,enable,clock,a_output[11]);
    d_flip_flop D13(a_input[12],reset,enable,clock,a_output[12]);
    d_flip_flop D14(a_input[13],reset,enable,clock,a_output[13]);
    d_flip_flop D15(a_input[14],reset,enable,clock,a_output[14]);
    d_flip_flop D16(a_input[15],reset,enable,clock,a_output[15]);
endmodule

module eight_bit_register(
    input [7:0] A,
    input [1:0]funsel,
    input E,
    input CLK,
    input reset,
    output [7:0]out
    );
    wire cout_1;
    wire cout_2;
    wire [7:0]multi;
    wire [7:0]increment;
    wire [7:0]decrement;
 
    A_eight_bit_full_adder_subtractor adder(out,8'd1,1'd0,cout_1,increment);
    A_eight_bit_full_adder_subtractor subtractor(out,8'd1,1'd1,cout_2,decrement);
    
    multiplexer_4to1 multiplexer(multi,decrement,increment,A,8'd0,funsel);
    
    eight_bit_d_flip_flop d_FF(multi,reset,E,CLK,out);
    
endmodule

module sixteen_bit_register(
    input [15:0] A,
    input [1:0]funsel,
    input E,
    input CLK,
    input reset,
    output [15:0]out
    );
    wire cout_1;
    wire cout_2;
    wire [15:0]multi;
    wire [15:0]increment;
    wire [15:0]decrement;
 
    A_sixteen_bit_full_adder_subtractor adder(out,16'd1,1'd0,cout_1,increment);
    A_sixteen_bit_full_adder_subtractor subtractor(out,16'd1,1'd1,cout_2,decrement);
    
    multiplexer_4to1 multiplexer(multi,decrement,increment,A,16'd0,funsel);
    
    sixteen_bit_d_flip_flop d_FF(multi,reset,E,CLK,out);
    
endmodule


module part2a (input_a,funsel,regsel,outAsel,outBsel,clk,outA,outB);
    input [7:0]input_a;
    input [1:0]funsel;
    input [3:0] regsel;
    input [1:0]outAsel;
    input [1:0]outBsel;
    input clk;
    output [7:0] outA;
    output [7:0] outB;
    wire [7:0] wire_r1;
    wire [7:0] wire_r2;
    wire [7:0] wire_r3;
    wire [7:0] wire_r4;
    eight_bit_register r1(input_a,funsel,~regsel[3],clk,1'b0,wire_r1);
    eight_bit_register r2(input_a,funsel,~regsel[2],clk,1'b0,wire_r2);
    eight_bit_register r3(input_a,funsel,~regsel[1],clk,1'b0,wire_r3);
    eight_bit_register r4(input_a,funsel,~regsel[0],clk,1'b0,wire_r4);
    
    multiplexer_4to1 muxA(outA,wire_r1,wire_r2,wire_r3,wire_r4,outAsel);
    multiplexer_4to1 muxB(outB,wire_r1,wire_r2,wire_r3,wire_r4,outBsel);
endmodule


module part2b (input_a,funsel,regsel,outCsel,outDsel,clk,outC,outD);
    input [7:0]input_a;
    input [1:0]funsel;
    input [2:0] regsel;
    input [1:0]outCsel;
    input [1:0]outDsel;
    input clk;
    output [7:0] outC;
    output [7:0] outD;
    wire [7:0] wire_pc;
    wire [7:0] wire_ar;
    wire [7:0] wire_sp;
    eight_bit_register pc(input_a,funsel,~regsel[2],clk,1'd0,wire_pc);
    eight_bit_register ar(input_a,funsel,~regsel[1],clk,1'd0,wire_ar);
    eight_bit_register sp(input_a,funsel,~regsel[0],clk,1'd0,wire_sp);
    
    multiplexer_4to1 muxC(outC,wire_pc,wire_pc,wire_ar,wire_sp,outCsel);
    multiplexer_4to1 muxD(outD,wire_pc,wire_pc,wire_ar,wire_sp,outDsel);
endmodule


module assignbits(dFF, inputs,LH,out);
    input [15:0]dFF;
    input LH;
    input [7:0]inputs;
    output reg [15:0]out;
    
    always @* begin
    if(LH) begin
        out[15:8] <= dFF[15:8];
        out[7:0] <= inputs;
        end
    else begin
        out[7:0] <= dFF[7:0];
        out[15:8] <= inputs;
        end
    end 
    
    
endmodule
module part2c (input_A,LH,funsel,clock,enable,IR_out);
    input [7:0]input_A;
    input LH;
    input [1:0]funsel;
    input clock;
    input enable;
    output [15:0]IR_out;
    wire cout1;
    wire cout2;
    wire [15:0] mux_LH;
    wire [15:0] mux_funsel;
    wire [15:0] ad;
    wire [15:0] sub;
    assignbits ass(IR_out,input_A,LH,mux_LH);
    A_sixteen_bit_full_adder_subtractor adder(IR_out,16'd1,1'b0,cout1,ad);
    A_sixteen_bit_full_adder_subtractor subtractor(IR_out,16'd1,1'b1,cout2,sub);
    multiplexer_4to1 mux2(mux_funsel,sub,ad,mux_LH,16'd0,funsel);
    sixteen_bit_d_flip_flop dFF(mux_funsel,1'b0,enable,clock,IR_out);
    
endmodule

   module alu(
         input [7:0] A,B,  // ALU 8-bit Inputs                 
         input [3:0] ALU_Sel,// ALU Selection
         output [7:0] ALU_Out, // ALU 8-bit Output
         output reg [3:0]ALUOutFlag
      );
      
      wire CarryOut; // Carry Out Flag    
      reg [7:0] ALU_Result;
      wire [8:0] tmp;
      // ALU out
      assign tmp = {1'b0,A} + {1'b0,B};
      assign CarryOut = tmp[8]; // Carryout flag
      wire CarryOutWithBirsey;
      wire [8:0] temp;
      assign temp = {1'b0,A} + {1'b0,B} + {8'b0 ,ALUOutFlag[1]} ;
      assign CarryOutWithBirsey = temp[8];
      reg CarryOutShl;
      reg CarryOutShr;
      always @(*)
      begin
         case(ALU_Sel)
         4'b0000:
         begin // Addition
            ALU_Result <= A;
            if(ALU_Result == 8'b0)
            begin
               ALUOutFlag[3] <= 1'b1;
            end
            else
            begin
               ALUOutFlag[3] <= 1'b0;
            end
            if(ALU_Result[7] == 1'b1)
            begin
               ALUOutFlag[2] <= 1'b1;
               end
               else
               begin
                  ALUOutFlag[2] <= 1'b0;
               end
         end
         4'b0001: // Subtraction
            begin
            ALU_Result <= B;
            if(ALU_Result == 8'b0)
            begin
            ALUOutFlag[3] <= 1'b1;
            end
            else
            begin
               ALUOutFlag[3] <= 1'b0;
            end
            if(ALU_Result[7] == 1'b1)
            begin
            ALUOutFlag[2] <= 1'b1;
            end
            else
            begin
               ALUOutFlag[2] <= 1'b0;
            end
         end
         4'b0010: // Multiplication
            begin
            ALU_Result <= ~A;
            if(ALU_Result == 8'b0)
               begin
               ALUOutFlag[3] <= 1'b1;
               end
               else
               begin
                  ALUOutFlag[3] <= 1'b0;
               end
               if(ALU_Result[7] == 1'b1)
               begin
               ALUOutFlag[2] <= 1'b1;
               end
               else
               begin
                  ALUOutFlag[2] <= 1'b0;
               end
            end
         4'b0011: // Division
            begin
            ALU_Result <= ~B;
            if(ALU_Result == 8'b0)
               begin
               ALUOutFlag[3] <= 1'b1;
               end
            else
            begin
               ALUOutFlag[3] <= 1'b0;
            end
               if(ALU_Result[7] == 1'b1)
               begin
               ALUOutFlag[2] <= 1'b1;
               end
               else
               begin
                  ALUOutFlag[2] <= 1'b0;
               end
            end
         4'b0100: // Logical shift left
            begin
            ALU_Result <= A + B;
            if(ALU_Result == 8'b0)
               begin
               ALUOutFlag[3] <= 1'b1;
               end
               else
               begin
                  ALUOutFlag[3] <= 1'b0;
               end
               if(ALU_Result[7] == 1'b1)
               begin
               ALUOutFlag[2] <= 1'b1;
               end
               else
               begin
                  ALUOutFlag[2] <= 1'b0;
               end
               if(CarryOut == 1'b1)
               begin
                  ALUOutFlag[1] <= 1'b1;
               end
               else
               begin 
                  ALUOutFlag[1] <= 1'b0;
               end
               if( A[7] == B[7] )
               begin
                  if( ALU_Result[7] != A[7])
                  begin
                        ALUOutFlag[0] <= 1'b1;
                  end
                  else
                  begin
                        ALUOutFlag[0] <= 1'b0;
                  end
               end
               else
               begin
                  ALUOutFlag[0] <= 1'b0;
               end
            end
            4'b0101: // Logical shift right
               begin
               ALU_Result <= A + B + {7'b0 , ALUOutFlag[1]};
            if(ALU_Result == 8'b0)
               begin
                  ALUOutFlag[3] <= 1'b1;
               end
            else
            begin
               ALUOutFlag[3] <= 1'b0;
            end
            if(ALU_Result[7] == 1'b1)
               begin
                  ALUOutFlag[2] <= 1'b1;
                  end
            else
            begin
               ALUOutFlag[2] <= 1'b0;
            end
            if(CarryOutWithBirsey == 1'b1)
               begin
                     ALUOutFlag[1] <= 1'b1;
               end
            else
            begin
               ALUOutFlag[1] <= 1'b0;
            end
            if( A[7] == B[7] )
               begin
                  if( ALU_Result[7] != A[7])
                  begin
                        ALUOutFlag[0] <= 1'b1;
                  end
                  else
                  begin
                        ALUOutFlag[0] <= 1'b0;
                  end
               end
            else
            ALUOutFlag[0] <= 1'b0;
               end
            4'b0110: // Rotate left
            begin
               ALU_Result <= A - B;
               if(ALU_Result == 8'b0)
               begin
                  ALUOutFlag[3] <= 1'b1;
               end
            if(ALU_Result[7] == 1'b1)
               begin
                  ALUOutFlag[2] <= 1'b1;
               end
            else
            begin
               ALUOutFlag[2] <= 1'b0;
            end
            if(ALUOutFlag[1] == 1'b1)
               begin
                     ALUOutFlag[1] <= 1'b1;
               end
            else
                  begin 
                     ALUOutFlag[1] <= 1'b0;
                  end
                  
               if( A > B)
               begin
                  if( ALU_Result[7] == 1'b1)
                  begin
                        ALUOutFlag[0] <= 1'b1;
                  end
                  else
                  begin
                        ALUOutFlag[0] <= 1'b0;
                  end
               end
               else
               begin
                  if( ALU_Result[7] == 1'b0)
                  begin
                        ALUOutFlag[0] <= 1'b1;
                  end
                  else
                  begin
                        ALUOutFlag[0] <= 1'b0;
                  end
               end
               end
            4'b0111: // Rotate right
            begin // Addition
               ALU_Result <= A & B;
               if(ALU_Result == 8'b0)
               begin
               ALUOutFlag[3] <= 1'b1;
               end
               if(ALU_Result[7] == 1'b1)
               begin
               ALUOutFlag[2] <= 1'b1;
               end
               else
               begin
               ALUOutFlag[2] <= 1'b0;
            end
            end
            4'b1000: //  Logical and 
            begin // Addition
            ALU_Result <= A | B;
            if(ALU_Result == 8'b0)
            begin
            ALUOutFlag[3] <= 1'b1;
            end
            if(ALU_Result[7] == 1'b1)
            begin
            ALUOutFlag[2] <= 1'b1;
            end
            else
            begin
               ALUOutFlag[2] <= 1'b0;
            end
               end
            4'b1001: //  Logical or
            begin // Addition
               ALU_Result <= A ^ B;
               if(ALU_Result == 8'b0)
               begin
               ALUOutFlag[3] <= 1'b1;
               end
               if(ALU_Result[7] == 1'b1)
               begin
               ALUOutFlag[2] <= 1'b1;
               end
               else
               begin
                  ALUOutFlag[2] <= 1'b0;
               end
               end
            4'b1010: //  Logical xor 
            begin
               CarryOutShl <= A[7];
            ALU_Result <= A << 1;
               if(ALU_Result == 8'b0)
            begin
               ALUOutFlag[3] <= 1'b1;
            end
            if(ALU_Result[7] == 1'b1)
            begin
               ALUOutFlag[2] <= 1'b1;
               end
            else
               begin
               ALUOutFlag[2] <= 1'b0;
            end
               if( CarryOutShl == 1'b1)
               begin
                  ALUOutFlag[1] <= 1'b1;
               end
               else
               begin
                  ALUOutFlag[1] <= 1'b0;
               end
            end
            4'b1011: //  Logical nor
            begin
               CarryOutShr <= A[0];
               ALU_Result <= A >> 1;
               if(ALU_Result == 8'b0)
               begin
               ALUOutFlag[3] <= 1'b1;
               end
               if(ALU_Result[7] == 1'b1)
               begin
               ALUOutFlag[2] <= 1'b1;
               end
               else
               begin
                  ALUOutFlag[2] <= 1'b0;
               end
               if( CarryOutShr == 1'b1)
               begin
                     ALUOutFlag[1] <= 1'b1;
               end
               else
               begin
                     ALUOutFlag[1] <= 1'b0;
               end
               end
            4'b1100: // Logical nand 
            begin
               ALU_Result <= A <<< 1;
               if(ALU_Result == 8'b0)
               begin
               ALUOutFlag[3] <= 1'b1;
               end
               if(ALU_Result[7] == 1'b1)
               begin
               ALUOutFlag[2] <= 1'b1;
               end
               else
               begin
                  ALUOutFlag[2] <= 1'b0;
               end
               if( ALU_Result[7] == A[7])
               begin
                  ALUOutFlag[0] <= 1'b0;
               end
               else
               begin
                  ALUOutFlag[0] <= 1'b1;
               end
               end
            4'b1101: // Logical xnor
               begin
               ALU_Result <= A >>> 1;
               if(ALU_Result == 8'b0)
                  begin
                  ALUOutFlag[3] <= 1'b1;
                  end
               end
            4'b1110: // Greater comparison
            begin
                  ALU_Result <= { A[6:0], ALUOutFlag[1]};
                  ALUOutFlag[1] <= A[7];
                  if(ALU_Result == 8'b0)
                     begin
                  ALUOutFlag[3] <= 1'b1;
                     end
               if(ALU_Result[7] == 1'b1)
                     begin
                     ALUOutFlag[2] <= 1'b1;
                     end
                  else
                  begin
                  ALUOutFlag[2] <= 1'b0;
               end
                  if( ALUOutFlag[1] == 1'b1)
                  begin
                        ALUOutFlag[1] <= 1'b1;
                  end
                  else
                  begin
                        ALUOutFlag[1] <= 1'b0;
                  end
                  if( A[7] == A[6])
                  begin
                     ALUOutFlag[0] <= 1'b0;
                  end
                  else
                  begin
                        ALUOutFlag[0] <= 1'b1;
                  end
            end
            4'b1111: // Equal comparison   
               begin
                  ALU_Result <= { ALUOutFlag[1], A[7:1] };
                           
                  if(ALU_Result == 8'b0)
                     begin
                     ALUOutFlag[3] <= 1'b1;
                     end
                  if(ALU_Result[7] == 1'b1)
                     begin
                     ALUOutFlag[2] <= 1'b1;
                        end
                     else
                  begin
                     ALUOutFlag[2] <= 1'b0;
                  end
                  
                  if( ALUOutFlag[1] == 1'b1)
                  begin
                        ALUOutFlag[1] <= 1'b1;
                  end
                  else
                  
                  begin
                        ALUOutFlag[1] <= 1'b0;
                  end
                  
                  if( ALUOutFlag[1] == A[7])
                  begin
                        ALUOutFlag[0] <= 1'b0;
                  end
                  else
                  begin
                        ALUOutFlag[0] <= 1'b1;
                  end
                     ALUOutFlag[1] <= A[0]; 
               end 
         endcase 
      end
         assign ALU_Out = ALU_Result; 
   endmodule

module ALUSystem(Clock,MuxASel,MuxBSel,MuxCSel,ALU_FunSel,RF_FunSel,ARF_FunSel,IR_Funsel,RF_OutASel,RF_OutBSel,ARF_OutCSel,ARF_OutDSel,RF_RegSel,ARF_RegSel,IR_LH,IR_Enable,Mem_WR,Mem_CS);
input Clock;
input [1:0]MuxASel;
input [1:0]MuxBSel;
input MuxCSel;
input [3:0]ALU_FunSel;
input [1:0]RF_FunSel;
input [1:0]ARF_FunSel;
input [1:0]IR_Funsel;
input [1:0]RF_OutASel;
input [1:0]RF_OutBSel;
input [1:0]ARF_OutCSel;
input [1:0]ARF_OutDSel;
input [3:0]RF_RegSel;
input [2:0]ARF_RegSel;
input IR_LH;
input IR_Enable;
input Mem_WR;
input Mem_CS;

wire [3:0]ALUOutFlag;
/*reg zeroFlag;
reg negativeFlag;
reg overflowFlag;
reg carryFlag;*/

wire [7:0]MuxAOut;
wire [7:0]AOut;
wire [7:0]BOut;
part2a RF(MuxAOut,RF_FunSel,RF_RegSel,RF_OutASel,RF_OutBSel,Clock,AOut,BOut);
wire [7:0]MuxBOut;
wire [7:0]ARF_COut;
wire [7:0]Address;
part2b ARF(MuxBOut,ARF_FunSel,ARF_RegSel,ARF_OutCSel,ARF_OutDSel,Clock,ARF_COut,Address);
wire [7:0]MemoryOut;
wire [15:0]IROut;
part2c IR(MemoryOut,IR_LH,IR_Funsel,Clock,IR_Enable,IROut);
wire [7:0]ALUOut;
Memory mem(Address,ALUOut,Mem_WR,Mem_CS,Clock,MemoryOut);
wire [7:0]MuxCOut;
//alu ALU(MuxCOut,BOut,ALU_FunSel,ALUOut,zeroFlag,negativeFlag,overflowFlag,carryFlag);
alu ALU(MuxCOut,BOut,ALU_FunSel,ALUOut,ALUOutFlag);
eight_bit_multiplexer_4to1 muxA(MuxAOut,ALUOut,ARF_COut,MemoryOut,IROut,MuxASel);
multiplexer_4to1_8bit muxB(MuxBOut,ALUOut,MemoryOut,IROut,MuxBSel);
multiplexer_2to1_8bit muxC(MuxCOut,ARF_COut,AOut,MuxCSel);
endmodule

