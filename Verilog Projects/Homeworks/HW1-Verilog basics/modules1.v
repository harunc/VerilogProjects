// Students: HELIN ASLI AKSOY & HARUN CITFCI 
// Create Date: 08.04.2022 23:06:27
// Module Name: design
// Project Name: Homework 1



//PART 1 Starts//
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

//PART 1 Finish//

//PART 1 ADDITION MODULS START//
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
//PART 1 ADDITION MODULS Finish//

//PART 2 //
module part2(
    input a,
    input b,
    input c,
    input d,
    output o
    );
    
    wire x;
    wire y;
    wire z;
    and_gate bd(~b,~d,x);
    and_three_gate abc(~a,b,c,y);
    and_three_gate acd(a,c,d,z);
    or_gate_3input out(x,y,z,o);
endmodule

//PART 3//
module part3(
    input a,
    input b,
    input c,
    input d,
    output o
    );
    
    wire bd1,bd2,bd3,abc1,abc2,abco1,acd1,acd2,acdo1,abcacdo1,abcacdo2;
    nand_gate bd_1(~b,~d,bd1);
    nand_three_gate abc_1(~a,b,c,abc1);
    nand_three_gate acd_1(a,c,d,acd1);
    nand_gate abcacd_1(abc1,acd1,abcacdo1);
    nand_gate abcacd_2(abcacdo1,abcacdo1,abcacdo2);
    nand_gate out(bd1,abcacdo2,o);
endmodule

//PART 4//
module part4(
    input d,
    input a,
    input b,
    input c,
    output o
    );

    wire d_not;
    wire or_1;
    wire or_2;
    
    not_gate notd(d,d_not);
    or_gate firstor(d_not, d, or_1);
    or_gate secor(d_not, d, or_2);
    
    multiplexer_8to1 multiplexer(o,d_not,d_not,0,or_1,d_not,or_2,0,d,c,b,a);    
endmodule


//PART 5 
module part5(
    input a,b,c,
    output o_F1,
    output o_F2
    );
    wire x1,y1;
    wire x2,y2;
    
    decoder_3to8 decoder_F1(a,b,c,0,0,0,x1,0,y1,0,0);
    or_gate orGateF1(x1,y1,o_F1);
    
    decoder_3to8 decoder_F2(a,b,c,0,0,0,0,0,0,x2,y2);
    or_gate orGateF2(x2,y2,o_F2);
endmodule


//PART 6//
module half_adder(
    input A,
    input B,
    output C,
    output S
    );
    
    xor_gate first_xor(A,B,S);
    and_gate first_and(A,B,C);
endmodule

//PART 7//
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

//PART 8//
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

//PART 9//
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

//part10 çalýþmýyor//
module sixteen_bit_full_adder_subtractor(
    input [15:0] A,
    input [15:0] B_input,
    input X, ////1 when subraction, 0 when summation
    output Cout,
    output [15:0] s
    );
    wire co1;
    //sadece adder
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
    
    eight_bit_full_adder digit0 (A[7:0], B_after_Xor[7:0], X, co1,s[7:0]);
    eight_bit_full_adder digit1 (A[15:8], B_after_Xor[15:8], co1, Cout,s[15:8]);
    //sadece Adder 
endmodule

//Addition part for 11//
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
//Addition for part 11//

//PART 11//
module part11(
    input [15:0] A,
    input [15:0] B,
    output Cout,
    output [15:0] S
    );
    wire co;

    wire [15:0] two_times_A;
    
    A_sixteen_bit_full_adder first_16_adder(A,A,0,co,two_times_A);

    A_sixteen_bit_full_adder_subtractor first(B,two_times_A, 1, Cout, S);
endmodule
