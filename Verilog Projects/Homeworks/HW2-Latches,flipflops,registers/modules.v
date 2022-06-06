`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Students Name:Helin Aslı Aksoy & Harun Çifci 
// Student Numbers: 150200705 & 150180089
// Create Date: 24.04.2022 14:39:32
// Design Name: 
// Module Name: homework2_modules
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

module nor_gate(
    input A,
    input B,
    output C
    );
    assign C = ~(A | B);
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

module or_gate_3input(
    input A,
    input B,
    input C,
    output D
    );
    assign D = A | B | C;
endmodule


module sr_latch_withouten(
    input S,
    input R,
    output Q,
    output Q_neg
    );
    
    nor_gate nor1(S, Q, Q_neg);
    nor_gate nor2 (R, Q_neg, Q);
endmodule

module sr_latch_withen(
    input S,
    input R,
    input E,
    output Q,
    output Q_neg
    );
    wire Sn;
    wire Rn;
    nand_gate nan1(S, E, Sn);
    nand_gate nand2 (R, E, Rn);
    
    nand_gate nand3 (Sn, Q_neg, Q);
    nand_gate nand4 (Rn, Q, Q_neg);
endmodule

module d_latch(
    input D,
    input E,
    output Q,
    output Q_NOT
    );
    wire not_D;
    wire nand1;
    wire nand2;
    
    nand_gate not_of_D (D, D, not_D);
    nand_gate nand_1 (D, E, nand1);
    nand_gate nand_2 (not_D, E, nand2);
    nand_gate nand_4 (nand2, Q, Q_NOT);
    nand_gate nand_3 (nand1, Q_NOT, Q);
endmodule

module d_flip_flop(
    input D,
    input  CLK,
    output Q,
    output Q_NOT
    );
    wire not_CLK;
    wire middle_Q;
    wire middle_Q_NOT;

    not_gate notCLK(CLK,not_CLK);

    d_latch d_latch_1(D,not_CLK, middle_Q, middle_Q_NOT);
    d_latch d_latch_2(middle_Q, CLK, Q, Q_NOT);
endmodule

module jk_flip_flop(
    input J,
    input K,
    input clock,
    output Q,
    output Q_neg
    );
    wire wire_J;
    wire wire_K;
    
    nand_three_gate nand_j(J,clock,Q_neg,wire_J);
    nand_three_gate nand_k(K,clock,Q,wire_K);
    sr_latch_withouten sr(wire_K, wire_J, Q, Q_neg);
    
endmodule

module jk_flip_flop1(J,K,clock,Q,Q_neg);
input wire J;
input wire K;
input wire clock;
output reg Q;
output reg Q_neg;
 always@(posedge clock) begin
        Q_neg <= ~Q;
        case ({J,K})
        2'b00: Q <= Q;
        2'b01: Q <= 0;
        2'b10: Q <= 1;
        2'b11: Q <= Q_neg;
        endcase
   end
endmodule

//zdkjfsfjskdfsdfsdfs
module jkff(j,k,clock,reset,q,qb);
input j,k,clock,reset;
output reg q,qb;
always@(posedge clock)
begin
case({reset,j,k})
3'b100 :q=q;
3'b101 :q=0;
3'b110 :q=1;
3'b111 :q=1;
default :q=0;
endcase
qb<=~q;
end
endmodule

module ripple_count(j,k,clock,reset,q,qb);
input j,k,clock,reset;
output wire [3:0]q,qb;

jkff JK1(j,k,clock,reset,q[0],qb[0]);
jkff JK2(j,k,q[0],reset,q[1],qb[1]);
jkff JK3(j,k,q[1],reset,q[2],qb[2]);
jkff JK4(j,k,q[2],reset,q[3],qb[3]);

endmodule
//kskdfskjdfksdfsdfsd


module asynchronous_up_counter(J,K,clock,A,B,C,D,A_neg,B_neg,C_neg,D_neg);
    input J;
    input K;
    input clock;
    output wire A;
    output wire B;
    output wire C;
    output wire D;
    
    output wire A_neg;
    output wire B_neg;
    output wire C_neg;
    output wire D_neg;
    
    jk_flip_flop1 A_ff(J,K,clock,A,A_neg);
    jk_flip_flop1 B_ff(J,K,A,B,B_neg);
    jk_flip_flop1 C_ff(J,K,B,C,C_neg);
    jk_flip_flop1 D_ff(J,K,C,D,D_neg);
endmodule


module synchronous_up_counter(
    input J,
    input K,
    input clock,
    output A,
    output B,
    output C,
    output D
    );
    
    wire AB;
    wire ABC;
    wire wire_A;
    wire wire_B;
    wire wire_C;
     
    jk_flip_flop A_ff(J,K,clock,wire_A);
    jk_flip_flop B_ff(wire_A,wire_A,clock,wire_B);
    and_gate and_AB(wire_A,wire_B,AB);
    jk_flip_flop C_ff(AB,AB,clock,wire_C);
    and_three_gate and_ABC(wire_A,wire_B,wire_C,ABC);
    jk_flip_flop D_ff(ABC,ABC,clock,D);
    
    assign A = wire_A;
    assign B = wire_B;
    assign C = wire_C; 
endmodule

module circular_shift_register(
    input [7:0] A,
    input wire CLK,
    input wire load_flag,
    output reg output_bit
    );
    reg D0;
    reg D1;
    reg D2;
    reg D3;
    reg D4;
    reg D5;
    reg D6;
    reg D7;
    reg temp;

    always @ (posedge CLK)
    begin

        if (load_flag == 1)
        begin
            output_bit <= A[7];

            D0 <= A[0];
            D1 <= A[1];
            D2 <= A[2];
            D3 <= A[3];
            D4 <= A[4];
            D5 <= A[5];
            D6 <= A[6];
            D7 <= A[7];
        end
        else if (load_flag == 0)
        begin
            output_bit <= D6;

            temp <= D7;
            D7 <= D6;
            D6 <= D5;
            D5 <= D4;
            D4 <= D3;
            D3 <= D2;
            D2 <= D1;
            D1 <= temp;

        end
    end
endmodule


