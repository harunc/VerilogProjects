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


module sr_latch_withouten_simulation(); 
   reg S;
   reg R;
   wire Q;
   wire Q_neg;
   sr_latch_withouten uut(S,R,Q,Q_neg);
   
   initial begin
       S = 1'b1; R = 1'b0;#200;
       S = 1'b0; R = 1'b0;#200;
       S = 1'b0; R = 1'b1;#200;
       S = 1'b0; R = 1'b0;#200;
       S = 1'b1; R = 1'b1;#200;
   end
endmodule

module sr_latch_withen_simulation(); 
   reg S;
   reg R;
   reg E;
   wire Q;
   wire Q_neg;
   sr_latch_withen uut(S,R,E,Q,Q_neg);
   
   initial begin
       S = 1'b1; R = 1'b0;E = 1'b1;#111.11;
       S = 1'b0; R = 1'b0;E = 1'b1;#111.11;
       S = 1'b0; R = 1'b1;E = 1'b1;#111.11;
       S = 1'b0; R = 1'b0;E = 1'b1;#111.11;
       S = 1'b0; R = 1'b0;E = 1'b0;#111.11;
       S = 1'b0; R = 1'b1;E = 1'b0;#111.11;
       S = 1'b1; R = 1'b0;E = 1'b0;#111.11;
       S = 1'b1; R = 1'b1;E = 1'b0;#111.11;
       S = 1'b1; R = 1'b1;E = 1'b1;#111.11;
   end
endmodule

module d_flip_flop_simulation(); 
   reg D;
   reg CLK;
   wire Q;
   wire Q_neg;
   d_flip_flop uut(D,CLK,Q,Q_neg);
   
   initial begin
       CLK = 0; D = 1;#166.66;
       D = 0;#166.66;
       D = 1;#166.66;
       D = 0;#166.66;
       D = 0;#166.66;
       D = 1;#166.66;
       D = 1;#166.66;
   end
   always
   begin
       CLK <= ~CLK; #50;
   end
endmodule

/*module jk_flip_flop_simulation(); 
   reg J;
   reg K;
   reg clock;
   wire Q;
   wire Q_neg;
   jk_flip_flop uut(J,K,clock,Q,Q_neg);

    initial begin
       clock = 0; J = 0; K = 0; #250;
       J = 0; K = 1; #250;
       J = 1; K = 0; #250;
       J = 1; K = 1; #250;
   end
   
   always
   begin
       clock <= ~clock; #30;
   end
endmodule*/

module jk_flip_flop_simulation(); 
   reg J;
   reg K;
   reg clock;
   wire Q;
   wire Q_neg;
   jk_flip_flop1 uut(J,K,clock,Q,Q_neg);

    initial begin
       clock = 0;J = 0; K = 1; #250;
       J = 0; K = 0; #250;
       J = 1; K = 1; #250;
       J = 1; K = 0; #250;
   end
   
   always
   begin
       clock = ~clock; #30;
   end
endmodule

/*
module asynchronous_up_counter_sim(); 
   reg J;
   reg K;
   reg CLK;
   wire A;
   wire B;
   wire C;
   wire D;
   asynchronous_up_counter uut(.J(J),.K(K),.CLK(clock),.A(A),.B(B),.C(C),.D(D));
   
   initial begin
       CLK = 0; J = 1;K=1;#1000;
   end
   always
   begin
       CLK = ~CLK; #50;
   end
endmodule
*/

module asynchronous_up_counter_sim();
    reg j;
    reg k;
    reg clock;
    wire A,B,C,D,A_neg,B_neg,C_neg,D_neg;
      
    asynchronous_up_counter uut(j,k,clock,A,B,C,D,A_neg,B_neg,C_neg,D_neg);
    initial begin
        clock = 0; j = 1; k=1; #200;
    end
    
     always
   begin
       clock = ~clock; #50;
   end
  

endmodule

module circular_shift_register_simulation(); 
   reg [7:0] A;
   reg CLK;
   reg load_flag;
   wire output_bit;
   circular_shift_register uut(A,CLK,load_flag,output_bit);

    initial begin
       CLK = 0; A = 8'b11000000; load_flag = 1; #125;
       A = 8'd1; load_flag = 0; #125;
       A = 8'd2; load_flag = 0; #125;
       A = 8'd3; load_flag = 0; #125;
       A = 8'd4; load_flag = 0; #125;
       A = 8'd5; load_flag = 0; #125;
       A = 8'd6; load_flag = 0; #125;
       A = 8'd7; load_flag = 1; #125;
   end

   always begin
       CLK <= ~CLK; #13.8888888889;
       CLK <= ~CLK; #111.111111111;
   end
endmodule


