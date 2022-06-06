`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITU BBF
// Engineer: Kadir Ozlem
// 
// Create Date: 26/04/2022 10:50:21 PM
// Design Name: Project1Test Module
// Module Name: Test Bench
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

//Please put it in your simulations.v 

module Project1Test();
    //Input Registers of ALUSystem
    reg[1:0] RF_OutASel; 
    reg[1:0] RF_OutBSel; 
    reg[1:0] RF_FunSel;
    reg[3:0] RF_RegSel;
    reg[3:0] ALU_FunSel;
    reg[1:0] ARF_OutCSel; 
    reg[1:0] ARF_OutDSel; 
    reg[1:0] ARF_FunSel;
    reg[2:0] ARF_RegSel;
    reg      IR_LH;
    reg      IR_Enable;
    reg[1:0] IR_Funsel;
    reg      Mem_WR;
    reg      Mem_CS;
    reg[1:0] MuxASel;
    reg[1:0] MuxBSel;
    reg      MuxCSel;
    reg      Clock;
    
    //Test Bench Connection of ALU System
    ALUSystem _ALUSystem(
    .RF_OutASel(RF_OutASel), 
    .RF_OutBSel(RF_OutBSel), 
    .RF_FunSel(RF_FunSel),
    .RF_RegSel(RF_RegSel),
    .ALU_FunSel(ALU_FunSel),
    .ARF_OutCSel(ARF_OutCSel), 
    .ARF_OutDSel(ARF_OutDSel), 
    .ARF_FunSel(ARF_FunSel),
    .ARF_RegSel(ARF_RegSel),
    .IR_LH(IR_LH),
    .IR_Enable(IR_Enable),
    .IR_Funsel(IR_Funsel),
    .Mem_WR(Mem_WR),
    .Mem_CS(Mem_CS),
    .MuxASel(MuxASel),
    .MuxBSel(MuxBSel),
    .MuxCSel(MuxCSel),
    .Clock(Clock)
    );
    
    //Test Vector Variables
    reg [31:0] VectorNum, Errors, TotalLine; 
    reg [34:0] TestVectors[10000:0];
    reg Reset, Operation;
    
    //Clock Signal Generation
    always 
    begin
        Clock = 1; #5; Clock = 0; #5; // 10ns period
    end
    
    //Read Test Bench Values
    initial begin
        $readmemb("TestBench.mem", TestVectors); // Read vectors
        VectorNum = 0; Errors = 0; TotalLine=0; Reset=0;// Initialize
    end
    
    // Apply test vectors on rising edge of clock
    always @(posedge Clock)
    begin
        #1; 
        {Operation, RF_OutASel, RF_OutBSel, RF_FunSel, 
        RF_RegSel, ALU_FunSel, ARF_OutCSel, ARF_OutDSel, 
        ARF_FunSel, ARF_RegSel, IR_LH, IR_Enable, IR_Funsel, 
        Mem_WR, Mem_CS, MuxASel, MuxBSel, MuxCSel} = TestVectors[VectorNum];
    end
    
    // Check results on falling edge of clk
    always @(negedge Clock)
        if (~Reset) // skip during reset
        begin
            $display("Input Values:");
            $display("Operation: %d", Operation);
            $display("Register File: OutASel: %d, OutBSel: %d, FunSel: %d, Regsel: %d", RF_OutASel, RF_OutBSel, RF_FunSel, RF_RegSel);            
            $display("ALU FunSel: %d", ALU_FunSel);
            $display("Addres Register File: OutCSel: %d, OutDSel: %d, FunSel: %d, Regsel: %d", ARF_OutCSel, ARF_OutDSel, ARF_FunSel, ARF_RegSel);            
            $display("Instruction Register: LH: %d, Enable: %d, FunSel: %d", IR_LH, IR_Enable, IR_Funsel);            
            $display("Memory: WR: %d, CS: %d", Mem_WR, Mem_CS);
            $display("MuxASel: %d, MuxBSel: %d, MuxCSel: %d", MuxASel, MuxBSel, MuxCSel);
            
            $display("");
            $display("Ouput Values:");
            $display("Register File: AOut: %d, BOut: %d", _ALUSystem.AOut, _ALUSystem.BOut);            
            $display("ALUOut: %d, ALUOutFlag: %d, ALUOutFlags: Z:%d, C:%d, N:%d, O:%d,", _ALUSystem.ALUOut, _ALUSystem.ALUOutFlag, _ALUSystem.ALUOutFlag[3],_ALUSystem.ALUOutFlag[2],_ALUSystem.ALUOutFlag[1],_ALUSystem.ALUOutFlag[0]);
            $display("Address Register File: COut: %d, DOut (Address): %d", _ALUSystem.ARF_COut, _ALUSystem.Address);            
            $display("Memory Out: %d", _ALUSystem.MemoryOut);            
            $display("Instruction Register: IROut: %d", _ALUSystem.IROut);            
            $display("MuxAOut: %d, MuxBOut: %d, MuxCOut: %d", _ALUSystem.MuxAOut, _ALUSystem.MuxBOut, _ALUSystem.MuxCOut);
            
            // increment array index and read next testvector
            VectorNum = VectorNum + 1;
            if (TestVectors[VectorNum] === 35'bx)
            begin
                $display("%d tests completed.",
                VectorNum);
                $finish; // End simulation
            end
        end
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

module A_eight_bit_full_adder_subtractor_test(); 
   reg [7:0]A;
   reg [7:0]B;
   reg Cin;
   wire Cout;
   wire [7:0]S;
   A_eight_bit_full_adder_subtractor uut(A,B,Cin,Cout,S);
   
   initial begin
       A = 8'd29; B = 8'd5; Cin = 1'd1;#125;
       A = 8'd100; B = 8'd92; Cin = 1'd1;#125;
       A = 8'd30; B = 8'd28; Cin = 1'd1;#125;
       A = 8'd191; B = 8'd2; Cin = 1'd1;#125;
       A = 8'd200; B = 8'd95; Cin = 1'd0;#125;
       A = 8'd49; B = 8'd25; Cin = 1'd0;#125;
       A = 8'd78; B = 8'd252; Cin = 1'd0;#125;
       A = 8'd43; B = 8'd59; Cin = 1'd0;#125;
   end
endmodule

module A_sixteen_bit_full_adder_subtractor_test(); 
   reg [15:0]A;
   reg [15:0]B_input;
   reg X;           //1 when subraction, 0 when summation
   wire Cout;
   wire [15:0]s;
   
   A_sixteen_bit_full_adder_subtractor uut(A,B_input,X,Cout,s);
   
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

module eight_bit_d_flip_flop_simulation(); 
   reg [7:0] D;
   reg reset;
   reg enable;
   reg CLK;
   wire [7:0] Q;
   eight_bit_d_flip_flop uut(D,reset, enable, CLK,Q);
   
   initial begin
       CLK = 0; enable=1; reset=0; D = 8'b01010101;#166.66;
       D = 8'b01010101; #166.66;
       D = 8'b11111111; #166.66;
       reset = 1;D = 8'b11110000; #166.66;
       enable = 0; D = 8'b01010101; #166.66;

   end
   always
   begin
       CLK <= ~CLK; #50;
   end
endmodule

module sixteen_bit_d_flip_flop_simulation(); 
   reg [15:0] D;
   reg reset;
   reg enable;
   reg CLK;
   wire [15:0] Q;
   sixteen_bit_d_flip_flop uut(D,reset, enable, CLK,Q);
   
   initial begin
       CLK = 0; enable=1; reset=0; D = 16'd98 ;#166.66;
       D = 16'd17; #166.66;
       D = 16'd34; #166.66;
       reset = 1; D =16'd165; #166.66;
       enable = 0; D = 16'd134; #166.66;

   end
   always
   begin
       CLK <= ~CLK; #50;
   end
endmodule

module eight_bit_register_test();
   reg [7:0] A;
   reg reset;
   reg enable;
   reg CLK;
   reg [1:0]funsel;
   wire [7:0] O;
   eight_bit_register uut(A,funsel,enable,CLK,reset,O);
   
   initial begin
       CLK = 0; enable=1; reset=0; A = 8'b01010101; funsel=2'b10;#166.66;//load
       funsel=2'b00; #166.66;//decrement
       funsel=2'b01; #166.66;//increment
       funsel=2'b11; #166.66;//Clear
      A = 8'b11110000; funsel=2'b10; #166.66;//load
       enable = 0; funsel=2'b11; #166.66;

   end
   always
   begin
       CLK <= ~CLK; #50;
   end    
    
endmodule

module sixteen_bit_register_test();
   reg [15:0] A;
   reg reset;
   reg enable;
   reg CLK;
   reg [1:0]funsel;
   wire [15:0] O;
   sixteen_bit_register uut(A,funsel,enable,CLK,reset,O);
   
   initial begin
       CLK = 0; enable=1; reset=0; A = 16'b0101010101010101; funsel=2'b10;#166.66;//load
       funsel=2'b00; #166.66;//decrement
       funsel=2'b01; #166.66;//increment
       funsel=2'b11; #166.66;//Clear
      A = 16'b1111000011110000; funsel=2'b10; #166.66;//load
       enable = 0; funsel=2'b11; #166.66;

   end
   always
   begin
       CLK <= ~CLK; #50;
   end    
    
endmodule

module part2a_test();
   reg [7:0] A;
   reg [1:0]funsel;
   reg [3:0]regsel;
   reg [1:0]outAsel;
   reg [1:0]outBsel;
   reg CLK;
   wire [7:0] OA;
   wire [7:0] OB;
   part2a uut(A,funsel,regsel,outAsel,outBsel,CLK,OA,OB);
   
   initial begin
       CLK = 0; regsel=4'b1110; A = 8'b01010101; funsel=2'b10; outAsel=2'b11 ; outBsel=2'b00;#166.66;//r1 load -> A,B
       funsel=2'b00;regsel=4'b1110;outAsel=2'b00 ; outBsel=2'b00; #166.66;//decrement r1->A,B
       funsel=2'b11; regsel=4'b1110; #166.66; outAsel=2'b00 ; outBsel=2'b00;//clear r1->A,B
       funsel=2'b10;regsel=4'b1001;A = 8'b11001100;outAsel=2'b01 ; outBsel=2'b10; #166.66;//load r2,r3->A,B
       funsel=2'b01;regsel=4'b1001;outAsel=2'b10 ; outBsel=2'b01; #166.66;//increment r1,r2->B,A
       funsel=2'b11;regsel=4'b0000;outAsel=2'b11 ; outBsel=2'b10; #166.66;//clear r1,r2,r3,r4->_,_,B,A

   end
   always
   begin
       CLK <= ~CLK; #50;
   end    
    
endmodule
module part2b_test();
   reg [7:0] A;
   reg [1:0]funsel;
   reg [2:0]regsel;
   reg [1:0]outCsel;
   reg [1:0]outDsel;
   reg CLK;
   wire [7:0] OC;
   wire [7:0] OD;
   part2b uut(A,funsel,regsel,outCsel,outDsel,CLK,OC,OD);
   
   initial begin
       CLK = 0; regsel=3'b011; A = 8'b01010101; funsel=2'b10; outCsel=2'b00 ; outDsel=2'b01;#166.66;// LOAD PC // OUTPUT PC
       funsel=2'b00;regsel=3'b011;outCsel=2'b00 ; outDsel=2'b01; #166.66;// DECREMENT PC // OUTPUT PC
       funsel=2'b11; regsel=3'b011; outCsel=2'b00 ; outDsel=2'b10;#166.66; // CLEAR PC // OUTPUT PC AND AR
       funsel=2'b10;regsel=3'b100;A = 8'b11001100;outCsel=2'b00 ; outDsel=2'b11; #166.66; // LOAD AR AND SP // OUTPUT PC AND SP
       funsel=2'b01;regsel=3'b100;outCsel=2'b10 ; outDsel=2'b11; #166.66;// INCREMENT AR AND SP // OUTPUT AR AND SP
       funsel=2'b11;regsel=3'b000;outCsel=2'b10 ; outDsel=2'b11; #166.66;// CLEAR ALL

   end
   always
   begin
       CLK <= ~CLK; #50;
   end    
    
endmodule



module part2c_test();
   reg [7:0] A;
   reg LH;
   reg enable;
   reg CLK;
   reg [1:0]funsel;
   wire [15:0] O;
   part2c uut(A,LH,funsel,CLK,enable,O);
   
   initial begin
       enable=1;LH=1'b1; A = 8'b00000001; funsel=2'b10;#5;CLK = 0;#166.66;//load
       LH=1'b0; enable=1; A = 8'b00000001; funsel=2'b10;#166.66;//load
       LH=1'b0; enable=1; A = 8'b11111111; funsel=2'b10;#166.66;//load
       funsel=2'b00; #166.66;//A
       funsel=2'b01; #166.66;//Clear
       funsel=2'b11; #166.66;//Clear

   end
   always
   begin
       CLK <= ~CLK; #50;
   end    
    
endmodule
/*
input [7:0] A,B,  // ALU 8-bit Inputs                 
       input [3:0] ALU_Sel,// ALU Selection
       output [7:0] ALU_Out, // ALU 8-bit Output
       output reg zeroFlag,
       output reg negativeFlag,
       output reg overflowFlag,
       output reg carryFlag*/
module ALU_test();
//Inputs
 reg[7:0] A,B;
 reg[3:0] ALU_Sel;

//Outputs
 wire [3:0]ALUOutFlag;
 wire[7:0] ALU_Out;
 /*wire zeroFlag;
 wire negativeFlag;
 wire overflowFlag;
 wire carryFlag;*/
 // Verilog code for ALU
 integer i;
 alu uut(
            A,B,  // ALU 8-bit Inputs                 
            ALU_Sel,// ALU Selection
            ALU_Out,
            ALUOutFlag // ALU 8-bit Output
            /*zeroFlag,
            negativeFlag,
            overflowFlag,
            carryFlag*/ // Carry Out Flag
     );
    initial begin
    // hold reset state for 100 ns.
      A = 8'b10001010;
      B = 8'b10000011;
      ALU_Sel = 4'h0;
      #50;
      for (i=0;i<=15;i=i+1)
      begin
       ALU_Sel = ALU_Sel + 4'b0001;
       #50;
      end
      
      A = 8'hF6;
      B = 8'h0A;
      
    end
endmodule