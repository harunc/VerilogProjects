`timescale 1ns / 1ps

module CharEncoder_simulation();
   reg [25:0] decodedChar;
   wire [7:0] char;
   
   CharEncoder uut(decodedChar, char);

   initial 
   begin
        decodedChar = 26'b10000000000000000000000000; #142.85;
        decodedChar = 26'b00000000000000000000000010; #142.85;
        decodedChar = 26'b00000000000000000000000100; #142.85;
        decodedChar = 26'b00000000000000000000001000; #142.85;
        decodedChar = 26'b00000000000000000000010000; #142.85;
        decodedChar = 26'b00000000000000000000100000; #142.85;
        decodedChar = 26'b00000000000000000001000000; #142.85;
   end

endmodule

module CharDecoder_simulation(); 
   reg [7:0] char;
   wire [25:0] decodedChar;

   CharDecoder uut(char, decodedChar);

   initial 
   begin
       char = 8'd90; #142.85;
       char = 8'd66; #142.85;
       char = 8'd67; #142.85;
       char = 8'd68; #142.85;
       char = 8'd69; #142.85;
       char = 8'd70; #142.85;
       char = 8'd71; #142.85;
   end

endmodule

module CircularLeftShiftSimulation(); 
   reg [25:0]data;
   reg [4:0]shiftAmount;
   wire [25:0]out;
   CircularLeftShift uut(shiftAmount, data, out);

   initial begin
        data = 26'b10000000000000000000000000; shiftAmount = 5'b00001; #250;
        data = 26'b01000000000000000000000000; shiftAmount = 5'b00001; #250;
        data = 26'b00100000000000000000000000; shiftAmount = 5'b00001; #250;
        data = 26'b00010000000000000000000000; shiftAmount = 5'b00001; #250;
   end
endmodule

module CircularRightShiftSimulation(); 
   reg [25:0]data;
   reg [4:0]shiftAmount;
   wire [25:0]out;
   CircularRightShift uut(shiftAmount,data, out);

   initial begin
        data = 26'b10000000000000000000000000; shiftAmount = 5'b00001; #250;
        data = 26'b01000000000000000000000000; shiftAmount = 5'b00001; #250;
        data = 26'b00100000000000000000000000; shiftAmount = 5'b00001; #250;
        data = 26'b00010000000000000000000000; shiftAmount = 5'b00001; #250;
   end
endmodule

module CaesarCipherEncryption_simulation();
    reg [7:0] plainChar;
    reg [4:0] shiftCount;
    wire [7:0] chipherChar;
   
   CaesarCipherEncryption uut(plainChar, shiftCount, chipherChar);

   initial 
   begin
        plainChar = 8'd72; shiftCount = 0; #200;
        plainChar = 8'd65; shiftCount = 4; #200;        
        plainChar = 8'd82; shiftCount = 20; #200;
        plainChar = 8'd85; shiftCount = 14; #200;
        plainChar = 8'd78; shiftCount = 0; #200;
   end

endmodule

module CaesarCipherDecryption_simulation();
    reg [7:0] chipherChar;
    reg [4:0] shiftCount;
    wire [7:0] decryptedChar;
   
   CaesarCipherDecryption uut(chipherChar, shiftCount, decryptedChar);

   initial
   begin
        chipherChar = 8'd72; shiftCount = 0; #200;
        chipherChar = 8'd69; shiftCount = 4; #200;        
        chipherChar = 8'd76; shiftCount = 20; #200;
        chipherChar = 8'd73; shiftCount = 14; #200;
        chipherChar = 8'd78; shiftCount = 0; #200;
   end

endmodule

module CaesarEnvironment_simulation();
    reg [7:0] plainChar;
    reg [4:0] shiftCount;
    wire [7:0] chipherChar;
    wire [7:0] decryptedChar;
   
   CaesarEnvironment uut(plainChar, shiftCount, chipherChar, decryptedChar);

   initial 
   begin
        plainChar = 8'd75; shiftCount = 3; #200;
        plainChar = 8'd65; shiftCount = 1; #200;        
        plainChar = 8'd68; shiftCount = 15; #200;
        plainChar = 8'd73; shiftCount = 4; #200;
        plainChar = 8'd82; shiftCount = 3; #200;
   end

endmodule

/*
module testVigenereEnvironment();
    reg [7:0] plainChar;
    reg [79:0] keyInput;
    reg load;
    reg clk;
    wire [7:0] chipherChar;
    wire [7:0] decryptedChar;

    VigenereEnvironment uut(plainChar, keyInput, load, clk, chipherChar, decryptedChar);

    initial begin
        clk = 0; load = 1; plainChar = "I"; keyInput = "KADIROZLEM"; #66.6;
                load = 0; plainChar = "S"; keyInput = "KADIROZLEM"; #66.6;
                load = 0; plainChar = "T"; keyInput = "KADIROZLEM"; #66.6;
                load = 0; plainChar = "A"; keyInput = "KADIROZLEM"; #66.6;
                load = 0; plainChar = "N"; keyInput = "KADIROZLEM"; #66.6;
                load = 0; plainChar = "B"; keyInput = "KADIROZLEM"; #66.6;
                load = 0; plainChar = "U"; keyInput = "KADIROZLEM"; #66.6;
                load = 0; plainChar = "L"; keyInput = "KADIROZLEM"; #66.6;
                load = 0; plainChar = "T"; keyInput = "KADIROZLEM"; #66.6;
                load = 0; plainChar = "E"; keyInput = "KADIROZLEM"; #66.6;
                load = 0; plainChar = "C"; keyInput = "KADIROZLEM"; #66.6;
                load = 0; plainChar = "H"; keyInput = "KADIROZLEM"; #66.6;
                load = 0; plainChar = "N"; keyInput = "KADIROZLEM"; #66.6;
                load = 0; plainChar = "I"; keyInput = "KADIROZLEM"; #66.6;
                load = 0; plainChar = "C"; keyInput = "KADIROZLEM"; #66.6;
    end

    always begin

       clk <= ~clk; #33.3;
   end
endmodule
*/

module Vigenere_Cipher_Encryption_simulation(); 
   reg [7:0] plainChar;
   reg [79:0] keyInput;
   reg L;
   reg Clock;
   wire [7:0] chipherChar;
   Vigenere_Cipher_Encryption uut(plainChar, keyInput, L, Clock, chipherChar);

   initial begin
       Clock = 0; L = 1; plainChar = "E"; keyInput = "KADIROZLEM"; #100;
                L = 0; plainChar = "R";  #100;
                L = 0; plainChar = "I";  #100;
                L = 0; plainChar = "K";  #100;
                L = 0; plainChar = "Y";  #100;
                L = 0; plainChar = "E";  #100;
                L = 0; plainChar = "R";  #100;
                L = 0; plainChar = "I";  #100;
                L = 0; plainChar = "M";  #100;
                L = 0; plainChar = "M";  #100;

   end

   always begin
       Clock <= ~Clock; #50;
   end

endmodule

module Vigenere_Cipher_Decryption_simulation(); 
   reg [7:0] chipherChar;
   reg [79:0] keyInput;
   reg load;
   reg CLK;
   wire [7:0] decryptedChar;
   Vigenere_Cipher_Decryption uut(chipherChar, keyInput, load, CLK, decryptedChar);

   initial begin
       CLK = 0; load = 1; chipherChar = "O"; keyInput = "KADIROZLEM"; #100;
                load = 0; chipherChar = "R"; #100;
                load = 0; chipherChar = "L"; #100;
                load = 0; chipherChar = "S"; #100;
                load = 0; chipherChar = "P"; #100;
                load = 0; chipherChar = "S"; #100;
                load = 0; chipherChar = "Q"; #100;
                load = 0; chipherChar = "T"; #100;
                load = 0; chipherChar = "Q"; #100;
                load = 0; chipherChar = "Y"; #100;
   end

   always begin
       CLK <= ~CLK; #50;
   end

endmodule


module VigenereEnvironment_simulation(); 
   reg [7:0] plainChar;
   reg [79:0] keyInput;
   reg load;
   reg CLK;
   reg CLK2;
   wire [7:0] chipherChar;
   wire [7:0] decryptedChar;
   VigenereEnvironment_Module uut(plainChar, keyInput, load, CLK, CLK2 ,chipherChar, decryptedChar);

   initial begin
       CLK = 0; CLK2 = 0; load = 1; plainChar = "P"; keyInput = "DIGITALLAB"; #100;
                load = 0; plainChar = "A"; #100;
                load = 0; plainChar = "L"; #100;
                load = 0; plainChar = "Y"; #100;
                load = 0; plainChar = "A"; #100;
                load = 0; plainChar = "C"; #100;
                load = 0; plainChar = "O"; #100;
                load = 0; plainChar = "L"; #100;
                load = 0; plainChar = "U"; #100;
                load = 0; plainChar = "K"; #100;

   end

   always begin

       CLK <= ~CLK; #50;
   end
   always begin
       #0.4
       CLK2 <= ~CLK2; #49.6;
   end

endmodule

module EnigmaCommunicationSimulation(); 
   reg [7:0]plainChar;
   reg [4:0]startPosition1;
   reg [4:0]startPosition2;
   reg [4:0]startPosition3;
   reg load;
   reg clock;
   wire [7:0]chipherChar;
   wire [7:0]decryptedChar;
   EnigmaCommunication uut(plainChar, startPosition1, startPosition2, startPosition3, load,clock, chipherChar,decryptedChar);

   initial begin
        clock= 0; plainChar = "K"; startPosition1 = 5'd24; startPosition2 = 5'd25; startPosition3 = 5'd25; load = 1; #250;
        plainChar = "A"; startPosition1 = 5'd24; startPosition2 = 5'd25; startPosition3 = 5'd25; load = 0; #250;
        plainChar = "D"; startPosition1 = 5'd24; startPosition2 = 5'd25; startPosition3 = 5'd25;#250;
        plainChar = "I"; startPosition1 = 5'd24; startPosition2 = 5'd25; startPosition3 = 5'd25;#250;
   end
   
   always begin
       clock <= ~clock; #125;
   end
endmodule


module EnigmaMachineSimulation(); 
   reg [7:0]char;
   reg [4:0]startPosition1;
   reg [4:0]startPosition2;
   reg [4:0]startPosition3;
   reg load;
   reg clock;
   wire [7:0]outChar;
   EnigmaMachine uut(char, startPosition1, startPosition2, startPosition3, load,clock, outChar);

   initial begin
        clock= 0; char = "K"; startPosition1 = 5'd0; startPosition2 = 5'd0; startPosition3 = 5'd0; load = 1; #250;
        char = "A"; startPosition1 = 5'd0; startPosition2 = 5'd0; startPosition3 = 5'd0; load = 0; #250;
        char = "D"; startPosition1 = 5'd0; startPosition2 = 5'd0; startPosition3 = 5'd0;#250;
        char = "I"; startPosition1 = 5'd0; startPosition2 = 5'd0; startPosition3 = 5'd0;#250;
   end
   
   always begin
       clock <= ~clock; #125;
   end
endmodule


module PlugBoardSimulation(); 
   reg [25:0]inputConnection;
   wire [25:0]outputConnection;
   Reflector uut(inputConnection, outputConnection);

   initial begin
        inputConnection = 26'b00000000000000000000010000;#250;
        inputConnection = 26'b00000000000000010000000000;#250;
        inputConnection = 26'b00000000000001000000000000;#250;
        inputConnection = 26'b00000000000000000000100000;#250;
   end
endmodule


module Rotor3Simulation(); 
   reg [25:0]forwardInput;
   reg [25:0]backwardInput;
   reg [4:0]startPosition;
   reg load;
   reg clockIn;
   wire [25:0]forwardOutput;
   wire [25:0]backwardOutput;
   Rotor3 uut(forwardInput, backwardInput, startPosition, load, clockIn, forwardOutput, backwardOutput);

   initial begin
        clockIn = 0; startPosition = 5'd4; forwardInput = 26'b00000000000000000000010000; backwardInput = 26'b00000000000000000000000001; load = 1; #250;
        startPosition = 5'd5; forwardInput = 26'b00000000000000010000000000; backwardInput = 26'b00000000000000000000000010; load = 0; #250;
        forwardInput = 26'b00000000000001000000000000; backwardInput = 26'b00000000000000000000000100; #250;
        forwardInput = 26'b00000000000000000000100000; backwardInput = 26'b00000000000000000000001000; #250;
   end
   
   always begin
       clockIn <= ~clockIn; #125;
   end
endmodule


module Rotor2Simulation(); 
   reg [25:0]forwardInput;
   reg [25:0]backwardInput;
   reg [4:0]startPosition;
   reg load;
   reg clockIn;
   wire clockOut;
   wire [25:0]forwardOutput;
   wire [25:0]backwardOutput;
   Rotor2 uut(forwardInput, backwardInput, startPosition, load, clockIn,clockOut, forwardOutput, backwardOutput);

   initial begin
        clockIn = 0; startPosition = 5'd4; forwardInput = 26'b00000000000000000000010000; backwardInput = 26'b00000000000000000000000001; load = 1; #250;
        startPosition = 5'd5; forwardInput = 26'b00000000000000010000000000; backwardInput = 26'b00000000000000000000000010; load = 0; #250;
        forwardInput = 26'b00000000000001000000000000; backwardInput = 26'b00000000000000000000000100; #250;
        forwardInput = 26'b00000000000000000000100000; backwardInput = 26'b00000000000000000000001000; #250;
   end
   
   always begin
       clockIn <= ~clockIn; #125;
   end
endmodule


module Rotor1Simulation(); 
   reg [25:0]forwardInput;
   reg [25:0]backwardInput;
   reg [4:0]startPosition;
   reg load;
   reg clockIn;
   wire clockOut;
   wire [25:0]forwardOutput;
   wire [25:0]backwardOutput;
   Rotor1 uut(forwardInput, backwardInput, startPosition, load, clockIn,clockOut, forwardOutput, backwardOutput);

   initial begin
        clockIn = 0; startPosition = 5'd0; forwardInput = 26'b00000000000000000000010000; backwardInput = 26'b00000000000000000000000001; load = 0; #250;
        startPosition = 5'd5; forwardInput = 26'b00000000000000010000000000; backwardInput = 26'b00000000000000000000000010; load = 1; #250;
        forwardInput = 26'b00000000000001000000000000; backwardInput = 26'b00000000000000000000000100; load = 0;#250;
        forwardInput = 26'b00000000000000000000100000; backwardInput = 26'b00000000000000000000001000; #250;
   end
   
   always begin
       clockIn <= ~clockIn; #125;
   end
endmodule
