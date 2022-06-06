module CharDecoder(char, decodedChar);
    input [7:0] char;
    output [25:0] decodedChar;
    wire [7:0] temp = char - 8'b01000001;
    reg [25:0] result;

    always @(*)
    begin
        result = 26'd0;
        result[temp] = 1;
    end

    assign decodedChar = result;
endmodule

module CharEncoder(decodedChar,char);
    input [25:0] decodedChar;
    output [7:0] char;
    reg [7:0] result;
    integer x;
    integer control;

    always @(*)
    begin
        control = 0;
        x = 0;
        result = 8'b00000000;
        for(x=0; x<26; x = x + 1)
        begin
            if(~(decodedChar[x] == 0 && control == 0))
            begin
                control = 1;
            end
            else
            begin
                result = result + 8'b00000001;
            end
        end
    end
    assign char = result + 8'b01000001;
endmodule

module CircularLeftShift(shiftAmount, data, out); 
    input [25:0]data;
    input [4:0]shiftAmount;
    output [25:0]out;

    integer x;

    reg shifted;
    reg [25:0] temp;
    
    always@(*)
    begin
        shifted = temp;
        temp = data;
        x = 0;

        for(x = 0; x < shiftAmount; x = x+1)
        begin
            shifted = temp[25];
            temp = temp << 1;
            temp[0] = shifted;
        end
    end
    assign out = temp;
endmodule


module CircularRightShift(shiftAmount, data, out);
    input [4:0]shiftAmount;
    input [25:0]data;
    output [25:0]out;

    integer x;

    reg shifted;
    reg [25:0]temp;
    
    always@(*)
    begin
        x = 0;
        shifted = temp;
        temp = data;
        
        for(x = 0; x < shiftAmount; x = x+1)
        begin
            shifted = temp[0];
            temp = temp >> 1;
            temp[25] = shifted;
        end
    end
    assign out = temp;
endmodule

module CaesarCipherDecryption(chipherChar, shiftCount, decryptedChar);
    input [7:0] chipherChar;
    input [4:0] shiftCount;
    output [7:0] decryptedChar;

    wire [25:0] decoded;
    wire [25:0] result;
    
    CharDecoder second_decoding(chipherChar, decoded);
    
    CircularRightShift second_shifting(shiftCount, decoded, result);
    
    CharEncoder second_encoding2(result, decryptedChar);
    
endmodule  

module CaesarCipherEncryption(
    input [7:0] plainChar,
    input [4:0] shiftCount,
    output [7:0] chipherChar
    );
    wire [25:0] result;
    wire [25:0] decoded;
    
    CharDecoder first_decoding(plainChar, decoded);
    
    CircularLeftShift first_shifting(shiftCount, decoded, result);
    
    CharEncoder first_encoding(result, chipherChar);
    
endmodule 

module CaesarEnvironment(plainChar, shiftCount, chipherChar, decryptedChar);
    input [7:0] plainChar;
    input [4:0] shiftCount;
    output [7:0] chipherChar;
    output [7:0] decryptedChar;
    
    wire [25:0] decoded;
    wire [25:0] result;
    
    CaesarCipherEncryption third_encoding(plainChar, shiftCount, chipherChar);
    
    CaesarCipherDecryption third_decoding(chipherChar, shiftCount, decryptedChar);
        
endmodule 

module VigenereEncryption(plainChar, keyInput, load, clk, chipherChar);
    input [7:0] plainChar;
    input [79:0] keyInput;
    input load;
    input clk;
    output reg [7:0] chipherChar;
    reg [9:0] keyRegister;

always @(posedge clk) begin
    if(load)begin
        keyRegister=keyInput;
    end

    else begin
        chipherChar=(plainChar+keyRegister)%26; //C_i = (P_i + K_i) mod 26
    end
end

endmodule

module VigenereDecryption(chipherChar, keyInput, load, clk, decryptedChar);
    input [7:0] chipherChar;
    input [79:0] keyInput;
    input load;
    input clk;
    output reg [7:0] decryptedChar;

    reg [9:0] keyRegister;

always @(posedge clk) begin
    if(load)
    begin
        keyRegister=keyInput;
    end
    else 
    begin
        decryptedChar=(chipherChar-keyRegister)%26; //D_i = (C_i - K_i) mod 26
    end
end

endmodule

module VigenereEnvironment(plainChar, keyInput, load, clk, chipherChar, decryptedChar);
    input [7:0] plainChar;
    input [79:0] keyInput;
    input load;
    input clk;
    output [7:0] chipherChar;
    output [7:0] decryptedChar;

    wire [7:0] temporary;

    VigenereEncryption encryption(plainChar, keyInput, load, clk, temporary);
    VigenereDecryption decryption(temporary, keyInput, load, clk, decryptedChar);

    assign chipherChar=temporary;
endmodule

module PlugBoard(
    input [25:0]charInput,
    input [25:0]backwardInput,
    output [25:0]forwardOutput,
    output [25:0]charOutput
    );

    assign   charOutput[4] = backwardInput[0];
    assign    charOutput[10] = backwardInput[1];
    assign    charOutput[12] = backwardInput[2];
    assign    charOutput[5] = backwardInput[3];
    assign    charOutput[11] = backwardInput[4];
    assign    charOutput[6] = backwardInput[5];
    assign    charOutput[3] = backwardInput[6];
    assign    charOutput[16] = backwardInput[7];
    assign    charOutput[21] = backwardInput[8];
    assign   charOutput[25] = backwardInput[9];
    assign   charOutput[13] = backwardInput[10];
    assign    charOutput[19] = backwardInput[11];
    assign    charOutput[14] = backwardInput[12];
    assign    charOutput[22] = backwardInput[13];
    assign    charOutput[24] = backwardInput[14];
    assign    charOutput[7] = backwardInput[15];
    assign    charOutput[23] = backwardInput[16];
    assign    charOutput[20] = backwardInput[17];
    assign    charOutput[18] = backwardInput[18];
    assign    charOutput[15] = backwardInput[19];
    assign    charOutput[0] = backwardInput[20];
    assign    charOutput[8] = backwardInput[21];
    assign    charOutput[1] = backwardInput[22];
    assign    charOutput[17] = backwardInput[23];
    assign    charOutput[2] = backwardInput[24];
    assign   charOutput[9] = backwardInput[25];
        //
    assign    forwardOutput[0] = charInput[4];
    assign    forwardOutput[1] = charInput[10];
    assign    forwardOutput[2] = charInput[12];
    assign    forwardOutput[3] = charInput[5];
    assign    forwardOutput[4] = charInput[11];
    assign    forwardOutput[5] = charInput[6];
    assign    forwardOutput[6] = charInput[3];
    assign   forwardOutput[7] = charInput[16];
    assign    forwardOutput[8] = charInput[21];
    assign    forwardOutput[9] = charInput[25];
    assign   forwardOutput[10] = charInput[13];
    assign    forwardOutput[11] = charInput[19];
    assign    forwardOutput[12] = charInput[14];
    assign    forwardOutput[13] = charInput[22];
    assign    forwardOutput[14] = charInput[24];
    assign    forwardOutput[15] = charInput[7];
    assign    forwardOutput[16] = charInput[23];
    assign    forwardOutput[17] = charInput[20];
    assign    forwardOutput[18] = charInput[18];
    assign    forwardOutput[19] = charInput[15];
    assign    forwardOutput[20] = charInput[0];
    assign    forwardOutput[21] = charInput[8];
    assign    forwardOutput[22] = charInput[1];
    assign    forwardOutput[23] = charInput[17];
    assign    forwardOutput[24] = charInput[2];
    assign    forwardOutput[25] = charInput[9];    
    
endmodule

module Rotor1(
    input [25:0]forwardInput,
    input [25:0]backwardInput,
    input [4:0]startPosition,
    input load,
    input clockIn,
    output clockOut,
    output [25:0]forwardOutput,
    output [25:0]backwardOutput
    );


    wire[25:0]forwardInput1;
    wire[25:0]forwardInput2;
    wire[25:0]forwardOutput1;
    
    wire[25:0]backwardInput1;
    wire[25:0]backwardInput2;
    wire[25:0]backwardOutput1;
    
    wire [4:0]startPos;
    
    reg load_flag = 0;
    always @(posedge load)
    begin
        load_flag = 1;
        //after this line, the input is shifted to right as much as startPosition
    end
    assign startPos[0] = startPosition[0] & load_flag;
    assign startPos[1] = startPosition[1] & load_flag;
    assign startPos[2] = startPosition[2] & load_flag;
    assign startPos[3] = startPosition[3] & load_flag;
    assign startPos[4] = startPosition[4] & load_flag;
    
    CircularRightShift shift_right_start(forwardInput,startPos,forwardInput1);
    CircularRightShift shift_right_start_back(backwardInput,startPos,backwardInput1);
    
    integer clock_counter = 0;
    reg clk;
    integer flag = 0;
    time clock_flag = 0;
    always @(posedge clockIn)
    begin
        clk <= 0;
        
        clock_flag = 1;       //shifting right one bit
        
        if(flag == 1)
        begin
            clk <= 0;
            flag = 0;
        end
        
        if(clock_counter == 25)
        begin
            clk <= 1;
            flag = 1;
            clock_counter = 0;
        end
        
        //increment the index
        clock_counter = clock_counter + 1;  
    end
    
    CircularRightShift shift_right_one(forwardInput1,clock_flag,forwardInput2);
    CircularRightShift shift_right_one_back(backwardInput1,clock_flag,backwardInput2);
    
    
        assign backwardOutput1[7] = backwardInput2[0];
        assign backwardOutput1[12] = backwardInput2[1];
        assign backwardOutput1[21] = backwardInput2[2];
        assign backwardOutput1[17] = backwardInput2[3];
        assign backwardOutput1[0] = backwardInput2[4];
        assign backwardOutput1[2] = backwardInput2[5];
        assign backwardOutput1[22] = backwardInput2[6];
        assign backwardOutput1[20] = backwardInput2[7];
        assign backwardOutput1[23] = backwardInput2[8];
        assign backwardOutput1[18] = backwardInput2[9];
        assign backwardOutput1[9] = backwardInput2[10];
        assign backwardOutput1[25] = backwardInput2[11];
        assign backwardOutput1[15] = backwardInput2[12];
        assign backwardOutput1[3] = backwardInput2[13];
        assign backwardOutput1[14] = backwardInput2[14];
        assign backwardOutput1[13] = backwardInput2[15];
        assign backwardOutput1[11] = backwardInput2[16];
        assign backwardOutput1[8] = backwardInput2[17];
        assign backwardOutput1[4] = backwardInput2[18];
        assign backwardOutput1[10] = backwardInput2[19];
        assign backwardOutput1[6] = backwardInput2[20];
        assign backwardOutput1[5] = backwardInput2[21];
        assign backwardOutput1[19] = backwardInput2[22];
        assign backwardOutput1[16] = backwardInput2[23];
        assign backwardOutput1[24] = backwardInput2[24];
        assign backwardOutput1[1] = backwardInput2[25];
        
        //
        assign forwardOutput1[0] = forwardInput2[7];
        assign forwardOutput1[1] = forwardInput2[12];
        assign forwardOutput1[2] = forwardInput2[21];
        assign forwardOutput1[3] = forwardInput2[17];
        assign forwardOutput1[4] = forwardInput2[0];
        assign forwardOutput1[5] = forwardInput2[2];
        assign forwardOutput1[6] = forwardInput2[22];
        assign forwardOutput1[7] = forwardInput2[20];
        assign forwardOutput1[8] = forwardInput2[23];
        assign forwardOutput1[9] = forwardInput2[18];
        assign forwardOutput1[10] = forwardInput2[9];
        assign forwardOutput1[11] = forwardInput2[25];
        assign forwardOutput1[12] = forwardInput2[15];
        assign forwardOutput1[13] = forwardInput2[3];
        assign forwardOutput1[14] = forwardInput2[14];
        assign forwardOutput1[15] = forwardInput2[13];
        assign forwardOutput1[16] = forwardInput2[11];
        assign forwardOutput1[17] = forwardInput2[8];
        assign forwardOutput1[18] = forwardInput2[4];
        assign forwardOutput1[19] = forwardInput2[10];
        assign forwardOutput1[20] = forwardInput2[6];
        assign forwardOutput1[21] = forwardInput2[5];
        assign forwardOutput1[22] = forwardInput2[19];
        assign forwardOutput1[23] = forwardInput2[16];
        assign forwardOutput1[24] = forwardInput2[24];
        assign forwardOutput1[25] = forwardInput2[1];
        
    CircularLeftShift shift_left_one(forwardOutput1,clock_flag,forwardOutput);
    CircularLeftShift shift_left_one_back(backwardOutput1,clock_flag,backwardOutput);
    
    assign clockOut = clk;
endmodule

module Rotor2(
    input [25:0]forwardInput,
    input [25:0]backwardInput,
    input [4:0]startPosition,
    input load,
    input clockIn,
    output clockOut,
    output [25:0]forwardOutput,
    output [25:0]backwardOutput
    );


    wire[25:0]forwardInput1;
    wire[25:0]forwardInput2;
    wire[25:0]forwardOutput1;
    
    wire[25:0]backwardInput1;
    wire[25:0]backwardInput2;
    wire[25:0]backwardOutput1;
    
    wire [4:0]startPos;
    
    reg load_flag = 0;
    always @(posedge load)
    begin
        load_flag = 1;
        //after this line, the input is shifted to right as much as startPosition
    end
    assign startPos[0] = startPosition[0] & load_flag;
    assign startPos[1] = startPosition[1] & load_flag;
    assign startPos[2] = startPosition[2] & load_flag;
    assign startPos[3] = startPosition[3] & load_flag;
    assign startPos[4] = startPosition[4] & load_flag;
    
    CircularRightShift shift_right_start(forwardInput,startPos,forwardInput1);
    CircularRightShift shift_right_start_back(backwardInput,startPos,backwardInput1);
    
    integer clock_counter = 0;
    reg clk;
    integer flag = 0;
    time clock_flag = 0;
    always @(posedge clockIn)
    begin
        clk <= 0;
        
        clock_flag = 1;       //shifting right one bit
        
        if(flag == 1)
        begin
            clk <= 0;
            flag = 0;
        end
        
        if(clock_counter == 25)
        begin
            clk <= 1;
            flag = 1;
            clock_counter = 0;
        end
        
        //increment the index
        clock_counter = clock_counter + 1;  
    end
    
    CircularRightShift shift_right_one(forwardInput1,clock_flag,forwardInput2);
    CircularRightShift shift_right_one_back(backwardInput1,clock_flag,backwardInput2);
    
    
        assign backwardOutput1[19] = backwardInput2[0];
        assign backwardOutput1[4] = backwardInput2[1];
        assign backwardOutput1[7] = backwardInput2[2];
        assign backwardOutput1[6] = backwardInput2[3];
        assign backwardOutput1[12] = backwardInput2[4];
        assign backwardOutput1[17] = backwardInput2[5];
        assign backwardOutput1[8] = backwardInput2[6];
        assign backwardOutput1[5] = backwardInput2[7];
        assign backwardOutput1[2] = backwardInput2[8];
        assign backwardOutput1[0] = backwardInput2[9];
        assign backwardOutput1[1] = backwardInput2[10];
        assign backwardOutput1[20] = backwardInput2[11];
        assign backwardOutput1[25] = backwardInput2[12];
        assign backwardOutput1[9] = backwardInput2[13];
        assign backwardOutput1[14] = backwardInput2[14];
        assign backwardOutput1[22] = backwardInput2[15];
        assign backwardOutput1[24] = backwardInput2[16];
        assign backwardOutput1[18] = backwardInput2[17];
        assign backwardOutput1[15] = backwardInput2[18];
        assign backwardOutput1[13] = backwardInput2[19];
        assign backwardOutput1[3] = backwardInput2[20];
        assign backwardOutput1[10] = backwardInput2[21];
        assign backwardOutput1[21] = backwardInput2[22];
        assign backwardOutput1[16] = backwardInput2[23];
        assign backwardOutput1[11] = backwardInput2[24];
        assign backwardOutput1[23] = backwardInput2[25];
        
        //
        assign forwardOutput1[0] = forwardInput2[19];
        assign forwardOutput1[1] = forwardInput2[4];
        assign forwardOutput1[2] = forwardInput2[7];
        assign forwardOutput1[3] = forwardInput2[6];
        assign forwardOutput1[4] = forwardInput2[12];
        assign forwardOutput1[5] = forwardInput2[17];
        assign forwardOutput1[6] = forwardInput2[8];
        assign forwardOutput1[7] = forwardInput2[5];
        assign forwardOutput1[8] = forwardInput2[2];
        assign forwardOutput1[9] = forwardInput2[0];
        assign forwardOutput1[10] = forwardInput2[1];
        assign forwardOutput1[11] = forwardInput2[20];
        assign forwardOutput1[12] = forwardInput2[25];
        assign forwardOutput1[13] = forwardInput2[9];
        assign forwardOutput1[14] = forwardInput2[14];
        assign forwardOutput1[15] = forwardInput2[22];
        assign forwardOutput1[16] = forwardInput2[24];
        assign forwardOutput1[17] = forwardInput2[18];
        assign forwardOutput1[18] = forwardInput2[15];
        assign forwardOutput1[19] = forwardInput2[13];
        assign forwardOutput1[20] = forwardInput2[3];
        assign forwardOutput1[21] = forwardInput2[10];
        assign forwardOutput1[22] = forwardInput2[21];
        assign forwardOutput1[23] = forwardInput2[16];
        assign forwardOutput1[24] = forwardInput2[11];
        assign forwardOutput1[25] = forwardInput2[23];
        
    CircularLeftShift shift_left_one(forwardOutput1,clock_flag,forwardOutput);
    CircularLeftShift shift_left_one_back(backwardOutput1,clock_flag,backwardOutput);
    
    assign clockOut = clk;
endmodule

module Rotor3(
    input [25:0]forwardInput,
    input [25:0]backwardInput,
    input [4:0]startPosition,
    input load,
    input clockIn,
    output [25:0]forwardOutput,
    output [25:0]backwardOutput
    );


    wire[25:0]forwardInput1;
    wire[25:0]forwardInput2;
    wire[25:0]forwardOutput1;
    
    wire[25:0]backwardInput1;
    wire[25:0]backwardInput2;
    wire[25:0]backwardOutput1;
    
    wire [4:0]startPos;
    
    reg load_flag = 0;
    always @(posedge load)
    begin
        load_flag = 1;
        //after this line, the input is shifted to right as much as startPosition
    end
    assign startPos[0] = startPosition[0] & load_flag;
    assign startPos[1] = startPosition[1] & load_flag;
    assign startPos[2] = startPosition[2] & load_flag;
    assign startPos[3] = startPosition[3] & load_flag;
    assign startPos[4] = startPosition[4] & load_flag;
    
    CircularRightShift shift_right_start(forwardInput,startPos,forwardInput1);
    CircularRightShift shift_right_start_back(backwardInput,startPos,backwardInput1);
    
    time clock_flag = 0;
    always @(posedge clockIn)
    begin        
        clock_flag = 1;       //shifting right one bit
    end
    
    CircularRightShift shift_right_one(forwardInput1,clock_flag,forwardInput2);
    CircularRightShift shift_right_one_back(backwardInput1,clock_flag,backwardInput2);
    
    
        assign backwardOutput1[19] = backwardInput2[0];
        assign backwardOutput1[0] = backwardInput2[1];
        assign backwardOutput1[6] = backwardInput2[2];
        assign backwardOutput1[1] = backwardInput2[3];
        assign backwardOutput1[15] = backwardInput2[4];
        assign backwardOutput1[2] = backwardInput2[5];
        assign backwardOutput1[18] = backwardInput2[6];
        assign backwardOutput1[3] = backwardInput2[7];
        assign backwardOutput1[16] = backwardInput2[8];
        assign backwardOutput1[4] = backwardInput2[9];
        assign backwardOutput1[20] = backwardInput2[10];
        assign backwardOutput1[5] = backwardInput2[11];
        assign backwardOutput1[21] = backwardInput2[12];
        assign backwardOutput1[13] = backwardInput2[13];
        assign backwardOutput1[25] = backwardInput2[14];
        assign backwardOutput1[7] = backwardInput2[15];
        assign backwardOutput1[24] = backwardInput2[16];
        assign backwardOutput1[8] = backwardInput2[17];
        assign backwardOutput1[23] = backwardInput2[18];
        assign backwardOutput1[9] = backwardInput2[19];
        assign backwardOutput1[22] = backwardInput2[20];
        assign backwardOutput1[11] = backwardInput2[21];
        assign backwardOutput1[17] = backwardInput2[22];
        assign backwardOutput1[10] = backwardInput2[23];
        assign backwardOutput1[14] = backwardInput2[24];
        assign backwardOutput1[12] = backwardInput2[25];
        
        //
        assign forwardOutput1[0] = forwardInput2[19];
        assign forwardOutput1[1] = forwardInput2[0];
        assign forwardOutput1[2] = forwardInput2[6];
        assign forwardOutput1[3] = forwardInput2[1];
        assign forwardOutput1[4] = forwardInput2[15];
        assign forwardOutput1[5] = forwardInput2[2];
        assign forwardOutput1[6] = forwardInput2[18];
        assign forwardOutput1[7] = forwardInput2[3];
        assign forwardOutput1[8] = forwardInput2[16];
        assign forwardOutput1[9] = forwardInput2[4];
        assign forwardOutput1[10] = forwardInput2[20];
        assign forwardOutput1[11] = forwardInput2[5];
        assign forwardOutput1[12] = forwardInput2[21];
        assign forwardOutput1[13] = forwardInput2[13];
        assign forwardOutput1[14] = forwardInput2[25];
        assign forwardOutput1[15] = forwardInput2[7];
        assign forwardOutput1[16] = forwardInput2[24];
        assign forwardOutput1[17] = forwardInput2[8];
        assign forwardOutput1[18] = forwardInput2[23];
        assign forwardOutput1[19] = forwardInput2[9];
        assign forwardOutput1[20] = forwardInput2[22];
        assign forwardOutput1[21] = forwardInput2[11];
        assign forwardOutput1[22] = forwardInput2[17];
        assign forwardOutput1[23] = forwardInput2[10];
        assign forwardOutput1[24] = forwardInput2[14];
        assign forwardOutput1[25] = forwardInput2[12];
        
    CircularLeftShift shift_left_one(forwardOutput1,clock_flag,forwardOutput);
    CircularLeftShift shift_left_one_back(backwardOutput1,clock_flag,backwardOutput);
    
endmodule

module Reflector(
    input [25:0]inputConnection,
    output [25:0]outputConnection
    );

    assign  outputConnection[0] = inputConnection[24];
    assign  outputConnection[1] = inputConnection[17];
    assign  outputConnection[2] = inputConnection[20];
    assign  outputConnection[3] = inputConnection[7];
    assign  outputConnection[4] = inputConnection[16];
    assign  outputConnection[5] = inputConnection[18];
    assign  outputConnection[6] = inputConnection[11];
    assign  outputConnection[7] = inputConnection[3];
    assign  outputConnection[8] = inputConnection[15];
    assign  outputConnection[9] = inputConnection[23];
    assign  outputConnection[10] = inputConnection[13];
    assign  outputConnection[11] = inputConnection[6];
    assign  outputConnection[12] = inputConnection[14];
    assign  outputConnection[13] = inputConnection[10];
    assign  outputConnection[14] = inputConnection[12];
    assign  outputConnection[15] = inputConnection[8];
    assign  outputConnection[16] = inputConnection[4];
    assign  outputConnection[17] = inputConnection[1];
    assign  outputConnection[18] = inputConnection[5];
    assign  outputConnection[19] = inputConnection[25];
    assign  outputConnection[20] = inputConnection[2];
    assign  outputConnection[21] = inputConnection[22];
    assign  outputConnection[22] = inputConnection[21];
    assign  outputConnection[23] = inputConnection[9];
    assign  outputConnection[24] = inputConnection[0];
    assign  outputConnection[25] = inputConnection[19];
endmodule


module EnigmaMachine(input [7:0]char,input [4:0]startPosition1,input [4:0]startPosition2,input [4:0]startPosition3,
    input load,
    input clock,
    output [7:0]outChar
    );
    wire clock1;
    wire clock2;
    wire [25:0] decodedChar;
    
    wire [25:0] backwardInput;
    wire [25:0] forwardOutput;
    wire [25:0] charOutput;
    
    wire [25:0] forwardOutput1;
    wire [25:0] backwardOutput1;
    
    wire [25:0] forwardOutput2;
    wire [25:0] backwardOutput2;
    
    wire [25:0] forwardOutput3;
    wire [25:0] backwardOutput3;
    
    CharDecoder decoder(char,decodedChar);
    PlugBoard plug(decodedChar,backwardInput,forwardOutput,charOutput);
    Rotor1 rotor1(forwardOutput,charOutput,startPosition1,load,clock,clock1,forwardOutput1,backwardOutput1);
    Rotor2 rotor2(forwardOutput1,backwardOutput1,startPosition2,load,clock1,clock2,forwardOutput2,backwardOutput2);
    Rotor3 rotor3(forwardOutput2,backwardOutput2,startPosition3,load,clock2,forwardOutput3,backwardOutput3);
    Reflector reflector1(forwardOutput3,backwardOutput3);
    
    CharEncoder encoder(backwardOutput3,outChar);
endmodule

module EnigmaCommunication(
    input [7:0]plainChar,
    input [4:0]startPosition1,
    input [4:0]startPosition2,
    input [4:0]startPosition3,
    input load,
    input clock,
    output [7:0]chipherChar,
    output [7:0]decryptedChar
    );
    
    EnigmaMachine enigma1(plainChar,startPosition1,startPosition2,startPosition3,load,clock,chipherChar);
    
    EnigmaMachine enigma2(chipherChar,startPosition1,startPosition2,startPosition3,load,clock,decryptedChar);
endmodule
