#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include "mcs48.h"

using namespace std;

int main()
{
  MCS48 mcs48;
  uint16_t wa = 0x0000;

  mcs48.writeROM(wa++, 0b00100111); // CLR A
  mcs48.writeROM(wa++, 0b00110111); // CPL A
  mcs48.writeROM(wa++, 0b00100111); // CLR A
  mcs48.writeROM(wa++, 0b00000011); // ADD A,
  mcs48.writeROM(wa++, 128);        // #128
  mcs48.writeROM(wa++, 0b00000011); // ADD A,
  mcs48.writeROM(wa++, 129);        // #129
  mcs48.writeROM(wa++, 0b10100111); // CPL C
  mcs48.writeROM(wa++, 0b10010111); // CLR C
  mcs48.writeROM(wa++, 0b11010101); // SEL RB1
  mcs48.writeROM(wa++, 0b11000101); // SEL RB0
  mcs48.writeROM(wa++, 0b00100011); // MOV A,
  mcs48.writeROM(wa++, 255);        // #255
  mcs48.writeROM(wa++, 0b11010011); // XRL A,
  mcs48.writeROM(wa++, 0b10101010); // #0b10101010
  mcs48.writeROM(wa++, 0b10101000); // MOV R0, A
  mcs48.writeROM(wa++, 0b11010101); // SEL RB1
  mcs48.writeROM(wa++, 0b10111001); // MOV R1, #123
  mcs48.writeROM(wa++, 123);
  mcs48.writeROM(wa++, 0b11000101); // SEL RB0
  mcs48.writeROM(wa++, 0b10111010); // MOV R2, #210
  mcs48.writeROM(wa++, 210);
  mcs48.writeROM(wa++, 0b00011100); // INC R4
  mcs48.writeROM(wa++, 0b00000100); // JMP 0
  mcs48.writeROM(wa++, 0);

  mcs48.reset();

  while (1)
  {
    cout << "\x1B[2J\x1B[H";

    mcs48.clock();
    mcs48.debug();

    this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return 0;
}