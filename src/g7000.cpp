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

  mcs48.writeROM(0, 0b00100111);  // CLR A
  mcs48.writeROM(1, 0b00110111);  // CPL A
  mcs48.writeROM(2, 0b00100111);  // CLR A
  mcs48.writeROM(3, 0b00000011);  // ADD A,
  mcs48.writeROM(4, 128);         // #128
  mcs48.writeROM(5, 0b00000011);  // ADD A,
  mcs48.writeROM(6, 129);         // #129
  mcs48.writeROM(7, 0b10100111);  // CPL C
  mcs48.writeROM(8, 0b10010111);  // CLR C
  mcs48.writeROM(9, 0b11010101);  // SEL RB1
  mcs48.writeROM(10, 0b11000101); // SEL RB0
  mcs48.writeROM(11, 0b00100011); // MOV A,
  mcs48.writeROM(12, 255);        // #255
  mcs48.writeROM(13, 0b11010011); // XRL A,
  mcs48.writeROM(14, 0b10101010); // #0b10101010
  mcs48.writeROM(15, 0b00000100); // JMP 0

  mcs48.reset();

  while (1)
  {
    mcs48.clock();
    mcs48.debug();

    this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return 0;
}