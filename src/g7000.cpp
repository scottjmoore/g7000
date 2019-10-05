#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <thread>

#include "bus.h"
#include "mcs48.h"
#include "intel8245.h"

using namespace std;

int main()
{
  BUS bus(4, 3, 1, 0);
  MCS48 mcs48(MCS48::CPUTYPE::CPU8048, &bus);
  INTEL8245 intel8245(&bus);

  uint16_t wa = 0x0000;

  /*
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
  */

  // mcs48.writeROM(wa++, 0b10100111); // 0000H  CPL C
  // mcs48.writeROM(wa++, 0b00010100); // 0001H  CALL 0005H
  // mcs48.writeROM(wa++, 0x05);
  // mcs48.writeROM(wa++, 0b00000100); // 0003H  JMP 0000H
  // mcs48.writeROM(wa++, 0x00);
  // mcs48.writeROM(wa++, 0b00010111); // 0005H  INC A
  // mcs48.writeROM(wa++, 0b10010111); // 0006H  CLR C
  // mcs48.writeROM(wa++, 0b00010100); // 0007H  CALL 000AH
  // mcs48.writeROM(wa++, 0x0a);
  // mcs48.writeROM(wa++, 0b10010011); // 0009H  RETR
  // mcs48.writeROM(wa++, 0b00010111); // 000aH  INC A
  // mcs48.writeROM(wa++, 0b10100111); // 000bH  CPL C
  // mcs48.writeROM(wa++, 0b10010011); // 000cH  RETR

  // mcs48.writeROM(wa++, 0b10111000); // MOV R0
  // mcs48.writeROM(wa++, 32);         // #32
  // mcs48.writeROM(wa++, 0b10111011); // MOV R3
  // mcs48.writeROM(wa++, 4);          // #4
  // mcs48.writeROM(wa++, 0b00010000); // INC @R0
  // mcs48.writeROM(wa++, 0b00011000); // INC R0
  // mcs48.writeROM(wa++, 0b11101011); // DJNZ R3
  // mcs48.writeROM(wa++, 0x04);       // 0004H

  // mcs48.writeROM(wa++, 0b00001001); // IN A, P1
  // mcs48.writeROM(wa++, 0b00001010); // IN A, P2
  // mcs48.writeROM(wa++, 0b00001000); // INS A, BUS
  // mcs48.writeROM(wa++, 0b00001100); // MOVD A, P4
  // mcs48.writeROM(wa++, 0b00001101); // MOVD A, P5
  // mcs48.writeROM(wa++, 0b00001110); // MOVD A, P6
  // mcs48.writeROM(wa++, 0b00001111); // MOVD A, P7
  // mcs48.writeROM(wa++, 0b00100011); // MOV A,
  // mcs48.writeROM(wa++, 0x0f);       // #0fH
  // mcs48.writeROM(wa++, 0b00111100); // MOVD P4, A
  // mcs48.writeROM(wa++, 0b00111101); // MOVD P5, A
  // mcs48.writeROM(wa++, 0b00111110); // MOVD P6, A
  // mcs48.writeROM(wa++, 0b00111111); // MOVD P7, A
  // mcs48.writeROM(wa++, 0b00111001); // OUTL P1, A
  // mcs48.writeROM(wa++, 0b00111010); // OUTL P2, A
  // mcs48.writeROM(wa++, 0b00000010); // OUTL BUS, A
  // mcs48.writeROM(wa++, 0b01000111); // SWAP A
  // mcs48.writeROM(wa++, 0b00111100); // MOVD P4, A
  // mcs48.writeROM(wa++, 0b00111101); // MOVD P5, A
  // mcs48.writeROM(wa++, 0b00111110); // MOVD P6, A
  // mcs48.writeROM(wa++, 0b00111111); // MOVD P7, A
  // mcs48.writeROM(wa++, 0b00111001); // OUTL P1, A
  // mcs48.writeROM(wa++, 0b00111010); // OUTL P2, A
  // mcs48.writeROM(wa++, 0b00000010); // OUTL BUS, A

  // mcs48.writeROM(wa++, 0b00100011); // MOV A,
  // mcs48.writeROM(wa++, 0x01);       // #01H
  // mcs48.writeROM(wa++, 0b11100111); // RL A,
  // mcs48.writeROM(wa++, 0b11100111); // RL A,
  // mcs48.writeROM(wa++, 0b11100111); // RL A,
  // mcs48.writeROM(wa++, 0b11100111); // RL A,
  // mcs48.writeROM(wa++, 0b11100111); // RL A,
  // mcs48.writeROM(wa++, 0b11100111); // RL A,
  // mcs48.writeROM(wa++, 0b11100111); // RL A,
  // mcs48.writeROM(wa++, 0b11100111); // RL A,
  // mcs48.writeROM(wa++, 0b00100011); // MOV A,
  // mcs48.writeROM(wa++, 0x80);       // #80H
  // mcs48.writeROM(wa++, 0b01110111); // RR A,
  // mcs48.writeROM(wa++, 0b01110111); // RR A,
  // mcs48.writeROM(wa++, 0b01110111); // RR A,
  // mcs48.writeROM(wa++, 0b01110111); // RR A,
  // mcs48.writeROM(wa++, 0b01110111); // RR A,
  // mcs48.writeROM(wa++, 0b01110111); // RR A,
  // mcs48.writeROM(wa++, 0b01110111); // RR A,
  // mcs48.writeROM(wa++, 0b01110111); // RR A,

  // mcs48.writeROM(wa++, 0b00100011); // MOV A,
  // mcs48.writeROM(wa++, 0x01);       // #01H
  // mcs48.writeROM(wa++, 0b11110111); // RLC A,
  // mcs48.writeROM(wa++, 0b11110111); // RLC A,
  // mcs48.writeROM(wa++, 0b11110111); // RLC A,
  // mcs48.writeROM(wa++, 0b11110111); // RLC A,
  // mcs48.writeROM(wa++, 0b11110111); // RLC A,
  // mcs48.writeROM(wa++, 0b11110111); // RLC A,
  // mcs48.writeROM(wa++, 0b11110111); // RLC A,
  // mcs48.writeROM(wa++, 0b11110111); // RLC A,
  // mcs48.writeROM(wa++, 0b11110111); // RLC A,
  // mcs48.writeROM(wa++, 0b00100011); // MOV A,
  // mcs48.writeROM(wa++, 0x80);       // #80H
  // mcs48.writeROM(wa++, 0b01100111); // RRC A,
  // mcs48.writeROM(wa++, 0b01100111); // RRC A,
  // mcs48.writeROM(wa++, 0b01100111); // RRC A,
  // mcs48.writeROM(wa++, 0b01100111); // RRC A,
  // mcs48.writeROM(wa++, 0b01100111); // RRC A,
  // mcs48.writeROM(wa++, 0b01100111); // RRC A,
  // mcs48.writeROM(wa++, 0b01100111); // RRC A,
  // mcs48.writeROM(wa++, 0b01100111); // RRC A,
  // mcs48.writeROM(wa++, 0b01100111); // RRC A,

  // mcs48.writeROM(wa++, 0b11100101); // SEL MB0
  // mcs48.writeROM(wa++, 0b00000100); // JMP 0004H
  // mcs48.writeROM(wa++, 0x04);
  // mcs48.writeROM(wa++, 0x00);       // NOP
  // mcs48.writeROM(wa++, 0b11110101); // SEL MB1
  // mcs48.writeROM(wa++, 0b00000100); // JMP 0800H
  // mcs48.writeROM(wa++, 0x00);

  // mcs48.writeROM(wa++, 0b00000000); // NOP
  // mcs48.writeROM(wa++, 0b00000000); // NOP
  // mcs48.writeROM(wa++, 0b00000000); // NOP
  // mcs48.writeROM(wa++, 0b00000000); // NOP
  // mcs48.writeROM(wa++, 0b00000000); // NOP
  // mcs48.writeROM(wa++, 0b00000000); // NOP
  // mcs48.writeROM(wa++, 0b00000000); // NOP
  // mcs48.writeROM(wa++, 0b00100011); // MOV A,
  // mcs48.writeROM(wa++, 0xff);       // #FFH
  // mcs48.writeROM(wa++, 0b01100010); // MOV T, A
  // mcs48.writeROM(wa++, 0b00100101); // EN TCNTI
  // mcs48.writeROM(wa++, 0b01010101); // STRT T

  // mcs48.writeROM(wa++, 0b00100011); // MOV A,
  // mcs48.writeROM(wa++, 0xff);       // #ffH
  // mcs48.writeROM(wa++, 0b10001100); // ORLD P4, A
  // mcs48.writeROM(wa++, 0b10001101); // ORLD P5, A
  // mcs48.writeROM(wa++, 0b10001110); // ORLD P6, A
  // mcs48.writeROM(wa++, 0b10001111); // ORLD P7, A
  // mcs48.writeROM(wa++, 0b10001001); // ORL P1,
  // mcs48.writeROM(wa++, 0xff);       // #ffH
  // mcs48.writeROM(wa++, 0b10001010); // ORL P2,
  // mcs48.writeROM(wa++, 0xff);       // #ffH

  ifstream biosfile;

  biosfile.open("build/roms/bios_usa_eur.bin", ios::binary | ios::in);

  for (int i = 0; i < 0x400; ++i)
  {
    uint8_t c;

    biosfile.read((char *)&c, 1);
    mcs48.writeROM(wa++, c);
  }

  biosfile.close();

  uint8_t cartridge[2048];
  ifstream cartridgefile;

  cartridgefile.open("build/roms/hockey_soccer_usa_eur.bin", ios::binary | ios::in);

  for (int i = 0; i < 0x800; ++i)
  {
    uint8_t c;

    cartridgefile.read((char *)&c, 1);
    cartridge[i] = c;
  }

  cartridgefile.close();

  bus.attachROM(cartridge, 0x400, 0x800);

  mcs48.reset(); // reset cpu

  uint8_t ic = 0;

  while (1)
  {
    cout << "\x1B[2J\x1B[H"; // clear console

    for (int i = 0; i < 64; i++)
    {
      mcs48.clock();     // clock cpu
      intel8245.clock(); // clock gpu
    }

    // mcs48.disassemble();
    mcs48.debug(); // output mcs-48 debug information
    cout << endl;
    // bus.debug(); // output bus debug information
    // cout << endl;
    intel8245.debug();

    ++ic;

    if (ic == 0x00)
    {
      // mcs48.interrupt(); // simulate interrupt
      mcs48.setF1(1);
    }
    else
    {
      mcs48.setF1(0);
    }

    // this_thread::sleep_for(chrono::milliseconds(500)); // wait for 500 ms
  }

  return 0;
}