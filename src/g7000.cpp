#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <thread>

#include "bus.h"
#include "mcs48.h"
#include "intel8245.h"
#include "keyboard.h"

using namespace std;

int main()
{
  BUS bus(4, 3, 1, 0);
  MCS48 mcs48(MCS48::CPUTYPE::CPU8048, &bus);
  INTEL8245 intel8245(&bus);
  KEYBOARD keyboard(&bus);

  ifstream biosfile;

  biosfile.open("build/roms/bios_usa_eur.bin", ios::binary | ios::in);

  uint16_t wa = 0x0000;

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

    mcs48.clock();     // clock cpu
    intel8245.clock(); // clock gpu
    keyboard.clock();  // clock keyboard

    // mcs48.disassemble();
    mcs48.debug(); // output mcs-48 debug information
    cout << endl;
    bus.debug(); // output bus debug information
    cout << endl;
    // intel8245.debug();

    ++ic;

    if (ic == 0x00)
    {
      // mcs48.interrupt(); // simulate interrupt
      mcs48.setF1(1);
    }
    else
    {
      if (ic == 0x03)
      {
        mcs48.setF1(0);
      }
    }

    // this_thread::sleep_for(chrono::milliseconds(500)); // wait for 500 ms
  }

  return 0;
}