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

#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"

class G7000 : public olc::PixelGameEngine
{
private:
  BUS *bus;
  MCS48 *mcs48;
  INTEL8245 *intel8245;
  KEYBOARD *keyboard;

public:
  G7000()
  {
    sAppName = "G7000";

    bus = new BUS(4, 2, 1, 0);                       // create bus class with 4 x 4 bit, 2 x 8 bit and 1 x 16 bit ports
    mcs48 = new MCS48(MCS48::CPUTYPE::CPU8048, bus); // create cpu class as an 8048 and attach it to the bus
    intel8245 = new INTEL8245(bus);                  // create intel 8245 display class and attach it to the bus
    keyboard = new KEYBOARD(bus);                    // create keyboard class and attach it to the bus
  }

public:
  bool OnUserCreate() override
  {
    ifstream biosfile; // file stream to read bios file

    biosfile.open("build/roms/bios_usa_eur.bin", ios::binary | ios::in); // open G7000 bios file

    for (int i = 0; i < 0x400; ++i) // loop through 1KB using 'i' as index
    {
      uint8_t c; //  byte size variable to hold byte read from bios file

      biosfile.read((char *)&c, 1); // read byte from bios file
      mcs48->writeROM(i, c);        // write bios byte into CPU internal ROM using 'i' as index
    }

    biosfile.close(); // close bios file

    uint8_t cartridge[2048]; // create array of 2048KB to store cartridge data
    ifstream cartridgefile;  // file stream to read cartridge file

    cartridgefile.open("build/roms/alpine_skiing_usa_eur.bin", ios::binary | ios::in); // open G7000 cartridge file

    for (int i = 0; i < 0x800; ++i) // loop through 2048KB using 'i' as index
    {
      uint8_t c; // byte size variable to hold byte read from cartidge rom

      cartridgefile.read((char *)&c, 1); // read byte from cartridge file
      cartridge[i] = c;                  // put cartridge byte into cartridge array at index 'i'
    }

    cartridgefile.close(); // close cartridge file

    bus->attachROM(cartridge, 0x400, 0x800); // attach rom cartridge to bus
    mcs48->reset();                          // reset cpu

    return true;
  }

  bool OnUserUpdate(float fElapsedTime) override
  {
    uint8_t ic = 0;

    // cout << "\x1B[2J\x1B[H"; // clear console

    intel8245->framestart();

    for (int i = 0; i < 6000; i++)
    {
      mcs48->clock();     // clock cpu
      intel8245->clock(); // clock gpu
      keyboard->clock();  // clock keyboard

      ++ic;

      if (ic == 0x01)
      {
        //mcs48.interrupt(); // simulate interrupt
        mcs48->setF1(1);
      }
      else
      {
        if (ic == 0x20)
        {
          mcs48->setF1(0);
          ic = 0x00;
        }
      }

      // mcs48.disassemble();
      // mcs48.debug(); // output mcs-48 debug information
      // cout << endl;
      // bus.debug(); // output bus debug information
      // cout << endl;
      // intel8245.debug();
      // cout << endl;
    }

    // called once per frame
    for (int x = 0; x < ScreenWidth(); x++)
      for (int y = 0; y < ScreenHeight(); y++)
        Draw(x, y, olc::Pixel(rand() % 255, rand() % 255, rand() % 255));

    return true;
  }
};

int main()
{
  G7000 g7000;
  if (g7000.Construct(288, 200, 4, 4))
    g7000.Start();

  return 0;
}

// int main()
// {

//   uint8_t ic = 0;

//   while (1)
//   {
//     cout << "\x1B[2J\x1B[H"; // clear console

//     for (int i = 0; i < 2048; i++)
//     {
//       mcs48.clock();     // clock cpu
//       intel8245.clock(); // clock gpu
//       keyboard.clock();  // clock keyboard
//     }

//     // mcs48.disassemble();
//     mcs48.debug(); // output mcs-48 debug information
//     cout << endl;
//     bus.debug(); // output bus debug information
//     cout << endl;
//     intel8245.debug();
//     cout << endl;

//     ++ic;

//     if (ic == 0x01)
//     {
//       //mcs48.interrupt(); // simulate interrupt
//       mcs48.setF1(1);
//     }
//     else
//     {
//       if (ic == 0x02)
//       {
//         mcs48.setF1(0);
//         ic = 0x00;
//       }
//     }

//     this_thread::sleep_for(chrono::milliseconds(100)); // wait for 500 ms
//   }

//   return 0;
// }