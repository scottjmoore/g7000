#include "bus.h"
#include "intel8245.h"

INTEL8245::INTEL8245(::BUS *_bus)
{
  bus = _bus;

  VRAM = new uint8_t[256];

  for (int i = 0; i < 256; i++)
    VRAM[i] = 0xaa;

  bus->setVRAM(VRAM);
}

INTEL8245::~INTEL8245()
{
  delete[] VRAM;
}

void INTEL8245::clock()
{
}

void INTEL8245::write(uint8_t address, uint8_t data)
{
  VRAM[address] = data;
}

uint8_t INTEL8245::read(uint8_t address)
{
  return VRAM[address];
}

void INTEL8245::debug()
{
  ios_base::fmtflags oldFlags = cout.flags();
  streamsize oldPrec = cout.precision();
  char oldFill = cout.fill();

  cout << internal << setfill('0');

  for (int i = 0; i < 256; i++)
  {
    cout << hex << setw(2) << unsigned(VRAM[i]) << "  ";

    if ((i % 16) == 15)
    {
      cout << endl;
    }
  }

  for (int i = 0; i < 12; i++) // decode single characters
  {
    uint8_t y = VRAM[0x10 + (i << 2)];
    uint8_t x = VRAM[0x11 + (i << 2)];
    uint8_t cp = VRAM[0x12 + (i << 2)];
    uint8_t clr = VRAM[0x13 + (i << 2)];
    uint8_t chr = ((y >> 1) + cp) >> 3;

    cout << dec << unsigned(x) << ", " << unsigned(y) << " " << charset[chr & 0b00111111] << endl;
  }

  cout.flags(oldFlags);
  cout.precision(oldPrec);
  cout.fill(oldFill);
}
