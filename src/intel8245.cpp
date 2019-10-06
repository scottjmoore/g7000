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
  // clear grid

  for (int i = 0; i < 9; i++)
  {
    for (int j = 0; j < 10; j++)
    {
      grid[i][j] = 0x00;
    }
  }

  // clear sprites

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 8; j++)
    {
      sprite[i][j] = 0x00;
    }
  }

  uint8_t address = 0x80;

  // decode sprites

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 8; j++)
    {
      sprite[i][j] = VRAM[address];
      address++;
    }
  }

  // decode grid registers

  address = 0xc0;

  for (int j = 0; j < 9; j++)
  {
    for (int i = 0; i < 8; i++)
    {
      if (VRAM[address] & (1 << i))
        grid[i][j] = 0x0f;
      else
        grid[i][j] = 0x00;
    }

    address++;
  }

  address = 0xd0;

  for (int i = 0; i < 9; i++)
  {
    if (VRAM[address] & 1)
      grid[i][9] = 0x0f;
    else
      grid[i][9] = 0x00;

    address++;
  }

  address = 0xe0;

  for (int j = 0; j < 10; j++)
  {
    for (int i = 0; i < 8; i++)
    {
      if (VRAM[address] & (1 << i))
      {
        grid[i][j] |= 0xf0;
      }
    }

    address++;
  }
}

void INTEL8245::write(uint8_t address, uint8_t data)
{
  VRAM[address] = data;
}

uint8_t INTEL8245::read(uint8_t address)
{
  return VRAM[address];
}

void INTEL8245::spritelineout(uint8_t byte)
{
  for (int i = 0; i < 8; i++)
  {
    if (byte & (1 << (7 - i)))
      cout << "@";
    else
      cout << " ";
  }
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

  cout << endl;

  for (int i = 0; i < 12; i++) // decode single characters
  {
    uint8_t y = VRAM[0x10 + (i << 2)];
    uint16_t x = VRAM[0x11 + (i << 2)];
    uint8_t cp = VRAM[0x12 + (i << 2)];
    uint8_t clr = VRAM[0x13 + (i << 2)];
    uint8_t chr = ((y >> 1) + cp) >> 3;

    x |= (clr & 0b00000001) << 8;
    clr = (clr & 0b00001110) >> 1;

    if (x < 248)
      cout << dec << unsigned(x) << ", " << unsigned(y) << ", " << charset[chr & 0b00111111] << ", " << colors[clr] << endl;
  }

  for (int i = 0; i < 9; i++)
  {
    for (int j = 0; j < 10; j++)
    {
      switch (grid[i][j])
      {
      case 0x00:
        cout << ".";
        break;
      case 0x0f:
        cout << "-";
        break;
      case 0xf0:
        cout << "|";
        break;
      case 0xff:
        cout << "+";
        break;
      default:
        cout << " ";
      }
    }

    if (i < 8)
    {
      cout << " \t";
      spritelineout(sprite[0][i]);
      cout << " \t";
      spritelineout(sprite[1][i]);
      cout << " \t";
      spritelineout(sprite[2][i]);
      cout << " \t";
      spritelineout(sprite[3][i]);
    }

    cout << endl;
  }

  cout.flags(oldFlags);
  cout.precision(oldPrec);
  cout.fill(oldFill);
}
