#include "bus.h"

BUS::BUS()
{
}

BUS::BUS(int __4bitbussize, int __8bitbussize, int __16bitbussize, int __32bitbussize)
{
  _4bitbussize = __4bitbussize;
  _8bitbussize = __8bitbussize;
  _16bitbussize = __16bitbussize;
  _32bitbussize = __32bitbussize;

  _4bitbus = new uint8_t *[_4bitbussize];
  _8bitbus = new uint8_t *[_8bitbussize];
  _16bitbus = new uint16_t *[_16bitbussize];
  _32bitbus = new uint32_t *[_32bitbussize];

  _4bitbusname = new string[_4bitbussize];
  _8bitbusname = new string[_8bitbussize];
  _16bitbusname = new string[_16bitbussize];
  _32bitbusname = new string[_32bitbussize];

  for (int i = 0; i < 256; i++)
  {
    xram[i] = 0xbb;
  }
}

BUS::~BUS()
{
  delete[] _4bitbus;
  delete[] _4bitbusname;
  delete[] _8bitbus;
  delete[] _8bitbusname;
  delete[] _16bitbus;
  delete[] _16bitbusname;
  delete[] _32bitbus;
  delete[] _32bitbusname;
}

void BUS::attachTo4BitBus(int index, uint8_t *source, string name)
{
  _4bitbus[index] = source;
  _4bitbusname[index] = name;
}

void BUS::attachTo8BitBus(int index, uint8_t *source, string name)
{
  _8bitbus[index] = source;
  _8bitbusname[index] = name;
}

void BUS::attachTo16BitBus(int index, uint16_t *source, string name)
{
  _16bitbus[index] = source;
  _16bitbusname[index] = name;
}

void BUS::attachTo32BitBus(int index, uint32_t *source, string name)
{
  _32bitbus[index] = source;
  _32bitbusname[index] = name;
}

uint8_t *BUS::connectTo4BitBus(int index) { return _4bitbus[index]; }

uint8_t *BUS::connectTo8BitBus(int index) { return _8bitbus[index]; }

uint16_t *BUS::connectTo16BitBus(int index) { return _16bitbus[index]; }

uint32_t *BUS::connectTo32BitBus(int index) { return _32bitbus[index]; }

void BUS::attachROM(uint8_t *data, uint16_t address_start, uint16_t length)
{
  rom = data;
  rom_address_start = address_start;
  rom_length = length;
}

void BUS::setVRAM(uint8_t *vram)
{
  this->vram = vram;
}

uint8_t BUS::read_8_16(uint16_t address)
{
  if ((address < 0xff) && (*_8bitbus[0] & 0b01001000) == 0b00000000)
  {
    return vram[address & 0xff];
  }

  if ((address < 0x7f) && (*_8bitbus[0] & 0b00010000) == 0b00000000)
  {
    return xram[address & 0xff];
  }

  if ((address >= rom_address_start) && (address <= (rom_address_start + rom_length)))
  {
    return rom[address - rom_address_start];
  }
  else
  {
    return 0x00;
  }
}

void BUS::write_8_16(uint16_t address, uint8_t data)
{
  if ((*_8bitbus[0] & 0b00001000) == 0b00000000)
  {
    vram[address & 0xff] = data;
  }

  if ((*_8bitbus[0] & 0b01010000) == 0b00000000)
  {
    xram[address & 0xff] = data;
  }
}

void BUS::debug()
{
  int i;

  if (_4bitbussize)
  {
    for (i = 0; i < _4bitbussize; i++)
    {
      cout << _4bitbusname[i] << " \t: " << setfill('0') << dec << unsigned(*_4bitbus[i]) << " \t" << hex << "0x" << setw(1) << unsigned(*_4bitbus[i]) << " \t0b" << bitset<4>(*_4bitbus[i]) << endl;
    }
  }

  if (_8bitbussize)
  {
    for (i = 0; i < _8bitbussize; i++)
    {
      cout << _8bitbusname[i] << " \t: " << setfill('0') << dec << unsigned(*_8bitbus[i]) << " \t" << hex << "0x" << setw(2) << unsigned(*_8bitbus[i]) << " \t0b" << bitset<8>(*_8bitbus[i]) << endl;
    }
  }

  if (_16bitbussize)
  {
    for (i = 0; i < _16bitbussize; i++)
    {
      cout << _16bitbusname[i] << " \t: " << setfill('0') << dec << unsigned(*_16bitbus[i]) << " \t" << hex << "0x" << setw(4) << unsigned(*_16bitbus[i]) << " \t0b" << bitset<16>(*_16bitbus[i]) << endl;
    }
  }

  if (_32bitbussize)
  {
    for (i = 0; i < _32bitbussize; i++)
    {
      cout << _32bitbusname[i] << " \t: " << setfill('0') << dec << unsigned(*_32bitbus[i]) << " \t" << hex << "0x" << setw(8) << unsigned(*_32bitbus[i]) << " \t0b" << bitset<32>(*_32bitbus[i]) << endl;
    }
  }

  cout << endl;

  for (int i = 0; i < 256; i++)
  {
    cout << hex << setw(2) << unsigned(xram[i]) << "  ";

    if ((i % 16) == 15)
    {
      cout << endl;
    }
  }
}