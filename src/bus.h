#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <bitset>

using namespace std;

class BUS
{
private:
  int _4bitbussize, _8bitbussize, _16bitbussize, _32bitbussize;

  uint8_t **_4bitbus;
  uint8_t **_8bitbus;
  uint16_t **_16bitbus;
  uint32_t **_32bitbus;

  string *_4bitbusname;
  string *_8bitbusname;
  string *_16bitbusname;
  string *_32bitbusname;

  uint8_t *rom;
  uint16_t rom_address_start;
  uint16_t rom_length;

  uint8_t *vram;
  uint8_t xram[256];

  void *VRAMwrite;

public:
  BUS();
  BUS(int _4bitbussize, int _8bitbussize, int _16bitbusize, int _32bitbussize);
  ~BUS();

  void attachTo4BitBus(int index, uint8_t *source, string name);
  void attachTo8BitBus(int index, uint8_t *source, string name);
  void attachTo16BitBus(int index, uint16_t *source, string name);
  void attachTo32BitBus(int index, uint32_t *source, string name);

  uint8_t *connectTo4BitBus(int index);
  uint8_t *connectTo8BitBus(int index);
  uint16_t *connectTo16BitBus(int index);
  uint32_t *connectTo32BitBus(int index);

  void attachROM(uint8_t *data, uint16_t address_start, uint16_t length);

  void setVRAM(uint8_t *vram);

  uint8_t read_8_16(uint16_t address);
  void write_8_16(uint16_t address, uint8_t data);

  void debug();
};
