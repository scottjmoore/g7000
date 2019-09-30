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
  uint8_t **_4bitbus;
  uint8_t **_8bitbus;
  uint16_t **_16bitbus;
  uint32_t **_32bitbus;

public:
  BUS();
  BUS(int _4bitbussize, int _8bitbussize, int _16bitbusize, int _32bitbussize);
  ~BUS();

  void AttachTo4BitBus(int index, uint8_t *source);
  void AttachTo8BitBus(int index, uint8_t *source);
  void AttachTo16BitBus(int index, uint16_t *source);
  void AttachTo32BitBus(int index, uint32_t *source);
};
