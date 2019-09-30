#include "bus.h"

BUS::BUS()
{
}

BUS::BUS(int _4bitbussize, int _8bitbussize, int _16bitbussize, int _32bitbussize)
{
  _4bitbus = new uint8_t *[_4bitbussize];
  _8bitbus = new uint8_t *[_8bitbussize];
  _16bitbus = new uint16_t *[_16bitbussize];
  _32bitbus = new uint32_t *[_32bitbussize];
}

BUS::~BUS()
{
}

void BUS::AttachTo4BitBus(int index, uint8_t *source)
{
  _4bitbus[index] = source;
}

void BUS::AttachTo8BitBus(int index, uint8_t *source)
{
  _8bitbus[index] = source;
}

void BUS::AttachTo16BitBus(int index, uint16_t *source)
{
  _16bitbus[index] = source;
}

void BUS::AttachTo32BitBus(int index, uint32_t *source)
{
  _32bitbus[index] = source;
}

void Debug()
{
}