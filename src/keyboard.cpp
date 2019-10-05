#include "bus.h"
#include "keyboard.h"

KEYBOARD::KEYBOARD(::BUS *bus)
{
  this->bus = bus;

  PORT1 = bus->connectTo8BitBus(0);
  PORT2 = bus->connectTo8BitBus(1);
}

KEYBOARD::~KEYBOARD()
{
}

void KEYBOARD::clock()
{
  if (!((*PORT1) & 0b00000100))
  {
    uint8_t row = (*PORT2) & 0b00000111;

    for (uint8_t col = 0; col < 8; col++)
    {
      if (key_pressed[row][col])
      {
        (*PORT2) &= 0b00001111;
        (*PORT2) |= ((col & 0b00000111) << 5);
        break;
      }
    }
  }
}