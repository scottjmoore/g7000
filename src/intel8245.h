#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <bitset>

#include "bus.h"

using namespace std;

class INTEL8245
{
private:
  ::BUS *bus;

  uint8_t HSYNC;
  uint8_t VBL;
  uint8_t HBL;
  uint8_t R;
  uint8_t G;
  uint8_t B;
  uint8_t L;
  uint8_t SND;

  uint8_t *VRAM;

  string charset[64] = {
      "0", "1", "2", "3", "4", "5", "6", "7",
      "8", "9", ":", "$", " ", "?", "L", "P",
      "+", "W", "E", "R", "T", "U", "I", "O",
      "Q", "S", "D", "F", "G", "H", "J", "K",
      "A", "Z", "X", "C", "V", "B", "M", ".",
      "-", "x", "÷", "=", "Y", "N", "/", "block",
      "10", "ball", "man right", "man right walk", "man left walk", "man left", "arrow right", "tree",
      "slope left", "slope right", "man forward", "\\", "ship 1", "plane", "ship 2", "ship 3"};

public:
  INTEL8245(::BUS *_bus);
  ~INTEL8245();

  void clock();
  uint32_t rgbl() { return (R << 24) | (G << 16) | (B << 8) | L; };

  void write(uint8_t address, uint8_t data);
  uint8_t read(uint8_t address);

  void debug();
};