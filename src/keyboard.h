#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <bitset>

#include "bus.h"

using namespace std;

class KEYBOARD
{
private:
  ::BUS *bus;
  uint8_t *PORT1;
  uint8_t *PORT2;

  string key_names[6][8] =
      {
          {"0", "1", "2", "3", "4", "5", "6", "7"},
          {"8", "9", "(NA)", "(NA)", "SPC", "?", "L", "P"},
          {"+", "W", "E", "R", "T", "U", "I", "O"},
          {"Q", "S", "D", "F", "G", "H", "J", "K"},
          {"A", "Z", "X", "C", "V", "B", "M", "."},
          {"-", "*", "/", "=", "YES", "NO", "CLR", "ENT"},
  };

  bool key_pressed[6][8] = {{false, true, false, false, false, false, false, false}, {false, false, false, false, false, false, false, false}, {false, false, false, false, false, false, false, false}, {false, false, false, false, false, false, false, false}, {false, false, false, false, false, false, false, false}, {false, false, false, false, false, false, false, false}};

public:
  KEYBOARD(::BUS *bus);
  ~KEYBOARD();

  void clock();
};
