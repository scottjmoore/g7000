#pragma once
//#include "Bus.h"
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <bitset>

using namespace std;

class MCS48
{
private:
  // Program status word bits

  enum PSW_BITS
  {
    S0 = 1 << 0,
    S1 = 1 << 1,
    S2 = 1 << 2,
    BS = 1 << 4,
    F1 = 1 << 4,
    F0 = 1 << 5,
    AC = 1 << 6,
    CY = 1 << 7,
  };

  // CPU registers

  uint8_t A = 0x00;        // 8 bit accumulator register
  uint8_t TC = 0x00;       // Timer counter
  uint8_t PSW = 0b0000100; // Program status word
  uint16_t PC = 0x0000;    // Program counter

  // CPU storage

  uint8_t RAM[64];
  uint8_t ROM[1024];

  // Emulator internal state

  uint8_t fetched;
  string decoded_opcode;

public:
  MCS48();
  ~MCS48();

  void reset();
  void clock();
  void fetch();
  uint8_t decode();

  uint8_t readROM(uint16_t address);
  void writeROM(uint16_t address, uint8_t data);
  uint8_t readRAM(uint8_t address);
  void writeRAM(uint8_t address, uint8_t data);
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t data);

  void debug();
  string decodedOpcode() { return decoded_opcode; }

  // Instruction set functions

  uint8_t ADD_A_R(uint8_t R);
  uint8_t ADD_A_data(uint8_t data);
  uint8_t ADDC_A_R(uint8_t R);
  uint8_t ADDC_A_data(uint8_t data);
  uint8_t ANL_A_R(uint8_t R);
  uint8_t ANL_A_data(uint8_t data);
  uint8_t ANL_BUS_data(uint8_t data);
  uint8_t ANL_P_data(uint8_t port, uint8_t data);
  uint8_t ANL_P_A(uint8_t port);

  uint8_t CALL(uint16_t address);
  uint8_t CLR_A();
  uint8_t CLR_C();
  uint8_t CLR_F0();
  uint8_t CLR_F1();
  uint8_t CPL_A();
  uint8_t CPL_C();
  uint8_t CPL_F0();
  uint8_t CPL_F1();

  uint8_t DA_A();
  uint8_t DEC_A();
  uint8_t DEC_R(uint8_t R);
  uint8_t DIS_I();
  uint8_t DIS_TCNTI();
  uint8_t DJNZ_R_address(uint8_t R, uint8_t address);

  uint8_t EN_I();
  uint8_t EN_TCNTI();
  uint8_t ENT0_CLK();

  uint8_t IN_A_P(uint8_t port);
  uint8_t INC_A();
  uint8_t INC_R(uint8_t R);

  uint8_t JMP(uint16_t address);

  uint8_t MOV_A_data(uint8_t data);
  uint8_t MOV_A_PSW();
  uint8_t MOV_A_RR(uint8_t reg);
  uint8_t MOV_PSW_A();
  uint8_t MOV_RR_A(uint8_t reg);
  uint8_t MOV_RR_data(uint8_t reg, uint8_t data);

  uint8_t NOP();

  uint8_t SEL_RB0();
  uint8_t SEL_RB1();

  uint8_t XRL_A_data(uint8_t data);
};