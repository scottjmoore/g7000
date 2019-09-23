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
    F0 = 1 << 5,
    AC = 1 << 6,
    CY = 1 << 7,
  };

  // CPU registers

  uint8_t A = 0x00;     // 8 bit accumulator register
  uint8_t TC = 0x00;    // Timer counter
  uint8_t PSW = 0x0000; // Program status word
  uint16_t PC = 0x0000; // Program counter

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
  void decode();
  uint8_t readROM(uint16_t address);
  void writeROM(uint16_t address, uint8_t data);
  uint8_t readRAM(uint8_t address);
  void writeRAM(uint8_t address, uint8_t data);

  void debug();
  string decodedOpcode() { return decoded_opcode; }

  // Instruction set functions

  void ADD_A_R(uint8_t R);
  void ADD_A_data(uint8_t data);
  void ADDC_A_R(uint8_t R);
  void ADDC_A_data(uint8_t data);
  void ANL_A_R(uint8_t R);
  void ANL_A_data(uint8_t data);
  void ANL_BUS_data(uint8_t data);
  void ANL_P_data(uint8_t port, uint8_t data);
  void ANL_P_A(uint8_t port);

  void CALL(uint16_t address);
  void CLR_A();
  void CLR_C();
  void CLR_F0();
  void CLR_F1();
  void CPL_A();
  void CPL_C();
  void CPL_F0();
  void CPL_F1();

  void DA_A();
  void DEC_A();
  void DEC_R(uint8_t R);
  void DIS_I();
  void DIS_TCNTI();
  void DJNZ_R_address(uint8_t R, uint8_t address);

  void EN_I();
  void EN_TCNTI();
  void ENT0_CLK();

  void IN_A_P(uint8_t port);
  void INC_A();
  void INC_R(uint8_t R);

  void MOV_A_data(uint8_t data);

  void NOP();

  void SEL_RB0();
  void SEL_RB1();

  void XRL_A_data(uint8_t data);
};