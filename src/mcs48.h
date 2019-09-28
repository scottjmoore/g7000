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

  uint8_t A = 0x00;        // 8 bit accumulator register
  uint8_t TC = 0x00;       // Timer counter
  uint8_t PSW = 0b0000100; // Program status word
  uint16_t PC = 0x0000;    // Program counter

  uint8_t F1 = 0b0;     // Flag 1 status bit (not stored in PSW)
  uint8_t PORT1 = 0x00; // 8 bit port 1 bus (P10 - P17)
  uint8_t PORT2 = 0x00; // 8 bit port 2 bus (P20 - P27)
  uint8_t BUS = 0x00;   // 8 bit bus (D0 - D7)

  // CPU storage

  uint16_t data_memory_size;
  uint16_t program_memory_size;

  uint8_t *RAM;
  uint8_t *ROM;
  uint8_t STACK = 8;

  // Emulator internal state

  uint8_t fetched;
  string decoded_opcode;

public:
  MCS48();
  ~MCS48();

  void reset();
  void interrupt();
  void timer_interrupt();
  void clock();
  void fetch();
  void push_pc_psw();
  void pop_pc_psw();
  void pop_pc();
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

  /*
    ADD A,R Add Register Contents to Accumulator
      |0110|1rrr|
      The contents of register R are added to the accumulator. Carry is affected.
      (A) <- (A) + (R) R = 0-7
      Example:
        ADDREG: ADD A,R6 R=0-7 ;ADD REG 6 CONTENTS TO ACC
  */

  uint8_t ADD_A_R(uint8_t R);

  /*
    ADD A,RC Add Data Memory Contents to Accumulator
      |0110|000r|
      The contents of the resident data memory location addressed by register Or' bits 0-5*are added to the accumulator. Carry is affected.
      (A) <- (A) + ((R)) R = 0-1
      Example:
        ADDM: MOV R0, #01FH   ; MOVE '1F' HEX TO REG 0
              ADD A, @R0      ;ADD VALUE OF LOCATION '1F' TO ACC
  */

  uint8_t ADD_A_RC(uint8_t R);
  uint8_t ADD_A_data(uint8_t data);
  uint8_t ADDC_A_R(uint8_t R);
  uint8_t ADDC_A_RC(uint8_t R);
  uint8_t ADDC_A_data(uint8_t data);
  uint8_t ANL_A_R(uint8_t R);
  uint8_t ANL_A_RC(uint8_t R);
  uint8_t ANL_A_data(uint8_t data);
  uint8_t ANL_BUS_data(uint8_t data);
  uint8_t ANL_P_data(uint8_t port, uint8_t data);
  uint8_t ANLD_P_A(uint8_t port);

  uint8_t CALL(uint16_t address);
  uint8_t CLR_A();
  uint8_t CLR_C();
  uint8_t CLR_F1();
  uint8_t CLR_F0();
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
  uint8_t IN_A_PO();
  uint8_t INS_A_BUS();

  uint8_t JBb(uint8_t bit, uint8_t address);
  uint8_t JC(uint8_t address);
  uint8_t JF0(uint8_t address);
  uint8_t JF1(uint8_t address);
  uint8_t JMP(uint16_t address);
  uint8_t JMPP();
  uint8_t JNC(uint8_t address);
  uint8_t JNI(uint8_t address);
  uint8_t JNT0(uint8_t address);
  uint8_t JNT1(uint8_t address);
  uint8_t JNZ(uint8_t address);
  uint8_t JTF(uint8_t address);
  uint8_t JT0(uint8_t address);
  uint8_t JT1(uint8_t address);
  uint8_t JZ(uint8_t address);

  uint8_t MOV_A_data(uint8_t data);
  uint8_t MOV_A_PSW();
  uint8_t MOV_A_R(uint8_t reg);
  uint8_t MOV_A_RC(uint8_t reg);
  uint8_t MOV_A_T();
  uint8_t MOV_PSW_A();
  uint8_t MOV_R_A(uint8_t reg);
  uint8_t MOV_R_data(uint8_t reg, uint8_t data);
  uint8_t MOV_RC_A(uint8_t reg);
  uint8_t MOV_RC_data(uint8_t reg, uint8_t data);
  uint8_t MOV_T_A();
  uint8_t MOVD_A_P();
  uint8_t MOVD_P_A();
  uint8_t MOVP_A_AC();
  uint8_t MOVP3_A_AC();
  uint8_t MOVX_A_RC(uint8_t reg);
  uint8_t MOVX_RC_A(uint8_t reg);

  uint8_t NOP();

  uint8_t RET();
  uint8_t RETR();

  uint8_t SEL_RB0();
  uint8_t SEL_RB1();

  uint8_t XRL_A_data(uint8_t data);
};