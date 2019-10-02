#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <bitset>

#include "bus.h"

using namespace std;

class MCS48
{
public:
  enum CPUTYPE
  {
    CPU8021,
    CPU8048,
    CPU8049,
  };

private:
  // Program status word bits

  enum PSW_BITS
  {
    S0 = 1 << 0, // bit 0 of stack register
    S1 = 1 << 1, // bit 1 of stack register
    S2 = 1 << 2, // bit 2 of stack register
    BS = 1 << 4, // bank switch bit
    F0 = 1 << 5, // f0 flag bit
    AC = 1 << 6, // auxillary carry bit
    CY = 1 << 7, // carry bit
  };

  enum TC_MODE
  {
    STOPPED,
    TIMER,
    COUNTER
  };

  // CPU registers

  uint8_t A = 0x00;        // 8 bit accumulator register
  uint8_t TC = 0x00;       // Timer/counter
  uint8_t PSW = 0b0000100; // Program status word
  uint16_t PC = 0x0000;    // Program counter

  uint8_t I = 0b0;      // Interrupt input
  uint8_t T0 = 0b0;     // Test 0 input
  uint8_t T1 = 0b0;     // Test 1 input
  uint8_t TF = 0b0;     // Timer flag overflow
  uint8_t F1 = 0b0;     // Flag 1 status bit (not stored in PSW)
  uint8_t DBF = 0b0;    // Memory bank switch
  uint8_t IE = 0b0;     // Interrupt enabled
  uint8_t TCNTIE = 0b0; // Timer/counter interrupt enabled

  uint8_t PORT0 = 0x00; // 8 bit port 0 bus (8021 only?)
  uint8_t PORT1 = 0x00; // 8 bit port 1 bus (P10 - P17)
  uint8_t PORT2 = 0x00; // 8 bit port 2 bus (P20 - P27)
  uint8_t BUS = 0x00;   // 8 bit bus (D0 - D7)
  uint8_t PORT4 = 0x0;  // 4 bit expander port 4 bus (8243)
  uint8_t PORT5 = 0x0;  // 4 bit expander port 5 bus (8243)
  uint8_t PORT6 = 0x0;  // 4 bit expander port 6 bus (8243)
  uint8_t PORT7 = 0x0;  // 4 bit expander port 7 bus (8243)

  // CPU type

  CPUTYPE cputype;

  // CPU storage

  uint16_t data_memory_size;    // size of internal data memory storage
  uint16_t program_memory_size; // size of internal program memory storage

  uint8_t *RAM; // pointer to internal data memory storage
  uint8_t *ROM; // pointer to internal program memory storage

  uint8_t STACK = 8; // offset to start of stack memory in the data memory

  // Emulator internal state

  int cycles;
  uint8_t fetched;
  string decoded_opcode;
  TC_MODE tc_mode = TC_MODE::STOPPED; // Timer/Counter mode
  uint8_t t_prescaler = 0x00;         // Timer pre scaler counter (32 cycles per timer tick)

  ::BUS bus;

public:
  MCS48(CPUTYPE _cputype, ::BUS &_bus);
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

  uint8_t readExternalRAM(uint8_t address);
  void writeExternalRAM(uint8_t address, uint8_t data);

  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t data);

  string decodedOpcode() { return decoded_opcode; }

  uint8_t ADD_A_R(uint8_t reg);
  uint8_t ADD_A_RC(uint8_t reg);
  uint8_t ADD_A_data(uint8_t data);

  uint8_t ADDC_A_R(uint8_t reg);
  uint8_t ADDC_A_RC(uint8_t reg);
  uint8_t ADDC_A_data(uint8_t data);

  uint8_t ANL_A_R(uint8_t reg);
  uint8_t ANL_A_RC(uint8_t reg);
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
  uint8_t DEC_R(uint8_t reg);

  uint8_t DIS_I();
  uint8_t DIS_TCNTI();

  uint8_t DJNZ_R_address(uint8_t reg, uint8_t address);

  uint8_t EN_I();
  uint8_t EN_TCNTI();
  uint8_t ENT0_CLK();

  uint8_t IN_A_P(uint8_t port);

  uint8_t INC_A();
  uint8_t INC_R(uint8_t reg);
  uint8_t INC_RC(uint8_t reg);

  uint8_t IN_A_P0();
  uint8_t INS_A_BUS();

  uint8_t JBB(uint8_t bit, uint8_t address);
  uint8_t JC(uint8_t address);
  uint8_t JF0(uint8_t address);
  uint8_t JF1(uint8_t address);
  uint8_t JMP(uint16_t address);
  uint8_t JMPP_AC();
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
  uint8_t MOVD_A_P(uint8_t port);
  uint8_t MOVD_P_A(uint8_t port);
  uint8_t MOVP_A_AC();
  uint8_t MOVP3_A_AC();
  uint8_t MOVX_A_RC(uint8_t reg);
  uint8_t MOVX_RC_A(uint8_t reg);

  uint8_t NOP();

  uint8_t ORL_A_R(uint8_t reg);
  uint8_t ORL_A_RC(uint8_t reg);
  uint8_t ORL_A_data(uint8_t data);
  uint8_t ORL_BUS_data(uint8_t data);
  uint8_t ORL_P_data(uint8_t port, uint8_t data);
  uint8_t ORLD_P_A(uint8_t port);

  uint8_t OUTL_P0_A(); // 8021 only
  uint8_t OUTL_BUS_A();
  uint8_t OUTL_P_A(uint8_t port);

  uint8_t RET();
  uint8_t RETR();

  uint8_t RL_A();
  uint8_t RLC_A();
  uint8_t RR_A();
  uint8_t RRC_A();

  uint8_t SEL_MB0();
  uint8_t SEL_MB1();
  uint8_t SEL_RB0();
  uint8_t SEL_RB1();

  uint8_t STOP_TCNT();
  uint8_t STRT_CNT();
  uint8_t STRT_T();

  uint8_t SWAP_A();

  uint8_t XCH_A_R(uint8_t reg);
  uint8_t XCH_A_RC(uint8_t reg);
  uint8_t XCHD_A_RC(uint8_t reg);

  uint8_t XRL_A_R(uint8_t reg);
  uint8_t XRL_A_RC(uint8_t reg);
  uint8_t XRL_A_data(uint8_t data);

  void debug();
};