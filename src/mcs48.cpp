#include "mcs48.h"

MCS48::MCS48()
{
  for (int i = 0; i < 64; i++)
  {
    RAM[i] = 0x00;
  }

  for (int i = 0; i < 1024; i++)
  {
    ROM[i] = 0x00;
  }
}

MCS48::~MCS48()
{
}

void MCS48::reset()
{
  PC = 0x0000;
  PSW = 0x00;
}

void MCS48::clock()
{
  fetch();
  decode();
}

void MCS48::fetch()
{
  fetched = readROM(PC++);

  if (PC >= 0x0800)
  {
    PC = 0x0000;
  }
}

void MCS48::decode()
{
  ostringstream stringout;

  switch (fetched)
  {
  case 0b00000011: // ADD A, #data
    stringout << "ADD A, ";
    fetch(); // fetch immediate data to add to accumulator
    stringout << "#0" << hex << setw(2) << unsigned(fetched) << "H";
    ADD_A_data(fetched); // execute instruction
    break;
  case 0b00100111: // CLR A
    stringout << "CLR A";
    CLR_A(); // execute instruction
    break;
  case 0b10010111: // CLR C
    stringout << "CLR C";
    CLR_C(); // execute instruction
    break;
  case 0b00110111: // CPL A
    stringout << "CPL A";
    CPL_A(); // execute instruction
    break;
  case 0b10100111: // CPL C
    stringout << "CPL C";
    CPL_C(); // execute instruction
    break;
  case 0b00100011: // MOV A, #data
    stringout << "MOV A, ";
    fetch(); // fetch immediate data to add to accumulator
    stringout << "#0" << hex << setw(2) << unsigned(fetched) << "H";
    MOV_A_data(fetched); // execute instruction
    break;
  case 0b00000000: // NOP
    stringout << "NOP";
    NOP(); // execute instruction
    break;
  case 0b11000101: // SEL RB0
    stringout << "SEL RB0";
    SEL_RB0(); // execute instruction
    break;
  case 0b11010101: // SEL RB1
    stringout << "SEL RB1";
    SEL_RB1(); // execute instruction
    break;
  case 0b11010011: // XLR A, #data
    stringout << "XLR A, ";
    fetch(); // fetch immediate data to add to accumulator
    stringout << "#0" << hex << setw(2) << unsigned(fetched) << "H";
    XRL_A_data(fetched); // execute instruction
    break;
  }

  decoded_opcode = stringout.str();
}

uint8_t MCS48::readROM(uint16_t address)
{
  uint16_t a = address & 0x07ff;

  return ROM[a];
}

void MCS48::writeROM(uint16_t address, uint8_t data)
{
  uint16_t a = address & 0x07ff;

  ROM[a] = data;
}

uint8_t MCS48::readRAM(uint8_t address)
{
  uint8_t a = address & 0x3f;

  return RAM[a];
}

void MCS48::writeRAM(uint8_t address, uint8_t data)
{
  uint16_t a = address & 0x3f;

  RAM[a] = data;
}

// instructions

void MCS48::ADD_A_data(uint8_t data)
{
  uint8_t OA = A;

  A = A + data;

  if (A < OA) // overflow, so set carry bit (CY)
  {
    PSW ^= PSW_BITS::CY;
  }
}

void MCS48::CLR_A()
{
  A = 0x00;
}

void MCS48::CLR_C()
{
  PSW = PSW & ~PSW_BITS::CY;
}

void MCS48::CPL_A()
{
  A = ~A;
}

void MCS48::CPL_C()
{
  PSW ^= PSW_BITS::CY;
}

void MCS48::MOV_A_data(uint8_t data)
{
  A = data;
}

void MCS48::NOP()
{
  // do nothing
}

void MCS48::SEL_RB0()
{
  PSW = PSW & ~PSW_BITS::BS;
}

void MCS48::SEL_RB1()
{
  PSW = PSW | PSW_BITS::BS;
}

void MCS48::XRL_A_data(uint8_t data)
{
  A ^= data;
}

// debug functions

void MCS48::debug()
{
  ios_base::fmtflags oldFlags = cout.flags();
  streamsize oldPrec = cout.precision();
  char oldFill = cout.fill();

  cout << internal << setfill('0');

  cout << "PC  : " << hex << "0x" << setw(4) << PC << " \t" << decoded_opcode << endl
       << endl;

  cout << "A   : " << hex << "0x" << setw(2) << unsigned(A) << " \t0b" << bitset<8>(A) << endl;
  cout << "TC  : " << hex << "0x" << setw(2) << unsigned(TC) << " \t0b" << bitset<8>(TC) << endl;
  cout << "PSW : " << hex << "0x" << setw(2) << unsigned(PSW) << " \t0b" << bitset<8>(PSW) << endl;

  cout << endl;

  cout.flags(oldFlags);
  cout.precision(oldPrec);
  cout.fill(oldFill);
}