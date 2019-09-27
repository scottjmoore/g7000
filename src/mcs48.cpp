#include "mcs48.h"

MCS48::MCS48()
{
  data_memory_size = 64;
  program_memory_size = 1024;

  RAM = new uint8_t[data_memory_size];
  ROM = new uint8_t[program_memory_size];

  for (int i = 0; i < data_memory_size; i++)
  {
    RAM[i] = 0x00;
  }

  for (int i = 0; i < program_memory_size; i++)
  {
    ROM[i] = 0x00;
  }

  PSW = 0b00001000;
}

MCS48::~MCS48()
{
  delete[] ROM;
  delete[] RAM;
}

void MCS48::reset()
{
  PC = 0x0000;
  PSW = 0b00001000;
}

void MCS48::clock()
{
  uint8_t cycles;

  fetch();
  cycles = decode();
}

void MCS48::fetch()
{
  fetched = readROM(PC++);

  if (PC >= 0x0800)
  {
    PC = 0x0000;
  }
}

uint8_t MCS48::decode()
{
  uint8_t cycles = 0x00;
  ostringstream stringout;

  stringout << setfill('0') << hex << setw(4) << PC - 1 << " : ";
  stringout << setfill('0') << hex << setw(2) << unsigned(fetched) << " ";

  switch (fetched)
  {
  case 0b00000011: // ADD A, #data
    fetch();       // fetch immediate data to add to accumulator
    stringout << setfill('0') << hex << setw(2) << unsigned(fetched) << " \t\t";
    stringout << "ADD A, ";
    stringout << "#" << setfill('0') << hex << setw(2) << unsigned(fetched) << "H";
    cycles = ADD_A_data(fetched); // execute instruction
    break;
  case 0b00100111: // CLR A
    stringout << "   \t\t";
    stringout << "CLR A";
    cycles = CLR_A(); // execute instruction
    break;
  case 0b10010111: // CLR C
    stringout << "   \t\t";
    stringout << "CLR C";
    cycles = CLR_C(); // execute instruction
    break;
  case 0b00110111: // CPL A
    stringout << "   \t\t";
    stringout << "CPL A";
    cycles = CPL_A(); // execute instruction
    break;
  case 0b10100111: // CPL C
    stringout << "   \t\t";
    stringout << "CPL C";
    cycles = CPL_C(); // execute instruction
    break;
  case 0b00010111: // CPL C
    stringout << "   \t\t";
    stringout << "INC A";
    cycles = INC_A(); // execute instruction
    break;
  case 0b00100011: // MOV A, #data
    fetch();       // fetch immediate data to add to accumulator
    stringout << setfill('0') << hex << setw(2) << unsigned(fetched) << " \t\t";
    stringout << "MOV A, ";
    stringout << "#" << setfill('0') << hex << setw(2) << unsigned(fetched) << "H";
    cycles = MOV_A_data(fetched); // execute instruction
    break;
  case 0b00000000: // NOP
    stringout << "   \t\t";
    stringout << "NOP";
    cycles = NOP(); // execute instruction
    break;
  case 0b11000101: // SEL RB0
    stringout << "   \t\t";
    stringout << "SEL RB0";
    cycles = SEL_RB0(); // execute instruction
    break;
  case 0b11010101: // SEL RB1
    stringout << "   \t\t";
    stringout << "SEL RB1";
    cycles = SEL_RB1(); // execute instruction
    break;
  case 0b11010011: // XLR A, #data
    fetch();       // fetch immediate data to add to accumulator
    stringout << setfill('0') << hex << setw(2) << unsigned(fetched) << " \t\t";
    stringout << "XLR A, ";
    stringout << "#" << setfill('0') << hex << unsigned(fetched) << "H";
    cycles = XRL_A_data(fetched); // execute instruction
    break;

  default:
    bool decoded = false;

    if ((fetched & 0b00011111) == 0b00000100) // JMP address
    {
      uint16_t address = (fetched & 0b11100000) << 3;

      fetch();
      address |= fetched;
      stringout << setfill('0') << hex << setw(2) << unsigned(fetched) << " \t\t";
      stringout << "JMP ";
      stringout << setfill('0') << hex << setw(4) << address << "H";
      cycles = JMP(address);
      decoded = true;
      break;
    }

    if (!decoded)
    {
      uint8_t reg = fetched & 0b00000111;

      switch (fetched & 0b11111000)
      {
      case 0b01101000: // ADD A, R
        stringout << "   \t\t";
        stringout << "ADD A, R" << unsigned(reg);
        cycles = ADD_A_R(reg);
        decoded = true;
        break;
      case 0b01100000: // ADD A, RC
        stringout << "   \t\t";
        stringout << "ADD A, @R" << unsigned(reg);
        cycles = ADD_A_RC(reg);
        decoded = true;
        break;
      case 0b00011000: // INC R
        stringout << "   \t\t";
        stringout << "INC R" << unsigned(reg);
        writeRegister(reg, readRegister(reg) + 1); // TODO : Change to opcode function
        cycles = 1;
        decoded = true;
        break;
      case 0b10101000: // MOV Rr, A
        stringout << "   \t\t";
        stringout << "MOV R" << unsigned(reg) << ", A";
        writeRegister(reg, A);
        cycles = 1;
        decoded = true;
        break;

      case 0b11111000: // MOV A, Rr
        stringout << "   \t\t";
        stringout << "MOV A, R" << unsigned(reg);
        A = readRegister(reg);
        cycles = 1;
        decoded = true;
        break;

      case 0b10100000: // MOV @RC, A
        stringout << "   \t\t";
        stringout << "MOV @R" << unsigned(reg) << ", A";
        cycles = MOV_RC_A(reg);
        decoded = true;
        break;

      case 0b10111000: // MOV Rr, #data
        fetch();
        stringout << setfill('0') << hex << setw(2) << unsigned(fetched) << " \t\t";
        stringout << "MOV R" << unsigned(reg) << ", ";
        stringout << "#" << setfill('0') << hex << setw(2) << unsigned(fetched) << "H";
        writeRegister(reg, fetched);
        cycles = 1;
        decoded = true;
        break;
      }
    }
  }

  decoded_opcode = stringout.str();

  return cycles;
}

uint8_t MCS48::readROM(uint16_t address)
{
  uint16_t a = address & (program_memory_size - 1);

  return ROM[a];
}

void MCS48::writeROM(uint16_t address, uint8_t data)
{
  uint16_t a = address & (program_memory_size - 1);

  ROM[a] = data;
}

uint8_t MCS48::readRAM(uint8_t address)
{
  uint8_t a = address & (data_memory_size - 1);

  return RAM[a];
}

void MCS48::writeRAM(uint8_t address, uint8_t data)
{
  uint16_t a = address & (data_memory_size - 1);

  RAM[a] = data;
}

uint8_t MCS48::readRegister(uint8_t reg)
{
  uint8_t offset = 0x00;

  if (PSW & PSW_BITS::BS)
  {
    offset = 0x18;
  }

  return readRAM(offset + (reg & 0b00000111));
}

void MCS48::writeRegister(uint8_t reg, uint8_t data)
{
  uint8_t offset = 0x00;

  if (PSW & PSW_BITS::BS)
  {
    offset = 0x18;
  }

  writeRAM(offset + (reg & 0b00000111), data);
}

// instructions

/*
    ADD A,R Add Register Contents to Accumulator
      |0110|1rrr|
      The contents of register R are added to the accumulator. Carry is affected.
      (A) <- (A) + (R) R = 0-7
      Example:
        ADDREG: ADD A,R6  ; ADD REG 6 CONTENTS TO ACC
  */

uint8_t MCS48::ADD_A_R(uint8_t R)
{
  uint8_t PA = A;

  A = A + readRegister(R);

  if (A < PA)
  {
    PSW |= PSW_BITS::CY;
  }
  else
  {
    PSW &= ~PSW_BITS::CY;
  }

  return 1;
}

/*
    ADD A,RC Add Data Memory Contents to Accumulator
      |0110|000r|
      The contents of the resident data memory location addressed by register Or' bits 0-5*are added to the accumulator. Carry is affected.
      (A) <- (A) + ((R)) R = 0-1
      Example:
        ADDM: MOV R0, #01FH   ; MOVE '1F' HEX TO REG 0
              ADD A, @R0      ;ADD VALUE OF LOCATION '1F' TO ACC
  */

uint8_t MCS48::ADD_A_RC(uint8_t R)
{
  uint8_t PA = A;

  A = A + readRAM(readRegister(R));

  if (A < PA)
  {
    PSW |= PSW_BITS::CY;
  }
  else
  {
    PSW &= ~PSW_BITS::CY;
  }

  return 1;
}

uint8_t MCS48::ADD_A_data(uint8_t data)
{
  uint8_t OA = A;

  A = A + data;

  if (A < OA) // overflow, so set carry bit (CY)
  {
    PSW ^= PSW_BITS::CY;
  }

  return 2;
}

uint8_t MCS48::ADDC_A_RC(uint8_t R)
{
  uint8_t PA = A;
  uint8_t C = PSW & PSW_BITS::CY ? 1 : 0;

  A = A + readRAM(readRegister(R)) + C;

  if (A < PA)
  {
    PSW |= PSW_BITS::CY;
  }
  else
  {
    PSW &= ~PSW_BITS::CY;
  }

  return 1;
}

uint8_t MCS48::ADDC_A_RC(uint8_t R)
{
  uint8_t PA = A;
  uint8_t C = PSW & PSW_BITS::CY ? 1 : 0;

  A = A + readRAM(readRegister(R)) + C;

  if (A < PA)
  {
    PSW |= PSW_BITS::CY;
  }
  else
  {
    PSW &= ~PSW_BITS::CY;
  }

  return 1;
}

uint8_t MCS48::ADDC_A_data(uint8_t data)
{
  uint8_t PA = A;
  uint8_t C = PSW & PSW_BITS::CY ? 1 : 0;

  A = A + data + C;

  if (A < PA) // overflow, so set carry bit (CY)
  {
    PSW |= PSW_BITS::CY;
  }
  else
  {
    PSW &= ~PSW_BITS::CY;
  }

  return 2;
}

uint8_t MCS48::ANL_A_R(uint8_t R)
{
  A = A & readRegister(R);

  return 1;
}

uint8_t MCS48::ANL_A_RC(uint8_t R)
{
  A = A & readRAM(readRegister(R));

  return 1;
}

uint8_t MCS48::ANL_A_data(uint8_t data)
{
  A = A & data;

  return 2;
}

uint8_t MCS48::CLR_A()
{
  A = 0x00;

  return 1;
}

uint8_t MCS48::CLR_C()
{
  PSW = PSW & ~PSW_BITS::CY;

  return 1;
}

uint8_t MCS48::CPL_A()
{
  A = ~A;

  return 1;
}

uint8_t MCS48::CPL_C()
{
  PSW ^= PSW_BITS::CY;

  return 1;
}

uint8_t MCS48::INC_A()
{
  A++;

  return 1;
}

uint8_t MCS48::JMP(uint16_t address)
{
  PC = address;

  return 2;
}

uint8_t MCS48::MOV_A_data(uint8_t data)
{
  A = data;

  return 2;
}

uint8_t MCS48::MOV_A_PSW()
{
  A = PSW;

  return 1;
}

uint8_t MCS48::MOV_A_R(uint8_t reg)
{
  A = readRegister(reg);

  return 1;
}

uint8_t MCS48::MOV_PSW_A()
{
  PSW = A;

  return 1;
}

uint8_t MCS48::MOV_R_A(uint8_t reg)
{
  writeRegister(reg, A);

  return 1;
}

uint8_t MCS48::MOV_R_data(uint8_t reg, uint8_t data)
{
  writeRegister(reg, data);

  return 2;
}

uint8_t MCS48::MOV_RC_A(uint8_t reg)
{
  writeRAM(readRegister(reg), A);

  return 2;
}

uint8_t MCS48::NOP()
{
  return 1;
}

uint8_t MCS48::SEL_RB0()
{
  PSW &= ~PSW_BITS::BS;

  return 1;
}

uint8_t MCS48::SEL_RB1()
{
  PSW |= PSW_BITS::BS;

  return 1;
}

uint8_t MCS48::XRL_A_data(uint8_t data)
{
  A ^= data;

  return 2;
}

// debug functions

void MCS48::debug()
{
  ios_base::fmtflags oldFlags = cout.flags();
  streamsize oldPrec = cout.precision();
  char oldFill = cout.fill();

  cout << internal << setfill('0');

  cout << decoded_opcode << endl
       << endl;

  cout << "PC  : " << hex << "0x" << setw(4) << PC << " \t" << endl;
  cout << "A   : " << dec << unsigned(A) << " \t" << hex << "0x" << setw(2) << unsigned(A) << " \t0b" << bitset<8>(A) << endl;
  cout << "TC  : " << dec << unsigned(TC) << " \t" << hex << "0x" << setw(2) << unsigned(TC) << " \t0b" << bitset<8>(TC) << endl;
  cout << "PSW : " << dec << unsigned(PSW) << " \t" << hex << "0x" << setw(2) << unsigned(PSW) << " \t0b" << bitset<8>(PSW) << endl;

  cout << endl;

  for (int i = 0; i < 64; i++)
  {
    cout << hex << setw(2) << unsigned(RAM[i]) << "  ";

    if ((i % 16) == 15)
    {
      cout << endl;
    }
  }

  cout.flags(oldFlags);
  cout.precision(oldPrec);
  cout.fill(oldFill);
}