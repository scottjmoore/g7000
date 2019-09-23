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

  PSW = 0b00001000;
}

MCS48::~MCS48()
{
}

void MCS48::reset()
{
  PC = 0x0000;
  PSW = 0b00001000;
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

  stringout << setfill('0') << hex << setw(4) << PC - 1 << " : ";
  stringout << setfill('0') << hex << setw(2) << unsigned(fetched) << " ";

  switch (fetched)
  {
  case 0b00000011: // ADD A, #data
    fetch();       // fetch immediate data to add to accumulator
    stringout << setfill('0') << hex << setw(2) << unsigned(fetched) << " \t\t";
    stringout << "ADD A, ";
    stringout << "#" << setfill('0') << hex << setw(2) << unsigned(fetched) << "H";
    ADD_A_data(fetched); // execute instruction
    break;
  case 0b00100111: // CLR A
    stringout << "   \t\t";
    stringout << "CLR A";
    CLR_A(); // execute instruction
    break;
  case 0b10010111: // CLR C
    stringout << "   \t\t";
    stringout << "CLR C";
    CLR_C(); // execute instruction
    break;
  case 0b00110111: // CPL A
    stringout << "   \t\t";
    stringout << "CPL A";
    CPL_A(); // execute instruction
    break;
  case 0b10100111: // CPL C
    stringout << "   \t\t";
    stringout << "CPL C";
    CPL_C(); // execute instruction
    break;
  case 0b00100011: // MOV A, #data
    fetch();       // fetch immediate data to add to accumulator
    stringout << setfill('0') << hex << setw(2) << unsigned(fetched) << " \t\t";
    stringout << "MOV A, ";
    stringout << "#" << setfill('0') << hex << setw(2) << unsigned(fetched) << "H";
    MOV_A_data(fetched); // execute instruction
    break;
  case 0b00000000: // NOP
    stringout << "   \t\t";
    stringout << "NOP";
    NOP(); // execute instruction
    break;
  case 0b11000101: // SEL RB0
    stringout << "   \t\t";
    stringout << "SEL RB0";
    SEL_RB0(); // execute instruction
    break;
  case 0b11010101: // SEL RB1
    stringout << "   \t\t";
    stringout << "SEL RB1";
    SEL_RB1(); // execute instruction
    break;
  case 0b11010011: // XLR A, #data
    fetch();       // fetch immediate data to add to accumulator
    stringout << setfill('0') << hex << setw(2) << unsigned(fetched) << " \t\t";
    stringout << "XLR A, ";
    stringout << "#" << setfill('0') << hex << unsigned(fetched) << "H";
    XRL_A_data(fetched); // execute instruction
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
      JMP(address);
      decoded = true;
      break;
    }

    if (!decoded)
    {
      uint8_t reg = fetched & 0b00000111;

      switch (fetched & 0b11111000)
      {
      case 0b00011000: // INC Rr
        stringout << "   \t\t";
        stringout << "INC R" << unsigned(reg);
        writeRegister(reg, readRegister(reg) + 1); // TODO : Change to opcode function
        decoded = true;
        break;
      case 0b10101000: // MOV Rr, A
        stringout << "   \t\t";
        stringout << "MOV R" << unsigned(reg) << ", A";
        writeRegister(reg, A);
        decoded = true;
        break;

      case 0b11111000: // MOV A, Rr
        stringout << "   \t\t";
        stringout << "MOV A, R" << unsigned(reg);
        A = readRegister(reg);
        decoded = true;
        break;

      case 0b10111000: // MOV Rr, #data
        fetch();
        stringout << setfill('0') << hex << setw(2) << unsigned(fetched) << " \t\t";
        stringout << "MOV R" << unsigned(reg) << ", ";
        stringout << "#" << setfill('0') << hex << setw(2) << unsigned(fetched) << "H";
        writeRegister(reg, fetched);
        decoded = true;
        break;
      }
    }
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

void MCS48::JMP(uint16_t address)
{
  PC = address;
}

void MCS48::MOV_A_data(uint8_t data)
{
  A = data;
}

void MCS48::MOV_A_PSW()
{
  A = PSW;
}

void MCS48::MOV_A_RR(uint8_t reg)
{
  A = readRegister(reg);
}

void MCS48::MOV_PSW_A()
{
  PSW = A;
}

void MCS48::MOV_RR_A(uint8_t reg)
{
  writeRegister(reg, A);
}

void MCS48::MOV_RR_data(uint8_t reg, uint8_t data)
{
  writeRegister(reg, data);
}

void MCS48::NOP()
{
  // do nothing
}

void MCS48::SEL_RB0()
{
  PSW &= ~PSW_BITS::BS;
}

void MCS48::SEL_RB1()
{
  PSW |= PSW_BITS::BS;
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