#include "bus.h"
#include "mcs48.h"

MCS48::MCS48(CPUTYPE _cputype, ::BUS &_bus)
{
  cputype = _cputype;
  bus = _bus;

  switch (cputype)
  {
  case CPUTYPE::CPU8021:
    data_memory_size = 64;
    program_memory_size = 1024;
    break;
  case CPUTYPE::CPU8048:
    data_memory_size = 64;
    program_memory_size = 1024;
    break;
  case CPUTYPE::CPU8049:
    data_memory_size = 128;
    program_memory_size = 2048;
    break;
  }

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

  cycles = 0;

  bus.attachTo4BitBus(0, &PORT4, "PORT 4");
  bus.attachTo4BitBus(1, &PORT5, "PORT 5");
  bus.attachTo4BitBus(2, &PORT6, "PORT 6");
  bus.attachTo4BitBus(3, &PORT7, "PORT 7");
  bus.attachTo8BitBus(0, &PORT1, "PORT 1");
  bus.attachTo8BitBus(1, &PORT2, "PORT 2");
  bus.attachTo8BitBus(2, &BUS, "BUS");
  bus.attachTo16BitBus(0, &PC, "PC");
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

void MCS48::interrupt()
{
  CALL(0x0003);
}

void MCS48::timer_interrupt()
{
  CALL(0x0007);
}

void MCS48::clock()
{
  if (cycles == 0)
  {
    fetch();

    cycles = decode();
  }

  --cycles;
}

void MCS48::fetch()
{
  fetched = readROM(PC++);

  PC &= 0b0000111111111111;
}

void MCS48::push_pc_psw()
{
  uint8_t firstbyte;
  uint8_t secondbyte;
  uint8_t stackpointer;

  firstbyte = PC & 0b000011111111;
  secondbyte = (PC & 0b111111111111) >> 8;
  secondbyte |= PSW & 0b11110000;

  stackpointer = (PSW & 0b00000111);
  writeRAM(STACK + (stackpointer << 1), firstbyte);
  writeRAM(STACK + (stackpointer << 1) + 1, secondbyte);

  stackpointer++;

  PSW = (PSW & 0b11111000) | (stackpointer & 0b00000111);
}

void MCS48::pop_pc_psw()
{
  uint8_t firstbyte;
  uint8_t secondbyte;
  uint8_t stackpointer;

  stackpointer = (PSW & 0b00000111);
  stackpointer--;

  firstbyte = readRAM(STACK + (stackpointer << 1));
  secondbyte = readRAM(STACK + (stackpointer << 1) + 1);

  PC = (uint16_t)firstbyte | (((uint16_t)secondbyte & 0b00001111) << 8);
  PSW = (secondbyte & 0b11110000) | (stackpointer & 0b00000111);
}

void MCS48::pop_pc()
{
  uint8_t firstbyte;
  uint8_t secondbyte;
  uint8_t stackpointer;

  stackpointer = (PSW & 0b00000111);
  stackpointer--;

  firstbyte = readRAM(STACK + stackpointer);
  secondbyte = readRAM(STACK + stackpointer + 1);

  PC = (uint16_t)firstbyte | (((uint16_t)secondbyte & 0b00001111) << 8);
  PSW = (PSW & 0b11111000) | (stackpointer & 0b00000111);
}

uint8_t MCS48::decode()
{
  uint8_t cycles = 0;
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
  case 0b00010011: // ADDC A, #data
    fetch();       // fetch immediate data to add to accumulator
    stringout << setfill('0') << hex << setw(2) << unsigned(fetched) << " \t\t";
    stringout << "ADDC A, ";
    stringout << "#" << setfill('0') << hex << setw(2) << unsigned(fetched) << "H";
    cycles = ADDC_A_data(fetched); // execute instruction
    break;
  case 0b01010011: // ANL A, #data
    fetch();       // fetch immediate data to add to accumulator
    stringout << setfill('0') << hex << setw(2) << unsigned(fetched) << " \t\t";
    stringout << "ANL A, ";
    stringout << "#" << setfill('0') << hex << setw(2) << unsigned(fetched) << "H";
    cycles = ANL_A_data(fetched); // execute instruction
    break;
  case 0b10011000: // ANL BUS, #data
    fetch();       // fetch immediate data to add to accumulator
    stringout << setfill('0') << hex << setw(2) << unsigned(fetched) << " \t\t";
    stringout << "ANL A, ";
    stringout << "#" << setfill('0') << hex << setw(2) << unsigned(fetched) << "H";
    cycles = ANL_BUS_data(fetched); // execute instruction
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
  case 0b10100101: // CLR F1
    stringout << "   \t\t";
    stringout << "CLR F1";
    cycles = CLR_F1(); // execute instruction
    break;
  case 0b10000101: // CLR F0
    stringout << "   \t\t";
    stringout << "CLR F0";
    cycles = CLR_F0(); // execute instruction
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
  case 0b10010101: // CPL F0
    stringout << "   \t\t";
    stringout << "CPL F0";
    cycles = CPL_F0(); // execute instruction
    break;
  case 0b10110101: // CPL F1
    stringout << "   \t\t";
    stringout << "CPL F1";
    cycles = CPL_F1(); // execute instruction
    break;
  case 0b01010111: // DA A
    stringout << "   \t\t";
    stringout << "DA A";
    cycles = DA_A(); // execute instruction
    break;
  case 0b00000111: // DEC A
    stringout << "   \t\t";
    stringout << "DEC A";
    cycles = DEC_A(); // execute instruction
    break;
  case 0b00010101: // DIS I
    stringout << "   \t\t";
    stringout << "DIS I";
    cycles = DIS_I(); // execute instruction
    break;
  case 0b00110101: // DIS TCNTI
    stringout << "   \t\t";
    stringout << "DIS TCNTI";
    cycles = DIS_TCNTI(); // execute instruction
    break;
  case 0b00000101: // EN I
    stringout << "   \t\t";
    stringout << "EN I";
    cycles = EN_I(); // execute instruction
    break;
  case 0b00100101: // EN TCNTI
    stringout << "   \t\t";
    stringout << "EN TCNTI";
    cycles = EN_TCNTI(); // execute instruction
    break;
  case 0b01110101: // ENT0 CLK
    stringout << "   \t\t";
    stringout << "ENT0 CLK";
    cycles = ENT0_CLK(); // execute instruction
    break;
  case 0b00010111: // INC A
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
  case 0b10000011: // RET
    stringout << "   \t\t";
    stringout << "RET";
    cycles = RET(); // execute instruction
    break;
  case 0b10010011: // RETR
    stringout << "   \t\t";
    stringout << "RETR";
    cycles = RETR(); // execute instruction
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
    bool decoded = false; // was the opcode decoded?

    uint16_t address; // storage for fetched address
    uint8_t reg;      // storage for fetched register
    uint8_t port;     // storage for fetched port

    switch ((fetched & 0b00011111)) // decode opcodes with address in top 3 bits
    {
    case 0b00000100: // JMP address
      address = (fetched & 0b11100000) << 3;

      fetch();
      address |= fetched;
      stringout << setfill('0') << hex << setw(2) << unsigned(fetched) << " \t\t";
      stringout << "JMP ";
      stringout << setfill('0') << hex << setw(4) << address << "H";
      cycles = JMP(address);
      decoded = true;
      break;
    case 0b00010100: // CALL address
      address = (fetched & 0b11100000) << 3;

      fetch();
      address |= fetched;
      stringout << setfill('0') << hex << setw(2) << unsigned(fetched) << " \t\t";
      stringout << "CALL ";
      stringout << setfill('0') << hex << setw(4) << address << "H";
      cycles = CALL(address);
      decoded = true;
      break;
    }

    if (!decoded) // decode opcodes with register in bottom 3/1 bits
    {
      reg = fetched & 0b00000111;

      switch (fetched & 0b11111000)
      {
      case 0b01101000: // ADD A, R
        stringout << "   \t\t";
        stringout << "ADD A, R" << unsigned(reg);
        cycles = ADD_A_R(reg);
        decoded = true;
        break;
      case 0b01100000: // ADD A, @R
        stringout << "   \t\t";
        stringout << "ADD A, @R" << unsigned(reg);
        cycles = ADD_A_RC(reg);
        decoded = true;
        break;
      case 0b01111000: // ADDC A, R
        stringout << "   \t\t";
        stringout << "ADDC A, R" << unsigned(reg);
        cycles = ADDC_A_R(reg);
        decoded = true;
        break;
      case 0b01110000: // ADDC A, @R
        stringout << "   \t\t";
        stringout << "ADDC A, @R" << unsigned(reg);
        cycles = ADDC_A_RC(reg);
        decoded = true;
        break;
      case 0b01011000: // ANL A, R
        stringout << "   \t\t";
        stringout << "ANL A, R" << unsigned(reg);
        cycles = ANL_A_R(reg);
        decoded = true;
        break;
      case 0b01010000: // ANL A, @R
        stringout << "   \t\t";
        stringout << "ANL A, @R" << unsigned(reg);
        cycles = ANL_A_RC(reg);
        decoded = true;
        break;
      case 0b11101000: // DJNZ R, address
        fetch();
        address = fetched;
        stringout << "   \t\t";
        stringout << "DJNZ R" << unsigned(reg) << ", ";
        stringout << setfill('0') << hex << setw(4) << address << "H";
        cycles = DJNZ_R_address(reg, address);
        decoded = true;
        break;
      case 0b00011000: // INC R
        stringout << "   \t\t";
        stringout << "INC R" << unsigned(reg);
        cycles = INC_R(reg);
        decoded = true;
        break;
      case 0b00010000: // INC @R
        stringout << "   \t\t";
        stringout << "INC @R" << unsigned(reg);
        cycles = INC_RC(reg);
        decoded = true;
        break;
      case 0b10101000: // MOV Rr, A
        stringout << "   \t\t";
        stringout << "MOV R" << unsigned(reg) << ", A";
        cycles = MOV_R_A(reg);
        decoded = true;
        break;

      case 0b11111000: // MOV A, Rr
        stringout << "   \t\t";
        stringout << "MOV A, R" << unsigned(reg);
        cycles = MOV_A_R(reg);
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

    if (!decoded) // decode opcodes with port in bottom 2 bits
    {
      port = fetched & 0b00000011;

      switch (fetched & 0b11111100)
      {
      case 0b00001000: // IN A, P
        stringout << "   \t\t";
        stringout << "IN A, P" << unsigned(port);
        cycles = IN_A_P(port);
        decoded = true;
        break;
      case 0b00001100: // MOVD A, P
        stringout << "   \t\t";
        stringout << "MOVD A, P" << unsigned(port + 4);
        cycles = MOVD_A_P(port);
        decoded = true;
        break;
      case 0b00111100: // MOVD P, A
        stringout << "   \t\t";
        stringout << "MOVD P" << unsigned(port + 4) << ", A";
        cycles = MOVD_P_A(port);
        decoded = true;
        break;
      case 0b10001000: // ORL P, #data
        fetch();
        stringout << "   \t\t";
        stringout << "ORL P" << unsigned(port) << ", #" << setfill('0') << hex << setw(2) << unsigned(fetched) << "H";
        cycles = ORL_P_data(port, fetched);
        decoded = true;
        break;
      case 0b10001100: // ORLD P, A
        fetch();
        stringout << "   \t\t";
        stringout << "ORL P" << unsigned(port) << ", A";
        cycles = ORLD_P_A(port);
        decoded = true;
        break;
      case 0b00111000: // OUTL P, A
        stringout << "   \t\t";
        stringout << "OUTL P" << unsigned(port) << ", A";
        cycles = OUTL_P_A(port);
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
  if (address < program_memory_size)
  {
    uint16_t a = address & (program_memory_size - 1);

    return ROM[a];
  }
  else
  {
    // read from external program memory
    return 0x00;
  }
}

void MCS48::writeROM(uint16_t address, uint8_t data)
{
  if (address < program_memory_size)
  {
    uint16_t a = address & (program_memory_size - 1);

    ROM[a] = data;
  }
  else
  {
    // write to external program memory
  }
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

uint8_t MCS48::readExternalRAM(uint8_t address)
{
  return 0x00;
}

void MCS48::writeExternalRAM(uint8_t address, uint8_t data)
{
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

uint8_t MCS48::ADDC_A_R(uint8_t R)
{
  uint8_t PA = A;
  uint8_t C = PSW & PSW_BITS::CY ? 1 : 0;

  A = A + readRegister(R) + C;

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

uint8_t MCS48::ANL_BUS_data(uint8_t data)
{
  BUS = BUS & data;

  return 2;
}

uint8_t MCS48::ANL_P_data(uint8_t port, uint8_t data)
{
  switch (port)
  {
  case 1:
    PORT1 = PORT1 & data;
    break;
  case 2:
    PORT2 = PORT2 & data;
    break;
  }

  return 2;
}

uint8_t MCS48::ANLD_P_A(uint8_t port)
{
  switch (port)
  {
  case 0:
    PORT4 = PORT4 & (A & 0b00001111);
    break;
  case 1:
    PORT5 = PORT5 & (A & 0b00001111);
    break;
  case 2:
    PORT6 = PORT6 & (A & 0b00001111);
    break;
  case 3:
    PORT7 = PORT7 & (A & 0b00001111);
    break;
  }

  return 2;
}

uint8_t MCS48::CALL(uint16_t address)
{
  push_pc_psw();

  PC = address;

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

uint8_t MCS48::CLR_F1()
{
  F1 = 0b0;

  return 1;
}

uint8_t MCS48::CLR_F0()
{
  PSW = PSW & ~PSW_BITS::F0;

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

uint8_t MCS48::CPL_F0()
{
  PSW ^= PSW_BITS::F0;

  return 1;
}

uint8_t MCS48::CPL_F1()
{
  F1 = ~F1;

  return 1;
}

uint8_t MCS48::DA_A()
{
  uint8_t PA = A;
  uint8_t nibble_lo = (A & 0b00001111);

  if ((nibble_lo > 9) || (PSW & PSW_BITS::AC))
  {
    A = A + 6;
  }

  uint8_t nibble_hi = (A & 0b11110000) >> 4;

  if ((nibble_hi > 9) || (PSW & PSW_BITS::CY))
  {
    A = A + 6;
  }

  if (A < PA)
    PSW |= PSW_BITS::CY;

  return 1;
}

uint8_t MCS48::DEC_A()
{
  A--;

  return 1;
}

uint8_t MCS48::DEC_R(uint8_t reg)
{
  writeRegister(reg, readRegister(reg) - 1);

  return 1;
}

uint8_t MCS48::DIS_I()
{
  // disable interrupts

  return 1;
}

uint8_t MCS48::DIS_TCNTI()
{
  // disable timer/counter interrupts

  return 1;
}

uint8_t MCS48::DJNZ_R_address(uint8_t reg, uint8_t address)
{
  DEC_R(reg);

  if (readRegister(reg) != 0x00)
  {
    PC = (PC & 0b1111111100000000) | address;
  }

  return 2;
}

uint8_t MCS48::EN_I()
{
  // enable interrupts

  return 1;
}

uint8_t MCS48::EN_TCNTI()
{
  // enable timer/counter interrupts

  return 1;
}

uint8_t MCS48::ENT0_CLK()
{
  // enable clock output

  return 1;
}

uint8_t MCS48::IN_A_P(uint8_t port)
{
  switch (port)
  {
  case 1:
    A = PORT1;
    break;
  case 2:
    A = PORT2;
    break;
  }

  return 2;
}

uint8_t MCS48::INC_A()
{
  A++;

  return 1;
}

uint8_t MCS48::INC_R(uint8_t reg)
{
  writeRegister(reg, readRegister(reg) + 1);

  return 1;
}

uint8_t MCS48::INC_RC(uint8_t reg)
{
  uint8_t address = readRegister(reg);
  writeRAM(address, readRAM(address) + 1);

  return 1;
}

uint8_t MCS48::IN_A_P0()
{
  A = BUS;

  return 1;
}

uint8_t MCS48::INS_A_BUS()
{
  A = BUS;

  return 2;
}

uint8_t MCS48::JBB(uint8_t bit, uint8_t address)
{
  uint8_t mask = 1 << bit;

  if (A & mask)
  {
    PC = (PC & 0b1111111100000000) | address;
  }

  return 2;
}

uint8_t MCS48::JC(uint8_t address)
{
  if (PSW & PSW_BITS::CY)
  {
    PC = (PC & 0b1111111100000000) | address;
  }

  return 2;
}

uint8_t MCS48::JF0(uint8_t address)
{
  if (PSW & PSW_BITS::F0)
  {
    PC = (PC & 0b1111111100000000) | address;
  }

  return 2;
}

uint8_t MCS48::JF1(uint8_t address)
{
  if (F1)
  {
    PC = (PC & 0b1111111100000000) | address;
  }

  return 2;
}

uint8_t MCS48::JMP(uint16_t address)
{
  PC = address;

  return 2;
}

uint8_t MCS48::JMPP_AC()
{
  uint16_t address = (PC & 0b1111111100000000) | A;

  PC = (PC & 0b1111111100000000) | readROM(address);

  return 2;
}

uint8_t MCS48::JNC(uint8_t address)
{
  if (!(PSW & PSW_BITS::CY))
  {
    PC = (PC & 0b1111111100000000) | address;
  }

  return 2;
}

uint8_t MCS48::JNI(uint8_t address)
{
  if (!I)
  {
    PC = (PC & 0b1111111100000000) | address;
  }

  return 2;
}

uint8_t MCS48::JNT0(uint8_t address)
{
  if (!T0)
  {
    PC = (PC & 0b1111111100000000) | address;
  }

  return 2;
}

uint8_t MCS48::JNT1(uint8_t address)
{
  if (!T1)
  {
    PC = (PC & 0b1111111100000000) | address;
  }

  return 2;
}

uint8_t MCS48::JNZ(uint8_t address)
{
  if (A != 0)
  {
    PC = (PC & 0b1111111100000000) | address;
  }

  return 2;
}

uint8_t MCS48::JTF(uint8_t address)
{
  if (TF != 0)
  {
    PC = (PC & 0b1111111100000000) | address;
  }

  return 2;
}

uint8_t MCS48::JT0(uint8_t address)
{
  if (T0 != 0)
  {
    PC = (PC & 0b1111111100000000) | address;
  }
  return 2;
}

uint8_t MCS48::JT1(uint8_t address)
{
  if (T1 != 0)
  {
    PC = (PC & 0b1111111100000000) | address;
  }

  return 2;
}

uint8_t MCS48::JZ(uint8_t address)
{
  if (A == 0)
  {
    PC = (PC & 0b1111111100000000) | address;
  }

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

uint8_t MCS48::MOV_A_RC(uint8_t reg)
{
  A = readRegister(readRegister(reg));

  return 1;
}

uint8_t MCS48::MOV_A_T()
{
  A = TC;

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

uint8_t MCS48::MOV_RC_data(uint8_t reg, uint8_t data)
{
  writeRAM(readRegister(reg), data);

  return 2;
}

uint8_t MCS48::MOV_T_A()
{
  TC = A;

  return 1;
}

uint8_t MCS48::MOVD_A_P(uint8_t port)
{
  switch (port)
  {
  case 0:
    A = PORT4 & 0b00001111;
    break;
  case 1:
    A = PORT5 & 0b00001111;
    break;
  case 2:
    A = PORT6 & 0b00001111;
    break;
  case 3:
    A = PORT7 & 0b00001111;
    break;
  }

  return 2;
}

uint8_t MCS48::MOVD_P_A(uint8_t port)
{
  switch (port)
  {
  case 0:
    PORT4 = (A & 0b00001111);
    break;
  case 1:
    PORT5 = (A & 0b00001111);
    break;
  case 2:
    PORT6 = (A & 0b00001111);
    break;
  case 3:
    PORT7 = (A & 0b00001111);
    break;
  }

  return 2;
}

uint8_t MCS48::MOVP_A_AC()
{
  uint16_t address = (PC & 0b1111111100000000) | A;

  A = readROM(address);

  return 2;
}

uint8_t MCS48::MOVP3_A_AC()
{
  uint16_t address = (0b0000001100000000) | A;

  A = readROM(address);

  return 2;
}

uint8_t MCS48::NOP()
{
  return 1;
}

uint8_t MCS48::ORL_A_R(uint8_t reg)
{
  return 1;
}

uint8_t MCS48::ORL_A_RC(uint8_t reg)
{
  return 1;
}

uint8_t MCS48::ORL_A_data(uint8_t data)
{
  return 1;
}

uint8_t MCS48::ORL_BUS_data(uint8_t data)
{
  return 1;
}

uint8_t MCS48::ORL_P_data(uint8_t port, uint8_t data)
{
  switch (port)
  {
  case 1:
    PORT1 |= data;
    break;
  case 2:
    PORT2 |= data;
    break;
  }

  return 2;
}

uint8_t MCS48::ORLD_P_A(uint8_t port)
{
  switch (port)
  {
  case 0:
    PORT4 |= A & 0b1111;
    break;
  case 1:
    PORT5 |= A & 0b1111;
    break;
  case 2:
    PORT6 |= A & 0b1111;
    break;
  case 3:
    PORT7 |= A & 0b1111;
    break;
  }

  return 2;
}

uint8_t MCS48::OUTL_P0_A()
{
  return 1;
}

uint8_t MCS48::OUTL_BUS_A()
{
  return 1;
}

uint8_t MCS48::OUTL_P_A(uint8_t port)
{
  switch (port)
  {
  case 1:
    PORT1 = A;
    break;
  case 2:
    PORT2 = A;
    break;
  }

  return 2;
}

uint8_t MCS48::RET()
{
  pop_pc();

  return 2;
}

uint8_t MCS48::RETR()
{
  pop_pc_psw();

  return 2;
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
  cout << "PSW : " << dec << unsigned(PSW) << " \t" << hex << "0x" << setw(2) << unsigned(PSW) << " \t0b" << bitset<8>(PSW) << " \t";
  cout << ((PSW & PSW_BITS::CY) ? "|CY|" : "|--|");
  cout << ((PSW & PSW_BITS::AC) ? "AC|" : "--|");
  cout << ((PSW & PSW_BITS::F0) ? "F0|" : "--|");
  cout << ((F1) ? "F1|" : "--|");
  cout << ((PSW & PSW_BITS::BS) ? "BS|" : "--|");
  cout << ((PSW & PSW_BITS::S2) ? "S2|" : "--|");
  cout << ((PSW & PSW_BITS::S1) ? "S1|" : "--|");
  cout << ((PSW & PSW_BITS::S0) ? "S0|" : "--|");
  cout << endl;
  cout << "SP  : " << dec << unsigned(PSW & 0b00000111) << " \t" << hex << "0x" << setw(2) << unsigned(PSW & 0b00000111) << " \t0b" << bitset<3>(PSW & 0b00000111) << " \t" << endl;
  cout << endl;

  for (int i = 0; i < data_memory_size; i++)
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