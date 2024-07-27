#include <cstdint>
#include <iostream>
#include <m6502.h>
#include <string>
#include <vector>

using namespace nes;

void CPU::reset() {
  registers.reset();
  // memory_.fill(0);
  // memory_[0xFFFC] = 0x00;
  // memory_[0xFFFD] = 0x02;
  initializeTables();
  registers.PC = bus->cpu_read(0xFFFC) | (bus->cpu_read(0xFFFD) << 8);
}

uint64_t CPU::step() {
  cycles_ = 0;
  Byte opcode = bus->cpu_read(registers.PC++);
  auto it = opcodeTable_.find(opcode);
  if (it == opcodeTable_.end()) {
    throw std::runtime_error("Unknown opcode: " + std::to_string(opcode));
    // std::cout << "Unknown opcode: " + std::to_string(opcode) << "\n";
  }
  const Instruction &instruction = it->second;

  cycles_ += instruction.cycles;

  if (instruction.needsExtraCycle) {
    handlePageCrossing(instruction);
  }

  switch (instruction.mnemonic) {
  case Operation::ADC:
    ADC(instruction);
    break;
  case Operation::AND:
    AND(instruction);
    break;
  case Operation::ASL:
    ASL(instruction);
    break;
  case Operation::BCC:
    BCC(instruction);
    break;
  case Operation::BCS:
    BCS(instruction);
    break;
  case Operation::BEQ:
    BEQ(instruction);
    break;
  case Operation::BIT:
    BIT(instruction);
    break;
  case Operation::BMI:
    BMI(instruction);
    break;
  case Operation::BNE:
    BNE(instruction);
    break;
  case Operation::BPL:
    BPL(instruction);
    break;
  case Operation::BRK:
    BRK(instruction);
    break;
  case Operation::BVC:
    BVC(instruction);
    break;
  case Operation::BVS:
    BVS(instruction);
    break;
  case Operation::CLC:
    CLC(instruction);
    break;
  case Operation::CLD:
    CLD(instruction);
    break;
  case Operation::CLI:
    CLI(instruction);
    break;
  case Operation::CLV:
    CLV(instruction);
    break;
  case Operation::CMP:
    CMP(instruction);
    break;
  case Operation::CPX:
    CPX(instruction);
    break;
  case Operation::CPY:
    CPY(instruction);
    break;
  case Operation::DEC:
    DEC(instruction);
    break;
  case Operation::DEX:
    DEX(instruction);
    break;
  case Operation::DEY:
    DEY(instruction);
    break;
  case Operation::EOR:
    EOR(instruction);
    break;
  case Operation::INC:
    INC(instruction);
    break;
  case Operation::INX:
    INX(instruction);
    break;
  case Operation::INY:
    INY(instruction);
    break;
  case Operation::JMP:
    JMP(instruction);
    break;
  case Operation::JSR:
    JSR(instruction);
    break;
  case Operation::LDA:
    LDA(instruction);
    break;
  case Operation::LDX:
    LDX(instruction);
    break;
  case Operation::LDY:
    LDY(instruction);
    break;
  case Operation::LSR:
    LSR(instruction);
    break;
  case Operation::NOP:
    NOP(instruction);
    break;
  case Operation::ORA:
    ORA(instruction);
    break;
  case Operation::PHA:
    PHA(instruction);
    break;
  case Operation::PHP:
    PHP(instruction);
    break;
  case Operation::PLA:
    PLA(instruction);
    break;
  case Operation::PLP:
    PLP(instruction);
    break;
  case Operation::ROL:
    ROL(instruction);
    break;
  case Operation::ROR:
    ROR(instruction);
    break;
  case Operation::RTI:
    RTI(instruction);
    break;
  case Operation::RTS:
    RTS(instruction);
    break;
  case Operation::SBC:
    SBC(instruction);
    break;
  case Operation::SEC:
    SEC(instruction);
    break;
  case Operation::SED:
    SED(instruction);
    break;
  case Operation::SEI:
    SEI(instruction);
    break;
  case Operation::STA:
    STA(instruction);
    break;
  case Operation::STX:
    STX(instruction);
    break;
  case Operation::STY:
    STY(instruction);
    break;
  case Operation::TAX:
    TAX(instruction);
    break;
  case Operation::TAY:
    TAY(instruction);
    break;
  case Operation::TSX:
    TSX(instruction);
    break;
  case Operation::TXA:
    TXA(instruction);
    break;
  case Operation::TXS:
    TXS(instruction);
    break;
  case Operation::TYA:
    TYA(instruction);
    break;
  case Operation::SLO:
    SLO(instruction);
    break;

  case Operation::RLA:
    RLA(instruction);
    break;

  case Operation::SRE:
    SRE(instruction);
    break;

  case Operation::RRA:
    RRA(instruction);
    break;

  case Operation::SAX:
    SAX(instruction);
    break;

  case Operation::LAX:
    LAX(instruction);
    break;

  case Operation::DCP:
    DCP(instruction);
    break;

  case Operation::ISC:
    ISC(instruction);
    break;

  case Operation::ANC:
    ANC(instruction);
    break;

  case Operation::ALR:
    ALR(instruction);
    break;

  case Operation::ARR:
    ARR(instruction);
    break;

  case Operation::XAA:
    XAA(instruction);
    break;

  case Operation::AXS:
    AXS(instruction);
    break;

  case Operation::AHX:
    AHX(instruction);
    break;

  case Operation::SHY:
    SHY(instruction);
    break;

  case Operation::SHX:
    SHX(instruction);
    break;

  case Operation::TAS:
    TAS(instruction);
    break;

  case Operation::LAS:
    LAS(instruction);
    break;

  case Operation::KIL:
    KIL(instruction);
    break;
  default:
    throw std::runtime_error("Unimplemented instruction");
    // std::cout << "Unimplemented instruction: "
    //           << std::to_string(instruction.opcode) << "\n";
    // fetchOperand(instruction.mode);
    // fetchAddress(instruction.mode);
  }
  return cycles_;
}

// Combined operation functions
// Load/Store Operations
void CPU::LDA(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  registers.A = value;
  updateZeroAndNegativeFlags(registers.A);
}

void CPU::LDX(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  registers.X = value;
  updateZeroAndNegativeFlags(registers.X);
}

void CPU::LDY(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  registers.Y = value;
  updateZeroAndNegativeFlags(registers.Y);
}

void CPU::STA(const Instruction &instruction) {
  Word address = fetchAddress(instruction.mode);
  bus->cpu_write(address, registers.A);
}

void CPU::STX(const Instruction &instruction) {
  Word address = fetchAddress(instruction.mode);
  bus->cpu_write(address, registers.X);
}

void CPU::STY(const Instruction &instruction) {
  Word address = fetchAddress(instruction.mode);
  bus->cpu_write(address, registers.Y);
}

// Transfer Operations
void CPU::TAX(const Instruction &instruction) {
  registers.X = registers.A;
  updateZeroAndNegativeFlags(registers.X);
}

void CPU::TAY(const Instruction &instruction) {
  registers.Y = registers.A;
  updateZeroAndNegativeFlags(registers.Y);
}

void CPU::TXA(const Instruction &instruction) {
  registers.A = registers.X;
  updateZeroAndNegativeFlags(registers.A);
}

void CPU::TYA(const Instruction &instruction) {
  registers.A = registers.Y;
  updateZeroAndNegativeFlags(registers.A);
}

// Stack Operations
void CPU::TSX(const Instruction &instruction) {
  registers.X = registers.SP;
  updateZeroAndNegativeFlags(registers.X);
}

void CPU::TXS(const Instruction &instruction) { registers.SP = registers.X; }

void CPU::PHA(const Instruction &instruction) {
  bus->cpu_write(0x100 + registers.SP, registers.A);
  registers.SP--;
}

void CPU::PHP(const Instruction &instruction) {
  Byte status = (registers.P.N << 7) | (registers.P.V << 6) |
                (registers.P.U << 5) | (registers.P.B << 4) |
                (registers.P.D << 3) | (registers.P.I << 2) |
                (registers.P.Z << 1) | registers.P.C;
  bus->cpu_write(0x100 + registers.SP, status);
  registers.SP--;
}

void CPU::PLA(const Instruction &instruction) {
  registers.SP++;
  registers.A = bus->cpu_read(0x100 + registers.SP);
  updateZeroAndNegativeFlags(registers.A);
}

void CPU::PLP(const Instruction &instruction) {
  registers.SP++;
  Byte status = bus->cpu_read(0x100 + registers.SP);
  registers.P.N = (status & 0x80) != 0;
  registers.P.V = (status & 0x40) != 0;
  registers.P.U = (status & 0x20) != 0;
  registers.P.B = (status & 0x10) != 0;
  registers.P.D = (status & 0x08) != 0;
  registers.P.I = (status & 0x04) != 0;
  registers.P.Z = (status & 0x02) != 0;
  registers.P.C = (status & 0x01) != 0;
}

// Logical Operations
void CPU::AND(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  registers.A &= value;
  updateZeroAndNegativeFlags(registers.A);
}

void CPU::EOR(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  registers.A ^= value;
  updateZeroAndNegativeFlags(registers.A);
}

void CPU::ORA(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  registers.A |= value;
  updateZeroAndNegativeFlags(registers.A);
}

void CPU::BIT(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  registers.P.Z = (registers.A & value) == 0;
  registers.P.N = (value & 0x80) != 0;
  registers.P.V = (value & 0x40) != 0;
}

// Arithmetic Operations
void CPU::ADC(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  Word result = registers.A + value + (registers.P.C ? 1 : 0);

  registers.P.C = result > 0xFF;
  registers.P.V = ((registers.A ^ result) & (value ^ result) & 0x80) != 0;
  registers.A = result & 0xFF;

  updateZeroAndNegativeFlags(registers.A);
}

void CPU::SBC(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  Word result = registers.A - value - (registers.P.C ? 0 : 1);

  registers.P.C = result < 0x100;
  registers.P.V =
      ((registers.A ^ result) & ((0xFF - value) ^ result) & 0x80) != 0;
  registers.A = result & 0xFF;
  updateZeroAndNegativeFlags(registers.A);
}

void CPU::CMP(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  Word result = registers.A - value;

  registers.P.C = registers.A >= value;
  updateZeroAndNegativeFlags(result & 0xFF);
}

void CPU::CPX(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  Word result = registers.X - value;

  registers.P.C = registers.X >= value;
  updateZeroAndNegativeFlags(result & 0xFF);
}

void CPU::CPY(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  Word result = registers.Y - value;

  registers.P.C = registers.Y >= value;
  updateZeroAndNegativeFlags(result & 0xFF);
}

// Increments/Decrements
void CPU::INC(const Instruction &instruction) {
  Word address = fetchAddress(instruction.mode);
  Byte value = bus->cpu_read(address);
  value++;
  bus->cpu_write(address, value);
  updateZeroAndNegativeFlags(value);
}

void CPU::INX(const Instruction &instruction) {
  registers.X++;
  updateZeroAndNegativeFlags(registers.X);
}

void CPU::INY(const Instruction &instruction) {
  registers.Y++;
  updateZeroAndNegativeFlags(registers.Y);
}

void CPU::DEC(const Instruction &instruction) {
  Word address = fetchAddress(instruction.mode);
  Byte value = bus->cpu_read(address);
  value--;
  bus->cpu_write(address, value);
  updateZeroAndNegativeFlags(value);
}

void CPU::DEX(const Instruction &instruction) {
  registers.X--;
  updateZeroAndNegativeFlags(registers.X);
}

void CPU::DEY(const Instruction &instruction) {
  registers.Y--;
  updateZeroAndNegativeFlags(registers.Y);
}

// Shifts
void CPU::ASL(const Instruction &instruction) {
  if (instruction.mode == AddressingMode::Accumulator) {
    registers.P.C = (registers.A & 0x80) != 0;
    registers.A <<= 1;
    updateZeroAndNegativeFlags(registers.A);
  } else {
    Word address = fetchAddress(instruction.mode);
    Byte value = bus->cpu_read(address);
    registers.P.C = (value & 0x80) != 0;
    value <<= 1;
    bus->cpu_write(address, value);
    updateZeroAndNegativeFlags(value);
  }
}

void CPU::LSR(const Instruction &instruction) {
  if (instruction.mode == AddressingMode::Accumulator) {
    registers.P.C = (registers.A & 0x01) != 0;
    registers.A >>= 1;
    updateZeroAndNegativeFlags(registers.A);
  } else {
    Word address = fetchAddress(instruction.mode);
    Byte value = bus->cpu_read(address);
    registers.P.C = (value & 0x01) != 0;
    value >>= 1;
    bus->cpu_write(address, value);
    updateZeroAndNegativeFlags(value);
  }
}

void CPU::ROL(const Instruction &instruction) {
  if (instruction.mode == AddressingMode::Accumulator) {
    Byte oldCarry = registers.P.C ? 1 : 0;
    registers.P.C = (registers.A & 0x80) != 0;
    registers.A = (registers.A << 1) | oldCarry;
    updateZeroAndNegativeFlags(registers.A);
  } else {
    Word address = fetchAddress(instruction.mode);
    Byte value = bus->cpu_read(address);
    Byte oldCarry = registers.P.C ? 1 : 0;
    registers.P.C = (value & 0x80) != 0;
    value = (value << 1) | oldCarry;
    bus->cpu_write(address, value);
    updateZeroAndNegativeFlags(value);
  }
}

void CPU::ROR(const Instruction &instruction) {
  if (instruction.mode == AddressingMode::Accumulator) {
    Byte oldCarry = registers.P.C ? 0x80 : 0;
    registers.P.C = (registers.A & 0x01) != 0;
    registers.A = (registers.A >> 1) | oldCarry;
    updateZeroAndNegativeFlags(registers.A);
  } else {
    Word address = fetchAddress(instruction.mode);
    Byte value = bus->cpu_read(address);
    Byte oldCarry = registers.P.C ? 0x80 : 0;
    registers.P.C = (value & 0x01) != 0;
    value = (value >> 1) | oldCarry;
    bus->cpu_write(address, value);
    updateZeroAndNegativeFlags(value);
  }
}

// Jumps and Calls
void CPU::JMP(const Instruction &instruction) {
  Word address = fetchAddress(instruction.mode);
  registers.PC = address;
}

void CPU::JSR(const Instruction &instruction) {
  registers.PC--;
  bus->cpu_write(0x100 + registers.SP, (registers.PC >> 8) & 0xFF);
  registers.SP--;
  bus->cpu_write(0x100 + registers.SP, registers.PC & 0xFF);
  registers.SP--;
  registers.PC = fetchAddress(AddressingMode::Absolute);
}

void CPU::RTS(const Instruction &instruction) {
  registers.SP++;
  Word low = bus->cpu_read(0x100 + registers.SP);
  registers.SP++;
  Word high = bus->cpu_read(0x100 + registers.SP);
  registers.PC = (high << 8) | low;
  registers.PC++;
}

// Branches
void CPU::BCC(const Instruction &instruction) {
  if (!registers.P.C) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers.PC += static_cast<int8_t>(offset);
  }
}

void CPU::BCS(const Instruction &instruction) {
  if (registers.P.C) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers.PC += static_cast<int8_t>(offset);
  }
}

void CPU::BEQ(const Instruction &instruction) {
  if (registers.P.Z) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers.PC += static_cast<int8_t>(offset);
  }
}

void CPU::BMI(const Instruction &instruction) {
  if (registers.P.N) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers.PC += static_cast<int8_t>(offset);
  }
}

void CPU::BNE(const Instruction &instruction) {
  if (!registers.P.Z) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers.PC += static_cast<int8_t>(offset);
  }
}

void CPU::BPL(const Instruction &instruction) {
  if (!registers.P.N) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers.PC += static_cast<int8_t>(offset);
  }
}

void CPU::BVC(const Instruction &instruction) {
  if (!registers.P.V) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers.PC += static_cast<int8_t>(offset);
  }
}

void CPU::BVS(const Instruction &instruction) {
  if (registers.P.V) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers.PC += static_cast<int8_t>(offset);
  }
}

// Status Flag Changes
void CPU::CLC(const Instruction &instruction) { registers.P.C = false; }

void CPU::CLD(const Instruction &instruction) { registers.P.D = false; }

void CPU::CLI(const Instruction &instruction) { registers.P.I = false; }

void CPU::CLV(const Instruction &instruction) { registers.P.V = false; }

void CPU::SEC(const Instruction &instruction) { registers.P.C = true; }

void CPU::SED(const Instruction &instruction) { registers.P.D = true; }

void CPU::SEI(const Instruction &instruction) { registers.P.I = true; }

// System Functions
void CPU::BRK(const Instruction &instruction) {
  registers.PC++;
  bus->cpu_write(0x100 + registers.SP, (registers.PC >> 8) & 0xFF);
  registers.SP--;
  bus->cpu_write(0x100 + registers.SP, registers.PC & 0xFF);
  registers.SP--;

  Byte status = (registers.P.N << 7) | (registers.P.V << 6) | (1 << 5) |
                (1 << 4) | (registers.P.D << 3) | (registers.P.I << 2) |
                (registers.P.Z << 1) | registers.P.C;
  bus->cpu_write(0x100 + registers.SP, status);
  registers.SP--;

  registers.P.I = true;
  registers.PC = bus->cpu_read(0xFFFE) | (bus->cpu_read(0xFFFF) << 8);
}

void CPU::NOP(const Instruction &instruction) {
  // No operation
}

void CPU::RTI(const Instruction &instruction) {
  registers.SP++;
  Byte status = bus->cpu_read(0x100 + registers.SP);
  registers.P.N = (status & 0x80) != 0;
  registers.P.V = (status & 0x40) != 0;
  registers.P.U = (status & 0x20) != 0;
  registers.P.B = (status & 0x10) != 0;
  registers.P.D = (status & 0x08) != 0;
  registers.P.I = (status & 0x04) != 0;
  registers.P.Z = (status & 0x02) != 0;
  registers.P.C = (status & 0x01) != 0;

  registers.SP++;
  Word low = bus->cpu_read(0x100 + registers.SP);
  registers.SP++;
  Word high = bus->cpu_read(0x100 + registers.SP);
  registers.PC = (high << 8) | low;
}

//
// Unofficial opcodes
//

void CPU::SLO(const Instruction &instruction) {
  Word address = fetchAddress(instruction.mode);
  Byte value = bus->cpu_read(address);
  Byte result = value << 1;
  bus->cpu_write(address, result);
  registers.A |= result;
  registers.P.C = value & 0x80;
  updateZeroAndNegativeFlags(registers.A);
}

void CPU::RLA(const Instruction &instruction) {
  Word address = fetchAddress(instruction.mode);
  Byte value = bus->cpu_read(address);
  Byte oldCarry = registers.P.C ? 1 : 0;
  Byte result = (value << 1) | oldCarry;
  bus->cpu_write(address, result);
  registers.A &= result;
  registers.P.C = value & 0x80;
  updateZeroAndNegativeFlags(registers.A);
}

void CPU::SRE(const Instruction &instruction) {
  Word address = fetchAddress(instruction.mode);
  Byte value = bus->cpu_read(address);
  Byte result = value >> 1;
  bus->cpu_write(address, result);
  registers.A ^= result;
  registers.P.C = value & 0x01;
  updateZeroAndNegativeFlags(registers.A);
}

void CPU::RRA(const Instruction &instruction) {
  Word address = fetchAddress(instruction.mode);
  Byte value = bus->cpu_read(address);
  Byte oldCarry = registers.P.C ? 1 : 0;
  Byte result = (value >> 1) | oldCarry;
  bus->cpu_write(address, result);
  ADC(Instruction(instruction.opcode, Operation::ADC, instruction.cycles,
                  instruction.mode, instruction.needsExtraCycle));
}

void CPU::SAX(const Instruction &instruction) {
  Word address = fetchAddress(instruction.mode);
  Byte result = registers.A & registers.X;
  bus->cpu_write(address, result);
}

void CPU::LAX(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  registers.A = registers.X = value;
  updateZeroAndNegativeFlags(registers.A);
}

void CPU::DCP(const Instruction &instruction) {
  Word address = fetchAddress(instruction.mode);
  Byte value = bus->cpu_read(address);
  Byte result = value - 1;
  bus->cpu_write(address, result);
  registers.P.C = (registers.A >= result);
  updateZeroAndNegativeFlags(registers.A - result);
}

void CPU::ISC(const Instruction &instruction) {
  Word address = fetchAddress(instruction.mode);
  Byte value = bus->cpu_read(address);
  Byte result = value + 1;
  bus->cpu_write(address, result);
  SBC(Instruction(instruction.opcode, Operation::SBC, instruction.cycles,
                  instruction.mode, instruction.needsExtraCycle));
}

void CPU::ANC(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  registers.A &= value;
  updateZeroAndNegativeFlags(registers.A);
  registers.P.C = registers.A & 0x80;
}

void CPU::ALR(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  registers.A &= value;
  registers.P.C = registers.A & 0x01;
  registers.A >>= 1;
  updateZeroAndNegativeFlags(registers.A);
}

void CPU::ARR(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  registers.A &= value;
  Byte oldCarry = registers.P.C ? 1 : 0;
  registers.A = (registers.A >> 1) | oldCarry;
  updateZeroAndNegativeFlags(registers.A);
  registers.P.C = registers.A & 0x40;
  registers.P.V = ((registers.A >> 5) ^ (registers.A >> 6));
}

void CPU::XAA(const Instruction &instruction) {
  registers.A = registers.X;
  Byte value = fetchOperand(instruction.mode);
  registers.A &= value;
  updateZeroAndNegativeFlags(registers.A);
}

void CPU::AXS(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  Word result = (registers.A & registers.X) - value;
  registers.X = result & 0xFF;
  registers.P.C = result < 0x100;
  updateZeroAndNegativeFlags(registers.X);
}

void CPU::AHX(const Instruction &instruction) {
  Word address = fetchAddress(instruction.mode);
  Byte result = registers.A & registers.X & (address >> 8);
  bus->cpu_write(address, result);
}

void CPU::SHY(const Instruction &instruction) {
  Word address = fetchAddress(instruction.mode);
  Byte result = registers.Y & ((address >> 8) + 1);
  bus->cpu_write(address, result);
}

void CPU::SHX(const Instruction &instruction) {
  Word address = fetchAddress(instruction.mode);
  Byte result = registers.X & ((address >> 8) + 1);
  bus->cpu_write(address, result);
}

void CPU::TAS(const Instruction &instruction) {
  Word address = fetchAddress(instruction.mode);
  registers.SP = registers.A & registers.X;
  Byte result = registers.SP & ((address >> 8) + 1);
  bus->cpu_write(address, result);
}

void CPU::LAS(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  registers.A = registers.X = registers.SP = registers.SP & value;
  updateZeroAndNegativeFlags(registers.A);
}

void CPU::KIL(const Instruction &instruction) {
  // This instruction halts the CPU
  // In an emulator, you might want to throw an exception or set a flag
  // throw std::runtime_error("KIL instruction executed");
}

void CPU::initializeTables() {
  auto addInstruction = [this](Byte opcode, const Operation &mnemonic,
                               std::uint32_t cycles, AddressingMode mode,
                               bool ec = false) {
    Instruction inst{opcode, mnemonic, cycles, mode, ec};
    opcodeTable_[opcode] = inst;
    if (operationTable_.find(mnemonic) == operationTable_.end()) {
      operationTable_[mnemonic] = std::vector<Instruction>();
    }
    operationTable_[mnemonic].push_back(inst);
  };

  Operation op;

  // ADC (Add with Carry)
  op = Operation::ADC;
  addInstruction(0x69, op, 2, AddressingMode::Immediate);
  addInstruction(0x65, op, 3, AddressingMode::ZeroPage);
  addInstruction(0x75, op, 4, AddressingMode::ZeroPageX);
  addInstruction(0x6D, op, 4, AddressingMode::Absolute);
  addInstruction(0x7D, op, 4, AddressingMode::AbsoluteX, true);
  addInstruction(0x79, op, 4, AddressingMode::AbsoluteY, true);
  addInstruction(0x61, op, 6, AddressingMode::IndexedIndirect);
  addInstruction(0x71, op, 5, AddressingMode::IndirectIndexed, true);

  // AND (Logical AND)
  op = Operation::AND;
  addInstruction(0x29, op, 2, AddressingMode::Immediate);
  addInstruction(0x25, op, 3, AddressingMode::ZeroPage);
  addInstruction(0x35, op, 4, AddressingMode::ZeroPageX);
  addInstruction(0x2D, op, 4, AddressingMode::Absolute);
  addInstruction(0x3D, op, 4, AddressingMode::AbsoluteX, true);
  addInstruction(0x39, op, 4, AddressingMode::AbsoluteY, true);
  addInstruction(0x21, op, 6, AddressingMode::IndexedIndirect);
  addInstruction(0x31, op, 5, AddressingMode::IndirectIndexed, true);

  // ASL (Arithmetic Shift Left)
  op = Operation::ASL;
  addInstruction(0x0A, op, 2, AddressingMode::Accumulator);
  addInstruction(0x06, op, 5, AddressingMode::ZeroPage);
  addInstruction(0x16, op, 6, AddressingMode::ZeroPageX);
  addInstruction(0x0E, op, 6, AddressingMode::Absolute);
  addInstruction(0x1E, op, 7, AddressingMode::AbsoluteX);

  // BCC (Branch if Carry Clear)
  op = Operation::BCC;
  addInstruction(0x90, op, 2, AddressingMode::Relative);

  // BCS (Branch if Carry Set)
  op = Operation::BCS;
  addInstruction(0xB0, op, 2, AddressingMode::Relative);

  // BEQ (Branch if Equal)
  op = Operation::BEQ;
  addInstruction(0xF0, op, 2, AddressingMode::Relative);

  // BIT (Bit Test)
  op = Operation::BIT;
  addInstruction(0x24, op, 3, AddressingMode::ZeroPage);
  addInstruction(0x2C, op, 4, AddressingMode::Absolute);

  // BMI (Branch if Minus)
  op = Operation::BMI;
  addInstruction(0x30, op, 2, AddressingMode::Relative);

  // BNE (Branch if Not Equal)
  op = Operation::BNE;
  addInstruction(0xD0, op, 2, AddressingMode::Relative);

  // BPL (Branch if Positive)
  op = Operation::BPL;
  addInstruction(0x10, op, 2, AddressingMode::Relative);

  // BRK (Force Interrupt)
  op = Operation::BRK;
  addInstruction(0x00, op, 7, AddressingMode::Implied);

  // BVC (Branch if Overflow Clear)
  op = Operation::BVC;
  addInstruction(0x50, op, 2, AddressingMode::Relative);

  // BVS (Branch if Overflow Set)
  op = Operation::BVS;
  addInstruction(0x70, op, 2, AddressingMode::Relative);

  // CLC (Clear Carry Flag)
  op = Operation::CLC;
  addInstruction(0x18, op, 2, AddressingMode::Implied);

  // CLD (Clear Decimal Mode)
  op = Operation::CLD;
  addInstruction(0xD8, op, 2, AddressingMode::Implied);

  // CLI (Clear Interrupt Disable)
  op = Operation::CLI;
  addInstruction(0x58, op, 2, AddressingMode::Implied);

  // CLV (Clear Overflow Flag)
  op = Operation::CLV;
  addInstruction(0xB8, op, 2, AddressingMode::Implied);

  // CMP (Compare)
  op = Operation::CMP;
  addInstruction(0xC9, op, 2, AddressingMode::Immediate);
  addInstruction(0xC5, op, 3, AddressingMode::ZeroPage);
  addInstruction(0xD5, op, 4, AddressingMode::ZeroPageX);
  addInstruction(0xCD, op, 4, AddressingMode::Absolute);
  addInstruction(0xDD, op, 4, AddressingMode::AbsoluteX, true);
  addInstruction(0xD9, op, 4, AddressingMode::AbsoluteY, true);
  addInstruction(0xC1, op, 6, AddressingMode::IndexedIndirect);
  addInstruction(0xD1, op, 5, AddressingMode::IndirectIndexed, true);

  // CPX (Compare X Register)
  op = Operation::CPX;
  addInstruction(0xE0, op, 2, AddressingMode::Immediate);
  addInstruction(0xE4, op, 3, AddressingMode::ZeroPage);
  addInstruction(0xEC, op, 4, AddressingMode::Absolute);

  // CPY (Compare Y Register)
  op = Operation::CPY;
  addInstruction(0xC0, op, 2, AddressingMode::Immediate);
  addInstruction(0xC4, op, 3, AddressingMode::ZeroPage);
  addInstruction(0xCC, op, 4, AddressingMode::Absolute);

  // DEC (Decrement Memory)
  op = Operation::DEC;
  addInstruction(0xC6, op, 5, AddressingMode::ZeroPage);
  addInstruction(0xD6, op, 6, AddressingMode::ZeroPageX);
  addInstruction(0xCE, op, 6, AddressingMode::Absolute);
  addInstruction(0xDE, op, 7, AddressingMode::AbsoluteX);

  // DEX (Decrement X Register)
  op = Operation::DEX;
  addInstruction(0xCA, op, 2, AddressingMode::Implied);

  // DEY (Decrement Y Register)
  op = Operation::DEY;
  addInstruction(0x88, op, 2, AddressingMode::Implied);

  // EOR (Exclusive OR)
  op = Operation::EOR;
  addInstruction(0x49, op, 2, AddressingMode::Immediate);
  addInstruction(0x45, op, 3, AddressingMode::ZeroPage);
  addInstruction(0x55, op, 4, AddressingMode::ZeroPageX);
  addInstruction(0x4D, op, 4, AddressingMode::Absolute);
  addInstruction(0x5D, op, 4, AddressingMode::AbsoluteX, true);
  addInstruction(0x59, op, 4, AddressingMode::AbsoluteY, true);
  addInstruction(0x41, op, 6, AddressingMode::IndexedIndirect);
  addInstruction(0x51, op, 5, AddressingMode::IndirectIndexed, true);

  // INC (Increment Memory)
  op = Operation::INC;
  addInstruction(0xE6, op, 5, AddressingMode::ZeroPage);
  addInstruction(0xF6, op, 6, AddressingMode::ZeroPageX);
  addInstruction(0xEE, op, 6, AddressingMode::Absolute);
  addInstruction(0xFE, op, 7, AddressingMode::AbsoluteX);

  // INX (Increment X Register)
  op = Operation::INX;
  addInstruction(0xE8, op, 2, AddressingMode::Implied);

  // INY (Increment Y Register)
  op = Operation::INY;
  addInstruction(0xC8, op, 2, AddressingMode::Implied);

  // JMP (Jump)
  op = Operation::JMP;
  addInstruction(0x4C, op, 3, AddressingMode::Absolute);
  addInstruction(0x6C, op, 5, AddressingMode::Indirect);

  // JSR (Jump to Subroutine)
  op = Operation::JSR;
  addInstruction(0x20, op, 6, AddressingMode::Absolute);

  // LDA (Load Accumulator)
  op = Operation::LDA;
  addInstruction(0xA9, op, 2, AddressingMode::Immediate);
  addInstruction(0xA5, op, 3, AddressingMode::ZeroPage);
  addInstruction(0xB5, op, 4, AddressingMode::ZeroPageX);
  addInstruction(0xAD, op, 4, AddressingMode::Absolute);
  addInstruction(0xBD, op, 4, AddressingMode::AbsoluteX, true);
  addInstruction(0xB9, op, 4, AddressingMode::AbsoluteY, true);
  addInstruction(0xA1, op, 6, AddressingMode::IndexedIndirect);
  addInstruction(0xB1, op, 5, AddressingMode::IndirectIndexed, true);

  // LDX (Load X Register)
  op = Operation::LDX;
  addInstruction(0xA2, op, 2, AddressingMode::Immediate);
  addInstruction(0xA6, op, 3, AddressingMode::ZeroPage);
  addInstruction(0xB6, op, 4, AddressingMode::ZeroPageY);
  addInstruction(0xAE, op, 4, AddressingMode::Absolute);
  addInstruction(0xBE, op, 4, AddressingMode::AbsoluteY, true);

  // LDY (Load Y Register)
  op = Operation::LDY;
  addInstruction(0xA0, op, 2, AddressingMode::Immediate);
  addInstruction(0xA4, op, 3, AddressingMode::ZeroPage);
  addInstruction(0xB4, op, 4, AddressingMode::ZeroPageX);
  addInstruction(0xAC, op, 4, AddressingMode::Absolute);
  addInstruction(0xBC, op, 4, AddressingMode::AbsoluteX, true);

  // LSR (Logical Shift Right)
  op = Operation::LSR;
  addInstruction(0x4A, op, 2, AddressingMode::Accumulator);
  addInstruction(0x46, op, 5, AddressingMode::ZeroPage);
  addInstruction(0x56, op, 6, AddressingMode::ZeroPageX);
  addInstruction(0x4E, op, 6, AddressingMode::Absolute);
  addInstruction(0x5E, op, 7, AddressingMode::AbsoluteX);

  // NOP (No Operation)
  op = Operation::NOP;
  addInstruction(0xEA, op, 2, AddressingMode::Implied);

  // Unofficial NOPs
  addInstruction(0x1A, op, 2, AddressingMode::Implied);
  addInstruction(0x3A, op, 2, AddressingMode::Implied);
  addInstruction(0x5A, op, 2, AddressingMode::Implied);
  addInstruction(0x7A, op, 2, AddressingMode::Implied);
  addInstruction(0xDA, op, 2, AddressingMode::Implied);
  addInstruction(0xFA, op, 2, AddressingMode::Implied);
  addInstruction(0x80, op, 2, AddressingMode::Immediate);
  addInstruction(0x82, op, 2, AddressingMode::Immediate);
  addInstruction(0x89, op, 2, AddressingMode::Immediate);
  addInstruction(0xC2, op, 2, AddressingMode::Immediate);
  addInstruction(0xE2, op, 2, AddressingMode::Immediate);
  addInstruction(0x04, op, 3, AddressingMode::ZeroPage);
  addInstruction(0x44, op, 3, AddressingMode::ZeroPage);
  addInstruction(0x64, op, 3, AddressingMode::ZeroPage);
  addInstruction(0x14, op, 4, AddressingMode::ZeroPageX);
  addInstruction(0x34, op, 4, AddressingMode::ZeroPageX);
  addInstruction(0x54, op, 4, AddressingMode::ZeroPageX);
  addInstruction(0x74, op, 4, AddressingMode::ZeroPageX);
  addInstruction(0xD4, op, 4, AddressingMode::ZeroPageX);
  addInstruction(0xF4, op, 4, AddressingMode::ZeroPageX);
  addInstruction(0x0C, op, 4, AddressingMode::Absolute);
  addInstruction(0x1C, op, 4, AddressingMode::AbsoluteX);
  addInstruction(0x3C, op, 4, AddressingMode::AbsoluteX);
  addInstruction(0x5C, op, 4, AddressingMode::AbsoluteX);
  addInstruction(0x7C, op, 4, AddressingMode::AbsoluteX);
  addInstruction(0xDC, op, 4, AddressingMode::AbsoluteX);
  addInstruction(0xFC, op, 4, AddressingMode::AbsoluteX);

  // ORA (Logical Inclusive OR)
  op = Operation::ORA;
  addInstruction(0x09, op, 2, AddressingMode::Immediate);
  addInstruction(0x05, op, 3, AddressingMode::ZeroPage);
  addInstruction(0x15, op, 4, AddressingMode::ZeroPageX);
  addInstruction(0x0D, op, 4, AddressingMode::Absolute);
  addInstruction(0x1D, op, 4, AddressingMode::AbsoluteX, true);
  addInstruction(0x19, op, 4, AddressingMode::AbsoluteY, true);
  addInstruction(0x01, op, 6, AddressingMode::IndexedIndirect);
  addInstruction(0x11, op, 5, AddressingMode::IndirectIndexed, true);

  // PHA (Push Accumulator)
  op = Operation::PHA;
  addInstruction(0x48, op, 3, AddressingMode::Implied);

  // PHP (Push Processor Status)
  op = Operation::PHP;
  addInstruction(0x08, op, 3, AddressingMode::Implied);

  // PLA (Pull Accumulator)
  op = Operation::PLA;
  addInstruction(0x68, op, 4, AddressingMode::Implied);

  // PLP (Pull Processor Status)
  op = Operation::PLP;
  addInstruction(0x28, op, 4, AddressingMode::Implied);

  // ROL (Rotate Left)
  op = Operation::ROL;
  addInstruction(0x2A, op, 2, AddressingMode::Accumulator);
  addInstruction(0x26, op, 5, AddressingMode::ZeroPage);
  addInstruction(0x36, op, 6, AddressingMode::ZeroPageX);
  addInstruction(0x2E, op, 6, AddressingMode::Absolute);
  addInstruction(0x3E, op, 7, AddressingMode::AbsoluteX);

  // ROR (Rotate Right)
  op = Operation::ROR;
  addInstruction(0x6A, op, 2, AddressingMode::Accumulator);
  addInstruction(0x66, op, 5, AddressingMode::ZeroPage);
  addInstruction(0x76, op, 6, AddressingMode::ZeroPageX);
  addInstruction(0x6E, op, 6, AddressingMode::Absolute);
  addInstruction(0x7E, op, 7, AddressingMode::AbsoluteX);

  // RTI (Return from Interrupt)
  op = Operation::RTI;
  addInstruction(0x40, op, 6, AddressingMode::Implied);

  // RTS (Return from Subroutine)
  op = Operation::RTS;
  addInstruction(0x60, op, 6, AddressingMode::Implied);

  // SBC (Subtract with Carry)
  op = Operation::SBC;
  addInstruction(0xE9, op, 2, AddressingMode::Immediate);
  addInstruction(0xE5, op, 3, AddressingMode::ZeroPage);
  addInstruction(0xF5, op, 4, AddressingMode::ZeroPageX);
  addInstruction(0xED, op, 4, AddressingMode::Absolute);
  addInstruction(0xFD, op, 4, AddressingMode::AbsoluteX, true);
  addInstruction(0xF9, op, 4, AddressingMode::AbsoluteY, true);
  addInstruction(0xE1, op, 6, AddressingMode::IndexedIndirect);
  addInstruction(0xF1, op, 5, AddressingMode::IndirectIndexed, true);

  // SEC (Set Carry Flag)
  op = Operation::SEC;
  addInstruction(0x38, op, 2, AddressingMode::Implied);

  // SED (Set Decimal Flag)
  op = Operation::SED;
  addInstruction(0xF8, op, 2, AddressingMode::Implied);

  // SEI (Set Interrupt Disable)
  op = Operation::SEI;
  addInstruction(0x78, op, 2, AddressingMode::Implied);

  // STA (Store Accumulator)
  op = Operation::STA;
  addInstruction(0x85, op, 3, AddressingMode::ZeroPage);
  addInstruction(0x95, op, 4, AddressingMode::ZeroPageX);
  addInstruction(0x8D, op, 4, AddressingMode::Absolute);
  addInstruction(0x9D, op, 5, AddressingMode::AbsoluteX);
  addInstruction(0x99, op, 5, AddressingMode::AbsoluteY);
  addInstruction(0x81, op, 6, AddressingMode::IndexedIndirect);
  addInstruction(0x91, op, 6, AddressingMode::IndirectIndexed);

  // STX (Store X Register)
  op = Operation::STX;
  addInstruction(0x86, op, 3, AddressingMode::ZeroPage);
  addInstruction(0x96, op, 4, AddressingMode::ZeroPageY);
  addInstruction(0x8E, op, 4, AddressingMode::Absolute);

  // STY (Store Y Register)
  op = Operation::STY;
  addInstruction(0x84, op, 3, AddressingMode::ZeroPage);
  addInstruction(0x94, op, 4, AddressingMode::ZeroPageX);
  addInstruction(0x8C, op, 4, AddressingMode::Absolute);

  // TAX (Transfer Accumulator to X)
  op = Operation::TAX;
  addInstruction(0xAA, op, 2, AddressingMode::Implied);

  // TAY (Transfer Accumulator to Y)
  op = Operation::TAY;
  addInstruction(0xA8, op, 2, AddressingMode::Implied);

  // TSX (Transfer Stack Pointer to X)
  op = Operation::TSX;
  addInstruction(0xBA, op, 2, AddressingMode::Implied);

  // TXA (Transfer X to Accumulator)
  op = Operation::TXA;
  addInstruction(0x8A, op, 2, AddressingMode::Implied);

  // TXS (Transfer X to Stack Pointer)
  op = Operation::TXS;
  addInstruction(0x9A, op, 2, AddressingMode::Implied);

  // TYA (Transfer Y to Accumulator)
  op = Operation::TYA;
  addInstruction(0x98, op, 2, AddressingMode::Implied);

  //
  // Unofficial Opcodes
  //

  // SLO (Shift Left then OR)
  op = Operation::SLO;
  addInstruction(0x07, op, 5, AddressingMode::ZeroPage);
  addInstruction(0x17, op, 6, AddressingMode::ZeroPageX);
  addInstruction(0x0F, op, 6, AddressingMode::Absolute);
  addInstruction(0x1F, op, 7, AddressingMode::AbsoluteX);
  addInstruction(0x1B, op, 7, AddressingMode::AbsoluteY);
  addInstruction(0x03, op, 8, AddressingMode::IndexedIndirect);
  addInstruction(0x13, op, 8, AddressingMode::IndirectIndexed);

  // RLA (Rotate Left then AND)
  op = Operation::RLA;
  addInstruction(0x27, op, 5, AddressingMode::ZeroPage);
  addInstruction(0x37, op, 6, AddressingMode::ZeroPageX);
  addInstruction(0x2F, op, 6, AddressingMode::Absolute);
  addInstruction(0x3F, op, 7, AddressingMode::AbsoluteX);
  addInstruction(0x3B, op, 7, AddressingMode::AbsoluteY);
  addInstruction(0x23, op, 8, AddressingMode::IndexedIndirect);
  addInstruction(0x33, op, 8, AddressingMode::IndirectIndexed);

  // SRE (Shift Right then EOR)
  op = Operation::SRE;
  addInstruction(0x47, op, 5, AddressingMode::ZeroPage);
  addInstruction(0x57, op, 6, AddressingMode::ZeroPageX);
  addInstruction(0x4F, op, 6, AddressingMode::Absolute);
  addInstruction(0x5F, op, 7, AddressingMode::AbsoluteX);
  addInstruction(0x5B, op, 7, AddressingMode::AbsoluteY);
  addInstruction(0x43, op, 8, AddressingMode::IndexedIndirect);
  addInstruction(0x53, op, 8, AddressingMode::IndirectIndexed);

  // RRA (Rotate Right then ADC)
  op = Operation::RRA;
  addInstruction(0x67, op, 5, AddressingMode::ZeroPage);
  addInstruction(0x77, op, 6, AddressingMode::ZeroPageX);
  addInstruction(0x6F, op, 6, AddressingMode::Absolute);
  addInstruction(0x7F, op, 7, AddressingMode::AbsoluteX);
  addInstruction(0x7B, op, 7, AddressingMode::AbsoluteY);
  addInstruction(0x63, op, 8, AddressingMode::IndexedIndirect);
  addInstruction(0x73, op, 8, AddressingMode::IndirectIndexed);

  // SAX (Store A AND X)
  op = Operation::SAX;
  addInstruction(0x87, op, 3, AddressingMode::ZeroPage);
  addInstruction(0x97, op, 4, AddressingMode::ZeroPageY);
  addInstruction(0x8F, op, 4, AddressingMode::Absolute);
  addInstruction(0x83, op, 6, AddressingMode::IndexedIndirect);

  // LAX (Load A and X)
  op = Operation::LAX;
  addInstruction(0xA7, op, 3, AddressingMode::ZeroPage);
  addInstruction(0xB7, op, 4, AddressingMode::ZeroPageY);
  addInstruction(0xAF, op, 4, AddressingMode::Absolute);
  addInstruction(0xBF, op, 4, AddressingMode::AbsoluteY, true);
  addInstruction(0xA3, op, 6, AddressingMode::IndexedIndirect);
  addInstruction(0xB3, op, 5, AddressingMode::IndirectIndexed, true);

  // DCP (Decrement then CMP)
  op = Operation::DCP;
  addInstruction(0xC7, op, 5, AddressingMode::ZeroPage);
  addInstruction(0xD7, op, 6, AddressingMode::ZeroPageX);
  addInstruction(0xCF, op, 6, AddressingMode::Absolute);
  addInstruction(0xDF, op, 7, AddressingMode::AbsoluteX);
  addInstruction(0xDB, op, 7, AddressingMode::AbsoluteY);
  addInstruction(0xC3, op, 8, AddressingMode::IndexedIndirect);
  addInstruction(0xD3, op, 8, AddressingMode::IndirectIndexed);

  // ISC (Increment then SBC)
  op = Operation::ISC;
  addInstruction(0xE7, op, 5, AddressingMode::ZeroPage);
  addInstruction(0xF7, op, 6, AddressingMode::ZeroPageX);
  addInstruction(0xEF, op, 6, AddressingMode::Absolute);
  addInstruction(0xFF, op, 7, AddressingMode::AbsoluteX);
  addInstruction(0xFB, op, 7, AddressingMode::AbsoluteY);
  addInstruction(0xE3, op, 8, AddressingMode::IndexedIndirect);
  addInstruction(0xF3, op, 8, AddressingMode::IndirectIndexed);

  // ANC (AND byte with accumulator then move bit 7 to carry)
  op = Operation::ANC;
  addInstruction(0x0B, op, 2, AddressingMode::Immediate);
  addInstruction(0x2B, op, 2, AddressingMode::Immediate);

  // ALR (AND then LSR)
  op = Operation::ALR;
  addInstruction(0x4B, op, 2, AddressingMode::Immediate);

  // ARR (AND then ROR)
  op = Operation::ARR;
  addInstruction(0x6B, op, 2, AddressingMode::Immediate);

  // XAA (Transfer X to A then AND)
  op = Operation::XAA;
  addInstruction(0x8B, op, 2, AddressingMode::Immediate);

  // AXS (AND X register with accumulator then subtract byte from result)
  op = Operation::AXS;
  addInstruction(0xCB, op, 2, AddressingMode::Immediate);

  // AHX (Store A AND X AND H)
  op = Operation::AHX;
  addInstruction(0x93, op, 6, AddressingMode::IndirectIndexed);
  addInstruction(0x9F, op, 5, AddressingMode::AbsoluteY);

  // SHY (Store Y AND H)
  op = Operation::SHY;
  addInstruction(0x9C, op, 5, AddressingMode::AbsoluteX);

  // SHX (Store X AND H)
  op = Operation::SHX;
  addInstruction(0x9E, op, 5, AddressingMode::AbsoluteY);

  // TAS (AND X register with accumulator and store result in SP, then AND
  // result with contents of H and store in memory)
  op = Operation::TAS;
  addInstruction(0x9B, op, 5, AddressingMode::AbsoluteY);

  // LAS (AND memory with SP, transfer result to A, X, and SP)
  op = Operation::LAS;
  addInstruction(0xBB, op, 4, AddressingMode::AbsoluteY, true);

  // KIL (Stop program counter)
  op = Operation::KIL;
  addInstruction(0x02, op, 0, AddressingMode::Implied);
  addInstruction(0x12, op, 0, AddressingMode::Implied);
  addInstruction(0x22, op, 0, AddressingMode::Implied);
  addInstruction(0x32, op, 0, AddressingMode::Implied);
  addInstruction(0x42, op, 0, AddressingMode::Implied);
  addInstruction(0x52, op, 0, AddressingMode::Implied);
  addInstruction(0x62, op, 0, AddressingMode::Implied);
  addInstruction(0x72, op, 0, AddressingMode::Implied);
  addInstruction(0x92, op, 0, AddressingMode::Implied);
  addInstruction(0xB2, op, 0, AddressingMode::Implied);
  addInstruction(0xD2, op, 0, AddressingMode::Implied);
  addInstruction(0xF2, op, 0, AddressingMode::Implied);
}

// Helper functions
std::vector<uint8_t> CPU::get_next_operation() {
  std::vector<uint8_t> operation;

  Byte opcode = bus->cpu_read(registers.PC);
  auto it = opcodeTable_.find(opcode);
  if (it == opcodeTable_.end()) {
    throw std::runtime_error("Unknown opcode: " + std::to_string(opcode));
    // std::cout << "Unknown opcode: " + std::to_string(opcode) << "\n";
  }
  const Instruction &instruction = it->second;
  Address address = fetchAddress(instruction.mode, true);
  // Word operand = fetchOperand(instruction.mode, true);
  operation.push_back(opcode);
  if (address != 0) {
    Byte hi = static_cast<uint8_t>(
        address >> 8); // Shift right by 8 bits to get the high byte
    Byte low = static_cast<uint8_t>(
        address & 0xFF); // Mask off the high byte to get the low byte;
    operation.push_back(low);
    operation.push_back(hi);
  }
  return operation;
}

Byte CPU::fetchOperand(AddressingMode mode, bool read_only) {
  Word pc = registers.PC;
  switch (mode) {
  case AddressingMode::Implied:
  case AddressingMode::Accumulator:
    // These modes don't fetch an operand
    return 0;

  case AddressingMode::Immediate:
    return bus->cpu_read(pc++);

  case AddressingMode::ZeroPage:
    return bus->cpu_read(bus->cpu_read(pc++));

  case AddressingMode::ZeroPageX:
    return bus->cpu_read((bus->cpu_read(pc++) + registers.X) & 0xFF);

  case AddressingMode::ZeroPageY:
    return bus->cpu_read((bus->cpu_read(pc++) + registers.Y) & 0xFF);

  case AddressingMode::Relative:
    cycles_++; // branch succeeds
    return bus->cpu_read(pc++);

  case AddressingMode::Absolute: {
    Word address = bus->cpu_read(pc++) | (bus->cpu_read(pc++) << 8);
    return bus->cpu_read(address);
  }

  case AddressingMode::AbsoluteX: {
    Word address = bus->cpu_read(pc++) | (bus->cpu_read(pc++) << 8);

    return bus->cpu_read(address + registers.X);
  }

  case AddressingMode::AbsoluteY: {
    Word address = bus->cpu_read(pc++) | (bus->cpu_read(pc++) << 8);
    return bus->cpu_read(address + registers.Y);
  }

  case AddressingMode::Indirect: {
    Word pointer = bus->cpu_read(pc++) | (bus->cpu_read(pc++) << 8);
    Word address =
        bus->cpu_read(pointer) |
        (bus->cpu_read((pointer & 0xFF00) | ((pointer + 1) & 0x00FF)) << 8);
    return bus->cpu_read(address);
  }

  case AddressingMode::IndexedIndirect: {
    Byte zeroPageAddress = (bus->cpu_read(pc++) + registers.X) & 0xFF;
    Word address = bus->cpu_read(zeroPageAddress) |
                   (bus->cpu_read((zeroPageAddress + 1) & 0xFF) << 8);
    return bus->cpu_read(address);
  }

  case AddressingMode::IndirectIndexed: {
    Byte zeroPageAddress = bus->cpu_read(pc++);
    Word address = bus->cpu_read(zeroPageAddress) |
                   (bus->cpu_read((zeroPageAddress + 1) & 0xFF) << 8);
    return bus->cpu_read(address + registers.Y);
  }
  default:
    throw std::runtime_error("Unimplemented addressing mode");
  }
  if (!read_only) {
    registers.PC = pc;
  }
}
Word CPU::fetchAddress(AddressingMode mode, bool read_only) {
  Word pc = registers.PC;
  Word address{};
  switch (mode) {
  case AddressingMode::Implied:
  case AddressingMode::Accumulator:
  case AddressingMode::Immediate:
    // These modes don't use an address
    address = 0;
    break;
  case AddressingMode::ZeroPage:
    address = bus->cpu_read(pc++);
    break;
  case AddressingMode::ZeroPageX:
    address = (bus->cpu_read(pc++) + registers.X) & 0xFF;
    break;
  case AddressingMode::ZeroPageY:
    address = (bus->cpu_read(pc++) + registers.Y) & 0xFF;
    break;
  case AddressingMode::Relative: {
    Byte offset = bus->cpu_read(pc++);
    // Convert to signed offset
    address = registers.PC + static_cast<int8_t>(offset);
    break;
  }
  case AddressingMode::Absolute:
    address = bus->cpu_read(pc++) | (bus->cpu_read(pc++) << 8);
    break;
  case AddressingMode::AbsoluteX: {
    Word base = bus->cpu_read(pc++) | (bus->cpu_read(pc++) << 8);
    address = base + registers.X;
    break;
  }
  case AddressingMode::AbsoluteY: {
    Word base = bus->cpu_read(pc++) | (bus->cpu_read(pc++) << 8);
    address = base + registers.Y;
    break;
  }
  case AddressingMode::Indirect: {
    Word pointer = bus->cpu_read(pc++) | (bus->cpu_read(pc++) << 8);
    // Simulate the 6502 bug with indirect jump
    if ((pointer & 0xFF) == 0xFF) {
      address = bus->cpu_read(pointer) | (bus->cpu_read(pointer & 0xFF00) << 8);
    } else {
      address = bus->cpu_read(pointer) | (bus->cpu_read(pointer + 1) << 8);
    }
    break;
  }
  case AddressingMode::IndexedIndirect: {
    Byte zeroPageAddress = (bus->cpu_read(pc++) + registers.X) & 0xFF;
    address = bus->cpu_read(zeroPageAddress) |
              (bus->cpu_read((zeroPageAddress + 1) & 0xFF) << 8);
    break;
  }
  case AddressingMode::IndirectIndexed: {
    Byte zeroPageAddress = bus->cpu_read(pc++);
    Word base = bus->cpu_read(zeroPageAddress) |
                (bus->cpu_read((zeroPageAddress + 1) & 0xFF) << 8);
    address = base + registers.Y;
    break;
  }
  default:
    throw std::runtime_error("Unimplemented addressing mode");
  }
  if (!read_only) {
    registers.PC = pc;
  }
  return address;
}
void CPU::updateZeroAndNegativeFlags(Byte value) {
  registers.P.Z = (value == 0);
  registers.P.N = (value & 0x80) != 0;
}

void CPU::handlePageCrossing(const Instruction &instruction) {
  Word baseAddress = fetchAddress(instruction.mode);
  registers.PC--;
  Byte finalAddress{0};
  switch (instruction.mode) {
  case AddressingMode::AbsoluteX:
    finalAddress = bus->cpu_read(baseAddress + registers.X);
    break;
  case AddressingMode::AbsoluteY:
    finalAddress = bus->cpu_read(baseAddress + registers.Y);
    break;
  case AddressingMode::IndirectIndexed:
    finalAddress = bus->cpu_read(baseAddress + registers.Y);
  }
  if ((baseAddress & 0xFF00) != (finalAddress & 0xFF00)) {
    // Page boundary crossed, add an extra cycle
    cycles_++;
  }
}
