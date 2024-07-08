#include <iostream>
#include <m6502.h>
#include <regex>
#include <stdexcept>

namespace m6502 {

void CPU::reset() {
  registers_.reset();
  memory_.fill(0);
  initializeTables();
  for (const auto &entry : opcodeTable_) {
    std::cout << "Opcode: " << std::hex << static_cast<int>(entry.first)
              << ", Mnemonic: " << entry.second.mnemonic
              << ", Cycles: " << std::dec << entry.second.cycles
              << ", Mode: " << static_cast<int>(entry.second.mode) << std::endl;
  }
  for (const auto &entry : executeTable_) {
    std::cout << "Mnemonic: " << entry.first << std::endl;
  }
}

void CPU::step() {
  // Fetch
  Byte opcode = read(registers_.PC);
  registers_.PC++;

  // Decode
  std::cout << std::hex << static_cast<int>(opcode) << std::endl;
  auto it = opcodeTable_.find(opcode);
  if (it == opcodeTable_.end()) {
    throw std::runtime_error("Unknown opcode");
  }
  const Instruction &instruction = it->second;

  // Execute
  auto execIt = executeTable_.find(instruction.mnemonic);
  if (execIt == executeTable_.end()) {
    throw std::runtime_error("Unimplemented instruction");
  }
  execIt->second(*this, instruction);
}

void CPU::run() {
  reset();
  uint8_t program[] = {
      0xA9, 0x05, // LDA #$05    ; Load 5 into accumulator
      0x85, 0x10, // STA $10     ; Store accumulator to zero page address $10
      0xA2, 0x0A, // LDX #$0A    ; Load 10 into X register
      0xA0, 0x15, // LDY #$15    ; Load 21 into Y register
      0x18,       // CLC         ; Clear carry flag
      0x69, 0x03, // ADC #$03    ; Add 3 to accumulator (should be 8 now)
      0x65, 0x10, // ADC $10     ; Add value at zero page address $10 (5) to
                  // accumulator (should be 13 now)
      0x8D, 0x00,
      0x20, // STA $2000   ; Store accumulator to absolute address $2000
      0xAD, 0x00, 0x20, // LDA $2000   ; Load from absolute address $2000
      0x29, 0x0F,       // AND #$0F    ; AND accumulator with 0F (should be 0D)
      0x49, 0xFF,       // EOR #$FF    ; XOR accumulator with FF (should be F2)
      0x09, 0x03,       // ORA #$03    ; OR accumulator with 03 (should be F3)
      0xC9,
      0xF0, // CMP #$F0    ; Compare accumulator with F0 (should set carry flag)
      0xB0, 0x02, // BCS +2      ; Branch if carry set (should branch)
      0xA9, 0x00, // LDA #$00    ; This should be skipped
      0xE8,       // INX         ; Increment X (should be 11 now)
      0xCA,       // DEX         ; Decrement X (should be 10 again)
      0xE0, 0x0A, // CPX #$0A    ; Compare X with 0A (should set zero flag)
      0xF0, 0x02, // BEQ +2      ; Branch if zero (should branch)
      0xA2, 0x00, // LDX #$00    ; This should be skipped
      0x38,       // SEC         ; Set carry flag
      0xA9, 0x05, // LDA #$05    ; Load 5 into accumulator
      0xE9, 0x03, // SBC #$03    ; Subtract 3 from accumulator (should be 2)
      0x0A, // ASL A       ; Arithmetic shift left accumulator (should be 4)
      0x4A, // LSR A       ; Logical shift right accumulator (should be 2)
      0x2A, // ROL A       ; Rotate left accumulator (should be 4 with carry
            // set)
      0x6A, // ROR A       ; Rotate right accumulator (should be 2 with carry
            // set)
      0xA9, 0xFF, // LDA #$FF    ; Load FF into accumulator
      0x48,       // PHA         ; Push accumulator to stack
      0xA9, 0x00, // LDA #$00    ; Load 00 into accumulator
      0x68, // PLA         ; Pull from stack to accumulator (should be FF again)
      0x20, 0x50, 0x00, // JSR $0050   ; Jump to subroutine at $0050
      0xEA,             // NOP         ; No operation
      0x00,             // BRK         ; Break
      // Subroutine at $0050
      0xE6, 0x10, // INC $10     ; Increment value at zero page address $10
      0xC6, 0x10, // DEC $10     ; Decrement value at zero page address $10
      0x60        // RTS         ; Return from subroutine
  };
  int i{0};
  for (const auto &byte : program) {
    write(i, byte);
    i++;
  }
  //  memory_[0] = 0xEA;
  while (true) {
    step();
    // Add any necessary break conditions
  }
}

CPU::AddressingMode CPU::detectAddressingMode(const std::string &instruction) {
  static const std::array<std::pair<std::regex, AddressingMode>, 11> patterns =
      {{{std::regex(R"(#\$[0-9A-Fa-f]{2})"), AddressingMode::Immediate},
        {std::regex(R"(\$[0-9A-Fa-f]{2}\s*,\s*X)"), AddressingMode::ZeroPageX},
        {std::regex(R"(\$[0-9A-Fa-f]{2}\s*,\s*Y)"), AddressingMode::ZeroPageY},
        {std::regex(R"(\$[0-9A-Fa-f]{2}(?!\s*,))"), AddressingMode::ZeroPage},
        {std::regex(R"(\$[0-9A-Fa-f]{4}\s*,\s*X)"), AddressingMode::AbsoluteX},
        {std::regex(R"(\$[0-9A-Fa-f]{4}\s*,\s*Y)"), AddressingMode::AbsoluteY},
        {std::regex(R"(\$[0-9A-Fa-f]{4}(?!\s*,))"), AddressingMode::Absolute},
        {std::regex(R"(\(\$[0-9A-Fa-f]{2}\s*,\s*X\))"),
         AddressingMode::IndexedIndirect},
        {std::regex(R"(\(\$[0-9A-Fa-f]{2}\)\s*,\s*Y)"),
         AddressingMode::IndirectIndexed},
        {std::regex(R"(\(\$[0-9A-Fa-f]{4}\))"), AddressingMode::Indirect},
        {std::regex(R"(^[A-Z]{3}\s+A$)"), AddressingMode::Accumulator}}};

  // Check for Implied mode first (no operands)
  if (instruction.find_first_of(" \t") == std::string::npos) {
    return AddressingMode::Implied;
  }

  // Check against all other patterns
  for (const auto &[regex, mode] : patterns) {
    if (std::regex_search(instruction, regex)) {
      return mode;
    }
  }

  // If no match found, assume it's Relative (for branch instructions)
  return AddressingMode::Relative;
}

// Byte CPU::read(Word address) const { return memory_[address]; }

// Word CPU::readWord(Word &address) {
//   Word lowByte = static_cast<Word>(read(address++));
//   Word highByte = static_cast<Word>(read(address++)) << 8;
//   return lowByte | highByte;
// }

void CPU::write(Word address, Byte value) { memory_[address] = value; }

void CPU::initializeExecuteTable() {
  executeTable_["LDA"] = &CPU::executeLDA;
  executeTable_["LDX"] = &CPU::executeLDX;
  executeTable_["LDY"] = &CPU::executeLDY;
  executeTable_["STA"] = &CPU::executeSTA;
  executeTable_["STX"] = &CPU::executeSTX;
  executeTable_["STY"] = &CPU::executeSTY;
  executeTable_["TAX"] = &CPU::executeTAX;
  executeTable_["TAY"] = &CPU::executeTAY;
  executeTable_["TXA"] = &CPU::executeTXA;
  executeTable_["TYA"] = &CPU::executeTYA;
  executeTable_["TSX"] = &CPU::executeTSX;
  executeTable_["TXS"] = &CPU::executeTXS;
  executeTable_["PHA"] = &CPU::executePHA;
  executeTable_["PHP"] = &CPU::executePHP;
  executeTable_["PLA"] = &CPU::executePLA;
  executeTable_["PLP"] = &CPU::executePLP;
  executeTable_["AND"] = &CPU::executeAND;
  executeTable_["EOR"] = &CPU::executeEOR;
  executeTable_["ORA"] = &CPU::executeORA;
  executeTable_["BIT"] = &CPU::executeBIT;
  executeTable_["ADC"] = &CPU::executeADC;
  executeTable_["SBC"] = &CPU::executeSBC;
  executeTable_["CMP"] = &CPU::executeCMP;
  executeTable_["CPX"] = &CPU::executeCPX;
  executeTable_["CPY"] = &CPU::executeCPY;
  executeTable_["INC"] = &CPU::executeINC;
  executeTable_["INX"] = &CPU::executeINX;
  executeTable_["INY"] = &CPU::executeINY;
  executeTable_["DEC"] = &CPU::executeDEC;
  executeTable_["DEX"] = &CPU::executeDEX;
  executeTable_["DEY"] = &CPU::executeDEY;
  executeTable_["ASL"] = &CPU::executeASL;
  executeTable_["LSR"] = &CPU::executeLSR;
  executeTable_["ROL"] = &CPU::executeROL;
  executeTable_["ROR"] = &CPU::executeROR;
  executeTable_["JMP"] = &CPU::executeJMP;
  executeTable_["JSR"] = &CPU::executeJSR;
  executeTable_["RTS"] = &CPU::executeRTS;
  executeTable_["BCC"] = &CPU::executeBCC;
  executeTable_["BCS"] = &CPU::executeBCS;
  executeTable_["BEQ"] = &CPU::executeBEQ;
  executeTable_["BMI"] = &CPU::executeBMI;
  executeTable_["BNE"] = &CPU::executeBNE;
  executeTable_["BPL"] = &CPU::executeBPL;
  executeTable_["BVC"] = &CPU::executeBVC;
  executeTable_["BVS"] = &CPU::executeBVS;
  executeTable_["CLC"] = &CPU::executeCLC;
  executeTable_["CLD"] = &CPU::executeCLD;
  executeTable_["CLI"] = &CPU::executeCLI;
  executeTable_["CLV"] = &CPU::executeCLV;
  executeTable_["SEC"] = &CPU::executeSEC;
  executeTable_["SED"] = &CPU::executeSED;
  executeTable_["SEI"] = &CPU::executeSEI;
  executeTable_["BRK"] = &CPU::executeBRK;
  executeTable_["NOP"] = &CPU::executeNOP;
  executeTable_["RTI"] = &CPU::executeRTI;
}

void CPU::initializeTables() {
  initializeExecuteTable();
  auto addInstruction = [this](Byte opcode, const std::string &mnemonic,
                               std::uint32_t cycles, AddressingMode mode) {
    Instruction inst{opcode, mnemonic, cycles, mode};
    opcodeTable_[opcode] = inst;
    translationTable_[InstructionKey{mnemonic, mode}] = inst;
  };

  // LDA (Load Accumulator)
  addInstruction(0xA9, "LDA", 2, AddressingMode::Immediate);
  addInstruction(0xA5, "LDA", 3, AddressingMode::ZeroPage);
  addInstruction(0xB5, "LDA", 4, AddressingMode::ZeroPageX);
  addInstruction(0xAD, "LDA", 4, AddressingMode::Absolute);
  addInstruction(0xBD, "LDA", 4, AddressingMode::AbsoluteX);
  addInstruction(0xB9, "LDA", 4, AddressingMode::AbsoluteY);
  addInstruction(0xA1, "LDA", 6, AddressingMode::IndexedIndirect);
  addInstruction(0xB1, "LDA", 5, AddressingMode::IndirectIndexed);

  // STA (Store Accumulator)
  addInstruction(0x85, "STA", 3, AddressingMode::ZeroPage);
  addInstruction(0x95, "STA", 4, AddressingMode::ZeroPageX);
  addInstruction(0x8D, "STA", 4, AddressingMode::Absolute);
  addInstruction(0x9D, "STA", 5, AddressingMode::AbsoluteX);
  addInstruction(0x99, "STA", 5, AddressingMode::AbsoluteY);
  addInstruction(0x81, "STA", 6, AddressingMode::IndexedIndirect);
  addInstruction(0x91, "STA", 6, AddressingMode::IndirectIndexed);

  // ADC (Add with Carry)
  addInstruction(0x69, "ADC", 2, AddressingMode::Immediate);
  addInstruction(0x65, "ADC", 3, AddressingMode::ZeroPage);
  addInstruction(0x75, "ADC", 4, AddressingMode::ZeroPageX);
  addInstruction(0x6D, "ADC", 4, AddressingMode::Absolute);
  addInstruction(0x7D, "ADC", 4, AddressingMode::AbsoluteX);
  addInstruction(0x79, "ADC", 4, AddressingMode::AbsoluteY);
  addInstruction(0x61, "ADC", 6, AddressingMode::IndexedIndirect);
  addInstruction(0x71, "ADC", 5, AddressingMode::IndirectIndexed);

  // JMP (Jump)
  addInstruction(0x4C, "JMP", 3, AddressingMode::Absolute);
  addInstruction(0x6C, "JMP", 5, AddressingMode::Indirect);

  // JSR (Jump to Subroutine)
  addInstruction(0x20, "JSR", 6, AddressingMode::Absolute);

  // RTS (Return from Subroutine)
  addInstruction(0x60, "RTS", 6, AddressingMode::Implied);

  // INX (Increment X Register)
  addInstruction(0xE8, "INX", 2, AddressingMode::Implied);

  // INY (Increment Y Register)
  addInstruction(0xC8, "INY", 2, AddressingMode::Implied);

  // NOP (No Operation)
  addInstruction(0xEA, "NOP", 2, AddressingMode::Implied);

  // CLC (Clear Carry Flag)
  addInstruction(0x18, "CLC", 2, AddressingMode::Implied);

  // SEC (Set Carry Flag)
  addInstruction(0x38, "SEC", 2, AddressingMode::Implied);
}

// Load/Store Operations
void CPU::LDA() {
  Byte value = fetchOperand(opcodeTable_[read(registers_.PC - 1)].mode);
  registers_.A = value;
  updateZeroAndNegativeFlags(registers_.A);
}

void CPU::LDX() {
  Byte value = fetchOperand(opcodeTable_[read(registers_.PC - 1)].mode);
  registers_.X = value;
  updateZeroAndNegativeFlags(registers_.X);
}

void CPU::LDY() {
  Byte value = fetchOperand(opcodeTable_[read(registers_.PC - 1)].mode);
  registers_.Y = value;
  updateZeroAndNegativeFlags(registers_.Y);
}

void CPU::STA() {
  Word address = fetchAddress(opcodeTable_[read(registers_.PC - 1)].mode);
  write(address, registers_.A);
}

void CPU::STX() {
  Word address = fetchAddress(opcodeTable_[read(registers_.PC - 1)].mode);
  write(address, registers_.X);
}

void CPU::STY() {
  Word address = fetchAddress(opcodeTable_[read(registers_.PC - 1)].mode);
  write(address, registers_.Y);
}

// Transfer Operations
void CPU::TAX() {
  registers_.X = registers_.A;
  updateZeroAndNegativeFlags(registers_.X);
}

void CPU::TAY() {
  registers_.Y = registers_.A;
  updateZeroAndNegativeFlags(registers_.Y);
}

void CPU::TXA() {
  registers_.A = registers_.X;
  updateZeroAndNegativeFlags(registers_.A);
}

void CPU::TYA() {
  registers_.A = registers_.Y;
  updateZeroAndNegativeFlags(registers_.A);
}

// Stack Operations
void CPU::TSX() {
  registers_.X = registers_.SP;
  updateZeroAndNegativeFlags(registers_.X);
}

void CPU::TXS() { registers_.SP = registers_.X; }

void CPU::PHA() {
  write(0x100 + registers_.SP, registers_.A);
  registers_.SP--;
}

void CPU::PHP() {
  Byte status = (registers_.P.N << 7) | (registers_.P.V << 6) |
                (registers_.P.U << 5) | (registers_.P.B << 4) |
                (registers_.P.D << 3) | (registers_.P.I << 2) |
                (registers_.P.Z << 1) | registers_.P.C;
  write(0x100 + registers_.SP, status);
  registers_.SP--;
}

void CPU::PLA() {
  registers_.SP++;
  registers_.A = read(0x100 + registers_.SP);
  updateZeroAndNegativeFlags(registers_.A);
}

void CPU::PLP() {
  registers_.SP++;
  Byte status = read(0x100 + registers_.SP);
  registers_.P.N = (status & 0x80) != 0;
  registers_.P.V = (status & 0x40) != 0;
  registers_.P.U = (status & 0x20) != 0;
  registers_.P.B = (status & 0x10) != 0;
  registers_.P.D = (status & 0x08) != 0;
  registers_.P.I = (status & 0x04) != 0;
  registers_.P.Z = (status & 0x02) != 0;
  registers_.P.C = (status & 0x01) != 0;
}

// Logical Operations
void CPU::AND() {
  Byte value = fetchOperand(opcodeTable_[read(registers_.PC - 1)].mode);
  registers_.A &= value;
  updateZeroAndNegativeFlags(registers_.A);
}

void CPU::EOR() {
  Byte value = fetchOperand(opcodeTable_[read(registers_.PC - 1)].mode);
  registers_.A ^= value;
  updateZeroAndNegativeFlags(registers_.A);
}

void CPU::ORA() {
  Byte value = fetchOperand(opcodeTable_[read(registers_.PC - 1)].mode);
  registers_.A |= value;
  updateZeroAndNegativeFlags(registers_.A);
}

void CPU::BIT() {
  Byte value = fetchOperand(opcodeTable_[read(registers_.PC - 1)].mode);
  registers_.P.Z = (registers_.A & value) == 0;
  registers_.P.N = (value & 0x80) != 0;
  registers_.P.V = (value & 0x40) != 0;
}

// Arithmetic Operations
void CPU::ADC() {
  Byte value = fetchOperand(opcodeTable_[read(registers_.PC - 1)].mode);
  Word result = registers_.A + value + (registers_.P.C ? 1 : 0);

  registers_.P.C = result > 0xFF;
  registers_.P.V = ((registers_.A ^ result) & (value ^ result) & 0x80) != 0;
  registers_.A = result & 0xFF;
  updateZeroAndNegativeFlags(registers_.A);
}

void CPU::SBC() {
  Byte value = fetchOperand(opcodeTable_[read(registers_.PC - 1)].mode);
  Word result = registers_.A - value - (registers_.P.C ? 0 : 1);

  registers_.P.C = result < 0x100;
  registers_.P.V =
      ((registers_.A ^ result) & ((0xFF - value) ^ result) & 0x80) != 0;
  registers_.A = result & 0xFF;
  updateZeroAndNegativeFlags(registers_.A);
}

void CPU::CMP() {
  Byte value = fetchOperand(opcodeTable_[read(registers_.PC - 1)].mode);
  Word result = registers_.A - value;

  registers_.P.C = registers_.A >= value;
  updateZeroAndNegativeFlags(result & 0xFF);
}

void CPU::CPX() {
  Byte value = fetchOperand(opcodeTable_[read(registers_.PC - 1)].mode);
  Word result = registers_.X - value;

  registers_.P.C = registers_.X >= value;
  updateZeroAndNegativeFlags(result & 0xFF);
}

void CPU::CPY() {
  Byte value = fetchOperand(opcodeTable_[read(registers_.PC - 1)].mode);
  Word result = registers_.Y - value;

  registers_.P.C = registers_.Y >= value;
  updateZeroAndNegativeFlags(result & 0xFF);
}

// Increments/Decrements
void CPU::INC() {
  Word address = fetchAddress(opcodeTable_[read(registers_.PC - 1)].mode);
  Byte value = read(address);
  value++;
  write(address, value);
  updateZeroAndNegativeFlags(value);
}

void CPU::INX() {
  registers_.X++;
  updateZeroAndNegativeFlags(registers_.X);
}

void CPU::INY() {
  registers_.Y++;
  updateZeroAndNegativeFlags(registers_.Y);
}

void CPU::DEC() {
  Word address = fetchAddress(opcodeTable_[read(registers_.PC - 1)].mode);
  Byte value = read(address);
  value--;
  write(address, value);
  updateZeroAndNegativeFlags(value);
}

void CPU::DEX() {
  registers_.X--;
  updateZeroAndNegativeFlags(registers_.X);
}

void CPU::DEY() {
  registers_.Y--;
  updateZeroAndNegativeFlags(registers_.Y);
}

// Shifts
void CPU::ASL() {
  if (opcodeTable_[read(registers_.PC - 1)].mode ==
      AddressingMode::Accumulator) {
    registers_.P.C = (registers_.A & 0x80) != 0;
    registers_.A <<= 1;
    updateZeroAndNegativeFlags(registers_.A);
  } else {
    Word address = fetchAddress(opcodeTable_[read(registers_.PC - 1)].mode);
    Byte value = read(address);
    registers_.P.C = (value & 0x80) != 0;
    value <<= 1;
    write(address, value);
    updateZeroAndNegativeFlags(value);
  }
}

void CPU::LSR() {
  if (opcodeTable_[read(registers_.PC - 1)].mode ==
      AddressingMode::Accumulator) {
    registers_.P.C = (registers_.A & 0x01) != 0;
    registers_.A >>= 1;
    updateZeroAndNegativeFlags(registers_.A);
  } else {
    Word address = fetchAddress(opcodeTable_[read(registers_.PC - 1)].mode);
    Byte value = read(address);
    registers_.P.C = (value & 0x01) != 0;
    value >>= 1;
    write(address, value);
    updateZeroAndNegativeFlags(value);
  }
}

void CPU::ROL() {
  if (opcodeTable_[read(registers_.PC - 1)].mode ==
      AddressingMode::Accumulator) {
    Byte oldCarry = registers_.P.C ? 1 : 0;
    registers_.P.C = (registers_.A & 0x80) != 0;
    registers_.A = (registers_.A << 1) | oldCarry;
    updateZeroAndNegativeFlags(registers_.A);
  } else {
    Word address = fetchAddress(opcodeTable_[read(registers_.PC - 1)].mode);
    Byte value = read(address);
    Byte oldCarry = registers_.P.C ? 1 : 0;
    registers_.P.C = (value & 0x80) != 0;
    value = (value << 1) | oldCarry;
    write(address, value);
    updateZeroAndNegativeFlags(value);
  }
}

void CPU::ROR() {
  if (opcodeTable_[read(registers_.PC - 1)].mode ==
      AddressingMode::Accumulator) {
    Byte oldCarry = registers_.P.C ? 0x80 : 0;
    registers_.P.C = (registers_.A & 0x01) != 0;
    registers_.A = (registers_.A >> 1) | oldCarry;
    updateZeroAndNegativeFlags(registers_.A);
  } else {
    Word address = fetchAddress(opcodeTable_[read(registers_.PC - 1)].mode);
    Byte value = read(address);
    Byte oldCarry = registers_.P.C ? 0x80 : 0;
    registers_.P.C = (value & 0x01) != 0;
    value = (value >> 1) | oldCarry;
    write(address, value);
    updateZeroAndNegativeFlags(value);
  }
}

// Jumps and Calls
void CPU::JMP() {
  Word address = fetchAddress(opcodeTable_[read(registers_.PC - 1)].mode);
  registers_.PC = address;
}

void CPU::JSR() {
  registers_.PC--;
  write(0x100 + registers_.SP, (registers_.PC >> 8) & 0xFF);
  registers_.SP--;
  write(0x100 + registers_.SP, registers_.PC & 0xFF);
  registers_.SP--;
  registers_.PC = fetchAddress(AddressingMode::Absolute);
}

void CPU::RTS() {
  registers_.SP++;
  Word low = read(0x100 + registers_.SP);
  registers_.SP++;
  Word high = read(0x100 + registers_.SP);
  registers_.PC = (high << 8) | low;
  registers_.PC++;
}

// Branches
void CPU::BCC() {
  if (!registers_.P.C) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers_.PC += static_cast<int8_t>(offset);
  }
}

void CPU::BCS() {
  if (registers_.P.C) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers_.PC += static_cast<int8_t>(offset);
  }
}

void CPU::BEQ() {
  if (registers_.P.Z) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers_.PC += static_cast<int8_t>(offset);
  }
}

void CPU::BMI() {
  if (registers_.P.N) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers_.PC += static_cast<int8_t>(offset);
  }
}

void CPU::BNE() {
  if (!registers_.P.Z) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers_.PC += static_cast<int8_t>(offset);
  }
}

void CPU::BPL() {
  if (!registers_.P.N) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers_.PC += static_cast<int8_t>(offset);
  }
}

void CPU::BVC() {
  if (!registers_.P.V) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers_.PC += static_cast<int8_t>(offset);
  }
}

void CPU::BVS() {
  if (registers_.P.V) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers_.PC += static_cast<int8_t>(offset);
  }
}

// Status Flag Changes
void CPU::CLC() { registers_.P.C = false; }

void CPU::CLD() { registers_.P.D = false; }

void CPU::CLI() { registers_.P.I = false; }

void CPU::CLV() { registers_.P.V = false; }

void CPU::SEC() { registers_.P.C = true; }

void CPU::SED() { registers_.P.D = true; }

void CPU::SEI() { registers_.P.I = true; }

// System Functions
void CPU::BRK() {
  registers_.PC++;
  write(0x100 + registers_.SP, (registers_.PC >> 8) & 0xFF);
  registers_.SP--;
  write(0x100 + registers_.SP, registers_.PC & 0xFF);
  registers_.SP--;

  Byte status = (registers_.P.N << 7) | (registers_.P.V << 6) | (1 << 5) |
                (1 << 4) | (registers_.P.D << 3) | (registers_.P.I << 2) |
                (registers_.P.Z << 1) | registers_.P.C;
  write(0x100 + registers_.SP, status);
  registers_.SP--;

  registers_.P.I = true;
  registers_.PC = read(0xFFFE) | (read(0xFFFF) << 8);
}

void CPU::NOP() {
  // No operation
}

void CPU::RTI() {
  registers_.SP++;
  Byte status = read(0x100 + registers_.SP);
  registers_.P.N = (status & 0x80) != 0;
  registers_.P.V = (status & 0x40) != 0;
  registers_.P.U = (status & 0x20) != 0;
  registers_.P.B = (status & 0x10) != 0;
  registers_.P.D = (status & 0x08) != 0;
  registers_.P.I = (status & 0x04) != 0;
  registers_.P.Z = (status & 0x02) != 0;
  registers_.P.C = (status & 0x01) != 0;

  registers_.SP++;
  Word low = read(0x100 + registers_.SP);
  registers_.SP++;
  Word high = read(0x100 + registers_.SP);
  registers_.PC = (high << 8) | low;
}

} // namespace m6502