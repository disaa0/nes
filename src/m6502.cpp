#include <algorithm>
#include <cctype>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <m6502.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace m6502 {

void CPU::reset() {
  registers_.reset();
  memory_.fill(0);
  memory_[0xFFFC] = 0x00;
  memory_[0xFFFD] = 0x02;
  initializeTables();
  registers_.PC = read(0xFFFC) | (read(0xFFFD) << 8);
}

void CPU::step() {
  Byte opcode = read(registers_.PC++);
  auto it = opcodeTable_.find(opcode);
  if (it == opcodeTable_.end()) {
    throw std::runtime_error("Unknown opcode: " + std::to_string(opcode));
  }
  const Instruction &instruction = it->second;

  // Use a switch statement instead of a map for better performance
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
  default:
    throw std::runtime_error("Unimplemented instruction");
  }
}
// template <typename T>
void CPU::run(const std::vector<std::string> program, const char mode) {
  reset();
  if (mode == 'a') {
    std::vector<uint8_t> code = assemble(program);
    // Print each string in the vector
    for (const auto &byte : code) {
      std::cout << byteToHex(byte) << " ";
    }
    std::cout << std::endl;
  }
  // else {
  //   for (Byte i = 0; i < sizeof(program); ++i) {
  //     write(i + 0x0200, program[i]);
  //   }
  // }

  while (true) {
    step();
    // Add any necessary break conditions
  }
}

std::string CPU::byteToHex(uint8_t byte) {
  std::stringstream ss;
  ss << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(byte);
  return ss.str();
}

std::string CPU::wordToHex(uint16_t word) {
  std::stringstream ss;
  ss << std::setw(4) << std::setfill('0') << std::hex << word;
  return ss.str();
}

std::vector<std::string> CPU::disassemble(const std::vector<uint8_t> &code) {
  std::vector<std::string> disassembled_code;
  uint16_t pc = 0;
  std::string current_section;

  while (pc < code.size()) {
    std::stringstream line;
    // line << std::setfill('0') << std::setw(4) << std::hex << pc << "  ";

    // Check for comments or section markers
    if (code[pc] == 0xFF) {
      pc++;
      if (pc < code.size() && code[pc] == 0xFF) {
        // Section marker
        pc++;
        current_section = "";
        while (pc < code.size() && code[pc] != 0x00) {
          current_section += static_cast<char>(code[pc]);
          pc++;
        }
        pc++; // Skip the null terminator
        line << "; Section: " << current_section;
        disassembled_code.push_back(line.str());
        continue;
      } else {
        // Comment
        std::string comment;
        while (pc < code.size() && code[pc] != 0x00) {
          comment += static_cast<char>(code[pc]);
          pc++;
        }
        pc++; // Skip the null terminator
        line << "; " << comment;
        disassembled_code.push_back(line.str());
        continue;
      }
    }

    uint8_t opcode = code[pc++];
    auto it = opcodeTable_.find(opcode);
    if (it == opcodeTable_.end()) {
      throw std::runtime_error("Unknown opcode: " + std::to_string(opcode));
    }

    const Instruction &instruction = it->second;
    // line << std::setw(2) << std::hex << static_cast<int>(opcode) << "  ";
    line << mnemonicToString(instruction.mnemonic) << " ";

    std::string operand;
    uint16_t address;

    switch (instruction.mode) {
    case AddressingMode::Immediate:
      operand = "#$" + byteToHex(code[pc++]);
      break;
    case AddressingMode::ZeroPage:
      operand = "$" + byteToHex(code[pc++]);
      break;
    case AddressingMode::ZeroPageX:
      operand = "$" + byteToHex(code[pc++]) + ",X";
      break;
    case AddressingMode::ZeroPageY:
      operand = "$" + byteToHex(code[pc++]) + ",Y";
      break;
    case AddressingMode::Absolute:
      address = (code[pc + 1] << 8) | code[pc];
      operand = "$" + wordToHex(address);
      pc += 2;
      break;
    case AddressingMode::AbsoluteX:
      address = (code[pc + 1] << 8) | code[pc];
      operand = "$" + wordToHex(address) + ",X";
      pc += 2;
      break;
    case AddressingMode::AbsoluteY:
      address = (code[pc + 1] << 8) | code[pc];
      operand = "$" + wordToHex(address) + ",Y";
      pc += 2;
      break;
    case AddressingMode::Indirect:
      address = (code[pc + 1] << 8) | code[pc];
      operand = "($" + wordToHex(address) + ")";
      pc += 2;
      break;
    case AddressingMode::IndexedIndirect:
      operand = "($" + byteToHex(code[pc++]) + ",X)";
      break;
    case AddressingMode::IndirectIndexed:
      operand = "($" + byteToHex(code[pc++]) + "),Y";
      break;
    case AddressingMode::Relative: {
      int8_t offset = static_cast<int8_t>(code[pc++]);
      uint16_t target = pc + offset;
      operand = "$" + wordToHex(target);
    } break;
    case AddressingMode::Implied:
    case AddressingMode::Accumulator:
      // No operand needed
      break;
    }

    line << std::setw(10) << std::left << operand;
    // line << "  ; " << instruction.cycles << " cycles";

    disassembled_code.push_back(line.str());
  }

  return disassembled_code;
}

CPU::Operation CPU::stringToMnemonic(const std::string &str) {
  static const std::unordered_map<std::string, Operation> mnemonicMap = {
      {"ADC", Operation::ADC}, {"AND", Operation::AND}, {"ASL", Operation::ASL},
      {"BCC", Operation::BCC}, {"BCS", Operation::BCS}, {"BEQ", Operation::BEQ},
      {"BIT", Operation::BIT}, {"BMI", Operation::BMI}, {"BNE", Operation::BNE},
      {"BPL", Operation::BPL}, {"BRK", Operation::BRK}, {"BVC", Operation::BVC},
      {"BVS", Operation::BVS}, {"CLC", Operation::CLC}, {"CLD", Operation::CLD},
      {"CLI", Operation::CLI}, {"CLV", Operation::CLV}, {"CMP", Operation::CMP},
      {"CPX", Operation::CPX}, {"CPY", Operation::CPY}, {"DEC", Operation::DEC},
      {"DEX", Operation::DEX}, {"DEY", Operation::DEY}, {"EOR", Operation::EOR},
      {"INC", Operation::INC}, {"INX", Operation::INX}, {"INY", Operation::INY},
      {"JMP", Operation::JMP}, {"JSR", Operation::JSR}, {"LDA", Operation::LDA},
      {"LDX", Operation::LDX}, {"LDY", Operation::LDY}, {"LSR", Operation::LSR},
      {"NOP", Operation::NOP}, {"ORA", Operation::ORA}, {"PHA", Operation::PHA},
      {"PHP", Operation::PHP}, {"PLA", Operation::PLA}, {"PLP", Operation::PLP},
      {"ROL", Operation::ROL}, {"ROR", Operation::ROR}, {"RTI", Operation::RTI},
      {"RTS", Operation::RTS}, {"SBC", Operation::SBC}, {"SEC", Operation::SEC},
      {"SED", Operation::SED}, {"SEI", Operation::SEI}, {"STA", Operation::STA},
      {"STX", Operation::STX}, {"STY", Operation::STY}, {"TAX", Operation::TAX},
      {"TAY", Operation::TAY}, {"TSX", Operation::TSX}, {"TXA", Operation::TXA},
      {"TXS", Operation::TXS}, {"TYA", Operation::TYA}};

  auto it = mnemonicMap.find(str);
  if (it != mnemonicMap.end()) {
    return it->second;
  }
  throw std::runtime_error("Unknown mnemonic: " + str);
}

bool CPU::matchAddressingMode(const std::string &operand, AddressingMode mode) {
  switch (mode) {
  case AddressingMode::Implied:
  case AddressingMode::Accumulator:
    return operand.empty() || operand == "A";
  case AddressingMode::Immediate:
    return operand.size() > 1 && operand[0] == '#';
  case AddressingMode::ZeroPage:
    return operand.size() <= 3 && operand[0] != '#' &&
           operand.find(',') == std::string::npos;
  case AddressingMode::ZeroPageX:
    return operand.size() > 2 && operand.back() == 'X' &&
           operand.find(',') != std::string::npos;
  case AddressingMode::ZeroPageY:
    return operand.size() > 2 && operand.back() == 'Y' &&
           operand.find(',') != std::string::npos;
  case AddressingMode::Absolute:
    return operand.size() > 3 && operand[0] != '#' &&
           operand.find(',') == std::string::npos;
  case AddressingMode::AbsoluteX:
    return operand.size() > 4 && operand.back() == 'X' &&
           operand.find(',') != std::string::npos;
  case AddressingMode::AbsoluteY:
    return operand.size() > 4 && operand.back() == 'Y' &&
           operand.find(',') != std::string::npos;
  case AddressingMode::Indirect:
    return operand.size() > 4 && operand.front() == '(' &&
           operand.back() == ')';
  case AddressingMode::IndexedIndirect:
    return operand.size() > 5 && operand.front() == '(' &&
           operand.find(",X)") != std::string::npos;
  case AddressingMode::IndirectIndexed:
    return operand.size() > 5 && operand.front() == '(' &&
           operand.find("),Y") != std::string::npos;
  case AddressingMode::Relative:
    return true; // All operands could potentially be relative (label or offset)
  default:
    return false;
  }
}

uint16_t CPU::parseOperand(const std::string &operand, AddressingMode mode) {
  std::string value = operand;
  if (mode == AddressingMode::Immediate) {
    value = value.substr(1);
  }
  if (value.front() == '$') {
    return std::stoul(value.substr(1), nullptr, 16);
  }
  if (std::isdigit(value.front())) {
    return std::stoul(value);
  }
  throw std::runtime_error("Invalid operand format: " + operand);
}

bool CPU::isLabel(const std::string &operand) {
  return std::all_of(operand.begin(), operand.end(),
                     [](char c) { return std::isalnum(c) || c == '_'; });
}

std::vector<uint8_t> CPU::assemble(const std::vector<std::string> &code) {
  std::vector<uint8_t> binary;
  std::unordered_map<std::string, uint16_t> labels;
  std::vector<std::pair<size_t, std::string>> unresolved_labels;

  // First pass: collect labels and assemble known instructions
  uint16_t address = 0;
  for (const auto &line : code) {
    std::istringstream iss(line);
    std::string token;
    iss >> token;

    // Handle comments
    if (token.empty() || token[0] == ';') {
      continue;
    }

    // Handle labels
    if (token.back() == ':') {
      labels[token.substr(0, token.length() - 1)] = address;
      continue;
    }

    // Handle sections
    if (token == ".section") {
      std::string section_name;
      iss >> section_name;
      binary.push_back(0xFF);
      binary.push_back(0xFF);
      for (char c : section_name) {
        binary.push_back(static_cast<uint8_t>(c));
      }
      binary.push_back(0x00);
      continue;
    }

    // Handle instructions
    Operation op = stringToMnemonic(token);
    auto it = operationTable_.find(op);
    if (it == operationTable_.end()) {
      throw std::runtime_error("Unknown mnemonic: " + token);
    }

    const auto &instructions = it->second;
    std::string operand;
    std::getline(iss, operand);
    operand.erase(0, operand.find_first_not_of(" \t"));
    operand.erase(operand.find_last_not_of(" \t") + 1);

    bool found = false;
    for (const auto &instr : instructions) {
      if (matchAddressingMode(operand, instr.mode)) {
        binary.push_back(instr.opcode);
        address++;
        found = true;

        if (instr.mode != AddressingMode::Implied &&
            instr.mode != AddressingMode::Accumulator) {
          if (isLabel(operand)) {
            unresolved_labels.emplace_back(binary.size(), operand);
            binary.push_back(0);
            binary.push_back(0);
            address += 2;
          } else {
            uint16_t value = parseOperand(operand, instr.mode);
            if (instr.mode == AddressingMode::Immediate ||
                instr.mode == AddressingMode::ZeroPage ||
                instr.mode == AddressingMode::ZeroPageX ||
                instr.mode == AddressingMode::ZeroPageY ||
                instr.mode == AddressingMode::IndexedIndirect ||
                instr.mode == AddressingMode::IndirectIndexed) {
              binary.push_back(value & 0xFF);
              address++;
            } else {
              binary.push_back(value & 0xFF);
              binary.push_back((value >> 8) & 0xFF);
              address += 2;
            }
          }
        }
        break;
      }
    }

    if (!found) {
      throw std::runtime_error("Invalid addressing mode for mnemonic: " +
                               token);
    }
  }

  // Second pass: resolve labels
  for (const auto &[pos, label] : unresolved_labels) {
    if (labels.find(label) == labels.end()) {
      throw std::runtime_error("Undefined label: " + label);
    }
    uint16_t value = labels[label];
    binary[pos] = value & 0xFF;
    binary[pos + 1] = (value >> 8) & 0xFF;
  }

  return binary;
}

// Combined operation functions
// Load/Store Operations
void CPU::LDA(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  registers_.A = value;
  updateZeroAndNegativeFlags(registers_.A);
}

void CPU::LDX(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  registers_.X = value;
  updateZeroAndNegativeFlags(registers_.X);
}

void CPU::LDY(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  registers_.Y = value;
  updateZeroAndNegativeFlags(registers_.Y);
}

void CPU::STA(const Instruction &instruction) {
  Word address = fetchAddress(instruction.mode);
  write(address, registers_.A);
}

void CPU::STX(const Instruction &instruction) {
  Word address = fetchAddress(instruction.mode);
  write(address, registers_.X);
}

void CPU::STY(const Instruction &instruction) {
  Word address = fetchAddress(instruction.mode);
  write(address, registers_.Y);
}

// Transfer Operations
void CPU::TAX(const Instruction &instruction) {
  registers_.X = registers_.A;
  updateZeroAndNegativeFlags(registers_.X);
}

void CPU::TAY(const Instruction &instruction) {
  registers_.Y = registers_.A;
  updateZeroAndNegativeFlags(registers_.Y);
}

void CPU::TXA(const Instruction &instruction) {
  registers_.A = registers_.X;
  updateZeroAndNegativeFlags(registers_.A);
}

void CPU::TYA(const Instruction &instruction) {
  registers_.A = registers_.Y;
  updateZeroAndNegativeFlags(registers_.A);
}

// Stack Operations
void CPU::TSX(const Instruction &instruction) {
  registers_.X = registers_.SP;
  updateZeroAndNegativeFlags(registers_.X);
}

void CPU::TXS(const Instruction &instruction) { registers_.SP = registers_.X; }

void CPU::PHA(const Instruction &instruction) {
  write(0x100 + registers_.SP, registers_.A);
  registers_.SP--;
}

void CPU::PHP(const Instruction &instruction) {
  Byte status = (registers_.P.N << 7) | (registers_.P.V << 6) |
                (registers_.P.U << 5) | (registers_.P.B << 4) |
                (registers_.P.D << 3) | (registers_.P.I << 2) |
                (registers_.P.Z << 1) | registers_.P.C;
  write(0x100 + registers_.SP, status);
  registers_.SP--;
}

void CPU::PLA(const Instruction &instruction) {
  registers_.SP++;
  registers_.A = read(0x100 + registers_.SP);
  updateZeroAndNegativeFlags(registers_.A);
}

void CPU::PLP(const Instruction &instruction) {
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
void CPU::AND(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  registers_.A &= value;
  updateZeroAndNegativeFlags(registers_.A);
}

void CPU::EOR(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  registers_.A ^= value;
  updateZeroAndNegativeFlags(registers_.A);
}

void CPU::ORA(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  registers_.A |= value;
  updateZeroAndNegativeFlags(registers_.A);
}

void CPU::BIT(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  registers_.P.Z = (registers_.A & value) == 0;
  registers_.P.N = (value & 0x80) != 0;
  registers_.P.V = (value & 0x40) != 0;
}

// Arithmetic Operations
void CPU::ADC(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  Word result = registers_.A + value + (registers_.P.C ? 1 : 0);

  registers_.P.C = result > 0xFF;
  registers_.P.V = ((registers_.A ^ result) & (value ^ result) & 0x80) != 0;
  registers_.A = result & 0xFF;
  updateZeroAndNegativeFlags(registers_.A);
}

void CPU::SBC(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  Word result = registers_.A - value - (registers_.P.C ? 0 : 1);

  registers_.P.C = result < 0x100;
  registers_.P.V =
      ((registers_.A ^ result) & ((0xFF - value) ^ result) & 0x80) != 0;
  registers_.A = result & 0xFF;
  updateZeroAndNegativeFlags(registers_.A);
}

void CPU::CMP(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  Word result = registers_.A - value;

  registers_.P.C = registers_.A >= value;
  updateZeroAndNegativeFlags(result & 0xFF);
}

void CPU::CPX(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  Word result = registers_.X - value;

  registers_.P.C = registers_.X >= value;
  updateZeroAndNegativeFlags(result & 0xFF);
}

void CPU::CPY(const Instruction &instruction) {
  Byte value = fetchOperand(instruction.mode);
  Word result = registers_.Y - value;

  registers_.P.C = registers_.Y >= value;
  updateZeroAndNegativeFlags(result & 0xFF);
}

// Increments/Decrements
void CPU::INC(const Instruction &instruction) {
  Word address = fetchAddress(instruction.mode);
  Byte value = read(address);
  value++;
  write(address, value);
  updateZeroAndNegativeFlags(value);
}

void CPU::INX(const Instruction &instruction) {
  registers_.X++;
  updateZeroAndNegativeFlags(registers_.X);
}

void CPU::INY(const Instruction &instruction) {
  registers_.Y++;
  updateZeroAndNegativeFlags(registers_.Y);
}

void CPU::DEC(const Instruction &instruction) {
  Word address = fetchAddress(instruction.mode);
  Byte value = read(address);
  value--;
  write(address, value);
  updateZeroAndNegativeFlags(value);
}

void CPU::DEX(const Instruction &instruction) {
  registers_.X--;
  updateZeroAndNegativeFlags(registers_.X);
}

void CPU::DEY(const Instruction &instruction) {
  registers_.Y--;
  updateZeroAndNegativeFlags(registers_.Y);
}

// Shifts
void CPU::ASL(const Instruction &instruction) {
  if (instruction.mode == AddressingMode::Accumulator) {
    registers_.P.C = (registers_.A & 0x80) != 0;
    registers_.A <<= 1;
    updateZeroAndNegativeFlags(registers_.A);
  } else {
    Word address = fetchAddress(instruction.mode);
    Byte value = read(address);
    registers_.P.C = (value & 0x80) != 0;
    value <<= 1;
    write(address, value);
    updateZeroAndNegativeFlags(value);
  }
}

void CPU::LSR(const Instruction &instruction) {
  if (instruction.mode == AddressingMode::Accumulator) {
    registers_.P.C = (registers_.A & 0x01) != 0;
    registers_.A >>= 1;
    updateZeroAndNegativeFlags(registers_.A);
  } else {
    Word address = fetchAddress(instruction.mode);
    Byte value = read(address);
    registers_.P.C = (value & 0x01) != 0;
    value >>= 1;
    write(address, value);
    updateZeroAndNegativeFlags(value);
  }
}

void CPU::ROL(const Instruction &instruction) {
  if (instruction.mode == AddressingMode::Accumulator) {
    Byte oldCarry = registers_.P.C ? 1 : 0;
    registers_.P.C = (registers_.A & 0x80) != 0;
    registers_.A = (registers_.A << 1) | oldCarry;
    updateZeroAndNegativeFlags(registers_.A);
  } else {
    Word address = fetchAddress(instruction.mode);
    Byte value = read(address);
    Byte oldCarry = registers_.P.C ? 1 : 0;
    registers_.P.C = (value & 0x80) != 0;
    value = (value << 1) | oldCarry;
    write(address, value);
    updateZeroAndNegativeFlags(value);
  }
}

void CPU::ROR(const Instruction &instruction) {
  if (instruction.mode == AddressingMode::Accumulator) {
    Byte oldCarry = registers_.P.C ? 0x80 : 0;
    registers_.P.C = (registers_.A & 0x01) != 0;
    registers_.A = (registers_.A >> 1) | oldCarry;
    updateZeroAndNegativeFlags(registers_.A);
  } else {
    Word address = fetchAddress(instruction.mode);
    Byte value = read(address);
    Byte oldCarry = registers_.P.C ? 0x80 : 0;
    registers_.P.C = (value & 0x01) != 0;
    value = (value >> 1) | oldCarry;
    write(address, value);
    updateZeroAndNegativeFlags(value);
  }
}

// Jumps and Calls
void CPU::JMP(const Instruction &instruction) {
  Word address = fetchAddress(instruction.mode);
  registers_.PC = address;
}

void CPU::JSR(const Instruction &instruction) {
  registers_.PC--;
  write(0x100 + registers_.SP, (registers_.PC >> 8) & 0xFF);
  registers_.SP--;
  write(0x100 + registers_.SP, registers_.PC & 0xFF);
  registers_.SP--;
  registers_.PC = fetchAddress(AddressingMode::Absolute);
}

void CPU::RTS(const Instruction &instruction) {
  registers_.SP++;
  Word low = read(0x100 + registers_.SP);
  registers_.SP++;
  Word high = read(0x100 + registers_.SP);
  registers_.PC = (high << 8) | low;
  registers_.PC++;
}

// Branches
void CPU::BCC(const Instruction &instruction) {
  if (!registers_.P.C) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers_.PC += static_cast<int8_t>(offset);
  }
}

void CPU::BCS(const Instruction &instruction) {
  if (registers_.P.C) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers_.PC += static_cast<int8_t>(offset);
  }
}

void CPU::BEQ(const Instruction &instruction) {
  if (registers_.P.Z) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers_.PC += static_cast<int8_t>(offset);
  }
}

void CPU::BMI(const Instruction &instruction) {
  if (registers_.P.N) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers_.PC += static_cast<int8_t>(offset);
  }
}

void CPU::BNE(const Instruction &instruction) {
  if (!registers_.P.Z) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers_.PC += static_cast<int8_t>(offset);
  }
}

void CPU::BPL(const Instruction &instruction) {
  if (!registers_.P.N) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers_.PC += static_cast<int8_t>(offset);
  }
}

void CPU::BVC(const Instruction &instruction) {
  if (!registers_.P.V) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers_.PC += static_cast<int8_t>(offset);
  }
}

void CPU::BVS(const Instruction &instruction) {
  if (registers_.P.V) {
    Byte offset = fetchOperand(AddressingMode::Relative);
    registers_.PC += static_cast<int8_t>(offset);
  }
}

// Status Flag Changes
void CPU::CLC(const Instruction &instruction) { registers_.P.C = false; }

void CPU::CLD(const Instruction &instruction) { registers_.P.D = false; }

void CPU::CLI(const Instruction &instruction) { registers_.P.I = false; }

void CPU::CLV(const Instruction &instruction) { registers_.P.V = false; }

void CPU::SEC(const Instruction &instruction) { registers_.P.C = true; }

void CPU::SED(const Instruction &instruction) { registers_.P.D = true; }

void CPU::SEI(const Instruction &instruction) { registers_.P.I = true; }

// System Functions
void CPU::BRK(const Instruction &instruction) {
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

void CPU::NOP(const Instruction &instruction) {
  // No operation
}

void CPU::RTI(const Instruction &instruction) {
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

void CPU::initializeTables() {
  auto addInstruction = [this](Byte opcode, const Operation &mnemonic,
                               std::uint32_t cycles, AddressingMode mode) {
    Instruction inst{opcode, mnemonic, cycles, mode};
    opcodeTable_[opcode] = inst;
    translationTable_[InstructionKey{mnemonic, mode}] = inst;
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
  addInstruction(0x7D, op, 4, AddressingMode::AbsoluteX);
  addInstruction(0x79, op, 4, AddressingMode::AbsoluteY);
  addInstruction(0x61, op, 6, AddressingMode::IndexedIndirect);
  addInstruction(0x71, op, 5, AddressingMode::IndirectIndexed);

  // AND (Logical AND)
  op = Operation::AND;
  addInstruction(0x29, op, 2, AddressingMode::Immediate);
  addInstruction(0x25, op, 3, AddressingMode::ZeroPage);
  addInstruction(0x35, op, 4, AddressingMode::ZeroPageX);
  addInstruction(0x2D, op, 4, AddressingMode::Absolute);
  addInstruction(0x3D, op, 4, AddressingMode::AbsoluteX);
  addInstruction(0x39, op, 4, AddressingMode::AbsoluteY);
  addInstruction(0x21, op, 6, AddressingMode::IndexedIndirect);
  addInstruction(0x31, op, 5, AddressingMode::IndirectIndexed);

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
  addInstruction(0xDD, op, 4, AddressingMode::AbsoluteX);
  addInstruction(0xD9, op, 4, AddressingMode::AbsoluteY);
  addInstruction(0xC1, op, 6, AddressingMode::IndexedIndirect);
  addInstruction(0xD1, op, 5, AddressingMode::IndirectIndexed);

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
  addInstruction(0x5D, op, 4, AddressingMode::AbsoluteX);
  addInstruction(0x59, op, 4, AddressingMode::AbsoluteY);
  addInstruction(0x41, op, 6, AddressingMode::IndexedIndirect);
  addInstruction(0x51, op, 5, AddressingMode::IndirectIndexed);

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
  addInstruction(0xBD, op, 4, AddressingMode::AbsoluteX);
  addInstruction(0xB9, op, 4, AddressingMode::AbsoluteY);
  addInstruction(0xA1, op, 6, AddressingMode::IndexedIndirect);
  addInstruction(0xB1, op, 5, AddressingMode::IndirectIndexed);

  // LDX (Load X Register)
  op = Operation::LDX;
  addInstruction(0xA2, op, 2, AddressingMode::Immediate);
  addInstruction(0xA6, op, 3, AddressingMode::ZeroPage);
  addInstruction(0xB6, op, 4, AddressingMode::ZeroPageY);
  addInstruction(0xAE, op, 4, AddressingMode::Absolute);
  addInstruction(0xBE, op, 4, AddressingMode::AbsoluteY);

  // LDY (Load Y Register)
  op = Operation::LDY;
  addInstruction(0xA0, op, 2, AddressingMode::Immediate);
  addInstruction(0xA4, op, 3, AddressingMode::ZeroPage);
  addInstruction(0xB4, op, 4, AddressingMode::ZeroPageX);
  addInstruction(0xAC, op, 4, AddressingMode::Absolute);
  addInstruction(0xBC, op, 4, AddressingMode::AbsoluteX);

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

  // ORA (Logical Inclusive OR)
  op = Operation::ORA;
  addInstruction(0x09, op, 2, AddressingMode::Immediate);
  addInstruction(0x05, op, 3, AddressingMode::ZeroPage);
  addInstruction(0x15, op, 4, AddressingMode::ZeroPageX);
  addInstruction(0x0D, op, 4, AddressingMode::Absolute);
  addInstruction(0x1D, op, 4, AddressingMode::AbsoluteX);
  addInstruction(0x19, op, 4, AddressingMode::AbsoluteY);
  addInstruction(0x01, op, 6, AddressingMode::IndexedIndirect);
  addInstruction(0x11, op, 5, AddressingMode::IndirectIndexed);

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
  addInstruction(0xFD, op, 4, AddressingMode::AbsoluteX);
  addInstruction(0xF9, op, 4, AddressingMode::AbsoluteY);
  addInstruction(0xE1, op, 6, AddressingMode::IndexedIndirect);
  addInstruction(0xF1, op, 5, AddressingMode::IndirectIndexed);

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
}

// Helper functions
Byte CPU::fetchOperand(AddressingMode mode) {
  switch (mode) {
  case AddressingMode::Implied:
  case AddressingMode::Accumulator:
    // These modes don't fetch an operand
    return 0;

  case AddressingMode::Immediate:
    return read(registers_.PC++);

  case AddressingMode::ZeroPage:
    return read(read(registers_.PC++));

  case AddressingMode::ZeroPageX:
    return read((read(registers_.PC++) + registers_.X) & 0xFF);

  case AddressingMode::ZeroPageY:
    return read((read(registers_.PC++) + registers_.Y) & 0xFF);

  case AddressingMode::Relative:
    return read(registers_.PC++);

  case AddressingMode::Absolute: {
    Word address = read(registers_.PC++) | (read(registers_.PC++) << 8);
    return read(address);
  }

  case AddressingMode::AbsoluteX: {
    Word address = read(registers_.PC++) | (read(registers_.PC++) << 8);
    return read(address + registers_.X);
  }

  case AddressingMode::AbsoluteY: {
    Word address = read(registers_.PC++) | (read(registers_.PC++) << 8);
    return read(address + registers_.Y);
  }

  case AddressingMode::Indirect: {
    Word pointer = read(registers_.PC++) | (read(registers_.PC++) << 8);
    Word address = read(pointer) |
                   (read((pointer & 0xFF00) | ((pointer + 1) & 0x00FF)) << 8);
    return read(address);
  }

  case AddressingMode::IndexedIndirect: {
    Byte zeroPageAddress = (read(registers_.PC++) + registers_.X) & 0xFF;
    Word address =
        read(zeroPageAddress) | (read((zeroPageAddress + 1) & 0xFF) << 8);
    return read(address);
  }

  case AddressingMode::IndirectIndexed: {
    Byte zeroPageAddress = read(registers_.PC++);
    Word address =
        read(zeroPageAddress) | (read((zeroPageAddress + 1) & 0xFF) << 8);
    return read(address + registers_.Y);
  }

  default:
    throw std::runtime_error("Unimplemented addressing mode");
  }
}

Word CPU::fetchAddress(AddressingMode mode) {
  switch (mode) {
  case AddressingMode::Implied:
  case AddressingMode::Accumulator:
  case AddressingMode::Immediate:
    // These modes don't use an address
    return 0;

  case AddressingMode::ZeroPage:
    return read(registers_.PC++);

  case AddressingMode::ZeroPageX:
    return (read(registers_.PC++) + registers_.X) & 0xFF;

  case AddressingMode::ZeroPageY:
    return (read(registers_.PC++) + registers_.Y) & 0xFF;

  case AddressingMode::Relative: {
    Byte offset = read(registers_.PC++);
    // Convert to signed offset
    return registers_.PC + static_cast<int8_t>(offset);
  }

  case AddressingMode::Absolute:
    return read(registers_.PC++) | (read(registers_.PC++) << 8);

  case AddressingMode::AbsoluteX: {
    Word base = read(registers_.PC++) | (read(registers_.PC++) << 8);
    return base + registers_.X;
  }

  case AddressingMode::AbsoluteY: {
    Word base = read(registers_.PC++) | (read(registers_.PC++) << 8);
    return base + registers_.Y;
  }

  case AddressingMode::Indirect: {
    Word pointer = read(registers_.PC++) | (read(registers_.PC++) << 8);
    // Simulate the 6502 bug with indirect jump
    if ((pointer & 0xFF) == 0xFF) {
      return read(pointer) | (read(pointer & 0xFF00) << 8);
    } else {
      return read(pointer) | (read(pointer + 1) << 8);
    }
  }

  case AddressingMode::IndexedIndirect: {
    Byte zeroPageAddress = (read(registers_.PC++) + registers_.X) & 0xFF;
    return read(zeroPageAddress) | (read((zeroPageAddress + 1) & 0xFF) << 8);
  }

  case AddressingMode::IndirectIndexed: {
    Byte zeroPageAddress = read(registers_.PC++);
    Word base =
        read(zeroPageAddress) | (read((zeroPageAddress + 1) & 0xFF) << 8);
    return base + registers_.Y;
  }

  default:
    throw std::runtime_error("Unimplemented addressing mode");
  }
}
void CPU::updateZeroAndNegativeFlags(Byte value) {
  registers_.P.Z = (value == 0);
  registers_.P.N = (value & 0x80) != 0;
}

std::string CPU::mnemonicToString(Operation mnemonic) {
  switch (mnemonic) {
  case Operation::ADC:
    return "ADC";
  case Operation::AND:
    return "AND";
  case Operation::ASL:
    return "ASL";
  case Operation::BCC:
    return "BCC";
  case Operation::BCS:
    return "BCS";
  case Operation::BEQ:
    return "BEQ";
  case Operation::BIT:
    return "BIT";
  case Operation::BMI:
    return "BMI";
  case Operation::BNE:
    return "BNE";
  case Operation::BPL:
    return "BPL";
  case Operation::BRK:
    return "BRK";
  case Operation::BVC:
    return "BVC";
  case Operation::BVS:
    return "BVS";
  case Operation::CLC:
    return "CLC";
  case Operation::CLD:
    return "CLD";
  case Operation::CLI:
    return "CLI";
  case Operation::CLV:
    return "CLV";
  case Operation::CMP:
    return "CMP";
  case Operation::CPX:
    return "CPX";
  case Operation::CPY:
    return "CPY";
  case Operation::DEC:
    return "DEC";
  case Operation::DEX:
    return "DEX";
  case Operation::DEY:
    return "DEY";
  case Operation::EOR:
    return "EOR";
  case Operation::INC:
    return "INC";
  case Operation::INX:
    return "INX";
  case Operation::INY:
    return "INY";
  case Operation::JMP:
    return "JMP";
  case Operation::JSR:
    return "JSR";
  case Operation::LDA:
    return "LDA";
  case Operation::LDX:
    return "LDX";
  case Operation::LDY:
    return "LDY";
  case Operation::LSR:
    return "LSR";
  case Operation::NOP:
    return "NOP";
  case Operation::ORA:
    return "ORA";
  case Operation::PHA:
    return "PHA";
  case Operation::PHP:
    return "PHP";
  case Operation::PLA:
    return "PLA";
  case Operation::PLP:
    return "PLP";
  case Operation::ROL:
    return "ROL";
  case Operation::ROR:
    return "ROR";
  case Operation::RTI:
    return "RTI";
  case Operation::RTS:
    return "RTS";
  case Operation::SBC:
    return "SBC";
  case Operation::SEC:
    return "SEC";
  case Operation::SED:
    return "SED";
  case Operation::SEI:
    return "SEI";
  case Operation::STA:
    return "STA";
  case Operation::STX:
    return "STX";
  case Operation::STY:
    return "STY";
  case Operation::TAX:
    return "TAX";
  case Operation::TAY:
    return "TAY";
  case Operation::TSX:
    return "TSX";
  case Operation::TXA:
    return "TXA";
  case Operation::TXS:
    return "TXS";
  case Operation::TYA:
    return "TYA";
  default:
    throw std::runtime_error("Unknown operation");
  }
}

} // namespace m6502