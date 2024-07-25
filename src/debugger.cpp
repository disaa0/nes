#include <debugger.h>

using namespace nes;

std::string Debugger::mnemonicToString(Operation mnemonic) {
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
    throw std::runtime_error("Unknown Operation");
  }
}
std::string Debugger::byteToHex(uint8_t byte) {
  std::stringstream ss;
  ss << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(byte);
  return ss.str();
}

std::string Debugger::wordToHex(uint16_t word) {
  std::stringstream ss;
  ss << std::setw(4) << std::setfill('0') << std::hex << word;
  return ss.str();
}

std::vector<std::string> Debugger::disassemble(const std::vector<uint8_t> &code, CPU *cpu) {
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
    auto it = cpu->opcodeTable_.find(opcode);
    if (it == cpu->opcodeTable_.end()) {
      throw std::runtime_error("Unknown opcode: " + std::to_string(opcode));
    }

    const CPU::Instruction &instruction = it->second;
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

Operation Debugger::stringToMnemonic(const std::string &str) {
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

bool Debugger::matchAddressingMode(const std::string &operand, AddressingMode mode) {
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

uint16_t Debugger::parseOperand(const std::string &operand, AddressingMode mode) {
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

bool Debugger::isLabel(const std::string &operand) {
  return std::all_of(operand.begin(), operand.end(),
                     [](char c) { return std::isalnum(c) || c == '_'; });
}

std::vector<uint8_t> Debugger::assemble(const std::vector<std::string> &code, CPU *cpu) {
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
    auto it = cpu->operationTable_.find(op);
    if (it == cpu->operationTable_.end()) {
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
