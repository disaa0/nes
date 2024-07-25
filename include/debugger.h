#pragma once
#include <cstdint>
#include <m6502.h>
#include <string>
#include <vector>

namespace nes {

class Debugger {
public:
  Debugger() {}
  ~Debugger() {}

  std::vector<std::string> disassemble(const std::vector<uint8_t> &code, CPU *cpu);
  std::vector<uint8_t> assemble(const std::vector<std::string> &code, CPU *cpu);

  //Helper functions
  std::string mnemonicToString(Operation mnemonic);
  std::string byteToHex(uint8_t byte);
  std::string wordToHex(uint16_t word);
  Operation stringToMnemonic(const std::string &str);
  bool matchAddressingMode(const std::string &operand,
                           AddressingMode mode);
  uint16_t parseOperand(const std::string &operand, AddressingMode mode);
  bool isLabel(const std::string &operand);
};

}
