#pragma once
#include <cstdint>
#include <m6502.h>
#include <memory>
#include <string>
#include <vector>

namespace nes {

class Debugger {
public:
  Debugger(std::shared_ptr<CPU> cpu) : cpu_(cpu) {}
  ~Debugger() {}

  std::vector<std::string> disassemble(const std::vector<uint8_t> &code);
  std::vector<uint8_t> assemble(const std::vector<std::string> &code);

  std::string mnemonicToString(Operation mnemonic);
  std::string byteToHex(uint8_t byte);
  std::string wordToHex(uint16_t word);
  Operation stringToMnemonic(const std::string &str);
  bool matchAddressingMode(const std::string &operand,
                           AddressingMode mode);
  uint16_t parseOperand(const std::string &operand, AddressingMode mode);
  bool isLabel(const std::string &operand);

private:
  std::shared_ptr<CPU> cpu_; // Shared pointer to the CPU
};

}