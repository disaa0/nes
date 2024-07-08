#include <m6502.h>
#include <string>
int main() {
  m6502::CPU cpu;
  //std::string instruction { "LDA #$0" };
  //m6502::AddressingMode mode = cpu.detectAddressingMode(instruction);
  cpu.run();
}
