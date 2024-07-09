#include <m6502.h>
#include <string>
int main() {
  m6502::CPU cpu;
  //std::string instruction { "LDA #$0" };
  //m6502::AddressingMode mode = cpu.detectAddressingMode(instruction);
  // std::vector<uint8_t> program = {
  //     0xA9, 0x05, // LDA #$05    ; Load 5 into accumulator
  //     0x85, 0x10, // STA $10     ; Store accumulator to zero page address $10
  //     0xA2, 0x0A, // LDX #$0A    ; Load 10 into X register
  //     0xA0, 0x15, // LDY #$15    ; Load 21 into Y register
  //     0x18,       // CLC         ; Clear carry flag
  // };
  std::vector<std::string> program = {
      "LDA #$05",    //; Load 5 into accumulator
      "STA $10",     //; Store accumulator to zero page address $10
      "LDX #$0A",    //; Load 10 into X register
      "LDY #$15",    //; Load 21 into Y register
      "CLC"             //; Clear carry flag
  };
  cpu.run(program,'a');
}
