#include "emulator.h"
#include <iostream>
#include <m6502.h>
#include <string>

#include <fstream>

int main() {
  const uint64_t CYCLES_PER_FRAME =
      17030; // For NTSC NES (57 cycles * 341 scanlines)
  auto cpu = std::make_shared<nes::CPU>(
      1789773); // Create a CPU with clock speed 1.79 MHz
  // m6502::CPU cpu(1789773);
  Emulator emulator(cpu);

  // std::cout << cpu.use_count();

  // std::string instruction { "LDA #$0" };
  // m6502::AddressingMode mode = cpu.detectAddressingMode(instruction);
  //  std::vector<uint8_t> program = {
  //      0xA9, 0x05, // LDA #$05    ; Load 5 into accumulator
  //      0x85, 0x10, // STA $10     ; Store accumulator to zero page address
  //      $10 0xA2, 0x0A, // LDX #$0A    ; Load 10 into X register 0xA0, 0x15,
  //      // LDY #$15    ; Load 21 into Y register 0x18,       // CLC         ;
  //      Clear carry flag
  //  };

  std::vector<std::string> program = {
      "LDA #$05", //; Load 5 into accumulator
      "STA $10",  //; Store accumulator to zero page address $10
      "LDX #$0A", //; Load 10 into X register
      "LDY #$15", //; Load 21 into Y register
      "CLC"       //; Clear carry flag
  };  

  emulator.run(program, CYCLES_PER_FRAME, 'a');
}
