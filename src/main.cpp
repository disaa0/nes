#include "emulator.h"

int main() {
  const uint64_t CYCLES_PER_FRAME =
      17030; // For NTSC NES (57 cycles * 341 scanlines)
      
  // m6502::CPU cpu(1789773);
  nes::Emulator emulator;

  // std::cout << cpu.use_count();

  emulator.run(CYCLES_PER_FRAME);
}
