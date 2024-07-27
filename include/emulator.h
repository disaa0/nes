#pragma once
#include <bus.h>
#include <cartridge.h>
#include <debugger.h>
#include <m6502.h>
#include <ppu.h>

namespace nes {

class Emulator {
public:
  Emulator() : cpu(CPU(&bus)) {}
  ~Emulator() {}

  void run(const uint64_t targetCycles);

private:
  Bus bus;
  CPU cpu;
  Cartridge cartridge;
  Debugger debugger;

  static constexpr uint64_t CYCLES_PER_FRAME = 29780;
  void synchronizeWithRealTime();
};

} // namespace nes
