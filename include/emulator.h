#pragma once
#include "cartrige.h"
#include <debugger.h>
#include <bus.h>
#include <m6502.h>
#include <memory>

class Emulator {
public:
  Emulator(std::shared_ptr<nes::CPU> cpu)
      : cpu_(cpu), debugger_(std::make_shared<nes::Debugger>(cpu)) {}
  ~Emulator() {}

  void run(const std::vector<std::string> program, const uint64_t targetCycles,
           const char mode);

private:
  std::shared_ptr<nes::CPU> cpu_;           // Shared pointer to the CPU
  std::shared_ptr<nes::Debugger> debugger_; // Shared pointer to the debugger
  std::shared_ptr<nes::Bus> bus_;           // Shared pointer to the Bus
  nes::Cartridge cartridge_;

  static constexpr uint64_t CYCLES_PER_FRAME = 29780;
  void synchronizeWithRealTime();
};