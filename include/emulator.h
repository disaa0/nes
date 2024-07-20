#pragma once
#include "cartrige.h"
#include <bus.h>
#include <debugger.h>
#include <m6502.h>
#include <memory>

namespace nes {

class Emulator {
public:
  Emulator() {}
  ~Emulator() {}

  void run(const uint64_t targetCycles);

private:
  std::shared_ptr<nes::Bus> bus_ =
      std::make_shared<nes::Bus>(); // Shared pointer to the Bus

  std::shared_ptr<nes::CPU> cpu_ =
      std::make_shared<nes::CPU>(bus_); // Shared pointer to the Bus

  std::shared_ptr<nes::Cartridge> cartridge_ =
      std::make_shared<nes::Cartridge>(); // Shared pointer to the Cartridge

  nes::Debugger debugger_ = nes::Debugger(cpu_);

  static constexpr uint64_t CYCLES_PER_FRAME = 29780;
  void synchronizeWithRealTime();
  void mapDevices();
};

} // namespace nes