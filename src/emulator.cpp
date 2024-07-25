#include <bus.h>
#include <cartridge.h>
#include <emulator.h>
#include <iostream>
#include <m6502.h>
#include <thread>

using namespace nes;

void Emulator::run(const uint64_t targetCycles) {
  // map Devices
  RAM ram = RAM(0x0800);
  PPU ppu = PPU(0x0008);
  pgrROM pgr = pgrROM(&cartridge);
  chrROM chr = chrROM(&cartridge);
  
  bus.cpu_map_device(&ram, 0x0000, 0x07FF);
  bus.cpu_map_device(&ppu, 0x2000, 0x3FFF);
  bus.cpu_map_device(&pgr, 0x8000, 0xFFFF);

  // load ROM
  cpu.reset();

  try {
    cartridge.loadROM("nestest.nes");
  } catch (const std::exception &e) {
    std::cerr << "Error loading ROM: " << e.what() << std::endl;
  }

  // start execution
  while (true) {
    cpu.run(targetCycles);
    synchronizeWithRealTime();
    // std::cout << cpu_->cycles_ << std::endl;
  }
}

void Emulator::synchronizeWithRealTime() {
  static auto lastFrameTime = std::chrono::high_resolution_clock::now();
  auto targetFrameTime = std::chrono::duration<double>(1.0 / 60.0); // 60 Hz

  auto now = std::chrono::high_resolution_clock::now();
  auto frameTime = now - lastFrameTime;

  if (frameTime < targetFrameTime) {
    std::this_thread::sleep_for(targetFrameTime - frameTime);
  }

  lastFrameTime = std::chrono::high_resolution_clock::now();
}
