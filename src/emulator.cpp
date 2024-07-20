#include <bus.h>
#include <cartrige.h>
#include <emulator.h>
#include <iostream>
#include <m6502.h>
#include <thread>

using namespace nes;

void Emulator::run(const uint64_t targetCycles) {
  mapDevices();
  cpu_->reset();
  
  try {
    cartridge_->loadROM("nestest.nes");
  } catch (const std::exception &e) {
    std::cerr << "Error loading ROM: " << e.what() << std::endl;
  }

  while (true) {
    cpu_->run(targetCycles);
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

void Emulator::mapDevices() {
  RAM ram = RAM(0x0800);
  PPU ppu = PPU(0x0008);
  chrROM chr = chrROM(cartridge_, cartridge_->chrROM);
  chrROM prg = chrROM(cartridge_, cartridge_->prgROM);
  bus_->mapDevice(&ram, 0x0000, 0x07FF);
  bus_->mapDevice(&ppu, 0x2000, 0x3FFF);
  bus_->mapDevice(&chr, 0x, 0x);
}