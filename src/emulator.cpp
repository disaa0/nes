#include <bus.h>
#include <cartridge.h>
#include <cstdint>
#include <emulator.h>
#include <iostream>
#include <m6502.h>
#include <sys/types.h>
#include <thread>
#include <vector>

using namespace nes;

void Emulator::run(const uint64_t targetCycles) {
  // map Devices
  // CPU
  RAM ram = RAM(0x0800);
  PPU ppu = PPU(&bus); // Used to acceess ppu's registers.
  pgrROM pgr = pgrROM(&cartridge);

  // PPU
  RAM vram = RAM(0x0800);
  chrROM chr = chrROM(&cartridge);
  RAM pram = RAM(0x0020);

  // CPU mapping
  bus.cpu_map_device(&ram, 0x0000, 0x07FF);
  bus.cpu_map_device(&ppu, 0x2000, 0x3FFF);
  bus.cpu_map_device(&pgr, 0x8000, 0xFFFF);

  // PPU mapping
  bus.ppu_map_device(&chr, 0x0000,
                     0x1FFF); // Pattern tables mapped to chrROM/chrRAM
  bus.ppu_map_device(&vram, 0x2000, 0x3EFF); // Nametables mapped to vRAM
  bus.ppu_map_device(&pram, 0x3F00, 0x3FFF); // Palette RAM

  // load ROM
  try {
    cartridge.loadROM("nestest.nes");
  } catch (const std::exception &e) {
    std::cerr << "Error loading ROM: " << e.what() << std::endl;
  }

  cpu.reset();
  ppu.reset();
  {}
  // start execution
  uint64_t total_cycles = 0;
  uint64_t frame_count = 0;
  while (true) {
    std::vector<u_int8_t> operation = cpu.get_next_operation();
    std::vector<std::string> instruction =
        debugger.disassemble(operation, &cpu);

    std::cout << frame_count << "/";
    std::cout << total_cycles << ":\t";
    for (uint16_t i{0}; i < instruction.size(); i++) {
      std::cout << instruction.at(i);
    }
    std::cout << "\n";

    uint64_t cpu_cycles = cpu.step();
    total_cycles += cpu_cycles;

    if (frame_count == 291 && total_cycles == 99) {
      std::cout << "hell yeah";
    }

    // Run PPU for 3 cycles per CPU cycle
    for (uint64_t i = 0; i < cpu_cycles * 3; ++i) {
      // ppu.step();
    }

    if (total_cycles >= targetCycles) {
      // ppu.render();
      synchronizeWithRealTime();
      total_cycles = 0;
      frame_count++;
    }

    // uint64_t endCycles = cycles + targetCycles;
  }
  synchronizeWithRealTime();
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
