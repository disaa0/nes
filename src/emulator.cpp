#include <cartrige.h>
#include <emulator.h>
#include <iostream>
#include <m6502.h>
#include <thread>

#include <fstream>

using namespace nes;

void Emulator::run(const std::vector<std::string> program,
                   const uint64_t targetCycles, const char mode) {
  cpu_->reset();

  // if (mode == 'a') {
  //   std::vector<uint8_t> code = debugger_->assemble(program);

  //   std::ofstream file("test.nes", std::ios::binary);

  //   if (!file.is_open()) {
  //     std::cerr << "Error opening file for writing." << std::endl;
  //     return;
  //   }

  //   // Write the data to the file
  //   file.write(reinterpret_cast<char *>(code.data()),
  //              code.size() * sizeof(int));

  //   // Close the file
  //   file.close();

  //   std::cout << "Data written" << std::endl;

  try {
    cartridge_.loadROM("nestest.nes");
  } catch (const std::exception &e) {
    std::cerr << "Error loading ROM: " << e.what() << std::endl;
  }
  // // Print each string in the vector
  // for (Byte i = 0; i < program.size(); i++) {
  //   std::cout << program.at(i) << "\n";
  //   cpu_->write(i + 0x0200, code.at(i));
  // }
  // for (Byte i = 0; i < code.size(); i++) {
  //   std::cout << debugger_->byteToHex(code.at(i)) << " ";
  //   cpu_->write(i + 0x0200, code.at(i));
  // }
  // std::cout << std::endl;
  // }
  // else {
  //   for (Byte i = 0; i < sizeof(program); ++i) {
  //     write(i + 0x0200, program[i]);
  //   }
  // }

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