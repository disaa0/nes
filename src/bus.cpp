#include <bus.h>
#include <array>
#include <cstdint>

using namespace nes;

void Bus::cpu_map_device(BusDevice *device, uint16_t startAddress,
                    uint16_t endAddress) {
  for (uint64_t addr = startAddress; addr <= endAddress; addr++) {
    cpu_memory_map[addr] = device;
  }
}

uint8_t Bus::cpu_read(uint16_t address) {
  if (cpu_memory_map[address]) {
    return cpu_memory_map[address]->read(address);
  }
  return 0;
}

void Bus::cpu_write(uint16_t address, uint8_t data) {
  if (cpu_memory_map[address]) {
    cpu_memory_map[address]->write(address, data);
  }
}


void Bus::ppu_map_device(BusDevice *device, uint16_t startAddress,
                    uint16_t endAddress) {
  for (uint64_t addr = startAddress; addr <= endAddress; addr++) {
    ppu_memory_map[addr] = device;
  }
}

uint8_t Bus::ppu_read(uint16_t address) {
  if (ppu_memory_map[address]) {
    return ppu_memory_map[address]->read(address);
  }
  return 0;
}

void Bus::ppu_write(uint16_t address, uint8_t data) {
  if (ppu_memory_map[address]) {
    ppu_memory_map[address]->write(address, data);
  }
}
