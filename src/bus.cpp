#include <array>
#include <bus.h>
#include <cstdint>

using namespace nes;

void Bus::cpu_map_device(IBusDevice *device, uint16_t startAddress,
                    uint16_t endAddress) {
  for (uint16_t addr = startAddress; addr <= endAddress; addr++) {
    memoryMap[addr] = device;
  }
}

uint8_t Bus::cpu_read(uint16_t address) {
  if (memoryMap[address]) {
    return memoryMap[address]->read(address);
  }
  return 0;
}

void Bus::cpu_write(uint16_t address, uint8_t data) {
  if (memoryMap[address]) {
    memoryMap[address]->write(address, data);
  }
}
