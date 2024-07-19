#pragma once
#include <array>
#include <cstdint>

namespace nes {

class IBusDevice {
public:
  virtual ~IBusDevice() = default;
  virtual uint8_t read(uint16_t address) = 0;
  virtual void write(uint16_t address, uint8_t data) = 0;
};

class Bus {
private:
  std::array<IBusDevice *, 65536> memoryMap;

public:
  void mapDevice(IBusDevice *device, uint16_t startAddress,
                 uint16_t endAddress);
  uint8_t read(uint16_t address);
  void write(uint16_t address, uint8_t data);
};

} // namespace nes