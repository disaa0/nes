#pragma once
#include <array>
#include <cstdint>
#include <vector>

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

class RAM : public IBusDevice {
private:
    std::vector<uint8_t> data;
public:
    RAM(size_t size) : data(size, 0) {}
    uint8_t read(uint16_t address) override { return data[address]; }
    void write(uint16_t address, uint8_t value) override { data[address] = value; }
};

class ROM : public IBusDevice {
private:
    std::vector<uint8_t> data;
public:
    ROM(const std::vector<uint8_t>& romData) : data(romData) {}
    uint8_t read(uint16_t address) override { return data[address]; }
    void write(uint16_t address, uint8_t value) override { /* Ignore writes */ }
};

} // namespace nes