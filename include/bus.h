#pragma once
#include <cartridge.h>
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
  void cpu_map_device(IBusDevice *device, uint16_t startAddress,
                 uint16_t endAddress);
  void ppu_map_device(IBusDevice *device, uint16_t startAddress,
                 uint16_t endAddress);
  uint8_t cpu_read(uint16_t address);
  void cpu_write(uint16_t address, uint8_t data);
};

class RAM : public IBusDevice {
private:
  std::vector<uint8_t> data;

public:
  RAM(size_t size) : data(size, 0) {}
  uint8_t read(uint16_t address) override {
    // Memory mirroring
    return data[address % data.size()];
  }
  void write(uint16_t address, uint8_t value) override {
    data[address] = value;
  }
};

class PPU : public IBusDevice {
private:
  std::vector<uint8_t> data;

public:
  PPU(size_t size) : data(size, 0) {}
  uint8_t read(uint16_t address) override { return data[address]; }
  void write(uint16_t address, uint8_t value) override {
    data[address % data.size()] = value;
  }
};



class pgrROM : public IBusDevice {
private:
  Cartridge *cartridge;
public:
  pgrROM(Cartridge *cart) : cartridge(cart) {}
  uint8_t read(uint16_t address) {
    return cartridge->mapper->readPRG(address);
  };
  void write(uint16_t address, uint8_t value) {
    return cartridge->mapper->writePRG(address, value);
  };
};

class chrROM : public IBusDevice{
private:
   Cartridge *cartridge;
public:
  chrROM(Cartridge *cart) : cartridge(cart) {}

  uint8_t read(uint16_t address) {
    return cartridge->mapper->readCHR(address);
  };
  void write(uint16_t address, uint8_t value) {
    return cartridge->mapper->writeCHR(address, value);
  };
};

} // namespace nes
