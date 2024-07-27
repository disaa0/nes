#pragma once
#include <array>
#include <cartridge.h>
#include <cstdint>
#include <vector>

namespace nes {
using Byte = std::uint8_t;
using Word = std::uint16_t;
using Address = std::uint16_t;

class BusDevice {
public:
  virtual ~BusDevice() = default;
  virtual uint8_t read(uint16_t address) = 0;
  virtual void write(uint16_t address, uint8_t data) = 0;
};

class RAM : public BusDevice {
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

class PPU_Registers : public BusDevice {
private:
  std::vector<uint8_t> data;

public:
  PPU_Registers(size_t size) : data(size, 0) {}
  uint8_t read(uint16_t address) override { return data[address]; }
  void write(uint16_t address, uint8_t value) override {
    data[address % data.size()] = value;
  }
};

class pgrROM : public BusDevice {
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

class chrROM : public BusDevice {
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

class Bus {
private:
  std::array<BusDevice *, 65536> cpu_memory_map; // 64 KB
  std::array<BusDevice *, 16384> ppu_memory_map; // 16 KB
public:
  void cpu_map_device(BusDevice *device, uint16_t startAddress,
                      uint16_t endAddress);
  uint8_t cpu_read(uint16_t address);
  void cpu_write(uint16_t address, uint8_t data);

  void ppu_map_device(BusDevice *device, uint16_t startAddress,
                      uint16_t endAddress);
  uint8_t ppu_read(uint16_t address);
  void ppu_write(uint16_t address, uint8_t data);
};

} // namespace nes
