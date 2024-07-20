#pragma once
#include <array>
#include <cartrige.h>
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

class prgROM : public IBusDevice {
private:
  std::shared_ptr<Cartridge> cartridge_;
  std::vector<uint8_t> data;

public:
  prgROM(std::shared_ptr<Cartridge> cartridge,
         const std::vector<uint8_t> &romData)
      : cartridge_(cartridge), data(romData) {}
  uint8_t read(uint16_t address) override {
    return cartridge_->mapper->readPRG(address);
  };
  void write (uint16_t address, uint8_t value) override {
    return cartridge_->mapper->writePRG(address, value);
  };};

class chrROM : public IBusDevice {
private:
  std::shared_ptr<Cartridge> cartridge_;
  std::vector<uint8_t> data;

public:
  chrROM(std::shared_ptr<Cartridge> cartridge,
         const std::vector<uint8_t> &romData)
      : cartridge_(cartridge), data(romData) {}
  uint8_t read(uint16_t address) override {
    return cartridge_->mapper->readCHR(address);
  };
  void write (uint16_t address, uint8_t value) override {
    return cartridge_->mapper->writeCHR(address, value);
  };
};

} // namespace nes