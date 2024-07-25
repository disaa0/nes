#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
namespace nes {

class Mapper {
public:
  virtual ~Mapper() = default;

  virtual uint8_t readPRG(uint16_t address) = 0;
  virtual uint8_t readCHR(uint16_t address) = 0;
  virtual void writePRG(uint16_t address, uint8_t value) = 0;
  virtual void writeCHR(uint16_t address, uint8_t value) = 0;
};

class Bus;
class Cartridge {
public:
  Cartridge(){};
  ~Cartridge() = default;
  void loadROM(const std::string &filename);

  std::unique_ptr<Mapper> mapper;

  std::vector<uint8_t> prgROM;
  std::vector<uint8_t> chrROM;
  std::vector<uint8_t> sram;

private:
  uint8_t mapperNumber;
  bool hasTrainer;

  void createMapper();
};
} // namespace nes
