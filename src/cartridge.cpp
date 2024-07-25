#include <bus.h>
#include <cartridge.h>
#include <cstring>
#include <fstream>
#include <iostream>

using namespace nes;
// https://www.nesdev.org/wiki/INES
void Cartridge::loadROM(const std::string &filename) {

  std::ifstream file(filename, std::ios::binary);
  if (!file) {
    throw std::runtime_error("Could not open ROM file");
  }

  // Read header
  char header[16];
  file.read(header, 16);

  // Check NES header
  if (std::strncmp(header, "NES\x1A", 4) != 0) {
    throw std::runtime_error("Invalid NES ROM format");
  }

  // Parse header
  uint8_t prgROMSize = header[4] * 16 * 1024; // 16KB units
  uint8_t chrROMSize = header[5] * 8 * 1024;  // 8KB units
  uint8_t flags6 = header[6];
  uint8_t flags7 = header[7];

  mapperNumber = ((flags7 & 0xF0) | (flags6 >> 4)) & 0xFF;
  hasTrainer = flags6 & 0x04;

  // Read trainer if present
  if (hasTrainer) {
    file.seekg(512, std::ios::cur);
  }

  // Read PRG-ROM
  prgROM.resize(prgROMSize);
  file.read(reinterpret_cast<char *>(prgROM.data()), prgROMSize);
  file.read(reinterpret_cast<char *>(prgROM.data()), prgROMSize);

  // Read CHR-ROM
  if (chrROMSize > 0) {
    chrROM.resize(chrROMSize);
    file.read(reinterpret_cast<char *>(chrROM.data()), chrROMSize);
  } else {
    // If CHR-ROM size is 0, it uses CHR-RAM
    chrROM.resize(8 * 1024); // Default to 8KB CHR-RAM
  }

  // Initialize SRAM if battery-backed RAM is present
  if (flags6 & 0x02) {
    sram.resize(8 * 1024); // 8KB SRAM
  }

  // Create appropriate mapper
  createMapper();

  std::cout << "Loaded ROM: " << filename << std::endl;
  std::cout << "Mapper: " << static_cast<int>(mapperNumber) << std::endl;
  std::cout << "PRG-ROM size: " << prgROM.size() << " bytes" << std::endl;
  std::cout << "CHR-ROM size: " << chrROM.size() << " bytes" << std::endl;

}

class NROMMapper : public Mapper {
private:
  std::vector<uint8_t> &prgROM;
  std::vector<uint8_t> &chrROM;
  bool is32KBPRG;

public:
  NROMMapper(std::vector<uint8_t> &prgROM, std::vector<uint8_t> &chrROM)
      : prgROM(prgROM), chrROM(chrROM) {
    is32KBPRG = (prgROM.size() == 32 * 1024);
  }

  uint8_t readPRG(uint16_t address) override {
    if (address >= 0x8000 && address <= 0xFFFF) {
      if (is32KBPRG) {
        return prgROM[address - 0x8000];
      } else {
        // For 16KB PRG-ROM, mirror the 16KB from 0x8000-0xBFFF to 0xC000-0xFFFF
        return prgROM[(address - 0x8000) & 0x3FFF];
      }
    }
    return 0; // Or handle error
  }

  void writePRG(uint16_t address, uint8_t value) override {
    // NROM doesn't allow writing to PRG-ROM
    // Ignore write or handle error
  }

  uint8_t readCHR(uint16_t address) override {
    if (address <= 0x1FFF) {
      return chrROM[address];
    }
    return 0; // Or handle error
  }

  void writeCHR(uint16_t address, uint8_t value) override {
    // If CHR-ROM is actually CHR-RAM, allow writing
    if (address <= 0x1FFF && chrROM.size() == 8 * 1024) {
      chrROM[address] = value;
    }
    // Otherwise, ignore write or handle error
  }
};

void Cartridge::createMapper() {

  switch (mapperNumber) {
  case 0:
    mapper = std::make_unique<NROMMapper>(prgROM, chrROM);
    break;
  // case 1:
  //   mapper = std::make_unique<MMC1Mapper>(prgROM, chrROM);
  //   break;
  // case 2:
  //   mapper = std::make_unique<UNROMMapper>(prgROM, chrROM);
  //   break;
  // case 3:
  //   mapper = std::make_unique<CNROMMapper>(prgROM, chrROM);
  //   break;
  // case 4:
  //   mapper = std::make_unique<MMC3Mapper>(prgROM, chrROM);
  //   break;
  // case 7:
  //   mapper = std::make_unique<AXROMMapper>(prgROM, chrROM);
  //   break;
  // case 66:
  //   mapper = std::make_unique<GxROMMapper>(prgROM, chrROM);
  //   break;
  default:
    throw std::runtime_error("Unsupported mapper type");
  }
}
