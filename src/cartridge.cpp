#include <cartrige.h>
#include <cstring>
#include <fstream>
#include <iostream>

using namespace nes;

void Cartridge::loadROM(const std::string& filename) {
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
        uint8_t prgROMSize = header[4] * 16 * 1024;  // 16KB units
        uint8_t chrROMSize = header[5] * 8 * 1024;   // 8KB units
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
        file.read(reinterpret_cast<char*>(prgROM.data()), prgROMSize);

        // Read CHR-ROM
        if (chrROMSize > 0) {
            chrROM.resize(chrROMSize);
            file.read(reinterpret_cast<char*>(chrROM.data()), chrROMSize);
        } else {
            // If CHR-ROM size is 0, it uses CHR-RAM
            chrROM.resize(8 * 1024);  // Default to 8KB CHR-RAM
        }

        // Initialize SRAM if battery-backed RAM is present
        if (flags6 & 0x02) {
            sram.resize(8 * 1024);  // 8KB SRAM
        }

        // Create appropriate mapper
        createMapper();

        std::cout << "Loaded ROM: " << filename << std::endl;
        std::cout << "Mapper: " << static_cast<int>(mapperNumber) << std::endl;
        std::cout << "PRG-ROM size: " << prgROM.size() << " bytes" << std::endl;
        std::cout << "CHR-ROM size: " << chrROM.size() << " bytes" << std::endl;
    }

// void Cartridge::loadROM(const std::string &filename) {
//   std::ifstream file(filename,
//                      std::ios::binary); // Open the binary file

//   if (!file.is_open()) {
//     std::cerr << "Error opening file." << std::endl;
//     return;
//   }

//   // Determine the file size
//   file.seekg(0, std::ios::end);
//   std::streamsize fileSize = file.tellg();
//   file.seekg(0, std::ios::beg); // Reset file pointer to the beginning

//   file.read(reinterpret_cast<char *>(romData.data()), fileSize);
//   file.close();

//   // Now 'data' contains the binary data from the file
//   std::cout << "File size: " << fileSize << std::endl;
// }

void Cartridge::createMapper() {
        // Here you would create the appropriate mapper based on mapperNumber
        // For this example, we'll just use a dummy mapper
        class DummyMapper : public Mapper {
        public:
            uint8_t readPRG(uint16_t address) override { return 0; }
            uint8_t readCHR(uint16_t address) override { return 0; }
            void writePRG(uint16_t address, uint8_t value) override {}
            void writeCHR(uint16_t address, uint8_t value) override {}
        };

        mapper = std::make_unique<DummyMapper>();
    }