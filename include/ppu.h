#pragma once

#include <SDL2/SDL.h>
#include <bus.h>

namespace nes {

class PPU : public BusDevice {
public:
  PPU(Bus *b) : bus(b), scanline(0), cycle(0), ppudata_read_buffer(0) {
    reset();
    initSDL();
  };
  ~PPU() override {
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
  }

  void render();
  void step();
  void reset();

  // Implement BusDevice interface
  uint8_t read(uint16_t address) override;
  void write(uint16_t address, uint8_t data) override;

private:
  std::shared_ptr<Bus> bus; // Shared pointer to the Bus
  // Internals
  std::array<uint8_t, 256> oam; // Object Attribute Memory
  // PPU state
  uint16_t v; // Current VRAM address (15 bits)
  uint16_t t; // Temporary VRAM address (15 bits)
  uint8_t x;  // Fine X scroll (3 bits)
  bool w;     // First or second write toggle
  // Rendering state
  int scanline;
  int cycle;
  // Internal read buffer for PPUDATA reads
  uint8_t ppudata_read_buffer;
  // Frame buffer
  std::array<uint32_t, 256 * 240> frame_buffer;
  // SDL-related members
  SDL_Window *window;
  SDL_Renderer *renderer;
  SDL_Texture *texture;

  // Registers
  //  https://www.nesdev.org/wiki/PPU_registers
  struct Registers {
    Byte ppuctrl;     // $2000 PPU control register
    Byte ppumask;     // $2001 PPU mask register
    Byte ppustatus;   // $2002 PPU status register
    Byte oamaddr;     // $2003 OAM address register
    Byte oamdata;     // $2004 OAM data register
    Byte ppuscroll;   // $2005 PPU scroll register
    Byte ppuaddr;     // $2006 PPU address register
    Byte ppudata;     // $2007 PPU data register
    Byte oamdma;      // $4014 OAM DMA register
    bool odd_frame;   // Tracks whether the current frame is odd or even
    Byte read_buffer; // Internal read buffer for PPUDATA

    // Latch for $2005 and $2006
    bool address_latch;

    // https://www.nesdev.org/wiki/PPU_power_up_state
    Registers() {
      // Set initial values at power-on
      ppuctrl = 0x00;
      ppumask = 0x00;
      ppustatus = 0x00; // The +0+x bits are handled by the PPU logic
      oamaddr = 0x00;
      ppuscroll = 0x00;
      ppuaddr = 0x00;
      ppudata = 0x00;
      read_buffer = 0x00;
      odd_frame = false;
      address_latch = false;
      // oamdata and oamdma are left uninitialized as per the table
    }

    void reset() {
      ppuctrl = 0x00;
      ppumask = 0x00;
      // ppustatus is unchanged on reset
      // oamaddr is unchanged on reset
      ppuscroll = 0x00;
      // ppuaddr is unchanged on reset
      read_buffer = 0x00;
      odd_frame = false;
      address_latch = false;
      // oamdata, oamdma, and ppudata are left unchanged as per the table
    }
  } registers;

  // Helper functions
  uint8_t ppu_read(uint16_t address);
  void ppu_write(uint16_t address, uint8_t value);
  void increment_vram_addr();

  // Rendering functions
  void fetch_tile_data();
  void render_pixel();
  uint32_t get_color(uint8_t palette_index);

  // SDL
  void initSDL();
};
} // namespace nes
