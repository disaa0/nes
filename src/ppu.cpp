#include <iostream>
#include <ppu.h>

const int SCREEN_WIDTH = 256;
const int SCREEN_HEIGHT = 240;
const int SCALE_FACTOR = 3;

using namespace nes;

void PPU::reset() {
  registers.reset();
  v = t = 0;
  x = 0;
  w = false;
  scanline = 0;
  cycle = 0;
  ppudata_read_buffer = 0;
  oam.fill(0);
  frame_buffer.fill(0);
}

void PPU::initSDL() {
  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    std::cerr << "SDL could not initialize! SDL_Error: " << SDL_GetError()
              << std::endl;
    return;
  }

  window =
      SDL_CreateWindow("NES Emulator", SDL_WINDOWPOS_UNDEFINED,
                       SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH * SCALE_FACTOR,
                       SCREEN_HEIGHT * SCALE_FACTOR, SDL_WINDOW_SHOWN);
  if (window == nullptr) {
    std::cerr << "Window could not be created! SDL_Error: " << SDL_GetError()
              << std::endl;
    return;
  }

  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
  if (renderer == nullptr) {
    std::cerr << "Renderer could not be created! SDL_Error: " << SDL_GetError()
              << std::endl;
    return;
  }

  texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888,
                              SDL_TEXTUREACCESS_STREAMING, SCREEN_WIDTH,
                              SCREEN_HEIGHT);
  if (texture == nullptr) {
    std::cerr << "Texture could not be created! SDL_Error: " << SDL_GetError()
              << std::endl;
    return;
  }
}
// void PPU::step() {
//     // PPU timing: 341 cycles per scanline, 262 scanlines per frame
//     if (++cycle > 340) {
//         cycle = 0;
//         if (++scanline > 261) {
//             scanline = 0;
//         }
//     }

//     // Visible scanlines
//     if (scanline < 240) {
//         if (cycle == 0) {
//             // Idle cycle
//         } else if (cycle <= 256) {
//             fetch_tile_data();
//             render_pixel();
//         } else if (cycle <= 320) {
//             // Fetch tile data for next scanline
//             fetch_tile_data();
//         } else {
//             // Fetch nametable data for next scanline
//         }
//     }
//     // Post-render scanline
//     else if (scanline == 240) {
//         // Do nothing
//     }
//     // Vertical blanking lines
//     else if (scanline >= 241 && scanline <= 260) {
//         if (scanline == 241 && cycle == 1) {
//             // Set VBlank flag
//             registers.ppustatus |= 0x80;
//             if (registers.ppuctrl & 0x80) {
//                 // Generate NMI
//                 // TODO: Implement NMI generation
//             }
//         }
//     }
//     // Pre-render scanline
//     else if (scanline == 261) {
//         if (cycle == 1) {
//             // Clear VBlank flag
//             registers.ppustatus &= ~0x80;
//         }
//     }
// }

void PPU::step() {
  // PPU timing: 341 cycles per scanline, 262 scanlines per frame
  if (++cycle > 340) {
    cycle = 0;
    if (++scanline > 261) {
      scanline = 0;
      render(); // Render the frame at the end of each frame
    }
  }

  // Visible scanlines
  if (scanline < 240) {
    if (cycle == 0) {
      // Idle cycle
    } else if (cycle <= 256) {
      fetch_tile_data();
      render_pixel();
    } else if (cycle <= 320) {
      // Fetch tile data for next scanline
      fetch_tile_data();
    } else {
      // Fetch nametable data for next scanline
    }
  }
  // Post-render scanline
  else if (scanline == 240) {
    // Do nothing
  }
  // Vertical blanking lines
  else if (scanline >= 241 && scanline <= 260) {
    if (scanline == 241 && cycle == 1) {
      // Set VBlank flag
      registers.ppustatus |= 0x80;
      if (registers.ppuctrl & 0x80) {
        // Generate NMI
        // TODO: Implement NMI generation
      }
    }
  }
  // Pre-render scanline
  else if (scanline == 261) {
    if (cycle == 1) {
      // Clear VBlank flag
      registers.ppustatus &= ~0x80;
    }
  }
}

void PPU::render() {
  SDL_UpdateTexture(texture, nullptr, frame_buffer.data(),
                    SCREEN_WIDTH * sizeof(uint32_t));
  SDL_RenderClear(renderer);
  SDL_RenderCopy(renderer, texture, nullptr, nullptr);
  SDL_RenderPresent(renderer);

  // Handle SDL events
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    if (event.type == SDL_QUIT) {
      // Handle quit event (you might want to signal the emulator to stop)
      SDL_Quit();
      exit(0);
    }
  }
}
// void PPU::render() {
//     // This function would be called at the end of each frame
//     // It should compose the final frame from the rendered pixels
//     // For now, we'll just print a placeholder message
//     std::cout << "Frame rendered\n";
// }

uint8_t PPU::read(uint16_t address) {
  uint8_t data = 0;
  switch (address & 0x7) {
  case 0x2: // PPUSTATUS
    data = registers.ppustatus;
    registers.ppustatus &= ~0x80; // Clear VBlank flag
    w = false;                    // Reset address latch
    break;
  case 0x4: // OAMDATA
    data = oam[registers.oamaddr];
    break;
  case 0x7: // PPUDATA
    data = ppudata_read_buffer;
    ppudata_read_buffer = ppu_read(v);
    if (v >= 0x3F00) { // Palette data is not buffered
      data = ppudata_read_buffer;
    }
    increment_vram_addr();
    break;
  }
  return data;
}

void PPU::write(uint16_t address, uint8_t data) {
  switch (address & 0x7) {
  case 0x0: // PPUCTRL
    registers.ppuctrl = data;
    t = (t & 0xF3FF) | ((data & 0x03) << 10);
    break;
  case 0x1: // PPUMASK
    registers.ppumask = data;
    break;
  case 0x3: // OAMADDR
    registers.oamaddr = data;
    break;
  case 0x4: // OAMDATA
    oam[registers.oamaddr++] = data;
    break;
  case 0x5: // PPUSCROLL
    if (!w) {
      t = (t & 0xFFE0) | (data >> 3);
      x = data & 0x07;
    } else {
      t = (t & 0x8FFF) | ((data & 0x07) << 12);
      t = (t & 0xFC1F) | ((data & 0xF8) << 2);
    }
    w = !w;
    break;
  case 0x6: // PPUADDR
    if (!w) {
      t = (t & 0x00FF) | ((data & 0x3F) << 8);
    } else {
      t = (t & 0xFF00) | data;
      v = t;
    }
    w = !w;
    break;
  case 0x7: // PPUDATA
    ppu_write(v, data);
    increment_vram_addr();
    break;
  }
}

uint8_t PPU::ppu_read(uint16_t address) { return bus->ppu_read(address); }

void PPU::ppu_write(uint16_t address, uint8_t value) {
  bus->ppu_write(address, value);
}

void PPU::increment_vram_addr() { v += (registers.ppuctrl & 0x04) ? 32 : 1; }

void PPU::fetch_tile_data() {
  // TODO: Implement actual tile fetching logic
  // For now, we'll just generate a test pattern
  uint16_t tile_x = cycle / 8;
  uint16_t tile_y = scanline / 8;
  uint8_t color = (tile_x + tile_y) % 4; // Simple checkerboard pattern

  // Store the color in the current pixel of the frame buffer
  frame_buffer[scanline * SCREEN_WIDTH + cycle] = get_color(color);
}

void PPU::render_pixel() {
  // The actual pixel rendering is now done in fetch_tile_data
  // This method can be used for additional per-pixel operations if needed
}

uint32_t PPU::get_color(uint8_t palette_index) {
  // Simple color palette for testing
  static const uint32_t palette[4] = {
      0xFF000000, // Black
      0xFF5555FF, // Red
      0xFF55FF55, // Green
      0xFFFF5555  // Blue
  };
  return palette[palette_index];
}
// void PPU::fetch_tile_data() {
//     // Placeholder implementation
//     // This function should fetch tile data from nametable and pattern
//     table
// }

// void PPU::render_pixel() {
//     // Placeholder implementation
//     // This function should render the current pixel based on fetched tile
//     data
