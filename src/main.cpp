#include <m6502.h>
#include <string>
int main() {
  m6502::CPU cpu;
  //std::string instruction { "LDA #$0" };
  //m6502::AddressingMode mode = cpu.detectAddressingMode(instruction);
  std::vector<uint8_t> program = {
      0xA9, 0x05, // LDA #$05    ; Load 5 into accumulator
      0x85, 0x10, // STA $10     ; Store accumulator to zero page address $10
      0xA2, 0x0A, // LDX #$0A    ; Load 10 into X register
      0xA0, 0x15, // LDY #$15    ; Load 21 into Y register
      0x18,       // CLC         ; Clear carry flag
      0x69, 0x03, // ADC #$03    ; Add 3 to accumulator (should be 8 now)
      0x65, 0x10, // ADC $10     ; Add value at zero page address $10 (5) to
                  // accumulator (should be 13 now)
      0x8D, 0x00,
      0x20, // STA $2000   ; Store accumulator to absolute address $2000
      0xAD, 0x00, 0x20, // LDA $2000   ; Load from absolute address $2000
      0x29, 0x0F,       // AND #$0F    ; AND accumulator with 0F (should be 0D)
      0x49, 0xFF,       // EOR #$FF    ; XOR accumulator with FF (should be F2)
      0x09, 0x03,       // ORA #$03    ; OR accumulator with 03 (should be F3)
      0xC9,
      0xF0,       // CMP #$F0    ; Compare accumulator with F0 (should set carry
                  // flag)
      0xB0, 0x02, // BCS +2      ; Branch if carry set (should branch)
      0xA9, 0x00, // LDA #$00    ; This should be skipped
      0xE8,       // INX         ; Increment X (should be 11 now)
      0xCA,       // DEX         ; Decrement X (should be 10 again)
      0xE0, 0x0A, // CPX #$0A    ; Compare X with 0A (should set zero flag)
      0xF0, 0x02, // BEQ +2      ; Branch if zero (should branch)
      0xA2, 0x00, // LDX #$00    ; This should be skipped
      0x38,       // SEC         ; Set carry flag
      0xA9, 0x05, // LDA #$05    ; Load 5 into accumulator
      0xE9, 0x03, // SBC #$03    ; Subtract 3 from accumulator (should be 2)
      0x0A, // ASL A       ; Arithmetic shift left accumulator (should be 4)
      0x4A, // LSR A       ; Logical shift right accumulator (should be 2)
      0x2A, // ROL A       ; Rotate left accumulator (should be 4 with carry
            // set)
      0x6A, // ROR A       ; Rotate right accumulator (should be 2 with carry
            // set)
      0xA9, 0xFF, // LDA #$FF    ; Load FF into accumulator
      0x48,       // PHA         ; Push accumulator to stack
      0xA9, 0x00, // LDA #$00    ; Load 00 into accumulator
      0x68,       // PLA         ; Pull from stack to accumulator (should be FF
                  // again)
      0x20, 0x50, 0x00, // JSR $0050   ; Jump to subroutine at $0050
      0xEA,             // NOP         ; No operation
      0x00,             // BRK         ; Break
      // Subroutine at $0050
      0xE6, 0x10, // INC $10     ; Increment value at zero page address $10
      0xC6, 0x10, // DEC $10     ; Decrement value at zero page address $10
      0x60        // RTS         ; Return from subroutine
  };
  cpu.run(program,'b');
}
