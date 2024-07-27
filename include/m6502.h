#pragma once

#include <bus.h>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace nes {

enum AddressingMode {
  Implied,
  Accumulator,
  Immediate,
  ZeroPage,
  ZeroPageX,
  ZeroPageY,
  Relative,
  Absolute,
  AbsoluteX,
  AbsoluteY,
  Indirect,
  IndexedIndirect,
  IndirectIndexed
};

enum Operation {
  ADC,
  AND,
  ASL,
  BCC,
  BCS,
  BEQ,
  BIT,
  BMI,
  BNE,
  BPL,
  BRK,
  BVC,
  BVS,
  CLC,
  CLD,
  CLI,
  CLV,
  CMP,
  CPX,
  CPY,
  DEC,
  DEX,
  DEY,
  EOR,
  INC,
  INX,
  INY,
  JMP,
  JSR,
  LDA,
  LDX,
  LDY,
  LSR,
  NOP,
  ORA,
  PHA,
  PHP,
  PLA,
  PLP,
  ROL,
  RTI,
  ROR,
  RTS,
  SBC,
  SEC,
  SED,
  SEI,
  STA,
  STX,
  STY,
  TAX,
  TAY,
  TSX,
  TXA,
  TXS,
  TYA,
  SLO,
  RLA,
  SRE,
  RRA,
  SAX,
  LAX,
  DCP,
  ISC,
  ANC,
  ALR,
  ARR,
  XAA,
  AXS,
  AHX,
  SHY,
  SHX,
  TAS,
  LAS,
  KIL
};

class CPU {
public:
  CPU(Bus *b, double clockSpeedHz = 1789773)
      : clockSpeedHz_(clockSpeedHz), bus(b){};
  ~CPU() = default;

  // Types
  // Instruction control
  struct Instruction {
    Byte opcode;
    Operation mnemonic;
    std::uint32_t cycles;
    AddressingMode mode;
    bool needsExtraCycle;

    Instruction(Byte op, Operation mn, std::uint32_t cyc, AddressingMode m,
                bool ec)
        : opcode(op), mnemonic(mn), cycles(cyc), mode(m), needsExtraCycle(ec) {}

    Instruction()
        : opcode(0), mnemonic(), cycles(0), mode(AddressingMode::Implied),
          needsExtraCycle(false) {}
  };

  // Functions
  void reset();
  uint64_t step();
  AddressingMode detectAddressingMode(const std::string &instruction);
  std::vector<uint8_t> get_next_operation();

  // Cycle counting
  uint64_t cycles_; // cycles each step took
  double clockSpeedHz_;
  std::chrono::high_resolution_clock::time_point lastCycleTime_;

  // Helper Tables
  std::unordered_map<Byte, Instruction> opcodeTable_;
  std::unordered_map<Operation, std::vector<Instruction>> operationTable_;

private:
  std::shared_ptr<Bus> bus; // Shared pointer to the Bus
  struct StatusFlags {
    bool C : 1; // Carry
    bool Z : 1; // Zero
    bool I : 1; // Interrupt Disable
    bool D : 1; // Decimal Mode
    bool B : 1; // B Flag
    bool U : 1; // U Flag (not used)
    bool V : 1; // Overflow
    bool N : 1; // Negative
  };

  struct Registers {
    Word PC;       // Program Counter
    Byte SP;       // Stack Pointer
    Byte A;        // Accumulator
    Byte X;        // Index Register X
    Byte Y;        // Index Register Y
    StatusFlags P; // Status Register

    void reset() {
      PC = 0;
      SP = 0xFF; // Initialize SP to 0xFF
      A = X = Y = 0;
      P = {false, false, true, false, true, true, false, false};
    }
  };

  // Registers
  Registers registers;

  // Combined operation functions
  void LDA(const Instruction &instruction);
  void LDX(const Instruction &instruction);
  void LDY(const Instruction &instruction);
  void STA(const Instruction &instruction);
  void STX(const Instruction &instruction);
  void STY(const Instruction &instruction);
  void ADC(const Instruction &instruction);
  void AND(const Instruction &instruction);
  void ASL(const Instruction &instruction);
  void BCC(const Instruction &instruction);
  void BCS(const Instruction &instruction);
  void BEQ(const Instruction &instruction);
  void BIT(const Instruction &instruction);
  void BMI(const Instruction &instruction);
  void BNE(const Instruction &instruction);
  void BPL(const Instruction &instruction);
  void BRK(const Instruction &instruction);
  void BVC(const Instruction &instruction);
  void BVS(const Instruction &instruction);
  void CLC(const Instruction &instruction);
  void CLD(const Instruction &instruction);
  void CLI(const Instruction &instruction);
  void CLV(const Instruction &instruction);
  void CMP(const Instruction &instruction);
  void CPX(const Instruction &instruction);
  void CPY(const Instruction &instruction);
  void DEC(const Instruction &instruction);
  void DEX(const Instruction &instruction);
  void DEY(const Instruction &instruction);
  void EOR(const Instruction &instruction);
  void INC(const Instruction &instruction);
  void INX(const Instruction &instruction);
  void INY(const Instruction &instruction);
  void JMP(const Instruction &instruction);
  void JSR(const Instruction &instruction);
  void LSR(const Instruction &instruction);
  void NOP(const Instruction &instruction);
  void ORA(const Instruction &instruction);
  void PHA(const Instruction &instruction);
  void PHP(const Instruction &instruction);
  void PLA(const Instruction &instruction);
  void PLP(const Instruction &instruction);
  void ROL(const Instruction &instruction);
  void ROR(const Instruction &instruction);
  void RTI(const Instruction &instruction);
  void RTS(const Instruction &instruction);
  void SBC(const Instruction &instruction);
  void SEC(const Instruction &instruction);
  void SED(const Instruction &instruction);
  void SEI(const Instruction &instruction);
  void TAX(const Instruction &instruction);
  void TAY(const Instruction &instruction);
  void TSX(const Instruction &instruction);
  void TXA(const Instruction &instruction);
  void TXS(const Instruction &instruction);
  void TYA(const Instruction &instruction);
  void SLO(const Instruction &instruction);
  void RLA(const Instruction &instruction);
  void SRE(const Instruction &instruction);
  void RRA(const Instruction &instruction);
  void SAX(const Instruction &instruction);
  void LAX(const Instruction &instruction);
  void DCP(const Instruction &instruction);
  void ISC(const Instruction &instruction);
  void ANC(const Instruction &instruction);
  void ALR(const Instruction &instruction);
  void ARR(const Instruction &instruction);
  void XAA(const Instruction &instruction);
  void AXS(const Instruction &instruction);
  void AHX(const Instruction &instruction);
  void SHY(const Instruction &instruction);
  void SHX(const Instruction &instruction);
  void TAS(const Instruction &instruction);
  void LAS(const Instruction &instruction);
  void KIL(const Instruction &instruction);

  void initializeTables();

  // Operations
  Byte fetchOperand(AddressingMode mode, bool read_only = false);
  Word fetchAddress(AddressingMode mode, bool read_only = false);
  void updateZeroAndNegativeFlags(Byte value);
  void handlePageCrossing(const Instruction &instruction);
};

} // namespace nes
