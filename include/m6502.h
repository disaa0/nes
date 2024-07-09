#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace m6502 {

using Byte = std::uint8_t;
using Word = std::uint16_t;
using Address = std::uint16_t;

class CPU {
public:
  enum class AddressingMode {
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
  enum class Operation {
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
    ROR,
    RTI,
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
    TYA
  };

  CPU() = default;
  ~CPU() = default;

  void reset();
  void step();
  // template <typename T> 
  void run(std::vector<std::string> program, const char mode = 'b');

  AddressingMode detectAddressingMode(const std::string &instruction);

private:
  static constexpr std::size_t MAX_MEMORY = 64 * 1024; // 64KB

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

  struct Instruction {
    Byte opcode;
    Operation mnemonic;
    std::uint32_t cycles;
    AddressingMode mode;

    Instruction(Byte op, Operation mn, std::uint32_t cyc, AddressingMode m)
        : opcode(op), mnemonic(mn), cycles(cyc), mode(m) {}

    Instruction()
        : opcode(0), mnemonic(), cycles(0), mode(AddressingMode::Implied) {}
  };

  struct InstructionKey {
    Operation mnemonic;
    AddressingMode mode;

    bool operator==(const InstructionKey &other) const {
      return mnemonic == other.mnemonic && mode == other.mode;
    }
  };

  struct InstructionKeyHash {
    std::size_t operator()(const InstructionKey &key) const {
      std::size_t h1 = std::hash<int>{}(static_cast<int>(key.mnemonic));
      std::size_t h2 = std::hash<int>{}(static_cast<int>(key.mode));
      return h1 ^ (h2 << 1); // or use boost::hash_combine
    }
  };

  Registers registers_;
  std::array<Byte, MAX_MEMORY> memory_;
  std::unordered_map<Byte, Instruction> opcodeTable_;
  std::unordered_map<InstructionKey, Instruction, InstructionKeyHash>
      translationTable_;
  std::unordered_map<Operation, std::vector<Instruction>> operationTable_;

  // Memory Access
  Byte read(Word address) const { return memory_[address]; }
  void write(Word address, Byte value) { memory_[address] = value; }

  // Cycle counting
  std::uint64_t totalCycles = 0;
  void addCycles(std::uint32_t cycles) { totalCycles += cycles; }

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

  void initializeTables();

  // Helper functions
  Byte fetchOperand(AddressingMode mode);
  Word fetchAddress(AddressingMode mode);
  void updateZeroAndNegativeFlags(Byte value);
  std::vector<std::string> disassemble(const std::vector<uint8_t> &code);
  std::string mnemonicToString(Operation mnemonic);
  std::string byteToHex(uint8_t byte);
  std::string wordToHex(uint16_t word);
  std::vector<uint8_t> assemble(const std::vector<std::string> &code);
  Operation stringToMnemonic(const std::string &str);
  bool matchAddressingMode(const std::string &operand, AddressingMode mode);
  uint16_t parseOperand(const std::string &operand, AddressingMode mode);
  bool isLabel(const std::string &operand);
};

} // namespace m6502