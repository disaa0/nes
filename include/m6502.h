#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace m6502 {

using Byte = std::uint8_t;
using Word = std::uint16_t;

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

  CPU() = default;
  ~CPU() = default;

  void reset();
  void step();
  void run();

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
      SP = 0;
      A = 0;
      X = 0;
      Y = 0;
      P = {false, false, true, false, true, true, false, false};
    }
  };

  struct Instruction {
    Word opcode;
    std::string mnemonic;
    std::uint32_t cycles;
    AddressingMode mode;

    Instruction(Word op, std::string mn, std::uint32_t cyc, AddressingMode m)
        : opcode(op), mnemonic(std::move(mn)), cycles(cyc), mode(m) {}

    // Default constructor
    Instruction()
        : opcode(0), mnemonic(), cycles(0), mode(AddressingMode::Implied) {}
  };

  // Custom key type for translationTable_
  struct InstructionKey {
    std::string mnemonic;
    CPU::AddressingMode mode;

    bool operator==(const InstructionKey &other) const {
      return mnemonic == other.mnemonic && mode == other.mode;
    }
  };

  // Custom hash function for InstructionKey
  // struct InstructionKeyHash {
  //   std::size_t operator()(const InstructionKey &key) const {
  //     return std::hash<std::string>()(key.mnemonic) ^
  //            (std::hash<int>()(static_cast<int>(key.mode)) << 1);
  //   }
  // };
  struct InstructionKeyHash {
    std::size_t operator()(const InstructionKey &key) const {
      std::size_t h1 = std::hash<std::string>{}(key.mnemonic);
      std::size_t h2 = std::hash<int>{}(static_cast<int>(key.mode));
      return h1 ^ (h2 << 1); // or use boost::hash_combine if available
    }
  };

  Registers registers_;
  std::array<Byte, MAX_MEMORY> memory_;
  std::unordered_map<Word, Instruction> opcodeTable_;
  std::unordered_map<InstructionKey, Instruction, InstructionKeyHash>
      translationTable_;

  // Memory Access
  // Byte read(Word& address) const;
  // Word readWord(Word& address);
  Byte read(Word address) const { return memory_[address]; }

  // Word readWord(Word &address) {
  //   Word lowByte = static_cast<Word>(read(address++));
  //   Word highByte = static_cast<Word>(read(address++)) << 8;
  //   return lowByte | highByte;
  // }

  void write(Word address, Byte value);

  void addCycles(std::uint32_t cycles) {
    // Implement cycle counting logic here
    // For example:
    totalCycles += cycles;
  }

  // Operations
  // Load/Store
  void LDA();
  void LDX();
  void LDY();
  void STA();
  void STX();
  void STY();
  // Transfer
  void TAX();
  void TAY();
  void TXA();
  void TYA();
  // Stack
  void TSX();
  void TXS();
  void PHA();
  void PHP();
  void PLA();
  void PLP();
  // Logical
  void AND();
  void EOR();
  void ORA();
  void BIT();
  // Arithmetic
  void ADC();
  void SBC();
  void CMP();
  void CPX();
  void CPY();
  // Increments/Decrements
  void INC();
  void INX();
  void INY();
  void DEC();
  void DEX();
  void DEY();
  // Shifts
  void ASL();
  void LSR();
  void ROL();
  void ROR();
  // Jumps and Calls
  void JMP();
  void JSR();
  void RTS();
  // Branches
  void BCC();
  void BCS();
  void BEQ();
  void BMI();
  void BNE();
  void BPL();
  void BVC();
  void BVS();
  // Flags
  void CLC();
  void CLD();
  void CLI();
  void CLV();
  void SEC();
  void SED();
  void SEI();
  // System
  void BRK();
  void NOP();
  void RTI();

  void initializeTables();

  using ExecuteFunction = std::function<void(CPU &, const Instruction &)>;
  std::unordered_map<std::string, ExecuteFunction> executeTable_;

  void initializeExecuteTable();

  static void executeLDA(CPU &cpu, const Instruction &instruction) {
    cpu.LDA();
    cpu.addCycles(instruction.cycles);
  }

  static void executeLDX(CPU &cpu, const Instruction &instruction) {
    cpu.LDX();
    cpu.addCycles(instruction.cycles);
  }

  static void executeLDY(CPU &cpu, const Instruction &instruction) {
    cpu.LDY();
    cpu.addCycles(instruction.cycles);
  }

  static void executeSTA(CPU &cpu, const Instruction &instruction) {
    cpu.STA();
    cpu.addCycles(instruction.cycles);
  }

  static void executeSTX(CPU &cpu, const Instruction &instruction) {
    cpu.STX();
    cpu.addCycles(instruction.cycles);
  }

  static void executeSTY(CPU &cpu, const Instruction &instruction) {
    cpu.STY();
    cpu.addCycles(instruction.cycles);
  }

  static void executeTAX(CPU &cpu, const Instruction &instruction) {
    cpu.TAX();
    cpu.addCycles(instruction.cycles);
  }

  static void executeTAY(CPU &cpu, const Instruction &instruction) {
    cpu.TAY();
    cpu.addCycles(instruction.cycles);
  }

  static void executeTXA(CPU &cpu, const Instruction &instruction) {
    cpu.TXA();
    cpu.addCycles(instruction.cycles);
  }

  static void executeTYA(CPU &cpu, const Instruction &instruction) {
    cpu.TYA();
    cpu.addCycles(instruction.cycles);
  }

  static void executeTSX(CPU &cpu, const Instruction &instruction) {
    cpu.TSX();
    cpu.addCycles(instruction.cycles);
  }

  static void executeTXS(CPU &cpu, const Instruction &instruction) {
    cpu.TXS();
    cpu.addCycles(instruction.cycles);
  }

  static void executePHA(CPU &cpu, const Instruction &instruction) {
    cpu.PHA();
    cpu.addCycles(instruction.cycles);
  }

  static void executePHP(CPU &cpu, const Instruction &instruction) {
    cpu.PHP();
    cpu.addCycles(instruction.cycles);
  }

  static void executePLA(CPU &cpu, const Instruction &instruction) {
    cpu.PLA();
    cpu.addCycles(instruction.cycles);
  }

  static void executePLP(CPU &cpu, const Instruction &instruction) {
    cpu.PLP();
    cpu.addCycles(instruction.cycles);
  }

  static void executeAND(CPU &cpu, const Instruction &instruction) {
    cpu.AND();
    cpu.addCycles(instruction.cycles);
  }

  static void executeEOR(CPU &cpu, const Instruction &instruction) {
    cpu.EOR();
    cpu.addCycles(instruction.cycles);
  }

  static void executeORA(CPU &cpu, const Instruction &instruction) {
    cpu.ORA();
    cpu.addCycles(instruction.cycles);
  }

  static void executeBIT(CPU &cpu, const Instruction &instruction) {
    cpu.BIT();
    cpu.addCycles(instruction.cycles);
  }

  static void executeADC(CPU &cpu, const Instruction &instruction) {
    cpu.ADC();
    cpu.addCycles(instruction.cycles);
  }

  static void executeSBC(CPU &cpu, const Instruction &instruction) {
    cpu.SBC();
    cpu.addCycles(instruction.cycles);
  }

  static void executeCMP(CPU &cpu, const Instruction &instruction) {
    cpu.CMP();
    cpu.addCycles(instruction.cycles);
  }

  static void executeCPX(CPU &cpu, const Instruction &instruction) {
    cpu.CPX();
    cpu.addCycles(instruction.cycles);
  }

  static void executeCPY(CPU &cpu, const Instruction &instruction) {
    cpu.CPY();
    cpu.addCycles(instruction.cycles);
  }

  static void executeINC(CPU &cpu, const Instruction &instruction) {
    cpu.INC();
    cpu.addCycles(instruction.cycles);
  }

  static void executeINX(CPU &cpu, const Instruction &instruction) {
    cpu.INX();
    cpu.addCycles(instruction.cycles);
  }

  static void executeINY(CPU &cpu, const Instruction &instruction) {
    cpu.INY();
    cpu.addCycles(instruction.cycles);
  }

  static void executeDEC(CPU &cpu, const Instruction &instruction) {
    cpu.DEC();
    cpu.addCycles(instruction.cycles);
  }

  static void executeDEX(CPU &cpu, const Instruction &instruction) {
    cpu.DEX();
    cpu.addCycles(instruction.cycles);
  }

  static void executeDEY(CPU &cpu, const Instruction &instruction) {
    cpu.DEY();
    cpu.addCycles(instruction.cycles);
  }

  static void executeASL(CPU &cpu, const Instruction &instruction) {
    cpu.ASL();
    cpu.addCycles(instruction.cycles);
  }

  static void executeLSR(CPU &cpu, const Instruction &instruction) {
    cpu.LSR();
    cpu.addCycles(instruction.cycles);
  }

  static void executeROL(CPU &cpu, const Instruction &instruction) {
    cpu.ROL();
    cpu.addCycles(instruction.cycles);
  }

  static void executeROR(CPU &cpu, const Instruction &instruction) {
    cpu.ROR();
    cpu.addCycles(instruction.cycles);
  }

  static void executeJMP(CPU &cpu, const Instruction &instruction) {
    cpu.JMP();
    cpu.addCycles(instruction.cycles);
  }

  static void executeJSR(CPU &cpu, const Instruction &instruction) {
    cpu.JSR();
    cpu.addCycles(instruction.cycles);
  }

  static void executeRTS(CPU &cpu, const Instruction &instruction) {
    cpu.RTS();
    cpu.addCycles(instruction.cycles);
  }

  static void executeBCC(CPU &cpu, const Instruction &instruction) {
    cpu.BCC();
    cpu.addCycles(instruction.cycles);
  }

  static void executeBCS(CPU &cpu, const Instruction &instruction) {
    cpu.BCS();
    cpu.addCycles(instruction.cycles);
  }

  static void executeBEQ(CPU &cpu, const Instruction &instruction) {
    cpu.BEQ();
    cpu.addCycles(instruction.cycles);
  }

  static void executeBMI(CPU &cpu, const Instruction &instruction) {
    cpu.BMI();
    cpu.addCycles(instruction.cycles);
  }

  static void executeBNE(CPU &cpu, const Instruction &instruction) {
    cpu.BNE();
    cpu.addCycles(instruction.cycles);
  }

  static void executeBPL(CPU &cpu, const Instruction &instruction) {
    cpu.BPL();
    cpu.addCycles(instruction.cycles);
  }

  static void executeBVC(CPU &cpu, const Instruction &instruction) {
    cpu.BVC();
    cpu.addCycles(instruction.cycles);
  }

  static void executeBVS(CPU &cpu, const Instruction &instruction) {
    cpu.BVS();
    cpu.addCycles(instruction.cycles);
  }

  static void executeCLC(CPU &cpu, const Instruction &instruction) {
    cpu.CLC();
    cpu.addCycles(instruction.cycles);
  }

  static void executeCLD(CPU &cpu, const Instruction &instruction) {
    cpu.CLD();
    cpu.addCycles(instruction.cycles);
  }

  static void executeCLI(CPU &cpu, const Instruction &instruction) {
    cpu.CLI();
    cpu.addCycles(instruction.cycles);
  }

  static void executeCLV(CPU &cpu, const Instruction &instruction) {
    cpu.CLV();
    cpu.addCycles(instruction.cycles);
  }

  static void executeSEC(CPU &cpu, const Instruction &instruction) {
    cpu.SEC();
    cpu.addCycles(instruction.cycles);
  }

  static void executeSED(CPU &cpu, const Instruction &instruction) {
    cpu.SED();
    cpu.addCycles(instruction.cycles);
  }

  static void executeSEI(CPU &cpu, const Instruction &instruction) {
    cpu.SEI();
    cpu.addCycles(instruction.cycles);
  }

  static void executeBRK(CPU &cpu, const Instruction &instruction) {
    cpu.BRK();
    cpu.addCycles(instruction.cycles);
  }

  static void executeNOP(CPU &cpu, const Instruction &instruction) {
    cpu.addCycles(instruction.cycles);
  }

  static void executeRTI(CPU &cpu, const Instruction &instruction) {
    cpu.RTI();
    cpu.addCycles(instruction.cycles);
  }

  // Helper functions
  Byte fetchOperand(AddressingMode mode) {
    switch (mode) {
    case AddressingMode::Immediate:
      return read(registers_.PC++);
    case AddressingMode::ZeroPage:
      return read(read(registers_.PC++));
    case AddressingMode::ZeroPageX:
      return read((read(registers_.PC++) + registers_.X) & 0xFF);
    case AddressingMode::ZeroPageY:
      return read((read(registers_.PC++) + registers_.Y) & 0xFF);
    case AddressingMode::Absolute:
      return read(fetchAddress(AddressingMode::Absolute));
    case AddressingMode::AbsoluteX:
      return read(fetchAddress(AddressingMode::Absolute) + registers_.X);
    case AddressingMode::AbsoluteY:
      return read(fetchAddress(AddressingMode::Absolute) + registers_.Y);

    default:
      throw std::runtime_error("Unimplemented addressing mode");
    }
  }

  Word fetchAddress(AddressingMode mode) {
    switch (mode) {
    case AddressingMode::ZeroPage:
      return read(registers_.PC++);
    case AddressingMode::ZeroPageX:
      return (read(registers_.PC++) + registers_.X) & 0xFF;
    case AddressingMode::ZeroPageY:
      return (read(registers_.PC++) + registers_.Y) & 0xFF;
    case AddressingMode::Absolute:
      return read(registers_.PC++) | (read(registers_.PC++) << 8);
    case AddressingMode::AbsoluteX:
      return (read(registers_.PC++) | (read(registers_.PC++) << 8)) +
             registers_.X;
    case AddressingMode::AbsoluteY:
      return (read(registers_.PC++) | (read(registers_.PC++) << 8)) +
             registers_.Y;
    case AddressingMode::Indirect: {
      Word ptr = read(registers_.PC++) | (read(registers_.PC++) << 8);
      return read(ptr) | (read((ptr + 1) & 0xFFFF) << 8);
    }
    default:
      throw std::runtime_error("Unimplemented addressing mode");
    }
  }

  void updateZeroAndNegativeFlags(Byte value) {
    registers_.P.Z = (value == 0);
    registers_.P.N = (value & 0x80) != 0;
  }

  std::uint64_t totalCycles = 0; // New member to keep track of total cycles
};

} // namespace m6502