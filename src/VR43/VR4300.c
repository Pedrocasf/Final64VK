//
// Created by pedrostarling2000 on 7/23/23.
//

#include "VR4300.h"
SDL_Window *window_inner = NULL;
const uint32_t KiB_SIZE = 1024;
const uint32_t MiB_SIZE = 1024 * KiB_SIZE;
const uint32_t GiB_SIZE = 1024 * MiB_SIZE;
const uint32_t MAX_ROM_SIZE = 64 * MiB_SIZE;
const uint32_t RAM_ADDR = 0;
const uint32_t RAM_LEN = 8 * MiB_SIZE;
const uint32_t ROM_ADDR = 512 * MiB_SIZE;
const uint32_t ROM_LEN = MiB_SIZE;
const uint32_t PRG_OFFSET = 0;
const uint32_t COP_COUNT = 2;
const uint32_t GPRS_COUNT = 32;
const uint32_t FGPRS_COUNT = 32;
static uint32_t rom_size = 0;
static uintptr_t io_addr = 0;
static uintptr_t cop_addr = 0;
static uintptr_t read_slot = 0;
static uint32_t write_slot = 0;
#ifdef _WIN32
#endif
typedef enum {
  R_BYTE,
  R_HALF,
  R_WORD,
  R_DWORD,
  W_BYTE,
  W_HALF,
  W_WORD,
  W_DWORD,
  NONE
} ReadWriteKind;
static ReadWriteKind RWK_slot = NONE;
void catch_sigsegv(int sig, siginfo_t *info, void *ucontext);
void build_vm_state(VM_state **state, char *rom_name) {
  // mmap VM State
  *state = (VM_state *)mmap(NULL, sizeof(VM_state),
                            PROT_READ | PROT_WRITE | PROT_EXEC,
                            MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  if (state == MAP_FAILED) {
    fprintf(stderr, "Could not map VM state \n");
    exit(errno);
  }
  // mmap General Pourpose Registers
  (*state)->gprs = (uint64_t *)mmap(NULL, sizeof(uint64_t) * GPRS_COUNT,
                                    PROT_READ | PROT_WRITE | PROT_EXEC,
                                    MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  if ((*state)->gprs == MAP_FAILED) {
    fprintf(stderr, "Could not map GPRs \n");
    exit(errno);
  }
  // mmap Floating-point General Pourpose Registers
  (*state)->fgprs = (double *)mmap(NULL, sizeof(double) * FGPRS_COUNT,
                                   PROT_READ | PROT_WRITE | PROT_EXEC,
                                   MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  if ((*state)->fgprs == MAP_FAILED) {
    fprintf(stderr, "Could not map FGPRs \n");
    exit(errno);
  }
  // mmap Control Status Registers
  (*state)->cops =
      mmap(NULL, COP_COUNT * sizeof(COP_Generic), PROT_READ | PROT_WRITE,
           MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  // Check if CSR mapping succeeded
  if ((*state)->cops == MAP_FAILED) {
    fprintf(stderr, "Could not map COPs memory\n");
    exit(errno);
  }
  // Map main memory
  (*state)->memory =
      mmap(NULL, 2 * GiB_SIZE, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);

  // Check if main memory mapping succeded
  if ((*state)->memory == MAP_FAILED) {
    fprintf(stderr, "Could not map memory\n");
    exit(errno);
  }
  cop_addr = (uintptr_t)(*state)->cops;

  // Open rom File
  FILE *rom_ptr = NULL;
  rom_ptr = fopen(rom_name, "rb");

  // Check if rom file is opened
  if (rom_ptr == NULL) {
    fprintf(stderr, "Could not open file '%s'\n", rom_name);
    exit(errno);
  }
  // Find rom size
  fseek(rom_ptr, 0L, SEEK_END);
  rom_size = ftell(rom_ptr);
  fseek(rom_ptr, 0L, SEEK_SET);
  // Check if rom size is smaller than MAX_rom_SIZE
  if (rom_size > MAX_ROM_SIZE) {
    fprintf(stderr, "rom size is too big\n");
    exit(errno);
  }
  // open rom File Descriptor
  int rom_fd = open(rom_name, O_RDONLY);

  // Check if rom file descriptor is opened
  if (rom_fd == -1) {
    fprintf(stderr, "Could not open rom file descriptor '%s'\n", rom_name);
    exit(errno);
  }
  // Map rom file to somwwhere in memory
  uint8_t *rom_mm = mmap(NULL, rom_size, PROT_READ, MAP_PRIVATE, rom_fd, 0L);
  // Check if rom mapping succeeded
  if (rom_mm == MAP_FAILED) {
    fprintf(stderr, "Could not map rom\n");
    exit(errno);
  }

  close(rom_fd);
  fclose(rom_ptr);

  return;
}
uint8_t rearrange(uint32_t instruction) {
  return ((instruction & (0x00000007 << 12)) >> 7) |
         ((instruction >> 2) & 0x1F);
}
uint8_t rd(uint32_t instruction) { return (instruction & 0x00000F80) >> 7; }
uint8_t rs1(uint32_t instruction) { return (instruction & 0x000F8000) >> 15; }
uint8_t rs2(uint32_t instruction) { return (instruction & 0x001F00000) >> 20; }
uint32_t u_imm(uint32_t instruction) { return instruction & 0xFFFFF000; }
uint32_t i_imm(uint32_t instruction) { return (((int32_t)instruction) >> 20); }
uint8_t shamt(uint32_t instruction) { return i_imm(instruction) & 0x1F; }
int32_t b_imm(uint32_t instruction) {
  return (int32_t)((int32_t)(instruction & 0x80000000) >> 19) |
         ((instruction & 0x80) << 4) | ((instruction >> 20) & 0x7e0) |
         ((instruction >> 7) & 0x1e);
}
uint32_t s_imm(uint32_t instruction) {
  return ((int32_t)(instruction & 0xfe000000) >> 20) |
         ((instruction >> 7) & 0x1f);
}
int32_t j_imm(uint32_t instruction) {
  return (int32_t)(((int32_t)(instruction & 0x80000000) >> 11) |
                   (instruction & 0xff000) | ((instruction >> 9) & 0x800) |
                   ((instruction >> 20) & 0x7fe));
}
bool funct7(uint32_t instruction) { return (instruction & 0x40000000) >> 31; }
static inline __attribute__((always_inline)) void
fetch_decode(uint32_t *pc, uint32_t *x, uint8_t *mem, uint32_t *csr);
void HALT(uint32_t *pc, uint32_t *x, uint8_t *mem, uint32_t *csr) {
  printf("full instr %032bb %08hhX\n", *pc, *pc);
  uint8_t instr = rearrange(*pc);
  printf("rearranged instr %08bb %d \n", instr, instr);
  printf("HALTED at addr  %08llX\n", ((uint8_t *)pc - mem));
  munmap(mem, 2 * GiB_SIZE);
  free(x);
  SDL_DestroyWindow(window_inner);
  SDL_Quit();
  exit(-3);
};
void catch_sigsegv(int sig, siginfo_t *info, void *ucontext) {
  ucontext_t *ctx = (ucontext_t *)ucontext;
  // printf("\n Signal %d received", sig);
  uintptr_t addr = (uintptr_t)(void *)info->si_addr;
  if (addr > csr_addr && addr < (csr_addr + (CSR_COUNT * sizeof(uint32_t)))) {
    fprintf(stderr, "Address %lX is in CSR space\n",
            (uintptr_t)((void *)info->si_addr - csr_addr));
  } else {
    // printf("\n at address %lx", addr);
    uint32_t mmio_addr = (addr - io_addr);
    // printf("\n at IO address %lx", mmio_addr);
    switch (mmio_addr >> 30) {
    case 1:
    case 3:
      switch (RWK_slot) {
      case W_BYTE:
        fprintf(stdout, "%c", (uint8_t)write_slot);
        break;
      default:
        fprintf(stdout, "%c", (uint32_t)write_slot);
      }
      break;
    case 2:
      if (RWK_slot == W_BYTE || RWK_slot == W_HALF || RWK_slot == W_WORD) {
        if (write_slot == 0x00005555) {
          SDL_DestroyWindow(window_inner);
          SDL_Quit();
          exit(0);
        }
      }
      break;
    default:
      SDL_DestroyWindow(window_inner);
      SDL_Quit();
      fprintf(stderr, "Unknown write at io address %lx\n", io_addr);
      fprintf(stderr, "Unknown write at address %lx\n", mmio_addr >> 30);
      exit(-6);
    }
  }
#if __x86_64
  (ctx->uc_mcontext.gregs[16]) += 4;
#elif __aarch64__
  (ctx->uc_mcontext.pc) += 4;
#elif __riscv
  (ctx->uc_mcontext.__gregs[1]) += 4;
#else
#error "only x86_64, arm64 and RISC-V64 supported"
#endif
}
void (*decode_table[256])(uint32_t *pc, uint32_t *a, uint8_t *mem) = {
    HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT,
    HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT,
    HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT,
    HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT,
    HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT,
    HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT,
    HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT,
    HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT,
    HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT,
    HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT,
    HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT,
    HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT,
    HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT,
    HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT,
    HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT,
    HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT,
    HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT,
    HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT,
    HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT,
    HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT,
    HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT, HALT,
    HALT, HALT, HALT, HALT};
static inline __attribute__((always_inline)) void
fetch_decode(uint32_t *pc, uint32_t *gprs, uint8_t *mem, uint32_t *cops) {
  // printf("full instr %032bb  %u", *pc, *pc);
  // printf("PC:%08llX INSTR:%08llX RE:%02llX\n", ((uint8_t *)pc - mem), *pc,
  //        rearrange(*pc));
  *gprs = 0;
  return (decode_table[rearrange(*pc)])(pc, gprs, mem, cops);
}
void begin(VM_state *state, SDL_Window *window) {
  // printf("begin vm execution");
  window_inner = window;
#ifndef _WIN32
  struct sigaction act = {0};
  act.sa_sigaction = catch_sigsegv;
  act.sa_flags = SA_SIGINFO;
  sigaction(SIGSEGV, &act, NULL);
#else

#endif
  state->pc = (uint32_t *)(state->memory + (RAM_ADDR + PRG_OFFSET));
  // printf("full instr %032bb  %08hhX\n", *state->pc, *state->pc);
  // uint8_t actual_instr = rearrange(*state->pc);
  // printf("rearranged instr %08bb\n", actual_instr);
  fetch_decode(state->pc, state->x, state->memory, state->cops);
}