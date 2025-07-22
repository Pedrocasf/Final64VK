//
// Created by pedrostarling2000 on 7/23/23.
//
#include <setjmp.h>
#include <sys/mman.h>
#include "VR4300.h"
#define _GNU_SOURCE
SDL_Window *window_inner = NULL;
const uint32_t KiB_SIZE = 1024;
const uint32_t MiB_SIZE = 1024 * KiB_SIZE;
const uint32_t GiB_SIZE = 1024 * MiB_SIZE;
const uint32_t RDRAM_SIZE = 8 * MiB_SIZE;
const uint32_t MAX_ROM_SIZE = 64 * MiB_SIZE;
const uint32_t RSP_DMEM_ADDR = 64 * MiB_SIZE;
const uint32_t RSP_DMEM_LEN = 4 * KiB_SIZE;
const uint32_t RSP_IMEM_ADDR = (64 * MiB_SIZE) + RSP_DMEM_LEN;
const uint32_t RSP_IMEM_LEN = 4 * KiB_SIZE;
const uint32_t HEADER_SIZE = 0x40;
const uint32_t INITIAL_PC = 0xA4000000;
const uint32_t RAM_ADDR = 0;
const uint32_t RAM_LEN = 8 * MiB_SIZE;
const uint32_t GPRS_COUNT = 32;
const uint32_t COP_COUNT = 3;
const uint32_t COP0_SYS_REG_COUNT = 32;
const uint32_t COP1_FPU_FPGPRS_COUNT = 32;
const uint32_t COP2_RCP_GPRS_COUNT = 32;
static sigjmp_buf trampoline;
static uintptr_t memory_base_addr = 0;
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
  *state = (VM_state *)calloc(1, sizeof(VM_state));

  /* Map main memory
  uint32_t *memory =
      mmap(NULL, 2 * GiB_SIZE, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);

  // Check if main memory mapping succeded
  if (memory == MAP_FAILED) {
    fprintf(stderr, "Could not map memory\n");
    exit(errno);
  }
*/
  // set adequate permissionns for RDRAM physical memory region
  int rdram_fd = memfd_create("rdram", 0);
  if (rdram_fd == -1) {
    fprintf(stderr, "Could not set create RDRAM file descriptor\n");
    exit(errno);
  }
  if (ftruncate(rdram_fd, RDRAM_SIZE) == -1){
    fprintf(stderr, "Could not set size of RDRAM file descriptor\n");
    exit(errno);
  }
  uint32_t *rdram = mmap(0, RDRAM_SIZE,
                       PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANONYMOUS, rdram_fd ,0);

  int rsp_dmem_prot = mprotect((*state)->memory + RSP_DMEM_ADDR, RSP_DMEM_LEN,
                       PROT_READ | PROT_WRITE | PROT_EXEC);

  // Check if physical memory region permissions setting succeeded
  if (rsp_dmem_prot == -1) {
    fprintf(stderr, "Could not set physical RSP DMEM memory region permissions\n");
    exit(errno);
  }
  // Open rom File
  FILE* rom_ptr = fopen(rom_name, "rb");

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
  (*state)->rom_fd = open(rom_name, O_RDONLY);
  // Check if rom file descriptor is opened
  if ((*state)->rom_fd == -1) {
    fprintf(stderr, "Could not open rom file descriptor '%s'\n", rom_name);
    exit(errno);
  }
  // Map rom file to somwwhere in host memory
  (*state)->rom =
      mmap(NULL, rom_size, PROT_READ, MAP_PRIVATE, (*state)->rom_fd, 0L);
  // Check if rom mapping succeeded
  if ((*state)->rom == MAP_FAILED) {
    fprintf(stderr, "Could not map rom\n");
    exit(errno);
  }
  // simulate the PIF ROM
  (*state)->gprs[11] = 0xFFFFFFFFA4000040;
  (*state)->gprs[20] = 0x0000000000000001;
  (*state)->gprs[22] = 0x000000000000003F;
  (*state)->gprs[29] = 0xFFFFFFFFA4001FF0;
  (*state)->sys[01] = 0x0000001F;
  (*state)->sys[12] = 0x34000000;
  (*state)->sys[15] = 0x00000B00;
  (*state)->sys[16] = 0x0006E463;
  memcpy((*state)->memory+RSP_DMEM_ADDR, (*state)->rom, RSP_DMEM_LEN * sizeof(uint8_t));
  //fclose(rom_ptr);
  return;
}
uint8_t rearrange(uint8_t instruction) {
  return (uint8_t)(instruction >>2);
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
static inline __attribute__((always_inline)) void fetch_decode(VM_state *vm);
void HALT(VM_state *vm) {
  printf("full instr %032bb %08hhX\n", *(vm->pc),*(vm->pc));
  uint8_t instr = rearrange(*(vm->pc));
  printf("rearranged instr %08bb %d \n", instr, instr);
  printf("HALTED at addr  %08llX\n", *(vm->pc) - *(vm->memory));
  munmap((vm->memory), 2 * GiB_SIZE);
  close(vm->rom_fd);
  munmap(vm->gprs, GPRS_COUNT * sizeof(uint64_t));
  SDL_DestroyWindow(window_inner);
  SDL_Quit();
  exit(-3);
};
void catch_sigsegv(int sig, siginfo_t *info, void *ucontext) {
  ucontext_t *ctx = (ucontext_t *)ucontext;
  fprintf(stderr,"\n Signal %d received", sig);
  uintptr_t addr = (uintptr_t)(void *)info->si_addr;
  SDL_DestroyWindow(window_inner);
  SDL_Quit();
  fprintf(stderr, "Unknown write at guest address %lx\n",addr - memory_base_addr);
  fprintf(stderr, "Unknown write at host address %lx\n",addr);
  exit(-6);
  siglongjmp(trampoline, true);
}
void (*decode_table[256])(VM_state *VM) = {
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
static inline __attribute__((always_inline)) uint32_t addr_translation(uint32_t addr){
  uint8_t segment = addr >> 28;
  const uint32_t addr_table[16] = {0x00000000,0x00000000,0x00000000,0x00000000,
                                  0x00000000,0x00000000,0x00000000,0x00000000, //KUSEG
                                  0x80000000,0x80000000, //KSEG0
                                  0xA0000000,0xA0000000, //KSEG1
                                  0xC0000000,0xC0000000, //KSSEG
                                  0xE0000000,0xE0000000 //KSEG3
                                  };
  fprintf(stdout, "SEGMENT %hhX\n", segment);
  return addr - addr_table[segment];
}
static inline __attribute__((always_inline)) void fetch_decode(VM_state *vm) {
  // printf("full instr %032bb  %u", *pc, *pc);
  // printf("PC:%08llX INSTR:%08llX RE:%02llX\n", ((uint8_t *)pc - mem), *pc,
  //        rearrange(*pc));
  *(vm->gprs) = 0;
  uint32_t virtual_addr = (uint32_t)(((uintptr_t)(vm->pc))-((uintptr_t)(vm->memory)));
  uint32_t physical_addr = addr_translation(virtual_addr);
  fprintf(stdout, "Physical addr:%08llX\n",physical_addr);
  uint32_t instr = (uint32_t*)(vm->memory+physical_addr);
  fprintf(stdout, "instr: %02llX", instr);
  return (decode_table[rearrange(instr)])(vm);
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
  state->pc = (uint32_t *)(state->memory + INITIAL_PC + HEADER_SIZE);

  fetch_decode(state);
}