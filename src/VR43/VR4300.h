//
// Created by pedrostarling2000 on 7/23/23.
//

#ifndef ESQUELETOVM_RV32I_H
#define ESQUELETOVM_RV32I_H
#ifdef _WIN32
#define _CRT_SECURE_NO_WARNINGS
#endif
#include <SDL2/SDL.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/mman.h>
#include <setjmp.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/ucontext.h>
#include <unistd.h>
typedef enum { COP0_SYS, COP1_FPU } CPU_COP_Kind;
typedef struct COP_Generic {
  void *cop;
  CPU_COP_Kind cpk;
} COP_Generic;
typedef struct VM_state {
  uint32_t *pc;
  uint64_t *gprs;
  double *fgprs;
  uint8_t *memory;
  COP_Generic *cops;
} VM_state;

void build_vm_state(VM_state **state, char *rom_name);

void begin(VM_state *state, SDL_Window *window);
#endif // ESQUELETOVM_RV32I_H
