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
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ucontext.h>
#include <unistd.h>

#include <sys/syscall.h>
#include <sys/types.h>

typedef struct VM_state {
  uint64_t gprs[32];
  uint32_t sys[32];
  double fpu[32];
  uint32_t rsp[32];
  int rom_fd;
  uint8_t *rom;
} VM_state;

void build_vm_state(VM_state **state, char *rom_name);

void begin(VM_state *state, SDL_Window *window);
#endif // ESQUELETOVM_RV32I_H
