//
// Created by pedrostarling2000 on 7/23/23.
//

#ifndef VR4300_H
#define VR4300_H
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
  uint32_t gprs[32];
  uint32_t sys[32];
  double fpu[32];
  uint32_t rsp[32];
  int rom_fd;
  int rsp_dmem_fd;
  int rdram_fd;
  uint32_t cart_size;
  void* rsp_dmem_mem[5];
  void* cart_mem[5];
  void* rdram[5];
  void* pc;
} VM_state;

void build_vm_state(VM_state **state, char *rom_name);

void begin(VM_state *state, SDL_Window *window);
void run_instr(VM_state *state, SDL_Window *window);
#endif // VR4300_H
