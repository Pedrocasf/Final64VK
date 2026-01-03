#include "VR43/VR4300.h"
#include <SDL2/SDL.h>
#include <stdbool.h>
#include <stdio.h>
const int SCREEN_WIDTH = 320;
const int SCREEN_HEIGHT = 320;
SDL_Window *window_outer = NULL;
int main(int argc, char *argv[]) {

  if (SDL_Init(SDL_INIT_VIDEO) == 0)
    window_outer = SDL_CreateWindow("Tutorial SDL 2", 50, 50, SCREEN_WIDTH,
                                    SCREEN_HEIGHT, 0);
  else {
    SDL_Log("Erro na inicialização: %s", SDL_GetError());
    exit(-7);
  }

  if (argc < 2) {
    fprintf(stderr, "no executable file provided");
    exit(-1);
  }
  // printf("Executable path: %s\n", argv[argc - 1]);
  VM_state *VR4300_State = NULL;
  build_vm_state(&VR4300_State, argv[argc - 1]);
  // printf("returned from state init\n");
  if (VR4300_State == NULL) {
    printf("Couldn't initialize vm state\n");
    exit(-2);
  }
  if (window_outer != NULL) {

      while (!SDL_QuitRequested()) {
          run_instr(VR4300_State, window_outer);
      }
    SDL_DestroyWindow(window_outer);
    SDL_Quit();
  } else {
    SDL_Log("Erro na criação da janela: %s", SDL_GetError());
    exit(-8);
  }
  return 0;
}
