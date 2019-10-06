#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <thread>

#include "./include/SDL.h"

#include "bus.h"
#include "mcs48.h"
#include "intel8245.h"
#include "keyboard.h"

using namespace std;

int main()
{
  const int SCREEN_WIDTH = 800;
  const int SCREEN_HEIGHT = 600;
  SDL_Window *window;
  Uint32 flags = SDL_WINDOW_OPENGL;

  if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
  {
    cout << "SDL could not initialise! SDL Error: " << SDL_GetError;
    return EXIT_FAILURE;
  }

  // --------------- SETUP SDL
  // 01 Create Window
  window = SDL_CreateWindow(
      "An SDL2 window",        // window title
      SDL_WINDOWPOS_UNDEFINED, // initial x position
      SDL_WINDOWPOS_UNDEFINED, // initial y position
      SCREEN_WIDTH,            // width, in pixels
      SCREEN_HEIGHT,           // height, in pixels
      flags                    // flags - see below
  );
  // Check if window was created
  if (window == NULL)
  {
    printf("Could not create window: %s\n", SDL_GetError());
  }

  // 02 Create Renderer
  SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC);
  // 03 Create Texture
  SDL_Texture *texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, SCREEN_WIDTH, SCREEN_HEIGHT);

  // Create a buffer for the image
  Uint32 *buffer = new Uint32[SCREEN_WIDTH * SCREEN_HEIGHT];

  // Set colors
  memset(buffer, 0x00, SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(Uint32)); // fills the BG
  for (int i = 0; i < SCREEN_HEIGHT * SCREEN_WIDTH; i++)
  {
    buffer[i] = 0x2B84ABFF;
  }

  // Update the texture
  SDL_UpdateTexture(texture, NULL, buffer, SCREEN_WIDTH * sizeof(Uint32));
  // Clear the renderer
  SDL_RenderClear(renderer);
  // Copy the texture into the renderer and present it to screen
  SDL_RenderCopy(renderer, texture, NULL, NULL);
  SDL_RenderPresent(renderer);

  // --------------- GAME LOOP
  // Game loop
  bool quit = false;
  SDL_Event event;

  while (!quit)
  {
    // Update particles
    // Draw particles
    // Check for messages/event

    while (SDL_PollEvent(&event))
    {
      if (event.type == SDL_QUIT)
      {
        quit = true;
      }
    }
  }

  // --------------- DESTROYERS
  // Deconstructors
  delete[] buffer;
  SDL_DestroyRenderer(renderer);
  SDL_DestroyTexture(texture);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return EXIT_SUCCESS;
  BUS bus(4, 3, 1, 0);
  MCS48 mcs48(MCS48::CPUTYPE::CPU8048, &bus);
  INTEL8245 intel8245(&bus);
  KEYBOARD keyboard(&bus);

  ifstream biosfile;

  biosfile.open("build/roms/bios_usa_eur.bin", ios::binary | ios::in);

  uint16_t wa = 0x0000;

  for (int i = 0; i < 0x400; ++i)
  {
    uint8_t c;

    biosfile.read((char *)&c, 1);
    mcs48.writeROM(wa++, c);
  }

  biosfile.close();

  uint8_t cartridge[2048];
  ifstream cartridgefile;

  cartridgefile.open("build/roms/alpine_skiing_usa_eur.bin", ios::binary | ios::in);

  for (int i = 0; i < 0x800; ++i)
  {
    uint8_t c;

    cartridgefile.read((char *)&c, 1);
    cartridge[i] = c;
  }

  cartridgefile.close();

  bus.attachROM(cartridge, 0x400, 0x800);

  mcs48.reset(); // reset cpu

  uint8_t ic = 0;

  while (1)
  {
    cout << "\x1B[2J\x1B[H"; // clear console

    for (int i = 0; i < 2048; i++)
    {
      mcs48.clock();     // clock cpu
      intel8245.clock(); // clock gpu
      keyboard.clock();  // clock keyboard
    }

    // mcs48.disassemble();
    mcs48.debug(); // output mcs-48 debug information
    cout << endl;
    bus.debug(); // output bus debug information
    cout << endl;
    intel8245.debug();
    cout << endl;

    ++ic;

    if (ic == 0x01)
    {
      //mcs48.interrupt(); // simulate interrupt
      mcs48.setF1(1);
    }
    else
    {
      if (ic == 0x02)
      {
        mcs48.setF1(0);
        ic = 0x00;
      }
    }

    this_thread::sleep_for(chrono::milliseconds(100)); // wait for 500 ms
  }

  return 0;
}