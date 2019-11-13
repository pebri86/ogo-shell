#pragma once

#include <stdint.h>

enum { KEYPAD_START = 1,
       KEYPAD_SELECT = 2,
       KEYPAD_UP = 4,
       KEYPAD_DOWN = 8,
       KEYPAD_LEFT = 16,
       KEYPAD_RIGHT = 32,
       KEYPAD_A = 64,
       KEYPAD_B = 128,
       KEYPAD_MENU = 256,
       KEYPAD_L = 512,
       KEYPAD_R = 1024,
};

void keypad_init(void);
uint16_t keypad_sample(void);
uint16_t keypad_debounce(uint16_t sample, uint16_t *changes);
