#include "stdint.h"
//  максимальное число диодов в ленте (лент 16 параллельно)
#define TAPE_LENGHT 300 // число светодиодов в самой длинной ленте
#define TAPE_LINE   3   // число лент (Циплять начиная с PD0 и до PD15)

//extern volatile uint32_t sinxro;
typedef union {
  struct {
    uint8_t g, r, b, a;
  };
  uint32_t all;
} LED_t;

extern LED_t color[TAPE_LENGHT][16];

void running_RGB_init(uint8_t tape, uint32_t bright);
int running_RGB_run(uint8_t tape, uint32_t speed); // 1 -очень быстро 100 медленно 200 ещё медленнее

//void colorful_RGB_init(uint8_t tape, uint32_t bright);
int colorful_RGB_run(uint8_t tape, uint32_t speed);

