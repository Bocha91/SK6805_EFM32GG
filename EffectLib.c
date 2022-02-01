#include "sk6805.h"

// бегущий RGB
void running_RGB_init(uint8_t tape, uint32_t bright)
{
  color[0][tape].r = bright;
  color[1][tape].g = bright;
  color[2][tape].b = bright;
}

int  running_RGB_run(uint8_t tape, uint32_t speed) // speed 1 -очень быстро 100 медленно 200 ещё медленнее
{
    static int count = 0;
    if( ++count < speed) return 0; // замедление
    count = 0;

    uint32_t temp[3]={ 
        color[TAPE_LENGHT-1][tape].all,
        color[TAPE_LENGHT-2][tape].all,
        color[TAPE_LENGHT-3][tape].all
    };

    for(int a=TAPE_LENGHT-1; a>=3 ;a--){
        color[a][tape].all = color[a-3][tape].all;
    }

    color[0][tape].all = temp[0];
    color[1][tape].all = temp[1];
    color[2][tape].all = temp[2];
    return 1;
}


// разноцветный
#define INIT_SIZE 12
static uint32_t init[INIT_SIZE]={ 
        0x0F0000, 0x000F00, 0x00000F ,
        0x0F0F00, 0x000F0F, 0x0F000F ,
        0x0F0703, 0x030F07, 0x07030F ,
        0x010101, 0x000101, 0x101000 
};

//void colorful_RGB_init(uint8_t tape, uint32_t bright){}
int colorful_RGB_run(uint8_t tape, uint32_t speed)
{
    static int j=0,k=0;
    int i = j;
    int Diod = 0;

    static int count = 0;
    if( ++count < speed) return 0; // замедление
    count = 0;

  do{
    color[i++][0].all =init[k++];
    if( k>=INIT_SIZE ) k=0;
  }while( i < TAPE_LENGHT );

  if( ++j >=12 ) j=0;

      if (color[0][tape].g)             --color[0][tape].g;
      else if (color[0][tape].r)        --color[0][tape].r;
      else if (color[0][tape].b)        --color[0][tape].b;
      else {
        color[0][tape].g = 0x60;
        color[0][tape].r = 0x60;
        color[0][tape].b = 0x30;
      }
      if (color[1][tape].r)             --color[1][tape].r;
      else if (color[1][tape].b)        --color[1][tape].b;
      else if (color[1][tape].g)        --color[1][tape].g;
      else {
        color[1][tape].g = 0x11;
        color[1][tape].r = 0x40;
        color[1][tape].b = 0x40;
      }

      if (color[2][tape].r)             ++color[2][tape].r;
      else if (color[2][tape].g)        ++color[2][tape].g;
      else if (color[2][tape].b)        ++color[2][tape].b;
      else {
        color[2][tape].g = 0x1;
        color[2][tape].r = 0x1;
        color[2][tape].b = 0x31;
      }
      if (color[3][tape].b)             ++color[3][tape].b;
      else if (color[3][tape].g)        ++color[3][tape].g;
      else if (color[3][tape].r)        ++color[3][tape].r;
      else {
        color[3][tape].g = 0x1;
        color[3][tape].r = 0x31;
        color[3][tape].b = 0x1;
      }
#ifdef DEBUG
      // белым светит последний элемент ленты
      color[TAPE_LENGHT - 1][tape].g = 0xFF;//0x55;
      color[TAPE_LENGHT - 1][tape].r = 0xFF;//0x05;
      color[TAPE_LENGHT - 1][tape].b = 0xFF;//0x0A;
#endif
    return 1;
}