#include "sk6805.h"
#include <stdio.h>
#include <math.h>
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
//-------------------------------------------------------------------
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
    // запоминаю цвет последних трёх светодиодов
    uint32_t temp[3]={ 
        color[TAPE_LENGHT-1][tape].all,
        color[TAPE_LENGHT-2][tape].all,
        color[TAPE_LENGHT-3][tape].all
    };
    // Переписываею ленту со здвигом на 3 светодиода в сторону дальнего конца
    for(int a=TAPE_LENGHT-1; a>=3 ;a--){
        color[a][tape].all = color[a-3][tape].all;
    }
    // закольцовываю 
    color[0][tape].all = temp[0];
    color[1][tape].all = temp[1];
    color[2][tape].all = temp[2];
    return 1;
}
//-------------------------------------------------------------------
// Накапливающий RGB
/*
void accumulating_RGB_init(uint8_t tape, uint32_t bright)
{
  color[0][tape].all = init[bright % INIT_SIZE];
  color[1][tape].all = init[bright/2 % INIT_SIZE];
  color[2][tape].all = init[bright/3 % INIT_SIZE];
}
*/
int  accumulating_RGB_run(uint8_t tape, uint32_t speed) // speed 1 -очень быстро 100 медленно 200 ещё медленнее
{
    static int count = 0;
    static uint32_t save[3]={0,0,0};
    static int clk = 0;
    static int start = 0;
    if(start == 0)
    {
      color[0][tape].all = init[(speed  +clk)  % INIT_SIZE];
      color[1][tape].all = init[(speed*2+clk) % INIT_SIZE];
      color[2][tape].all = init[(speed*3+clk) % INIT_SIZE];
      for(int a=TAPE_LENGHT-1; a>=3 ;a--){
         color[a][tape].all = 0;
      }
      save[0] = 0;
      save[1] = 0;
      save[2] = 0;
      start++;
    }



    if( ++count < speed) return 0; // замедление
    count = 0;
    // запоминаю цвет последних трёх светодиодов
    uint32_t temp[3]={ 
        color[TAPE_LENGHT-1][tape].all,
        color[TAPE_LENGHT-2][tape].all,
        color[TAPE_LENGHT-3][tape].all
    };
    // Переписываею ленту со здвигом на 3 светодиода в сторону дальнего конца
    for(int a=TAPE_LENGHT-1; a>=3 ;a--){
        color[a][tape].all = color[a-3][tape].all;
    }
    // закольцовываю 
    color[0][tape].all = temp[0];
    color[1][tape].all = temp[1];
    color[2][tape].all = temp[2];

    if( temp[0] || temp[1] || temp[2] )
    {
        save[0] = temp[0];
        save[1] = temp[1];
        save[2] = temp[2];
    }else if(save[0] || save[1] || save[2] )
    {
        color[0][tape].all = init[(clk+0) % INIT_SIZE];
        color[1][tape].all = init[(clk+1) % INIT_SIZE];
        color[2][tape].all = init[(clk+2) % INIT_SIZE];
        save[0] = 0;
        save[1] = 0;
        save[2] = 0;
    }
    clk++;
    if(clk % (TAPE_LENGHT*10) == 0)
    { start = 0; }
    return 1;
}
//-------------------------------------------------------------------
// Накапливающий 1 RGB


/*
void accumulating_1_RGB_init(uint8_t tape, uint32_t bright)
{
  color[LENS*0][tape].all = init[bright % INIT_SIZE];
  color[LENS*1][tape].all = init[bright/2 % INIT_SIZE];
  color[LENS*2][tape].all = init[bright/3 % INIT_SIZE];
}
*/
int  accumulating_1_RGB_run(uint8_t tape, uint32_t speed) // speed 1 -очень быстро 100 медленно 200 ещё медленнее
{
   
    static int count = 0;
    static uint32_t save[3];
    static int clk = 0;
    static const int DELS = 12;
    static const int LENS = (TAPE_LENGHT / 3);

    static int start = 0;
    if(start == 0)
    {
      for(int a=TAPE_LENGHT-1; a>=0 ;a--){
         color[a][tape].all = 0;
      }
      color[LENS*0][tape].all = init[(speed*0+clk) % INIT_SIZE];
      color[LENS*1][tape].all = init[(speed*2+clk) % INIT_SIZE];
      color[LENS*2][tape].all = init[(speed*3+clk) % INIT_SIZE];
      save[0] = 0;
      save[1] = 0;
      save[2] = 0;
      start++;
    }


    if( ++count < speed) return 0; // замедление
    count = 0;

    // запоминаю цвет последних трёх светодиодов
    uint32_t temp[3]={ 
        color[LENS*1-1][tape].all,
        color[LENS*2-1][tape].all,
        color[LENS*3-1][tape].all
    };
    // Переписываею ленту со здвигом на 3 светодиода в сторону дальнего конца
    for(int a=(LENS-1); a>=1 ;a--){
        color[LENS*0+a][tape].all = color[LENS*0+(a-1)][tape].all;
        color[a+LENS*1][tape].all = color[(a-1)+LENS*1][tape].all;
        color[a+LENS*2][tape].all = color[(a-1)+LENS*2][tape].all;
    }
    // закольцовываю 
    color[LENS*0][tape].all = temp[0];
    color[LENS*1][tape].all = temp[1];
    color[LENS*2][tape].all = temp[2];

    if( temp[0] ){
        save[0] = temp[0];
    }else if(save[0] )
    {
        color[LENS*0][tape].all = init[(clk+0) % INIT_SIZE];
        save[0] = 0;
    }
    if( temp[1] ){
        save[1] = temp[1];
    }else if(save[1] )
    {
        color[LENS*1][tape].all = init[(clk+1) % INIT_SIZE];
        save[1] = 0;
    }
    if( temp[2] ){
        save[2] = temp[2];
    }else if(save[2] )
    {
        color[LENS*2][tape].all = init[(clk+2) % INIT_SIZE];
        save[2] = 0;
    }
    clk++;
 
    if(clk % (TAPE_LENGHT*10) == 0)
    { start = 0; }
//    printf("start=%d\n",start);

    return 1;
}
//-------------------------------------------------------------------

int ost(int ot){ // вернёт следующее значение делителя без остатка
    for(int s = ot-1; s>0; s--)
    {
        if( (TAPE_LENGHT % s) ) continue;
        return s;
    }   
    return 0;
}

int  accumulating_n_RGB_run(uint8_t tape, uint32_t speed) // speed 1 -очень быстро 100 медленно 200 ещё медленнее
{
    static int count = 0;
    static uint32_t save[20];
    static int clk = 0;
    static int dels = sqrt(TAPE_LENGHT); // число точек зарождения
    static int LENS;// шаг

    static int start = 0;
    if(start == 0)
    {
      // гасим ленту
      for(int i=TAPE_LENGHT-1; i>=0 ;i--){
         color[i][tape].all = 0;
      }

      if(dels <= 1 ) dels = sqrt(TAPE_LENGHT);
      dels = ost((dels<20)?dels:20);
      LENS = TAPE_LENGHT/dels; 

      for( int i=0; i<dels; i++)
      {
        color[LENS*i][tape].all = init[(speed*i+clk) % INIT_SIZE];
        save[i] = 0;      
      }
      start++;
       clk=0;
    }

    if( ++count < speed*dels*dels/3) return 0; // замедление
    count = 0;

    // запоминаю цвет последних светодиодов
    uint32_t temp[20];
    for(int i=0; i<dels; i++)
    {
        temp[i] = color[LENS*(i+1)-1][tape].all;
    }

    // Переписываею ленту
    for( int i=0; i<dels; i++){
        for(int a=(LENS-1); a>=1 ;a--){
            color[LENS*i+a][tape].all = color[LENS*i+(a-1)][tape].all;
        }
    }
    // закольцовываю 
    for( int i=0; i<dels; i++){
        color[LENS*i][tape].all = temp[i];
    }

    // удлиняю
    for( int i=0; i<dels; i++){
        if( temp[i] ){
            save[i] = temp[i];
        }else if(save[i] )
        {
            color[LENS*i][tape].all = init[(clk+i) % INIT_SIZE];
            save[i] = 0;
        }
    }
    clk++;
 
    if(clk > (LENS*(LENS-1)) )
    { start = 0; }
//    printf("start=%d\n",start);

    return 1;
}
//-------------------------------------------------------------------