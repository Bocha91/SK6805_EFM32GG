#include "sk6805.h"
#include <stdio.h>
#include <math.h>
// ������������
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
    if( ++count < speed) return 0; // ����������
    count = 0;

  do{
    color[i++][tape].all =init[k++];
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
      // ����� ������ ��������� ������� �����
      color[TAPE_LENGHT - 1][tape].g = 0xFF;//0x55;
      color[TAPE_LENGHT - 1][tape].r = 0xFF;//0x05;
      color[TAPE_LENGHT - 1][tape].b = 0xFF;//0x0A;
#endif
    return 1;
}
//-------------------------------------------------------------------
// ������� RGB
void running_RGB_init(uint8_t tape, uint32_t bright)
{
  color[0][tape].r = bright;
  color[1][tape].g = bright;
  color[2][tape].b = bright;
}
int  running_RGB_run(uint8_t tape, uint32_t speed) // speed 1 -����� ������ 100 �������� 200 ��� ���������
{
    static int count = 0;
    if( ++count < speed) return 0; // ����������
    count = 0;
    // ��������� ���� ��������� ��� �����������
    uint32_t temp[3]={ 
        color[TAPE_LENGHT-1][tape].all,
        color[TAPE_LENGHT-2][tape].all,
        color[TAPE_LENGHT-3][tape].all
    };
    // ������������ ����� �� ������� �� 3 ���������� � ������� �������� �����
    for(int a=TAPE_LENGHT-1; a>=3 ;a--){
        color[a][tape].all = color[a-3][tape].all;
    }
    // ������������� 
    color[0][tape].all = temp[0];
    color[1][tape].all = temp[1];
    color[2][tape].all = temp[2];
    return 1;
}
//-------------------------------------------------------------------
// ������������� RGB
/*
void accumulating_RGB_init(uint8_t tape, uint32_t bright)
{
  color[0][tape].all = init[bright % INIT_SIZE];
  color[1][tape].all = init[bright/2 % INIT_SIZE];
  color[2][tape].all = init[bright/3 % INIT_SIZE];
}
*/
int  accumulating_RGB_run(uint8_t tape, uint32_t speed) // speed 1 -����� ������ 100 �������� 200 ��� ���������
{
    static int count = 0;
    static uint32_t save[3]={0,0,0};
    static int clk = 0;
    static int start = 0;
    if(start == 0)
    {
      color[0][tape].all = init[(speed*1+clk) % INIT_SIZE];
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



    if( ++count < speed) return 0; // ����������
    count = 0;
    // ��������� ���� ��������� ��� �����������
    uint32_t temp[3]={ 
        color[TAPE_LENGHT-1][tape].all,
        color[TAPE_LENGHT-2][tape].all,
        color[TAPE_LENGHT-3][tape].all
    };
    // ������������ ����� �� ������� �� 3 ���������� � ������� �������� �����
    for(int a=TAPE_LENGHT-1; a>=3 ;a--){
        color[a][tape].all = color[a-3][tape].all;
    }
    // ������������� 
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
// ������������� 1 RGB


/*
void accumulating_1_RGB_init(uint8_t tape, uint32_t bright)
{
  color[LENS*0][tape].all = init[bright % INIT_SIZE];
  color[LENS*1][tape].all = init[bright/2 % INIT_SIZE];
  color[LENS*2][tape].all = init[bright/3 % INIT_SIZE];
}
*/
int  accumulating_1_RGB_run(uint8_t tape, uint32_t speed) // speed 1 -����� ������ 100 �������� 200 ��� ���������
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


    if( ++count < speed) return 0; // ����������
    count = 0;

    // ��������� ���� ��������� ��� �����������
    uint32_t temp[3]={ 
        color[LENS*1-1][tape].all,
        color[LENS*2-1][tape].all,
        color[LENS*3-1][tape].all
    };
    // ������������ ����� �� c������ �� 3 ���������� � ������� �������� �����
    for(int a=(LENS-1); a>=1 ;a--){
        color[LENS*0+a][tape].all = color[LENS*0+(a-1)][tape].all;
        color[a+LENS*1][tape].all = color[(a-1)+LENS*1][tape].all;
        color[a+LENS*2][tape].all = color[(a-1)+LENS*2][tape].all;
    }
    // ������������� 
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

int ost(int ot){ // ����� ��������� �������� �������� ��� �������
    for(int s = ot-1; s>0; s--)
    {
        if( (TAPE_LENGHT % s) ) continue;
        return s;
    }   
    return 0;
}

// ��������� ���������� ������� ���������� � ����� 1
int  accumulating_n_RGB_run(uint8_t tape, uint32_t speed) // speed 1 -����� ������ 100 �������� 200 ��� ���������
{
    static int count = 0;
    static uint32_t save[20];
    static int clk = 0;
    static int dels = sqrt(TAPE_LENGHT); // ����� ����� ����������
    static int LENS;// ���
    static int start = 0;
    if(start == 0)
    {
      // ����� �����
      for(int i=TAPE_LENGHT-1; i>=0 ;i--){
         color[i][tape].all = 0;
      }

      if(dels <= 1 ) dels = sqrt(TAPE_LENGHT);
      dels = ost((dels<20)?dels:20); // ����� ����� ����� ����������
      LENS = TAPE_LENGHT/dels; 

      for( int i=0; i<dels; i++)
      {
        color[LENS*i][tape].all = init[(speed*i+clk) % INIT_SIZE];
        save[i] = 0;      
      }
      start++;
       clk=0;
    }

    if( ++count < speed*dels*dels/3) return 0; // ����������
    count = 0;

    // ��������� ���� ��������� �����������
    uint32_t temp[20];
    for(int i=0; i<dels; i++)
    {
        temp[i] = color[LENS*(i+1)-1][tape].all;
    }

    // ������������ �����
    for( int i=0; i<dels; i++){
        for(int a=(LENS-1); a>=1 ;a--){
            color[LENS*i+a][tape].all = color[LENS*i+(a-1)][tape].all;
        }
    }
    // ������������� 
    for( int i=0; i<dels; i++){
        color[LENS*i][tape].all = temp[i];
    }

    // �������
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
//    printf("start=%d\n",start);  // �� �������� ��� ���������

    return 1;
}
//-------------------------------------------------------------------
// ��������� ���������� ������� ���������� � ���������� ����� 
int  accumulating_n2_RGB_run(uint8_t tape, uint32_t speed) // speed 1 -����� ������ 100 �������� 200 ��� ���������
{
    static int count = 0;
    static uint32_t save[20];
    static int clk = 0;
    static int dels = sqrt(TAPE_LENGHT); // ����� ����� ����������
    static int LENS;  // ������� (��� �����)
    static int n2=1;  // ��� ���������
    static int start = 0;

    if(start == 0)
    {
        // ����� �����
        for(int i=TAPE_LENGHT-1; i>=0 ;i--){
            color[i][tape].all = 0;
        }
        if(dels <= 1 ) dels = sqrt(TAPE_LENGHT);
        dels = ost((dels<20)?dels:20);
        LENS = TAPE_LENGHT/dels;
        n2 = sqrt(LENS/dels);
        for( int i=0; i<dels; i++)
        {
            color[LENS*i][tape].all = init[(speed*i+clk) % INIT_SIZE];
            save[i] = 0;      
        }
        start++;
        clk=0;
        //printf("speed=%d n2=%d dels=%d LENS=%d\n",1+speed*dels*2,n2,dels,LENS);

    }

    //if( ++count < speed*dels*dels/3 ) return 0; // ����������
    if( ++count < 1+speed*dels*2 ) return 0; // ����������
    count = 0;

    // ��������� ���� ��������� �����������
    uint32_t temp[20];
    for(int i=0; i<dels; i++)
    {
        temp[i] = color[LENS*(i+1)-1][tape].all;
    }

    // ������������ �����
    for( int i=0; i<dels; i++){
        for(int a=(LENS-1); a>=1 ;a--){
            color[LENS*i+a][tape].all = color[LENS*i+(a-1)][tape].all;
        }
    }
    // ������������� 
    for( int i=0; i<dels; i++){
        color[LENS*i][tape].all = temp[i];
    }

    // �������
    for( int i=0; i<dels; i++){
        if( temp[i] ){
            save[i] = n2;//temp[i];
        }else if(save[i] )
        {
            color[LENS*i][tape].all = init[(clk+i) % INIT_SIZE];
            --save[i];
        }
    }
    clk+=n2;
 
    if(clk > (LENS*(LENS-n2)) )
    { start = 0; }

    return 1;
}
/*
speed=75 n2=1  dels=15 LENS=20
speed=48 n2=1  dels=12 LENS=25
speed=33 n2=1  dels=10 LENS=30
speed=12 n2=2  dels=6  LENS=50
speed=8  n2=3  dels=5  LENS=60
speed=5  n2=4  dels=4  LENS=75
speed=3  n2=5  dels=3  LENS=100
speed=1  n2=8  dels=2  LENS=150
speed=0  n2=17 dels=1  LENS=300

*/
//-------------------------------------------------------------------
// ��������� ���������� ������� ���������� � ���������� ����� 


st_accumulating_n2_RGB st_accumulating_n2_RGB_init(uint8_t tape, uint32_t speed) // speed 1 -����� ������ 100 �������� 200 ��� ���������
{
    st_accumulating_n2_RGB this;
    this.tape = tape;
    this.speed = speed;
    this.count = 0;
    //save[20];
    this.clk = 0;
    this.dels = sqrt(TAPE_LENGHT); // ����� ����� ����������
    //LENS;  // ������� (��� �����)
    this.n2=1;  // ��� ���������
    this.start = 0;
    return this;
}

int  st_accumulating_n2_RGB_run(st_accumulating_n2_RGB *this ) // speed 1 -����� ������ 100 �������� 200 ��� ���������
{

    if(this->start == 0)
    {
        // ����� �����
        for(int i=TAPE_LENGHT-1; i>=0 ;i--){
            color[i][this->tape].all = 0;
        }
        if(this->dels <= 1 ) this->dels = sqrt(TAPE_LENGHT);
        this->dels = ost((this->dels<20)?this->dels:20);
        this->LENS = TAPE_LENGHT/this->dels;
        this->n2 = sqrt(this->LENS/this->dels);
        for( int i=0; i<this->dels; i++)
        {
            color[this->LENS*i][this->tape].all = init[(this->speed*i+this->clk) % INIT_SIZE];
            this->save[i] = 0;      
        }
        this->start++;
        this->clk=0;
        //printf("speed=%d n2=%d dels=%d LENS=%d\n",1+this->speed*this->dels*2,this->n2,this->dels,this->LENS);

    }

    //if( ++count < speed*dels*dels/3 ) return 0; // ����������
    if( ++this->count < 1+this->speed*this->dels*2 ) return 0; // ����������
    this->count = 0;

    // ��������� ���� ��������� �����������
    uint32_t temp[20];
    for(int i=0; i<this->dels; i++)
    {
        temp[i] = color[this->LENS*(i+1)-1][this->tape].all;
    }

    // ������������ �����
    for( int i=0; i<this->dels; i++){
        for(int a=(this->LENS-1); a>=1 ;a--){
            color[this->LENS*i+a][this->tape].all = color[this->LENS*i+(a-1)][this->tape].all;
        }
    }
    // ������������� 
    for( int i=0; i<this->dels; i++){
        color[this->LENS*i][this->tape].all = temp[i];
    }

    // �������
    for( int i=0; i<this->dels; i++){
        if( temp[i] ){
            this->save[i] = this->n2;//temp[i];
        }else if(this->save[i] )
        {
            color[this->LENS*i][this->tape].all = init[(this->clk+i) % INIT_SIZE];
            --this->save[i];
        }
    }
    this->clk+=this->n2;
 
    if(this->clk > (this->LENS*(this->LENS-this->n2)) )
    { this->start = 0; }

    return 1;
}
