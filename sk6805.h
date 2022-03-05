#include "stdint.h"
//  ������������ ����� ������ � ����� (���� 16 �����������)
#define TAPE_LENGHT 300 // ����� ����������� � ����� ������� �����
#define TAPE_LINE   3   // ����� ���� (������� ������� � PD0 � �� PD15)

//extern volatile uint32_t sinxro;
typedef union {
  struct {
    uint8_t g, r, b, a;
  };
  uint32_t all;
} LED_t;

extern LED_t color[TAPE_LENGHT][16];

//void running_RGB_init(uint8_t tape, uint32_t bright);
int running_RGB_run(uint8_t tape, uint32_t speed); // 1 -����� ������ 100 �������� 200 ��� ���������
//void accumulating_RGB_init(uint8_t tape, uint32_t bright);
int accumulating_RGB_run(uint8_t tape, uint32_t speed); // 1 -����� ������ 100 �������� 200 ��� ���������
//void accumulating_1_RGB_init(uint8_t tape, uint32_t bright);
int accumulating_1_RGB_run(uint8_t tape, uint32_t speed); // 1 -����� ������ 100 �������� 200 ��� ���������

int accumulating_n_RGB_run(uint8_t tape, uint32_t speed); // 1 -����� ������ 100 �������� 200 ��� ���������
int accumulating_n2_RGB_run(uint8_t tape, uint32_t speed); // 1 -����� ������ 100 �������� 200 ��� ���������
//void colorful_RGB_init(uint8_t tape, uint32_t bright);
int colorful_RGB_run(uint8_t tape, uint32_t speed);

typedef struct st_accumulating_n2_RGB{
    uint8_t  tape;
    uint32_t speed;
    int count;// = 0;
    uint32_t save[20];
    int clk;// = 0;
    int dels;// = sqrt(TAPE_LENGHT); // ����� ����� ����������
    int LENS;  // ������� (��� �����)
    int n2;// =1;  // ��� ���������
    int start;// = 0;
} st_accumulating_n2_RGB; 
st_accumulating_n2_RGB st_accumulating_n2_RGB_init(uint8_t tape, uint32_t speed); // speed 1 -����� ������ 100 �������� 200 ��� ���������
int  st_accumulating_n2_RGB_run(st_accumulating_n2_RGB *this ); // speed 1 -����� ������ 100 �������� 200 ��� ���������
