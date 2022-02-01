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

void running_RGB_init(uint8_t tape, uint32_t bright);
int running_RGB_run(uint8_t tape, uint32_t speed); // 1 -����� ������ 100 �������� 200 ��� ���������

//void colorful_RGB_init(uint8_t tape, uint32_t bright);
int colorful_RGB_run(uint8_t tape, uint32_t speed);

