#include "em_device.h"
#include "em_common.h"
#include "dmactrl.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_device.h"
#include "em_dma.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_prs.h"
#include "em_system.h"
#include "em_timer.h"
//#include "hal-config.h"

// iaeneiaeuiia ?enei aeiaia a eaioa (eaio 16 ia?aeeaeuii)  43,86,129,171
#define TAPE_LENGHT 120

int32_t count[3];
#define PWM_TABLE_SIZE (24 * TAPE_LENGHT)
#define PWM_FREQ 800000
#define LOCATION TIMER_ROUTE_LOCATION_LOC1
#define DMA_CHANNEL_TIMER3CC0 2
#define DMA_CHANNEL_TIMER3CC1 3
#define DMA_CHANNEL_TIMER3CC2 4

/* DMA callback structure */
DMA_CB_TypeDef cb[3];

uint32_t topValue;
volatile uint32_t sinxro = 0;
typedef union {
  struct {
    uint8_t g, r, b, a;
  };
  uint32_t all;
} LED_t;

LED_t color[TAPE_LENGHT][16];
LED_t color1[TAPE_LENGHT][16];

uint16_t PWMTableA[TAPE_LENGHT][24]; // = {0x5555,0x0aaaa,0x5555,0x0aaaa,0x5555,0x0aaaa,0x5555,0x0aaaa};
uint16_t PWMTableFF = 0xFFFF;
uint16_t PWMTable00 = 0x0000;

void setupTimerA(void);
void ColorToRAW(void);
void ab(int chennal, int *a, int *b) {
  if (count[chennal] > 1024) {
    *a = 1024;
    count[chennal] -= 1024;
    if (count[chennal] > 1024) {
      *b = 1024;
      count[chennal] -= 1024;
    } else {
      *b = count[chennal];
      count[chennal] = 0;
    }
  } else {
    *a = count[chennal];
    *b = 0;
    count[chennal] = -1;
  }
}
#ifdef DEBUG
uint32_t dbg[80] = {2, 0xff};
volatile uint32_t *DWT_CYCCNT = (uint32_t *)0xE0001004;
#endif

//#define WDT_CYCLE ((uint32_t*) 0xE0001004L)
/**************************************************************************/ /**
 * @brief  Call-back called when DMA transfer is complete
 *****************************************************************************/
void transferComplete(unsigned int channel, bool primary, void *user) {
  register int a;
  (void)user;
#ifdef DEBUG
  GPIO_PinOutSet(gpioPortE, 3);
#endif
  uint32_t src = 0;

  if (count[1] > 0) {
    src = (uint32_t)PWMTableA + (PWM_TABLE_SIZE - count[1]) * 2;

    if (count[1] >= 1024) {
      count[1] -= 1024;
      a = 1024;
    } else {
      a = count[1];
      count[1] = 0;
    }
    DMA_RefreshPingPong(DMA_CHANNEL_TIMER3CC0,
        primary,
        false,
        NULL,
        NULL,
        a - 1,
        false);
    DMA_RefreshPingPong(DMA_CHANNEL_TIMER3CC1,
        primary,
        false,
        NULL,
        (void *)src,
        a - 1,
        false);
    DMA_RefreshPingPong(DMA_CHANNEL_TIMER3CC2,
        primary,
        false,
        NULL,
        NULL,
        a - 1,
        false);

  } else if (count[1] == 0) {
    count[1] = -1;
    src = -1;
  } else {
    DMA_ChannelEnable(DMA_CHANNEL_TIMER3CC0, false);
    DMA_ChannelEnable(DMA_CHANNEL_TIMER3CC1, false);
    DMA_ChannelEnable(DMA_CHANNEL_TIMER3CC2, false);
    GPIO->P[gpioPortD].DOUT = 0;
    sinxro = 1;
    setupTimerA();
  }
#ifdef DEBUG
  if (dbg[0] < 80) {
    dbg[dbg[0]++] = primary ? channel : channel + 0x10;
    dbg[dbg[0]++] = src;
    dbg[dbg[0]++] = *DWT_CYCCNT - dbg[1];
  } else
    dbg[0]++;
  GPIO_PinOutClear(gpioPortE, 3);
#endif
}
/**************************************************************************/ /**
 * @brief Configure DMA for Ping-Pong transfers
 *****************************************************************************/
void setupDma(void) {
  int a, b;
  DMA_Init_TypeDef dmaInit;
  DMA_CfgChannel_TypeDef chnlCfg;
  DMA_CfgDescr_TypeDef descrCfg;

  count[0] = count[1] = count[2] = PWM_TABLE_SIZE;

  /* Initializing the DMA */
  dmaInit.hprot = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);

  //        0
  /* Setup call-back function */
  cb[0].cbFunc = transferComplete;
  cb[0].userPtr = NULL;

  /* Setting up channel */
  chnlCfg.highPri = false;
#if MULTIDMA
  chnlCfg.enableInt = true;
#else
  chnlCfg.enableInt = false;
#endif
  chnlCfg.select = DMAREQ_TIMER3_CC0;
  chnlCfg.cb = &cb[0];
  DMA_CfgChannel(DMA_CHANNEL_TIMER3CC0, &chnlCfg);

  /* Setting up channel descriptor */
  descrCfg.dstInc = dmaDataIncNone;
  descrCfg.srcInc = dmaDataIncNone;
  descrCfg.size = dmaDataSize2;
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot = 0;
  DMA_CfgDescr(DMA_CHANNEL_TIMER3CC0, true, &descrCfg);
  DMA_CfgDescr(DMA_CHANNEL_TIMER3CC0, false, &descrCfg);

  /* Enabling PingPong Transfer*/
  ab(0, &a, &b);
  if (b)
    DMA_ActivatePingPong(DMA_CHANNEL_TIMER3CC0,
        false,
        (void *)&(GPIO->P[gpioPortD].DOUT),
        (void *)&PWMTableFF,
        a - 1,
        (void *)&(GPIO->P[gpioPortD].DOUT),
        (void *)&PWMTableFF,
        b - 1);
  else
    DMA_ActivateBasic(DMA_CHANNEL_TIMER3CC0,
        true,
        false,
        (void *)&(GPIO->P[gpioPortD].DOUT),
        (void *)&PWMTableFF,
        a - 1);

  //                 1
  /* Setup call-back function */
  cb[1].cbFunc = transferComplete;
  cb[1].userPtr = NULL;

  /* Setting up channel */
  chnlCfg.highPri = false;
#if MULTIDMA
  chnlCfg.enableInt = true;
#else
  chnlCfg.enableInt = false;
#endif
  chnlCfg.select = DMAREQ_TIMER3_CC1;
  chnlCfg.cb = &cb[1];
  DMA_CfgChannel(DMA_CHANNEL_TIMER3CC1, &chnlCfg);

  /* Setting up channel descriptor */
  descrCfg.dstInc = dmaDataIncNone;
  descrCfg.srcInc = dmaDataInc2;
  descrCfg.size = dmaDataSize2;
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot = 0;
  DMA_CfgDescr(DMA_CHANNEL_TIMER3CC1, true, &descrCfg);
  DMA_CfgDescr(DMA_CHANNEL_TIMER3CC1, false, &descrCfg);

  /* Enabling PingPong Transfer*/

  ab(1, &a, &b);
  if (b)
    DMA_ActivatePingPong(DMA_CHANNEL_TIMER3CC1,
        false,
        (void *)&(GPIO->P[gpioPortD].DOUT),
        (void *)&PWMTableA,
        a - 1,
        (void *)&(GPIO->P[gpioPortD].DOUT),
        (void *)&PWMTableA + a * 2,
        b - 1);
  else
    DMA_ActivateBasic(DMA_CHANNEL_TIMER3CC1,
        true,
        false,
        (void *)&(GPIO->P[gpioPortD].DOUT),
        (void *)&PWMTableA,
        a - 1);

  //                    2
  /* Setup call-back function */
  cb[2].cbFunc = transferComplete;
  cb[2].userPtr = NULL;

  /* Setting up channel */
  chnlCfg.highPri = false;
  chnlCfg.enableInt = true;
  chnlCfg.select = DMAREQ_TIMER3_CC2; //DMAREQ_TIMER3_UFOF;
  chnlCfg.cb = &cb[2];
  DMA_CfgChannel(DMA_CHANNEL_TIMER3CC2, &chnlCfg);

  /* Setting up channel descriptor */
  descrCfg.dstInc = dmaDataIncNone;
  descrCfg.srcInc = dmaDataIncNone;
  descrCfg.size = dmaDataSize2;
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot = 0;
  DMA_CfgDescr(DMA_CHANNEL_TIMER3CC2, true, &descrCfg);
  DMA_CfgDescr(DMA_CHANNEL_TIMER3CC2, false, &descrCfg);

  ab(2, &a, &b);

  /* Enabling PingPong Transfer*/
  if (b)
    DMA_ActivatePingPong(DMA_CHANNEL_TIMER3CC2,
        false,
        (void *)&(GPIO->P[gpioPortD].DOUT),
        (void *)&PWMTable00,
        a - 1,
        (void *)&(GPIO->P[gpioPortD].DOUT),
        (void *)&PWMTable00,
        b - 1);
  else
    DMA_ActivateBasic(DMA_CHANNEL_TIMER3CC2,
        true,
        false,
        (void *)&(GPIO->P[gpioPortD].DOUT),
        (void *)&PWMTable00,
        a - 1);
}
void setupTimerA() {
  TIMER_Reset(TIMER3);

  /* Select timer parameters */
  TIMER_Init_TypeDef timerInit =
      {
          .enable = true,
          .debugRun = true,
          .prescale = timerPrescale1, //timerPrescale64,
          .clkSel = timerClkSelHFPerClk,
          .fallAction = timerInputActionNone,
          .riseAction = timerInputActionNone,
          .mode = timerModeUp,
          .dmaClrAct = true,
          .quadModeX4 = false,
          .oneShot = false,
          .sync = false,
      };
  /* Enable overflow interrupt */
  TIMER_IntEnable(TIMER3, TIMER_IF_OF);
  /* Enable TIMER0 interrupt vector in NVIC */
  NVIC_EnableIRQ(TIMER3_IRQn);
  /* Set TIMER Top value */
  TIMER_TopSet(TIMER3, topValue * 67);
  /* Configure TIMER */
  TIMER_Init(TIMER3, &timerInit);
#ifdef DEBUG
  GPIO_PinOutSet(gpioPortE, 1);
#endif
}

void setupTimerB(void) {
  TIMER_Reset(TIMER3);
  /* Setup DMA */
  setupDma();

  /* Select CC channel parameters */
  TIMER_InitCC_TypeDef timerCCInit =
      {
          .eventCtrl = timerEventEveryEdge,
          .edge = timerEdgeBoth,
          .prsSel = timerPRSSELCh0,
          .cufoa = timerOutputActionNone,
          .cofoa = timerOutputActionNone,
          .cmoa = timerOutputActionToggle,
          .mode = timerCCModePWM,
          .filter = false,
          .prsInput = false,
          .coist = false,
          .outInvert = false,
      };

  /* Configure CC channel 0,1,2 */
  TIMER_InitCC(TIMER3, 0, &timerCCInit);
  TIMER_InitCC(TIMER3, 1, &timerCCInit);
  TIMER_InitCC(TIMER3, 2, &timerCCInit);

  /* Route CC0 to location and enable pin */
  //TIMER3->ROUTE |= (TIMER_ROUTE_CC2PEN | TIMER_ROUTE_CC1PEN | TIMER_ROUTE_CC0PEN | LOCATION);
  TIMER3->ROUTE |= (TIMER_ROUTE_CC0PEN | LOCATION);

  /* Set compare value starting at first value in PWMTableA */
  TIMER_CompareSet(TIMER3, 0, 0);                // set 1
  TIMER_CompareSet(TIMER3, 1, topValue / 4 + 1); // set data
  TIMER_CompareSet(TIMER3, 2, topValue / 2 + 1); // set 0

  /* Select timer parameters */
  TIMER_Init_TypeDef timerInit =
      {
          .enable = true,
          .debugRun = true,
          .prescale = timerPrescale1, //timerPrescale64,
          .clkSel = timerClkSelHFPerClk,
          .fallAction = timerInputActionNone,
          .riseAction = timerInputActionNone,
          .mode = timerModeUp,
          .dmaClrAct = true, /* Clear DMA request when selected channel is active */
          .quadModeX4 = false,
          .oneShot = false,
          .sync = false,
      };

  TIMER_TopSet(TIMER3, topValue);

  GPIO->P[gpioPortD].DOUT = PWMTable00;
  TIMER_Init(TIMER3, &timerInit);
}

/**************************************************************************/ /**
 * @brief TIMER0_IRQHandler
 * Interrupt Service Routine TIMER0 Interrupt Line
 *****************************************************************************/
void TIMER3_IRQHandler(void) {
#ifdef DEBUG
  GPIO_PinOutClear(gpioPortE, 1);
#endif
  /* Clear flag for TIMER0 overflow interrupt */
  TIMER_IntClear(TIMER3, TIMER_IF_OF);
  /* Disable TIMER0 interrupt vector in NVIC */
  NVIC_DisableIRQ(TIMER3_IRQn);

  setupTimerB();
}


int main(void) {
  /* Initialize chip */
  CHIP_Init();

  MSC->READCTRL =
      //                  MSC_READCTRL_MODE_WS2SCBTP  // 2 oaeoa caaa??ee e ?ac?aoaiea i?aaauai?ee aey iaieo aaoaae ia?aoiaa
      MSC_READCTRL_MODE_WS2          // 2 oaeoa caaa??ee aac ?ac?aoaiea i?aaauai?ee aey iaieo aaoaae ia?aoiaa
      | MSC_READCTRL_AIDIS           // cai?ao aaoiiaoe?aneiai na?ina eaoa eiiaia i?e caiene ai oeao
                                     //                | MSC_READCTRL_PREFETCH       // ?ac?aoeou iia?a?a?uaa ?oaiea
                                     //                | MSC_READCTRL_RAMCEN         // ?ac?aoeou eaoe?iaaiea eiiaia a RAM
      | MSC_READCTRL_BUSSTRATEGY_DMA // o AIA i?ei?eoao ia?auiey e iao?eoa oei
                                     //                | MSC_READCTRL_BUSSTRATEGY_CPU  // o CPU i?ei?eoao ia?auiey e iao?eoa oei
      | MSC_READCTRL_IFCDIS          //Ioee??eou eyo eiiaia aey aioo?aiiae oeyo-iaiyoe
      ;

  //SystemHFXOClock = EFM32_HFXO_FREQ;
  //SystemLFXOClock = EFM32_LFXO_FREQ;
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

  CMU->CTRL = (CMU->CTRL & ~_CMU_CTRL_HFXOBOOST_MASK) | CMU_CTRL_HFXOBOOST_100PCENT;

  /* Enable HFXO as high frequency clock, HFCLK (depending on external oscillator this will probably be 32MHz) */
  CMU->OSCENCMD = CMU_OSCENCMD_HFXOEN;
  while (!(CMU->STATUS & CMU_STATUS_HFXORDY))
    ;
  CMU->CMD = CMU_CMD_HFCLKSEL_HFXO;

  /* LFXO setup */
  CMU->HFCORECLKDIV |= CMU_HFCORECLKDIV_HFCORECLKLEDIV; /* Enable DIV4 factor for peripheral clock */
  CMU->CTRL |= CMU_CTRL_HFLE;                           //High-Frequency LE Interface

  CMU->CTRL = (CMU->CTRL & ~_CMU_CTRL_LFXOBOOST_MASK) | CMU_CTRL_LFXOBOOST_70PCENT;
  EMU->AUXCTRL = (EMU->AUXCTRL & ~_EMU_AUXCTRL_REDLFXOBOOST_MASK) | EMU_AUXCTRL_REDLFXOBOOST;

  /* Enable LE clock and LFXO oscillator */
  CMU->HFCORECLKEN0 |= CMU_HFCORECLKEN0_LE;
  CMU->OSCENCMD |= CMU_OSCENCMD_LFXOEN;
  /* Wait until LFXO ready */
  /* Note that this could be done more energy friendly with an interrupt in EM1 */
  while (!(CMU->STATUS & CMU_STATUS_LFXORDY))
    ;

  /* Select LFXO as clock source for LFACLK */
  CMU->LFCLKSEL = (CMU->LFCLKSEL & ~_CMU_LFCLKSEL_LFA_MASK) | CMU_LFCLKSEL_LFA_LFXO;

  /* Select LFXO as clock source for LFBCLK */
  CMU->LFCLKSEL = (CMU->LFCLKSEL & ~_CMU_LFCLKSEL_LFB_MASK) | CMU_LFCLKSEL_LFB_HFCORECLKLEDIV2;

  /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);
  /* Enable clock for TIMER0 module */
  CMU_ClockEnable(cmuClock_TIMER3, true);

  GPIO_PinModeSet(gpioPortD, 0, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortD, 1, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortD, 4, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortD, 5, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortD, 6, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortD, 7, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortD, 8, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortD, 9, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortD, 10, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortD, 11, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortD, 12, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortD, 13, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortD, 14, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortD, 15, gpioModePushPull, 0);

#ifdef DEBUG
  /* Set CC0 pin (PD1) as output */
  GPIO_PinModeSet(gpioPortE, 0, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortE, 1, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortE, 2, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortE, 3, gpioModePushPull, 0);
#endif
  /* Set Top Value */
  topValue = CMU_ClockFreqGet(cmuClock_HFPER) / PWM_FREQ;

#ifdef DEBUG
  uint16_t *p = &PWMTableA[0][0];
  for (int i = 0; i < PWM_TABLE_SIZE; i += 2) {
    *(p++) = 0xaaaa;
    *(p++) = 0x5555;
  }

  dbg[1] = *DWT_CYCCNT;
#endif

  setupTimerA();
#define LINE 0
  while (1) {
    /* Go to EM1, while TIMER tuns compare output */
    EMU_EnterEM1();
    if (sinxro) {
#ifdef DEBUG
      GPIO_PinOutSet(gpioPortE, 2);
#endif
   #define INIT_SIZE 12
   uint32_t init[INIT_SIZE]={ 
        0x0F0000, 0x000F00, 0x00000F ,
        0x0F0F00, 0x000F0F, 0x0F000F ,
        0x0F0703, 0x030F07, 0x07030F ,
        0x000000, 0x000000, 0x000000 
   };

  static int j=0,k=0;
  int i = j;

  do{
    color[i++][0].all =init[k++];
    if( k>=INIT_SIZE ) k=0;
  }while( i < TAPE_LENGHT );
  if( ++j >=12 ) j=0;

      if (color[0][LINE].g)             --color[0][LINE].g;
      else if (color[0][LINE].r)        --color[0][LINE].r;
      else if (color[0][LINE].b)        --color[0][LINE].b;
      else {
        color[0][LINE].g = 0x60;
        color[0][LINE].r = 0x60;
        color[0][LINE].b = 0x30;
      }
      if (color[1][LINE].r)             --color[1][LINE].r;
      else if (color[1][LINE].b)        --color[1][LINE].b;
      else if (color[1][LINE].g)        --color[1][LINE].g;
      else {
        color[1][LINE].g = 0x11;
        color[1][LINE].r = 0x40;
        color[1][LINE].b = 0x40;
      }

      if (color[2][LINE].r)             ++color[2][LINE].r;
      else if (color[2][LINE].g)        ++color[2][LINE].g;
      else if (color[2][LINE].b)        ++color[2][LINE].b;
      else {
        color[2][LINE].g = 0x1;
        color[2][LINE].r = 0x1;
        color[2][LINE].b = 0x31;
      }
      if (color[3][LINE].b)             ++color[3][LINE].b;
      else if (color[3][LINE].g)        ++color[3][LINE].g;
      else if (color[3][LINE].r)        ++color[3][LINE].r;
      else {
        color[3][LINE].g = 0x1;
        color[3][LINE].r = 0x31;
        color[3][LINE].b = 0x1;
      }

#ifdef DEBUG
      color[TAPE_LENGHT - 1][LINE].g = 0x55;
      color[TAPE_LENGHT - 1][LINE].r = 0x05;
      color[TAPE_LENGHT - 1][LINE].b = 0x0A;
#endif
      ColorToRAW();
      sinxro = 0;
#ifdef DEBUG
      GPIO_PinOutClear(gpioPortE, 2);
#endif
    }
  }
}
void ColorToRAW(void) {
  uint32_t shi;
  int bits;
  int diod;
  for (diod = 0; diod < TAPE_LENGHT; ++diod) {
#if 0
    bool update = false;
    for (int i = 0; i < 16; i++) {
      if (color1[diod][i].all != color[diod][i].all) {
        color1[diod][i].all = color[diod][i].all;
        update = true;
      }
    }
    if (update == false) continue;
#endif
    for (shi = 0x00800000, bits = 0; bits < 24; bits++) // oeee ii aeoai aeiaa
    {
      register uint16_t col = 0x0001;
      register uint16_t PWM = 0x0000;
      for (int i = 0; i < 16; i++) {
        if (color[diod][i].all & shi) PWM |= col;
        col <<= 1;
      }
      PWMTableA[diod][bits] = PWM;
      shi >>= 1;
    }
  }
}