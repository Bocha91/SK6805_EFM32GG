/*****************************************************************************
 *                   SEGGER Microcontroller GmbH & Co. KG                    *
 *            Solutions for real time microcontroller applications           *
 *****************************************************************************
 *                                                                           *
 *               (c) 2017 SEGGER Microcontroller GmbH & Co. KG               *
 *                                                                           *
 *           Internet: www.segger.com   Support: support@segger.com          *
 *                                                                           *
 *****************************************************************************/

/*****************************************************************************
 *                         Preprocessor Definitions                          *
 *                         ------------------------                          *
 * VECTORS_IN_RAM                                                            *
 *                                                                           *
 *   If defined, an area of RAM will large enough to store the vector table  *
 *   will be reserved.                                                       *
 *                                                                           *
 *****************************************************************************/

  .syntax unified
  .code 16

  .section .init, "ax"
  .align 0

/*****************************************************************************
 * Default Exception Handlers                                                *
 *****************************************************************************/

  .thumb_func
  .weak NMI_Handler
NMI_Handler:
  b .

  .thumb_func
  .weak HardFault_Handler
HardFault_Handler:
  b .

  .thumb_func
  .weak SVC_Handler
SVC_Handler:
  b .

  .thumb_func
  .weak PendSV_Handler
PendSV_Handler:
  b .

  .thumb_func
  .weak SysTick_Handler
SysTick_Handler:
  b .

  .thumb_func
Dummy_Handler:
  b .

#if defined(__OPTIMIZATION_SMALL)

  .weak DMA_IRQHandler
  .thumb_set DMA_IRQHandler,Dummy_Handler

  .weak GPIO_EVEN_IRQHandler
  .thumb_set GPIO_EVEN_IRQHandler,Dummy_Handler

  .weak TIMER0_IRQHandler
  .thumb_set TIMER0_IRQHandler,Dummy_Handler

  .weak USART0_RX_IRQHandler
  .thumb_set USART0_RX_IRQHandler,Dummy_Handler

  .weak USART0_TX_IRQHandler
  .thumb_set USART0_TX_IRQHandler,Dummy_Handler

  .weak USB_IRQHandler
  .thumb_set USB_IRQHandler,Dummy_Handler

  .weak ACMP0_IRQHandler
  .thumb_set ACMP0_IRQHandler,Dummy_Handler

  .weak ADC0_IRQHandler
  .thumb_set ADC0_IRQHandler,Dummy_Handler

  .weak DAC0_IRQHandler
  .thumb_set DAC0_IRQHandler,Dummy_Handler

  .weak I2C0_IRQHandler
  .thumb_set I2C0_IRQHandler,Dummy_Handler

  .weak I2C1_IRQHandler
  .thumb_set I2C1_IRQHandler,Dummy_Handler

  .weak GPIO_ODD_IRQHandler
  .thumb_set GPIO_ODD_IRQHandler,Dummy_Handler

  .weak TIMER1_IRQHandler
  .thumb_set TIMER1_IRQHandler,Dummy_Handler

  .weak TIMER2_IRQHandler
  .thumb_set TIMER2_IRQHandler,Dummy_Handler

  .weak TIMER3_IRQHandler
  .thumb_set TIMER3_IRQHandler,Dummy_Handler

  .weak USART1_RX_IRQHandler
  .thumb_set USART1_RX_IRQHandler,Dummy_Handler

  .weak USART1_TX_IRQHandler
  .thumb_set USART1_TX_IRQHandler,Dummy_Handler

  .weak LESENSE_IRQHandler
  .thumb_set LESENSE_IRQHandler,Dummy_Handler

  .weak USART2_RX_IRQHandler
  .thumb_set USART2_RX_IRQHandler,Dummy_Handler

  .weak USART2_TX_IRQHandler
  .thumb_set USART2_TX_IRQHandler,Dummy_Handler

  .weak UART0_RX_IRQHandler
  .thumb_set UART0_RX_IRQHandler,Dummy_Handler

  .weak UART0_TX_IRQHandler
  .thumb_set UART0_TX_IRQHandler,Dummy_Handler

  .weak UART1_RX_IRQHandler
  .thumb_set UART1_RX_IRQHandler,Dummy_Handler

  .weak UART1_TX_IRQHandler
  .thumb_set UART1_TX_IRQHandler,Dummy_Handler

  .weak LEUART0_IRQHandler
  .thumb_set LEUART0_IRQHandler,Dummy_Handler

  .weak LEUART1_IRQHandler
  .thumb_set LEUART1_IRQHandler,Dummy_Handler

  .weak LETIMER0_IRQHandler
  .thumb_set LETIMER0_IRQHandler,Dummy_Handler

  .weak PCNT0_IRQHandler
  .thumb_set PCNT0_IRQHandler,Dummy_Handler

  .weak PCNT1_IRQHandler
  .thumb_set PCNT1_IRQHandler,Dummy_Handler

  .weak PCNT2_IRQHandler
  .thumb_set PCNT2_IRQHandler,Dummy_Handler

  .weak RTC_IRQHandler
  .thumb_set RTC_IRQHandler,Dummy_Handler

  .weak BURTC_IRQHandler
  .thumb_set BURTC_IRQHandler,Dummy_Handler

  .weak CMU_IRQHandler
  .thumb_set CMU_IRQHandler,Dummy_Handler

  .weak VCMP_IRQHandler
  .thumb_set VCMP_IRQHandler,Dummy_Handler

  .weak LCD_IRQHandler
  .thumb_set LCD_IRQHandler,Dummy_Handler

  .weak MSC_IRQHandler
  .thumb_set MSC_IRQHandler,Dummy_Handler

  .weak AES_IRQHandler
  .thumb_set AES_IRQHandler,Dummy_Handler

  .weak EBI_IRQHandler
  .thumb_set EBI_IRQHandler,Dummy_Handler

  .weak EMU_IRQHandler
  .thumb_set EMU_IRQHandler,Dummy_Handler

#else

  .thumb_func
  .weak DMA_IRQHandler
DMA_IRQHandler:
  b .

  .thumb_func
  .weak GPIO_EVEN_IRQHandler
GPIO_EVEN_IRQHandler:
  b .

  .thumb_func
  .weak TIMER0_IRQHandler
TIMER0_IRQHandler:
  b .

  .thumb_func
  .weak USART0_RX_IRQHandler
USART0_RX_IRQHandler:
  b .

  .thumb_func
  .weak USART0_TX_IRQHandler
USART0_TX_IRQHandler:
  b .

  .thumb_func
  .weak USB_IRQHandler
USB_IRQHandler:
  b .

  .thumb_func
  .weak ACMP0_IRQHandler
ACMP0_IRQHandler:
  b .

  .thumb_func
  .weak ADC0_IRQHandler
ADC0_IRQHandler:
  b .

  .thumb_func
  .weak DAC0_IRQHandler
DAC0_IRQHandler:
  b .

  .thumb_func
  .weak I2C0_IRQHandler
I2C0_IRQHandler:
  b .

  .thumb_func
  .weak I2C1_IRQHandler
I2C1_IRQHandler:
  b .

  .thumb_func
  .weak GPIO_ODD_IRQHandler
GPIO_ODD_IRQHandler:
  b .

  .thumb_func
  .weak TIMER1_IRQHandler
TIMER1_IRQHandler:
  b .

  .thumb_func
  .weak TIMER2_IRQHandler
TIMER2_IRQHandler:
  b .

  .thumb_func
  .weak TIMER3_IRQHandler
TIMER3_IRQHandler:
  b .

  .thumb_func
  .weak USART1_RX_IRQHandler
USART1_RX_IRQHandler:
  b .

  .thumb_func
  .weak USART1_TX_IRQHandler
USART1_TX_IRQHandler:
  b .

  .thumb_func
  .weak LESENSE_IRQHandler
LESENSE_IRQHandler:
  b .

  .thumb_func
  .weak USART2_RX_IRQHandler
USART2_RX_IRQHandler:
  b .

  .thumb_func
  .weak USART2_TX_IRQHandler
USART2_TX_IRQHandler:
  b .

  .thumb_func
  .weak UART0_RX_IRQHandler
UART0_RX_IRQHandler:
  b .

  .thumb_func
  .weak UART0_TX_IRQHandler
UART0_TX_IRQHandler:
  b .

  .thumb_func
  .weak UART1_RX_IRQHandler
UART1_RX_IRQHandler:
  b .

  .thumb_func
  .weak UART1_TX_IRQHandler
UART1_TX_IRQHandler:
  b .

  .thumb_func
  .weak LEUART0_IRQHandler
LEUART0_IRQHandler:
  b .

  .thumb_func
  .weak LEUART1_IRQHandler
LEUART1_IRQHandler:
  b .

  .thumb_func
  .weak LETIMER0_IRQHandler
LETIMER0_IRQHandler:
  b .

  .thumb_func
  .weak PCNT0_IRQHandler
PCNT0_IRQHandler:
  b .

  .thumb_func
  .weak PCNT1_IRQHandler
PCNT1_IRQHandler:
  b .

  .thumb_func
  .weak PCNT2_IRQHandler
PCNT2_IRQHandler:
  b .

  .thumb_func
  .weak RTC_IRQHandler
RTC_IRQHandler:
  b .

  .thumb_func
  .weak BURTC_IRQHandler
BURTC_IRQHandler:
  b .

  .thumb_func
  .weak CMU_IRQHandler
CMU_IRQHandler:
  b .

  .thumb_func
  .weak VCMP_IRQHandler
VCMP_IRQHandler:
  b .

  .thumb_func
  .weak LCD_IRQHandler
LCD_IRQHandler:
  b .

  .thumb_func
  .weak MSC_IRQHandler
MSC_IRQHandler:
  b .

  .thumb_func
  .weak AES_IRQHandler
AES_IRQHandler:
  b .

  .thumb_func
  .weak EBI_IRQHandler
EBI_IRQHandler:
  b .

  .thumb_func
  .weak EMU_IRQHandler
EMU_IRQHandler:
  b .

#endif

/*****************************************************************************
 * Vector Table                                                              *
 *****************************************************************************/

  .section .vectors, "ax"
  .align 0
  .global _vectors
  .extern __stack_end__
  .extern Reset_Handler

_vectors:
  .word __stack_end__
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word 0 /* Reserved */
  .word 0 /* Reserved */
  .word 0 /* Reserved */
  .word 0 /* Reserved */
  .word 0 /* Reserved */
  .word 0 /* Reserved */
  .word 0 /* Reserved */
  .word SVC_Handler
  .word 0 /* Reserved */
  .word 0 /* Reserved */
  .word PendSV_Handler
  .word SysTick_Handler
  .word DMA_IRQHandler
  .word GPIO_EVEN_IRQHandler
  .word TIMER0_IRQHandler
  .word USART0_RX_IRQHandler
  .word USART0_TX_IRQHandler
  .word USB_IRQHandler
  .word ACMP0_IRQHandler
  .word ADC0_IRQHandler
  .word DAC0_IRQHandler
  .word I2C0_IRQHandler
  .word I2C1_IRQHandler
  .word GPIO_ODD_IRQHandler
  .word TIMER1_IRQHandler
  .word TIMER2_IRQHandler
  .word TIMER3_IRQHandler
  .word USART1_RX_IRQHandler
  .word USART1_TX_IRQHandler
  .word LESENSE_IRQHandler
  .word USART2_RX_IRQHandler
  .word USART2_TX_IRQHandler
  .word UART0_RX_IRQHandler
  .word UART0_TX_IRQHandler
  .word UART1_RX_IRQHandler
  .word UART1_TX_IRQHandler
  .word LEUART0_IRQHandler
  .word LEUART1_IRQHandler
  .word LETIMER0_IRQHandler
  .word PCNT0_IRQHandler
  .word PCNT1_IRQHandler
  .word PCNT2_IRQHandler
  .word RTC_IRQHandler
  .word BURTC_IRQHandler
  .word CMU_IRQHandler
  .word VCMP_IRQHandler
  .word LCD_IRQHandler
  .word MSC_IRQHandler
  .word AES_IRQHandler
  .word EBI_IRQHandler
  .word EMU_IRQHandler
_vectors_end:

#ifdef VECTORS_IN_RAM
  .section .vectors_ram, "ax"
  .align 0
  .global _vectors_ram

_vectors_ram:
  .space _vectors_end - _vectors, 0
#endif
