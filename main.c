#include"stm32f4_gpio.h"
#include"stm32f4_usart.h"

#define BOOL u32
#define FALSE 0U
#define TRUE 1U

//Interrupts

/*
 * Konfiguracja przerwania zewnętrznego od pinu
 * EXTICR[x] gdzie x
 * 0 dla linii 0-3 (Rejestr EXTICR1)
 * 1 dla linii 4-7 (Rejestr EXTICR2)
 * 2 dla linii 8-11 (Rejestr EXTICR3)
 * 3 dla linii 12-15 (Rejestr EXTICR4)
*/

/* static void EXTI_lineConfig(GPIOPortE EXTI_PortSourceGPIOx, uint8_t EXTI_PinSourcex)
{
  if(RCC_APB2ENR_SYSCFGEN != (RCC->APB2ENR & RCC_APB2ENR_SYSCFGEN))
  {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  }
  uint32_t tmp = 0x00;
  tmp = ((uint32_t)0x0F) << (0x04 * (EXTI_PinSourcex & (uint8_t)0x03));

  SYSCFG->EXTICR[EXTI_PinSourcex >> 0x02] &= ~tmp;
  SYSCFG->EXTICR[EXTI_PinSourcex >> 0x02] |= (((uint32_t)EXTI_PortSourceGPIOx) <<
    (0x04 * (EXTI_PinSourcex & (uint8_t)0x03)));
}

void EXTI0_IRQHandler(void)
{
  if ((EXTI->PR & EXTI_PR_PR0) != 0)
  {
     USART_sendCharacter('a');
  }
} */

int main(void)
{
  SystemInit();
  GPIO_clock(B);
  GPIO_setupPin(GPIOB, 7, GPIO_Alternate, GPIO_Output_PP,
  GPIO_PULL_NO, GPIO_Speed_50MHz);
  GPIO_setupPin(GPIOB, 6, GPIO_Alternate, GPIO_Output_PP,
  GPIO_PULL_NO, GPIO_Speed_50MHz);
  GPIO_AFConfig(GPIOB, 6, GPIO_AF_USART1);
  GPIO_AFConfig(GPIOB, 7, GPIO_AF_USART1);
  USART_setupIrqAll(USART1, 84000000UL, 9600);
  // NVIC_EnableIRQ(EXTI0_IRQn);
  // RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  // EXTI_lineConfig(A, 0);
  // //napisz setter do EXTI; zrozum działanie EXTI dokładnie, oddziel libkę z GPIO, zrób opisy
  // EXTI->RTSR = EXTI_RTSR_TR0;
  // EXTI->FTSR = EXTI_FTSR_TR0;
  // EXTI->IMR = EXTI_IMR_MR0;
  while(1)
  {
  }
}
