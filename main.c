#include"stm32f4_gpio.h"
#include"stm32f4_usart.h"

#define BOOL u32
#define FALSE 0U
#define TRUE 1U

volatile char USART_buffer[CIRC_BUFF_SIZE_WORDS] = {'\0'};
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


void DMA2_Stream2_IRQHandler(void)
{
	/* transmission complete interrupt */
	if (DMA2->LISR & DMA_LISR_TCIF2)
	{
        USART_buffer[CIRC_BUFF_SIZE_WORDS - 1] = '\0';
        DMA2->LIFCR = DMA_LIFCR_CTCIF2;  // acknowledge interrupt
        USART1->CR1 |= USART_CR1_TXEIE; //set USART TX flag to send data via USART1 TX

	}
}

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
  /* DMA2 Steam 2 (for USART1 RX) & 7 (for USART1 TX) */
  while(DMA2_Stream2->CR & DMA_SxCR_EN);
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
  DMA2_Stream2->CR |= (0b100 << 25); //set Ch4
  DMA2_Stream2->CR |= DMA_SxCR_CIRC; //enable circular mode
  DMA2_Stream2->CR &= ~DMA_SxCR_DIR; //be sure that we transfer from peripherial->memory
  DMA2_Stream2->CR &= ~DMA_SxCR_MSIZE; //be sure to transfer byte
  DMA2_Stream2->CR &= ~DMA_SxCR_PSIZE; //be sure to transfer byte
  DMA2_Stream2->CR |= DMA_SxCR_MINC; //circular mode with memory increment
  DMA2_Stream2->CR |= DMA_SxCR_PL_1;
  DMA2_Stream2->NDTR = CIRC_BUFF_SIZE_WORDS - 1; //no of data (bytes/h-words/words) to copy
  DMA2_Stream2->M0AR = (u32)USART_buffer; //set memory address to write
  DMA2_Stream2->PAR = (u32)&USART1->DR; //set peripherial data adress to adress of USART1
  //DMA2_Stream2->CR |= DMA_SxCR_HTIE; //half-transfer interrupt enabled
  DMA2_Stream2->CR |= DMA_SxCR_TCIE; //transfer complete interrupt enabled
  USART1->CR3 |= USART_CR3_DMAR; //USART1 settings!
  NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  DMA2_Stream2->CR |= DMA_SxCR_EN;

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
