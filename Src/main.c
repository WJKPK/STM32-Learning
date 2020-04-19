#include"FreeRTOS.h"
#include"task.h"
#include"stm32f4_gpio.h"
#include"stm32f4_usart.h"
#include"stm32f4_dma.h"


volatile char USART_buffer[CIRC_BUFF_SIZE_WORDS] = {'\0'};
//Interrupts

typedef enum
{
	risingTrigger,
	fallingTrigger,
	risingAndFallingTrigger,
	noOfTriggers,
}triggerTypeE;

/*
 * Configuration of EXTI. The register EXTICR[x] is set, where x:
 * 0 for lines 0-3 (Rejestr EXTICR1)
 * 1 for lines 4-7 (Rejestr EXTICR2)
 * 2 for lines 8-11 (Rejestr EXTICR3)
 * 3 for lines 12-15 (Rejestr EXTICR4)
 * Also rising/faling/risingAndFalling edge trigger is set.
 * Dont forget about enable proper IRQ and write IRQHandler.
*/

static void EXTI_lineConfig(GPIOPortE EXTI_PortSourceGPIOx, uint8_t EXTI_PinSourcex,
		triggerTypeE triggerType)
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

  if(triggerType == risingTrigger || triggerType == risingAndFallingTrigger) 
  {
  EXTI->RTSR |= (1UL) << EXTI_PinSourcex;
  }

  if(triggerType == fallingTrigger || triggerType == risingAndFallingTrigger)
  {
  EXTI->FTSR |= (1UL) << EXTI_PinSourcex;
  }

  EXTI->IMR |= (1UL) << EXTI_PinSourcex;
}

void EXTI0_IRQHandler(void)
{
  if ((EXTI->PR & EXTI_PR_PR0) != 0)
  {
     USART_sendString(USART1, "Interrupt working!\r\n");
	 EXTI->PR |= EXTI_PR_PR0;
  }
}


void DMA2_Stream2_IRQHandler(void)
{
  /* transmission complete interrupt */
  if (DMA2->LISR & DMA_LISR_TCIF2)
  {
    USART_buffer[CIRC_BUFF_SIZE_WORDS - 1] = '\0';
    DMA2->LIFCR = DMA_LIFCR_CTCIF2;  // acknowledge interrupt
    DMA2_Stream7->CR |= DMA_SxCR_EN; //enable DMA2_Stream7
    //USART1->CR1 |= USART_CR1_TXEIE; //set USART TX flag to send data via USART1 TX
  }
}
void DMA2_Stream7_IRQHandler(void) /* */
{
  if (DMA2->HISR & DMA_HISR_TCIF7)
  {
    DMA2->HIFCR = DMA_HIFCR_CTCIF7;  // acknowledge interrupt
	DMA2_Stream7->CR &= ~DMA_SxCR_EN; //disable DMA2_Stream7 to prevent
	//constantly sending characters by USART
  }
}

void simpleTaskFunction(void *pvParameters)
{
  while(TRUE)
  {
    USART_sendString(USART1, "1st Task in progress\r\n");
	vTaskDelay(1000/portTICK_RATE_MS);
  }
}

int main(void)
{
  SystemInit();
  /* USART PIN CONFIG */
  GPIO_clock(B);
  GPIO_setupPin(GPIOB, 7, GPIO_Alternate, GPIO_Output_PP,
  GPIO_PULL_NO, GPIO_Speed_50MHz);
  GPIO_setupPin(GPIOB, 6, GPIO_Alternate, GPIO_Output_PP,
  GPIO_PULL_NO, GPIO_Speed_50MHz);
  GPIO_AFConfig(GPIOB, 6, GPIO_AF_USART1);
  GPIO_AFConfig(GPIOB, 7, GPIO_AF_USART1);
  USART_setupIrqAll(USART1, 84000000UL, 9600);
  /* DMA2 Steam 2 (for USART1 RX) & 7 (for USART1 TX) */
  DMA2_setupDMAforUSART1withCircularMode();
  /* EXTI0 config */
  EXTI_lineConfig(A, 0, risingAndFallingTrigger);
  NVIC_EnableIRQ(EXTI0_IRQn);
  /* SPI PIN CONFIG */
  xTaskCreate(simpleTaskFunction, "simpleTaskFunction", 50, NULL, 1, NULL);
  vTaskStartScheduler();
  while(1)
  {
  }
}
