#include"FreeRTOS.h"
#include"task.h"
#include"stm32f4_gpio.h"
#include"stm32f4_usart.h"
#include"stm32f4_dma.h"
#include"lis302dl.h"

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

/* Read/Write command */
#define READWRITE_CMD              ((uint16_t)0x8000)
/* Multiple byte read/write command */
#define MULTIPLEBYTE_CMD           ((uint16_t)0x4000)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                 ((uint16_t)0x0000)

int8_t SPI_sendFrame(SPI_TypeDef* SPIx, uint8_t frame)
{
  GPIO_writeBit(GPIOE, 3, FALSE);
  SPIx->DR = frame;
  while(!(SPIx->SR & SPI_SR_TXE));
  while(!(SPIx->SR & SPI_SR_RXNE));
  GPIO_writeBit(GPIOE, 3, TRUE);
  return SPIx->DR;
  }

int8_t SPI_readRegister(SPI_TypeDef* SPIx, uint8_t adress)
{
  GPIO_writeBit(GPIOE, 3, FALSE);
  // bit 15 is 1 for read for lis302dl
  uint16_t frame = 0;
  frame |= READWRITE_CMD;
  frame |= (uint16_t)(adress << 8);
  // Send data
  SPI1->DR = frame;
  // wait until tx buf is empty (TXE flag)
  while (!(SPI1->SR & SPI_SR_TXE));
  // wait until rx buf is not empty (RXNE flag)
  while (!(SPI1->SR & SPI_SR_RXNE));

  GPIO_writeBit(GPIOE, 3, TRUE);
  return (int8_t)SPI1->DR;
}

void SPI_writeRegister(SPI_TypeDef* SPIx, uint8_t adress, uint8_t data)
{
  GPIO_writeBit(GPIOE, 3, FALSE);
  // bit 15 is 0 for write for lis302dl
  uint32_t frame = 0;
  frame |= data;
  frame |= (uint16_t)(adress << 8);
  // Send data
  SPI1->DR = frame;
  // wait until transmit is done (TXE flag)
  while (!(SPI1->SR & SPI_SR_TXE));
  // wait until rx buf is not empty (RXNE flag)
  while (!(SPI1->SR & SPI_SR_RXNE));

  GPIO_writeBit(GPIOE, 3, TRUE);
  (void)SPI1->DR; // dummy read
}

void readDataFromAccelerometer(void *pvParameters)
{
  while(TRUE)
  {
	USART_sendString(USART1, embeddItoa(SPI_readRegister(SPI1, LIS302_REG_OUT_X)));
	USART_sendString(USART1, "\t");
	USART_sendString(USART1, embeddItoa(SPI_readRegister(SPI1, LIS302_REG_OUT_Y)));
    USART_sendString(USART1, "\t");
	USART_sendString(USART1, embeddItoa(SPI_readRegister(SPI1, LIS302_REG_OUT_Y)));
	USART_sendString(USART1, "\r\n");

	vTaskDelay(100/portTICK_RATE_MS);
  }
}

int main(void)
{
  SystemInit();
  GPIO_clock(B);
  GPIO_clock(A);
  GPIO_clock(E);

 /* EXTI and USART1 pins setings */
  GPIO_setupPin(GPIOA, 0, GPIO_Input, GPIO_Output_PP,
  GPIO_PULL_NO, GPIO_Speed_50MHz);
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
  EXTI_lineConfig(A, 0, fallingTrigger);
  NVIC_EnableIRQ(EXTI0_IRQn);
  NVIC_SetPriority(EXTI0_IRQn, 3);

  /* SPI pinout setup */
  GPIO_setupPin(GPIOA, 6, GPIO_Alternate, GPIO_Output_PP,
  GPIO_PULL_NO, GPIO_Speed_50MHz);
  GPIO_setupPin(GPIOA, 7, GPIO_Alternate, GPIO_Output_PP,
  GPIO_PULL_NO, GPIO_Speed_50MHz);
  GPIO_setupPin(GPIOE, 3, GPIO_Output, GPIO_Output_PP,
  GPIO_PULL_NO, GPIO_Speed_50MHz);
  GPIO_setupPin(GPIOA, 5, GPIO_Alternate, GPIO_Output_PP,
  GPIO_PULL_NO, GPIO_Speed_50MHz);
  GPIO_writeBit(GPIOE, 3, TRUE);
  GPIO_AFConfig(GPIOA, 5, GPIO_AF_SPI1);
  GPIO_AFConfig(GPIOA, 6, GPIO_AF_SPI1);
  GPIO_AFConfig(GPIOA, 7, GPIO_AF_SPI1);

  /* SPI PIN CONFIG */
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  SPI1->CR1 = 0;
  SPI1->CR1 &= ~SPI_CR1_CPHA; //frist clock transition is the first data capture
  SPI1->CR1 &= ~SPI_CR1_CPOL; //set to 1 when idle
  SPI1->CR1 |= SPI_CR1_DFF;
  SPI1->CR1 |= SPI_CR1_MSTR; //master config
  SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_1 | SPI_CR1_BR_0;
  SPI1->CR1 |= SPI_CR1_SPE;
  SPI_writeRegister(SPI1, 0x21, 0x40);
  SPI_writeRegister(SPI1, 0x20, 0x47);

  xTaskCreate(readDataFromAccelerometer, "readDataFromLIS302LD", 50, NULL, 1, NULL);
  vTaskStartScheduler();
  while(1)
  {
  }
}
