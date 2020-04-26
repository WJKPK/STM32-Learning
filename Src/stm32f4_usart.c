#include<stm32f4xx.h>
#include"stm32f4_usart.h"

/*
 * USART1 Interrupt handler function
 * If USART1 status register is set and interrupt is present
 * sending sting from selected buffer and clear interrupt flag
 * */

//void USART1_IRQHandler()
//{
  /* If buffer is empty and interrupt has been turn on */
//  if((USART1->SR & USART_SR_TXE) && (USART1->CR1 & USART_CR1_TXEIE))
/*  {
    USART_sendString(USART1, &USART_buffer[0]);
    USART1->CR1 &= ~USART_CR1_TXEIE;
  }
}*/

#define INT_DIGITS 4 /* enough for 8integer */

char *embeddItoa(int8_t i)
{
  /* Room for INT_DIGITS digits, - and '\0' */
  static char buf[INT_DIGITS + 2];
  char *p = buf + INT_DIGITS + 1;	/* points to terminating '\0' */
  if (i >= 0)
  {
    do
	{
      *--p = '0' + (i % 10);
      i /= 10;
    } while (i != 0);
    return p;
  }
  else
  {			/* i < 0 */
    do
	{
      *--p = '0' - (i % 10);
      i /= 10;
    } while (i != 0);
    *--p = '-';
  }
  return p;
}


static u16 USART_baudRateCalc(USART_TypeDef* USARTx, u32 APBfrequency, u32 baudRate)
 {
    u32 tmpreg = 0;
    u32 integerdivider = 0;
    u32 fractionaldivider = 0;

    /* Determine the integer part */
    if ((USARTx->CR1 & USART_CR1_OVER8) != 0)
    {
      /* Integer part computing in case Oversampling mode is 8 Samples */
      integerdivider = ((25 * APBfrequency) / (2 * baudRate));
    }
    else /* if ((USARTx->CR1 & USART_CR1_OVER8) == 0) */
    {
      /* Integer part computing in case Oversampling mode is 16 Samples */
      integerdivider = ((25 * APBfrequency) / (4 * baudRate));
    }
    tmpreg = (integerdivider / 100) << 4;

    /* Determine the fractional part */
    fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

    /* Implement the fractional part in the register */
    if ((USARTx->CR1 & USART_CR1_OVER8) != 0)
    {
      tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
    }
    else /* if ((USARTx->CR1 & USART_CR1_OVER8) == 0) */
    {
      tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);
    }
    return (u16)tmpreg;
  }

static void USART_initIrqCommon(USART_TypeDef *USARTx, u32 clockApbFreq, u32 baudRate)
{
  /* Turn on USART */
  USARTx->CR1 |= (USART_CR1_TE | USART_CR1_RE);
  USARTx->BRR = USART_baudRateCalc(USARTx, clockApbFreq, baudRate);
  USARTx->CR1 |= USART_CR1_UE;
  //USARTx->CR1 |= USART_CR1_RXNEIE; // turn on RX when DMA not supported
  USARTx->CR3 |= USART_CR3_DMAR; //support UART RX with DMA
  USARTx->CR3 |= USART_CR3_DMAT;
}

/* Function initialize USART/UART pheripherial. Default with DMA support to TX/RX data.
 * Input parameters:
 * > USARTx: pointer to USARTx x=(1..6)
 * > clockApbFreq: frequency of APB bus in Hz.
 * > baudRate in bps typical values: (1200, 2400, 4800, 9600, 19200, 38400, 57600, and 115200)
 * */

void USART_setupIrqAll(USART_TypeDef *USARTx, u32 clockApbFreq, u32 baudRate)
{
  if(USARTx == USART1)
  {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    USART_initIrqCommon(USARTx, clockApbFreq, baudRate);
    NVIC_EnableIRQ(USART1_IRQn);
  }
  else if (USARTx == USART2)
  {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART_initIrqCommon(USARTx, clockApbFreq, baudRate);
    NVIC_EnableIRQ(USART2_IRQn);
  }
  else if (USARTx == USART3)
  {
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    USART_initIrqCommon(USARTx, clockApbFreq, baudRate);
    NVIC_EnableIRQ(USART3_IRQn);
  }
  else if (USARTx == UART4)
  {
    RCC->AHB1ENR |= RCC_APB1ENR_UART4EN;
    USART_initIrqCommon(USARTx, clockApbFreq, baudRate);
    NVIC_EnableIRQ(UART4_IRQn);
  }
  else if (USARTx == UART5)
  {
    RCC->AHB1ENR |= RCC_APB1ENR_UART5EN;
    USART_initIrqCommon(USARTx, clockApbFreq, baudRate);
    NVIC_EnableIRQ(UART5_IRQn);
  }
  else if (USARTx == USART6)
  {
    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
    USART_initIrqCommon(USARTx, clockApbFreq, baudRate);
    NVIC_EnableIRQ(USART6_IRQn);
  }
  else
  {
    //Put error handling here
  }
}

/*
 * Send single character by USART
 * */

void USART_sendCharacter(USART_TypeDef *USARTx, volatile char Ascii)
{
    USART1->DR = Ascii;
      while(!(USARTx->SR & USART_SR_TXE))
  {
    ;//check if transmission is complete
  }
}

/*
 * Send string of characters by USART
 * */
void USART_sendString(USART_TypeDef *USARTx, volatile char* AsciiString)
{
  while(*AsciiString)
  {
    USART_sendCharacter(USARTx, *AsciiString);
    AsciiString++;
  }
}
