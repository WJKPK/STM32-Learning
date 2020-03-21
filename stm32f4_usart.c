#include<stm32f4xx.h>
#include"stm32f4_usart.h"

void USART1_IRQHandler()
{
  if(USART1->SR & USART_SR_RXNE)
  {
    char data = USART1->DR;
    writeToCircBuff(&rxTxQueue, data);
  }
  /* If buffer is empty and interrupt has been turn on */
  if((USART1->SR & USART_SR_TXE) && (USART1->CR1 & USART_CR1_TXEIE))
  {
    USART_sendString(rxTxQueue.bufferArr);
    USART1->CR1 &= ~USART_CR1_TXEIE;
  }
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
  /* Set RX interrupt ON */
  USARTx->CR1 |= USART_CR1_RXNEIE;
}

/*
 *
 *
 *
*/

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

void writeToCircBuff(volatile circularBufferS* circBuff, char elem)
{
  circBuff->bufferArr[circBuff->currIdx] = elem;
  circBuff->currIdx++;
  if(circBuff->currIdx > CIRC_BUFF_SIZE_WORDS - 2)
  {
    circBuff->bufferArr[CIRC_BUFF_SIZE_WORDS - 1] = '\0';
    circBuff->currIdx = circBuff->startPhraseIdx;
    /* If we reach end of buffer - TURN ON TXE INTERRUPT */
    USART1->CR1 |= USART_CR1_TXEIE;
  }
}

void USART_sendCharacter(volatile char Ascii)
{
    USART1->DR = Ascii;
      while(!(USART1->SR & USART_SR_TXE))
  {
    ;//check if transmission is complete
  }
}

void USART_sendString(volatile char* AsciiString)
{
  while(*AsciiString)
  {
    USART_sendCharacter(*AsciiString);
    AsciiString++;
  }
}
