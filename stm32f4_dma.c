#include<stm32f4xx.h>
#include"stm32f4_usart.h"
#include"stm32f4_dma.h"

/*
 * setup DMA2_Stream2 for USART1 (Channel 4) RX
 * */
void DMA2_setupDMAforUSART1withCircularMode()
{
 //wait until be possible to set all parameters (DMA disabled)
 while(DMA2_Stream2->CR & DMA_SxCR_EN);

 RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
 DMA2_Stream2->CR |= (DMA_SxCR_Ch_4 << DMA_SxCR_Ch_Pos); //set Ch4
 DMA2_Stream2->CR |= DMA_SxCR_CIRC; //enable circular mode
 DMA2_Stream2->CR &= ~DMA_SxCR_DIR; //be sure that we transfer from peripherial->memory
 DMA2_Stream2->CR &= ~DMA_SxCR_MSIZE; //be sure to transfer byte
 DMA2_Stream2->CR &= ~DMA_SxCR_PSIZE; //be sure to transfer byte
 DMA2_Stream2->CR |= DMA_SxCR_MINC; //circular mode with memory increment
 DMA2_Stream2->CR |= DMA_SxCR_PL_1;
 DMA2_Stream2->CR &= ~DMA_SxCR_DIR;
 DMA2_Stream2->NDTR = CIRC_BUFF_SIZE_WORDS - 1; //no of data (bytes/h-words/words) to copy
 DMA2_Stream2->M0AR = (u32)USART_buffer; //set memory address to write
 DMA2_Stream2->PAR = (u32)&USART1->DR; //set peripherial data adress to adress of USART1
 //DMA2_Stream2->CR |= DMA_SxCR_HTIE; //half-transfer interrupt enabled
 DMA2_Stream2->CR |= DMA_SxCR_TCIE; //transfer complete interrupt enabled
 NVIC_EnableIRQ(DMA2_Stream2_IRQn);
 DMA2_Stream2->CR |= DMA_SxCR_EN;

 while(DMA2_Stream7->CR & DMA_SxCR_EN);

 DMA2_Stream7->CR |= (DMA_SxCR_Ch_4 << DMA_SxCR_Ch_Pos); //set Ch4
 DMA2_Stream7->CR |= DMA_SxCR_CIRC; //enable circular mode
 DMA2_Stream7->CR &= ~DMA_SxCR_DIR; //be sure that we transfer from peripherial->memory
 DMA2_Stream7->CR &= ~DMA_SxCR_MSIZE; //be sure to transfer byte
 DMA2_Stream7->CR &= ~DMA_SxCR_PSIZE; //be sure to transfer byte
 DMA2_Stream7->CR |= DMA_SxCR_MINC; //circular mode with memory increment
 DMA2_Stream7->CR |= DMA_SxCR_PL_1;
 DMA2_Stream7->CR |= DMA_SxCR_DIR_MP; //memory to peripherial transfer
 DMA2_Stream7->NDTR = CIRC_BUFF_SIZE_WORDS - 1; //no of data (bytes/h-words/words) to copy
 DMA2_Stream7->M0AR = (u32)USART_buffer; //set memory address to write
 DMA2_Stream7->PAR = (u32)&USART1->DR; //set peripherial data adress to adress of USART1
 //DMA2_Stream2->CR |= DMA_SxCR_HTIE; //half-transfer interrupt enabled
 DMA2_Stream7->CR |= DMA_SxCR_TCIE; //transfer complete interrupt enabled
 NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

