#ifndef USART_HEADER
#define USART_HEADER

#define CIRC_BUFF_SIZE_WORDS 3

extern volatile char USART_buffer[CIRC_BUFF_SIZE_WORDS];

void USART_setupIrqAll(USART_TypeDef *USARTx, u32 clockApbFreq, u32 baudRate);
void USART_sendCharacter(USART_TypeDef *USARTx, volatile char Ascii);
void USART_sendString(USART_TypeDef *USARTx, volatile char* AsciiString);

#endif 
