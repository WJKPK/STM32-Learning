#ifndef USART_HEADER
#define USART_HEADER

#define CIRC_BUFF_SIZE_WORDS 20

typedef struct circularBufferS
{
  char bufferArr[CIRC_BUFF_SIZE_WORDS];
  u16 startPhraseIdx;
  u16 currIdx;
} circularBufferS;


/* Global circular buffer used for USART interrupts */
volatile static circularBufferS rxTxQueue =
{
.bufferArr = {'0'},
.startPhraseIdx = 0,
.currIdx = 0
};

void USART_setupIrqAll(USART_TypeDef *USARTx, u32 clockApbFreq, u32 baudRate);
void writeToCircBuff(volatile circularBufferS* circBuff, char elem);
void USART_sendCharacter(volatile char Ascii);
void USART_sendString(volatile char* AsciiString);

#endif