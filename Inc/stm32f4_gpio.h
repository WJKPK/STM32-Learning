#ifndef GPIO_HEADER
#define GPIO_HEADER

#include<stdlib.h>
#include<stm32f4xx.h>

#define GPIO_Pin u32
#define BIT_LEN_u16_REGISTER 16

typedef enum {
  A = 0,
  B, C, D, E, F, G, H, I /* */
} GPIOPortE;

typedef enum
{
  GPIO_Speed_2MHz   = 0x00, /*!< Low speed */
  GPIO_Speed_25MHz  = 0x01, /*!< Medium speed */
  GPIO_Speed_50MHz  = 0x02, /*!< Fast speed */
  GPIO_Speed_100MHz = 0x03  /*!< High speed on 30 pF (80 MHz Output max speed on 15 pF) */
}GPIOSpeedE;

typedef enum
{
  GPIO_PULL_NO = 0x00, /* */
  GPIO_PULL_UP = 0x01, /* */
  GPIO_PULL_DOWN = 0x02 /* */
}GPIOPullE;

typedef enum
{
  GPIO_Output_PP = 0x00, /* */
  GPIO_Output_ODrain = 0x01 /* */
}GPIOOutputDriveE;

typedef enum
{
  GPIO_Input   = 0x00, /*!< GPIO Input Mode */
  GPIO_Output  = 0x01, /*!< GPIO Output Mode */
  GPIO_Alternate   = 0x02, /*!< GPIO Alternate function Mode */
  GPIO_Analog   = 0x03  /*!< GPIO Analog Mode */
}GPIOModeE;

/* GPIO_Alternat_function_selection_define */

#define GPIO_AF_RTC_50Hz      ((u32)0x00)  /* RTC_50Hz Alternate Function mapping */
#define GPIO_AF_MCO           ((u32)0x00)  /* MCO (MCO1 and MCO2) Alternate Function mapping */
#define GPIO_AF_TAMPER        ((u32)0x00)  /* TAMPER (TAMPER_1 and TAMPER_2) Alternate Function mapping */
#define GPIO_AF_SWJ           ((u32)0x00)  /* SWJ (SWD and JTAG) Alternate Function mapping */
#define GPIO_AF_TRACE         ((u32)0x00)  /* TRACE Alternate Function mapping */

/* AF 1 selection */
#define GPIO_AF_TIM1          ((u32)0x01)  /* TIM1 Alternate Function mapping */
#define GPIO_AF_TIM2          ((u32)0x01)  /* TIM2 Alternate Function mapping */

/* AF 2 selection */
#define GPIO_AF_TIM3          ((u32)0x02)  /* TIM3 Alternate Function mapping */
#define GPIO_AF_TIM4          ((u32)0x02)  /* TIM4 Alternate Function mapping */
#define GPIO_AF_TIM5          ((u32)0x02)  /* TIM5 Alternate Function mapping */

/* AF 3 selection */
#define GPIO_AF_TIM8          ((u32)0x03)  /* TIM8 Alternate Function mapping */
#define GPIO_AF_TIM9          ((u32)0x03)  /* TIM9 Alternate Function mapping */
#define GPIO_AF_TIM10         ((u32)0x03)  /* TIM10 Alternate Function mapping */
#define GPIO_AF_TIM11         ((u32)0x03)  /* TIM11 Alternate Function mapping */

/* AF 4 selection */
#define GPIO_AF_I2C1          ((u32)0x04)  /* I2C1 Alternate Function mapping */
#define GPIO_AF_I2C2          ((u32)0x04)  /* I2C2 Alternate Function mapping */
#define GPIO_AF_I2C3          ((u32)0x04)  /* I2C3 Alternate Function mapping */

/* AF 5 selection */
#define GPIO_AF_SPI1          ((u32)0x05)  /* SPI1 Alternate Function mapping */
#define GPIO_AF_SPI2          ((u32)0x05)  /* SPI2/I2S2 Alternate Function mapping */

/* AF 6 selection */
#define GPIO_AF_SPI3          ((u32)0x06)  /* SPI3/I2S3 Alternate Function mapping */

/* AF 7 selection */
#define GPIO_AF_USART1        ((u32)0x07)  /* USART1 Alternate Function mapping */
#define GPIO_AF_USART2        ((u32)0x07)  /* USART2 Alternate Function mapping */
#define GPIO_AF_USART3        ((u32)0x07)  /* USART3 Alternate Function mapping */
#define GPIO_AF_I2S3ext       ((u32)0x07)  /* I2S3ext Alternate Function mapping */

/* AF 8 selection */
#define GPIO_AF_UART4         ((u32)0x08)  /* UART4 Alternate Function mapping */
#define GPIO_AF_UART5         ((u32)0x08)  /* UART5 Alternate Function mapping */
#define GPIO_AF_USART6        ((u32)0x08)  /* USART6 Alternate Function mapping */

/* AF 9 selection */
#define GPIO_AF_CAN1          ((u32)0x09)  /* CAN1 Alternate Function mapping */
#define GPIO_AF_CAN2          ((u32)0x09)  /* CAN2 Alternate Function mapping */
#define GPIO_AF_TIM12         ((u32)0x09)  /* TIM12 Alternate Function mapping */
#define GPIO_AF_TIM13         ((u32)0x09)  /* TIM13 Alternate Function mapping */
#define GPIO_AF_TIM14         ((u32)0x09)  /* TIM14 Alternate Function mapping */

/* AF 10 selection */
#define GPIO_AF_OTG_FS         ((u32)0xA)  /* OTG_FS Alternate Function mapping */
#define GPIO_AF_OTG_HS         ((u32)0xA)  /* OTG_HS Alternate Function mapping */

/* AF 11 selection */
#define GPIO_AF_ETH            ((u32)0x0B)  /* ETHERNET Alternate Function mapping */

/* AF 12 selection */
#define GPIO_AF_FSMC           ((u32)0xC)  /* FSMC Alternate Function mapping */
#define GPIO_AF_OTG_HS_FS      ((u32)0xC)  /* OTG HS configured in FS, Alternate Function mapping */
#define GPIO_AF_SDIO           ((u32)0xC)  /* SDIO Alternate Function mapping */

/* AF 13 selection */
#define GPIO_AF_DCMI           ((u32)0x0D)  /* DCMI Alternate Function mapping */

/* AF 15 selection */
#define GPIO_AF_EVENTOUT       ((u32)0x0F)  /* EVENTOUT Alternate Function mapping */

void GPIO_clock(GPIOPortE port);
void GPIO_setupPin(GPIO_TypeDef* GPIOx, GPIO_Pin pin, GPIOModeE mode,
    GPIOOutputDriveE drive, GPIOPullE pull, GPIOSpeedE speed);
void GPIO_AFConfig(GPIO_TypeDef* GPIOx, GPIO_Pin GPIO_PinSource, u32 GPIO_AF);
void GPIO_setPin(GPIO_TypeDef* GPIOx, GPIO_Pin pin);
void GPIO_writeBit(GPIO_TypeDef* GPIOx, GPIO_Pin pin, BOOL bitValue);
void GPIO_toggleBits(GPIO_TypeDef* GPIOx, GPIO_Pin pin);
u32 GPIO_readOutputDataBit(GPIO_TypeDef* GPIOx, GPIO_Pin pin);
u16 GPIO_readOutputData(GPIO_TypeDef* GPIOx);
u16 GPIO_readInputDataBit(GPIO_TypeDef* GPIOx, GPIO_Pin pin);
u16 GPIO_readInputData(GPIO_TypeDef* GPIOx);

#endif
