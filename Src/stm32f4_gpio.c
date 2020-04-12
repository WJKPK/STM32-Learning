#include<stm32f4_gpio.h>

void GPIO_clock(GPIOPortE port)
{
    RCC->AHB1ENR |= (u32)(0x01 << port);
}

void GPIO_setupPin(GPIO_TypeDef* GPIOx, GPIO_Pin pin, GPIOModeE mode,
    GPIOOutputDriveE drive, GPIOPullE pull, GPIOSpeedE speed)
{
  u32 doubledPinMov = pin * 2;
  GPIOx->MODER |= (mode << doubledPinMov);
  GPIOx->OTYPER |= (drive << pin);
  GPIOx->PUPDR |= (pull << doubledPinMov);
  GPIOx->OSPEEDR |= (speed << doubledPinMov);
}

void GPIO_setPin(GPIO_TypeDef* GPIOx, GPIO_Pin pin)
{
  GPIOx->ODR |= (0x01 << pin);
}

void GPIO_writeBit(GPIO_TypeDef* GPIOx, GPIO_Pin pin, BOOL bitValue)
{
  if (bitValue != FALSE)
  {
    GPIOx->BSRRL = (0x01 << pin);
  }
  else
  {
    GPIOx->BSRRH = (0x01 << pin);
  }
}

void GPIO_toggleBits(GPIO_TypeDef* GPIOx, GPIO_Pin pin)
{
  GPIOx->ODR ^= (0x01 << pin);
}

u32 GPIO_readOutputDataBit(GPIO_TypeDef* GPIOx, GPIO_Pin pin)
{
  return ((GPIOx->ODR & (0x01 << pin)) >> pin);
}

u16 GPIO_readOutputData(GPIO_TypeDef* GPIOx)
{
  return ((u16)GPIOx->ODR);
}

u16 GPIO_readInputDataBit(GPIO_TypeDef* GPIOx, GPIO_Pin pin)
{
  return (u16)((GPIOx->IDR & (0x01 << pin)) >> pin);
}

u16 GPIO_readInputData(GPIO_TypeDef* GPIOx)
{
  return ((u16)GPIOx->IDR);
}

void GPIO_AFConfig(GPIO_TypeDef* GPIOx, GPIO_Pin GPIO_PinSource, u32 GPIO_AF)
{
  uint32_t temp = 0x00;
  uint32_t temp_2 = 0x00;

  temp = (GPIO_AF << ((GPIO_PinSource & (u32)0x07) * 0x04));
  GPIOx->AFR[GPIO_PinSource >> 0x03] &= ~((u32)0x0F << ((GPIO_PinSource & (u32)0x07) * 4)) ;
  temp_2 = GPIOx->AFR[GPIO_PinSource >> 0x03] | temp;
  GPIOx->AFR[GPIO_PinSource >> 0x03] = temp_2;
}