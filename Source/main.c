/**
 * @file main.c
 * @brief
 */

#include "Stm32f407.h"
#include "GPIO.h"
#include "SPI.h"

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}

int main(void)
{

	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfiguration.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfiguration.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfiguration.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfiguration.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfiguration.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
		delay();
	}
	return 0;
}
