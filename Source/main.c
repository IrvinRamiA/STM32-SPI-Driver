/**
 * @file main.c
 * @brief
 */

#include "Stm32f407.h"
#include "GPIO.h"
#include "Utils.h"

int main(void)
{
	GPIO_SetPinAsOutput(GPIOD, GPIO_PIN_NO_14);

	while (TRUE)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_14);
		delay();
	}

	return STD_RETURN_OK;
}
