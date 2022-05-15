/**
 * @file GPIO.h
 * @brief
 */

#ifndef GPIO_H
#define GPIO_H

#include "Stm32f407.h"

/**
 * @brief This is a Configuration structure for a GPIO pin
 */
typedef struct
{
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;  /* Possible values from @GPIO_PIN_MODES */
    uint8_t GPIO_PinSpeed; /* Possible values from @GPIO_PIN_SPEED */
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinOPType;
    uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfiguration_t;

/**
 * @brief This is a Handle structure for a GPIO pin
 */
typedef struct
{
    GPIO_RegDef_t *pGPIOx;                         /* This holds the base address of the GPIO port to which the pin belongs */
    GPIO_PinConfiguration_t GPIO_PinConfiguration; /* This holds GPIO pin configuration settings */
} GPIO_Handle_t;

/**
 * @brief GPIO_PIN_NUMBERS
 */
#define GPIO_PIN_NO_0 0
#define GPIO_PIN_NO_1 1
#define GPIO_PIN_NO_2 2
#define GPIO_PIN_NO_3 3
#define GPIO_PIN_NO_4 4
#define GPIO_PIN_NO_5 5
#define GPIO_PIN_NO_6 6
#define GPIO_PIN_NO_7 7
#define GPIO_PIN_NO_8 8
#define GPIO_PIN_NO_9 9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15

/**
 * @brief GPIO_PIN_MODES
 */
#define GPIO_MODE_IN 0x00u
#define GPIO_MODE_OUT 0x01u
#define GPIO_MODE_ALTFN 0x02u
#define GPIO_MODE_ANALOG 0x03u
#define GPIO_MODE_IT_FT 0x04u
#define GPIO_MODE_IT_RT 0x05u
#define GPIO_MODE_IT_RFT 0x06u

/**
 * @brief GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP 0
#define GPIO_OP_TYPE_OD 1

/**
 * @brief GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW 0
#define GPIO_SPEED_MEDIUM 1
#define GPIO_SPEED_FAST 2
#define GPOI_SPEED_HIGH 3

/**
 * @brief GPIO pin pull up AND pull down configuration macros
 */
#define GPIO_NO_PUPD 0
#define GPIO_PIN_PU 1
#define GPIO_PIN_PD 2

/**
 * @brief APIs supported by GPIO Driver:
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t state);

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* GPIO_H */
