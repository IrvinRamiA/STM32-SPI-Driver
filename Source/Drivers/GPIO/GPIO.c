/**
 * @file GPIO.c
 * @brief
 */

#include "GPIO.h"
#include "Stm32f407.h"

/**
 * @brief This function controls the peripheral clock for each GPIO peripheral
 *
 * @param pGPIOx GPIOx (x: A, B, C, D, E, F, G, H, I)
 * @param state ENABLE / DISABLE
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t state)
{
    if (ENABLE == state)
    {
        if( GPIOA == pGPIOx)
        {
            GPIOA_PCLK_EN();
        }
        else if( GPIOB == pGPIOx)
        {
            GPIOB_PCLK_EN();
        }
        else if( GPIOC == pGPIOx)
        {
            GPIOC_PCLK_EN();
        }
        else if( GPIOD == pGPIOx)
        {
            GPIOD_PCLK_EN();
        }
        else if( GPIOE == pGPIOx)
        {
            GPIOE_PCLK_EN();
        }
        else if( GPIOF == pGPIOx)
        {
            GPIOF_PCLK_EN();
        }
        else if( GPIOG == pGPIOx)
        {
            GPIOG_PCLK_EN();
        }
        else if( GPIOH == pGPIOx)
        {
            GPIOH_PCLK_EN();
        }
        else if( GPIOI == pGPIOx)
        {
            GPIOI_PCLK_EN();
        }
    }
    else
    {
        /* TODO */
    }
}

/**
 * @brief This function initializes the GPIO peripheral
 *
 * @param pGPIOHandle
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t tempRegister = 0;

    /* Enable the peripheral clock */
    GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

    /* 1 . Configuration of mode for GPIO pin */
    if (pGPIOHandle->GPIO_PinConfiguration.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        /* Non interrupt mode */
        /* MODER - Mode Register (32 bits) - For Pin 0: MODER0 [1:0] -> For Pin 15: MODER15 [1:0] */
        /* 00: Input, 01: Output, 10: Alternate function, 11: Analog mode*/
        tempRegister = (pGPIOHandle->GPIO_PinConfiguration.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER |= tempRegister;
    }
    else
    {
        // this part will code later . ( interrupt mode)
        if (pGPIOHandle->GPIO_PinConfiguration.GPIO_PinMode == GPIO_MODE_IT_FT)
        {
            // 1. configure the FTSR
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber);
            // Clear the corresponding RTSR bit
            EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber);
        }
        else if (pGPIOHandle->GPIO_PinConfiguration.GPIO_PinMode == GPIO_MODE_IT_RT)
        {
            // 1 . configure the RTSR
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber);
            // Clear the corresponding RTSR bit
            EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber);
        }
        else if (pGPIOHandle->GPIO_PinConfiguration.GPIO_PinMode == GPIO_MODE_IT_RFT)
        {
            // 1. configure both FTSR and RTSR
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber);
            // Clear the corresponding RTSR bit
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber);
        }

        /* 2. Configure the GPIO port selection in SYSCFG_EXTICR */
        uint8_t temp1 = pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber / 4;
        uint8_t temp2 = pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber % 4;
        uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
        SYSCFG_PCLK_EN();
        SYSCFG->EXTICR[temp1] = portCode << (temp2 * 4);

        /* 3 . Enable the EXTI interrupt delivery using IMR */
        EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber;
    }

    // 2. configure the speed
    tempRegister = (pGPIOHandle->GPIO_PinConfiguration.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber)); // clearing
    pGPIOHandle->pGPIOx->OSPEEDR |= tempRegister;

    // 3. configure the pupd settings
    tempRegister = (pGPIOHandle->GPIO_PinConfiguration.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber)); // clearing
    pGPIOHandle->pGPIOx->PUPDR |= tempRegister;

    // 4. configure the optype
    tempRegister = (pGPIOHandle->GPIO_PinConfiguration.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber); // clearing
    pGPIOHandle->pGPIOx->OTYPER |= tempRegister;

    /* 5. configure the alt functionality */
    if (pGPIOHandle->GPIO_PinConfiguration.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        /* Configure the alt function registers */
        uint8_t temp1, temp2;

        temp1 = pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber % 8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); // clearing
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfiguration.GPIO_PinAltFunMode << (4 * temp2));
    }
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if (pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    else if (pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    else if (pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    else if (pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }
    else if (pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET();
    }
    else if (pGPIOx == GPIOG)
    {
        GPIOG_REG_RESET();
    }
    else if (pGPIOx == GPIOH)
    {
        GPIOH_REG_RESET();
    }
    else if (pGPIOx == GPIOI)
    {
        GPIOI_REG_RESET();
    }
}

void GPIO_SetPinAsOutput(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	GPIO_Handle_t gpioHandle;

	gpioHandle.pGPIOx = pGPIOx;
	gpioHandle.GPIO_PinConfiguration.GPIO_PinNumber = pinNumber;
	gpioHandle.GPIO_PinConfiguration.GPIO_PinMode = GPIO_MODE_OUT;
	gpioHandle.GPIO_PinConfiguration.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioHandle.GPIO_PinConfiguration.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioHandle.GPIO_PinConfiguration.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&gpioHandle);
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t value;

    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

    return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t value;

    value = (uint16_t)pGPIOx->IDR;

    return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if (Value == GPIO_PIN_SET)
    {
        // write 1 to the output data register at the bit field corresponding to the pin number
        pGPIOx->ODR |= (1 << PinNumber);
    }
    else
    {
        // write 0
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
}

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

    if (EnorDi == ENABLE)
    {
        if (IRQNumber <= 31)
        {
            // program ISER0 register
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber > 31 && IRQNumber < 64) // 32 to 63
        {
            // program ISER1 register
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            // program ISER2 register //64 to 95
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
    }
    else
    {
        if (IRQNumber <= 31)
        {
            // program ICER0 register
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber > 31 && IRQNumber < 64)
        {
            // program ICER1 register
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            // program ICER2 register
            *NVIC_ICER2 |= (1 << (IRQNumber % 64));
        }
    }
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    // 1. first lets find out the ipr register
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
    // clear the exti pr register corresponding to the pin number
    if (EXTI->PR & (1 << PinNumber))
    {
        // clear
        EXTI->PR |= (1 << PinNumber);
    }
}
