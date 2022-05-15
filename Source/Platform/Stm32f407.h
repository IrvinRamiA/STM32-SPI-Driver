/**
 * @file Stm32f407.h
 * @brief This file contains definitions for STM32F407 devices
 */

#ifndef STM3F407_H
#define STM3F407_H

#include <stddef.h>
#include "StandardTypes.h"

#define __vo                            volatile
#define __weak                          __attribute__((weak))

/*----------------------------------------------------------------------------*/
/* START: PROCESSOR SPECIFIC DETAILS                                          */
/*----------------------------------------------------------------------------*/

/**
 * @brief ARM Cortex Mx Processor NVIC ISERx register Addresses
 */
#define NVIC_ISER0                      ((__vo uint32_t *)0xE000E100)
#define NVIC_ISER1                      ((__vo uint32_t *)0xE000E104)
#define NVIC_ISER2                      ((__vo uint32_t *)0xE000E108)
#define NVIC_ISER3                      ((__vo uint32_t *)0xE000E10c)

/**
 * @brief ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0                      ((__vo uint32_t *)0XE000E180)
#define NVIC_ICER1                      ((__vo uint32_t *)0XE000E184)
#define NVIC_ICER2                      ((__vo uint32_t *)0XE000E188)
#define NVIC_ICER3                      ((__vo uint32_t *)0XE000E18C)

/**
 * @brief ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR               ((__vo uint32_t *)0xE000E400)

/**
 * @brief Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED          4

/*----------------------------------------------------------------------------*/
/* END: PROCESSOR SPECIFIC DETAILS                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* START: BASE ADDRESSES DEFINITIONS                                          */
/*----------------------------------------------------------------------------*/

/**
 * @brief Base addresses of Flash and SRAM memories. Information obtained in
 * reference manual (RM0090 - STM32F407 advanced ARM-based 32-bits MCUs).
 * RM - Page 71 - Table 3
 */
#define FLASH_BASEADDR                  0x08000000U
#define SRAM1_BASEADDR                  0x20000000U
#define SRAM2_BASEADDR                  0x2001C000U
#define ROM_BASEADDR                    0x1FFF0000U
#define SRAM                            SRAM1_BASEADDR

/**
 * @brief AHBx and APBx Bus Peripheral base addresses.
 * AHBx: Advanced High-performance Bus (x: 1, 2, 3)
 * APBx: Advanced Peripheral Bus (x: 1, 2)
 * RM - Pages 64-67 - Table 1
 */

#define PERIPH_BASEADDR                 0x40000000U
#define APB1PERIPH_BASEADDR             PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR             0x40010000U
#define AHB1PERIPH_BASEADDR             0x40020000U
#define AHB2PERIPH_BASEADDR             0x50000000U

/**
 * @brief Base addresses of peripherals which are hanging on AHB1 bus.
 * In this case only GPIO, RCC peripherals are defined.
 * RM - Pages 64-67 - Table 1
 */

#define GPIOA_BASEADDR                  (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                  (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR                  (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR                  (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR                  (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR                  (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR                  (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR                  (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR                  (AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR                    (AHB1PERIPH_BASEADDR + 0x3800)

/**
 * @brief Base addresses of peripherals which are hanging on APB1 bus
 * In this case only SPI peripherals are defined.
 * RM - Pages 64-67 - Table 1
 */
#define SPI2_BASEADDR                   (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR                   (APB1PERIPH_BASEADDR + 0x3C00)

/**
 * @brief Base addresses of peripherals which are hanging on APB2 bus
 * In this case only SPI peripherals are defined.
 * RM - Pages 64-67 - Table 1
 */
#define SPI1_BASEADDR                   (APB2PERIPH_BASEADDR + 0x3000)

/*----------------------------------------------------------------------------*/
/* END: BASE ADDRESSES DEFINITIONS                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* START: PERIPHERAL REGISTER DEFINITION STRUCTURES                           */
/*----------------------------------------------------------------------------*/

/**
 * @brief Peripheral register definition structure for GPIO.
 * The registers have to be accessed by byte (8 bits), half-words (16 bits) or 
 * words (32 bits).
 * RM - Page 287
 */
typedef struct
{
    __vo uint32_t MODER;   /* GPIO Port Mode Register,              Address offset: 0x00 */
    __vo uint32_t OTYPER;  /* GPIO Port Output Register,            Address offset: 0x04 */
    __vo uint32_t OSPEEDR; /* GPIO Port Output Speed Register,      Address offset: 0x08 */
    __vo uint32_t PUPDR;   /* GPIO Port Pull-up/Pull-down Register, Address offset: 0x0C */
    __vo uint32_t IDR;     /* GPIO Port Input Data Register,        Address offset: 0x10 */
    __vo uint32_t ODR;     /* GPIO Port Output Data Register,       Address offset: 0x14 */
    __vo uint32_t BSRR;    /* GPIO Port Bit Set/Reset Register,     Address offset: 0x18 */
} GPIO_RegDef_t;

/**
 * @brief Peripheral register definition structure for RCC.
 */
typedef struct
{
    __vo uint32_t CR;
    __vo uint32_t PLLCFGR;
    __vo uint32_t CFGR;
    __vo uint32_t CIR;
    __vo uint32_t AHB1RSTR;
    __vo uint32_t AHB2RSTR;
    __vo uint32_t AHB3RSTR;
    uint32_t RESERVED0;
    __vo uint32_t APB1RSTR;
    __vo uint32_t APB2RSTR;
    uint32_t RESERVED1[2];
    __vo uint32_t AHB1ENR;
    __vo uint32_t AHB2ENR;
    __vo uint32_t AHB3ENR;
    uint32_t RESERVED2;
    __vo uint32_t APB1ENR;
    __vo uint32_t APB2ENR;
    uint32_t RESERVED3[2];
    __vo uint32_t AHB1LPENR;
    __vo uint32_t AHB2LPENR;
    __vo uint32_t AHB3LPENR;
    uint32_t RESERVED4;
    __vo uint32_t APB1LPENR;
    __vo uint32_t APB2LPENR;
    uint32_t RESERVED5[2];
    __vo uint32_t BDCR;
    __vo uint32_t CSR;
    uint32_t RESERVED6[2];
    __vo uint32_t SSCGR;
    __vo uint32_t PLLI2SCFGR;
    __vo uint32_t PLLSAICFGR;
    __vo uint32_t DCKCFGR;
    __vo uint32_t CKGATENR;
    __vo uint32_t DCKCFGR2;
} RCC_RegDef_t;

/**
 * @brief Peripheral register definition structure for SPI.
 * The registers have to be accessed by half-words (16 bits) or words (32 bits).
 * RM - Page 925 - Table 129
 */
typedef struct
{
    __vo uint32_t CR1;     /* SPI Control Register 1,         Address offset: 0x00 */
    __vo uint32_t CR2;     /* SPI Control Register 2,         Address offset: 0x04 */
    __vo uint32_t SR;      /* SPI Status Register,            Address offset: 0x08 */
    __vo uint32_t DR;      /* SPI Data Register,              Address offset: 0x0C */
    __vo uint32_t CRCPR;   /* SPI CRC Polynomial Register,    Address offset: 0x10 */
    __vo uint32_t RXCRCR;  /* SPI RX CRC Register,            Address offset: 0x14 */
    __vo uint32_t TXCRCR;  /* SPI TX CRC Register,            Address offset: 0x18 */
    __vo uint32_t I2SCFGR; /* SPI_I2S Configuration Register, Address offset: 0x1C */
    __vo uint32_t I2SPR;   /* SPI_I2S Prescaler Register,     Address offset: 0x20 */
} SPI_RegDef_t;

/*----------------------------------------------------------------------------*/
/* END: PERIPHERAL REGISTER DEFINITION STRUCTURES                             */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* START: PERIPHERAL DEFINITIONS (BASE ADDRESSES TYPECASTED TO REGDEF_T)      */
/*----------------------------------------------------------------------------*/

#define GPIOA                           ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB                           ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC                           ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD                           ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE                           ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF                           ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG                           ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH                           ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI                           ((GPIO_RegDef_t *)GPIOI_BASEADDR)

#define RCC                             ((RCC_RegDef_t *)RCC_BASEADDR)

#define SPI1                            ((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2                            ((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3                            ((SPI_RegDef_t *)SPI3_BASEADDR)

/*----------------------------------------------------------------------------*/
/* END: PERIPHERAL DEFINITIONS (BASE ADDRESSES TYPECASTED TO REGDEF_T)        */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* START: GENERAL MACROS FOR PERIPHERALS                                      */
/*----------------------------------------------------------------------------*/

/**
 * @brief Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()                 (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()                 (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()                 (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()                 (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()                 (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()                 (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()                 (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()                 (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()                 (RCC->AHB1ENR |= (1 << 8))

/**
 * @brief Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()                  (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()                  (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()                  (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()                  (RCC->APB2ENR |= (1 << 13))

/**
 *@brief Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()             \
    do                                \
    {                                 \
        (RCC->AHB1RSTR |= (1 << 0));  \
        (RCC->AHB1RSTR &= ~(1 << 0)); \
    } while (0)
#define GPIOB_REG_RESET()             \
    do                                \
    {                                 \
        (RCC->AHB1RSTR |= (1 << 1));  \
        (RCC->AHB1RSTR &= ~(1 << 1)); \
    } while (0)
#define GPIOC_REG_RESET()             \
    do                                \
    {                                 \
        (RCC->AHB1RSTR |= (1 << 2));  \
        (RCC->AHB1RSTR &= ~(1 << 2)); \
    } while (0)
#define GPIOD_REG_RESET()             \
    do                                \
    {                                 \
        (RCC->AHB1RSTR |= (1 << 3));  \
        (RCC->AHB1RSTR &= ~(1 << 3)); \
    } while (0)
#define GPIOE_REG_RESET()             \
    do                                \
    {                                 \
        (RCC->AHB1RSTR |= (1 << 4));  \
        (RCC->AHB1RSTR &= ~(1 << 4)); \
    } while (0)
#define GPIOF_REG_RESET()             \
    do                                \
    {                                 \
        (RCC->AHB1RSTR |= (1 << 5));  \
        (RCC->AHB1RSTR &= ~(1 << 5)); \
    } while (0)
#define GPIOG_REG_RESET()             \
    do                                \
    {                                 \
        (RCC->AHB1RSTR |= (1 << 6));  \
        (RCC->AHB1RSTR &= ~(1 << 6)); \
    } while (0)
#define GPIOH_REG_RESET()             \
    do                                \
    {                                 \
        (RCC->AHB1RSTR |= (1 << 7));  \
        (RCC->AHB1RSTR &= ~(1 << 7)); \
    } while (0)
#define GPIOI_REG_RESET()             \
    do                                \
    {                                 \
        (RCC->AHB1RSTR |= (1 << 8));  \
        (RCC->AHB1RSTR &= ~(1 << 8)); \
    } while (0)

/**
 * @brief Returns port code for given GPIOx base address
 */
/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x) ((x == GPIOA) ? 0 : (x == GPIOB) ? 1 \
                                                   : (x == GPIOC) ? 2 \
                                                   : (x == GPIOD) ? 3 \
                                                   : (x == GPIOE) ? 4 \
                                                   : (x == GPIOF) ? 5 \
                                                   : (x == GPIOG) ? 6 \
                                                   : (x == GPIOH) ? 7 \
                                                   : (x == GPIOI) ? 8 \
                                                   : 0)

/**
 * @brief IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * @note Update these macros with valid values according to your MCU
 * @todo You may complete this list for other peripherals
 */

#define IRQ_NO_SPI1                     35
#define IRQ_NO_SPI2                     36
#define IRQ_NO_SPI3                     51

/**
 * @brief For all the possible priority levels
 */
#define NVIC_IRQ_PRI0                   0
#define NVIC_IRQ_PRI15                  15

/**
 * @brief Generic macros
 */

#define ENABLE                          1u
#define DISABLE                         0u
#define SET                             ENABLE
#define RESET                           DISABLE
#define GPIO_PIN_SET                    SET
#define GPIO_PIN_RESET                  RESET
#define FLAG_RESET                      RESET
#define FLAG_SET                        SET

/*----------------------------------------------------------------------------*/
/* END: GENERAL MACROS FOR PERIPHERALS                                        */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* START: BIT POSITION DEFINITIONS OF SPI PERIPHERAL                          */
/*----------------------------------------------------------------------------*/

/**
 * @brief Bit position definitions SPI_CR1 (Control Register 1).
 * RM - Page 916
 */
#define SPI_CR1_CPHA                    0
#define SPI_CR1_CPOL                    1
#define SPI_CR1_MSTR                    2
#define SPI_CR1_BR                      3
#define SPI_CR1_SPE                     6
#define SPI_CR1_LSBFIRST                7
#define SPI_CR1_SSI                     8
#define SPI_CR1_SSM                     9
#define SPI_CR1_RXONLY                  10
#define SPI_CR1_DFF                     11
#define SPI_CR1_CRCNEXT                 12
#define SPI_CR1_CRCEN                   13
#define SPI_CR1_BIDIOE                  14
#define SPI_CR1_BIDIMODE                15

/**
 * @brief position definitions SPI_CR2 (Control Register 2).
 * RM - Page 918
 */
#define SPI_CR2_RXDMAEN                 0
#define SPI_CR2_TXDMAEN                 1
#define SPI_CR2_SSOE                    2
#define SPI_CR2_FRF                     4
#define SPI_CR2_ERRIE                   5
#define SPI_CR2_RXNEIE                  6
#define SPI_CR2_TXEIE                   7

/**
 * @brief Bit position definitions SPI_SR (Status Register 1).
 * RM - Page 919
 */
#define SPI_SR_RXNE                     0
#define SPI_SR_TXE                      1
#define SPI_SR_CHSIDE                   2
#define SPI_SR_UDR                      3
#define SPI_SR_CRCERR                   4
#define SPI_SR_MODF                     5
#define SPI_SR_OVR                      6
#define SPI_SR_BSY                      7
#define SPI_SR_FRE                      8

/*----------------------------------------------------------------------------*/
/* END: BIT POSITION DEFINITIONS OF SPI PERIPHERAL                            */
/*----------------------------------------------------------------------------*/

/* #include "SPI.h" */

#endif /* STM3F407_H */
