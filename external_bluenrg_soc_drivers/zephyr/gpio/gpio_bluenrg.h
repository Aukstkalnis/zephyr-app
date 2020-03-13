
#ifndef ZEPHYR_DRIVERS_GPIO_GPIO_BLUENRG_H_
#define ZEPHYR_DRIVERS_GPIO_GPIO_BLUENRG_H_

#include <drivers/clock_control.h>
#include <drivers/clock_control/clock_bluenrg.h>
#include <drivers/gpio.h>

#define BLUENRG_GPIO_PIN_0                 ((uint32_t)0x00000001)  /*!< Pin 0 selected */
#define BLUENRG_GPIO_PIN_1                 ((uint32_t)0x00000002)  /*!< Pin 1 selected */
#define BLUENRG_GPIO_PIN_2                 ((uint32_t)0x00000004)  /*!< Pin 2 selected */
#define BLUENRG_GPIO_PIN_3                 ((uint32_t)0x00000008)  /*!< Pin 3 selected */
#define BLUENRG_GPIO_PIN_4                 ((uint32_t)0x00000010)  /*!< Pin 4 selected */
#define BLUENRG_GPIO_PIN_5                 ((uint32_t)0x00000020)  /*!< Pin 5 selected */
#define BLUENRG_GPIO_PIN_6                 ((uint32_t)0x00000040)  /*!< Pin 6 selected */
#define BLUENRG_GPIO_PIN_7                 ((uint32_t)0x00000080)  /*!< Pin 7 selected */
#define BLUENRG_GPIO_PIN_8                 ((uint32_t)0x00000100)  /*!< Pin 8 selected */
#define BLUENRG_GPIO_PIN_9                 ((uint32_t)0x00000200)  /*!< Pin 9 selected */
#define BLUENRG_GPIO_PIN_10                ((uint32_t)0x00000400)  /*!< Pin 10 selected */
#define BLUENRG_GPIO_PIN_11                ((uint32_t)0x00000800)  /*!< Pin 11 selected */
#define BLUENRG_GPIO_PIN_12                ((uint32_t)0x00001000)  /*!< Pin 12 selected */
#define BLUENRG_GPIO_PIN_13                ((uint32_t)0x00002000)  /*!< Pin 13 selected */
#define BLUENRG_GPIO_PIN_14                ((uint32_t)0x00004000)  /*!< Pin 14 selected */

#define BLUENRG_GPIO_PIN_15                ((uint32_t)0x00008000)  /*!< Pin 15 selected */
#define BLUENRG_GPIO_PIN_16                ((uint32_t)0x00010000)  /*!< Pin 16 selected */
#define BLUENRG_GPIO_PIN_17                ((uint32_t)0x00020000)  /*!< Pin 17 selected */
#define BLUENRG_GPIO_PIN_18                ((uint32_t)0x00040000)  /*!< Pin 18 selected */
#define BLUENRG_GPIO_PIN_19                ((uint32_t)0x00080000)  /*!< Pin 19 selected */
#define BLUENRG_GPIO_PIN_20                ((uint32_t)0x00100000)  /*!< Pin 20 selected */
#define BLUENRG_GPIO_PIN_21                ((uint32_t)0x00200000)  /*!< Pin 21 selected */
#define BLUENRG_GPIO_PIN_22                ((uint32_t)0x00400000)  /*!< Pin 22 selected */
#define BLUENRG_GPIO_PIN_23                ((uint32_t)0x00800000)  /*!< Pin 23 selected */
#define BLUENRG_GPIO_PIN_24                ((uint32_t)0x01000000)  /*!< Pin 24 selected */
#define BLUENRG_GPIO_PIN_25                ((uint32_t)0x02000000)  /*!< Pin 25 selected */
#define BLUENRG_GPIO_PIN_All               ((uint32_t)0x03FFFFFF)  /*!< All pins selected */

#define BLUENRG_CLOCK_PERIPH_GPIO           CLOCK_PERIPH_GPIO       
#define BLUENRG_CLOCK_PERIPH_NVM            CLOCK_PERIPH_NVM        
#define BLUENRG_CLOCK_PERIPH_SYS_CONTROL    CLOCK_PERIPH_SYS_CONTROL
#define BLUENRG_CLOCK_PERIPH_UART           CLOCK_PERIPH_UART       
#define BLUENRG_CLOCK_PERIPH_SPI            CLOCK_PERIPH_SPI        
#define BLUENRG_CLOCK_PERIPH_WDG            CLOCK_PERIPH_WDG        
#define BLUENRG_CLOCK_PERIPH_ADC            CLOCK_PERIPH_ADC        
#define BLUENRG_CLOCK_PERIPH_I2C1           CLOCK_PERIPH_I2C1       
#define BLUENRG_CLOCK_PERIPH_I2C2           CLOCK_PERIPH_I2C2       
#define BLUENRG_CLOCK_PERIPH_MTFX1          CLOCK_PERIPH_MTFX1      
#define BLUENRG_CLOCK_PERIPH_MTFX2          CLOCK_PERIPH_MTFX2      
#define BLUENRG_CLOCK_PERIPH_RTC            CLOCK_PERIPH_RTC        
#define BLUENRG_CLOCK_PERIPH_DMA            CLOCK_PERIPH_DMA        
#define BLUENRG_CLOCK_PERIPH_RNG            CLOCK_PERIPH_RNG        
#define BLUENRG_CLOCK_PERIPH_PKA            CLOCK_PERIPH_PKA       

struct gpio_bluenrg_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	/* port base address */
	u32_t *base;
	struct bluenrg_pclken pclken;
};

struct gpio_bluenrg_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	/* Enabled INT pins generating a cb */
	u32_t cb_pins;
	/* user ISR cb */
	sys_slist_t cb;
};

/**
 * @brief helper for configuration of GPIO pin
 *
 * @param base_addr GPIO port base address
 * @param pin IO pin
 * @param func GPIO mode
 * @param altf Alternate function
 */

int gpio_bluenrg_configure(u32_t *base_addr, int pin, int conf);

#endif /* ZEPHYR_DRIVERS_GPIO_GPIO_BLUENRG_H_ */
