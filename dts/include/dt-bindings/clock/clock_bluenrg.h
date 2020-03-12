#ifndef ZEPHYR_DRIVERS_CLOCK_CONTROL_BLUENRG_H_
#define ZEPHYR_DRIVERS_CLOCK_CONTROL_BLUENRG_H_

#define BLUENRG_CLOCK_PERIPH_GPIO           0x00000001
#define BLUENRG_CLOCK_PERIPH_NVM            0x00000002
#define BLUENRG_CLOCK_PERIPH_SYS_CONTROL    0x00000004
#define BLUENRG_CLOCK_PERIPH_UART           0x00000008
#define BLUENRG_CLOCK_PERIPH_SPI            0x00000010
#define BLUENRG_CLOCK_PERIPH_WDG            0x00000080
#define BLUENRG_CLOCK_PERIPH_ADC            0x00000100
#define BLUENRG_CLOCK_PERIPH_I2C1           0x00000200
#define BLUENRG_CLOCK_PERIPH_I2C2           0x00000400
#define BLUENRG_CLOCK_PERIPH_MTFX1          0x00000800
#define BLUENRG_CLOCK_PERIPH_MTFX2          0x00001000
#define BLUENRG_CLOCK_PERIPH_RTC            0x00002000
#define BLUENRG_CLOCK_PERIPH_DMA            0x00010000
#define BLUENRG_CLOCK_PERIPH_RNG            0x00020000
#define BLUENRG_CLOCK_PERIPH_PKA            0x000C0000

#endif /* ZEPHYR_DRIVERS_CLOCK_CONTROL_BLUENRG_H_ */
