/* SPDX-License-Identifier: Apache-2.0 */

/* SoC level DTS fixup file */

#define DT_NUM_IRQ_PRIO_BITS DT_ARM_V6M_NVIC_E000E100_ARM_NUM_IRQ_PRIORITY_BITS

/* End of SoC Level DTS fixup file */
#define DT_GPIO_BLUENRG_GPIO_BASE_ADDRESS       DT_ST_BLUENRG_GPIO_40000000_BASE_ADDRESS
#define DT_GPIO_BLUENRG_GPIO_CLOCK_BITS_0       DT_ST_BLUENRG_GPIO_40000000_CLOCK_BITS_0
#define DT_GPIO_BLUENRG_GPIO_CLOCK_BUS_0        DT_ST_BLUENRG_GPIO_40000000_CLOCK_BUS_0
#define DT_GPIO_BLUENRG_GPIO_CLOCK_CONTROLLER   DT_ST_BLUENRG_GPIO_40000000_CLOCK_CONTROLLER
#define DT_GPIO_BLUENRG_GPIO_LABEL              DT_ST_BLUENRG_GPIO_40000000_LABEL
#define DT_GPIO_BLUENRG_GPIO_SIZE               DT_ST_BLUENRG_GPIO_40000000_SIZE
// #define DT_GPIO_BLUENRG_GPIO_CLOCK_BITS         DT_ST_BLUENRG_GPIO_40000000_CLOCK_BITS
#define DT_GPIO_BLUENRG_GPIO_CLOCK_BUS          DT_ST_BLUENRG_GPIO_40000000_CLOCK_BUS

#define DT_UART_BLUENRG_USART_1_BASE_ADDRESS    DT_ST_BLUENRG_UART_40300000_BASE_ADDRESS
#define DT_UART_BLUENRG_USART_1_BAUD_RATE       DT_ST_BLUENRG_UART_40300000_CURRENT_SPEED
#define DT_UART_BLUENRG_USART_1_IRQ_PRI         DT_ST_BLUENRG_UART_40300000_IRQ_0_PRIORITY
#define DT_UART_BLUENRG_USART_1_NAME            DT_ST_BLUENRG_UART_40300000_LABEL
#define DT_USART_1_IRQ                          DT_ST_BLUENRG_UART_40300000_IRQ_0
// #define DT_UART_BLUENRG_USART_1_CLOCK_BITS      DT_ST_BLUENRG_UART_40300000_CLOCK_BITS
#define DT_UART_BLUENRG_USART_1_CLOCK_BUS       DT_ST_BLUENRG_UART_40300000_CLOCK_BUS
#define DT_UART_BLUENRG_USART_1_HW_FLOW_CONTROL DT_ST_BLUENRG_UART_40300000_HW_FLOW_CONTROL

#define DT_ADC_BASE_ADDRESS                DT_ST_BLUENRG_ADC_40800000_BASE_ADDRESS
#define DT_ADC_IRQ                         DT_ST_BLUENRG_ADC_40800000_IRQ_0
#define DT_ADC_IRQ_PRI                     DT_INST_0_BLUENRG_BLUENRG_ADC_IRQ_0
#define DT_ADC_NAME                        DT_ST_BLUENRG_ADC_40800000_LABEL
#define DT_ADC_CLOCK_BUS                   DT_ST_BLUENRG_ADC_40800000_CLOCK_BUS_0

// 37-15