
#ifndef ZEPHYR_DRIVERS_INTERRUPT_CONTROLLER_INTC_EXTI_BLUENRG_H_
#define ZEPHYR_DRIVERS_INTERRUPT_CONTROLLER_INTC_EXTI_BLUENRG_H_

#include <zephyr/types.h>

/* device name */
#define BLUENRG_EXTI_NAME "bluenrg-exti"

typedef void (*bluenrg_exti_callback_t) (int line, void *user);
/**
 * @brief set EXTI interrupt callback
 *
 * @param line EXI# line
 * @param cb   user callback
 * @param data user data
 */
int bluenrg_exti_set_callback(int line, bluenrg_exti_callback_t cb, void *data);

/**
 * @brief unset EXTI interrupt callback
 *
 * @param line EXI# line
 */
void bluenrg_exti_unset_callback(int line);

#endif /* ZEPHYR_DRIVERS_INTERRUPT_CONTROLLER_INTC_EXTI_STM32_H_ */
