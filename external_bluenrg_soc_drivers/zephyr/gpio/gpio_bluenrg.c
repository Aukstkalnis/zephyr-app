#include <errno.h>

#include <kernel.h>
#include <device.h>
#include <soc.h>
#include <init.h>
#include <drivers/gpio.h>
#include <sys/sys_io.h>
#include "gpio_bluenrg.h"
#include <dt-bindings/pinctrl/stm32-pinctrlf1.h>
#include <dt-bindings/pinctrl/bluenrg-pinctrl.h>
#include <drivers/clock_control/clock_bluenrg.h>
#include <drivers/interrupt_controller/exti_bluenrg.h>
#include <sys/util.h>
#include <misc.h>
#include "gpio_utils.h"
#include <irq.h>
/**
 * @brief Common GPIO driver for BlueNRG MCUs.
 */
typedef enum {
  IRQ_ON_FALLING_EDGE = 0,
  IRQ_ON_RISING_EDGE,
  IRQ_ON_BOTH_EDGE
} SdkEvalButtonIrq;

/**
 * @brief Configure the hardware.
 */
int gpio_bluenrg_configure(u32_t *base_addr, int pin, int conf)
{
    GPIO_InitType GPIO_InitStructure;
  	GPIO_InitStructure.GPIO_Pin = 1<<pin;
  	GPIO_InitStructure.GPIO_Mode = conf;
  	GPIO_InitStructure.GPIO_Pull = 0x00;
  	GPIO_InitStructure.GPIO_HighPwr = 0x00;
  	GPIO_Init(&GPIO_InitStructure);

    return 0;
}

static int gpio_bluenrg_port_get_raw(struct device *dev, u32_t *value)
{
    const struct gpio_bluenrg_config *cfg = dev->config->config_info;
    GPIO_Type *gpio = (GPIO_Type *)cfg->base;
    *value = (u32_t)gpio->DATA;
    return 0;
}

static int gpio_bluenrg_port_toggle_bits(struct device *dev,
				       gpio_port_pins_t pins)
{
	const struct gpio_bluenrg_config *cfg = dev->config->config_info;
	GPIO_Type *gpio = (GPIO_Type *)cfg->base;

    assert_param(IS_GPIO_PINS(pins));
	WRITE_REG(gpio->DATA, READ_REG(gpio->DATA) ^ pins);

	return 0;
}

static int gpio_bluenrg_port_clear_bits_raw(struct device *dev,
					  gpio_port_pins_t pins)
{
	const struct gpio_bluenrg_config *cfg = dev->config->config_info;
	GPIO_Type *gpio = (GPIO_Type *)cfg->base;
	WRITE_REG(gpio->DATC, pins);
	return 0;
}

static int gpio_bluenrg_port_set_bits_raw(struct device *dev,
					gpio_port_pins_t pins)
{
	const struct gpio_bluenrg_config *cfg = dev->config->config_info;
	GPIO_Type *gpio = (GPIO_Type *)cfg->base;
    WRITE_REG(gpio->DATS, pins);

	return 0;
}

static int gpio_bluenrg_config(struct device *dev,
			     gpio_pin_t pin, gpio_flags_t flags)
{
    int err = 0;
	if (err != 0) {
		goto release_lock;
	}
    if ((flags & GPIO_OUTPUT) != 0) {
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			gpio_bluenrg_port_set_bits_raw(dev, BIT(pin));
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			gpio_bluenrg_port_clear_bits_raw(dev, BIT(pin));
		}
	}
release_lock:
    return err;
}

static int gpio_bluenrg_port_set_masked_raw(struct device *dev,
					  gpio_port_pins_t mask,
					  gpio_port_value_t value)
{
	const struct gpio_bluenrg_config *cfg = dev->config->config_info;
	GPIO_Type *gpio = (GPIO_Type *)cfg->base;
	u32_t port_value = (u32_t) gpio->DATA;

    WRITE_REG(gpio->DATA, (port_value & ~mask) | (mask & value));

	return 0;
}

void bluerng_exti_disable(int pin)
{
	GPIO_EXTICmd(pin, DISABLE);
}

static void gpio_bluenrg_isr(int line, void *arg)
{
	struct device *dev = arg;
	struct gpio_bluenrg_data *data = dev->driver_data;

	if ((BIT(line) & data->cb_pins) != 0) {
		gpio_fire_callbacks(&data->cb, dev, BIT(line));
	}
}

static int gpio_bluenrg_pin_interrupt_configure(struct device *dev,
		gpio_pin_t pin, enum gpio_int_mode mode,
		enum gpio_int_trig trig)
{
	// const struct gpio_bluenrg_config *cfg = dev->config->config_info;
	struct gpio_bluenrg_data *data = dev->driver_data;
    GPIO_EXTIConfigType GPIO_EXTIStructure;
	// int edge = 0;
	int err = 0;

    GPIO_EXTIStructure.GPIO_Pin = 1<<pin;

	if (mode == GPIO_INT_MODE_DISABLED) {
		bluerng_exti_disable(pin);
		bluenrg_exti_unset_callback(pin);
		data->cb_pins &= ~BIT(pin);
		goto release_lock;
	}

	/* Level trigger interrupts not supported */
	if (mode == GPIO_INT_MODE_LEVEL) {
		err = -ENOTSUP;
		goto release_lock;
	}

    NVIC_InitType NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = GPIO_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LOW_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // set callback
	if (bluenrg_exti_set_callback(pin, gpio_bluenrg_isr, dev) != 0) {
		err = -EBUSY;
		goto release_lock;
	}

	data->cb_pins |= BIT(pin);
    
	switch (trig) {
	case GPIO_INT_TRIG_LOW:
        GPIO_EXTIStructure.GPIO_Event = IRQ_ON_FALLING_EDGE;
		break;
	case GPIO_INT_TRIG_HIGH:
        GPIO_EXTIStructure.GPIO_Event = IRQ_ON_RISING_EDGE;
		break;
	case GPIO_INT_TRIG_BOTH:
        GPIO_EXTIStructure.GPIO_Event = IRQ_ON_BOTH_EDGE;
		break;
	}
	GPIO_EXTIStructure.GPIO_IrqSense = GPIO_IrqSense_Edge;
    GPIO_EXTIConfig(&GPIO_EXTIStructure);
    /* Clear pending interrupt */
    GPIO_ClearITPendingBit(1<<pin);
    /* Enable the interrupt */
    GPIO_EXTICmd(1<<pin, ENABLE);

release_lock:

	return err;
}

static int gpio_bluenrg_manage_callback(struct device *dev,
				      struct gpio_callback *callback,
				      bool set)
{
	struct gpio_bluenrg_data *data = dev->driver_data;

	return gpio_manage_callback(&data->cb, callback, set);
}

static int gpio_bluenrg_enable_callback(struct device *dev,
				      gpio_pin_t pin)
{
	struct gpio_bluenrg_data *data = dev->driver_data;

	data->cb_pins |= BIT(pin);

	return 0;
}

static int gpio_bluenrg_disable_callback(struct device *dev,
				       gpio_pin_t pin)
{
	struct gpio_bluenrg_data *data = dev->driver_data;

	data->cb_pins &= ~BIT(pin);

	return 0;
}

static int gpio_bluenrg_init(struct device *device)
{
	const struct gpio_bluenrg_config *cfg = device->config->config_info;

	struct device *clk = device_get_binding(BLUENRG_CLOCK_CONTROL_NAME);

	if (clock_control_on(clk, (clock_control_subsys_t *)&cfg->pclken) != 0) {
		return -EIO;
	}
	return 0;
}

static const struct gpio_driver_api gpio_bluenrg_driver = {
	.pin_configure = gpio_bluenrg_config,
	.port_get_raw = gpio_bluenrg_port_get_raw,
	.port_set_masked_raw = gpio_bluenrg_port_set_masked_raw,
	.port_set_bits_raw = gpio_bluenrg_port_set_bits_raw,
	.port_clear_bits_raw = gpio_bluenrg_port_clear_bits_raw,
	.port_toggle_bits = gpio_bluenrg_port_toggle_bits,
	.pin_interrupt_configure = gpio_bluenrg_pin_interrupt_configure,
	.manage_callback = gpio_bluenrg_manage_callback,
	.enable_callback = gpio_bluenrg_enable_callback,
	.disable_callback = gpio_bluenrg_disable_callback,
};

#define GPIO_DEVICE_INIT(__name, __suffix, __base_addr, __cenr, __bus) \
	static const struct gpio_bluenrg_config gpio_bluenrg_cfg_## __suffix = {   \
		.common = {						 \
			 .port_pin_mask = 0,		 \
		},							     \
		.base = (u32_t *)__base_addr,	 \
		.pclken = { .bus = __bus }		 \
	};								     \
	static struct gpio_bluenrg_data gpio_bluenrg_data_## __suffix;	       \
	DEVICE_AND_API_INIT(gpio_bluenrg_## __suffix,			       \
			    __name,					       \
			    gpio_bluenrg_init,				       \
			    &gpio_bluenrg_data_## __suffix,		       \
			    &gpio_bluenrg_cfg_## __suffix,		       \
			    POST_KERNEL,				       \
			    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,	       \
			    &gpio_bluenrg_driver)

#define GPIO_DEVICE_INIT_BLUENRG()		      \
	GPIO_DEVICE_INIT(DT_GPIO_BLUENRG_GPIO_LABEL,	      \
			 GPIO,				      \
			 DT_GPIO_BLUENRG_GPIO_BASE_ADDRESS, \
			 DT_GPIO_BLUENRG_GPIO_CLOCK_BITS,   \
			 DT_GPIO_BLUENRG_GPIO_CLOCK_BUS)

#ifdef CONFIG_GPIO_BLUENRG
GPIO_DEVICE_INIT_BLUENRG();
#endif /* CONFIG_GPIO_BLUENRG */
