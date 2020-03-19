#include <soc.h>
#include <drivers/clock_control.h>
#include <sys/util.h>
#include <clock_bluenrg.h>

static inline int bluenrg_clock_control_on(struct device *dev,
					 clock_control_subsys_t sub_system)
{
    struct bluenrg_pclken *pclken = (struct bluenrg_pclken *)(sub_system);
    ARG_UNUSED(dev);
    SysCtrl_PeripheralClockCmd(pclken->bus, ENABLE);
    return 0;
}

static inline int bluenrg_clock_control_off(struct device *dev,
					 clock_control_subsys_t sub_system)
{
    struct bluenrg_pclken *pclken = (struct bluenrg_pclken *)(sub_system);
    ARG_UNUSED(dev);
    SysCtrl_PeripheralClockCmd(pclken->bus, DISABLE);
    return 0;
}

static int bluenrg_clock_control_get_subsys_rate(struct device *clock,
					clock_control_subsys_t sub_system,
					u32_t *rate)
{
	*rate = 32000000;
	return 0;
}

static int bluenrg_clock_control_init(struct device *dev)
{
	
	return 0;
}

static struct clock_control_driver_api bluenrg_clock_control_api = {
	.on = bluenrg_clock_control_on,
	.off = bluenrg_clock_control_off,
	.get_rate = bluenrg_clock_control_get_subsys_rate,
};

/**
 * @brief RCC device, note that priority is intentionally set to 1 so
 * that the device init runs just after SOC init
 */
DEVICE_AND_API_INIT(rcc_bluenrg, BLUENRG_CLOCK_CONTROL_NAME,
		    &bluenrg_clock_control_init,
		    NULL, NULL,
		    PRE_KERNEL_1,
		    CONFIG_CLOCK_CONTROL_BLUENRG_DEVICE_INIT_PRIORITY,
		    &bluenrg_clock_control_api);
